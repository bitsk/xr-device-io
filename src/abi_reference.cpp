#include <xr/device/abi_reference.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace a081::head {
namespace {
// These checks and zero initialization are choices in this reference, not
// assertions about error handling or padding in the vendor stub.
std::uint32_t size32(std::size_t n)
{
    if (n > std::numeric_limits<std::int32_t>::max())
        throw std::length_error("A081 stub uses signed 32-bit lengths");
    return static_cast<std::uint32_t>(n);
}
void u32(Bytes &b, std::size_t p, std::uint32_t v)
{
    for (unsigned i = 0; i != 4; ++i) b.at(p + i) = static_cast<std::uint8_t>(v >> (8 * i));
}
void u64(Bytes &b, std::size_t p, std::uint64_t v)
{
    u32(b, p, static_cast<std::uint32_t>(v));
    u32(b, p + 4, static_cast<std::uint32_t>(v >> 32));
}
void f32(Bytes &b, std::size_t p, float f)
{
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    std::uint32_t v;
    std::memcpy(&v, &f, sizeof(v));
    u32(b, p, v);
}
template <std::size_t N> Bytes array_bytes(const Opaque<N> &a)
{
    return Bytes(a.begin(), a.end());
}
template <std::size_t N> Bytes records(const std::vector<Opaque<N>> &v)
{
    Bytes out(size32(v.size() * N));
    std::size_t offset = 0;
    for (const auto &item : v) {
        std::copy(item.begin(), item.end(), out.begin() + offset);
        offset += N;
    }
    return out;
}
Bytes nul_string(std::string_view s)
{
    if (s.find('\0') != std::string_view::npos)
        throw std::invalid_argument("embedded NUL in XML/name");
    size32(s.size() + 1);
    Bytes out(s.begin(), s.end());
    out.push_back(0);
    return out;
}
Invocation one_word(std::uint32_t sc, std::uint32_t word)
{
    Invocation call{sc, {Bytes(4)}, {}};
    u32(call.inputs[0], 0, word);
    return call;
}
} // namespace

CameraId64 camera_id(std::string_view name)
{
    if (name.size() > 63 || name.find('\0') != std::string_view::npos)
        throw std::invalid_argument("calibration name exceeds63 bytes or contains NUL");
    CameraId64 out{};
    std::copy(name.begin(), name.end(), out.begin());
    return out;
}

Invocation tracker_device_init(const Bytes &a, const Bytes &b,
                               const std::vector<CameraId64> &ids)
{
    Invocation call{tracker_method::device_init, {Bytes(12), a, b, records(ids)}, {Bytes(4)}};
    u32(call.inputs[0], 0, size32(a.size()));
    u32(call.inputs[0], 4, size32(b.size()));
    u32(call.inputs[0], 8, size32(ids.size()));
    return call;
}

Invocation tracker_set_fds(const TrackerFds72 &fds)
{
    return {tracker_method::set_fds, {array_bytes(fds)}, {Bytes(4)}};
}

Invocation tracker_imu_update(const std::vector<TrackerImu64> &samples,
                              const TrackerState156 &state)
{
    Invocation call{tracker_method::imu_update, {Bytes(160), records(samples)}, {Bytes(160)}};
    u32(call.inputs[0], 0, size32(samples.size()));
    std::copy(state.begin(), state.end(), call.inputs[0].begin() + 4);
    return call;
}

Invocation tracker_camera_update(std::uint64_t word,
                                 const std::vector<TrackerCameraFrame> &frames,
                                 const TrackerState156 &state)
{
    if (frames.size() > 253) throw std::length_error("FastRPC input buffer count exceeds255");
    Invocation call{scalar(5, size32(frames.size() + 2), 1),
                    {Bytes(168), Bytes(frames.size() * 88)}, {Bytes(160)}};
    u64(call.inputs[0], 0, word);
    u32(call.inputs[0], 8, size32(frames.size()));
    std::copy(state.begin(), state.end(), call.inputs[0].begin() + 12);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        auto &wire = call.inputs[1];
        const auto &f = frames[i];
        const auto offset = i * 88;
        std::copy(f.native_prefix.begin(), f.native_prefix.end(), wire.begin() + offset);
        u32(wire, offset + 16, size32(f.payload.size()));
        u32(wire, offset + 20, f.native_word24);
        std::copy(f.native_bytes28_91.begin(), f.native_bytes28_91.end(), wire.begin() + offset + 24);
        call.inputs.push_back(f.payload);
    }
    return call;
}

std::int32_t decode_tracker_result(const Bytes &out, TrackerState156 &state)
{
    if (out.size() != 160) throw std::invalid_argument("tracker result must be160 bytes");
    std::uint32_t value = 0;
    for (unsigned i = 0; i != 4; ++i) value |= static_cast<std::uint32_t>(out[i]) << (8 * i);
    std::copy(out.begin() + 4, out.end(), state.begin());
    std::int32_t signed_value;
    std::memcpy(&signed_value, &value, sizeof(value));
    return signed_value;
}

Invocation qvr_start(std::string_view config6, std::string_view calibration,
                     std::string_view config3, std::uint32_t flags,
                     const std::vector<CameraId64> &ids,
                     const std::vector<float> &coefficients, float a, float b)
{
    Invocation call{qvr_method::start,
                    {Bytes(32), nul_string(config6), nul_string(calibration), nul_string(config3),
                     records(ids), Bytes(size32(coefficients.size() * 4))}, {}};
    auto &p = call.inputs[0];
    for (unsigned i = 0; i != 3; ++i) u32(p, 4 * i, size32(call.inputs[i + 1].size()));
    u32(p, 12, flags);
    u32(p, 16, size32(ids.size()));
    u32(p, 20, size32(coefficients.size()));
    f32(p, 24, a);
    f32(p, 28, b);
    for (std::size_t i = 0; i < coefficients.size(); ++i) f32(call.inputs[5], 4 * i, coefficients[i]);
    return call;
}

Invocation qvr_set_ring_layout(RingId id, const RingDescriptor &d)
{
    Invocation call{qvr_method::ring_layout, {Bytes(8), Bytes(28)}, {}};
    u32(call.inputs[0], 0, static_cast<std::uint32_t>(id));
    u32(call.inputs[0], 4, 7);
    const std::array<std::uint32_t, 7> words{
        static_cast<std::uint32_t>(d.fd), d.block_bytes, d.index_offset, d.ring_offset,
        d.element_bytes, d.count, d.type_tag};
    for (std::size_t i = 0; i < words.size(); ++i) u32(call.inputs[1], i * 4, words[i]);
    return call;
}

Invocation qvr_set_camera_buffers(const std::vector<std::int32_t> &fds,
                                  const std::vector<std::uint32_t> &sizes)
{
    Invocation call{qvr_method::camera_buffers,
                    {Bytes(8), Bytes(size32(fds.size() * 4)), Bytes(size32(sizes.size() * 4))}, {}};
    u32(call.inputs[0], 0, size32(fds.size()));
    u32(call.inputs[0], 4, size32(sizes.size()));
    for (std::size_t i = 0; i < fds.size(); ++i) u32(call.inputs[1], i * 4, static_cast<std::uint32_t>(fds[i]));
    for (std::size_t i = 0; i < sizes.size(); ++i) u32(call.inputs[2], i * 4, sizes[i]);
    return call;
}

Invocation qvr_camera_frame(std::uint32_t width, std::uint32_t height,
                            std::uint64_t time_ns, const Bytes &primary,
                            const Bytes &secondary, const std::vector<CameraId64> &ids,
                            std::int32_t frame_number, std::int32_t aux_frame_number)
{
    Invocation call{qvr_method::camera_frame, {Bytes(40), primary, secondary, records(ids)}, {}};
    auto &p = call.inputs[0];
    u32(p, 0, width);
    u32(p, 4, height);
    u64(p, 8, time_ns);
    u32(p, 16, size32(primary.size()));
    u32(p, 20, size32(secondary.size()));
    u32(p, 24, size32(ids.size()));
    u32(p, 28, static_cast<std::uint32_t>(frame_number));
    u32(p, 32, static_cast<std::uint32_t>(aux_frame_number));
    // +36..39 is tail alignment padding, zeroed by this reference only.
    return call;
}

Invocation qvr_tracking_mode(std::uint32_t dsp_mode)
{
    return one_word(qvr_method::tracking_mode, dsp_mode);
}
Invocation qvr_stop_imu_pose()
{
    return one_word(qvr_method::stop_imu_pose, 41);
}
std::int64_t android_to_qtimer_offset(std::uint64_t qtimer_ns, std::uint64_t boottime_ns)
{
    return static_cast<std::int64_t>(qtimer_ns) - static_cast<std::int64_t>(boottime_ns);
}
std::uint64_t boottime_to_qtimer(std::uint64_t boottime_ns, std::int64_t offset_ns)
{
    return boottime_ns + static_cast<std::uint64_t>(offset_ns);
}
} // namespace a081::head
