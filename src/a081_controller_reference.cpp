#include <xr/device/a081_controller_reference.hpp>

#include <cstring>
#include <limits>

namespace a081::controller {
namespace {
std::uint16_t le16(const std::uint8_t* p)
{
    return std::uint16_t(p[0]) | (std::uint16_t(p[1]) << 8);
}
std::uint32_t le32(const std::uint8_t* p)
{
    return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) |
           (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}
double signed16(const std::uint8_t* p)
{
    const auto bits = le16(p);
    return bits < 0x8000 ? double(bits) : double(int(bits) - 65536);
}
std::int8_t signed8(std::uint8_t v)
{
    return static_cast<std::int8_t>(v < 128 ? int(v) : int(v) - 256);
}
double float32le(const std::uint8_t* p)
{
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    const auto bits = le32(p);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
} // namespace

bool decode_preintegrated_transfer(const std::uint8_t* bytes,
                                  std::size_t size, PreintegratedPacket& output)
{
    if (bytes == nullptr || size != spi_transfer_bytes ||
        bytes[0] != 0xa5 || bytes[1] != 0x5a) {
        return false;
    }
    const auto* p = bytes + 2;
    if (((p[0] >> 4) & 7) != 7) {
        return false;
    }
    unsigned checksum = 0;
    for (std::size_t i = 0; i < 33; ++i) {
        checksum += p[i];
    }
    if (std::uint8_t(checksum) != p[33]) {
        return false;
    }
    output = {
        std::uint8_t(p[0] >> 7), std::uint8_t(p[0] & 15),
        std::uint64_t(le32(p + 3)) | (std::uint64_t(le16(p + 29)) << 32),
        {signed16(p + 7) / 10000.0, signed16(p + 9) / 10000.0,
         signed16(p + 11) / 10000.0, signed16(p + 13) / 10000.0},
        {float32le(p + 15), float32le(p + 19), float32le(p + 23)},
        std::uint8_t(p[2] >> 7), std::uint8_t(p[1] >> 4),
        std::uint8_t(p[1] & 15), std::uint8_t(p[2] & 15),
        {p[27], p[28]},
        {std::int16_t(p[2] >> 7), std::int16_t(p[31]), std::int16_t(p[32])},
    };
    return true;
}

AuxiliaryUpdate decode_auxiliary(const PreintegratedPacket& packet)
{
    AuxiliaryUpdate out{};
    const auto a = packet.auxiliary_bytes[0];
    const auto b = packet.auxiliary_bytes[1];
    if (packet.auxiliary_mode == 2) {
        out.kind = AuxiliaryUpdate::Kind::Joystick;
        out.joystick_x = signed8(b);
        out.joystick_y = signed8(a);
    } else if (packet.auxiliary_mode == 1) {
        out.kind = AuxiliaryUpdate::Kind::ButtonsFingerAndTouch;
        out.buttons_raw = a & 0xf0; // Original value is 0..240, not shifted.
        out.touch_raw = a & 0x0f;
        out.middle_finger_raw = b >> 4;
        out.index_finger_raw = b & 0x0f;
    }
    return out;
}

QuaternionWxyz relative_orientation(QuaternionWxyz p, QuaternionWxyz q)
{
    return {p.w*q.w + p.x*q.x + p.y*q.y + p.z*q.z,
            p.w*q.x - q.w*p.x - q.z*p.y + q.y*p.z,
            p.w*q.y - q.w*p.y - q.x*p.z + q.z*p.x,
            p.w*q.z - q.w*p.z - q.y*p.x + q.x*p.y};
}

bool recover_body_acceleration(QuaternionWxyz q,
                               const std::array<double, 3>& previous,
                               const std::array<double, 3>& current,
                               int delta_ms, std::array<double, 3>& acceleration)
{
    if (delta_ms <= 0) {
        return false;
    }
    const double x = current[0] - previous[0];
    const double y = current[1] - previous[1];
    const double z = current[2] - previous[2];
    const double tx = 2.0 * (q.z*y - q.y*z);
    const double ty = 2.0 * (q.x*z - q.z*x);
    const double tz = 2.0 * (q.y*x - q.x*y);
    const double dt = delta_ms * 0.001;
    acceleration = {(x + q.w*tx + q.z*ty - q.y*tz) / dt,
                    (y + q.w*ty + q.x*tz - q.z*tx) / dt,
                    (z + q.w*tz + q.y*tx - q.x*ty) / dt};
    return true;
}

double ClockFit::to_system(double controller_time) const
{
    return intercept + slope * controller_time;
}
} // namespace a081::controller
