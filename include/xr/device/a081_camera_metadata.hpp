// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <xr/device/a081_camera_pixels.hpp>
#include <xr/device/camera-preview.hpp>
#include <xr/device/a081_camera_timing.hpp>
#include <linux/qcom-camss-timing.h>
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace a081::stock::camera {
inline std::mutex output_mutex;
inline bool continuous_capture_enabled()
{
    const char *value = std::getenv("A081_QVR_CONTINUOUS");
    return value && std::string(value) == "1";
}
struct CapturedFrame {
    head::Bytes y8;
    std::uint32_t sequence{}, buffer_index{}, flags{};
    std::uint64_t dq_timestamp_ns{}, receive_monotonic_ns{}, receive_boottime_ns{};
};
using Timing = qcom_camss_frame_timing;
static_assert(sizeof(Timing) == 216 && offsetof(Timing, stream_id) == 32 &&
              offsetof(Timing, sof_boottime_ns) == 96 && offsetof(Timing, header_words) == 128 && offsetof(Timing, sensor_sequence) == 160);

// This proves the kernel associated a successful control write with this SOF.
// It does not establish the physical MCU LED phase or optical pose validity.
inline bool valid_pattern_exposure(const Timing &r)
{
    constexpr auto required = QCOM_CAMSS_TIMING_PATTERN_ACTIVE | QCOM_CAMSS_TIMING_EXPOSURE_VALID;
    return (r.flags & required) == required && !r.exposure_status && r.exposure_phase < 3 &&
        r.exposure_lines && r.analogue_gain && r.frame_length_lines &&
        r.exposure_ns && r.exposure_ns <= 1000000000ULL &&
        r.phase_anchor_sof_qtimer && r.control_apply_sof_qtimer >= r.phase_anchor_sof_qtimer &&
        r.control_apply_sof_qtimer < r.sof_qtimer && r.control_applied_boottime_ns &&
        r.control_applied_boottime_ns < r.sof_boottime_ns;
}
inline std::uint64_t frame_exposure_ns(const Timing &r, std::uint64_t fixed)
{
    if (!(r.flags & QCOM_CAMSS_TIMING_PATTERN_ACTIVE)) return fixed;
    return valid_pattern_exposure(r) ? r.exposure_ns : 0;
}

inline void timing_ioctl(int fd, unsigned long operation, void *value)
{
    int rc;
    do { rc = ioctl(fd, operation, value); } while (rc < 0 && errno == EINTR);
    if (rc < 0) throw std::runtime_error(std::string("QCFT ioctl: ") + std::strerror(errno));
}

class TimingStream {
public:
    explicit TimingStream(std::string node) : node_(std::move(node)) {}
    ~TimingStream() { close(); }
    int fd() const { return fd_; }
    void open(const std::string &output, unsigned slot)
    {
        fd_ = ::open(node_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw std::runtime_error("Open QCFT node failed");
        v4l2_capability caps{};
        timing_ioctl(fd_, VIDIOC_QUERYCAP, &caps);
        const auto cap = caps.capabilities & V4L2_CAP_DEVICE_CAPS ? caps.device_caps : caps.capabilities;
        if (!(cap & V4L2_CAP_META_CAPTURE) || !(cap & V4L2_CAP_STREAMING))
            throw std::runtime_error("QCFT META_CAPTURE/STREAMING required");
        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_META_CAPTURE;
        timing_ioctl(fd_, VIDIOC_G_FMT, &format);
        if (format.fmt.meta.dataformat != V4L2_META_FMT_QCOM_CAMSS_TIMING || format.fmt.meta.buffersize != sizeof(Timing))
            throw std::runtime_error("QCFT v2 216-byte format required");
        v4l2_requestbuffers request{};
        request.count = 8; request.type = V4L2_BUF_TYPE_META_CAPTURE; request.memory = V4L2_MEMORY_MMAP;
        timing_ioctl(fd_, VIDIOC_REQBUFS, &request);
        if (request.count < 2 || request.count > 32) throw std::runtime_error("QCFT buffer count invalid");
        maps_.reserve(request.count);
        for (unsigned i = 0; i < request.count; ++i) {
            auto b = buffer(i);
            timing_ioctl(fd_, VIDIOC_QUERYBUF, &b);
            if (b.length < sizeof(Timing)) throw std::runtime_error("QCFT buffer too short");
            void *p = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, b.m.offset);
            if (p == MAP_FAILED) throw std::runtime_error("QCFT mmap failed");
            maps_.push_back({p, b.length});
            timing_ioctl(fd_, VIDIOC_QBUF, &b);
        }
        archive_path_ = output + "/camera-slot" + std::to_string(slot) + "-timing.bin";
        const auto &path = archive_path_;
        const char *continuous_value = std::getenv("A081_QVR_CONTINUOUS");
        retain_segments_ = continuous_value && std::string(continuous_value) == "1";
        archive_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
        if (archive_ < 0) throw std::runtime_error("Create QCFT raw record archive failed");
        std::lock_guard<std::mutex> log(output_mutex);
        std::cout << "stage=camera_timing_format slot=" << slot << " node=" << std::quoted(node_)
                  << " fourcc=QCFT bytes=216 version=2 buffers=" << request.count << std::endl;
    }
    void start()
    {
        auto type = V4L2_BUF_TYPE_META_CAPTURE;
        timing_ioctl(fd_, VIDIOC_STREAMON, &type);
        streaming_ = true;
        std::lock_guard<std::mutex> log(output_mutex);
        std::cout << "stage=camera_timing_started node=" << std::quoted(node_) << std::endl;
    }
    bool dequeue(Timing &out, std::uint32_t &flags)
    {
        auto b = buffer(0);
        int rc;
        do { rc = ioctl(fd_, VIDIOC_DQBUF, &b); } while (rc < 0 && errno == EINTR);
        if (rc < 0 && errno == EAGAIN) return false;
        if (rc < 0) throw std::runtime_error("QCFT DQBUF failed");
        if (b.index >= maps_.size() || b.bytesused != sizeof(Timing))
            throw std::runtime_error("QCFT payload/index invalid");
        std::memcpy(&out, maps_[b.index].address, sizeof(out));
        flags = b.flags;
        auto enqueue = buffer(b.index);
        timing_ioctl(fd_, VIDIOC_QBUF, &enqueue);
        if (out.version != QCOM_CAMSS_TIMING_VERSION || out.length != sizeof(out) || b.sequence != out.image_sequence)
            throw std::runtime_error("QCFT version/length/sequence mismatch");
        constexpr std::uint64_t segment_limit = (8ULL * 1024 * 1024 / sizeof(Timing)) * sizeof(Timing);
        if (retain_segments_ && archive_segment_bytes_ >= segment_limit) {
            const int rc = ::close(archive_); archive_ = -1;
            const auto retained = archive_path_ + (archive_segments_ ? ".previous" : ".first");
            if (rc || ::rename(archive_path_.c_str(), retained.c_str()))
                throw std::runtime_error("QCFT archive rotation failed");
            archive_ = ::open(archive_path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
            if (archive_ < 0) throw std::runtime_error("Create QCFT next archive failed");
            ++archive_segments_; archive_segment_bytes_ = 0;
            std::lock_guard<std::mutex> log(output_mutex);
            std::cout << "stage=camera_timing_archive_rotation node=" << std::quoted(node_)
                      << " completed_segments=" << archive_segments_ << " records=" << archive_records_
                      << " next_stream_id=" << out.stream_id << " next_sequence=" << out.image_sequence
                      << " segment_limit=" << segment_limit << " retained=first+previous+current" << std::endl;
        }
        const auto *bytes = reinterpret_cast<const std::uint8_t *>(&out);
        std::size_t offset = 0;
        while (offset < sizeof(out)) {
            const auto n = ::write(archive_, bytes + offset, sizeof(out) - offset);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) throw std::runtime_error("QCFT archive write failed");
            offset += static_cast<std::size_t>(n);
        }
        ++archive_records_; archive_segment_bytes_ += sizeof(out);
        return true;
    }
    bool close() noexcept
    {
        bool ok = true;
        if (streaming_) {
            auto type = V4L2_BUF_TYPE_META_CAPTURE;
            const int rc = ioctl(fd_, VIDIOC_STREAMOFF, &type);
            std::cout << "stage=camera_timing_streamoff node=" << std::quoted(node_) << " rc=" << rc << std::endl;
            ok = rc == 0; streaming_ = false;
        }
        for (const auto &m : maps_) if (munmap(m.address, m.bytes)) ok = false;
        maps_.clear();
        if (fd_ >= 0) { if (::close(fd_)) ok = false; fd_ = -1; }
        if (archive_ >= 0) {
            if (::close(archive_)) ok = false;
            archive_ = -1;
            std::lock_guard<std::mutex> log(output_mutex);
            std::cout << "stage=camera_timing_archive_summary node=" << std::quoted(node_)
                      << " records=" << archive_records_ << " completed_segments=" << archive_segments_
                      << " current_bytes=" << archive_segment_bytes_ << " bounded=" << retain_segments_ << std::endl;
        }
        return ok;
    }
private:
    static v4l2_buffer buffer(unsigned index)
    {
        v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_META_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = index; return b;
    }
    struct Mapping { void *address; std::size_t bytes; };
    std::vector<Mapping> maps_;
    std::string node_;
    int fd_ = -1, archive_ = -1;
    bool streaming_{}, retain_segments_{};
    std::string archive_path_;
    std::uint64_t archive_records_{}, archive_segments_{}, archive_segment_bytes_{};
};

struct TimedFrame { CapturedFrame image; Timing timing; };
class TimingJoin {
public:
    TimingJoin(unsigned slot, std::uint64_t exposure_ns, std::function<void(TimedFrame &&)> publish = {})
        : slot_(slot), exposure_ns_(exposure_ns), publish_(std::move(publish)) {}
    void image(CapturedFrame &&frame)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        const auto sequence = frame.sequence;
        if (!images_.emplace(sequence, std::move(frame)).second) ++rejected_;
        match(sequence);
        if (images_.size() > 32) { images_.erase(images_.begin()); ++missing_metadata_; }
    }
    void metadata(const Timing &r, std::uint32_t flags)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        ++records_;
        if (!stream_id_) stream_id_ = r.stream_id;
        if (!r.stream_id || r.stream_id != stream_id_)
            throw std::runtime_error("QCFT stream generation changed within one capture");
        dropped_ = std::max(dropped_, std::uint64_t(r.metadata_dropped));
        const auto header_sof = std::uint64_t(r.header_words[1]) << 24 | r.header_words[0] >> 8;
        constexpr auto required = QCOM_CAMSS_TIMING_VALID | QCOM_CAMSS_TIMING_HEADER_PRESENT | QCOM_CAMSS_TIMING_HEADER_MATCHED;
        bool valid = !(flags & V4L2_BUF_FLAG_ERROR) && (r.flags & required) == required &&
            !r.lookup_status && r.qtimer_frequency_hz == 19200000 && r.clock_bracket_ns < 5000 &&
            r.sof_qtimer && header_sof == r.sof_qtimer && r.sof_qtimer < r.eof_qtimer && r.eof_qtimer <= r.sample_qtimer &&
            r.sof_monotonic_ns && r.eof_monotonic_ns > r.sof_monotonic_ns &&
            r.sof_boottime_ns && r.eof_boottime_ns > r.sof_boottime_ns && r.csid == slot_ && r.rdi == 0;
        for (auto reserved : r.reserved) valid = valid && reserved == 0;
        if (valid) {
            valid = r.sample_qtimer - r.sof_qtimer < 19200000;
            if (valid) {
                const auto sof_delta = (r.sample_qtimer - r.sof_qtimer) * 10000 / 192;
                const auto eof_delta = (r.sample_qtimer - r.eof_qtimer) * 10000 / 192;
                valid = r.sample_monotonic_ns >= sof_delta && r.sample_boottime_ns >= sof_delta &&
                    r.sof_monotonic_ns == r.sample_monotonic_ns - sof_delta &&
                    r.eof_monotonic_ns == r.sample_monotonic_ns - eof_delta &&
                    r.sof_boottime_ns == r.sample_boottime_ns - sof_delta &&
                    r.eof_boottime_ns == r.sample_boottime_ns - eof_delta;
            }
        }
        if (!valid) ++invalid_metadata_;
        if (!timings_.emplace(r.image_sequence, Entry{r, valid}).second) ++rejected_;
        match(r.image_sequence);
        if (timings_.size() > 32) { timings_.erase(timings_.begin()); ++orphan_metadata_; }
    }
    bool summary()
    {
        // Called only after both workers have stopped and metadata is drained.
        missing_metadata_ += images_.size(); orphan_metadata_ += timings_.size();
        images_.clear(); timings_.clear();
        std::lock_guard<std::mutex> log(output_mutex);
        std::cout << "stage=camera_timing_summary slot=" << slot_ << " stream_id=" << stream_id_
                  << " records=" << records_ << " joined=" << joined_ << " rejected=" << rejected_
                  << " invalid_metadata=" << invalid_metadata_ << " metadata_dropped=" << dropped_
                  << " images_without_metadata=" << missing_metadata_ << " metadata_without_dequeued_image=" << orphan_metadata_
                  << " qvr_submitted=0" << std::endl;
        // A final metadata record can belong to an image not dequeued before
        // STREAMOFF. Count it separately; never invent an image to match it.
        return joined_ > 0 && !rejected_ && !invalid_metadata_ && !dropped_ && !missing_metadata_;
    }
private:
    a081::preview::Tap preview_;
    struct Entry { Timing record; bool valid; };
    void match(std::uint32_t sequence)
    {
        auto image = images_.find(sequence); auto meta = timings_.find(sequence);
        if (image == images_.end() || meta == timings_.end()) return;
        const auto &i = image->second; const auto &t = meta->second.record;
        const bool valid = meta->second.valid && !(i.flags & V4L2_BUF_FLAG_ERROR) && i.y8.size() == image_bytes &&
            (i.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC &&
            (i.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) == V4L2_BUF_FLAG_TSTAMP_SRC_EOF &&
            i.buffer_index == t.image_buffer_index && i.dq_timestamp_ns == t.eof_monotonic_ns / 1000 * 1000;
        if (valid) ++joined_; else ++rejected_;
        StockY8Time stock_time{};
        const auto effective_exposure = frame_exposure_ns(t, exposure_ns_);
        const bool interpreted = valid && effective_exposure && stock_a081_y8_time(t.sof_boottime_ns, effective_exposure, stock_time);
        // Complete QCFT records are archived before joining. Limit only
        // healthy continuous-mode text; never suppress a rejected pair.
        const auto now = i.receive_monotonic_ns;
        if (!sampled_text_ || joined_ + rejected_ <= 4 || !valid || !interpreted || now >= next_text_ns_) {
            next_text_ns_ = now + 1000000000ULL;
            std::lock_guard<std::mutex> log(output_mutex);
            std::cout << "stage=camera_timing_join slot=" << slot_ << " stream_id=" << t.stream_id
                      << " sequence=" << sequence << " index=" << i.buffer_index << " cookie=" << t.enqueue_cookie
                      << " valid=" << valid << " sof_qtimer=" << t.sof_qtimer << " eof_qtimer=" << t.eof_qtimer
                      << " sof_boottime_ns=" << t.sof_boottime_ns << " eof_boottime_ns=" << t.eof_boottime_ns
                      << " bracket_ns=" << t.clock_bracket_ns << " fixed_nominal_exposure_ns=" << exposure_ns_
                      << " exposure_phase=" << t.exposure_phase << " exposure_status=" << t.exposure_status
                      << " effective_exposure_ns=" << effective_exposure << " exposure_flags=" << t.flags
                      << " phase_anchor_sof_qtimer=" << t.phase_anchor_sof_qtimer
                      << " stock_y8_submit_ns=" << (interpreted ? stock_time.submit_boottime_ns : 0)
                      << " qvr_submitted=0" << std::endl;
        }
        preview_.timed_y8(slot_, sequence, i.receive_monotonic_ns, i.y8.data(), i.y8.size(),
            valid, valid && valid_pattern_exposure(t), t.exposure_phase, effective_exposure,
            t.sensor_sequence, t.sof_monotonic_ns, t.stream_id, t.phase_anchor_sof_qtimer,
            t.exposure_status, t.flags);
        if (valid && publish_) publish_(TimedFrame{std::move(image->second), t});
        images_.erase(image); timings_.erase(meta);
    }
    const bool sampled_text_{continuous_capture_enabled()};
    std::uint64_t next_text_ns_{};
    std::mutex mutex_;
    std::map<std::uint32_t, CapturedFrame> images_;
    std::map<std::uint32_t, Entry> timings_;
    unsigned slot_;
    std::uint64_t exposure_ns_, stream_id_{}, records_{}, joined_{}, rejected_{}, invalid_metadata_{}, dropped_{}, missing_metadata_{}, orphan_metadata_{};
    std::function<void(TimedFrame &&)> publish_;
};
} // namespace a081::stock::camera
