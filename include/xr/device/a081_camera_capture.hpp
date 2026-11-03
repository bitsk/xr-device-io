// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Real-device camera input component. The kernel owner configures the media
// graph and sensor controls first. No graph, GPIO, exposure or sync policy is
// guessed here. Capturing images is separate from admitting them to QVR.
#include <xr/device/a081_camera_metadata.hpp>
#include <xr/device/camera-preview.hpp>
#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace a081::stock::camera {
inline int xioctl(int fd, unsigned long request, void *arg)
{
    int result;
    do { result = ioctl(fd, request, arg); } while (result < 0 && errno == EINTR);
    return result;
}
inline void checked(int result, const char *operation)
{
    if (result < 0) throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
inline std::uint64_t clock_ns(clockid_t id)
{
    timespec now{};
    checked(clock_gettime(id, &now), "camera clock");
    return std::uint64_t(now.tv_sec) * 1000000000ULL + now.tv_nsec;
}
inline void save_bytes(const std::string &path, const head::Bytes &bytes)
{
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    checked(fd, "create camera snapshot");
    std::size_t offset = 0;
    int error = 0;
    while (offset < bytes.size()) {
        const auto n = ::write(fd, bytes.data() + offset, bytes.size() - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) { error = n < 0 ? errno : EIO; break; }
        offset += static_cast<std::size_t>(n);
    }
    if (::close(fd) && !error) error = errno;
    if (error) { errno = error; checked(-1, "write camera snapshot"); }
}

class Capture {
public:
    explicit Capture(std::string node) : node_(std::move(node)) {}
    ~Capture() { close(); }
    Capture(const Capture &) = delete;
    Capture &operator=(const Capture &) = delete;
    int fd() const { return fd_; }
    dev_t device() const { return device_; }
    std::uint64_t frames = 0, errors = 0, sequence_gaps = 0;

    void open()
    {
        if (const char *gate = std::getenv("A081_CAMERA_AE_GATE_FILE")) {
            if (*gate != '/') throw std::runtime_error("AE snapshot gate must be an absolute path");
            ae_gate_file_ = gate;
        }
        fd_ = ::open(node_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        checked(fd_, "open camera");
        struct stat st{};
        checked(fstat(fd_, &st), "camera fstat");
        if (!S_ISCHR(st.st_mode)) throw std::runtime_error("Camera is not a character device");
        device_ = st.st_rdev;
        v4l2_capability cap{};
        checked(xioctl(fd_, VIDIOC_QUERYCAP, &cap), "camera QUERYCAP");
        const auto capabilities = cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps : cap.capabilities;
        if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) || !(capabilities & V4L2_CAP_STREAMING))
            throw std::runtime_error("CAMSS capture-mplane/streaming interface required");
        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        checked(xioctl(fd_, VIDIOC_G_FMT, &format), "camera G_FMT");
        const auto &p = format.fmt.pix_mp;
        if (p.width != width || p.height != height || p.num_planes != 1 || p.field != V4L2_FIELD_NONE ||
            (p.pixelformat != V4L2_PIX_FMT_GREY && p.pixelformat != V4L2_PIX_FMT_Y10P && p.pixelformat != V4L2_PIX_FMT_Y10))
            throw std::runtime_error("Configure 640x480 progressive GREY/Y10P/Y10 first");
        fourcc_ = p.pixelformat;
        stride_ = p.plane_fmt[0].bytesperline;
        size_ = p.plane_fmt[0].sizeimage;
        {
        std::lock_guard<std::mutex> log(output_mutex);
        std::cout << "stage=camera_format node=" << std::quoted(node_)
                  << " driver=" << std::quoted(std::string(reinterpret_cast<char *>(cap.driver), strnlen(reinterpret_cast<char *>(cap.driver), sizeof(cap.driver))))
                  << " bus=" << std::quoted(std::string(reinterpret_cast<char *>(cap.bus_info), strnlen(reinterpret_cast<char *>(cap.bus_info), sizeof(cap.bus_info))))
                  << " fourcc=" << fourcc_ << " width=640 height=480 planes=1 stride=" << stride_
                  << " sizeimage=" << size_ << " calibration_identity=unassigned" << std::endl;
        }
        v4l2_requestbuffers request{};
        request.count = 8;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        request.memory = V4L2_MEMORY_MMAP;
        checked(xioctl(fd_, VIDIOC_REQBUFS, &request), "camera REQBUFS");
        if (request.count < 2 || request.count > 32) throw std::runtime_error("Invalid camera buffer count");
        for (unsigned i = 0; i < request.count; ++i) {
            v4l2_plane plane{};
            auto buffer = descriptor(i, plane);
            checked(xioctl(fd_, VIDIOC_QUERYBUF, &buffer), "camera QUERYBUF");
            if (buffer.length != 1 || plane.length < size_) throw std::runtime_error("Invalid camera plane layout");
            void *address = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, plane.m.mem_offset);
            if (address == MAP_FAILED) checked(-1, "camera mmap");
            mappings_.push_back({address, plane.length});
            checked(xioctl(fd_, VIDIOC_QBUF, &buffer), "camera QBUF");
        }
    }
    void start()
    {
        auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        checked(xioctl(fd_, VIDIOC_STREAMON, &type), "camera STREAMON");
        streaming_ = true;
        std::lock_guard<std::mutex> output_lock(output_mutex);
        std::cout << "stage=camera_stream_started node=" << std::quoted(node_) << std::endl;
    }
    bool dequeue(const std::string &output, unsigned slot, CapturedFrame *frame = nullptr)
    {
        v4l2_plane plane{};
        auto buffer = descriptor(0, plane);
        if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) return false;
            checked(-1, "camera DQBUF");
        }
        const auto receive_mono = clock_ns(CLOCK_MONOTONIC);
        const auto receive_boot = clock_ns(CLOCK_BOOTTIME);
        if (buffer.index >= mappings_.size()) throw std::runtime_error("Invalid dequeued buffer index");
        const auto &mapping = mappings_[buffer.index];
        const bool layout_ok = buffer.length == 1 && plane.data_offset <= plane.bytesused &&
                               plane.bytesused <= mapping.length;
        const bool buffer_error = buffer.flags & V4L2_BUF_FLAG_ERROR;
        const auto *pixels = static_cast<const std::uint8_t *>(mapping.address) + (layout_ok ? plane.data_offset : 0);
        head::Bytes y8, raw;
        const bool converted = !buffer_error && layout_ok &&
            to_y8(pixels, plane.bytesused - plane.data_offset, stride_, fourcc_, y8);
        // The bounded startup-AE owner publishes this marker only after the
        // physical gate rises. Discard 30 subsequent valid frames, keeping
        // pre-barrier/stale startup images out of the exposure decision.
        if (converted && !ae_gate_file_.empty() && ::access(ae_gate_file_.c_str(), F_OK) == 0)
            ++ae_gate_frames_;
        const bool snapshot = converted && !saved_ && !output.empty() &&
            (ae_gate_file_.empty() || ae_gate_frames_ > 30);
        if (snapshot) raw.assign(pixels, pixels + plane.bytesused - plane.data_offset);
        // Copy before returning ownership to DMA; never read queued mappings.
        v4l2_plane queued_plane{};
        queued_plane.length = static_cast<__u32>(mapping.length);
        auto enqueue = descriptor(buffer.index, queued_plane);
        checked(xioctl(fd_, VIDIOC_QBUF, &enqueue), "camera requeue");
        ++frames;
        if (!converted) ++errors;
        bool sequence_gap = false;
        if (have_sequence_) {
            const std::uint32_t delta = buffer.sequence - last_sequence_;
            if (delta > 1 && delta < 0x80000000u) {
                sequence_gaps += delta - 1;
                sequence_gap = true;
            }
        }
        last_sequence_ = buffer.sequence;
        have_sequence_ = true;
        const bool timestamp_ok = buffer.timestamp.tv_sec >= 0 && buffer.timestamp.tv_usec >= 0 && buffer.timestamp.tv_usec < 1000000;
        const auto timestamp = timestamp_ok ? std::uint64_t(buffer.timestamp.tv_sec) * 1000000000ULL +
                                            std::uint64_t(buffer.timestamp.tv_usec) * 1000ULL : 0;
        // The capture thread must return to DQBUF promptly. Full raw QCFT
        // remains available; keep every error and slow dequeue in text too.
        const bool delayed = timestamp && receive_mono >= timestamp &&
                             receive_mono - timestamp >= 100000000ULL;
        if (!sampled_text_ || frames <= 4 || !converted || !timestamp_ok || sequence_gap || delayed ||
            receive_mono >= next_text_ns_) {
        next_text_ns_ = receive_mono + 1000000000ULL;
        std::lock_guard<std::mutex> output_lock(output_mutex);
        std::cout << "stage=camera_frame node=" << std::quoted(node_) << " slot=" << slot
                  << " index=" << buffer.index << " sequence=" << buffer.sequence << " flags=" << buffer.flags
                  << " bytesused=" << plane.bytesused << " data_offset=" << plane.data_offset
                  << " dq_timestamp_ns=" << timestamp << " timestamp_type=" << (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK)
                  << " timestamp_source=" << (buffer.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK)
                  << " receive_monotonic_ns=" << receive_mono << " receive_boottime_ns=" << receive_boot
                  << " buffer_error=" << buffer_error << " y8_converted=" << converted
                  << " exposure_start_verified=0 qvr_submitted=0" << std::endl;
        }
        if (snapshot) {
            const auto prefix = output + "/camera-slot" + std::to_string(slot) + "-first";
            save_bytes(prefix + ".raw", raw);
            save_bytes(prefix + ".y8", y8);
            saved_ = true;
            std::lock_guard<std::mutex> output_lock(output_mutex);
            std::cout << "stage=camera_snapshot slot=" << slot << " sequence=" << buffer.sequence
                      << " prefix=" << std::quoted(prefix) << " projection=linear_msb8" << std::endl;
        }
        if (converted) preview_.y8(slot, buffer.sequence, receive_mono, y8.data(), y8.size());
        if (frame) *frame = {std::move(y8), buffer.sequence, buffer.index, buffer.flags,
                            timestamp, receive_mono, receive_boot};
        return true;
    }
    bool close() noexcept
    {
        bool ok = true;
        if (streaming_) {
            auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            const int rc = xioctl(fd_, VIDIOC_STREAMOFF, &type);
            std::lock_guard<std::mutex> output_lock(output_mutex);
            std::cout << "stage=camera_streamoff node=" << std::quoted(node_) << " rc=" << rc
                      << " errno=" << (rc < 0 ? errno : 0) << std::endl;
            ok = rc == 0;
            streaming_ = false;
        }
        for (const auto &mapping : mappings_) if (munmap(mapping.address, mapping.length)) ok = false;
        mappings_.clear();
        if (fd_ >= 0) { if (::close(fd_)) ok = false; fd_ = -1; }
        return ok;
    }
private:
    static v4l2_buffer descriptor(unsigned index, v4l2_plane &plane)
    {
        v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = index;
        b.length = 1;
        b.m.planes = &plane;
        return b;
    }
    struct Mapping { void *address; std::size_t length; };
    a081::preview::Tap preview_;
    const bool sampled_text_{continuous_capture_enabled()};
    std::uint64_t next_text_ns_{};
    std::string node_;
    std::string ae_gate_file_;
    unsigned ae_gate_frames_{};
    int fd_ = -1;
    dev_t device_{};
    unsigned stride_{}, size_{};
    std::uint32_t fourcc_{}, last_sequence_{};
    bool streaming_{}, saved_{}, have_sequence_{};
    std::vector<Mapping> mappings_;
};

inline volatile std::sig_atomic_t stopped = 0;
inline void stop(int) { stopped = 1; }

// Real MMAP capture, optionally publishing only verified QCFT/image pairs to
// the original-library input thread. This layer never invokes the DSP itself.
struct CaptureCallbacks {
    std::function<void(unsigned, TimedFrame &&)> frame;
    std::function<void(std::uint64_t)> gate;
    std::atomic<bool> *abort{};
};
inline int capture_live(const CaptureCallbacks *callbacks = nullptr)
{
    const char *nodes = std::getenv("A081_QVR_CAMERA_NODES");
    const char *output = std::getenv("A081_QVR_OUTPUT_DIR");
    std::ifstream model_file("/proc/device-tree/model", std::ios::binary);
    const std::string model((std::istreambuf_iterator<char>(model_file)), {});
    if (model.find("A081") == std::string::npos || !nodes || !output || output[0] != '/') {
        std::cerr << "Requires A081, configured A081_QVR_CAMERA_NODES and a fresh A081_QVR_OUTPUT_DIR\n";
        return 2;
    }
    int lock = -1;
    int stop_fd = -1;
    bool parent_stopped = false;
    std::atomic<bool> stop_workers{false}, worker_failed{false};
    std::atomic<bool> stop_timing_workers{false};
    std::vector<std::thread> workers;
    std::vector<std::thread> timing_workers;
    std::vector<std::unique_ptr<TimingStream>> timings;
    std::vector<std::unique_ptr<TimingJoin>> joiners;
    bool ok = true;
    std::vector<std::unique_ptr<Capture>> cameras;
    struct sigaction old_int{}, old_term{}, action{};
    bool int_set = false, term_set = false;
    try {
        struct stat directory{};
        checked(stat(output, &directory), "camera output directory");
        if (!S_ISDIR(directory.st_mode)) throw std::runtime_error("Output must be a directory");
        lock = ::open("/run/a081-camera.lock", O_CREAT | O_RDWR | O_CLOEXEC, 0600);
        checked(lock, "camera lock open");
        checked(flock(lock, LOCK_EX | LOCK_NB), "camera lock");
        // Optional parent-owned pipe: the GPIO owner lowers the gate before
        // sending 'S', so normal teardown cannot STREAMOFF with gate high.
        if (const char *value = std::getenv("A081_QVR_CAMERA_STOP_FD")) {
            char *end = nullptr;
            errno = 0;
            const long parsed = std::strtol(value, &end, 10);
            if (errno || !*value || *end || parsed < 3 || parsed > 1048575)
                throw std::runtime_error("Invalid camera stop pipe fd");
            stop_fd = static_cast<int>(parsed);
            struct stat pipe{};
            checked(fstat(stop_fd, &pipe), "camera stop pipe");
            if (!S_ISFIFO(pipe.st_mode)) throw std::runtime_error("Camera stop fd must be a pipe");
        }
        stopped = 0;
        action.sa_handler = stop;
        sigemptyset(&action.sa_mask);
        if (!callbacks) {
            checked(sigaction(SIGINT, &action, &old_int), "camera SIGINT"); int_set = true;
            checked(sigaction(SIGTERM, &action, &old_term), "camera SIGTERM"); term_set = true;
        }
        std::istringstream input(nodes);
        std::string node;
        while (std::getline(input, node, ',')) {
            if (node.compare(0, 10, "/dev/video") || node.size() == 10 ||
                node.find_first_not_of("0123456789", 10) != std::string::npos || cameras.size() == 4)
                throw std::runtime_error("Use one to four distinct /dev/videoN nodes");
            auto capture = std::make_unique<Capture>(node);
            capture->open();
            for (const auto &other : cameras)
                if (other->device() == capture->device()) throw std::runtime_error("Duplicate camera node");
            cameras.push_back(std::move(capture));
        }
        if (cameras.empty()) throw std::runtime_error("Camera node list is empty");
        if (const char *timing_nodes = std::getenv("A081_QVR_CAMERA_TIMING_NODES")) {
            if (cameras.size() != 4) throw std::runtime_error("QCFT mode requires the validated four-slot A081 graph");
            const char *exposure = std::getenv("A081_QVR_CAMERA_EXPOSURE_NS");
            char *end = nullptr;
            errno = 0;
            const auto exposure_ns = exposure ? std::strtoull(exposure, &end, 10) : 0;
            if (errno || !exposure_ns || !end || *end || exposure_ns > 1000000000ULL)
                throw std::runtime_error("QCFT interpretation requires fixed pre-STREAMON exposure metadata");
            std::istringstream timing_input(timing_nodes);
            while (std::getline(timing_input, node, ',')) {
                if (node.compare(0, 10, "/dev/video") || node.size() == 10 ||
                    node.find_first_not_of("0123456789", 10) != std::string::npos || timings.size() >= cameras.size())
                    throw std::runtime_error("Invalid QCFT node list");
                auto stream = std::make_unique<TimingStream>(node);
                stream->open(output, timings.size());
                const unsigned slot = timings.size();
                std::function<void(TimedFrame &&)> publish;
                if (callbacks && callbacks->frame)
                    publish = [callbacks, slot](TimedFrame &&frame) { callbacks->frame(slot, std::move(frame)); };
                auto slot_exposure_ns = exposure_ns;
                if (slot >= 2) {
                    const char *ctrl_exposure = std::getenv("A081_CONTROLLER_CAMERA_EXPOSURE_NS");
                    if (ctrl_exposure) {
                        char *ctrl_end = nullptr;
                        errno = 0;
                        slot_exposure_ns = std::strtoull(ctrl_exposure, &ctrl_end, 10);
                        if (errno || !slot_exposure_ns || !ctrl_end || *ctrl_end || slot_exposure_ns > 1000000000ULL)
                            throw std::runtime_error("Invalid real controller pair exposure metadata");
                    }
                }
                joiners.push_back(std::make_unique<TimingJoin>(slot, slot_exposure_ns, std::move(publish)));
                timings.push_back(std::move(stream));
            }
            if (timings.size() != cameras.size()) throw std::runtime_error("One ancillary QCFT node per image required");
            timing_workers.reserve(timings.size());
            for (unsigned i = 0; i < timings.size(); ++i) {
                timings[i]->start();
                timing_workers.emplace_back([&, i] {
                    try {
                        // Keep draining until images are stopped, then finish
                        // the already queued records before metadata STREAMOFF.
                        while (true) {
                            pollfd descriptor{timings[i]->fd(), POLLIN, 0};
                            const int rc = poll(&descriptor, 1, 20);
                            if (rc < 0 && errno == EINTR) continue;
                            checked(rc, "QCFT poll");
                            if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                                throw std::runtime_error("QCFT poll error");
                            if (descriptor.revents & POLLIN) {
                                Timing record{}; std::uint32_t flags = 0;
                                if (timings[i]->dequeue(record, flags)) joiners[i]->metadata(record, flags);
                            } else if (stop_timing_workers.load()) break;
                        }
                    } catch (const std::exception &error) {
                        std::lock_guard<std::mutex> log(output_mutex);
                        std::cerr << "stage=camera_timing_error slot=" << i << " detail=" << std::quoted(error.what()) << std::endl;
                        worker_failed.store(true);
                    }
                });
            }
        }
        const auto start = clock_ns(CLOCK_BOOTTIME);
        const char *continuous_value = std::getenv("A081_QVR_CONTINUOUS");
        const bool continuous = stop_fd >= 0 && continuous_value && std::string(continuous_value) == "1";
        const unsigned seconds = continuous ? 0 : (stop_fd < 0 ? 5 : 370);
        {
        std::lock_guard<std::mutex> log(output_mutex);
        std::cout << "stage=camera_capture_scope max_seconds=" << seconds << " nodes=" << cameras.size()
                  << " camera_sync=unverified physical_ids=unassigned qvr_submissions=0" << std::endl;
        }
        // Each stream must drain as soon as its own STREAMON returns. Opening
        // later sensors can take longer than eight frames on the first one.
        // A serial start-all-then-read loop starved slot0 on the real A081
        // (seq8/9 marked ERROR); enlarging queues would only hide that stall.
        workers.reserve(cameras.size());
        for (unsigned i = 0; i < cameras.size(); ++i) {
            // Preserve stock stream-open ordering while earlier streams drain.
            cameras[i]->start();
            workers.emplace_back([&, i] {
            try {
                auto &camera = *cameras[i];
                while (!stop_workers.load() && !stopped) {
                    pollfd descriptor{camera.fd(), POLLIN, 0};
                    const int rc = poll(&descriptor, 1, 20);
                    if (rc < 0 && errno == EINTR) continue;
                    checked(rc, "camera stream poll");
                    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                        throw std::runtime_error("Camera stream poll error");
                    if (descriptor.revents & POLLIN) {
                        CapturedFrame frame;
                        if (camera.dequeue(output, i, timings.empty() ? nullptr : &frame) && !timings.empty())
                            joiners[i]->image(std::move(frame));
                    }
                }
            } catch (const std::exception &error) {
                std::lock_guard<std::mutex> output_lock(output_mutex);
                std::cerr << "stage=camera_error slot=" << i << " detail=" << std::quoted(error.what()) << std::endl;
                worker_failed.store(true);
            }
            });
        }
        while (!stopped && !worker_failed.load() && !(callbacks && callbacks->abort && callbacks->abort->load()) &&
               (continuous || clock_ns(CLOCK_BOOTTIME) - start < std::uint64_t(seconds) * 1000000000ULL)) {
            pollfd stop_request{stop_fd, POLLIN, 0};
            const int rc = poll(&stop_request, 1, 20);
            if (rc < 0 && errno == EINTR) continue;
            checked(rc, "camera poll");
            if (stop_fd >= 0 && stop_request.revents) {
                char command = 0;
                if (!(stop_request.revents & POLLIN) || ::read(stop_fd, &command, 1) != 1)
                    throw std::runtime_error("Camera gate owner disconnected without stop command");
                if (command == 'G' && callbacks && callbacks->gate) {
                    std::uint64_t gate_time = 0;
                    if (::read(stop_fd, &gate_time, sizeof(gate_time)) != sizeof(gate_time) ||
                        gate_time < start || gate_time > clock_ns(CLOCK_BOOTTIME))
                        throw std::runtime_error("Invalid BOOTTIME for physical camera gate event");
                    callbacks->gate(gate_time);
                    continue;
                }
                if (command != 'S') throw std::runtime_error("Unknown camera gate control message");
                parent_stopped = true;
                std::lock_guard<std::mutex> output_lock(output_mutex);
                std::cout << "stage=camera_stop_request monotonic_ns=" << clock_ns(CLOCK_MONOTONIC) << std::endl;
                break;
            }
        }
        ok = !stopped && !worker_failed.load() && (stop_fd < 0 || parent_stopped);
    } catch (const std::exception &error) {
        std::lock_guard<std::mutex> output_lock(output_mutex);
        std::cerr << "stage=camera_error detail=" << std::quoted(error.what()) << std::endl;
        ok = false;
    }
    stop_workers.store(true);
    for (auto &worker : workers) worker.join();
    ok = !worker_failed.load() && ok;
    for (unsigned i = 0; i < cameras.size(); ++i) {
        auto &camera = *cameras[i];
        ok = camera.close() && ok;
        std::lock_guard<std::mutex> log(output_mutex);
        std::cout << "stage=camera_capture_summary slot=" << i << " frames=" << camera.frames
                  << " error_frames=" << camera.errors << " sequence_gaps=" << camera.sequence_gaps
                  << " qvr_submitted=0 visual_6dof=not_tested" << std::endl;
        ok = camera.frames > 0 && camera.errors == 0 && ok;
    }
    stop_timing_workers.store(true);
    for (auto &worker : timing_workers) worker.join();
    for (auto &timing : timings) ok = timing->close() && ok;
    for (auto &joiner : joiners) ok = joiner->summary() && ok;
    ok = !worker_failed.load() && ok;
    if (term_set) sigaction(SIGTERM, &old_term, nullptr);
    if (int_set) sigaction(SIGINT, &old_int, nullptr);
    if (lock >= 0) ::close(lock);
    return ok ? 0 : 1;
}
} // namespace a081::stock::camera
