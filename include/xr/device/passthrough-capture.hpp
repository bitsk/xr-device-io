// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// IMX471 buffers are owned by the same capture process and camera lock as the
// four tracking inputs. Configuration belongs to the session owner. No DSP
// submissions or tracking-camera controls are made by this component.
#include <xr/device/camera-preview.hpp>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <vector>
#include <xr/device/passthrough-export.hpp>

namespace a081::device::passthrough {
inline void checked(int result, const char *operation) {
    if (result < 0) throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
inline int ioctl_retry(int fd, unsigned long operation, void *argument) {
    int result;
    do { result = ioctl(fd, operation, argument); } while (result < 0 && errno == EINTR);
    return result;
}
inline std::uint64_t monotonic_ns() {
    timespec ts{};
    checked(clock_gettime(CLOCK_MONOTONIC, &ts), "IMX clock");
    return std::uint64_t(ts.tv_sec)*1000000000 + ts.tv_nsec;
}
class Capture {
    int fd_ = -1;
    dev_t device_{};
    bool streaming_ = false;
    unsigned stride_ = 0, width_ = 0, height_ = 0;
    struct Mapping { void *data; std::size_t size; };
    std::vector<Mapping> maps_;
    a081::preview::Tap tap_;
    SnapshotRing export_;
    static v4l2_buffer buffer(v4l2_plane &plane, unsigned index = 0) {
        v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.memory = V4L2_MEMORY_MMAP; b.index = index; b.length = 1; b.m.planes = &plane;
        return b;
    }
public:
    std::uint64_t frames = 0, invalid = 0;
    int fd() const { return fd_; }
    dev_t device() const { return device_; }
    Capture() = default;
    Capture(const Capture &) = delete;
    Capture &operator=(const Capture &) = delete;
    void open(const char *path) {
        std::string node(path);
        if (node.compare(0, 10, "/dev/video") || node.size() == 10 || node.find_first_not_of("0123456789", 10) != std::string::npos)
            throw std::runtime_error("Expected configured /dev/videoN node");
        fd_ = ::open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
        checked(fd_, "open camera");
        struct stat st{}; checked(fstat(fd_, &st), "fstat");
        device_ = st.st_rdev;
        if (!S_ISCHR(st.st_mode)) throw std::runtime_error("Not a camera character device");
        v4l2_capability cap{}; checked(ioctl_retry(fd_, VIDIOC_QUERYCAP, &cap), "QUERYCAP");
        if (std::strncmp(reinterpret_cast<char *>(cap.driver), "qcom-camss", sizeof(cap.driver)))
            throw std::runtime_error("Expected CAMSS capture node");
        v4l2_format fmt{}; fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        checked(ioctl_retry(fd_, VIDIOC_G_FMT, &fmt), "G_FMT");
        const auto &p = fmt.fmt.pix_mp;
        stride_ = p.plane_fmt[0].bytesperline;
        width_ = p.width; height_ = p.height;
        if (!((p.width == 1928 && p.height == 1088) || (p.width == 1728 && p.height == 1728)) || p.num_planes != 1 || p.field != V4L2_FIELD_NONE ||
            p.pixelformat != V4L2_PIX_FMT_SRGGB10P || stride_ < width_*5/4 || p.plane_fmt[0].sizeimage < stride_*height_)
            throw std::runtime_error("Configure a supported IMX471 pRAA crop first");
        v4l2_requestbuffers req{}; req.type = fmt.type; req.memory = V4L2_MEMORY_MMAP; req.count = 8;
        checked(ioctl_retry(fd_, VIDIOC_REQBUFS, &req), "REQBUFS");
        if (req.count < 2 || req.count > 32) throw std::runtime_error("Invalid buffer count");
        for (unsigned i = 0; i < req.count; ++i) {
            v4l2_plane plane{}; auto b = buffer(plane, i);
            checked(ioctl_retry(fd_, VIDIOC_QUERYBUF, &b), "QUERYBUF");
            if (b.length != 1 || plane.length < p.plane_fmt[0].sizeimage) throw std::runtime_error("Invalid plane");
            void *data = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, plane.m.mem_offset);
            if (data == MAP_FAILED) checked(-1, "mmap");
            maps_.push_back({data, plane.length});
            checked(ioctl_retry(fd_, VIDIOC_QBUF, &b), "QBUF");
        }
    }
    void start() {
        auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        checked(ioctl_retry(fd_, VIDIOC_STREAMON, &type), "STREAMON"); streaming_ = true;
    }
    void frame(unsigned slot) {
        v4l2_plane plane{}; auto b = buffer(plane);
        if (ioctl_retry(fd_, VIDIOC_DQBUF, &b) < 0) { if (errno == EAGAIN) return; checked(-1, "DQBUF"); }
        const auto stamp = monotonic_ns();
        if (b.index >= maps_.size()) throw std::runtime_error("Invalid buffer index");
        const auto &map = maps_[b.index];
        if (!(b.flags & V4L2_BUF_FLAG_ERROR) && b.length == 1 &&
            plane.data_offset <= plane.bytesused && plane.bytesused <= map.size &&
            plane.bytesused-plane.data_offset >= std::size_t(stride_)*height_) {
            ++frames;
            tap_.rggb10(slot, b.sequence, stamp, static_cast<const std::uint8_t *>(map.data)+plane.data_offset,
                        plane.bytesused-plane.data_offset, stride_, width_, height_);
            export_.publish(slot, b.sequence, stamp, static_cast<const std::uint8_t *>(map.data)+plane.data_offset,
                            plane.bytesused-plane.data_offset, stride_, width_, height_);
        } else { ++invalid; }
        // Nonblocking tap copies/sends before ownership goes back to DMA.
        v4l2_plane queued{}; queued.length = map.size; auto q = buffer(queued, b.index);
        checked(ioctl_retry(fd_, VIDIOC_QBUF, &q), "requeue");
    }
    bool close() {
        bool ok = true;
        if (streaming_) {
            auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            if (ioctl_retry(fd_, VIDIOC_STREAMOFF, &type) < 0) { std::perror("IMX STREAMOFF"); ok = false; }
            streaming_ = false;
        }
        for (const auto &map : maps_) if (munmap(map.data, map.size)) ok = false;
        maps_.clear();
        if (fd_ >= 0) { if (::close(fd_)) ok = false; fd_ = -1; }
        return ok;
    }
    ~Capture() { close(); }
};
}
