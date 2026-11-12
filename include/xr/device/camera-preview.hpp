// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Optional observer tap. One object per capture thread. Never waits on the
// consumer and never passes a DMA mapping past QBUF. A missing viewer disables
// sampling; backpressure drops previews, not tracking frames.
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace a081::preview {
class Tap {
    int fd_ = -1;
    sockaddr_un address_{};
    std::array<std::uint64_t, 4> next_{}; // short0, short1, long2, unknown
    std::uint64_t session_ = 0;
    unsigned lane_ = 3;
    std::array<std::uint8_t, 104 + 128 * 1024> packet_{};
    bool enabled_ = false;
    void word(unsigned at, std::uint64_t value, unsigned bytes = 4) {
        for (unsigned i = 0; i < bytes; ++i) packet_[at+i] = (value >> (i*8)) & 255;
    }
    bool ready(std::uint64_t now, unsigned lane = 3) {
        lane_ = lane;
        if (!enabled_ || now < next_[lane_]) return false;
        next_[lane_] = now + 33333333; // At most 30 frames/s per camera AND phase; no phase starvation.
        if (fd_ < 0) {
            fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
            if (fd_ < 0) { next_[lane_] = now + 1000000000; return false; }
            int size = 512 * 1024;
            setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
        }
        // The receiver binds only while a viewer requests frames.
        if (access(address_.sun_path, F_OK)) { next_[lane_] = now + 1000000000; return false; }
        return true;
    }
    void send(unsigned slot, std::uint64_t sequence, std::uint64_t now,
              unsigned width, unsigned height, unsigned channels, unsigned header = 48) {
        const unsigned size = width*height*channels;
        std::memcpy(packet_.data(), header == 104 ? "ACP2" : "ACP1", 4);
        word(4, slot); word(8, session_, 8); word(16, sequence, 8); word(24, now, 8);
        word(32, width); word(36, height); word(40, channels); word(44, size);
        if (sendto(fd_, packet_.data(), header+size, MSG_DONTWAIT | MSG_NOSIGNAL,
                   reinterpret_cast<sockaddr *>(&address_), sizeof(address_)) < 0)
            next_[lane_] = now + 100000000; // No retry of an old frame.
    }
public:
    Tap() {
        const char *path = std::getenv("A081_CAMERA_PREVIEW_SOCKET");
        if (!path || path[0] != '/' || std::strlen(path) >= sizeof(address_.sun_path)) return;
        address_.sun_family = AF_UNIX;
        std::strcpy(address_.sun_path, path);
        timespec ts{};
        if (clock_gettime(CLOCK_MONOTONIC, &ts)) return;
        session_ = (std::uint64_t(ts.tv_sec)*1000000000 + ts.tv_nsec) ^ (std::uint64_t(getpid()) << 32);
        if (!session_) session_ = 1;
        enabled_ = true;
    }
    ~Tap() { if (fd_ >= 0) close(fd_); }
    Tap(const Tap &) = delete;
    Tap &operator=(const Tap &) = delete;
    void y8(unsigned slot, std::uint64_t sequence, std::uint64_t now,
            const std::uint8_t *pixels, std::size_t length) {
        if (slot >= 4 || length != 640*480 || !ready(now)) return;
        for (unsigned y = 0; y < 240; ++y)
            for (unsigned x = 0; x < 320; ++x)
                packet_[48+y*320+x] = pixels[(y*2)*640+x*2];
        send(slot, sequence, now, 320, 240, 1);
    }
    // Called only after image/QCFT sequence, index and EOF association.
    // Invalid/missing phase evidence stays in the unclassified stream.
    void timed_y8(unsigned slot, std::uint64_t sequence, std::uint64_t receive_mono,
                  const std::uint8_t *pixels, std::size_t length, bool joined,
                  bool phase_valid, unsigned phase, std::uint64_t exposure_ns,
                  std::uint64_t sensor_sequence, std::uint64_t sof_mono,
                  std::uint64_t stream_id, std::uint64_t anchor,
                  std::int32_t exposure_status, unsigned timing_flags) {
        const bool classified = joined && phase_valid && phase < 3;
        if (slot >= 4 || length != 640*480 || !ready(receive_mono, classified ? phase : 3)) return;
        word(48, phase < 3 ? phase : UINT32_MAX);
        word(52, (joined ? 1u : 0u) | (classified ? 2u : 0u));
        word(56, exposure_ns, 8); word(64, sensor_sequence, 8); word(72, sof_mono, 8);
        word(80, stream_id, 8); word(88, anchor, 8);
        word(96, static_cast<std::uint32_t>(exposure_status)); word(100, timing_flags);
        for (unsigned y = 0; y < 240; ++y)
            for (unsigned x = 0; x < 320; ++x)
                packet_[104+y*320+x] = pixels[(y*2)*640+x*2];
        send(slot, sequence, receive_mono, 320, 240, 1, 104);
    }
    // IMX471 pRAA: RGGB packed RAW10, high eight bits are the first four
    // bytes in each five-byte group. Downsample one Bayer cell per 8x8 block;
    // no white balance/ISP/lens correction is claimed.
    void rggb10(unsigned slot, std::uint64_t sequence, std::uint64_t now,
                const std::uint8_t *pixels, std::size_t length, unsigned stride,
                unsigned width = 1928, unsigned height = 1088) {
        if (slot < 4 || slot > 5 || !((width == 1928 && height == 1088) ||
            (width == 1728 && height == 1728)) || stride < width*5/4 ||
            length < std::size_t(stride)*height || !ready(now)) return;
        const unsigned step = height > 1088 ? 12 : 8;
        const unsigned w = width/step, h = height/step;
        for (unsigned y = 0; y < h; ++y) {
            for (unsigned x = 0; x < w; ++x) {
                const unsigned pixel = x*step;
                const auto *top = pixels+(y*step)*stride+(pixel/4)*5+pixel%4;
                const auto *bottom = top+stride;
                auto *rgb = packet_.data()+48+(y*w+x)*3;
                rgb[0] = top[0]; rgb[1] = (unsigned(top[1])+bottom[0])/2; rgb[2] = bottom[1];
            }
        }
        send(slot, sequence, now, w, h, 3);
    }
};
}
