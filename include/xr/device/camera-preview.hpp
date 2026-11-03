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
    std::uint64_t next_ = 0, session_ = 0;
    std::array<std::uint8_t, 48 + 128 * 1024> packet_{};
    bool enabled_ = false;
    void word(unsigned at, std::uint64_t value, unsigned bytes = 4) {
        for (unsigned i = 0; i < bytes; ++i) packet_[at+i] = (value >> (i*8)) & 255;
    }
    bool ready(std::uint64_t now) {
        if (!enabled_ || now < next_) return false;
        next_ = now + 66666667; // At most 15 distinct source frames/s per slot.
        if (fd_ < 0) {
            fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
            if (fd_ < 0) { next_ = now + 1000000000; return false; }
            int size = 512 * 1024;
            setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &size, sizeof(size));
        }
        // The receiver binds only while an authenticated viewer requests frames.
        if (access(address_.sun_path, F_OK)) { next_ = now + 1000000000; return false; }
        return true;
    }
    void send(unsigned slot, std::uint64_t sequence, std::uint64_t now,
              unsigned width, unsigned height, unsigned channels) {
        const unsigned size = width*height*channels;
        std::memcpy(packet_.data(), "ACP1", 4);
        word(4, slot); word(8, session_, 8); word(16, sequence, 8); word(24, now, 8);
        word(32, width); word(36, height); word(40, channels); word(44, size);
        if (sendto(fd_, packet_.data(), 48+size, MSG_DONTWAIT | MSG_NOSIGNAL,
                   reinterpret_cast<sockaddr *>(&address_), sizeof(address_)) < 0)
            next_ = now + 100000000; // No retry of an old frame.
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
    // IMX471 pRAA: RGGB packed RAW10, high eight bits are the first four
    // bytes in each five-byte group. Downsample one Bayer cell per 8x8 block;
    // no white balance/ISP/lens correction is claimed.
    void rggb10(unsigned slot, std::uint64_t sequence, std::uint64_t now,
                const std::uint8_t *pixels, std::size_t length, unsigned stride) {
        if (slot < 4 || slot > 5 || stride < 2410 || length < std::size_t(stride)*1088 || !ready(now)) return;
        for (unsigned y = 0; y < 136; ++y) {
            for (unsigned x = 0; x < 241; ++x) {
                const auto *top = pixels+(y*8)*stride+x*10;
                const auto *bottom = top+stride;
                auto *rgb = packet_.data()+48+(y*241+x)*3;
                rgb[0] = top[0]; rgb[1] = (unsigned(top[1])+bottom[0])/2; rgb[2] = bottom[1];
            }
        }
        send(slot, sequence, now, 241, 136, 3);
    }
};
}
