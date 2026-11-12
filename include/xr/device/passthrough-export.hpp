// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// QSP1: three fixed-size RAW10 snapshot files per physical camera. All pixels
// are copied before QBUF. Readers take a shared flock while copying; the writer
// only tries an exclusive lock and skips busy buffers. No consumer delays DMA.
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>

namespace a081::device::passthrough {
struct SnapshotHeader {
    std::uint32_t magic, version, width, height, stride, format, slot, payload_bytes;
    std::uint64_t generation, sequence, received_monotonic_ns, reserved;
};
static_assert(sizeof(SnapshotHeader) == 64);
class SnapshotRing {
    struct Entry { int fd = -1; void *data = MAP_FAILED; std::string path; ino_t inode = 0; };
    std::array<Entry, 3> entries_;
    std::size_t bytes_ = 0;
    unsigned next_ = 0;
    bool attempted_ = false;
public:
    SnapshotRing() = default;
    SnapshotRing(const SnapshotRing &) = delete;
    SnapshotRing &operator=(const SnapshotRing &) = delete;
    ~SnapshotRing() {
        for (auto &e : entries_) {
            if (e.data != MAP_FAILED) munmap(e.data, bytes_);
            if (e.fd >= 0) ::close(e.fd);
            struct stat st{};
            if (e.inode && !lstat(e.path.c_str(), &st) && st.st_ino == e.inode) unlink(e.path.c_str());
        }
    }
    void publish(unsigned slot, std::uint64_t sequence, std::uint64_t stamp,
                 const std::uint8_t *pixels, std::size_t length, unsigned stride, unsigned width, unsigned height) {
        const std::size_t payload = std::size_t(stride) * height;
        if (slot < 4 || slot > 5 || length < payload) return;
        if (!attempted_) {
            attempted_ = true;
            const char *directory = std::getenv("A081_PASSTHROUGH_EXPORT_DIRECTORY");
            if (!directory || *directory != '/') return;
            struct stat dir{};
            if (lstat(directory, &dir) || !S_ISDIR(dir.st_mode) || dir.st_uid || (dir.st_mode & 0022)) return;
            bytes_ = sizeof(SnapshotHeader) + payload;
            for (unsigned i = 0; i < entries_.size(); ++i) {
                auto &e = entries_[i];
                e.path = std::string(directory) + "/slot" + std::to_string(slot) + "." + std::to_string(i) + ".raw";
                const auto temporary = e.path + "." + std::to_string(getpid());
                e.fd = ::open(temporary.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0640);
                if (e.fd < 0) continue;
                if (ftruncate(e.fd, bytes_) || fchown(e.fd, 0, dir.st_gid) || fchmod(e.fd, 0640)) { unlink(temporary.c_str()); continue; }
                e.data = mmap(nullptr, bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, e.fd, 0);
                if (e.data == MAP_FAILED) { unlink(temporary.c_str()); continue; }
                std::memset(e.data, 0, sizeof(SnapshotHeader));
                struct stat st{};
                if (fstat(e.fd, &st) || rename(temporary.c_str(), e.path.c_str())) { unlink(temporary.c_str()); continue; }
                e.inode = st.st_ino;
            }
        }
        if (bytes_ != sizeof(SnapshotHeader) + payload) return;
        for (unsigned n = 0; n < entries_.size(); ++n) {
            auto &e = entries_[next_++ % entries_.size()];
            if (!e.inode || e.data == MAP_FAILED || flock(e.fd, LOCK_EX | LOCK_NB)) continue;
            std::memcpy(static_cast<std::uint8_t *>(e.data) + sizeof(SnapshotHeader), pixels, payload);
            const SnapshotHeader header{0x31505351u, 1, width, height, stride, V4L2_PIX_FMT_SRGGB10P,
                slot, static_cast<std::uint32_t>(payload), static_cast<std::uint64_t>(getpid()), sequence, stamp, 0};
            std::memcpy(e.data, &header, sizeof(header));
            flock(e.fd, LOCK_UN);
            break;
        }
    }
};
}
