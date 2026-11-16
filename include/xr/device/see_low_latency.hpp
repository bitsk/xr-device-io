// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace xr::see {
struct Suid { std::uint64_t low, high; };
constexpr std::size_t event_bytes = 104;
using Event = std::array<std::uint8_t, event_bytes>;

// Local SOCK_SEQPACKET contract. Event retains the complete Android direct
// record, including its producer counter, original timestamp and bias fields.
// The timestamp offset is measured against BOOTTIME, never first-arrival time.
constexpr std::uint32_t packet_magic = 0x314c5358; // XSL1
struct Packet {
    std::uint32_t magic = packet_magic, kind{}; // 0 accel, 1 gyro
    std::uint64_t arrival_boottime_ns{}, clock_uncertainty_ns{};
    std::uint32_t requested_period_us{}, reserved{};
    Event event{};
};
static_assert(sizeof(Packet) == 136 && offsetof(Packet, event) == 32);

std::uint32_t get32(const Event &, std::size_t);
std::uint64_t get64(const Event &, std::size_t);
bool decode(const Packet &, bool gyro, std::uint64_t &time_ns, float xyz[3]);

// Borrow an already initialized SDSP session and a module opened by its
// listener owner. No second hardware owner, implicit DSP selection or fallback
// to normal QMI is created here. Stop and deinit must precede memory release.
class LowLatencyChannel {
public:
    LowLatencyChannel(int session, std::uint32_t module) : session_(session), module_(module) {}
    ~LowLatencyChannel();
    LowLatencyChannel(const LowLatencyChannel &) = delete;
    LowLatencyChannel &operator=(const LowLatencyChannel &) = delete;
    int initialize();
    int configure(Suid, std::int64_t boot_offset_ns, int sensor_handle);
    int wait(); // synchronous DSP sns_llcm_poll notification; no polling timer
    int drain(std::vector<Event> &);
    int stop();
    int deinitialize();
    std::uint64_t skipped() const { return skipped_; }
private:
    int call(std::uint32_t scalar, void *in, std::size_t in_size,
             void *out = nullptr, std::size_t out_size = 0);
    int sync(std::uint64_t flags);
    int session_, dma_fd_ = -1;
    std::uint32_t module_, channel_{}, last_counter_{};
    void *mapping_ = nullptr;
    bool initialized_{}, retain_{};
    std::atomic<bool> needs_stop_{false};
    std::uint64_t skipped_{};
    // Stock uses 4096 bytes; 39 complete 104-byte records fit in each channel.
    static constexpr std::size_t bytes = 4096;
};
} // namespace xr::see
