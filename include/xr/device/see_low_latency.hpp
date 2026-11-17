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
// The event timestamp remains QTimer nanoseconds. The measured signed offset
// converts it to BOOTTIME in decode(), never from first-arrival time. DSP
// configuration uses zero offset; original VR mode bypasses firmware offset
// addition. This measured tuple supplies the sole AP clock conversion.
// Linux PLL phase: MAXPHASE / 2^SHIFT_PLL = 125 ms/s; frequency and
// adjtime each add at most 0.5 ms/s. The producer gates nominal tick, no PPS
// and no boot ntp_tick_adj. This is a model bound, not measured clock noise.
constexpr std::uint64_t clock_rate_bound_ppb = 126000000;
constexpr std::uint64_t qtimer_tick_ns = 53; // ceil(1e9 / 19200000)
constexpr std::uint32_t packet_magic = 0x344c5358; // XSL4
constexpr std::size_t allocation_bytes = 4096;
constexpr std::size_t native_capacity = allocation_bytes / event_bytes;
// The producer owns the SDSP channel and original DMA allocation. SCM_RIGHTS
// transfers references to those allocations, in accel/gyro order, before Start.
// Stop/Stopped precedes the consumer's CDSP teardown and final fd release.
enum class Command : std::uint32_t { buffers = 1, start = 2, stop = 3, stopped = 4 };
struct Control {
    std::uint32_t magic = packet_magic;
    Command command{};
    std::uint32_t allocation_bytes = xr::see::allocation_bytes;
    std::uint32_t event_bytes = xr::see::event_bytes;
    std::uint32_t channels = 2, period_us = 1250, flags = 2, reserved{};
};
static_assert(sizeof(Control) == 32);
int send_control(int socket, Command, const std::array<int, 2> *fds = nullptr);
// Receive exactly one control record. Fds remain owned by the caller on
// success; every received fd is closed on an invalid record.
int receive_control(int socket, Control &, std::array<int, 2> &fds);
struct Packet {
    std::uint32_t magic = packet_magic, kind{}; // 0 accel, 1 gyro
    std::uint64_t arrival_boottime_ns{}, clock_uncertainty_ns{};
    std::uint32_t requested_period_us{}, reserved{};
    std::int64_t qtimer_to_boottime_ns{};
    std::uint64_t clock_anchor_qtimer_ns{}, clock_anchor_uncertainty_ns{};
    std::uint64_t clock_rate_bound_ppb = xr::see::clock_rate_bound_ppb;
    Event event{};
};
static_assert(sizeof(Packet) == 168 && offsetof(Packet, event) == 64);

std::uint32_t get32(const Event &, std::size_t);
std::uint64_t get64(const Event &, std::size_t);
std::uint64_t mapping_uncertainty(std::uint64_t age_ns, std::uint64_t anchor_uncertainty_ns);
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
    int configure(Suid, int sensor_handle);
    int wait(); // synchronous DSP sns_llcm_poll notification; no polling timer
    int drain(std::vector<Event> &);
    int stop();
    int deinitialize();
    std::uint64_t skipped() const { return skipped_; }
    int dma_fd() const { return dma_fd_; } // borrowed until channel destruction
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
    static constexpr std::size_t bytes = allocation_bytes;
};
} // namespace xr::see
