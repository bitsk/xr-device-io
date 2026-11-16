// SPDX-License-Identifier: GPL-3.0-or-later
#include <xr/device/see_low_latency.hpp>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <linux/dma-buf.h>
#include <misc/fastrpc.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace xr::see {
namespace {
void put32(void *p, std::size_t off, std::uint32_t v) {
    auto *b = static_cast<std::uint8_t *>(p) + off;
    for (unsigned i=0; i<4; ++i) b[i] = v >> (8*i);
}
void put64(void *p, std::size_t off, std::uint64_t v) {
    put32(p, off, v); put32(p, off+4, v >> 32);
}
std::uint32_t distance(std::uint32_t old, std::uint32_t next) {
    // Android counters wrap UINT32_MAX -> 1, skipping the empty marker 0.
    return next >= old ? next-old : UINT32_MAX-old+next;
}
}
std::uint32_t get32(const Event &e, std::size_t n) {
    return std::uint32_t(e[n]) | std::uint32_t(e[n+1])<<8 |
           std::uint32_t(e[n+2])<<16 | std::uint32_t(e[n+3])<<24;
}
std::uint64_t get64(const Event &e, std::size_t n) {
    return get32(e,n) | std::uint64_t(get32(e,n+4))<<32;
}
bool decode(const Packet &p, bool gyro, std::uint64_t &time, float xyz[3]) {
    const auto &e = p.event;
    // Flags 2 request Android uncalibrated samples. Do not silently accept a
    // calibrated source or subtract the bias fields a second time.
    if (p.magic != packet_magic || p.kind != unsigned(gyro) || get32(e,0) != event_bytes ||
        get32(e,8) != (gyro ? 16u : 35u) || !get32(e,12) ||
        !p.arrival_boottime_ns || p.clock_uncertainty_ns > 1000000 || p.reserved ||
        p.requested_period_us!=1250) return false;
    const auto raw_stamp = get64(e,16);
    const __int128 converted = static_cast<__int128>(raw_stamp) + p.qtimer_to_boottime_ns;
    if (!raw_stamp || converted<=0 || converted>UINT64_MAX) return false;
    const auto stamp = static_cast<std::uint64_t>(converted);
    if ((stamp > p.arrival_boottime_ns && stamp-p.arrival_boottime_ns > p.clock_uncertainty_ns) ||
        (stamp < p.arrival_boottime_ns && p.arrival_boottime_ns-stamp > 1000000000)) return false;
    float values[3];
    for (unsigned i=0; i<3; ++i) {
        auto word = get32(e,24+4*i); std::memcpy(values+i,&word,4);
        if (!std::isfinite(values[i])) return false;
    }
    time = stamp; std::copy(values,values+3,xyz); return true;
}
int LowLatencyChannel::call(std::uint32_t sc, void *in, std::size_t ni, void *out, std::size_t no) {
    fastrpc_invoke_args args[2]{};
    args[0].ptr = reinterpret_cast<std::uintptr_t>(in); args[0].length = ni; args[0].fd = -1;
    args[1].ptr = reinterpret_cast<std::uintptr_t>(out); args[1].length = no; args[1].fd = -1;
    fastrpc_invoke req{}; req.handle=module_; req.sc=sc;
    req.args=reinterpret_cast<std::uintptr_t>(args);
    int rc = ioctl(session_, FASTRPC_IOCTL_INVOKE, &req);
    return rc == -1 ? -errno : rc;
}
int LowLatencyChannel::sync(std::uint64_t flags) {
    dma_buf_sync req{flags}; int rc;
    do { rc=ioctl(dma_fd_,DMA_BUF_IOCTL_SYNC,&req); } while (rc<0 && errno==EINTR);
    return rc<0 ? -errno : rc;
}
int LowLatencyChannel::initialize() {
    if (mapping_ || initialized_) return -EALREADY;
    fastrpc_alloc_dma_buf req{}; req.fd=-1; req.size=bytes;
    if (ioctl(session_,FASTRPC_IOCTL_ALLOC_DMA_BUFF,&req)) return -errno;
    dma_fd_=req.fd;
    auto *p=mmap(nullptr,bytes,PROT_READ|PROT_WRITE,MAP_SHARED,dma_fd_,0);
    if (p==MAP_FAILED) return -errno;
    mapping_=p;
    int rc=sync(DMA_BUF_SYNC_START|DMA_BUF_SYNC_WRITE);
    if (rc) return rc;
    std::memset(mapping_,0,bytes);
    rc=sync(DMA_BUF_SYNC_END|DMA_BUF_SYNC_WRITE);
    if (rc) return rc;
    std::uint8_t input[12]{}, output[4]{};
    put32(input,0,dma_fd_); put32(input,4,bytes); // stock third word is 0
    // A failed init can still have mapped memory; retain until session exit.
    retain_=true;
    rc=call(0x00010100,input,sizeof(input),output,sizeof(output));
    if (rc) return rc;
    std::memcpy(&channel_,output,4);
    if (!channel_) return -EPROTO;
    initialized_=true; return 0;
}
int LowLatencyChannel::configure(Suid id, int handle) {
    if (!initialized_ || needs_stop_) return -EINVAL;
    // Stock QVR requests VERY_FAST: the verified HAL table selects 1250us.
    // The configuration default and advertised extra rates do not override it.
    if ((!id.low && !id.high) || handle<=0) return -EINVAL;
    std::uint8_t input[48]{};
    // Keep the shared record in QTimer nanoseconds. The original HAL passes
    // an elapsedRealtime offset here, but this firmware was observed to leave
    // the record unconverted. Zero prevents a second, DSP-side conversion;
    // Packet carries our measured offset for the single AP-side conversion.
    put32(input,0,channel_); put64(input,8,0);
    put64(input,16,id.low); put64(input,24,id.high);
    put32(input,32,1250); put32(input,36,2); put32(input,40,handle);
    needs_stop_=true;
    return call(0x01010000,input,sizeof(input));
}
int LowLatencyChannel::wait() {
    if (!needs_stop_) return -EINVAL;
    std::uint8_t input[4]{}; put32(input,0,channel_);
    return call(0x04010000,input,sizeof(input));
}
int LowLatencyChannel::drain(std::vector<Event> &out) {
    if (!initialized_) return -EINVAL;
    int rc=sync(DMA_BUF_SYNC_START|DMA_BUF_SYNC_READ);
    if (rc) return rc;
    std::vector<Event> pending;
    auto *base=static_cast<const std::uint8_t *>(mapping_);
    for (std::size_t i=0; i<bytes/event_bytes; ++i) {
        const auto *src=base+i*event_bytes;
        auto *counter=reinterpret_cast<const std::uint32_t *>(src+12);
        auto before=__atomic_load_n(counter,__ATOMIC_ACQUIRE);
        if (!before) continue;
        Event e{}, second{};
        const volatile std::uint8_t *v=src;
        for (std::size_t k=0;k<e.size();++k) e[k]=v[k];
        std::atomic_thread_fence(std::memory_order_acquire);
        for (std::size_t k=0;k<second.size();++k) second[k]=v[k];
        if (e!=second) continue;
        if (before!=__atomic_load_n(counter,__ATOMIC_ACQUIRE) || before!=get32(e,12)) continue;
        auto delta=distance(last_counter_,before);
        if (delta && delta<0x80000000u) pending.push_back(e);
    }
    rc=sync(DMA_BUF_SYNC_END|DMA_BUF_SYNC_READ);
    if (rc) return rc;
    std::sort(pending.begin(),pending.end(),[this](const Event &a,const Event &b) {
        return distance(last_counter_,get32(a,12)) < distance(last_counter_,get32(b,12));
    });
    for (const auto &e:pending) {
        auto counter=get32(e,12), delta=distance(last_counter_,counter);
        if (last_counter_ && delta>1) skipped_+=delta-1;
        last_counter_=counter; out.push_back(e);
    }
    return 0;
}
int LowLatencyChannel::stop() {
    if (!needs_stop_) return 0;
    std::uint8_t input[4]{}; put32(input,0,channel_);
    int rc=call(0x02010000,input,sizeof(input));
    if (!rc) needs_stop_=false;
    return rc;
}
int LowLatencyChannel::deinitialize() {
    if (!initialized_) return 0;
    if (needs_stop_) return -EBUSY;
    std::uint8_t input[4]{}; put32(input,0,channel_);
    int rc=call(0x03010000,input,sizeof(input));
    if (!rc) { initialized_=false; retain_=false; }
    return rc;
}
LowLatencyChannel::~LowLatencyChannel() {
    // Failed cleanup is resolved by the private session owner closing its
    // FastRPC process, never by freeing memory still writable by the DSP.
    if (retain_) return;
    if (mapping_) munmap(mapping_,bytes);
    if (dma_fd_>=0) close(dma_fd_);
}
} // namespace xr::see
