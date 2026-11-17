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
#include <sys/socket.h>
#include <unistd.h>

namespace xr::see {
int send_control(int socket, Command command, const std::array<int, 2> *fds) {
    Control control{}; control.command=command;
    iovec io{&control,sizeof(control)};
    alignas(cmsghdr) char ancillary[CMSG_SPACE(2*sizeof(int))]{};
    msghdr message{};message.msg_iov=&io;message.msg_iovlen=1;
    if(fds) {
        if(command!=Command::buffers || (*fds)[0]<0 || (*fds)[1]<0) return -EINVAL;
        message.msg_control=ancillary;message.msg_controllen=sizeof(ancillary);
        auto *header=CMSG_FIRSTHDR(&message);header->cmsg_level=SOL_SOCKET;
        header->cmsg_type=SCM_RIGHTS;header->cmsg_len=CMSG_LEN(2*sizeof(int));
        std::memcpy(CMSG_DATA(header),fds->data(),2*sizeof(int));
    } else if(command==Command::buffers) return -EINVAL;
    const auto n=sendmsg(socket,&message,MSG_NOSIGNAL);
    return n==sizeof(control) ? 0 : (n<0 ? -errno : -EPROTO);
}
int receive_control(int socket, Control &out, std::array<int, 2> &fds) {
    Control control{};iovec io{&control,sizeof(control)};
    alignas(cmsghdr) char ancillary[CMSG_SPACE(2*sizeof(int))]{};
    msghdr message{};message.msg_iov=&io;message.msg_iovlen=1;
    message.msg_control=ancillary;message.msg_controllen=sizeof(ancillary);
    const auto n=recvmsg(socket,&message,MSG_CMSG_CLOEXEC|MSG_TRUNC);
    if(n<0) return -errno;
    std::array<int,2> received{{-1,-1}};unsigned count=0;bool invalid=false;
    for(auto *h=CMSG_FIRSTHDR(&message);h;h=CMSG_NXTHDR(&message,h)) {
        if(h->cmsg_level!=SOL_SOCKET || h->cmsg_type!=SCM_RIGHTS || h->cmsg_len<CMSG_LEN(0)) {
            invalid=true;continue;
        }
        const auto bytes=h->cmsg_len-CMSG_LEN(0);
        if(bytes%sizeof(int)) invalid=true;
        for(std::size_t at=0;at+sizeof(int)<=bytes;at+=sizeof(int)) {
            int fd=-1;std::memcpy(&fd,static_cast<char *>(static_cast<void *>(CMSG_DATA(h)))+at,sizeof(fd));
            if(count<received.size()) received[count++]=fd;
            else { ::close(fd);invalid=true; }
        }
    }
    invalid=invalid || n!=sizeof(control) || (message.msg_flags&(MSG_CTRUNC|MSG_TRUNC)) ||
        control.magic!=packet_magic || control.allocation_bytes!=allocation_bytes ||
        control.event_bytes!=event_bytes || control.channels!=2 || control.period_us!=1250 ||
        control.flags!=2 || control.reserved ||
        static_cast<unsigned>(control.command)<1 || static_cast<unsigned>(control.command)>4 ||
        count!=(control.command==Command::buffers ? 2u : 0u);
    if(invalid) { for(int fd:received) if(fd>=0) ::close(fd);return -EPROTO; }
    out=control;fds=received;return 0;
}
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
std::uint64_t mapping_uncertainty(std::uint64_t age, std::uint64_t anchor_uncertainty) {
    const auto value=static_cast<unsigned __int128>(anchor_uncertainty)+
        (static_cast<unsigned __int128>(age)*clock_rate_bound_ppb+999999999)/1000000000;
    // Sentinel means the bound cannot be represented in the wire tuple.
    return value>=UINT64_MAX ? UINT64_MAX : static_cast<std::uint64_t>(value);
}
bool decode(const Packet &p, bool gyro, std::uint64_t &time, float xyz[3]) {
    const auto &e = p.event;
    // Flag bit1 selects Android104 layout. Types16/35 select uncalibrated
    // variants; bit2 is the independent is_calibrated flag. No bias subtraction.
    if (p.magic != packet_magic || p.kind != unsigned(gyro) || get32(e,0) != event_bytes ||
        get32(e,8) != (gyro ? 16u : 35u) || !get32(e,12) ||
        !p.arrival_boottime_ns || !p.clock_anchor_qtimer_ns || p.reserved ||
        p.clock_anchor_uncertainty_ns < 2*qtimer_tick_ns ||
        p.clock_rate_bound_ppb != clock_rate_bound_ppb ||
        p.requested_period_us!=1250) return false;
    const auto raw_stamp = get64(e,16);
    // A stable record was copied before this batch's fresh counter anchor.
    // Reject future raw events using counter causality, independently of the
    // deliberately conservative historical conversion uncertainty.
    if (raw_stamp > p.clock_anchor_qtimer_ns &&
        raw_stamp-p.clock_anchor_qtimer_ns > qtimer_tick_ns) return false;
    const auto age = raw_stamp>p.clock_anchor_qtimer_ns ? raw_stamp-p.clock_anchor_qtimer_ns :
                     p.clock_anchor_qtimer_ns-raw_stamp;
    if(p.clock_uncertainty_ns==UINT64_MAX ||
        p.clock_uncertainty_ns!=mapping_uncertainty(age,p.clock_anchor_uncertainty_ns))
        return false;
    const __int128 anchor_boot = static_cast<__int128>(p.clock_anchor_qtimer_ns)+p.qtimer_to_boottime_ns;
    if (anchor_boot<=0 || anchor_boot>p.arrival_boottime_ns) return false;
    const __int128 converted = static_cast<__int128>(raw_stamp) + p.qtimer_to_boottime_ns;
    if (!raw_stamp || converted<=0 || converted>UINT64_MAX) return false;
    const auto stamp = static_cast<std::uint64_t>(converted);
    if(stamp>p.arrival_boottime_ns && stamp-p.arrival_boottime_ns>qtimer_tick_ns) return false;
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
