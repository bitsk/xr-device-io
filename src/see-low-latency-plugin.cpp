// SPDX-License-Identifier: GPL-3.0-or-later
#include <xr/device/see_low_latency.hpp>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <fstream>
#include <string>
#include <poll.h>
#include <pthread.h>
#include <sys/file.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/timex.h>
#include <thread>
#include <time.h>
#include <unistd.h>
extern "C" {
#include <libhexagonrpc/session.h>
#include <libhexagonrpc/interface/remotectl.h>
void a081_apps_mem_counters(std::uint64_t out[4]) __attribute__((weak));
}

namespace {
struct TerminationSignals {
    int fd=-1;
    ~TerminationSignals() { if(fd>=0) close(fd); }
    bool initialize() {
        sigset_t current, watched;
        // Refuse an old daemon instead of leaving its listener interruptible.
        if(pthread_sigmask(SIG_SETMASK,nullptr,&current) ||
           sigismember(&current,SIGTERM)!=1 || sigismember(&current,SIGINT)!=1) return false;
        sigemptyset(&watched);sigaddset(&watched,SIGTERM);sigaddset(&watched,SIGINT);
        fd=signalfd(-1,&watched,SFD_CLOEXEC|SFD_NONBLOCK);
        return fd>=0;
    }
    bool consume() {
        signalfd_siginfo info{};
        if(read(fd,&info,sizeof(info))!=sizeof(info)) return false;
        std::fprintf(stderr,"stage=see_termination signal=%u\n",info.ssi_signo);
        return info.ssi_signo==SIGTERM || info.ssi_signo==SIGINT;
    }
};
std::uint64_t now() {
    timespec t{}; if (clock_gettime(CLOCK_BOOTTIME,&t)) return 0;
    return std::uint64_t(t.tv_sec)*1000000000+t.tv_nsec;
}
struct ClockAnchor {
    std::int64_t offset{};
    std::uint64_t qtimer{}, uncertainty{};
};
bool clock_policy() {
    std::ifstream source("/sys/devices/system/clocksource/clocksource0/current_clocksource");
    std::string name; source>>name;
    std::ifstream command("/proc/cmdline");std::string argument;
    if(name!="arch_sys_counter" || !command) return false;
    while(command>>argument) if(argument.rfind("ntp_tick_adj=",0)==0 && argument!="ntp_tick_adj=0") return false;
    return true;
}
bool clock_anchor(ClockAnchor &anchor) {
#if defined(__aarch64__)
    // Read-only. MAXPHASE, SHIFT_PLL and MAX_TICKADJ bound all PLL time
    // constants, frequency changes and adjtime corrections. Unsupported tick
    // or PPS discipline must not silently invalidate that bound.
    timex discipline{};
    if(adjtimex(&discipline)<0 || discipline.tick!=10000 || sysconf(_SC_CLK_TCK)!=100 ||
       (discipline.status&STA_PPSTIME)) return false;
    std::uint64_t best=UINT64_MAX;
    for (unsigned i=0;i<8;++i) {
        auto before=now(); std::uint64_t ticks,freq;
        asm volatile("isb; mrs %0, cntvct_el0; mrs %1, cntfrq_el0" : "=r"(ticks),"=r"(freq) :: "memory");
        auto after=now();
        if (!before || after<before || freq!=19200000) return false;
        auto span=after-before;
        auto ns=static_cast<std::uint64_t>((static_cast<unsigned __int128>(ticks)*1000000000)/freq);
        if (span<best) {
            const __int128 offset=static_cast<__int128>(before)+span/2-ns;
            if(offset<INT64_MIN || offset>INT64_MAX) return false;
            best=span;anchor.offset=static_cast<std::int64_t>(offset);anchor.qtimer=ns;
        }
    }
    if (best>1000000) return false;
    // Two counter quanta cover counter conversion plus the 19.2 MHz
    // clocksource multiplier rounding over the accepted <=1 s record age.
    anchor.uncertainty=(best+1)/2+2*xr::see::qtimer_tick_ns;return true;
#else
    (void)anchor;return false;
#endif
}
bool suid(const char *name,xr::see::Suid &id) {
    const char *p=getenv(name); if (!p || std::strlen(p)!=33 || p[16]!=':') return false;
    for(unsigned i=0;i<33;++i) if(i!=16 &&
        !((p[i]>='0' && p[i]<='9') || (p[i]>='a' && p[i]<='f') || (p[i]>='A' && p[i]<='F'))) return false;
    char *end=nullptr; errno=0;
    id.low=std::strtoull(p,&end,16); if (errno || end!=p+16) return false;
    id.high=std::strtoull(p+17,&end,16);
    return !errno && !*end && (id.low || id.high);
}
int run() {
    TerminationSignals termination;
    if(!termination.initialize()) {
        std::fprintf(stderr,"stage=see_gate error=synchronous_signal_daemon_required\n");return 2;
    }
    bool interrupted=false;
    int session=hexagonrpc_fd_from_env();
    xr::see::Suid ids[2]{};
    const char *path=getenv("XR_SEE_IMU_SOCKET");
    const char *ack=getenv("XR_SEE_LOW_LATENCY");
    if (session<0 || !a081_apps_mem_counters || !ack || std::strcmp(ack,"1") ||
        !path || std::strncmp(path,"/run/",5) || std::strlen(path)>=sizeof(sockaddr_un::sun_path) ||
        !suid("XR_SEE_ACCEL_SUID",ids[0]) || !suid("XR_SEE_GYRO_SUID",ids[1]) ||
        (ids[0].low==ids[1].low && ids[0].high==ids[1].high)) {
        std::fprintf(stderr,"stage=see_gate error=explicit_session_socket_suids_required\n"); return 2;
    }
    // Same lock as the existing IIO producer. The tracker socket consumer must
    // not open or configure IIO, so only this process owns the selected source.
    int lock=open("/run/a081-iio.lock",O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
    if (lock<0 || flock(lock,LOCK_EX|LOCK_NB)) { if(lock>=0) close(lock); return 2; }
    int server=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0);
    sockaddr_un addr{}; addr.sun_family=AF_UNIX; std::strcpy(addr.sun_path,path);
    // The session can observe the pathname immediately after bind. Create it
    // as 0600 atomically instead of exposing a 0700 window before chmod.
    auto oldmask=umask(0177);
    int rc=server<0 ? -1 : bind(server,reinterpret_cast<sockaddr *>(&addr),sizeof(addr));
    umask(oldmask);
    // Refuse an existing socket; never unlink another owner's endpoint.
    if (rc) { if(server>=0)close(server);close(lock);return 2; }
    if (chmod(path,0600) || listen(server,1)) {
        close(server);unlink(path);close(lock);return 2;
    }
    int peer=-1;
    while (!interrupted) {
        pollfd p[]{{server,POLLIN,0},{termination.fd,POLLIN,0}};rc=poll(p,2,100);
        if (rc<0 && errno!=EINTR) break;
        if(p[1].revents) { interrupted=termination.consume();break; }
        if(p[0].revents) { peer=accept4(server,nullptr,nullptr,SOCK_CLOEXEC|SOCK_NONBLOCK);break; }
    }
    if(peer<0) { close(server);unlink(path);close(lock);return interrupted ? 0 : 1; }
    ucred credentials{}; socklen_t clen=sizeof(credentials);
    if (getsockopt(peer,SOL_SOCKET,SO_PEERCRED,&credentials,&clen) || credentials.uid!=geteuid()) {
        close(peer);close(server);unlink(path);close(lock);return 2;
    }
    ClockAnchor initial;
    // Camera/SDK startup may precede this connection by many seconds. Never
    // calibrate a session before its consumer handshake.
    if(!clock_policy() || !clock_anchor(initial)) {
        std::fprintf(stderr,"stage=see_clock error=unsupported_clock_discipline\n");
        close(peer);close(server);unlink(path);close(lock);return 2;
    }
    std::uint32_t module=0, loader=0; char message[1024]{};
    // The stock stub opens this name and retains the winning module handle
    // for the process lifetime. Do not force a second, file-URI module.
    constexpr char name[]="sns_low_lat_stream";
    rc=remotectl_open(session,sizeof(name),name,&module,sizeof(message),message,&loader);
    std::fprintf(stderr,"stage=see_open boottime_ns=%llu name=%s rc=%d loader=%u message=%s\n",
        (unsigned long long)now(),name,rc,loader,message);
    if(rc || loader) { close(peer);close(server);unlink(path);close(lock);return 1; }
    std::uint64_t before[4]{}; a081_apps_mem_counters(before);
    xr::see::LowLatencyChannel accel(session,module),gyro(session,module);
    xr::see::LowLatencyChannel *channels[]{&accel,&gyro};
    constexpr unsigned period=1250;
    bool ok=true;
    for(unsigned i=0;i<2 && ok;++i) {
        std::fprintf(stderr,"stage=see_init_begin boottime_ns=%llu kind=%u\n",(unsigned long long)now(),i);
        rc=channels[i]->initialize();
        std::fprintf(stderr,"stage=see_init_end boottime_ns=%llu kind=%u rc=%d\n",(unsigned long long)now(),i,rc);
        if (!rc) rc=channels[i]->configure(ids[i],i+1);
        std::fprintf(stderr,"stage=see_config boottime_ns=%llu kind=%u period_us=%u rc=%d dsp_offset_ns=0 qtimer_to_boottime_ns=%lld uncertainty_ns=%llu\n",
                     (unsigned long long)now(),i,period,rc,(long long)initial.offset,(unsigned long long)initial.uncertainty);
        ok=rc==0;
    }
    std::atomic<bool> stopping{!ok}, failed{false}, disconnected{false};
    std::atomic<std::uint64_t> received[2]{};
    std::mutex output;
    std::thread workers[2];
    if(ok) for(unsigned i=0;i<2;++i) workers[i]=std::thread([&,i] {
        std::uint64_t last=0,last_raw=0,max_age=0,max_uncertainty=0;
        ClockAnchor previous=initial;
        bool first=true;
        while(!stopping.load() && !disconnected.load() && !failed.load()) {
            if(first) std::fprintf(stderr,"stage=see_poll_begin boottime_ns=%llu kind=%u\n",(unsigned long long)now(),i);
            int result=channels[i]->wait();
            if(first || result) std::fprintf(stderr,"stage=see_poll_end boottime_ns=%llu kind=%u rc=%d stopping=%u\n",
                (unsigned long long)now(),i,result,unsigned(stopping.load()));
            if(stopping.load()) break;
            std::vector<xr::see::Event> events;
            if(!result) result=channels[i]->drain(events);
            if(first) std::fprintf(stderr,"stage=see_first_drain boottime_ns=%llu kind=%u rc=%d events=%zu\n",
                (unsigned long long)now(),i,result,events.size());
            first=false;
            if(result) { std::fprintf(stderr,"stage=see_read kind=%u rc=%d\n",i,result);failed=true;break; }
            if(events.empty()) continue;
            ClockAnchor anchor;
            if(!clock_anchor(anchor) || anchor.qtimer<=previous.qtimer) {
                std::fprintf(stderr,"stage=see_clock error=anchor_failed kind=%u\n",i);failed=true;break;
            }
            const auto elapsed=anchor.qtimer-previous.qtimer;
            const __int128 drift=static_cast<__int128>(anchor.offset)-previous.offset;
            const auto bound=xr::see::mapping_uncertainty(elapsed,anchor.uncertainty+previous.uncertainty);
            if(drift>bound || drift<-static_cast<__int128>(bound)) {
                std::fprintf(stderr,"stage=see_clock error=discontinuity kind=%u\n",i);failed=true;break;
            }
            previous=anchor;
            for(const auto &event:events) {
                xr::see::Packet packet{};packet.kind=i;packet.arrival_boottime_ns=now();
                packet.clock_anchor_qtimer_ns=anchor.qtimer;
                packet.clock_anchor_uncertainty_ns=anchor.uncertainty;
                packet.qtimer_to_boottime_ns=anchor.offset;
                const auto raw=xr::see::get64(event,16);
                const auto age=anchor.qtimer>raw ? anchor.qtimer-raw : raw-anchor.qtimer;
                packet.clock_uncertainty_ns=xr::see::mapping_uncertainty(age,anchor.uncertainty);
                packet.requested_period_us=period;packet.event=event;
                std::uint64_t stamp=0;float xyz[3];
                if(!xr::see::decode(packet,i==1,stamp,xyz) || stamp<=last || raw<=last_raw) {
                    std::fprintf(stderr,"stage=see_decode_rejected kind=%u version=%u type=%u counter=%u raw_qtimer_ns=%llu arrival_ns=%llu uncertainty_ns=%llu previous_ns=%llu qtimer_to_boottime_ns=%lld anchor_qtimer_ns=%llu anchor_uncertainty_ns=%llu anchor_age_ns=%llu\n",
                        i,xr::see::get32(event,0),xr::see::get32(event,8),xr::see::get32(event,12),
                        (unsigned long long)xr::see::get64(event,16),(unsigned long long)packet.arrival_boottime_ns,
                        (unsigned long long)packet.clock_uncertainty_ns,(unsigned long long)last,(long long)anchor.offset,
                        (unsigned long long)anchor.qtimer,(unsigned long long)anchor.uncertainty,(unsigned long long)age);
                    failed=true;break;
                }
                last=stamp;last_raw=raw;
                max_age=std::max(max_age,age);max_uncertainty=std::max(max_uncertainty,packet.clock_uncertainty_ns);
                std::lock_guard<std::mutex> guard(output);
                // A slow consumer fails explicitly; it never stalls sensor
                // acquisition behind an unbounded queue or fabricates samples.
                if(send(peer,&packet,sizeof(packet),MSG_DONTWAIT|MSG_NOSIGNAL)!=sizeof(packet)) {
                    if(errno==EPIPE || errno==ECONNRESET) disconnected=true; else failed=true;
                    break;
                }
                received[i]=packet.arrival_boottime_ns;
            }
        }
        std::fprintf(stderr,"stage=see_clock_summary kind=%u max_anchor_age_ns=%llu max_model_uncertainty_ns=%llu rate_bound_ppb=%llu\n",
            i,(unsigned long long)max_age,(unsigned long long)max_uncertainty,(unsigned long long)xr::see::clock_rate_bound_ppb);
    });
    auto start=now();
    while(ok && !interrupted && !failed.load() && !disconnected.load()) {
        pollfd p[]{{peer,POLLIN,0},{termination.fd,POLLIN,0}};rc=poll(p,2,100);
        if(rc<0 && errno!=EINTR) { ok=false;break; }
        if(p[1].revents) { interrupted=termination.consume();ok=interrupted;break; }
        if(p[0].revents) {
            char byte;
            auto n=recv(peer,&byte,1,MSG_DONTWAIT|MSG_PEEK);
            if(n==0 || (n<0 && errno==ECONNRESET)) disconnected=true; else ok=false;
            break;
        }
        auto current=now();
        for(unsigned i=0;i<2;++i) {
            auto last=received[i].load();
            if(current>(last?last:start)+1000000000) {
                std::fprintf(stderr,"stage=see_timeout boottime_ns=%llu kind=%u last_ns=%llu\n",
                    (unsigned long long)current,i,(unsigned long long)last);
                ok=false;break;
            }
        }
    }
    stopping=true;
    bool stopped=true;
    for(unsigned i=0;i<2;++i) {
        std::fprintf(stderr,"stage=see_stop_begin boottime_ns=%llu kind=%u\n",(unsigned long long)now(),i);
        rc=channels[i]->stop();stopped=rc==0 && stopped;
        std::fprintf(stderr,"stage=see_stop_end boottime_ns=%llu kind=%u rc=%d\n",(unsigned long long)now(),i,rc);
    }
    if(!stopped) {
        std::fprintf(stderr,"stage=see_cleanup error=stop_failed_retain_until_session_exit\n");
        // The plugin is private to this listener process. Exit closes its
        // session without destroying mappings underneath an outstanding RPC.
        std::_Exit(1);
    }
    for(auto &worker:workers) if(worker.joinable()) worker.join();
    std::fprintf(stderr,"stage=see_workers_joined boottime_ns=%llu\n",(unsigned long long)now());
    for(unsigned i=0;i<2;++i) {
        std::fprintf(stderr,"stage=see_deinit_begin boottime_ns=%llu kind=%u\n",(unsigned long long)now(),i);
        rc=channels[i]->deinitialize();ok=rc==0 && ok;
        std::fprintf(stderr,"stage=see_deinit_end boottime_ns=%llu kind=%u rc=%d\n",(unsigned long long)now(),i,rc);
    }
    std::uint64_t after[4]{};a081_apps_mem_counters(after);
    bool balanced=after[2]==before[2] && after[3]==before[3] &&
        after[0]-before[0]==after[1]-before[1];
    std::fprintf(stderr,"stage=see_cleanup maps=%llu unmaps=%llu live=%llu errors=%llu gyro_skipped=%llu accel_skipped=%llu\n",
        (unsigned long long)(after[0]-before[0]),(unsigned long long)(after[1]-before[1]),
        (unsigned long long)after[2],(unsigned long long)(after[3]-before[3]),
        (unsigned long long)gyro.skipped(),(unsigned long long)accel.skipped());
    if(!balanced) std::_Exit(1);
    // init registers a DSP process-local-storage destructor in this module.
    // deinit stops streaming and unmaps memory but does not unregister that
    // destructor. Keep its code loaded until FastRPC process teardown, as
    // the stock stub does, instead of unloading it before PLS cleanup.
    std::fprintf(stderr,"stage=see_session_release boottime_ns=%llu module=0x%08x ok=%u failed=%u disconnected=%u\n",
        (unsigned long long)now(),module,unsigned(ok),unsigned(failed.load()),unsigned(disconnected.load()));
    close(peer);close(server);unlink(path);close(lock);
    return ok && !failed.load() ? 0 : 1;
}
}
extern "C" int a081_qvr_client_synchronous_signals() { return 1; }
extern "C" int a081_qvr_client_main() { return run(); }
