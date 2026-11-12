// SPDX-License-Identifier: GPL-2.0-or-later
// Single real Nordic transport owner: forward raw packets to the SDK helper,
// optionally return timestamps of actual camera frames. Never pairs or flashes.
#include <xr/device/a081_controller_input_wire.h>
#include <xr/device/a081_radio_clock_reference.h>
#define _Static_assert static_assert
#include <xr/device/a081_controller_wire.h>
#undef _Static_assert
#include <xr/device/a081_camera_timing.hpp>
#include <linux/a081-nordic.h>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace {
volatile sig_atomic_t stopped;
void stop(int) { stopped = 1; }
struct Fd { int fd{-1}; ~Fd() { if (fd >= 0) close(fd); } };
std::uint64_t now(clockid_t id)
{
    timespec t{}; if (clock_gettime(id, &t)) throw std::runtime_error("clock_gettime");
    return std::uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
std::string read_file(const char *path)
{
    FILE *f = fopen(path, "r"); if (!f) throw std::runtime_error(path);
    char b[128]{}; const auto n = fread(b, 1, sizeof(b)-1, f); fclose(f);
    return std::string(b, n);
}
struct ClockObservation {
    std::uint64_t boot_before, monotonic, boot_after;
};
void log_exchange(const a081_nordic_dual_transfer &transfer, const ClockObservation &clock)
{
    std::printf(",\"irq_sequence\":%llu,\"irq_ns\":%llu,\"completion_ns\":%llu,\"boot_before_ns\":%llu,\"monotonic_ns\":%llu,\"boot_after_ns\":%llu,\"tx\":\"",
        static_cast<unsigned long long>(transfer.irq_sequence),
        static_cast<unsigned long long>(transfer.irq_timestamp_ns),
        static_cast<unsigned long long>(transfer.completion_timestamp_ns),
        static_cast<unsigned long long>(clock.boot_before),
        static_cast<unsigned long long>(clock.monotonic),
        static_cast<unsigned long long>(clock.boot_after));
    for (auto byte : transfer.tx) std::printf("%02x", byte);
    std::printf("\",\"rx\":\"");
    for (auto byte : transfer.rx) std::printf("%02x", byte);
    std::puts("\",\"spi_bytes\":72}");
}
struct TimestampFollowup {
    unsigned hand{}, remaining{};
    std::uint64_t attempt{}, request_sequence{}, irq_sequence{};
};
void set_radio_scheduling()
{
    // Stock SerialPortLoop requests nice -11 and FIFO44 through the JNI
    // scheduling callback. Android's foreground cpuset restricts it to4-7;
    // its earlier CPU3 affinity request cannot escape that cpuset. Linux
    // has no corresponding Android policy service or foreground cpuset.
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    for (int cpu = 4; cpu <= 7; ++cpu) CPU_SET(cpu, &cpus);
    sched_param requested{};
    requested.sched_priority = 44;
    if (setpriority(PRIO_PROCESS, 0, -11) ||
        sched_setaffinity(0, sizeof(cpus), &cpus) ||
        sched_setscheduler(0, SCHED_FIFO, &requested))
        throw std::runtime_error(std::string("Cannot apply stock radio scheduling: ") + std::strerror(errno));
    sched_param actual{};
    if (sched_getaffinity(0, sizeof(cpus), &cpus) || sched_getparam(0, &actual) ||
        sched_getscheduler(0) != SCHED_FIFO || actual.sched_priority != 44)
        throw std::runtime_error("Stock radio scheduling readback failed");
    unsigned mask = 0;
    for (int cpu = 0; cpu < 8; ++cpu)
        if (CPU_ISSET(cpu, &cpus)) mask |= 1U << cpu;
    std::printf("{\"kind\":\"radio_scheduling\",\"pid\":%ld,\"policy\":\"SCHED_FIFO\",\"priority\":44,\"cpu_mask\":%u}\n",
                static_cast<long>(getpid()), mask);
}
}
int main(int argc, char **argv) try {
    if (argc == 2 && std::string(argv[1]) == "--print-abi") {
        std::puts(A081_CONTROLLER_INPUT_ABI_DESCRIPTOR); return 0;
    }
    std::string socket_path; unsigned seconds = 30; bool camera_time = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--socket" && i+1 < argc) socket_path = argv[++i];
        else if (arg == "--seconds" && i+1 < argc) {
            char *end = nullptr; errno = 0; auto n = std::strtoul(argv[++i], &end, 10);
            if (errno || !*argv[i] || *end || n > 3600) throw std::runtime_error("seconds must be0(continuous)..3600");
            seconds = n;
        } else if (arg == "--camera-time") camera_time = true;
        else throw std::runtime_error("Usage: controller-radio --socket PATH [--seconds0..3600] [--camera-time]");
    }
    if (geteuid() != 0 || read_file("/proc/device-tree/model").find("A081") == std::string::npos ||
        socket_path.empty() || socket_path.front() != '/' || socket_path.size() >= sizeof(sockaddr_un::sun_path))
        throw std::runtime_error("Expected root A081 and absolute socket");
    const auto root = read_file("/etc/a081-rootfs-id");
    if (root != "a081-steamos-arm64\n" && root != "a081-debian-trixie-arm64\n" && root != "a081-openkylin-3.0-arm64\n") throw std::runtime_error("Unexpected rootfs");
    Fd lock; lock.fd = open("/run/mix-xr-stock-controller.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (lock.fd < 0 || flock(lock.fd, LOCK_EX | LOCK_NB)) throw std::runtime_error("Controller lock busy");
    struct stat st{};
    if (lstat(socket_path.c_str(), &st) || !S_ISSOCK(st.st_mode) || st.st_uid != 0 || (st.st_mode & 0077))
        throw std::runtime_error("Input socket must be root-owned mode0600");
    Fd sock; sock.fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    sockaddr_un addr{}; addr.sun_family = AF_UNIX; std::strcpy(addr.sun_path, socket_path.c_str());
    ucred cred{}; socklen_t cred_size = sizeof(cred);
    if (sock.fd < 0 || connect(sock.fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) ||
        getsockopt(sock.fd, SOL_SOCKET, SO_PEERCRED, &cred, &cred_size) || cred_size != sizeof(cred) || cred.uid != 0)
        throw std::runtime_error("Cannot connect to root SDK helper");
    Fd dev; dev.fd = open("/dev/a081-nordic", O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (dev.fd < 0) throw std::runtime_error("Cannot open Nordic");
    static_assert(sizeof(a081_nordic_dual_transfer) == 168);
    setvbuf(stdout, nullptr, _IOLBF, 0); signal(SIGINT, stop); signal(SIGTERM, stop);
    set_radio_scheduling();
    std::uint64_t sequence=0, requests=0, transmitted=0, discarded=0, version_time[2]{}, last_sof[2]{}, type7[2]{}, fresh[2]{};
    std::uint64_t packet_types[2][8]{}, invalid_headers=0, invalid_checksums=0, unmatched_types=0;
    std::uint8_t version[2]{};
    std::array<a081_controller_timestamp_request, 2> pending{};
    std::array<bool, 2> have{};
    std::uint64_t next_log=0;
    std::uint64_t tx_attempts[2]{}, next_tx_log[2]{};
    // Separate slots retain each first-eight TX across later transmissions.
    std::array<TimestampFollowup, 16> followups{};
    const auto deadline = now(CLOCK_MONOTONIC) + std::uint64_t(seconds) * 1000000000ULL;
    std::puts(camera_time ? "{\"kind\":\"radio_scope\",\"timestamp_tx\":true,\"tail_policy\":\"EE_padding_adapter_choice_not_stock_stack_copy\",\"pairing_modified\":false,\"firmware_written\":false}" : "{\"kind\":\"radio_scope\",\"timestamp_tx\":false}");
    while (!stopped && (!seconds || now(CLOCK_MONOTONIC) < deadline)) {
        pollfd p[2]{{dev.fd,POLLIN,0},{sock.fd,POLLIN,0}};
        int r = poll(p,2,50); if (r < 0 && errno == EINTR) continue;
        if (r < 0) throw std::runtime_error("poll");
        for (const auto &x : p) if (x.revents & (POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("Radio/helper disconnected");
        if (p[1].revents & POLLIN) {
            for (unsigned n=0;n<8;++n) {
                a081_controller_timestamp_request request{};
                const auto received = recv(sock.fd,&request,sizeof(request),MSG_DONTWAIT|MSG_TRUNC);
                if (received<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) break;
                if (received<=0) throw std::runtime_error("SDK helper closed");
                ++requests;
                const auto mono=now(CLOCK_MONOTONIC), boot=now(CLOCK_BOOTTIME);
                const auto &h=request.header;
                a081::stock::camera::StockY8Time stock_time{};
                const bool exposure_valid=request.ctrl_exposure_ns<=1000000000ULL &&
                    a081::stock::camera::stock_a081_y8_time(request.ctrl_sof_boottime_ns,request.ctrl_exposure_ns,stock_time) &&
                    stock_time.exposure_start_boottime_ns==request.ctrl_exposure_start_boottime_ns;
                if (!camera_time || !exposure_valid || received!=sizeof(request) || h.magic!=A081_CONTROLLER_INPUT_MAGIC ||
                    h.version!=A081_CONTROLLER_INPUT_VERSION || h.bytes!=sizeof(request) || h.type!=A081_CONTROLLER_OUTPUT_TIMESTAMP_REQUEST ||
                    request.hand>1 || request.exposure_phase>1 || request.reserved || !request.phase_anchor_sof_qtimer ||
                    a081_radio_hand_for_exposure_phase(request.exposure_phase)!=int(request.hand) ||
                    !h.sent_monotonic_ns || h.sent_monotonic_ns>mono || mono-h.sent_monotonic_ns>=100000000ULL ||
                    !request.ctrl_exposure_start_boottime_ns || request.ctrl_exposure_start_boottime_ns>=boot || boot-request.ctrl_exposure_start_boottime_ns>=100000000ULL ||
                    request.ctrl_sof_boottime_ns<=last_sof[request.hand]) {++discarded;continue;}
                if(have[request.hand]) ++discarded;
                pending[request.hand]=request;have[request.hand]=true;
            }
        }
        if (!(p[0].revents&POLLIN)) continue;
        a081_nordic_dual_transfer transfer{};std::memset(transfer.tx,0xee,sizeof(transfer.tx));
        int selected=-1;
        for(unsigned h=0;h<2;++h) if(have[h] && (selected<0 || pending[h].ctrl_sof_boottime_ns<pending[selected].ctrl_sof_boottime_ns)) selected=h;
        if(selected>=0) {
            const auto boot=now(CLOCK_BOOTTIME);
            a081_camera_time_body body{};const auto &q=pending[selected];
            // Android keeps PatternID/version until replaced or the source restarts.
            // Type2 is sparse; only this actual camera SOF must be fresh.
            if(!version_time[selected] || boot<=q.ctrl_exposure_start_boottime_ns || boot-q.ctrl_exposure_start_boottime_ns>=100000000ULL ||
               !a081_radio_pack_camera_time_body(q.exposure_phase,q.ctrl_exposure_start_boottime_ns,static_cast<a081_controller_hand>(selected),version[selected],&body)) {have[selected]=false;selected=-1;++discarded;}
            else std::memcpy(transfer.tx,body.bytes,sizeof(body.bytes));
        }
        const int result=ioctl(dev.fd,A081_NORDIC_IOC_DUAL_TRANSFER,&transfer);
        if(result<0) {
            if(errno==EAGAIN||errno==EINTR)continue;
            if(errno==ENOTTY)throw std::runtime_error("Kernel lacks required atomic72-byte dual-controller ioctl; install the matching Nordic module");
            throw std::runtime_error("Nordic72 transfer");
        }
        const ClockObservation clock{now(CLOCK_BOOTTIME), now(CLOCK_MONOTONIC), now(CLOCK_BOOTTIME)};
        for (auto &watch : followups) if (watch.remaining) {
            std::printf("{\"kind\":\"radio_timestamp_followup\",\"first_rx_sequence\":%llu,\"origin_hand\":%u,\"origin_attempt\":%llu,\"origin_request_sequence\":%llu,\"origin_irq_sequence\":%llu,\"successor_exchange\":%u,\"timestamp_tx_hand\":%d",
                static_cast<unsigned long long>(sequence+1), watch.hand,
                static_cast<unsigned long long>(watch.attempt),
                static_cast<unsigned long long>(watch.request_sequence),
                static_cast<unsigned long long>(watch.irq_sequence), 9-watch.remaining, selected);
            log_exchange(transfer, clock);
            --watch.remaining;
        }
        // Stock spiRecv retains its queued command when the first RX word is
        // FFFF. Retry on the next IRQ, subject to the exposure-start freshness gate.
        if(selected>=0 && (transfer.rx[0]!=0xff || transfer.rx[1]!=0xff)) {
            last_sof[selected]=pending[selected].ctrl_sof_boottime_ns;
            have[selected]=false;
            ++transmitted;
        }
        // RX's independent 1Hz sample can miss every timestamp command.
        // Bounded trials retain all actual TX and eight exchanges after each
        // hand's first eight attempts. Continuous runs keep sparse TX only.
        if(selected>=0) {
            const auto log_time=now(CLOCK_MONOTONIC);
            ++tx_attempts[selected];
            if(seconds || tx_attempts[selected]<=4 || log_time>=next_tx_log[selected]) {
                next_tx_log[selected]=log_time+1000000000ULL;
                const auto &q=pending[selected];
                std::printf("{\"kind\":\"radio_timestamp_tx\",\"first_rx_sequence\":%llu,\"hand\":%d,\"attempt\":%llu,\"version\":%u,\"request_sequence\":%llu,\"request_sent_monotonic_ns\":%llu,\"ctrl_frame_id\":%u,\"exposure_phase\":%u,\"phase_anchor_sof_qtimer\":%llu,\"ctrl_sof_boottime_ns\":%llu,\"ctrl_exposure_start_boottime_ns\":%llu,\"ctrl_exposure_ns\":%llu,\"retry_pending\":%s",
                    static_cast<unsigned long long>(sequence+1),selected,static_cast<unsigned long long>(tx_attempts[selected]),unsigned(version[selected]),
                    static_cast<unsigned long long>(q.header.sequence),static_cast<unsigned long long>(q.header.sent_monotonic_ns),q.ctrl_frame_id,q.exposure_phase,
                    static_cast<unsigned long long>(q.phase_anchor_sof_qtimer),static_cast<unsigned long long>(q.ctrl_sof_boottime_ns),
                    static_cast<unsigned long long>(q.ctrl_exposure_start_boottime_ns),static_cast<unsigned long long>(q.ctrl_exposure_ns),have[selected]?"true":"false");
                log_exchange(transfer, clock);
            }
            if (seconds && tx_attempts[selected]<=8) {
                auto &watch=followups[unsigned(selected)*8+tx_attempts[selected]-1];
                watch={unsigned(selected),8,tx_attempts[selected],pending[selected].header.sequence,transfer.irq_sequence};
            }
        }
        for (unsigned offset=0;offset<72u;offset+=36) {
        const auto *raw=transfer.rx+offset;
        a081_controller_wire_packet decoded{};
        const auto decode_result=a081_controller_decode_wire(raw,&decoded);
        if(decode_result==A081_WIRE_OK) {
            const auto *data=decoded.payload;unsigned hand=decoded.hand,type=decoded.type;
            ++packet_types[hand][type];
            if((type==4 || type==5) && packet_types[hand][type]==1) {
                std::printf("{\"kind\":\"radio_first_sync_packet\",\"sequence\":%llu,\"hand\":%u,\"wire_type\":%u,\"rx_offset\":%u,\"timestamp_tx_hand\":%d,\"checksum_policy\":\"%s\"",
                    static_cast<unsigned long long>(sequence+1),hand,type,offset,selected,
                    type==4?"none_stock_type4":"p29_sum_p0_28");
                log_exchange(transfer,clock);
            }
            if(type==2){
                if(!version_time[hand] || version[hand]!=data[22]) {
                    std::printf("{\"kind\":\"radio_pattern_version\",\"sequence\":%llu,\"hand\":%u,\"version\":%u,\"previous_version\":%u,\"first_version\":%s,\"rx_offset\":%u",
                        static_cast<unsigned long long>(sequence+1),hand,unsigned(data[22]),unsigned(version[hand]),version_time[hand]?"false":"true",offset);
                    log_exchange(transfer,clock);
                }
                version[hand]=data[22];version_time[hand]=now(CLOCK_MONOTONIC);
            }
            if(type==7){++type7[hand];std::uint64_t clock=0;for(unsigned i=0;i<4;++i)clock|=std::uint64_t(data[3+i])<<(8*i);clock|=std::uint64_t(data[29])<<32|std::uint64_t(data[30])<<40;std::uint64_t ns=0;
                if(version_time[hand] && a081_radio_scale_type7_time(clock,version[hand],&ns) && a081_radio_stock_time_window(ns,now(CLOCK_BOOTTIME),nullptr))++fresh[hand];}
        }
        else if(decode_result==A081_WIRE_HEADER) ++invalid_headers;
        else if(decode_result==A081_WIRE_CHECKSUM) ++invalid_checksums;
        else ++unmatched_types;
        a081_controller_radio_input packet{};
        packet.header={A081_CONTROLLER_INPUT_MAGIC,A081_CONTROLLER_INPUT_VERSION,sizeof(packet),A081_CONTROLLER_INPUT_RADIO,++sequence,now(CLOCK_MONOTONIC)};
        packet.irq_monotonic_ns=transfer.irq_timestamp_ns;packet.completion_monotonic_ns=transfer.completion_timestamp_ns;
        std::memcpy(packet.raw,raw,sizeof(packet.raw));
        if(send(sock.fd,&packet,sizeof(packet),MSG_DONTWAIT|MSG_NOSIGNAL)!=static_cast<ssize_t>(sizeof(packet)))throw std::runtime_error("SDK helper backpressure; refusing stale radio backlog");
        const auto log_time=now(CLOCK_MONOTONIC);
        if (log_time>=next_log) {
            next_log=log_time+1000000000ULL;
            std::printf("{\"kind\":\"nordic_rx\",\"sequence\":%llu,\"irq_ns\":%llu,\"completion_ns\":%llu,\"timestamp_tx_hand\":%d,\"tx\":\"",static_cast<unsigned long long>(sequence),static_cast<unsigned long long>(transfer.irq_timestamp_ns),static_cast<unsigned long long>(transfer.completion_timestamp_ns),selected);
            for(unsigned i=0;i<72u;++i) std::printf("%02x",transfer.tx[i]);
            std::printf("\",\"rx\":\"");
            for(unsigned i=0;i<36;++i) std::printf("%02x",raw[i]);
            std::printf("\",\"spi_bytes\":%u,\"rx_offset\":%u}\n",72u,offset);
        }
        }
    }
    std::printf("{\"kind\":\"radio_packet_counts\",\"invalid_headers\":%llu,\"invalid_checksums\":%llu,\"unmatched_types\":%llu,\"accepted_wire_types\":[",
        static_cast<unsigned long long>(invalid_headers),static_cast<unsigned long long>(invalid_checksums),static_cast<unsigned long long>(unmatched_types));
    for(unsigned hand=0;hand<2;++hand) {
        std::printf("%s[",hand?",":"");
        for(unsigned type=0;type<8;++type)
            std::printf("%s%llu",type?",":"",static_cast<unsigned long long>(packet_types[hand][type]));
        std::printf("]");
    }
    std::puts("]}");
    std::printf("{\"kind\":\"radio_summary\",\"packets\":%llu,\"timestamp_requests\":%llu,\"timestamp_transmitted\":%llu,\"timestamp_discarded\":%llu,\"left_type7\":%llu,\"right_type7\":%llu,\"left_clock_window\":%llu,\"right_clock_window\":%llu,\"position_tracking\":false}\n",static_cast<unsigned long long>(sequence),static_cast<unsigned long long>(requests),static_cast<unsigned long long>(transmitted),static_cast<unsigned long long>(discarded),static_cast<unsigned long long>(type7[0]),static_cast<unsigned long long>(type7[1]),static_cast<unsigned long long>(fresh[0]),static_cast<unsigned long long>(fresh[1]));
    return 0;
} catch(const std::exception &e){std::fprintf(stderr,"ERROR: %s (%s)\n",e.what(),std::strerror(errno));return 1;}
