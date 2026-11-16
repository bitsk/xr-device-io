// SPDX-License-Identifier: GPL-2.0-or-later
// VR700's original serial transport. Only actual received frames are forwarded.
#include <xr/device/a081_controller_input_wire.h>
#include <xr/device/a081_radio_clock_reference.h>
#define _Static_assert static_assert
#include <xr/device/a081_controller_wire.h>
#undef _Static_assert
#include <xr/device/a081_camera_timing.hpp>
#include <linux/gpio.h>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace {
using u64=uint64_t;
volatile sig_atomic_t stopped;
void stop(int) { stopped=1; }
void require(bool ok,const char *why) { if(!ok)throw std::runtime_error(why); }
struct Fd {
    int fd{-1};
    Fd()=default; Fd(const Fd&)=delete; Fd& operator=(const Fd&)=delete;
    ~Fd() { if(fd>=0)close(fd); }
};
u64 now(clockid_t id=CLOCK_MONOTONIC)
{
    timespec t{};require(!clock_gettime(id,&t),"clock_gettime");
    return u64(t.tv_sec)*1000000000ULL+t.tv_nsec;
}
std::string read_file(const std::string &path)
{
    FILE *f=fopen(path.c_str(),"rb");if(!f)return {};
    char data[1024]{};const auto size=fread(data,1,sizeof(data),f);fclose(f);
    return {data,size};
}
bool compatible(const std::string &data,const char *name)
{
    for(size_t pos=0;pos<data.size();) {
        const auto end=data.find('\0',pos);
        if(data.substr(pos,end==std::string::npos ? end : end-pos)==name)return true;
        if(end==std::string::npos)break;
        pos=end+1;
    }
    return false;
}
void delay(unsigned milliseconds)
{
    timespec t{milliseconds/1000,long(milliseconds%1000)*1000000};
    while(nanosleep(&t,&t)<0 && errno==EINTR){}
}
struct Power {
    Fd lines;
    void set(unsigned bits) {
        gpio_v2_line_values values{};values.mask=3;values.bits=bits;
        require(!ioctl(lines.fd,GPIO_V2_LINE_SET_VALUES_IOCTL,&values),"controller GPIO set");
    }
    void start(const std::string &chosen)
    {
        for(unsigned i=0;i<64 && lines.fd<0;++i) {
            const auto name="gpiochip"+std::to_string(i);
            const auto path="/dev/"+name;
            if(!chosen.empty() && chosen!=path)continue;
            auto node=read_file("/sys/bus/gpio/devices/"+name+"/device/of_node/compatible");
            if(node.empty())node=read_file("/sys/bus/gpio/devices/"+name+"/of_node/compatible");
            if(!compatible(node,"qcom,sm8250-pinctrl"))continue;
            Fd chip;chip.fd=open(path.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW);
            gpiochip_info info{};
            require(chip.fd>=0 && !ioctl(chip.fd,GPIO_GET_CHIPINFO_IOCTL,&info),"TLMM gpiochip info");
            require(std::strstr(info.label,"pinctrl") && info.lines>115,"Unexpected TLMM label/size");
            gpio_v2_line_request request{};
            request.offsets[0]=41;request.offsets[1]=115;request.num_lines=2;
            std::strcpy(request.consumer,"xr-controller-uart");
            request.config.flags=GPIO_V2_LINE_FLAG_OUTPUT;
            request.config.num_attrs=1;
            request.config.attrs[0].attr.id=GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
            request.config.attrs[0].attr.values=0;request.config.attrs[0].mask=3;
            require(!ioctl(chip.fd,GPIO_V2_GET_LINE_IOCTL,&request),"Controller power/reset already owned");
            lines.fd=request.fd;
            std::printf("{\"kind\":\"controller_gpio_owner\",\"chip\":\"%s\",\"label\":\"%s\",\"power\":41,\"reset\":115}\n",path.c_str(),info.label);
        }
        require(lines.fd>=0,"No verified SM8250 TLMM gpiochip");
        // Levels are stock evidence; 100/500ms are conservative engineering
        // margins, not measurements of the original driver's reset timing.
        set(1);delay(100);if(stopped)return;
        set(3);delay(500);
    }
    ~Power() {
        if(lines.fd>=0) {
            gpio_v2_line_values values{};values.mask=3;values.bits=1;
            const int reset=ioctl(lines.fd,GPIO_V2_LINE_SET_VALUES_IOCTL,&values);
            values.bits=0;const int power=ioctl(lines.fd,GPIO_V2_LINE_SET_VALUES_IOCTL,&values);
            if(reset || power)std::fputs("controller GPIO shutdown failed\n",stderr);
        }
    }
};
struct Uart {
    Fd dev;termios saved{};bool restore{};
    void open_device(const std::string &path)
    {
        dev.fd=open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC|O_NOFOLLOW);
        struct stat st{};
        require(dev.fd>=0 && !fstat(dev.fd,&st) && S_ISCHR(st.st_mode),"UART open");
        require(!ioctl(dev.fd,TIOCEXCL),"UART exclusive ownership");
        require(!tcgetattr(dev.fd,&saved),"UART termios");restore=true;
        termios t=saved;cfmakeraw(&t);
        t.c_cflag&=~(CSIZE|PARENB|CSTOPB);t.c_cflag|=CS8|CREAD|CLOCAL|CRTSCTS;
        t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;
        require(!cfsetispeed(&t,B921600) && !cfsetospeed(&t,B921600) && !tcsetattr(dev.fd,TCSANOW,&t),"UART 921600 8N1 RTSCTS");
        termios actual{};require(!tcgetattr(dev.fd,&actual) && cfgetispeed(&actual)==B921600 &&
            cfgetospeed(&actual)==B921600 && (actual.c_cflag&(CSIZE|PARENB|CSTOPB|CRTSCTS))==(CS8|CRTSCTS),"UART settings readback");
        require(!tcflush(dev.fd,TCIOFLUSH),"UART initial flush");
    }
    void write_frame(const uint8_t *bytes,size_t size)
    {
        const auto deadline=now()+20000000ULL;
        size_t offset=0;
        while(offset<size) {
            const auto result=write(dev.fd,bytes+offset,size-offset);
            if(result>0) { offset+=size_t(result);continue; }
            if(result<0 && errno==EINTR)continue;
            require(result<0 && (errno==EAGAIN || errno==EWOULDBLOCK),"UART write");
            require(now()<deadline && !stopped,"UART TX deadline");
            pollfd p{dev.fd,POLLOUT,0};const int rc=poll(&p,1,2);
            require(rc>=0 || errno==EINTR,"UART TX poll");
            require(!(p.revents&(POLLERR|POLLHUP|POLLNVAL)),"UART TX disconnected");
        }
    }
    ~Uart() {
        if(dev.fd>=0) { if(restore)tcsetattr(dev.fd,TCSANOW,&saved);ioctl(dev.fd,TIOCNXCL); }
    }
};
}
int main(int argc,char **argv) try {
    if(argc==2 && !std::strcmp(argv[1],"--print-abi")) {std::puts(A081_CONTROLLER_INPUT_ABI_DESCRIPTOR);return 0;}
    std::string socket_path,device="/dev/ttyHS1",gpiochip;unsigned seconds=30;bool camera_time=false;
    for(int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        if(arg=="--socket" && i+1<argc)socket_path=argv[++i];
        else if(arg=="--device" && i+1<argc)device=argv[++i];
        else if(arg=="--gpiochip" && i+1<argc)gpiochip=argv[++i];
        else if(arg=="--baud" && i+1<argc)require(!std::strcmp(argv[++i],"921600"),"Only verified baud 921600 is supported");
        else if(arg=="--seconds" && i+1<argc) {
            char *end=nullptr;errno=0;const auto value=std::strtoul(argv[++i],&end,10);
            require(!errno && *argv[i] && !*end && value<=3600,"seconds must be 0..3600");seconds=unsigned(value);
        } else if(arg=="--camera-time")camera_time=true;
        else throw std::runtime_error("Usage: xr-controller-uart --socket PATH [--device /dev/ttyHS1] [--baud 921600] [--gpiochip PATH] [--seconds 0..3600] [--camera-time]");
    }
    require(geteuid()==0 && compatible(read_file("/proc/device-tree/compatible"),"iqiyi,vr700"),"Expected root on VR700");
    require(device=="/dev/ttyHS1" && !socket_path.empty() && socket_path.front()=='/' && socket_path.size()<sizeof(sockaddr_un::sun_path),"Expected verified UART and absolute socket");
    setvbuf(stdout,nullptr,_IOLBF,0);signal(SIGINT,stop);signal(SIGTERM,stop);signal(SIGPIPE,SIG_IGN);
    Fd lock;lock.fd=open("/run/xr-controller-uart.lock",O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    require(lock.fd>=0 && !flock(lock.fd,LOCK_EX|LOCK_NB),"Controller UART owner busy");
    struct stat st{};
    require(!lstat(socket_path.c_str(),&st) && S_ISSOCK(st.st_mode) && st.st_uid==0 && !(st.st_mode&0077),"Expected root-only SDK socket");
    Fd sock;sock.fd=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC|SOCK_NONBLOCK,0);
    sockaddr_un address{};address.sun_family=AF_UNIX;std::strcpy(address.sun_path,socket_path.c_str());
    ucred peer{};socklen_t peer_size=sizeof(peer);
    require(sock.fd>=0 && !connect(sock.fd,reinterpret_cast<sockaddr *>(&address),sizeof(address)) &&
        !getsockopt(sock.fd,SOL_SOCKET,SO_PEERCRED,&peer,&peer_size) && peer_size==sizeof(peer) && peer.uid==0,"SDK peer connection");
    Uart uart;uart.open_device(device);Power power;power.start(gpiochip);
    std::puts("{\"kind\":\"controller_uart_ready\",\"baud\":921600,\"flow_control\":\"RTSCTS\",\"rx_bytes\":36,\"heartbeat_ms\":100}");
    std::array<uint8_t,16384> buffer{};size_t used=0;
    u64 sequence=0,rx=0,valid=0,bad=0,dropped=0,heartbeats=0,timestamps=0,rejected=0,request_sequence=0;
    u64 types[2][8]{},last_sof[2]{};bool pattern[2]{};
    auto next_heartbeat=now()+100000000ULL,next_log=now()+1000000000ULL;
    const auto deadline=now()+u64(seconds)*1000000000ULL;
    while(!stopped && (!seconds || now()<deadline)) {
        pollfd p[2]{{uart.dev.fd,POLLIN,0},{sock.fd,POLLIN,0}};
        const int rc=poll(p,2,5);if(rc<0 && errno==EINTR)continue;require(rc>=0,"UART poll");
        for(const auto &fd:p)require(!(fd.revents&(POLLHUP|POLLERR|POLLNVAL)),"UART/helper disconnected");
        if(p[0].revents&POLLIN) {
            const auto size=read(uart.dev.fd,buffer.data()+used,buffer.size()-used);
            if(size<0 && errno!=EINTR && errno!=EAGAIN)throw std::runtime_error("UART read");
            if(size>0) {
                const auto completion=now();used+=size_t(size);rx+=u64(size);size_t consumed=0;
                while(used-consumed>=36) {
                    a081_controller_wire_packet decoded{};
                    const auto result=a081_controller_decode_wire(buffer.data()+consumed,&decoded);
                    if(result!=A081_WIRE_OK) {++consumed;++bad;continue;}
                    ++valid;++types[decoded.hand][decoded.type];
                    if(decoded.type==2)pattern[decoded.hand]=true;
                    if(decoded.type==2 || decoded.type==7) {
                        a081_controller_radio_input frame{};
                        frame.header={A081_CONTROLLER_INPUT_MAGIC,A081_CONTROLLER_INPUT_VERSION,sizeof(frame),A081_CONTROLLER_INPUT_RADIO,++sequence,now()};
                        // UART has no captured IRQ timestamp. Never invent one.
                        frame.completion_monotonic_ns=completion;
                        std::memcpy(frame.raw,buffer.data()+consumed,36);
                        const auto sent=send(sock.fd,&frame,sizeof(frame),MSG_DONTWAIT|MSG_NOSIGNAL);
                        if(sent<0 && (errno==EAGAIN || errno==EWOULDBLOCK))++dropped;
                        else require(sent==sizeof(frame),"SDK packet send");
                    }
                    consumed+=36;
                }
                if(consumed) {used-=consumed;std::memmove(buffer.data(),buffer.data()+consumed,used);}
            }
        }
        if(p[1].revents&POLLIN)for(unsigned i=0;i<8;++i) {
            a081_controller_timestamp_request q{};
            const auto size=recv(sock.fd,&q,sizeof(q),MSG_DONTWAIT|MSG_TRUNC);
            if(size<0 && (errno==EAGAIN || errno==EINTR))break;
            require(size>0,"SDK helper closed");
            const auto mono=now(),boot=now(CLOCK_BOOTTIME);const auto &h=q.header;
            a081::stock::camera::StockY8Time time{};
            const bool exposure=q.ctrl_exposure_ns<=1000000000ULL &&
                a081::stock::camera::stock_a081_y8_time(q.ctrl_sof_boottime_ns,q.ctrl_exposure_ns,time) &&
                time.exposure_start_boottime_ns==q.ctrl_exposure_start_boottime_ns;
            if(size!=sizeof(q) || !camera_time || !exposure || h.magic!=A081_CONTROLLER_INPUT_MAGIC ||
               h.version!=A081_CONTROLLER_INPUT_VERSION || h.bytes!=sizeof(q) || h.type!=A081_CONTROLLER_OUTPUT_TIMESTAMP_REQUEST ||
               h.sequence<=request_sequence || !h.sent_monotonic_ns || h.sent_monotonic_ns>mono || mono-h.sent_monotonic_ns>=33000000ULL ||
               q.hand>1 || q.exposure_phase>1 || q.reserved || !q.phase_anchor_sof_qtimer ||
               a081_radio_hand_for_exposure_phase(q.exposure_phase)!=int(q.hand) || !pattern[q.hand] ||
               !q.ctrl_exposure_start_boottime_ns || q.ctrl_exposure_start_boottime_ns>=boot || boot-q.ctrl_exposure_start_boottime_ns>=33000000ULL ||
               q.ctrl_sof_boottime_ns<=last_sof[q.hand]) {++rejected;continue;}
            request_sequence=h.sequence;
            a081_camera_time_body body{};
            // VR700 WriteCameraTime always divides microseconds by 30.5,
            // regardless of PatternID version. Its UART Write sends 12 bytes.
            require(a081_radio_pack_camera_time_body(q.exposure_phase,q.ctrl_exposure_start_boottime_ns,
                static_cast<a081_controller_hand>(q.hand),0,&body),"camera timestamp encoding");
            uart.write_frame(body.bytes,sizeof(body.bytes));last_sof[q.hand]=q.ctrl_sof_boottime_ns;++timestamps;
        }
        if(now()>=next_heartbeat) {
            // OEM HeartBeatThread -> WriteSyncWithIMU, 100ms per hand.
            const uint8_t left[]={0xa5,8,0x51,0,0,0,0,0x59};
            const uint8_t right[]={0xa5,8,0xd2,0,0,0,0,0xda};
            uart.write_frame(left,sizeof(left));uart.write_frame(right,sizeof(right));heartbeats+=2;
            next_heartbeat=now()+100000000ULL;
        }
        if(now()>=next_log) {
            std::printf("{\"kind\":\"controller_uart_status\",\"rx_bytes\":%llu,\"frames\":%llu,\"resync_bytes\":%llu,\"dropped\":%llu,\"heartbeat_tx\":%llu,\"timestamp_tx\":%llu,\"timestamp_rejected\":%llu,\"pattern\":[%llu,%llu],\"imu\":[%llu,%llu]}\n",
                (unsigned long long)rx,(unsigned long long)valid,(unsigned long long)bad,(unsigned long long)dropped,
                (unsigned long long)heartbeats,(unsigned long long)timestamps,(unsigned long long)rejected,
                (unsigned long long)types[0][2],(unsigned long long)types[1][2],(unsigned long long)types[0][7],(unsigned long long)types[1][7]);
            next_log=now()+1000000000ULL;
        }
    }
    return 0;
} catch(const std::exception &error) {std::fprintf(stderr,"xr-controller-uart: %s (%s)\n",error.what(),std::strerror(errno));return 1;}
