// SPDX-License-Identifier: GPL-2.0-or-later
// Real A081 transport input. Controller clock and integrated vector retain
// their original semantics: neither is an OpenXR pose or Linux timestamp.
#include <xr/device/a081_controller_reference.hpp>
#include <linux/a081-nordic.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <xr/device/a081_radio_clock_reference.h>

static volatile sig_atomic_t stopped;
static void stop(int) { stopped=1; }
static void number(double value) { if(std::isfinite(value)) std::printf("%.12g",value);else std::printf("null"); }
struct Device {int fd{-1};~Device(){if(fd>=0)close(fd);}};
// Original JNI SetEncryptWay(2) selects libserialport DecodePackage_META:
// XOR payload[0] and payload[2..33] with payload[1], retaining payload[1].
// ELF 0x121d2; evidence: controller/startup-contract. The wire header stays
// untouched. In particular, a45b/a758/a659 are NOT repaired into a55a.
static bool decode_meta(const uint8_t raw[36], std::array<uint8_t,36>& out)
{
    std::memcpy(out.data(),raw,out.size());
    if(out[0]!=0xa5 || out[1]!=0x5a)return false;
    const auto key=out[3];
    out[2]^=key;
    for(size_t i=4;i<out.size();++i)out[i]^=key;
    unsigned sum=0;for(size_t i=2;i<35;++i)sum+=out[i];
    return static_cast<uint8_t>(sum)==out[35];
}
int main(int argc,char **argv) try {
    if(argc==2 && std::string(argv[1])=="--transport-abi") {std::puts("a081-nordic-atomic72-v1");return 0;}
    unsigned duration=15;
    if(argc==3 && std::string(argv[1])=="--seconds") {
        char *end;errno=0;unsigned long value=std::strtoul(argv[2],&end,10);
        if(errno || !*argv[2] || *end || !value || value>3600)throw std::runtime_error("seconds must be 1..3600");
        duration=static_cast<unsigned>(value);
    } else if(argc!=1)throw std::runtime_error("Usage: a081-nordic-stream [--seconds N]");
    char model[128]{};FILE *f=std::fopen("/proc/device-tree/model","r");
    if(!f)throw std::runtime_error("A081 model unavailable");
    const auto model_bytes=std::fread(model,1,sizeof(model)-1,f);std::fclose(f);
    if(!model_bytes)throw std::runtime_error("A081 model is empty");
    if(!std::strstr(model,"A081"))throw std::runtime_error("A081 hardware required");
    static_assert(sizeof(a081_nordic_dual_transfer)==168,"Unexpected Nordic transport ABI");
    Device dev;dev.fd=open("/dev/a081-nordic",O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if(dev.fd<0)throw std::runtime_error(std::string("open Nordic: ")+std::strerror(errno));
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);setvbuf(stdout,nullptr,_IOLBF,0);
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(duration);
    uint64_t transfers=0,packets=0,valid=0,last_sequence=0,coalesced=0,valid_meta=0,pattern_packets=0;std::array<uint64_t,2> hands{}, meta_hands{}, pattern_hands{}, clock_backwards{}, last_clock{}, clock_window{}, invalid_slots{};
    std::array<uint8_t,2> versions{};std::array<bool,2> known{};
    while(!stopped && std::chrono::steady_clock::now()<end) {
        pollfd event{dev.fd,POLLIN,0};int ready=poll(&event,1,100);
        if(ready<0){if(errno==EINTR)continue;throw std::runtime_error("Nordic poll failed");}
        if(!ready)continue;
        if(event.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("Nordic transport unavailable");
        if(!(event.revents&POLLIN))continue;
        a081_nordic_dual_transfer transfer{};std::memset(transfer.tx,0xee,sizeof(transfer.tx));
        if(ioctl(dev.fd,A081_NORDIC_IOC_DUAL_TRANSFER,&transfer)<0){if(errno==EAGAIN || errno==EINTR)continue;throw std::runtime_error(std::string("Nordic transfer: ")+std::strerror(errno));}
        ++transfers;
        if(last_sequence && transfer.irq_sequence>last_sequence+1)coalesced+=transfer.irq_sequence-last_sequence-1;
        last_sequence=transfer.irq_sequence;
        timespec boot{};if(clock_gettime(CLOCK_BOOTTIME,&boot))throw std::runtime_error("clock_gettime");
        const uint64_t observed_boot=uint64_t(boot.tv_sec)*1000000000ULL+boot.tv_nsec;
        for(unsigned offset=0;offset<72;offset+=36) {
        ++packets;
        const auto *raw=transfer.rx+offset;
        a081::controller::PreintegratedPacket packet{};
        std::array<uint8_t,36> plain{};
        const bool payload_valid=decode_meta(raw,plain);
        const bool decoded=payload_valid && a081::controller::decode_preintegrated_transfer(plain.data(),plain.size(),packet);
        std::printf("{\"kind\":\"nordic_rx\",\"irq_ns\":%llu,\"completion_ns\":%llu,\"irq_sequence\":%llu,\"rx\":\"",static_cast<unsigned long long>(transfer.irq_timestamp_ns),static_cast<unsigned long long>(transfer.completion_timestamp_ns),static_cast<unsigned long long>(transfer.irq_sequence));
        for(unsigned i=0;i<36;++i)std::printf("%02x",raw[i]);
        std::printf("\",\"spi_bytes\":72,\"rx_offset\":%u,\"raw72\":\"",offset);
        for(auto byte:transfer.rx)std::printf("%02x",byte);
        std::printf("\",\"meta_checksum_valid\":%s,\"preintegrated_checksum_valid\":%s",payload_valid?"true":"false",decoded?"true":"false");
        if(payload_valid) {
            ++valid_meta;const auto* p=plain.data()+2;
            ++meta_hands[p[0]>>7];
            std::printf(",\"wire_type\":%u,\"wire_hand\":%u,\"meta_key\":%u",(p[0]>>4)&7,p[0]>>7,p[1]);
            if(((p[0]>>4)&7)==2) {
                ++pattern_packets;++pattern_hands[p[0]>>7];versions[p[0]>>7]=p[22];known[p[0]>>7]=true;
                std::printf(",\"pattern_version\":[%u,%u,%u],\"pattern_status_byte\":%u,\"pattern_status_low\":%u,\"pattern_status_high\":%u",p[24],p[23],p[22],p[25],p[25]&15,p[25]>>4);
            }
        }
        if(!payload_valid)++invalid_slots[offset/36];
        if(decoded) {
            ++valid;++hands[packet.hand];
            if(last_clock[packet.hand] && packet.clock_value_48<=last_clock[packet.hand])++clock_backwards[packet.hand];
            last_clock[packet.hand]=packet.clock_value_48;
            uint64_t scaled=0;
            const bool clock_known=known[packet.hand] && a081_radio_scale_type7_time(packet.clock_value_48,versions[packet.hand],&scaled);
            const bool window=clock_known && a081_radio_stock_time_window(scaled,observed_boot,nullptr);
            if(window)++clock_window[packet.hand];
            std::printf(",\"clock_version_known\":%s,\"scaled_clock_ns\":%llu,\"observed_boottime_ns\":%llu,\"stock_time_window\":%s",clock_known?"true":"false",static_cast<unsigned long long>(scaled),static_cast<unsigned long long>(observed_boot),window?"true":"false");
            std::printf(",\"hand_bit\":%u,\"controller_clock48\":%llu,\"quaternion_wxyz\":[",packet.hand,static_cast<unsigned long long>(packet.clock_value_48));
            number(packet.orientation.w);std::putchar(',');number(packet.orientation.x);std::putchar(',');number(packet.orientation.y);std::putchar(',');number(packet.orientation.z);
            std::printf("],\"cumulative_vector\":[");for(unsigned i=0;i<3;i++){if(i)std::putchar(',');number(packet.vector[i]);}
            std::printf("],\"battery_nibble\":%u,\"payload1_low_nibble\":%u,\"auxiliary_mode\":%u,\"auxiliary_bytes\":[%u,%u]",packet.battery_nibble,packet.payload1_low_nibble,packet.auxiliary_mode,packet.auxiliary_bytes[0],packet.auxiliary_bytes[1]);
            const auto controls=a081::controller::decode_auxiliary(packet);
            if(controls.kind==a081::controller::AuxiliaryUpdate::Kind::ButtonsFingerAndTouch) {
                std::printf(",\"stock_buttons_raw\":%u,\"stock_touch_raw\":%u,\"stock_index_finger_raw\":%u,\"stock_middle_finger_raw\":%u",controls.buttons_raw,controls.touch_raw,controls.index_finger_raw,controls.middle_finger_raw);
            } else if(controls.kind==a081::controller::AuxiliaryUpdate::Kind::Joystick) {
                std::printf(",\"stock_joystick_raw\":[%d,%d]",controls.joystick_x,controls.joystick_y);
            }
        }
        std::puts("}");
        }
    }
    std::printf("{\"kind\":\"summary\",\"packets\":%llu,\"valid_meta\":%llu,\"pattern_packets\":%llu,\"valid_preintegrated\":%llu,\"hand0\":%llu,\"hand1\":%llu,\"coalesced_irq_notifications\":%llu,\"position_tracking\":false}\n",static_cast<unsigned long long>(packets),static_cast<unsigned long long>(valid_meta),static_cast<unsigned long long>(pattern_packets),static_cast<unsigned long long>(valid),static_cast<unsigned long long>(hands[0]),static_cast<unsigned long long>(hands[1]),static_cast<unsigned long long>(coalesced));
    for(unsigned hand=0;hand<2;++hand)std::printf("{\"kind\":\"hand_summary\",\"hand\":%u,\"valid_meta\":%llu,\"type2\":%llu,\"type7\":%llu,\"clock_backwards_or_duplicate\":%llu,\"clock_window\":%llu}\n",hand,static_cast<unsigned long long>(meta_hands[hand]),static_cast<unsigned long long>(pattern_hands[hand]),static_cast<unsigned long long>(hands[hand]),static_cast<unsigned long long>(clock_backwards[hand]),static_cast<unsigned long long>(clock_window[hand]));
    std::printf("{\"kind\":\"transfer_summary\",\"spi_bytes\":72,\"transfers\":%llu,\"invalid_offset0\":%llu,\"invalid_offset36\":%llu}\n",static_cast<unsigned long long>(transfers),static_cast<unsigned long long>(invalid_slots[0]),static_cast<unsigned long long>(invalid_slots[1]));
    return 0;
} catch(const std::exception& e){std::fprintf(stderr,"ERROR: %s\n",e.what());return 1;}
