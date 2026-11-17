// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <xr/device/a081_controller_input_wire.h>
#include <xr/device/abi_reference.hpp>
#include <xr/device/a081_camera_groups.hpp>
#include <xr/device/a081_camera_pixels.hpp>
#include <xr/device/a081_camera_timing.hpp>
#include <cmath>
#include <cstdlib>
#include <array>
#include <functional>
#include <utility>
#include <fcntl.h>
#include <linux/memfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <unistd.h>

namespace a081::stock::controller {
struct NativeHeadPose {
    std::uint64_t sample_qtimer_ns{};
    std::array<float,3> position{};
    std::array<float,4> orientation_xyzw{};
};
using NativeHistoryQuery=std::function<bool(std::uint64_t query_qtimer_ns,NativeHeadPose &)>;

// Optional tee from the existing camera owner. Pixels are copied into a sealed
// memfd; SDK release callbacks own their mappings independently of V4L2/QVR.
// No second camera open, synthetic frame, timestamp replacement or prediction.
class FrameExporter {
public:
    explicit FrameExporter(const char *environment_key = "A081_CONTROLLER_INPUT_SOCKET")
        : hand_observer_(std::string(environment_key) == "A081_HAND_INPUT_SOCKET")
    {
        const char *path = std::getenv(environment_key);
        if (!path || !*path) return;
        if (geteuid() != 0 || path[0] != '/' || std::strlen(path) >= sizeof(sockaddr_un::sun_path))
            throw std::runtime_error("Invalid controller frame endpoint");
        path_ = path;
        if (std::getenv("A081_CONTROLLER_CANDIDATE_PHASE") || std::getenv("A081_CONTROLLER_FRAME_PHASE_PROFILE"))
            throw std::runtime_error("Sequence-derived controller phases are unsupported; require QCFT v2 kernel schedule");
        const char *device = std::getenv("XR_DEVICE");
        if (hand_observer_ && device && std::string(device) == "vr700") {
            // capture_live verifies the selected DT identity. The session
            // reads these fixed controls back before STREAMON; no AE/pattern
            // runs in this mode. Preserve their nominal, non-optical scope.
            auto exposure = [](const char *key) {
                const char *value = std::getenv(key);
                char *end = nullptr;
                errno = 0;
                const auto ns = value ? std::strtoull(value, &end, 10) : 0;
                if (errno || !value || *value < '0' || *value > '9' || !end || *end ||
                    !ns || ns > 1000000000ULL)
                    throw std::runtime_error("Missing/invalid fixed hand exposure controls");
                return ns;
            };
            fixed_hand_exposure_ns_ = exposure("A081_QVR_CAMERA_EXPOSURE_NS");
            if (fixed_hand_exposure_ns_ != exposure("A081_CONTROLLER_CAMERA_EXPOSURE_NS"))
                throw std::runtime_error("Fixed hand capture requires equal four-camera exposure");
        }
        std::cout << "stage=controller_frame_export configured=1 phase_source="
                  << (fixed_hand_exposure_ns_ ? "vr700_fixed_controls" : "kernel_sof_schedule")
                  << " observer=" << (hand_observer_ ? "hand" : "controller")
                  << " fixed_nominal_exposure_ns=" << fixed_hand_exposure_ns_
                  << " optical_phase_verified=0 hmd_sample=native_ring_nearest_no_prediction" << std::endl;
    }
    ~FrameExporter()
    {
        if (fd_ >= 0) ::close(fd_);
        if (!path_.empty())
            std::cout << "stage=controller_frame_export_summary sent=" << sent_
                      << " missing_pose=" << missing_pose_ << " dropped=" << dropped_
                      << " optical_phase_verified=0" << std::endl;
    }
    // Bind before capture starts; clear only after capture and main frame
    // callers have stopped. The callback owns no Ring or DMA lifetime.
    void set_native_history_query(NativeHistoryQuery query) { history_query_=std::move(query); }
    // Invoked by the serialized group-ready callback, independently of QVR
    // RPC completion. Capture is joined before this exporter is destroyed.
    void frame(const camera::FrameGroup &group)
    {
        if (path_.empty()) return;
        const auto &meta = group.frames[2].timing;
        // A separate observer consumes only matched long-exposure groups.
        // The existing controller exporter continues receiving every phase.
        if (hand_observer_ && ((!fixed_hand_exposure_ns_ && meta.exposure_phase != 2) || !connect())) return;
        if (fixed_hand_exposure_ns_) {
            // Require the actual fixed-mode QCFT state for all four verified
            // image pairs. Missing SOF or an invalid/partial schedule is not a
            // fixed-exposure fallback. FrameGroups retains its 1 ms gate.
            for (const auto &frame : group.frames) {
                const auto &t = frame.timing;
                if (!(t.flags & QCOM_CAMSS_TIMING_VALID) ||
                    (t.flags & (QCOM_CAMSS_TIMING_PATTERN_ACTIVE | QCOM_CAMSS_TIMING_EXPOSURE_VALID)) ||
                    !t.sof_qtimer || !t.sof_boottime_ns || t.exposure_phase != UINT32_MAX ||
                    t.exposure_status != -ENODATA || t.exposure_ns || t.phase_anchor_sof_qtimer ||
                    t.control_apply_sof_qtimer || t.control_applied_boottime_ns) {
                    ++dropped_; return;
                }
            }
        } else if (!camera::valid_pattern_exposure(meta) || !camera::valid_pattern_exposure(group.frames[3].timing)) {
            ++dropped_; return;
        }
        const auto exposure_ns = fixed_hand_exposure_ns_ ? fixed_hand_exposure_ns_ : meta.exposure_ns;
        const auto ctrl_sof = meta.sof_boottime_ns;
        camera::StockY8Time stock_time{};
        if (!camera::stock_a081_y8_time(ctrl_sof, exposure_ns, stock_time)) { ++dropped_; return; }
        const auto ctrl_time = stock_time.exposure_start_boottime_ns;
        camera::StockY8NativeTime native_time{};
        if(!camera::stock_a081_y8_native_time(meta.sof_qtimer,exposure_ns,native_time)) { ++dropped_;return; }
        NativeHeadPose head_pose{};std::uint64_t sample_boot{};
        // The nonzero camera exposure-start query selects an original Ring1
        // record by absolute timestamp distance, including future records.
        // Its epoch remains the returned sample epoch, never the query time.
        bool have_pose=history_query_ && history_query_(native_time.exposure_start_qtimer_ns,head_pose);
        if(have_pose) {
            // Numeric representation only: keep the callback's raw record and
            // sample epoch, normalize this SDK-bound local quaternion copy.
            // No norm tolerance, quality, state or query-distance admission.
            double norm=0;
            for(float x:head_pose.position) if(!std::isfinite(x)) have_pose=false;
            for(float x:head_pose.orientation_xyzw) {
                if(!std::isfinite(x)) have_pose=false;
                norm+=double(x)*x;
            }
            if(!std::isfinite(norm) || norm<=0) have_pose=false;
            if(have_pose) {
                const auto length=std::sqrt(norm);
                for(auto &x:head_pose.orientation_xyzw) x=static_cast<float>(x/length);
                have_pose=camera::native_sample_boottime(head_pose.sample_qtimer_ns,meta.sample_qtimer,
                                                        meta.sample_boottime_ns,sample_boot);
            }
        }
        if (!have_pose) ++missing_pose_;
        head::Bytes tracking, ctrl;
        if (!camera::side_by_side(group.frames[0].image.y8, group.frames[1].image.y8, tracking) ||
            !camera::side_by_side(group.frames[2].image.y8, group.frames[3].image.y8, ctrl)) {
            ++dropped_; return;
        }
        std::uint64_t bracket = 0;
        for (const auto &f : group.frames) bracket = std::max(bracket, std::uint64_t(f.timing.clock_bracket_ns));
        if (!connect()) { ++dropped_; return; }
        a081_controller_frame_input packet{};
        packet.header = {A081_CONTROLLER_INPUT_MAGIC, A081_CONTROLLER_INPUT_VERSION, sizeof(packet),
                         A081_CONTROLLER_INPUT_FRAME, ++sequence_, monotonic_ns()};
        packet.view_width = camera::width; packet.height = camera::height;
        packet.stride_bytes = camera::width * 2;
        packet.tracking_frame_id = group.frames[0].image.sequence;
        packet.ctrl_frame_id = group.frames[2].image.sequence;
        packet.flags = A081_CONTROLLER_FRAME_REAL_QCFT_PAIR | A081_CONTROLLER_FRAME_STOCK_CAMERA_ORDER;
        if (have_pose) packet.flags |= A081_CONTROLLER_FRAME_RAW_QVR_POSE;
        packet.flags |= fixed_hand_exposure_ns_ ? A081_CONTROLLER_FRAME_FIXED_EXPOSURE_CONTROLS :
                                                 A081_CONTROLLER_FRAME_KERNEL_EXPOSURE_PHASE;
        packet.ctrl_sensor_sequence = meta.sensor_sequence;
        packet.ctrl_exposure_phase = meta.exposure_phase;
        packet.ctrl_phase_anchor_sof_qtimer = meta.phase_anchor_sof_qtimer;
        packet.ctrl_control_apply_sof_qtimer = meta.control_apply_sof_qtimer;
        packet.ctrl_control_applied_boottime_ns = meta.control_applied_boottime_ns;
        packet.tracking_sof_boottime_ns = group.frames[0].timing.sof_boottime_ns;
        packet.ctrl_sof_boottime_ns = ctrl_sof;
        packet.ctrl_exposure_start_boottime_ns = ctrl_time;
        packet.ctrl_exposure_ns = exposure_ns;
        packet.camera_clock_bracket_ns = bracket;
        if (have_pose) {
            packet.hmd_pose_boottime_ns = sample_boot;
            std::copy(head_pose.position.begin(),head_pose.position.end(), packet.hmd_position);
            std::copy(head_pose.orientation_xyzw.begin(),head_pose.orientation_xyzw.end(), packet.hmd_orientation_xyzw);
        }
        packet.mosaic_bytes[0] = tracking.size(); packet.mosaic_bytes[1] = ctrl.size();
        packet.mosaic_offsets[1] = tracking.size();
        // Zero hand state delegates to helper's fresh checksum-verified radio
        // state. It is not an assertion that either physical hand is enabled.
        const int memory = static_cast<int>(syscall(SYS_memfd_create, "a081-real-controller-frame", MFD_CLOEXEC | MFD_ALLOW_SEALING));
        if (memory < 0) { ++dropped_; return; }
        bool ok = write_all(memory, tracking) && write_all(memory, ctrl) &&
                  fcntl(memory, F_ADD_SEALS, F_SEAL_WRITE | F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL) == 0;
        if (ok) {
            iovec iov{&packet, sizeof(packet)};
            alignas(cmsghdr) char control[CMSG_SPACE(sizeof(memory))]{};
            msghdr msg{}; msg.msg_iov = &iov; msg.msg_iovlen = 1;
            msg.msg_control = control; msg.msg_controllen = sizeof(control);
            auto *c = CMSG_FIRSTHDR(&msg); c->cmsg_level = SOL_SOCKET; c->cmsg_type = SCM_RIGHTS;
            c->cmsg_len = CMSG_LEN(sizeof(memory)); std::memcpy(CMSG_DATA(c), &memory, sizeof(memory));
            ok = sendmsg(fd_, &msg, MSG_DONTWAIT | MSG_NOSIGNAL) == static_cast<ssize_t>(sizeof(packet));
        }
        ::close(memory);
        if (ok) {
            ++sent_;
            if (sent_ == 1 || sent_ % 100 == 0)
                std::cout << "stage=controller_frame_export sent=" << sent_
                          << " ctrl_frame=" << packet.ctrl_frame_id
                          << " ctrl_sof_ns=" << packet.ctrl_sof_boottime_ns
                          << " ctrl_exposure_start_ns=" << packet.ctrl_exposure_start_boottime_ns
                          << " ctrl_exposure_ns=" << packet.ctrl_exposure_ns
                          << " hmd_pose_ns=" << packet.hmd_pose_boottime_ns
                          << " hmd_valid=" << have_pose
                          << " hmd_query_qtimer_ns=" << native_time.exposure_start_qtimer_ns
                          << " hmd_sample_qtimer_ns=" << head_pose.sample_qtimer_ns
                          << " hmd_query_distance_ns=" << (have_pose ?
                              (head_pose.sample_qtimer_ns>native_time.exposure_start_qtimer_ns ?
                               head_pose.sample_qtimer_ns-native_time.exposure_start_qtimer_ns :
                               native_time.exposure_start_qtimer_ns-head_pose.sample_qtimer_ns) : 0)
                          << " exposure_phase=" << packet.ctrl_exposure_phase << std::endl;
        }
        else { ++dropped_; ::close(fd_); fd_ = -1; }
    }
private:
    static std::uint64_t monotonic_ns()
    {
        timespec t{}; if (clock_gettime(CLOCK_MONOTONIC, &t)) return 0;
        return std::uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
    }
    static bool write_all(int fd, const head::Bytes &bytes)
    {
        std::size_t offset = 0;
        while (offset < bytes.size()) {
            const auto n = write(fd, bytes.data() + offset, bytes.size() - offset);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) return false;
            offset += static_cast<std::size_t>(n);
        }
        return true;
    }
    bool connect()
    {
        if (fd_ >= 0) return true;
        struct stat st{};
        if (lstat(path_.c_str(), &st) || !S_ISSOCK(st.st_mode) || st.st_uid != 0 || (st.st_mode & 0077)) return false;
        const int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
        if (fd < 0) return false;
        sockaddr_un address{}; address.sun_family = AF_UNIX; std::strcpy(address.sun_path, path_.c_str());
        ucred cred{}; socklen_t n = sizeof(cred);
        if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
            getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &cred, &n) || n != sizeof(cred) || cred.uid != 0) {
            ::close(fd); return false;
        }
        fd_ = fd; return true;
    }
    bool hand_observer_ = false;
    std::uint64_t fixed_hand_exposure_ns_{};
    std::string path_;
    NativeHistoryQuery history_query_;
    int fd_{-1};
    std::uint64_t sequence_{}, sent_{}, dropped_{}, missing_pose_{};
};
} // namespace a081::stock::controller
