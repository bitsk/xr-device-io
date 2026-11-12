// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <xr/device/a081_controller_input_wire.h>
#include <xr/head/production-v2/a081_tracking_wire.h>
#include <xr/device/a081_camera_groups.hpp>
#include <xr/device/a081_camera_pixels.hpp>
#include <xr/device/a081_camera_timing.hpp>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fcntl.h>
#include <linux/memfd.h>
#include <mutex>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <unistd.h>

namespace a081::stock::controller {
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
        std::cout << "stage=controller_frame_export configured=1 phase_source=kernel_sof_schedule"
                  << " optical_phase_verified=0 hmd_sample=real_previous_no_prediction" << std::endl;
    }
    ~FrameExporter()
    {
        if (fd_ >= 0) ::close(fd_);
        if (!path_.empty())
            std::cout << "stage=controller_frame_export_summary sent=" << sent_
                      << " missing_pose=" << missing_pose_ << " dropped=" << dropped_
                      << " optical_phase_verified=0" << std::endl;
    }
    void pose(const a081_tracking_packet &p)
    {
        if (path_.empty() || p.pose_read_status != A081_POSE_COPIED || p.pose_quality != 1 ||
            p.tracking_state != 4 || p.pose_boottime_ns != p.gyro_boottime_ns ||
            !p.pose_boottime_ns || (p.flags & A081_TRACKING_KNOWN_FLAGS) != A081_TRACKING_KNOWN_FLAGS)
            return;
        double norm = 0;
        for (float x : p.rotation_xyzw) { if (!std::isfinite(x)) return; norm += x * x; }
        for (float x : p.translation) if (!std::isfinite(x)) return;
        if (std::abs(norm - 1) > .01) return;
        std::lock_guard<std::mutex> guard(poses_mutex_);
        if (!poses_.empty() && p.pose_boottime_ns <= poses_.back().pose_boottime_ns) return;
        poses_.push_back(p);
        while (poses_.size() > 128) poses_.pop_front();
    }
    // Invoked by the serialized group-ready callback, independently of QVR
    // RPC completion. Capture is joined before this exporter is destroyed.
    void frame(const camera::FrameGroup &group)
    {
        if (path_.empty()) return;
        const auto &meta = group.frames[2].timing;
        // A separate observer consumes only matched long-exposure groups.
        // The existing controller exporter continues receiving every phase.
        if (hand_observer_ && (meta.exposure_phase != 2 || !connect())) return;
        if (!camera::valid_pattern_exposure(meta) || !camera::valid_pattern_exposure(group.frames[3].timing)) {
            ++dropped_; return;
        }
        const auto ctrl_sof = meta.sof_boottime_ns;
        camera::StockY8Time stock_time{};
        if (!camera::stock_a081_y8_time(ctrl_sof, meta.exposure_ns, stock_time)) { ++dropped_; return; }
        const auto ctrl_time = stock_time.exposure_start_boottime_ns;
        a081_tracking_packet head_pose{};
        bool have_pose = false;
        {
            std::lock_guard<std::mutex> guard(poses_mutex_);
            auto pose = std::find_if(poses_.rbegin(), poses_.rend(), [&](const auto &p) {
                return p.pose_boottime_ns <= ctrl_time;
            });
            have_pose = pose != poses_.rend() && ctrl_time - pose->pose_boottime_ns < 20000000ULL;
            if (have_pose) head_pose = *pose;
        }
        // Stock camera timestamps keep the radio clock alive even while QVR
        // has no valid head pose. Mark that absence explicitly; the SDK must
        // still reject image fusion without a real, recent head pose.
        if (!have_pose) ++missing_pose_;
        head::Bytes tracking, ctrl;
        if (!camera::side_by_side(group.frames[0].image.y8, group.frames[1].image.y8, tracking) ||
            !camera::side_by_side(group.frames[2].image.y8, group.frames[3].image.y8, ctrl)) {
            ++dropped_; return;
        }
        std::uint64_t bracket = 0;
        for (const auto &f : group.frames) bracket = std::max(bracket, std::uint64_t(f.timing.clock_bracket_ns));
        if (bracket >= 5000 || !connect()) { ++dropped_; return; }
        a081_controller_frame_input packet{};
        packet.header = {A081_CONTROLLER_INPUT_MAGIC, A081_CONTROLLER_INPUT_VERSION, sizeof(packet),
                         A081_CONTROLLER_INPUT_FRAME, ++sequence_, monotonic_ns()};
        packet.view_width = camera::width; packet.height = camera::height;
        packet.stride_bytes = camera::width * 2;
        packet.tracking_frame_id = group.frames[0].image.sequence;
        packet.ctrl_frame_id = group.frames[2].image.sequence;
        packet.flags = A081_CONTROLLER_FRAME_REAL_QCFT_PAIR | A081_CONTROLLER_FRAME_STOCK_CAMERA_ORDER;
        if (have_pose) packet.flags |= A081_CONTROLLER_FRAME_RAW_QVR_POSE;
        packet.flags |= A081_CONTROLLER_FRAME_KERNEL_EXPOSURE_PHASE;
        packet.ctrl_sensor_sequence = meta.sensor_sequence;
        packet.ctrl_exposure_phase = meta.exposure_phase;
        packet.ctrl_phase_anchor_sof_qtimer = meta.phase_anchor_sof_qtimer;
        packet.ctrl_control_apply_sof_qtimer = meta.control_apply_sof_qtimer;
        packet.ctrl_control_applied_boottime_ns = meta.control_applied_boottime_ns;
        packet.tracking_sof_boottime_ns = group.frames[0].timing.sof_boottime_ns;
        packet.ctrl_sof_boottime_ns = ctrl_sof;
        packet.ctrl_exposure_start_boottime_ns = ctrl_time;
        packet.ctrl_exposure_ns = meta.exposure_ns;
        packet.camera_clock_bracket_ns = bracket;
        if (have_pose) {
            packet.hmd_pose_boottime_ns = head_pose.pose_boottime_ns;
            std::copy(std::begin(head_pose.translation), std::end(head_pose.translation), packet.hmd_position);
            std::copy(std::begin(head_pose.rotation_xyzw), std::end(head_pose.rotation_xyzw), packet.hmd_orientation_xyzw);
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
                          << " hmd_age_ns=" << (have_pose ? packet.ctrl_exposure_start_boottime_ns - packet.hmd_pose_boottime_ns : 0)
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
    std::string path_;
    std::mutex poses_mutex_;
    std::deque<a081_tracking_packet> poses_;
    int fd_{-1};
    std::uint64_t sequence_{}, sent_{}, dropped_{}, missing_pose_{};
};
} // namespace a081::stock::controller
