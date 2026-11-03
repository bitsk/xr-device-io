// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <xr/device/a081_camera_metadata.hpp>
#include <array>
#include <cstdlib>
#include <sstream>

namespace a081::stock::camera {
static_assert(sizeof(qcom_camss_exposure_pattern) == 88 &&
              offsetof(qcom_camss_exposure_pattern, anchor_sof_qtimer) == 8 &&
              offsetof(qcom_camss_exposure_pattern, steps) == 24 &&
              offsetof(qcom_camss_exposure_pattern, reserved) == 72);
// One arm operation, anchored to a matched group of real hardware SOFs.
// The kernel advances the schedule only on SOF IRQs. No userspace timer or
// image sequence number decides the phase. All sensor IO stays with this owner.
class ExposurePattern {
public:
    ExposurePattern()
    {
        const char *profile = std::getenv("A081_CONTROLLER_EXPOSURE_PROFILE");
        enabled_ = profile && std::string(profile) == "kernel-three-phase";
        if (!enabled_) return;
        phase2_lines_ = positive_env("A081_CONTROLLER_PATTERN_PHASE2_LINES");
        phase2_gain_ = positive_env("A081_CONTROLLER_PATTERN_PHASE2_GAIN");
        if (phase2_lines_ > 670 || phase2_gain_ > 1023)
            throw std::runtime_error("Invalid held AE settings for 690-line pattern");
        const char *nodes = std::getenv("A081_CONTROLLER_PATTERN_CSID_NODES");
        if (!nodes) throw std::runtime_error("Pattern requires four owned CSID nodes");
        std::istringstream input(nodes);
        for (auto &node : nodes_) {
            if (!std::getline(input, node, ',') || node.rfind("/dev/v4l-subdev", 0) != 0)
                throw std::runtime_error("Invalid pattern CSID node list");
        }
        std::string extra;
        if (std::getline(input, extra, ',')) throw std::runtime_error("Extra pattern CSID node");
    }
    ~ExposurePattern() { disable_armed(); }
    bool enabled() const { return enabled_; }
    bool admit(const std::array<TimedFrame, 4> &frames)
    {
        if (!enabled_) return true;
        if (failed_) return false;
        if (armed_mask_ != 15) {
            // Check the raw sensor SOF domain too: independent host clock
            // brackets and exposure-start arithmetic must not pick the anchor.
            std::uint64_t earliest = UINT64_MAX, latest = 0;
            for (const auto &frame : frames) {
                earliest = std::min(earliest, std::uint64_t(frame.timing.sof_qtimer));
                latest = std::max(latest, std::uint64_t(frame.timing.sof_qtimer));
            }
            if (!earliest || latest - earliest > 19200) { have_previous_ = false; period_count_ = 0; return false; }
            if (!have_previous_) { restart_history(frames); return false; }
            std::array<std::uint32_t, 4> periods{};
            for (unsigned i = 0; i < 4; ++i) {
                const auto &m = frames[i].timing;
                if (frames[i].image.sequence - previous_sequence_[i] != 1 ||
                    m.sof_qtimer <= previous_sof_[i] || m.sof_qtimer - previous_sof_[i] > 19200000) {
                    restart_history(frames); return false;
                }
                periods[i] = m.sof_qtimer - previous_sof_[i];
                if (period_count_) {
                    const auto reference = interval_history_[i][0];
                    const auto delta = periods[i] > reference ? periods[i] - reference : reference - periods[i];
                    if (delta > reference / 100) { restart_history(frames); return false; }
                }
            }
            const auto bounds = std::minmax_element(periods.begin(), periods.end());
            if (!*bounds.first || *bounds.second - *bounds.first > 19200) { restart_history(frames); return false; }
            for (unsigned i = 0; i < 4; ++i) interval_history_[i][period_count_] = periods[i];
            remember(frames);
            if (++period_count_ < 8) return false;
            // Nine consecutive, matched hardware frames establish eight
            // stable intervals after the gate transition. The kernel checks
            // its own recent SOF history too; a rejected arm fails this run.
            for (unsigned i = 0; i < 4; ++i) {
                std::uint64_t sum = 0;
                for (auto ticks : interval_history_[i]) sum += ticks;
                periods[i] = sum / 8;
                qcom_camss_exposure_pattern config{};
                config.version = QCOM_CAMSS_EXPOSURE_PATTERN_VERSION;
                config.enable = 1;
                config.anchor_sof_qtimer = frames[i].timing.sof_qtimer;
                config.period_qtimer = periods[i];
                config.steps[0] = {2, 32, 690, 0};
                config.steps[1] = {2, 32, 690, 0};
                config.steps[2] = {phase2_lines_, phase2_gain_, 690, 0};
                const int rc = operation(i, config);
                if (rc) {
                    failed_ = true;
                    std::cout << "stage=camera_pattern_arm_failed slot=" << i << " rc=" << rc << std::endl;
                    disable_armed();
                    throw std::runtime_error("Kernel exposure pattern arm failed; stopping all camera streams");
                }
                armed_mask_ |= 1u << i;
                anchors_[i] = config.anchor_sof_qtimer;
                std::cout << "stage=camera_pattern_arm slot=" << i
                          << " anchor_sof_qtimer=" << anchors_[i] << " measured_period_qtimer=" << periods[i]
                          << " observed_sof_frames=9 phase_at_anchor=0 optical_phase_verified=0" << std::endl;
            }
            return false;
        }
        for (unsigned i = 0; i < 4; ++i) {
            const auto &m = frames[i].timing;
            if (!valid_pattern_exposure(m) || m.phase_anchor_sof_qtimer != anchors_[i] ||
                m.exposure_phase != frames[0].timing.exposure_phase || m.frame_length_lines != 690)
                return false;
            const auto expected_lines = m.exposure_phase == 2 ? phase2_lines_ : 2;
            const auto expected_gain = m.exposure_phase == 2 ? phase2_gain_ : 32;
            if (m.exposure_lines != expected_lines || m.analogue_gain != expected_gain ||
                m.exposure_ns != std::uint64_t(expected_lines) * 928 * 1000000000ULL / 48000000)
                return false;
        }
        return true;
    }
private:
    void disable_armed() noexcept
    {
        for (unsigned i = 0; i < 4; ++i) if (armed_mask_ & (1u << i)) {
            qcom_camss_exposure_pattern config{};
            config.version = QCOM_CAMSS_EXPOSURE_PATTERN_VERSION;
            const int rc = operation(i, config);
            std::cout << "stage=camera_pattern_disable slot=" << i << " rc=" << rc << std::endl;
            if (!rc) armed_mask_ &= ~(1u << i);
        }
    }
    static std::uint32_t positive_env(const char *name)
    {
        const char *value = std::getenv(name);
        char *end = nullptr;
        errno = 0;
        const auto n = value ? std::strtoul(value, &end, 10) : 0;
        if (errno || !value || !*value || !end || *end || !n || n > UINT32_MAX)
            throw std::runtime_error(std::string("Invalid pattern control: ") + name);
        return n;
    }
    int operation(unsigned slot, qcom_camss_exposure_pattern &config) noexcept
    {
        const int fd = ::open(nodes_[slot].c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) return -errno;
        int rc;
        do { rc = ioctl(fd, VIDIOC_QCOM_CAMSS_EXPOSURE_PATTERN, &config); } while (rc < 0 && errno == EINTR);
        const int status = rc < 0 ? -errno : 0;
        ::close(fd);
        return status;
    }
    void restart_history(const std::array<TimedFrame, 4> &frames)
    {
        period_count_ = 0;
        remember(frames);
    }
    void remember(const std::array<TimedFrame, 4> &frames)
    {
        for (unsigned i = 0; i < 4; ++i) {
            previous_sof_[i] = frames[i].timing.sof_qtimer;
            previous_sequence_[i] = frames[i].image.sequence;
        }
        have_previous_ = true;
    }
    bool enabled_{}, have_previous_{}, failed_{};
    unsigned armed_mask_{}, period_count_{};
    std::array<std::array<std::uint32_t, 8>, 4> interval_history_{};
    std::uint32_t phase2_lines_{}, phase2_gain_{};
    std::array<std::string, 4> nodes_;
    std::array<std::uint64_t, 4> previous_sof_{}, anchors_{};
    std::array<std::uint32_t, 4> previous_sequence_{};
};
} // namespace a081::stock::camera
