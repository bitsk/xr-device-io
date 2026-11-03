// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <xr/device/a081_camera_metadata.hpp>
#include <xr/device/a081_camera_pattern.hpp>
#include <array>
#include <deque>
#include <functional>

namespace a081::stock::camera {
struct FrameGroup {
    std::array<TimedFrame, 4> frames;
    std::uint64_t submit_boottime_ns{};
};

// Live, verified QCFT/image pairs only. Sequence numbers are independent
// across cameras. Use the stock1ms stereo tolerance with fixed equal exposure
// and gain; never manufacture a missing view or retime a stale image.
class FrameGroups {
public:
    explicit FrameGroups(std::uint64_t exposure_ns,
                         std::function<void(const FrameGroup &)> ready = {})
        : ready_callback_(std::move(ready)), exposure_ns_(exposure_ns) {}
    void gate_high(std::uint64_t after_boottime_ns)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (earliest_ || !after_boottime_ns) throw std::runtime_error("Duplicate/invalid physical camera gate event");
        // Ignore startup and the real gate transition before admitting VIO.
        earliest_ = after_boottime_ns + 200000000ULL;
    }
    void push(unsigned slot, TimedFrame &&frame)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (slot >= 4) throw std::runtime_error("Unknown camera role");
        if (!earliest_ || frame.timing.sof_boottime_ns < earliest_) { ++startup_discard_; return; }
        auto &queue = pending_[slot];
        queue.push_back(std::move(frame));
        if (queue.size() > 4) { queue.pop_front(); ++unmatched_; }
        while (std::all_of(pending_.begin(), pending_.end(), [](const auto &q) { return !q.empty(); })) {
            std::uint64_t latest = 0;
            for (auto &q : pending_) latest = std::max(latest, std::uint64_t(q.front().timing.sof_boottime_ns));
            bool aligned = true;
            for (auto &q : pending_) {
                if (latest - q.front().timing.sof_boottime_ns > 1000000ULL) {
                    q.pop_front(); ++unmatched_; aligned = false;
                }
            }
            if (!aligned) continue;
            FrameGroup group;
            for (unsigned i = 0; i < 4; ++i) { group.frames[i] = std::move(pending_[i].front()); pending_[i].pop_front(); }
            if (!pattern_.admit(group.frames)) { ++exposure_discard_; continue; }
            const auto exposure = frame_exposure_ns(group.frames[0].timing, exposure_ns_);
            if (!exposure) { ++exposure_discard_; continue; }
            StockY8Time time{};
            if (!stock_a081_y8_time(group.frames[0].timing.sof_boottime_ns, exposure, time))
                throw std::runtime_error("Invalid real camera exposure timing");
            group.submit_boottime_ns = time.submit_boottime_ns;
            // The controller tee consumes each verified group before the
            // head RPC queue can discard it. push() serializes this callback.
            if (ready_callback_) ready_callback_(group);
            ready_.push_back(std::move(group)); ++prepared_;
            // Bound latency and memory if original RPC processing is slower
            // than the camera. Report these actual drops; never replay them.
            if (ready_.size() > 2) { ready_.pop_front(); ++backpressure_drop_; }
        }
    }
    bool take(std::uint64_t latest_imu_ns, FrameGroup &out)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (ready_.empty() || ready_.front().submit_boottime_ns > latest_imu_ns) return false;
        out = std::move(ready_.front()); ready_.pop_front(); return true;
    }
    bool empty()
    {
        std::lock_guard<std::mutex> guard(mutex_); return ready_.empty();
    }
    void summary()
    {
        std::lock_guard<std::mutex> guard(mutex_);
        std::cout << "stage=camera_group_summary prepared=" << prepared_ << " startup_discard=" << startup_discard_
                  << " exposure_discard=" << exposure_discard_ << " unmatched_views=" << unmatched_ << " backpressure_drop=" << backpressure_drop_
                  << " queued=" << ready_.size() << " tolerance_ns=1000000 same_sequence_required=0" << std::endl;
    }
private:
    ExposurePattern pattern_;
    std::mutex mutex_;
    std::array<std::deque<TimedFrame>, 4> pending_;
    std::deque<FrameGroup> ready_;
    std::function<void(const FrameGroup &)> ready_callback_;
    std::uint64_t exposure_ns_, earliest_{}, prepared_{}, startup_discard_{}, unmatched_{}, backpressure_drop_{}, exposure_discard_{};
};
} // namespace a081::stock::camera
