// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace a081::stock::camera {
struct StockY8Time {
    std::uint64_t exposure_start_boottime_ns{};
    std::uint64_t submit_boottime_ns{};
};
struct StockY8NativeTime {
    std::uint64_t exposure_start_qtimer_ns{}, submit_qtimer_ns{};
};
// Same conversion as original SLPI Android104 in VR mode, using the raw
// 19.2 MHz hardware epoch. Check representation, not an elapsed-time limit.
inline bool qtimer_nanoseconds(std::uint64_t ticks,std::uint64_t &out)
{
    // Quotient/remainder form is exact on both ARM32 and ARM64.
    const auto whole=ticks/12, fraction=(ticks%12)*625/12;
    if(whole>(UINT64_MAX-fraction)/625) return false;
    out=whole*625+fraction;return true;
}
// Map a real native sample epoch with this same frame's measured clock tuple.
// The returned sample may be on either side of the camera query. No arrival
// timestamp, fixed age cap or interpolation participates in this conversion.
inline bool native_sample_boottime(std::uint64_t sample_qtimer_ns,
                                   std::uint64_t frame_sample_qtimer_ticks,
                                   std::uint64_t frame_sample_boottime_ns,
                                   std::uint64_t &out)
{
    std::uint64_t anchor_ns{};
    if(!sample_qtimer_ns || !frame_sample_qtimer_ticks || !frame_sample_boottime_ns ||
        !qtimer_nanoseconds(frame_sample_qtimer_ticks,anchor_ns)) return false;
    if(sample_qtimer_ns>=anchor_ns) {
        const auto delta=sample_qtimer_ns-anchor_ns;
        if(frame_sample_boottime_ns>UINT64_MAX-delta) return false;
        out=frame_sample_boottime_ns+delta;
    } else {
        const auto delta=anchor_ns-sample_qtimer_ns;
        if(frame_sample_boottime_ns<=delta) return false;
        out=frame_sample_boottime_ns-delta;
    }
    return true;
}
inline bool stock_a081_y8_epoch(std::uint64_t sof_ns,std::uint64_t exposure_ns,
                                std::uint64_t &start,std::uint64_t &submit)
{
    constexpr std::uint64_t line_ns=19335;
    if(!exposure_ns || sof_ns<=exposure_ns || sof_ns-exposure_ns<=line_ns) return false;
    start=sof_ns-exposure_ns-line_ns;
    submit=start+exposure_ns/2;
    return true;
}

// Original A081 CamX software convention, camera-input-contract.json:
// SensorNode subtracts same-request exposure, rounded static-mode line time,
// and optional ADCReadoutTime; these OV7251 modules have line=19335ns, ADC=0.
// HAL3 Y8 then selects skew=0 and QVR adds integer exposure/2. This is NOT an
// independent measurement of the current sensor's optical exposure offset.
//
// Caller must supply the actual image-associated SOF in BOOTTIME and exposure
// effective for that frame. Never substitute EOF, IRQ notification/receipt,
// requested-but-not-yet-effective controls, or a frame sequence from another
// stream. Do not recompute the stock line correction from external FSIN FPS.
inline bool stock_a081_y8_time(std::uint64_t sof_boottime_ns,
                              std::uint64_t effective_exposure_ns,
                              StockY8Time &out)
{
    // Preserve subtraction order: odd E cannot be simplified to S-floor(E/2).
    return stock_a081_y8_epoch(sof_boottime_ns,effective_exposure_ns,
                               out.exposure_start_boottime_ns,out.submit_boottime_ns);
}
inline bool stock_a081_y8_native_time(std::uint64_t sof_qtimer_ticks,
                                     std::uint64_t effective_exposure_ns,
                                     StockY8NativeTime &out)
{
    std::uint64_t sof_ns{};
    if(!sof_qtimer_ticks || !qtimer_nanoseconds(sof_qtimer_ticks,sof_ns)) return false;
    return stock_a081_y8_epoch(sof_ns,effective_exposure_ns,
                               out.exposure_start_qtimer_ns,out.submit_qtimer_ns);
}
} // namespace a081::stock::camera
