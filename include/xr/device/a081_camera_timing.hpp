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
    const auto ns=(static_cast<unsigned __int128>(ticks)*625)/12;
    if(ns>UINT64_MAX) return false;
    out=static_cast<std::uint64_t>(ns);return true;
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
