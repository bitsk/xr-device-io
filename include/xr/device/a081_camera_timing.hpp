// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace a081::stock::camera {
struct StockY8Time {
    std::uint64_t exposure_start_boottime_ns{};
    std::uint64_t submit_boottime_ns{};
};

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
    constexpr std::uint64_t stock_line_correction_ns = 19335;
    if (!effective_exposure_ns || sof_boottime_ns <= effective_exposure_ns ||
        sof_boottime_ns - effective_exposure_ns <= stock_line_correction_ns)
        return false;
    const auto start = sof_boottime_ns - effective_exposure_ns - stock_line_correction_ns;
    // Preserve subtraction order: odd E cannot be simplified to S-floor(E/2).
    out = {start, start + effective_exposure_ns / 2};
    return true;
}
} // namespace a081::stock::camera
