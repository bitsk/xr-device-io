#ifndef A081_RADIO_CLOCK_REFERENCE_H
#define A081_RADIO_CLOCK_REFERENCE_H

#include <xr/device/a081_controller_abi.h>

/* Pure arithmetic/packing recovered from the pinned A081 stock binaries.
 * No device IO, TX, clock mutation, sensor generation, or ready flag.
 * See radio-clock-contract.json for evidence and unresolved firmware behavior.
 */

struct a081_camera_time_body {
    uint8_t bytes[12];
    uint64_t encoded_time;
    enum a081_controller_hand hand;
};

/* Original JNI phases select right, left, then no timestamp command.
 * Linux uses the explicit kernel SOF-associated exposure phase. Independent
 * V4L2 image sequences cannot establish that phase or physical LED alignment.
 */
static inline int a081_radio_hand_for_exposure_phase(uint32_t exposure_phase)
{
    switch (exposure_phase) {
    case 0: return A081_HAND_RIGHT;
    case 1: return A081_HAND_LEFT;
    default: return -1;
    }
}

/* head_version_third must come from a checksum-verified type-2 packet for
 * version_hand: decoded payload[22]. ctrl_exposure_start_boottime_ns must be
 * the actual CTRL exposure start derived from QCFT and effective exposure,
 * mapped to BOOTTIME, never an invented/current wall time.
 * The hand match and nonzero time checks are adapter prerequisites; the byte
 * encoding and floating conversions below reproduce the original call chain.
 * Only the 12-byte body is recovered. Do not extend it to a 36-byte SPI transfer
 * with guessed padding: the original Write ignores its length and copies 36
 * bytes from the caller's stack, whose final 24 bytes are not a defined pad.
 */
static inline int a081_radio_pack_camera_time_body(
    uint32_t exposure_phase, uint64_t ctrl_exposure_start_boottime_ns,
    enum a081_controller_hand version_hand, uint8_t head_version_third,
    struct a081_camera_time_body *out)
{
    int hand = a081_radio_hand_for_exposure_phase(exposure_phase);
    if (!out || hand < 0 || (int)version_hand != hand || !ctrl_exposure_start_boottime_ns)
        return 0;
    uint64_t value = (uint64_t)((double)ctrl_exposure_start_boottime_ns * 0.001);
    if (head_version_third < 25)
        value = (uint64_t)((double)value / 30.5);
    out->hand = (enum a081_controller_hand)hand;
    out->encoded_time = value;
    out->bytes[0] = 0xa5;
    out->bytes[1] = 12;
    out->bytes[2] = hand == A081_HAND_RIGHT ? 0x12 : 0x11;
    unsigned sum = out->bytes[1] + out->bytes[2];
    for (unsigned i = 0; i < 8; ++i) {
        out->bytes[3 + i] = (uint8_t)(value >> (i * 8));
        sum += out->bytes[3 + i];
    }
    out->bytes[11] = (uint8_t)sum;
    return 1;
}

/* type-7 calls the tracking callback with this scaled value directly. It does
 * not call LR::Inference or ConvertIMUToSys. In the pinned build both
 * TimeCalibrator::Avaliable and LR::Available return 1 unconditionally.
 * This conversion alone makes NO assertion that the sample is synchronized.
 */
static inline int a081_radio_scale_type7_time(uint64_t wire_time48,
                                           uint8_t head_version_third,
                                           uint64_t *sample_ns)
{
    if (!sample_ns || wire_time48 > UINT64_C(0xffffffffffff))
        return 0;
    *sample_ns = wire_time48 * (head_version_third < 25 ? UINT64_C(30500)
                                                         : UINT64_C(1000));
    return 1;
}

/* This is only the exact positive-sample/delay predicate in the original
 * parser. Callers must separately check actual clock-domain uncertainty,
 * header/META/checksum, version, quaternion, duplicate/backwards samples and
 * callback/lifecycle conditions. A match is neither a handshake ACK nor pose
 * validity. No receive-time replacement or learned affine fit is applied.
 */
static inline int a081_radio_stock_time_window(uint64_t sample_ns,
                                             uint64_t receive_boottime_ns,
                                             uint64_t *positive_delay_ns)
{
    if (!sample_ns || receive_boottime_ns <= sample_ns)
        return 0;
    uint64_t delay = receive_boottime_ns - sample_ns;
    if (positive_delay_ns)
        *positive_delay_ns = delay;
    return delay < UINT64_C(33000000);
}

#endif
