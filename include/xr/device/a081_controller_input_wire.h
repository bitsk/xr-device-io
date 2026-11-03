#ifndef A081_CONTROLLER_INPUT_WIRE_H
#define A081_CONTROLLER_INPUT_WIRE_H

#include <stdint.h>

/* Local little-endian ARM64 -> ARM32 SOCK_SEQPACKET interface. Only the root
 * capture/radio owner may connect. Pixels are real Y8 camera mosaics in one
 * sealed memfd passed with SCM_RIGHTS; neither process sends pointers. */
#define A081_CONTROLLER_INPUT_MAGIC UINT32_C(0x43494e41)
#define A081_CONTROLLER_INPUT_VERSION 3u
#define A081_CONTROLLER_INPUT_ABI_DESCRIPTOR "A081_CONTROLLER_INPUT_ABI:v3:radio88:frame216:timestamp80"
/* Retained in each producer/worker ELF so packaging can reject mixed ABIs. */
#if defined(__GNUC__)
__attribute__((used)) static const char a081_controller_input_abi_descriptor[] = A081_CONTROLLER_INPUT_ABI_DESCRIPTOR;
#endif
enum { A081_CONTROLLER_INPUT_RADIO = 1, A081_CONTROLLER_INPUT_FRAME = 2,
       A081_CONTROLLER_OUTPUT_TIMESTAMP_REQUEST = 3 };
enum {
    A081_CONTROLLER_FRAME_REAL_QCFT_PAIR = 1,
    A081_CONTROLLER_FRAME_STOCK_CAMERA_ORDER = 2,
    /* Optional: without this bit only camera/radio clock processing is valid;
     * all HMD fields are zero and SDK image fusion must reject the frame. */
    A081_CONTROLLER_FRAME_RAW_QVR_POSE = 4,
    /* Kernel SOF-associated schedule validity; not physical LED acceptance. */
    A081_CONTROLLER_FRAME_KERNEL_EXPOSURE_PHASE = 8,
    A081_CONTROLLER_FRAME_REQUIRED = 15
};

struct a081_controller_input_header {
    uint32_t magic, version, bytes, type;
    uint64_t sequence, sent_monotonic_ns;
};

struct a081_controller_radio_input {
    struct a081_controller_input_header header;
    uint64_t irq_monotonic_ns, completion_monotonic_ns;
    uint8_t raw[36];
    uint32_t reserved;
};

struct a081_controller_frame_input {
    struct a081_controller_input_header header;
    uint32_t view_width, height, stride_bytes;
    uint32_t tracking_frame_id, ctrl_frame_id, flags;
    uint64_t tracking_sof_boottime_ns, ctrl_sof_boottime_ns;
    /* Version2 retains raw QCFT SOF and carries the stock exposure-start time. */
    uint64_t ctrl_exposure_start_boottime_ns, ctrl_exposure_ns;
    uint64_t hmd_pose_boottime_ns, camera_clock_bracket_ns;
    double hmd_position[3], hmd_orientation_xyzw[4];
    uint32_t mosaic_bytes[2], mosaic_offsets[2];
    /* Reserved unless an authoritative state source exists. Zero means the
     * helper derives state5/7 from actual type7 just as the stock parser does. */
    uint32_t stock_hand_state[2];
    uint32_t ctrl_sensor_sequence, ctrl_exposure_phase;
    uint64_t ctrl_phase_anchor_sof_qtimer, ctrl_control_apply_sof_qtimer;
    uint64_t ctrl_control_applied_boottime_ns;
};

/* Sent back only to the connected radio owner after a real camera frame.
 * The radio owner applies its verified per-hand PatternID version and actual
 * stock SPI command framing. This is not a request to pair or write firmware. */
struct a081_controller_timestamp_request {
    struct a081_controller_input_header header;
    uint64_t ctrl_sof_boottime_ns, ctrl_exposure_start_boottime_ns, ctrl_exposure_ns;
    uint32_t ctrl_frame_id, hand;
    uint32_t exposure_phase, reserved;
    uint64_t phase_anchor_sof_qtimer;
};

#ifdef __cplusplus
static_assert(sizeof(a081_controller_input_header) == 32, "input header ABI");
static_assert(sizeof(a081_controller_radio_input) == 88, "radio input ABI");
static_assert(sizeof(a081_controller_frame_input) == 216, "frame input ABI");
static_assert(sizeof(a081_controller_timestamp_request) == 80, "timestamp request ABI");
#endif
#endif
