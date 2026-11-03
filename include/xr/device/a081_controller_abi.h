#ifndef A081_CONTROLLER_ABI_H
#define A081_CONTROLLER_ABI_H

#include <stdint.h>

/* Pinned to the A081 ControllerCoreService ARM32 SDK; not a public vendor SDK.
 * Evidence and unresolved semantics are recorded in README.md. ELF offsets below
 * have the Thumb bit cleared. A live function pointer must KEEP its Thumb bit.
 */
#define A081_CONTROLLER_SDK_SHA256 \
    "7008b975aa23b92a2cc9a43278b626d44512fbce597e92858e18f13b4609fe6a"
#define A081_CONTROLLER_SDK_BUILD_ID \
    "761b70cc35dc428f8c8e01710bb0198b156ec66c"
#define A081_CONTROLLER_FACTORY_SYMBOL "_Z11getInstance4HANDi"
#define A081_CONTROLLER_FACTORY_OFFSET UINT32_C(0x39122c)
#define A081_CONTROLLER_VTABLE_OFFSET UINT32_C(0x7bc400)

enum a081_controller_hand {
    A081_HAND_LEFT = 0,
    A081_HAND_RIGHT = 1,
    A081_HAND_BOTH = 2
};

/* The exported factory accepts HAND=0 or 2. Original JNI uses (2, 2).
 * A HAND=1 factory request is rejected, even though per-hand methods use 0/1.
 */
typedef void *(*a081_controller_factory_fn)(int hand, int mode);

enum a081_controller_vtable_byte_offset {
    A081_SLOT_INITIAL_MODE = 0x04,
    A081_SLOT_HMD_POSE_INPUT = 0x0c,
    A081_SLOT_IMU_INPUT = 0x18,
    A081_SLOT_IMAGE_INPUT = 0x1c,
    A081_SLOT_POSE_LISTENER = 0x20,
    A081_SLOT_GET_POSE = 0x34,
    A081_SLOT_RIGHT_IMU = 0x5c,
    A081_SLOT_RIGHT_IMAGE = 0x60,
    A081_SLOT_SCHEDULER = 0x6c,
    A081_SLOT_VSYNC = 0x7c
};

/* The method called by JNI immediately after factory creation. The numeric mode
 * is confirmed; calling it an algorithm start/stop API is NOT yet justified.
 */
typedef int (*a081_controller_initial_mode_fn)(void *self, int mode);

/* JNI pushes HMD pose before image input. Unlike controller output, HMD input
 * quaternion is XYZW. SDK performs an internal coordinate transformation;
 * the caller must supply the original QVR coordinate convention, not silently
 * substitute an already converted OpenXR pose. Observed normal return is 1.
 */
typedef int (*a081_controller_push_hmd_pose_fn)(
    void *self, const double position[3], const double quaternion_xyzw[4],
    uint64_t timestamp_ns);

/* ARM AAPCS softfp: the uint64_t argument consumes an aligned register pair or
 * aligned stack pair. Do not manually flatten it to two positional uint32_t's.
 * values = quaternion wxyz then cumulative inertial vector (not position).
 * sideband = status, tick low byte, tick high byte. Time must already be mapped
 * into the SDK system-clock domain; raw device ticks are not accepted here.
 * The observed normal return from slot +0x18 is 0. This does not certify that
 * fusion accepted the sample: internal time/version checks can discard it.
 */
typedef int (*a081_controller_push_imu_fn)(
    void *self, const double values[7], const int16_t sideband[3],
    uint64_t timestamp_ns, int hand);

struct a081_controller_imu_input {
    double values[7];
    int16_t sideband[3];
    uint64_t aligned_time_ns;
};

/* Confirmed call layout at SDK 0x3865b0 / JNI 0x10fa0. All arrays contain double.
 * timestamp_ns is an in/out pointer, not uint64_t passed by value. The four
 * motion-vector meanings/coordinates remain unverified, so retain neutral names.
 * confidence is returned in r0 under Android's ARM softfp ABI.
 */
typedef float (*a081_controller_get_pose_fn)(
    void *self, double position[3], double quaternion_wxyz[4],
    uint64_t *timestamp_ns, double motion_a[3], double motion_b[3],
    double motion_c[3], double motion_d[3], int hand);

/* A live pose also requires optical input, IMU time alignment, HMD pose callback,
 * calibration, and the service lifecycle. Merely calling get_pose is not 6DoF.
 * No factory/global object destructor has been recovered; a probe exits the
 * isolated process with _exit instead of deleting the singleton or dlclosing it.
 */

#endif
