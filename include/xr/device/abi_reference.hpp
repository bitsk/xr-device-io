// Offline, independently written reference for the A081 stock AP wire ABI.
// See evidence-index.json and README.md. This does not invoke a DSP or hardware.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace a081::head {

using Bytes = std::vector<std::uint8_t>;
template <std::size_t N> using Opaque = std::array<std::uint8_t, N>;

// These are intentionally distinct: equal size does not establish equal ABI.
using TrackerImu64 = Opaque<64>;       // tracker_6dof_imuupdate payload, fields unresolved here.
using TrackerState156 = Opaque<156>;   // In/out tracker state, NOT the public 192-byte pose.
using TrackerFds72 = Opaque<72>;
using CameraId64 = Opaque<64>;         // NUL-terminated calibration name in QVR service.

// Owned wire buffers, in FastRPC order: all inputs, then all outputs. A transport
// adapter must translate these to its own ABI; never send host pointers as data.
struct Invocation {
    std::uint32_t scalar{};
    std::vector<Bytes> inputs;
    std::vector<Bytes> outputs;
};

constexpr std::uint32_t scalar(std::uint32_t method, std::uint32_t in,
                               std::uint32_t out = 0)
{
    return ((method & 31u) << 24) | ((in & 255u) << 16) | ((out & 255u) << 8);
}

// Distinct module handles / method namespaces. Methods >=31 use a selector
// word in input primitive buffer 0; do not shift logical method 40 into scalar.
namespace tracker_method {
constexpr auto device_init = scalar(2, 4, 1);
constexpr auto set_fds = scalar(3, 1, 1);
constexpr auto imu_update = scalar(6, 2, 1);
constexpr auto get_version = scalar(9, 2, 2);
constexpr auto calibration = scalar(17, 2, 1);
}
namespace qvr_method {
constexpr auto init = scalar(2, 0);
constexpr auto start = scalar(3, 6);
constexpr auto stop = scalar(4, 0);
constexpr auto forward_block = scalar(6, 1);
constexpr auto camera_frame = scalar(7, 4);
constexpr auto tracking_mode = scalar(8, 1);
constexpr auto qtimer = scalar(9, 0, 1);
constexpr auto external_sensor_ready = scalar(10, 0);
constexpr auto ring_layout = scalar(12, 2);
constexpr auto dsp_clock = scalar(20, 0, 1);
constexpr auto camera_buffers = scalar(25, 3);
constexpr auto external_sensor_format = scalar(31, 1); // prim[0] =31, prim size28.
constexpr auto start_imu_pose = scalar(31, 4);         // prim[0] =40, prim size40.
constexpr auto stop_imu_pose = scalar(31, 1);          // prim[0] =41, prim size4.
}
namespace camera_method {
constexpr auto init = scalar(0, 0);
constexpr auto qtimer = scalar(4, 0, 1);
constexpr auto notify_frame = scalar(5, 1);
constexpr auto add_dsp_client = scalar(7, 1);
constexpr auto create_client = scalar(10, 2);
constexpr auto create_camera = scalar(12, 2);
constexpr auto frame_buffer_fd = scalar(14, 2);
constexpr auto ring_layout = scalar(16, 2);
constexpr auto sync_clock = scalar(21, 1);
}

// QVR's internal IDs, not QVRSERVICE_RING_BUFFER_ID in the public SDK.
enum class RingId : std::uint32_t {
    raw_gyro = 0, pose = 1, debug_6dof = 4, raw_accel = 5,
    health = 7, predicted_pose = 8, frame_pose = 23, dl_prediction = 29,
};

// GetSharedMemLayout writes 32 bytes. SetRingBufferLayout sends words 0..6.
// type_tag occupies the first field called "reserved" in the public SDK.
struct RingDescriptor {
    std::int32_t fd;
    std::uint32_t block_bytes;
    std::uint32_t index_offset;
    std::uint32_t ring_offset;
    std::uint32_t element_bytes;
    std::uint32_t count;
    std::uint32_t type_tag;
    std::uint32_t reserved;
};
static_assert(sizeof(RingDescriptor) == 32);

struct RingShape {
    RingId id;
    std::uint32_t header_bytes;
    std::uint32_t element_bytes;
    std::uint32_t count;
    std::uint32_t type_tag;
};
// Static descriptors decoded from libqvrservice.so, not measured allocations.
constexpr RingShape pose_ring{RingId::pose, 64, 192, 80, 0};
constexpr RingShape predicted_pose_ring{RingId::predicted_pose, 64, 192, 8, 8};
constexpr RingShape frame_pose_ring{RingId::frame_pose, 64, 64, 80, 10};
constexpr RingShape dl_prediction_ring{RingId::dl_prediction, 64, 128, 80, 15};

// The stock external-sensor callback caches accelerometer samples and publishes
// one record on each gyro sample. This describes that shared ring only.
struct ExternalSensorRecord64 {
    std::uint64_t gyro_time_ns;   // +0: copied verbatim from callback timestamp.
    std::uint64_t accel_time_ns;  // +8: most recently cached accel timestamp.
    float gyro[3];               // +16; SDK callback specifies rad/s.
    float accel[3];              // +28; SDK callback specifies m/s^2.
    std::uint8_t zeroed[20];      // +40..59: zero stores in this writer.
    std::uint8_t unwritten[4];    // +60..63: writer does not set these bytes.
};
static_assert(sizeof(ExternalSensorRecord64) == 64);
static_assert(offsetof(ExternalSensorRecord64, gyro) == 16);
static_assert(offsetof(ExternalSensorRecord64, accel) == 28);

// ARM32 tracker cameraupdate has 96-byte host records. The pointer at +16 is
// extracted into an individual FastRPC input buffer; wire records are88 bytes.
// Semantics of prefix/tail remain deliberately unspecified.
struct TrackerCameraFrame {
    Opaque<16> native_prefix;
    Bytes payload;                  // native pointer +16 and length +20.
    std::uint32_t native_word24;
    Opaque<64> native_bytes28_91;
    // Native +92..95 padding is not copied to the wire record.
};

CameraId64 camera_id(std::string_view name);
Invocation tracker_device_init(const Bytes &blob_a, const Bytes &blob_b,
                               const std::vector<CameraId64> &ids);
Invocation tracker_set_fds(const TrackerFds72 &fds);
Invocation tracker_imu_update(const std::vector<TrackerImu64> &samples,
                              const TrackerState156 &state);
Invocation tracker_camera_update(std::uint64_t primitive_word,
                                 const std::vector<TrackerCameraFrame> &frames,
                                 const TrackerState156 &state);
// Only use after transport success. Remote result is separate from transport
// status. The stock stub copies state even when the remote result is nonzero.
std::int32_t decode_tracker_result(const Bytes &out, TrackerState156 &state);

// Text lengths include the trailing NUL. flags is NOT the tracking mode enum.
// Tail floats are sensor/magnetometer requested Hz; mode1 does not consume
// them. See system/tracking/head/sensor-rate-contract.json.
Invocation qvr_start(std::string_view config_6dof, std::string_view calibration,
                     std::string_view config_3dof, std::uint32_t flags,
                     const std::vector<CameraId64> &ids,
                     const std::vector<float> &prediction_coefficients,
                     float final_float_a, float final_float_b);
Invocation qvr_set_ring_layout(RingId id, const RingDescriptor &desc);
Invocation qvr_set_camera_buffers(const std::vector<std::int32_t> &fds,
                                  const std::vector<std::uint32_t> &sizes);
Invocation qvr_camera_frame(std::uint32_t width, std::uint32_t height,
                            std::uint64_t time_ns, const Bytes &primary,
                            const Bytes &secondary, const std::vector<CameraId64> &ids,
                            std::int32_t frame_number, std::int32_t aux_frame_number);
Invocation qvr_tracking_mode(std::uint32_t dsp_mode); // positional=1; rotational=0; mag=2.
Invocation qvr_stop_imu_pose();

// AP service uses sensors_timeutil::qtimer_get_time_ns, not raw DSP clock ticks.
// Capture this offset for the current boot. CLOCK_MONOTONIC is a separate base.
std::int64_t android_to_qtimer_offset(std::uint64_t qtimer_ns, std::uint64_t boottime_ns);
std::uint64_t boottime_to_qtimer(std::uint64_t boottime_ns, std::int64_t offset_ns);

namespace sdk_4_0_6 {
// Reference layout from the separately supplied public QVRTypes.h. The stock
// service confirms192-byte pose slots; all field semantics have NOT been
// independently verified against the A081 DSP producer. Do not cast state156.
struct HeadPose192 {
    float rotation_xyzw[4];
    float translation[3];
    std::uint32_t reserved0;
    std::uint64_t timestamp;
    std::uint64_t reserved1;
    float angular_s[3]; std::uint32_t reserved2;
    float angular_b[3]; std::uint32_t reserved3;
    float angular_bdt[3]; std::uint32_t reserved4;
    float angular_bdt2[3];
    std::uint16_t tracking_state;
    std::uint16_t warnings;
    std::uint32_t magnetometer_flags;
    float pose_quality;
    float sensor_quality;
    float camera_quality;
    float translation_s[3]; std::uint32_t reserved6;
    float translation_b[3]; std::uint32_t reserved7;
    std::uint8_t reserved8[32];
};
static_assert(sizeof(HeadPose192) == 192);
static_assert(offsetof(HeadPose192, timestamp) == 32);
static_assert(offsetof(HeadPose192, tracking_state) == 108);
static_assert(offsetof(HeadPose192, pose_quality) == 116);
static_assert(offsetof(HeadPose192, translation_s) == 128);
} // namespace sdk_4_0_6
} // namespace a081::head
