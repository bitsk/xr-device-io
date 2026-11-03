#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Offline reference reconstructed from A081 ControllerCoreService ARM libraries.
// This is neither the vendor SDK ABI nor an OpenXR driver. See README.md for
// evidence, image-base conventions, and the remaining unknowns.
namespace a081::controller {

inline constexpr std::size_t spi_transfer_bytes = 36;
inline constexpr std::uint16_t notification_event_type = 1;   // EV_KEY
inline constexpr std::uint16_t notification_event_code = 190;

struct QuaternionWxyz {
    double w, x, y, z;
};

struct PreintegratedPacket {
    std::uint8_t hand;                 // payload[0] >> 7, not an OpenXR hand enum
    std::uint8_t header_low_nibble;    // meaning not established
    std::uint64_t clock_value_48;      // hardware/firmware clock, NOT XrTime
    QuaternionWxyz orientation;       // four signed int16 / 10000, w,x,y,z
    std::array<double, 3> vector;      // cumulative vector when SDK selector >=20;
                                      // differentiated into acceleration, not position
    std::uint8_t status_bit;           // payload[2] >> 7
    std::uint8_t battery_nibble;       // payload[1] >> 4
    std::uint8_t payload1_low_nibble;  // payload[1] & 0xf; NOT button callback data
    std::uint8_t auxiliary_mode;       // payload[2] & 0xf
    std::array<std::uint8_t, 2> auxiliary_bytes; // payload[27], payload[28]
    std::array<std::int16_t, 3> sideband; // {status_bit, payload[31], payload[32]}
};

// Confirmed active packet path: A5 5A + 34-byte type-7 packet, additive sum8.
// ELF VA 0x123e0 (fields), 0x1229c (sum8), 0x109c0 (SPI/framing).
bool decode_preintegrated_transfer(const std::uint8_t* bytes,
                                  std::size_t size, PreintegratedPacket& output);

struct AuxiliaryUpdate {
    enum class Kind { Unknown, ButtonsFingerAndTouch, Joystick } kind;
    std::int8_t joystick_x, joystick_y;
    // Original serial callback values; JNI remaps hands/axes/bits separately.
    // No physical A/B/menu naming or OpenXR normalization is implied.
    std::uint8_t buttons_raw, touch_raw, middle_finger_raw, index_finger_raw;
};

// The firmware multiplexes auxiliary data; consumers preserve the previous
// values of fields not present in this packet. Nibbles retain vendor scaling.
AuxiliaryUpdate decode_auxiliary(const PreintegratedPacket& packet);

// Confirmed 3DoF preprocessing: previous.conjugate() * current. This stage
// precedes AngleAxis/dt and Mahony correction, it is not a 6DoF estimator.
// ELF VA 0x3a50 of libcontroller3dof.so; input quaternion order checked at 0x3b64.
QuaternionWxyz relative_orientation(QuaternionWxyz previous,
                                    QuaternionWxyz current);

// SDK ELF 0x30ad40: mode field at FusionImpl+0xa7df0 defaults to 20; for >=20
// it differences this cumulative vector, inverse-rotates by previous normalized
// q, and divides by the positive, timestamp/tick-validated interval in ms.
// This is the mathematical step, not the complete clock/version/calibration
// state machine. An already decoded old-mode acceleration must bypass it.
bool recover_body_acceleration(QuaternionWxyz previous_unit_orientation,
                               const std::array<double, 3>& previous_vector,
                               const std::array<double, 3>& current_vector,
                               int validated_delta_ms,
                               std::array<double, 3>& acceleration);

// TimeCalibrator uses an affine fit, y = slope*x + intercept (ELF 0x1356a).
// Fit parameters require measured synchronization pairs. Identity is not a
// verified conversion from the controller clock to Linux/OpenXR time.
struct ClockFit {
    double slope;
    double intercept;
    double to_system(double controller_time) const;
};

} // namespace a081::controller
