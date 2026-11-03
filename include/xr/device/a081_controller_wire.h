#ifndef A081_CONTROLLER_WIRE_H
#define A081_CONTROLLER_WIRE_H

#include <xr/device/a081_controller_abi.h>

#include <float.h>
#include <stddef.h>
#include <string.h>

/* The wire contains A5 5A followed by 34 META-encoded bytes. This decoder
 * accepts actual transport frames; it does not construct sensor samples.
 * Evidence: startup-contract.md and input-contract.md. All p[] offsets below
 * refer to the decoded 34-byte payload, not the whole SPI transaction.
 */
struct a081_controller_wire_packet {
    uint8_t payload[34];
    unsigned type;
    enum a081_controller_hand hand;
};

enum a081_controller_wire_result {
    A081_WIRE_OK,
    A081_WIRE_HEADER,
    A081_WIRE_CHECKSUM,
    A081_WIRE_TYPE
};

static inline enum a081_controller_wire_result
a081_controller_decode_wire(const uint8_t raw[36],
                            struct a081_controller_wire_packet *packet)
{
    if (raw[0] != 0xa5 || raw[1] != 0x5a)
        return A081_WIRE_HEADER;
    memcpy(packet->payload, raw + 2, sizeof(packet->payload));
    uint8_t *p = packet->payload;
    const uint8_t key = p[1];
    p[0] ^= key;
    for (size_t i = 2; i < sizeof(packet->payload); ++i)
        p[i] ^= key;
    packet->type = (p[0] >> 4) & 7;
    packet->hand = (enum a081_controller_hand)(p[0] >> 7);
    /* Stock PackageType uses a different check for each wire type. Its
     * CheckSyncPackage (type 4) returns true without inspecting a checksum.
     * Types 5/6 end their checked payload at p[29]; type 3 has no parser.
     */
    size_t checksum_index;
    switch (packet->type) {
    case 0: case 1: case 2: case 7:
        checksum_index = 33;
        break;
    case 4:
        return A081_WIRE_OK;
    case 5: case 6:
        checksum_index = 29;
        break;
    default:
        return A081_WIRE_TYPE;
    }
    unsigned sum = 0;
    for (size_t i = 0; i < checksum_index; ++i)
        sum += p[i]; /* p[1] remains the original key and IS in the sum. */
    if ((uint8_t)sum != p[checksum_index])
        return A081_WIRE_CHECKSUM;
    return A081_WIRE_OK;
}

static inline uint16_t a081_controller_le16(const uint8_t *p)
{
    return (uint16_t)((unsigned)p[0] | ((unsigned)p[1] << 8));
}

static inline uint32_t a081_controller_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Scaling does not establish clock synchronization. The caller must verify the
 * original radio/camera synchronization before treating this as BOOTTIME ns.
 * version_third is p[22] from a verified type-2 head PatternID packet for THIS
 * hand, not a guessed handset firmware version or the SDK fusion selector.
 */
static inline int
a081_controller_unpack_type7(const struct a081_controller_wire_packet *packet,
                            uint8_t version_third,
                            struct a081_controller_imu_input *out,
                            uint64_t *wire_clock)
{
    _Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
                   "The stock payload uses IEEE754 binary32");
    if (packet->type != 7)
        return 0;
    const uint8_t *p = packet->payload;
    for (size_t i = 0; i < 4; ++i) {
        const unsigned bits = a081_controller_le16(p + 7 + i * 2);
        const int value = bits < 0x8000 ? (int)bits : (int)bits - 65536;
        out->values[i] = (double)value / 10000.0;
    }
    for (size_t i = 0; i < 3; ++i) {
        const uint32_t bits = a081_controller_le32(p + 15 + i * 4);
        float value;
        memcpy(&value, &bits, sizeof(value));
        out->values[4 + i] = value;
    }
    out->sideband[0] = p[2] >> 7;
    out->sideband[1] = p[31];
    out->sideband[2] = p[32];
    *wire_clock = (uint64_t)a081_controller_le32(p + 3) |
                  ((uint64_t)a081_controller_le16(p + 29) << 32);
    out->aligned_time_ns = *wire_clock * (version_third < 25 ? 30500 : 1000);
    return 1;
}

#endif
