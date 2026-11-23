// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <stdint.h>

#define A081_HAPTIC_MAGIC UINT32_C(0x31485641)
#define A081_HAPTIC_MAX_AGE_NS UINT64_C(100000000)
#define A081_HAPTIC_MIN_MS 10
#define A081_HAPTIC_MAX_MS 1000
#define A081_HAPTIC_MAX_STRENGTH 15

/* One request from the current graphics session. A zero strength is a stop. */
struct a081_haptic_request
{
    uint64_t monotonic_ns;
    uint32_t magic;
    uint16_t duration_ms;
    uint8_t hand;
    uint8_t strength;
};

#ifdef __cplusplus
static_assert(sizeof(a081_haptic_request) == 16);
#endif
