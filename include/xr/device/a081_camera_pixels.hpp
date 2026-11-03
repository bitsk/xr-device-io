// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <xr/device/abi_reference.hpp>
#include <cstdint>
#include <cstring>
#include <linux/videodev2.h>

namespace a081::stock::camera {
inline constexpr unsigned width = 640, height = 480;
inline constexpr std::size_t image_bytes = width * height;

// Convert actual V4L2 memory, respecting the negotiated stride. This linear
// 10-bit -> 8-bit projection matches the recovered stock default gamma=1,
// OV7251 black_bias=0 (camera-input-contract.json); it does not emulate other
// gamma/LUT/client overrides. Stock uses RAW16; CAMSS may export Y10P.
// No timestamps, camera identities or missing pixels are manufactured here.
inline bool to_y8(const void *pixels, std::size_t available, unsigned stride,
                  std::uint32_t fourcc, head::Bytes &out)
{
    unsigned row_bytes;
    switch (fourcc) {
    case V4L2_PIX_FMT_GREY: row_bytes = width; break;
    case V4L2_PIX_FMT_Y10P: row_bytes = width * 5 / 4; break;
    case V4L2_PIX_FMT_Y10: row_bytes = width * 2; break;
    default: return false;
    }
    if (!pixels || stride < row_bytes ||
        available < std::size_t(stride) * (height - 1) + row_bytes) return false;
    out.resize(image_bytes);
    const auto *source = static_cast<const std::uint8_t *>(pixels);
    for (unsigned y = 0; y < height; ++y) {
        const auto *row = source + std::size_t(y) * stride;
        auto *target = out.data() + y * width;
        if (fourcc == V4L2_PIX_FMT_GREY) {
            std::memcpy(target, row, width);
        } else if (fourcc == V4L2_PIX_FMT_Y10P) {
            // Standard MIPI RAW10: four high bytes, then their packed low2s.
            for (unsigned x = 0; x < width; x += 4)
                std::memcpy(target + x, row + x / 4 * 5, 4);
        } else {
            for (unsigned x = 0; x < width; ++x) {
                const unsigned sample = unsigned(row[2 * x]) | unsigned(row[2 * x + 1]) << 8;
                target[x] = static_cast<std::uint8_t>((sample & 1023u) >> 2);
            }
        }
    }
    return true;
}

// Exact DSP 1280x480 layout, not two whole planes concatenated. The caller
// must first establish physical A/B identity and exposure pairing. This
// function neither selects frames nor certifies their synchronization.
inline bool side_by_side(const head::Bytes &a, const head::Bytes &b, head::Bytes &out)
{
    if (a.size() != image_bytes || b.size() != image_bytes || &out == &a || &out == &b)
        return false;
    out.resize(2 * image_bytes);
    for (unsigned y = 0; y < height; ++y) {
        std::memcpy(out.data() + y * 2 * width, a.data() + y * width, width);
        std::memcpy(out.data() + y * 2 * width + width, b.data() + y * width, width);
    }
    return true;
}
} // namespace a081::stock::camera
