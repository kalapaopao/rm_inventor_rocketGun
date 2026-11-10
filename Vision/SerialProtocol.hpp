#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include "DetectionTypes.hpp"

namespace vision {
// Wire-compatible with Misaka21/dart FixedPacket<8>: FF, signed int16 LE,
// four zero reserved bytes, 0D. Independently implemented, no checksum.
using YawPacket = std::array<unsigned char, 8>;
inline YawPacket encodeYaw(std::int16_t yaw) {
    const auto bits = static_cast<std::uint16_t>(yaw);
    return {0xFF, static_cast<unsigned char>(bits & 0xFF),
            static_cast<unsigned char>(bits >> 8), 0, 0, 0, 0, 0x0D};
}

// Current filtered light centre, NOT future position or calibrated sight angle.
inline bool selectYaw(const VisionResult& result,
                      std::chrono::steady_clock::time_point now,
                      int maxAgeMs, std::int16_t& yaw) {
    const double age = std::chrono::duration<double,std::milli>(now - result.captureTimestamp).count();
    if (!result.yawOffsetValid || result.captureTimestamp.time_since_epoch().count() == 0 ||
        age < 0 || age > maxAgeMs || !std::isfinite(result.yawOffsetPixels)) return false;
    yaw = static_cast<std::int16_t>(std::lround(std::clamp(
        result.yawOffsetPixels, -32768.0, 32767.0)));
    return true;
}
}  // namespace vision
