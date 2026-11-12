#pragma once

// 串口通讯协议 v1（TX only，定长 6 字节）：
//   SOF 0xA5 | TARGET(0/1) | int16 YAW 小端 | CRC16-Modbus 小端
// YAW = 滤波后灯心 x - 瞄准线 x，原图像素，右正左负。
// 例：A5 01 7B 00 51 D8 = 有目标 +123px；A5 00 00 00 22 E8 = 无目标。
// 与旧版 8 字节 FF..0D 帧不兼容。协议文档：docs/串口通讯协议_电控对接.md

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "DetectionTypes.hpp"

namespace vision {

constexpr unsigned char kSof = 0xA5;
using Frame = std::array<unsigned char, 6>;

// CRC-16/MODBUS：初值 0xFFFF，反射多项式 0xA001，无最终异或。
inline std::uint16_t crc16Modbus(const unsigned char* data, std::size_t length) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001U) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001U)
                                  : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

// 打包一帧。targetValid=false 时 YAW 强制为 0、TARGET=0。
inline Frame encodeFrame(bool targetValid, std::int16_t yaw) {
    const std::uint16_t bits =
        static_cast<std::uint16_t>(targetValid ? yaw : std::int16_t{0});
    Frame frame{};
    frame[0] = kSof;
    frame[1] = targetValid ? 1 : 0;
    frame[2] = static_cast<unsigned char>(bits & 0xFFU);
    frame[3] = static_cast<unsigned char>((bits >> 8) & 0xFFU);
    const std::uint16_t crc = crc16Modbus(frame.data(), 4);
    frame[4] = static_cast<unsigned char>(crc & 0xFFU);
    frame[5] = static_cast<unsigned char>((crc >> 8) & 0xFFU);
    return frame;
}

// 当前滤波灯心的水平像素偏差，不是预测值、不是光轴角。
inline bool selectYaw(const VisionResult& result,
                      std::chrono::steady_clock::time_point now,
                      int maxAgeMs, std::int16_t& yaw) {
    const double age = std::chrono::duration<double, std::milli>(now - result.captureTimestamp).count();
    if (!result.yawOffsetValid || result.captureTimestamp.time_since_epoch().count() == 0 ||
        age < 0 || age > maxAgeMs || !std::isfinite(result.yawOffsetPixels)) return false;
    yaw = static_cast<std::int16_t>(std::lround(std::clamp(
        result.yawOffsetPixels, -32768.0, 32767.0)));
    return true;
}

struct DecodedFrame {
    bool targetValid = false;
    std::int16_t yaw = 0;
};

// 从字节流中解出一帧（供测试与 PC 端模拟解码使用）。
// 成功：返回 true，consumed = 帧尾之后的偏移（含之前跳过的垃圾字节）。
// 失败：返回 false，consumed = 应跳过的字节数（0 表示帧头后数据不足，需继续收）。
inline bool decodeFrame(const unsigned char* data, std::size_t length,
                        DecodedFrame& out, std::size_t& consumed) {
    std::size_t i = 0;
    while (i < length && data[i] != kSof) ++i;
    if (i >= length) { consumed = length; return false; }   // 全是垃圾，无帧头
    if (length - i < 6) { consumed = i; return false; }     // 帧头后不足一帧
    const unsigned char* f = data + i;
    const std::uint16_t crc = crc16Modbus(f, 4);
    const std::uint16_t got = static_cast<std::uint16_t>(
        f[4] | (static_cast<std::uint16_t>(f[5]) << 8));
    if (crc != got || f[1] > 1) { consumed = i + 1; return false; } // 校验失败/非法 TARGET，跳过帧头重同步
    out.targetValid = f[1] == 1;
    out.yaw = static_cast<std::int16_t>(static_cast<std::uint16_t>(
        f[2] | (static_cast<std::uint16_t>(f[3]) << 8)));
    if (!out.targetValid) out.yaw = 0;
    consumed = i + 6;
    return true;
}

}  // namespace vision
