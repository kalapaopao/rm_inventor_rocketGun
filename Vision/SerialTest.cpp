#include "SerialOutput.hpp"
#include "SerialOverlay.hpp"
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>

void require(bool value, const char* text) { if (!value) throw std::runtime_error(text); }

bool readFrame(int fd, vision::Frame& frame, int timeoutMs) {
    size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto n = read(fd, frame.data() + offset, frame.size() - offset);
        if (n > 0) { offset += static_cast<size_t>(n); if (offset == frame.size()) return true; }
        else { pollfd descriptor{fd, POLLIN, 0}; poll(&descriptor, 1, 5); }
    }
    return false;
}

// 在超时内读到期望帧（跳过中间帧）。
bool expectFrame(int fd, const vision::Frame& want, int timeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    vision::Frame got{};
    while (std::chrono::steady_clock::now() < deadline) {
        const int left = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count());
        if (left <= 0 || !readFrame(fd, got, left)) return false;
        if (got == want) return true;
    }
    return false;
}

// 在窗口内确认只收到 TARGET=0 帧，没有 TARGET=1。
bool onlyNoTargetFrames(int fd, int windowMs) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(windowMs);
    bool sawAny = false, sawTarget = false;
    vision::Frame got{};
    while (std::chrono::steady_clock::now() < deadline) {
        const int left = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count());
        if (left <= 0 || !readFrame(fd, got, left)) break;
        sawAny = true;
        if (got[1] == 1) sawTarget = true;
    }
    return sawAny && !sawTarget;
}

int main() {
    int master = -1;
    try {
        using namespace vision;

        // ---- 线格式：与协议文档示例逐字节对齐 ----
        require(encodeFrame(true, 123) == Frame{0xA5, 0x01, 0x7B, 0x00, 0x51, 0xD8}, "positive frame");
        require(encodeFrame(true, -123) == Frame{0xA5, 0x01, 0x85, 0xFF, 0x51, 0xF8}, "negative frame");
        require(encodeFrame(true, 0) == Frame{0xA5, 0x01, 0x00, 0x00, 0x73, 0x28}, "valid-zero frame");
        require(encodeFrame(false, 0) == Frame{0xA5, 0x00, 0x00, 0x00, 0x22, 0xE8}, "no-target frame");
        require(encodeFrame(true, 2000) == Frame{0xA5, 0x01, 0xD0, 0x07, 0x6F, 0x2A}, "large frame");
        require(encodeFrame(false, 123) == encodeFrame(false, 0), "no-target forces yaw 0");

        // ---- CRC16-Modbus 已知值 ----
        const unsigned char head[] = {0xA5, 0x01, 0x7B, 0x00};
        require(crc16Modbus(head, 4) == 0xD851, "crc16 known value");

        // ---- 解码：往返 + 拒绝坏帧 ----
        {
            DecodedFrame decoded{};
            std::size_t consumed = 0;
            Frame f = encodeFrame(true, -321);
            require(decodeFrame(f.data(), f.size(), decoded, consumed) && consumed == 6 &&
                    decoded.targetValid && decoded.yaw == -321, "decode roundtrip");
            f = encodeFrame(false, 0);
            require(decodeFrame(f.data(), f.size(), decoded, consumed) &&
                    !decoded.targetValid && decoded.yaw == 0, "decode no-target");
            Frame bad = encodeFrame(true, 55);
            bad[5] ^= 0xFF;  // 破坏 CRC
            require(!decodeFrame(bad.data(), bad.size(), decoded, consumed), "decode accepts bad crc");
            const unsigned char junk[] = {0x11, 0x22, 0xA5, 0x01, 0x7B, 0x00, 0x51, 0xD8};
            require(decodeFrame(junk, sizeof(junk), decoded, consumed) && consumed == 8 &&
                    decoded.yaw == 123, "decode skips junk prefix");
            const unsigned char partialFrame[] = {0xA5, 0x01, 0x7B};
            require(!decodeFrame(partialFrame, sizeof(partialFrame), decoded, consumed) && consumed == 0,
                    "decode partial");
            const unsigned char onlyJunk[] = {0x11, 0x22};
            require(!decodeFrame(onlyJunk, sizeof(onlyJunk), decoded, consumed) && consumed == 2,
                    "decode all junk");
        }

        // ---- selectYaw 语义 ----
        VisionResult result;
        result.captureTimestamp = std::chrono::steady_clock::now();
        result.yawOffsetValid = true; result.yawOffsetPixels = 12.6;
        std::int16_t yaw = 0;
        require(selectYaw(result, result.captureTimestamp, 100, yaw) && yaw == 13, "rounding");
        result.yawOffsetPixels = 1e9;
        require(selectYaw(result, result.captureTimestamp, 100, yaw) && yaw == 32767, "positive saturation");
        result.yawOffsetPixels = -1e9;
        require(selectYaw(result, result.captureTimestamp, 100, yaw) && yaw == -32768, "negative saturation");
        result.yawOffsetPixels = std::numeric_limits<double>::quiet_NaN();
        require(!selectYaw(result, result.captureTimestamp, 100, yaw), "NaN allowed");
        result.yawOffsetPixels = 0;
        require(!selectYaw(result, result.captureTimestamp + std::chrono::milliseconds(101), 100, yaw), "stale allowed");
        require(!selectYaw(result, result.captureTimestamp - std::chrono::milliseconds(1), 100, yaw), "future timestamp");

        // ---- 叠层绘制不崩 ----
        cv::Mat image(240, 640, CV_8UC3, cv::Scalar());
        drawSerialPanel(image, result, false, 100, {});
        require(cv::countNonZero(image.reshape(1)) > 0, "overlay missing");

        // ---- PTY 真实系统调用路径 ----
        master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        require(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0, "PTY creation");
        SerialConfig config; config.port = ptsname(master); config.periodMs = 10; config.maxAgeMs = 100;
        std::mutex mutex;
        result.captureTimestamp = std::chrono::steady_clock::now();
        result.yawOffsetPixels = -123; result.yawOffsetValid = true;
        SerialOutput serial;
        require(serial.start(config, [&](VisionResult& out) {
            std::lock_guard<std::mutex> lock(mutex); out = result; return true;
        }), "PTY sender start");
        require(expectFrame(master, encodeFrame(true, -123), 200), "PTY target frame");

        // 结果过期 -> 发 TARGET=0，不再停发
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        { unsigned char drain[1024]; while (read(master, drain, sizeof(drain)) > 0) {} } // 丢弃过期前缓存的 TARGET=1 帧
        require(onlyNoTargetFrames(master, 80), "stale result should send TARGET=0 only");

        // 无有效目标 -> TARGET=0 帧
        {
            std::lock_guard<std::mutex> lock(mutex);
            result.captureTimestamp = std::chrono::steady_clock::now(); result.yawOffsetValid = false;
        }
        require(expectFrame(master, encodeFrame(false, 0), 200), "invalid result should send no-target frame");

        // 有效零 -> TARGET=1 yaw=0
        {
            std::lock_guard<std::mutex> lock(mutex);
            result.captureTimestamp = std::chrono::steady_clock::now();
            result.yawOffsetValid = true; result.yawOffsetPixels = 0;
        }
        require(expectFrame(master, encodeFrame(true, 0), 200), "valid zero frame");

        serial.stop();
        const auto status = serial.status();
        require(status.hasLastSent && status.lastYaw == 0 && status.lastTargetValid &&
                status.sent >= 3 && !status.running, "TX status");

        // 断线 -> 发送线程停止并计错
        {
            std::lock_guard<std::mutex> lock(mutex);
            result.captureTimestamp = std::chrono::steady_clock::now();
        }
        require(serial.start(config, [&](VisionResult& out) {
            std::lock_guard<std::mutex> lock(mutex); out = result; return true;
        }), "restart failed");
        { Frame f{}; require(readFrame(master, f, 200), "restart did not transmit"); }
        close(master); master = -1;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
        while (serial.running() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        require(!serial.running() && serial.status().errors > 0, "disconnect not detected");
        serial.stop();

        config.baud = 12345;
        require(!serial.start(config, [](VisionResult&) { return false; }), "unsupported baud accepted");

        std::cout << "PASS: 6-byte frames/CRC16/decode, yaw rounding+saturation, "
                     "stale->TARGET=0, invalid->TARGET=0, valid zero, PTY TX, status, disconnect, baud\n";
        return 0;
    } catch (const std::exception& e) {
        if (master >= 0) close(master);
        std::cerr << e.what() << '\n';
        return 1;
    }
}
