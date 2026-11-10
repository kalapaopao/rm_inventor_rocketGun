#pragma once

// TX-only: 8 bytes FF | int16 yaw_px LE | 00 00 00 00 | 0D.
// Current filtered pixel offset, right positive. Invalid/stale results send nothing.
// No checksum in this compatibility protocol. MCU MUST enforce receive timeout.
// See docs/SerialYawProtocol.md; not compatible with the former 20-byte packet.

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "DetectionTypes.hpp"
#include "SerialProtocol.hpp"
#include <condition_variable>
#include <mutex>

namespace vision {

struct SerialConfig {
    std::string port;          // 如 /dev/ttyUSB0；空则不启动
    int baud = 115200;
    int periodMs = 50;         // 固定发送周期
    int maxAgeMs = 500;
};

struct SerialStatus {
    bool running = false;
    std::uint64_t sent = 0, errors = 0;
    bool hasLastSent = false;
    std::int16_t lastYaw = 0;
};

class SerialOutput {
public:
    // provider 返回最新结果快照（VisionPipeline::getLatestResult 的包装）。
    using ResultProvider = std::function<bool(VisionResult&)>;

    SerialOutput() = default;
    ~SerialOutput() { stop(); }

    SerialOutput(const SerialOutput&) = delete;
    SerialOutput& operator=(const SerialOutput&) = delete;

    // 打开串口并启动发送线程；失败返回 false（串口异常不允许拖死采集）。
    bool start(const SerialConfig& config, ResultProvider provider);
    void stop();

    bool running() const { return running_.load(); }
    std::uint64_t sentFrames() const { return sentFrames_.load(); }
    SerialStatus status() const;

private:
    void loop();
    bool openPort();
    bool writePacket(const YawPacket& packet);

    SerialConfig config_;
    ResultProvider provider_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<std::uint64_t> sentFrames_{0};
    std::atomic<std::uint64_t> sendErrors_{0};
    mutable std::mutex statusMutex_;
    bool hasLastSent_ = false;
    std::int16_t lastYaw_ = 0;
    std::mutex waitMutex_;
    std::condition_variable wake_;
};

}  // namespace vision
