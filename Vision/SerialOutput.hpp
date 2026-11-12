#pragma once

// TX-only 串口发送线程，协议 v1 定长 6 字节：
//   A5 | TARGET(0/1) | int16 yaw_px LE | CRC16-Modbus LE
// 每个周期都发一帧：有新鲜目标发 TARGET=1（含有效 0），否则发 TARGET=0。
// 这样电控能区分“链路活着但无目标”和“链路断开”，无需靠停发猜。
// 电控仍必须实现独立接收超时。协议文档：docs/串口通讯协议_电控对接.md

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "DetectionTypes.hpp"
#include "SerialProtocol.hpp"

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
    // 最近一次“有目标”帧的偏差；hasLastSent 仅在该值有效时为 true。
    bool hasLastSent = false;
    std::int16_t lastYaw = 0;
    // 最近一次成功写出的帧是否携带有效目标（用于显示）。
    bool lastTargetValid = false;
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
    bool writePacket(const Frame& packet);

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
    bool lastTargetValid_ = false;
    std::mutex waitMutex_;
    std::condition_variable wake_;
};

}  // namespace vision
