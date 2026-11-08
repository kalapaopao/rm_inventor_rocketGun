#pragma once

// 串口输出线程（方案 §9.9，默认 enable_serial=0）。
// 按固定周期发送最新目标状态，不为每个检测结果排队（方案 §5.4）。
//
// 帧格式（小端，参照 reference/2024_Dart_Algorithm vision_protocol 的
// SOF+CRC 风格；因项目未定义正式协议，这是可调整的默认实现）：
//   [0] 0xA5 SOF
//   [1] tracking_state: 0=未跟踪 1=跟踪中(预测有效) 2=跟踪中(预测无效)
//   [2..5] float future_x  (像素，prediction_horizon_seconds 外推)
//   [6..9] float future_y
//   [10..13] float current_x
//   [14..17] float current_y
//   [18..19] uint16 CRC16(Modbus)，覆盖前 18 字节
// 共 20 字节。

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "DetectionTypes.hpp"

namespace vision {

struct SerialConfig {
    std::string port;          // 如 /dev/ttyUSB0；空则不启动
    int baud = 115200;
    int periodMs = 50;         // 固定发送周期
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

private:
    void loop();
    bool openPort();
    void buildFrame(const VisionResult& result, unsigned char* buf) const;

    SerialConfig config_;
    ResultProvider provider_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<std::uint64_t> sentFrames_{0};
    std::atomic<std::uint64_t> sendErrors_{0};
};

}  // namespace vision
