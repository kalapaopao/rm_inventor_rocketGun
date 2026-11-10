#pragma once

// VisionPipeline：传统绿色圆灯检测、跟踪与标定视线角。
// - 采集线程调用 submitFrame()（非阻塞，只有最新一帧待处理，
//   新帧到达覆盖旧帧并计入 droppedFrames，保证推理变慢时延迟不增长）；
// - 独立工作线程完成全图绿色分割 -> 圆形筛选 -> 灯心拟合 -> 卡尔曼跟踪；
// - getLatestResult() 供显示/录像/串口线程取"最新结果"快照，互相不阻塞。
// 相机代码只依赖本类，不接触任何推理库 API（方案 §6）。

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>
#include "DetectionTypes.hpp"
#include "TargetTracker.hpp"
#include "TargetGeometry.hpp"

namespace vision {

struct VisionConfig {
    bool enabled = false;
    std::string backend = "traditional";
    bool calibrationEnabled = false;
    std::string calibrationFile;
    GreenLightConfig greenLight;
    bool enableTracker = true;
    double predictionHorizonSeconds = 8.0;
    double maxPredictionSeconds = 10.0;
    int lostTimeoutMs = 250;
    int minConfirmFrames = 3;
    bool enableSerial = false;
    std::string serialPort;      // enable_serial=1 时必填，如 /dev/ttyUSB0
    int serialBaud = 115200;
    int serialPeriodMs = 50;
    int serialMaxAgeMs = 500;
    double yawAimOffsetPx = 0; // raw pixels, positive moves aim line right

    TrackerConfig trackerConfig() const {
        TrackerConfig c;
        c.enabled = enableTracker;
        c.predictionHorizonSeconds = predictionHorizonSeconds;
        c.maxPredictionSeconds = maxPredictionSeconds;
        c.lostTimeoutMs = lostTimeoutMs;
        c.minConfirmFrames = minConfirmFrames;
        return c;
    }
};

// 模型 ROI 中精调绿色圆灯，跟踪灯心并计算标定视线角。
class VisionPipeline {
public:
    using Clock = std::chrono::steady_clock;

    explicit VisionPipeline(VisionConfig config);
    ~VisionPipeline();

    VisionPipeline(const VisionPipeline&) = delete;
    VisionPipeline& operator=(const VisionPipeline&) = delete;

    // 加载可选标定并启动工作线程；失败返回 false。
    bool start();

    void stop();

    // 采集线程调用：提交 BGR 帧与采集完成时间戳。始终非阻塞。
    // captureFps 由采集线程自己测好带进来（方案 §11.5 分开统计）。
    void submitFrame(cv::Mat bgrFrame, Clock::time_point captureTimestamp,
                     float captureFps);

    // 输出线程调用：拷贝最新结果快照。没有结果时返回 false。
    // 消费者可用 VisionResult::sequence 判断是否为新结果。
    bool getLatestResult(VisionResult& out);

    const VisionConfig& config() const { return config_; }

private:
    void workLoop();
    void processOne(cv::Mat& frame, Clock::time_point captureTimestamp,
                    float captureFps);
    void publishResult(VisionResult result);
    void recordLatency(double ms);

    VisionConfig config_;
    CameraGeometry geometry_;

    TargetTracker tracker_;

    std::thread worker_;
    std::atomic<bool> stopping_{false};

    // 最新待处理帧（队列深度 1，新帧覆盖旧帧，方案 §5.2）。
    std::mutex pendingMutex_;
    cv::Mat pendingFrame_;
    Clock::time_point pendingTimestamp_{};
    float pendingCaptureFps_ = 0.0F;
    bool pendingValid_ = false;
    std::condition_variable pendingCondition_;

    // 最新结果快照（输出线程消费，方案 §5.4）。
    std::mutex resultMutex_;
    VisionResult latestResult_;
    bool resultValid_ = false;
    std::uint64_t publishedSequence_ = 0;

    // 端到端延迟环形样本（p50/p95），限制内存。
    std::mutex latencyMutex_;
    std::deque<double> latencySamples_;

    // 推理 FPS 平滑值。
    double detectionFps_ = 0.0;
    Clock::time_point lastDetection_{};
    std::atomic<std::uint64_t> droppedFrames_{0};  // submitFrame 覆盖旧帧计数
    std::uint64_t trackingLostCount_ = 0;
    std::uint64_t processedFrames_ = 0;
};

}  // namespace vision
