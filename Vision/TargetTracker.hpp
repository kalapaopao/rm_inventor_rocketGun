#pragma once

// 卡尔曼跟踪与未来位置预测（方案 §7）。
// - 状态模型 [x, y, vx, vy]，二维匀速；检测有效时更新 x/y，
//   检测丢失时只做短时间预测（lost_timeout_ms 内）。
// - "预测 horizon 秒后的位置"只是当前状态向前外推，绝不回写滤波器。
// - 输出必须带 prediction_valid / uncertainty：协方差过大、速度突变
//   或连续丢失超阈值时置 invalid，不输出看似精确的未来位置。

#include <chrono>
#include <cstdint>

#include <opencv2/video/tracking.hpp>
#include "DetectionTypes.hpp"

namespace vision {

struct TrackerConfig {
    bool enabled = true;
    double predictionHorizonSeconds = 8.0;   // 对外输出的预测时间
    double maxPredictionSeconds = 10.0;      // 超过则 prediction_valid=false
    int lostTimeoutMs = 250;                 // 丢失超过该时长后重置跟踪
    int minConfirmFrames = 3;                // 连续命中多少帧才确认目标
    double processNoisePos = 4.0;            // 过程噪声：位置 (px^2)
    double processNoiseVel = 40.0;           // 过程噪声：速度 (px^2/s^2)
    double measurementNoise = 9.0;           // 测量噪声 (px^2)
    double uncertaintyLimit = 80.0;          // 位置标准差上限(px)，超过则预测无效
    double velocityJumpLimit = 600.0;        // 单帧速度突变上限(px/s)，超过则视为异常
};

class TargetTracker {
public:
    using Clock = std::chrono::steady_clock;

    void configure(const TrackerConfig& config) { config_ = config; }
    const TrackerConfig& config() const { return config_; }

    // 每帧调用一次。detections 为当前帧（已复筛）检测；
    // now 为该帧采集时间戳，用于真实 dt 与丢失计时。
    TrackedTarget update(const std::vector<Detection>& detections, Clock::time_point now);

    // 外部强制重置（例如 vision.enabled 切换）。
    void reset();

private:
    TrackerConfig config_;
    cv::KalmanFilter kf_{4, 2, 0, CV_32F};
    bool initialized_ = false;
    int confirmHits_ = 0;                // 连续命中计数
    int missedFrames_ = 0;
    int stableHits_ = 0;                 // 速度正常观测的连续帧数
    Clock::time_point lastUpdate_{};
    Clock::time_point lastSeen_{};
    float lastVx_ = 0.0F;
    float lastVy_ = 0.0F;
    bool velocityAnomaly_ = false;
};

}  // namespace vision
