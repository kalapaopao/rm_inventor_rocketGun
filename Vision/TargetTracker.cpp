#include "TargetTracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vision {

namespace {

// 速度突变后需要连续正常观测的帧数，超过才恢复预测输出。
constexpr int kStableFramesToRecover = 5;

// 从检测列表中挑选与当前跟踪位置最近的框（单目标场景）。
// 没有跟踪历史时选分数最高的检测。
const Detection* pickBest(const std::vector<Detection>& detections, bool hasTrack,
                          float trackX, float trackY) {
    if (detections.empty()) return nullptr;
    const Detection* best = nullptr;
    // Distance metrics are negative.  Initialising to -1 would reject every
    // candidate more than one pixel away from the predicted position.
    double bestMetric = -std::numeric_limits<double>::infinity();
    for (const Detection& d : detections) {
        double metric;
        if (hasTrack) {
            const double dx = d.centerX() - trackX;
            const double dy = d.centerY() - trackY;
            metric = -std::hypot(dx, dy);  // 距离越近越好
        } else {
            metric = d.score;
        }
        if (metric > bestMetric) {
            bestMetric = metric;
            best = &d;
        }
    }
    return best;
}

}  // namespace

void TargetTracker::reset() {
    initialized_ = false;
    confirmHits_ = 0;
    missedFrames_ = 0;
    velocityAnomaly_ = false;
    stableHits_ = 0;
    lastVx_ = 0.0F;
    lastVy_ = 0.0F;
}

TrackedTarget TargetTracker::update(const std::vector<Detection>& detections,
                                    Clock::time_point now) {
    TrackedTarget out;
    out.horizonSeconds = config_.predictionHorizonSeconds;

    if (!config_.enabled) return out;

    const double dt = initialized_
        ? std::chrono::duration<double>(now - lastUpdate_).count()
        : 0.0;

    const Detection* matched = nullptr;
    if (initialized_) {
        // 用上一时刻的最优估计（statePost）作为匹配门控中心。
        const cv::Mat& state = kf_.statePost;
        matched = pickBest(detections, true, state.at<float>(0), state.at<float>(1));
    } else {
        matched = pickBest(detections, false, 0.0F, 0.0F);
    }

    if (!initialized_) {
        if (matched == nullptr) return out;  // 还没有第一次观测，保持未确认
        // 初始化滤波器：位置取观测中心，速度先置 0。
        kf_.init(4, 2, 0, CV_32F);
        kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);  // dt 在每次 predict 前按真实间隔写入
        kf_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar(1e-2));
        cv::setIdentity(kf_.measurementNoiseCov,
                        cv::Scalar(config_.measurementNoise));
        cv::setIdentity(kf_.errorCovPost, cv::Scalar(100.0));
        kf_.statePost.at<float>(0) = matched->centerX();
        kf_.statePost.at<float>(1) = matched->centerY();
        kf_.statePost.at<float>(2) = 0.0F;
        kf_.statePost.at<float>(3) = 0.0F;
        initialized_ = true;
        confirmHits_ = 1;
        missedFrames_ = 0;
        stableHits_ = 0;
        lastUpdate_ = now;
        lastSeen_ = now;
        velocityAnomaly_ = false;

        out.confirmed = confirmHits_ >= config_.minConfirmFrames;
        out.currentX = matched->centerX();
        out.currentY = matched->centerY();
        // 首帧没有速度信息，不给未来位置。
        return out;
    }

    // 已初始化：先按真实 dt 预测，再用匹配到的检测更新（或只预测）。
    kf_.transitionMatrix.at<float>(0, 2) = static_cast<float>(dt);
    kf_.transitionMatrix.at<float>(1, 3) = static_cast<float>(dt);
    kf_.predict();

    if (matched != nullptr) {
        const cv::Mat meas = (cv::Mat_<float>(2, 1) << matched->centerX(),
                              matched->centerY());
        kf_.correct(meas);
        confirmHits_ = std::max(confirmHits_, 0) + 1;
        missedFrames_ = 0;
        lastSeen_ = now;
    } else {
        ++missedFrames_;
    }

    const float x = kf_.statePost.at<float>(0);
    const float y = kf_.statePost.at<float>(1);
    const float vx = kf_.statePost.at<float>(2);
    const float vy = kf_.statePost.at<float>(3);

    // 速度突变检测：单帧速度变化超限视为异常，冻结预测；
    // 连续若干帧正常观测后解除，避免单帧噪声永久锁死预测。
    if (matched != nullptr) {
        if (std::hypot(vx - lastVx_, vy - lastVy_) > config_.velocityJumpLimit) {
            velocityAnomaly_ = true;
            stableHits_ = 0;
        } else {
            ++stableHits_;
            if (stableHits_ >= kStableFramesToRecover) velocityAnomaly_ = false;
        }
        lastVx_ = vx;
        lastVy_ = vy;
    }

    // The next update interval must be measured from this frame, not from the
    // first frame that initialised the tracker.
    lastUpdate_ = now;

    const bool lostTooLong =
        std::chrono::duration<double, std::milli>(now - lastSeen_).count() >
        config_.lostTimeoutMs;
    if (lostTooLong) {
        // 丢失过久：状态不可信，重置等待下一次观测。
        const bool wasConfirmed = confirmHits_ >= config_.minConfirmFrames;
        reset();
        out.justLost = wasConfirmed;
        return out;
    }

    // 不确定度：位置协方差的标准差（取 x/y 中较大者）。
    const double stdX = std::sqrt(static_cast<double>(kf_.errorCovPost.at<float>(0, 0)));
    const double stdY = std::sqrt(static_cast<double>(kf_.errorCovPost.at<float>(1, 1)));
    const double uncertainty = std::max(stdX, stdY);

    out.confirmed = confirmHits_ >= config_.minConfirmFrames;
    out.missedFrames = missedFrames_;
    out.currentX = x;
    out.currentY = y;
    out.currentVx = vx;
    out.currentVy = vy;
    out.uncertainty = uncertainty;

    // 预测有效性（方案 7.4）：确认 + horizon 合法 + 未丢失过久 + 速度无突变
    // + 协方差在限内，才输出未来位置。
    const double horizon = config_.predictionHorizonSeconds;
    out.predictionValid = out.confirmed && matched != nullptr && !velocityAnomaly_ &&
                          horizon > 0.0 && horizon <= config_.maxPredictionSeconds &&
                          uncertainty <= config_.uncertaintyLimit;
    if (out.predictionValid) {
        out.futureX = x + vx * static_cast<float>(horizon);
        out.futureY = y + vy * static_cast<float>(horizon);
    }
    return out;
}

}  // namespace vision
