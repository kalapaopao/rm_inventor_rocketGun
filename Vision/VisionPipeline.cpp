#include "VisionPipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <opencv2/imgproc.hpp>
#include "../Camera/AimOverlay.hpp"

namespace vision {

namespace {

// 叠框始终使用原图像素坐标，不与去畸变后的视线角混用。
void drawOverlay(cv::Mat& image, const VisionResult& result) {
    for (const Detection& d : result.detections) {
        const cv::Scalar color = d.refinePassed
            ? cv::Scalar(60, 60, 255)      // 红框：通过复筛
            : cv::Scalar(160, 160, 160);   // 灰框：模型框但未通过复筛
        cv::rectangle(image,
                      cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
                      cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
                      color, 2);
        if (d.lightRefined) {
            cv::circle(image, {cvRound(d.lightX), cvRound(d.lightY)},
                       std::max(1, cvRound(d.lightRadius)), cv::Scalar(0,255,0), 2);
            cv::drawMarker(image, {cvRound(d.lightX), cvRound(d.lightY)},
                           cv::Scalar(0,255,255), cv::MARKER_CROSS, 10, 1);
        }
        std::ostringstream label;
        label << "area " << std::fixed << std::setprecision(0) << d.score;
        cv::putText(image, label.str(),
                    cv::Point(static_cast<int>(d.x1),
                              std::max(12, static_cast<int>(d.y1) - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }

    if (result.target.predictionValid) {
        // 当前位置(黄十字) 与预测的未来位置(青色圆)。
        const cv::Point cur(static_cast<int>(result.target.currentX),
                            static_cast<int>(result.target.currentY));
        cv::drawMarker(image, cur, cv::Scalar(60, 220, 220), cv::MARKER_CROSS, 16, 2);
        const cv::Point fut(static_cast<int>(result.target.futureX),
                            static_cast<int>(result.target.futureY));
        cv::circle(image, fut, 12, cv::Scalar(220, 220, 60), 2, cv::LINE_AA);
        cv::line(image, cur, fut, cv::Scalar(220, 220, 60), 1, cv::LINE_AA);
    }

    drawCentreCrosshair(image);
    const VisionStats& s = result.stats;
    const int baseline = std::max(24, image.rows - 110);
    const auto line = [&](int idx, const std::string& text) {
        cv::putText(image, text, cv::Point(10, baseline + idx * 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(80, 255, 80), 1,
                    cv::LINE_AA);
    };
    std::ostringstream t;
    t << std::fixed << std::setprecision(1)
      << "Cap " << s.captureFps << " Detect " << s.detectionFps << " Disp "
      << s.displayFps << " drop " << s.droppedFrames;
    line(0, t.str());
    t.str("");
    t << std::fixed << std::setprecision(1)
      << "e2e p50 " << s.e2eLatencyP50Ms << " p95 " << s.e2eLatencyP95Ms
      << " ms  detect " << s.detectMs << " ms";
    line(1, t.str());
    t.str("");
    t << "track " << (result.target.confirmed ? "OK" : "--")
      << " lost " << s.trackingLostCount
      << " pred " << (result.target.predictionValid ? "valid" : "off");
    line(2, t.str());
}

}  // namespace

VisionPipeline::VisionPipeline(VisionConfig config)
    : config_(std::move(config)) {
}

VisionPipeline::~VisionPipeline() { stop(); }

bool VisionPipeline::start() {
    if (config_.backend != "traditional") {
        std::cerr << "[VisionPipeline] unsupported backend: " << config_.backend << '\n';
        return false;
    }
    if (config_.calibrationEnabled && !geometry_.load(config_.calibrationFile)) {
        std::cerr << "[Calibration] invalid intrinsics: " << config_.calibrationFile << '\n';
        return false;
    }
    tracker_.configure(config_.trackerConfig());

    stopping_.store(false);
    worker_ = std::thread(&VisionPipeline::workLoop, this);
    std::cout << "[VisionPipeline] started, backend=" << config_.backend << '\n';
    return true;
}

void VisionPipeline::stop() {
    bool expected = false;
    if (stopping_.compare_exchange_strong(expected, true)) {
        pendingCondition_.notify_all();
        if (worker_.joinable()) worker_.join();
    }
}

void VisionPipeline::submitFrame(cv::Mat bgrFrame,
                                 Clock::time_point captureTimestamp,
                                 float captureFps) {
    {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        if (pendingValid_) {
            // 队列深度恒为 1：旧帧直接丢弃（方案 §5.2 / 验收标准）。
            droppedFrames_.fetch_add(1, std::memory_order_relaxed);
        }
        pendingFrame_ = std::move(bgrFrame);  // 移动接管，避免整帧 clone
        pendingTimestamp_ = captureTimestamp;
        pendingCaptureFps_ = captureFps;
        pendingValid_ = true;
    }
    pendingCondition_.notify_one();
}

bool VisionPipeline::getLatestResult(VisionResult& out) {
    std::lock_guard<std::mutex> lock(resultMutex_);
    if (!resultValid_) return false;
    out = latestResult_;
    return true;
}

void VisionPipeline::recordLatency(double ms) {
    std::lock_guard<std::mutex> lock(latencyMutex_);
    latencySamples_.push_back(ms);
    while (latencySamples_.size() > 256) latencySamples_.pop_front();
}

void VisionPipeline::publishResult(VisionResult result) {
    {
        std::lock_guard<std::mutex> lock(resultMutex_);
        result.sequence = ++publishedSequence_;
        latestResult_ = std::move(result);
        resultValid_ = true;
    }
}

void VisionPipeline::workLoop() {
    while (!stopping_.load()) {
        cv::Mat frame;
        Clock::time_point ts{};
        float captureFps = 0.0F;
        {
            std::unique_lock<std::mutex> lock(pendingMutex_);
            pendingCondition_.wait(lock, [this] {
                return pendingValid_ || stopping_.load();
            });
            if (stopping_.load() && !pendingValid_) break;
            frame = std::move(pendingFrame_);
            ts = pendingTimestamp_;
            captureFps = pendingCaptureFps_;
            pendingValid_ = false;
        }
        if (frame.empty()) continue;
        processOne(frame, ts, captureFps);
    }
}

void VisionPipeline::processOne(cv::Mat& frame, Clock::time_point captureTimestamp,
                                float captureFps) {
    if (config_.calibrationEnabled && frame.size() != geometry_.imageSize()) {
        std::cerr << "[Calibration] frame size differs from calibration; result invalid\n";
        tracker_.reset();
        VisionResult invalid;
        invalid.captureTimestamp = captureTimestamp;
        publishResult(std::move(invalid));
        return;
    }
    const auto tDetect = Clock::now();
    std::vector<Detection> detections = detectGreenLights(frame, config_.greenLight);
    const double detectMs = std::chrono::duration<double, std::milli>(Clock::now()-tDetect).count();
    if (lastDetection_.time_since_epoch().count() > 0) {
        const double dt = std::chrono::duration<double>(tDetect-lastDetection_).count();
        if (dt > 0) detectionFps_ = detectionFps_ > 0 ?
            0.9*detectionFps_ + 0.1/dt : 1.0/dt;
    }
    lastDetection_ = tDetect;
    const auto tPost0 = Clock::now();
    TrackedTarget target;
    if (config_.enableTracker) {
        target = tracker_.update(detections, captureTimestamp);
        if (target.justLost) ++trackingLostCount_;
    }
    const double trackingMs =
        std::chrono::duration<double, std::milli>(Clock::now() - tPost0).count();

    // ---- 统计与发布 ----
    const auto now = Clock::now();
    recordLatency(std::chrono::duration<double, std::milli>(now - captureTimestamp)
                      .count());
    ++processedFrames_;

    VisionResult result;
    result.stats.captureFps = captureFps;
    result.stats.detectionFps = static_cast<float>(detectionFps_);
    result.stats.detectMs = detectMs;
    result.stats.trackingMs = trackingMs;
    result.stats.droppedFrames =
        droppedFrames_.load(std::memory_order_relaxed);  // 累计值
    result.stats.trackingLostCount = trackingLostCount_;
    result.stats.processedFrames = processedFrames_;
    {
        std::lock_guard<std::mutex> lock(latencyMutex_);
        std::vector<double> samples(latencySamples_.begin(), latencySamples_.end());
        std::sort(samples.begin(), samples.end());
        if (!samples.empty()) {
            auto pct = [&](double p) {
                const size_t idx =
                    std::min(samples.size() - 1,
                             static_cast<size_t>(p * (samples.size() - 1)));
                return samples[idx];
            };
            result.stats.e2eLatencyP50Ms = pct(0.50);
            result.stats.e2eLatencyP95Ms = pct(0.95);
        }
    }

    result.detections = std::move(detections);
    result.target = target;
    result.yawOffsetValid = target.confirmed && target.missedFrames == 0 &&
        std::isfinite(target.currentX) && target.currentX >= 0 && target.currentX < frame.cols;
    result.yawOffsetPixels = target.currentX - (frame.cols / 2 + config_.yawAimOffsetPx);
    if (config_.calibrationEnabled && target.confirmed) {
        cv::Point2d angle;
        result.currentAnglesValid = geometry_.angles({target.currentX, target.currentY}, frame.size(), angle);
        if (result.currentAnglesValid) {
            result.currentYaw = angle.x; result.currentPitch = angle.y;
        }
        result.futureAnglesValid = target.predictionValid &&
            geometry_.angles({target.futureX, target.futureY}, frame.size(), angle);
        if (result.futureAnglesValid) {
            result.futureYaw = angle.x; result.futurePitch = angle.y;
        }
    }
    result.annotated = frame.clone();  // 供显示/录像的叠框副本
    drawOverlay(result.annotated, result);
    result.captureTimestamp = captureTimestamp;
    publishResult(std::move(result));
}

}  // namespace vision
