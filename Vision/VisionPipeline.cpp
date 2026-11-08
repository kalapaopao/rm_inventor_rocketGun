#include "VisionPipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <opencv2/imgproc.hpp>
#include "OnnxRuntimeBackend.hpp"
#include "Preprocess.hpp"

namespace vision {

namespace {

// ROI 传统视觉复筛（方案 §5.3）：裁剪检测框，用绿色圆灯与高亮灯条
// 的颜色/亮度判据确认这是"装甲板+灯条+圆灯"目标而不是误检。
bool refineDetection(const cv::Mat& bgr, const Detection& det,
                     const RefineConfig& cfg) {
    const int x1 = std::max(0, static_cast<int>(det.x1));
    const int y1 = std::max(0, static_cast<int>(det.y1));
    const int x2 = std::min(bgr.cols - 1, static_cast<int>(det.x2));
    const int y2 = std::min(bgr.rows - 1, static_cast<int>(det.y2));
    const int w = x2 - x1 + 1;
    const int h = y2 - y1 + 1;
    if (w < 4 || h < 4) return false;
    if (w * h < cfg.minAreaPx) return false;
    if (w * h > bgr.total() * cfg.maxAreaRatio / 100.0) return false;

    cv::Mat roi = bgr(cv::Rect(x1, y1, w, h));
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // 绿色圆灯：OpenCV 绿色 Hue 约 35..85。
    cv::Mat green;
    cv::inRange(hsv, cv::Scalar(35, 60, 60), cv::Scalar(85, 255, 255), green);
    const double greenRatio =
        static_cast<double>(cv::countNonZero(green)) / static_cast<double>(roi.total());

    // 高亮灯条：V 通道阈值。
    std::vector<cv::Mat> hsvCh;
    cv::split(hsv, hsvCh);
    const double brightRatio = static_cast<double>(
        cv::countNonZero(hsvCh[2] > 200)) / static_cast<double>(roi.total());

    return greenRatio >= cfg.minGreenRatio || brightRatio >= cfg.minBrightRatio;
}

void drawOverlay(cv::Mat& image, const VisionResult& result) {
    for (const Detection& d : result.detections) {
        const cv::Scalar color = d.refinePassed
            ? cv::Scalar(60, 60, 255)      // 红框：通过复筛
            : cv::Scalar(160, 160, 160);   // 灰框：模型框但未通过复筛
        cv::rectangle(image,
                      cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
                      cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
                      color, 2);
        std::ostringstream label;
        label << std::fixed << std::setprecision(2) << d.score;
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

    const VisionStats& s = result.stats;
    const int baseline = std::max(24, image.rows - 110);
    const auto line = [&](int idx, const std::string& text) {
        cv::putText(image, text, cv::Point(10, baseline + idx * 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(80, 255, 80), 1,
                    cv::LINE_AA);
    };
    std::ostringstream t;
    t << std::fixed << std::setprecision(1)
      << "Cap " << s.captureFps << " Inf " << s.inferenceFps << " Disp "
      << s.displayFps << " drop " << s.droppedFrames;
    line(0, t.str());
    t.str("");
    t << std::fixed << std::setprecision(1)
      << "e2e p50 " << s.e2eLatencyP50Ms << " p95 " << s.e2eLatencyP95Ms
      << " ms  inf " << s.inferMs << " ms";
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
    if (config_.backend != "onnxruntime_cpu") {
        // 当前只实现了 CPU 后端；其他后端名在 start() 时报错，由调用方退出。
        std::cout << "[VisionPipeline] unknown backend '" << config_.backend
                  << "', will report load failure\n";
    }
}

VisionPipeline::~VisionPipeline() { stop(); }

bool VisionPipeline::start() {
    if (config_.backend != "onnxruntime_cpu") {
        std::cerr << "[VisionPipeline] unsupported backend: " << config_.backend << '\n';
        return false;
    }
    backend_ = std::make_unique<OnnxRuntimeBackend>();
    if (!backend_->load(config_.modelPath)) {
        std::cerr << "[VisionPipeline] model load failed: " << config_.modelPath << '\n';
        backend_.reset();
        return false;
    }
    // 模型实际输入尺寸优先于配置值（配置只是期望值）。
    auto* ort = dynamic_cast<OnnxRuntimeBackend*>(backend_.get());
    if (ort != nullptr) {
        if (ort->inputWidth() != config_.inputWidth ||
            ort->inputHeight() != config_.inputHeight) {
            std::cout << "[VisionPipeline] model input " << ort->inputWidth() << "x"
                      << ort->inputHeight() << " overrides config "
                      << config_.inputWidth << "x" << config_.inputHeight << '\n';
        }
    }
    tracker_.configure(config_.trackerConfig());

    // ORT 首次推理要分配内存/跑一遍图优化（实测约 240ms），用空图预热 3 次，
    // 避免把暖机延迟算进真实帧的 p50/p95。
    {
        auto* ortBackend = dynamic_cast<OnnxRuntimeBackend*>(backend_.get());
        if (ortBackend != nullptr) {
            cv::Mat warm(ortBackend->inputHeight(), ortBackend->inputWidth(),
                         CV_32FC3, cv::Scalar(114.0 / 255.0));
            std::vector<float> warmOut;
            for (int i = 0; i < 3; ++i) backend_->infer(warm, warmOut);
        }
    }

    stopping_.store(false);
    worker_ = std::thread(&VisionPipeline::workLoop, this);
    std::cout << "[VisionPipeline] started, backend=" << backend_->name() << '\n';
    return true;
}

void VisionPipeline::stop() {
    bool expected = false;
    if (stopping_.compare_exchange_strong(expected, true)) {
        pendingCondition_.notify_all();
        if (worker_.joinable()) worker_.join();
        backend_.reset();
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
    // ---- 预处理 ----
    const auto tPre0 = Clock::now();
    LetterboxParams params;
    cv::Mat input = letterbox(frame, config_.inputWidth, params);
    const double preprocessMs =
        std::chrono::duration<double, std::milli>(Clock::now() - tPre0).count();

    // ---- 推理 ----
    const auto tInf0 = Clock::now();
    std::vector<float> output;
    if (!backend_->infer(input, output)) {
        return;  // 单帧失败不结束线程，连续失败由外部 stop 处理
    }
    const double inferMs =
        std::chrono::duration<double, std::milli>(Clock::now() - tInf0).count();
    if (lastInfer_.time_since_epoch().count() > 0) {
        const double dt =
            std::chrono::duration<double>(tInf0 - lastInfer_).count();
        if (dt > 0.0) {
            const double inst = 1.0 / dt;
            inferenceFps_ = inferenceFps_ > 0.0 ? 0.9 * inferenceFps_ + 0.1 * inst : inst;
        }
    }
    lastInfer_ = tInf0;

    // ---- 解码 + 复筛 + 跟踪 ----
    const auto tPost0 = Clock::now();
    std::vector<Detection> detections;
    decodeEnd2End(output, params, config_.confidenceThreshold, detections);

    for (Detection& d : detections) {
        d.refinePassed = !config_.enableRefine ||
                         refineDetection(frame, d, refine_);
    }
    // 未通过复筛的框不参与跟踪。
    std::vector<Detection> valid;
    valid.reserve(detections.size());
    for (const Detection& d : detections) {
        if (d.refinePassed) valid.push_back(d);
    }

    TrackedTarget target;
    if (config_.enableTracker) {
        target = tracker_.update(valid, captureTimestamp);
        if (target.justLost) ++trackingLostCount_;
    }
    const double refineMs =
        std::chrono::duration<double, std::milli>(Clock::now() - tPost0).count();

    // ---- 统计与发布 ----
    const auto now = Clock::now();
    recordLatency(std::chrono::duration<double, std::milli>(now - captureTimestamp)
                      .count());
    ++processedFrames_;

    VisionResult result;
    result.stats.captureFps = captureFps;
    result.stats.inferenceFps = static_cast<float>(inferenceFps_);
    result.stats.preprocessMs = preprocessMs;
    result.stats.inferMs = inferMs;
    result.stats.refineMs = refineMs;
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

    result.annotated = frame.clone();  // 供显示/录像的叠框副本
    drawOverlay(result.annotated, result);
    result.detections = std::move(detections);
    result.target = target;
    result.captureTimestamp = captureTimestamp;
    publishResult(std::move(result));
}

}  // namespace vision
