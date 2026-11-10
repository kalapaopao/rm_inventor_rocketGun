#include "CameraSetting.hpp"
#include "AimOverlay.hpp"
#ifdef RM_ENABLE_VISION
#include "../Vision/SerialOverlay.hpp"
#endif

#include <MvCameraControl.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

#ifndef DEFAULT_RECORDING_DIR
#define DEFAULT_RECORDING_DIR "recordings"
#endif

namespace {

constexpr double kDisplayScale = 0.8;

bool checkResult(const char* operation, int result) {
    if (result == MV_OK) return true;
    std::cerr << operation << " failed, error = 0x"
              << std::hex << result << std::dec << '\n';
    return false;
}

class ImageBufferGuard {
public:
    ImageBufferGuard(void* handle, MV_FRAME_OUT& frame)
        : handle_(handle), frame_(frame) {}
    ImageBufferGuard(const ImageBufferGuard&) = delete;
    ImageBufferGuard& operator=(const ImageBufferGuard&) = delete;
    ~ImageBufferGuard() {
        if (active_) MV_CC_FreeImageBuffer(handle_, &frame_);
    }

private:
    void* handle_;
    MV_FRAME_OUT& frame_;
    bool active_ = true;
};

void drawMetrics(cv::Mat& image, float pipelineFps, double captureMs) {
    drawCentreCrosshair(image);
    const int baseline = std::max(24, image.rows - 20);
    const int lineHeight = 25;
    const auto draw = [&](const std::string& text, int line) {
        cv::putText(image, text, cv::Point(10, baseline - lineHeight * line),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0),
                    2, cv::LINE_AA);
    };
    std::ostringstream text;
    text << "Total FPS: " << std::fixed << std::setprecision(1) << pipelineFps;
    draw(text.str(), 3);
    text.str("");
    text.clear();
    text << "Capture: " << std::fixed << std::setprecision(1) << captureMs << " ms";
    draw(text.str(), 2);
    draw("Algorithm: 0.0 ms", 1);
    draw("Comm: 0.0 ms", 0);
}

class OutputWorker {
public:
    explicit OutputWorker(const CameraConfig& config) : config_(config) {}
    ~OutputWorker() { stop(); }
    bool stopRequested() const { return stopRequested_.load(std::memory_order_relaxed); }

#ifdef RM_ENABLE_VISION
    void setSerialProvider(std::function<vision::SerialStatus()> provider) {
        serialStatusProvider_ = std::move(provider);
    }
#endif
    void start() { worker_ = std::thread(&OutputWorker::loop, this); }

    // 视觉模式：显示/录像只消费 VisionPipeline 的最新结果快照（方案 §5.4），
    // 与采集、推理线程互不阻塞；必须在 start() 前设置。
#ifdef RM_ENABLE_VISION
    void setVisionProvider(
        std::function<bool(vision::VisionResult&)> provider) {
        visionProvider_ = std::move(provider);
    }
#endif

    bool submit(const cv::Mat& frame, float fps, double captureMs) {
        if (stopRequested_.load(std::memory_order_relaxed)) return false;
        cv::Mat copy = frame.clone();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latestFrame_ = std::move(copy);
            pipelineFps_ = fps;
            captureMs_ = captureMs;
            hasFrame_ = true;
        }
        condition_.notify_one();
        return true;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    void loop() {
#ifdef RM_ENABLE_VISION
        if (visionProvider_) {
            visionLoop();
            return;
        }
#endif
        legacyLoop();
    }

    // 原有路径（vision.enabled=0）：与之前行为一致。
    void legacyLoop() {
        VideoRecorder recorder(DEFAULT_RECORDING_DIR, 30.0);
        bool recorderStarted = false;
        while (true) {
            cv::Mat frame;
            float fps = 0.0F;
            double captureMs = 0.0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] { return hasFrame_ || stopping_; });
                if (!hasFrame_ && stopping_) break;
                frame = std::move(latestFrame_);
                fps = pipelineFps_;
                captureMs = captureMs_;
                hasFrame_ = false;
            }

            if (config_.recordSeconds > 0.0) {
                if (!recorderStarted) {
                    if (!recorder.start(frame.size(), frame.channels() != 1)) {
                        stopRequested_.store(true, std::memory_order_relaxed);
                        break;
                    }
                    recorderStarted = true;
                }
                recorder.write(frame);
            }

            if (config_.displayEnabled) {
                cv::Mat displayFrame;
                cv::resize(frame, displayFrame, cv::Size(), kDisplayScale,
                           kDisplayScale, cv::INTER_AREA);
                drawMetrics(displayFrame, fps, captureMs);
                cv::imshow(config_.windowName, displayFrame);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q') {
                    stopRequested_.store(true, std::memory_order_relaxed);
                    break;
                }
            }
        }
        recorder.stop();
        if (config_.displayEnabled) cv::destroyAllWindows();
    }

    // 视觉路径：轮询最新结果快照。没有新帧时短暂休眠，不 busy-wait。
    // 显示、录像、统计都在这里消费"已处理结果"，不会拖慢采集/推理。
#ifdef RM_ENABLE_VISION
    void visionLoop() {
        VideoRecorder recorder(DEFAULT_RECORDING_DIR, 30.0);
        bool recorderStarted = false;
        double displayFps = 0.0;
        std::uint64_t lastSequence = 0;
        using SteadyClock = std::chrono::steady_clock;
        SteadyClock::time_point lastDisplay = SteadyClock::now();
        SteadyClock::time_point lastLog = SteadyClock::now();
        SteadyClock::time_point lastPanelRefresh{};
        while (!stopRequested_.load(std::memory_order_relaxed)) {
            vision::VisionResult result;
            if (!visionProvider_(result)) {
                // 没有结果或还是上次消费过的同一份快照：短暂休眠，不 busy-wait。
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
                continue;
            }
            if (result.sequence == lastSequence) {
                // Keep TX/error/staleness visible even when inference stops.
                // This is a UI refresh only: never record/count a duplicate result.
                const auto now = SteadyClock::now();
                if (config_.displayEnabled && !result.annotated.empty() &&
                    now-lastPanelRefresh >= std::chrono::milliseconds(100)) {
                    cv::Mat panel = result.annotated.clone(), display;
                    const auto status = serialStatusProvider_ ? serialStatusProvider_() : vision::SerialStatus{};
                    vision::drawSerialPanel(panel, result, config_.vision.enableSerial,
                                            config_.vision.serialMaxAgeMs, status);
                    cv::resize(panel, display, {}, kDisplayScale, kDisplayScale, cv::INTER_AREA);
                    cv::imshow(config_.windowName, display);
                    const int key = cv::waitKey(1);
                    if (key == 27 || key == 'q' || key == 'Q') stopRequested_.store(true);
                    lastPanelRefresh = now;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
                continue;
            }
            lastSequence = result.sequence;
            if (result.annotated.empty()) continue;
            result.annotated = result.annotated.clone(); // Never modify shared pipeline snapshots.
            const auto status = serialStatusProvider_ ? serialStatusProvider_() : vision::SerialStatus{};
            vision::drawSerialPanel(result.annotated, result, config_.vision.enableSerial,
                                    config_.vision.serialMaxAgeMs, status);
            const auto now = SteadyClock::now();
            const double dt = std::chrono::duration<double>(now - lastDisplay).count();
            if (dt > 1e-3) {  // 下限保护：dt 异常小时跳过本次 FPS 平滑
                displayFps = displayFps > 0.0
                    ? 0.9 * displayFps + 0.1 * (1.0 / dt)
                    : 1.0 / dt;
            }
            lastDisplay = now;
            result.stats.displayFps = static_cast<float>(displayFps);

            if (config_.recordSeconds > 0.0) {
                if (!recorderStarted) {
                    if (!recorder.start(result.annotated.size(), true)) {
                        stopRequested_.store(true, std::memory_order_relaxed);
                        break;
                    }
                    recorderStarted = true;
                }
                recorder.write(result.annotated);
            }

            if (config_.displayEnabled) {
                cv::Mat displayFrame;
                cv::resize(result.annotated, displayFrame, cv::Size(),
                           kDisplayScale, kDisplayScale, cv::INTER_AREA);
                // 管线已在 annotated 上叠加检测框与统计，这里不再画 metrics。
                cv::imshow(config_.windowName, displayFrame);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q') {
                    stopRequested_.store(true, std::memory_order_relaxed);
                    break;
                }
            }

            // 周期性记录四类 FPS 与延迟（方案 §9.6 / §11.5 / 验收标准）。
            if (std::chrono::duration<double>(now - lastLog).count() >= 2.0) {
                lastLog = now;
                const vision::VisionStats& s = result.stats;
                std::cout << "[STATS] cap=" << s.captureFps << " detect="
                          << s.detectionFps << " disp=" << s.displayFps
                          << " | e2e p50=" << s.e2eLatencyP50Ms << "ms p95="
                          << s.e2eLatencyP95Ms << "ms | detect_ms="
                          << s.detectMs << " drop=" << s.droppedFrames
                          << " lost=" << s.trackingLostCount << '\n';
            }
        }
        recorder.stop();
        if (config_.displayEnabled) cv::destroyAllWindows();
    }

    std::function<vision::SerialStatus()> serialStatusProvider_;
    std::function<bool(vision::VisionResult&)> visionProvider_;
#endif
    CameraConfig config_;
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable condition_;
    cv::Mat latestFrame_;
    float pipelineFps_ = 0.0F;
    double captureMs_ = 0.0;
    bool hasFrame_ = false;
    bool stopping_ = false;
    std::atomic<bool> stopRequested_{false};
};

}  // namespace

bool CameraConfig::load(const std::string& path) {
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        std::cerr << "Cannot open camera config: " << path << '\n';
        return false;
    }

    const cv::FileNode camera = storage["camera"];
    if (camera.empty()) {
        std::cerr << "Missing 'camera' section in config: " << path << '\n';
        return false;
    }

    int deviceIndexValue = static_cast<int>(deviceIndex);
    int grabTimeoutValue = static_cast<int>(grabTimeoutMs);
    int bayerQualityValue = static_cast<int>(bayerQuality);
    int adcBitDepthValue = static_cast<int>(adcBitDepth);
    int frameRateEnableValue = acquisitionFrameRateEnable ? 1 : 0;

    if (!camera["device_index"].empty()) camera["device_index"] >> deviceIndexValue;
    if (!camera["width"].empty()) camera["width"] >> frameWidth;
    if (!camera["height"].empty()) camera["height"] >> frameHeight;
    if (!camera["exposure_time"].empty()) camera["exposure_time"] >> exposureTime;
    if (!camera["gain"].empty()) camera["gain"] >> gain;
    if (!camera["grab_timeout_ms"].empty()) camera["grab_timeout_ms"] >> grabTimeoutValue;
    if (!camera["bayer_quality"].empty()) camera["bayer_quality"] >> bayerQualityValue;
    if (!camera["adc_bit_depth"].empty()) camera["adc_bit_depth"] >> adcBitDepthValue;
    if (!camera["acquisition_frame_rate_enable"].empty()) {
        camera["acquisition_frame_rate_enable"] >> frameRateEnableValue;
    }
    if (!camera["window_name"].empty()) camera["window_name"] >> windowName;
    if (!camera["record_seconds"].empty()) camera["record_seconds"] >> recordSeconds;

    if (deviceIndexValue < 0 || frameWidth <= 0 || frameHeight <= 0 ||
        exposureTime <= 0.0F || gain < 0.0F || grabTimeoutValue <= 0 ||
        bayerQualityValue < 0 || bayerQualityValue > 3 ||
        (adcBitDepthValue != 8 && adcBitDepthValue != 12) ||
        (frameRateEnableValue != 0 && frameRateEnableValue != 1) ||
        recordSeconds < 0.0 || windowName.empty()) {
        std::cerr << "Invalid camera config: " << path << '\n';
        return false;
    }

    deviceIndex = static_cast<unsigned int>(deviceIndexValue);
    grabTimeoutMs = static_cast<unsigned int>(grabTimeoutValue);
    bayerQuality = static_cast<unsigned int>(bayerQualityValue);
    adcBitDepth = static_cast<unsigned int>(adcBitDepthValue);
    acquisitionFrameRateEnable = frameRateEnableValue == 1;

#ifdef RM_ENABLE_VISION
    // vision 配置块（方案 §8）。节点不存在时保持默认 enabled=0 纯取流行为。
    const cv::FileNode visionNode = storage["vision"];
    if (!visionNode.empty()) {
        vision::VisionConfig& v = vision;
        const auto calibration = visionNode["calibration"];
        if (!calibration.empty()) {
            int enabled = 0;
            calibration["enabled"] >> enabled;
            v.calibrationEnabled = enabled != 0;
            calibration["file"] >> v.calibrationFile;
            if (v.calibrationEnabled) {
                if (v.calibrationFile.empty()) return false;
                std::filesystem::path file(v.calibrationFile);
                if (file.is_relative()) file = std::filesystem::path(path).parent_path() / file;
                v.calibrationFile = file.lexically_normal().string();
            }
        }
        const auto green = visionNode["green_light"];
        if (!green.empty()) {
            auto& g = v.greenLight;
            if (!green["hue_min"].empty()) green["hue_min"] >> g.hueMin;
            if (!green["hue_max"].empty()) green["hue_max"] >> g.hueMax;
            if (!green["saturation_min"].empty()) green["saturation_min"] >> g.saturationMin;
            if (!green["value_min"].empty()) green["value_min"] >> g.valueMin;
            if (!green["max_area_px"].empty()) green["max_area_px"] >> g.maxArea;
            if (!green["min_area_px"].empty()) green["min_area_px"] >> g.minArea;
            if (!green["min_circularity"].empty()) green["min_circularity"] >> g.minCircularity;
            if (!green["min_axis_ratio"].empty()) green["min_axis_ratio"] >> g.minAxisRatio;
            if (g.hueMin < 0 || g.hueMax > 179 || g.hueMin > g.hueMax ||
                g.saturationMin < 0 || g.saturationMin > 255 || g.valueMin < 0 ||
                g.valueMin > 255 || !std::isfinite(g.minArea) || g.minArea <= 0 || !std::isfinite(g.maxArea) || g.maxArea < g.minArea ||
                !std::isfinite(g.minCircularity) || g.minCircularity <= 0 ||
                g.minCircularity > 1 || !std::isfinite(g.minAxisRatio) ||
                g.minAxisRatio <= 0 || g.minAxisRatio > 1) return false;
        }
        if (!visionNode["enabled"].empty()) {
            int enabled = v.enabled ? 1 : 0;
            visionNode["enabled"] >> enabled;
            v.enabled = enabled != 0;
        }
        if (!visionNode["backend"].empty()) visionNode["backend"] >> v.backend;
        if (!visionNode["enable_tracker"].empty()) {
            int tracker = v.enableTracker ? 1 : 0;
            visionNode["enable_tracker"] >> tracker;
            v.enableTracker = tracker != 0;
        }
        if (!visionNode["prediction_horizon_seconds"].empty()) {
            visionNode["prediction_horizon_seconds"] >> v.predictionHorizonSeconds;
        }
        if (!visionNode["max_prediction_seconds"].empty()) {
            visionNode["max_prediction_seconds"] >> v.maxPredictionSeconds;
        }
        if (!visionNode["lost_timeout_ms"].empty()) {
            int timeout = v.lostTimeoutMs;
            visionNode["lost_timeout_ms"] >> timeout;
            v.lostTimeoutMs = timeout;
        }
        if (!visionNode["min_confirm_frames"].empty()) {
            int frames = v.minConfirmFrames;
            visionNode["min_confirm_frames"] >> frames;
            v.minConfirmFrames = frames;
        }
        if (!visionNode["enable_serial"].empty()) {
            int serial = v.enableSerial ? 1 : 0;
            visionNode["enable_serial"] >> serial;
            v.enableSerial = serial != 0;
        }
        if (!visionNode["serial_port"].empty()) visionNode["serial_port"] >> v.serialPort;
        if (!visionNode["serial_baud"].empty()) {
            int baud = v.serialBaud;
            visionNode["serial_baud"] >> baud;
            v.serialBaud = baud;
        }
        if (!visionNode["serial_period_ms"].empty()) {
            int period = v.serialPeriodMs;
            visionNode["serial_period_ms"] >> period;
            v.serialPeriodMs = period;
        }

        if (!visionNode["serial_max_age_ms"].empty()) visionNode["serial_max_age_ms"] >> v.serialMaxAgeMs;
        if (!visionNode["yaw_aim_offset_px"].empty()) visionNode["yaw_aim_offset_px"] >> v.yawAimOffsetPx;
        if (v.enabled && (v.serialPeriodMs <= 0 || v.serialPeriodMs > 1000 ||
                          v.serialMaxAgeMs <= 0 || !std::isfinite(v.yawAimOffsetPx) ||
                          (v.enableSerial && v.serialPort.empty()))) {
            std::cerr << "Invalid serial timing/port or yaw aim offset\n";
            return false;
        }
        if (v.enabled && v.predictionHorizonSeconds > v.maxPredictionSeconds) {
            std::cerr << "vision.prediction_horizon_seconds exceeds max_prediction_seconds\n";
            return false;
        }

    }
#else
    if (!storage["vision"].empty()) {
        std::cerr << "Warning: config has a 'vision' section but this binary was "
                     "built without the vision pipeline (RM_ENABLE_VISION); "
                     "running capture-only.\n";
    }
#endif

    return true;
}

CameraSetting::CameraSetting(CameraConfig config)
    : config_(std::move(config)) {}

bool CameraSetting::setupCamera(void* handle) {
    bool success = true;
    success &= checkResult("Set ExposureAuto",
                           MV_CC_SetEnumValue(handle, "ExposureAuto", 0));
    success &= checkResult("Set GainAuto",
                           MV_CC_SetEnumValue(handle, "GainAuto", 0));
    success &= checkResult("Set PixelFormat",
                           MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerRG8));
    success &= checkResult(
        "Set ADC bit depth",
        MV_CC_SetEnumValue(handle, "ADCBitDepth", config_.adcBitDepth == 8U ? 0U : 3U));
    success &= checkResult("Set Width", MV_CC_SetIntValue(
        handle, "Width", static_cast<unsigned int>(config_.frameWidth)));
    success &= checkResult("Set Height", MV_CC_SetIntValue(
        handle, "Height", static_cast<unsigned int>(config_.frameHeight)));
    success &= checkResult("Set ExposureTime",
                           MV_CC_SetFloatValue(handle, "ExposureTime", config_.exposureTime));
    success &= checkResult("Set Gain",
                           MV_CC_SetFloatValue(handle, "Gain", config_.gain));
    success &= checkResult(
        "Set acquisition frame rate enable",
        MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable",
                           config_.acquisitionFrameRateEnable));
    success &= checkResult("Set Bayer conversion quality",
                           MV_CC_SetBayerCvtQuality(handle, config_.bayerQuality));
    return success;
}

void CameraSetting::printCameraFrameRateInfo(void* handle) const {
    MVCC_FLOATVALUE value{};
    if (MV_CC_GetFloatValue(handle, "ResultingFrameRate", &value) == MV_OK) {
        std::cout << "[CAMERA] ResultingFrameRate=" << value.fCurValue << '\n';
    }
}

float CameraSetting::calculateFPS(Clock::time_point startTime,
                                  std::uint64_t frameCount,
                                  Clock::time_point now) const {
    const double seconds = std::chrono::duration<double>(now - startTime).count();
    if (seconds <= 0.0) return 0.0F;
    return static_cast<float>(static_cast<double>(frameCount) / seconds);
}

void CameraSetting::displayFPS(cv::Mat& image, float pipelineFps, double captureMs) const {
    drawMetrics(image, pipelineFps, captureMs);
}

bool CameraSetting::processFrame(const cv::Mat& frame,
                                 Clock::time_point startTime,
                                 std::uint64_t frameCount,
                                 float pipelineFps,
                                 double captureMs) {
    if (frame.empty()) return false;
    (void)startTime;
    (void)frameCount;
    return outputSink_ ? outputSink_(frame, pipelineFps, captureMs) : true;
}

bool CameraSetting::run() {
    MV_CC_DEVICE_INFO_LIST list{};
    if (!checkResult("Enumerate cameras",
                     MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list)) ||
        list.nDeviceNum == 0) {
        std::cerr << "No Hikrobot camera found.\n";
        return false;
    }
    if (config_.deviceIndex >= list.nDeviceNum) {
        std::cerr << "Camera device_index " << config_.deviceIndex
                  << " is out of range; detected " << list.nDeviceNum << " camera(s).\n";
        return false;
    }

    void* handle = nullptr;
    bool grabbing = false;
    if (!checkResult("Create camera handle",
                     MV_CC_CreateHandle(&handle, list.pDeviceInfo[config_.deviceIndex]))) {
        return false;
    }

    auto cleanup = [&]() {
        if (grabbing) MV_CC_StopGrabbing(handle);
        if (handle != nullptr) {
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
        }
        if (config_.displayEnabled) cv::destroyAllWindows();
    };

    if (!checkResult("Open camera", MV_CC_OpenDevice(handle)) ||
        !setupCamera(handle) ||
        !checkResult("Disable trigger mode",
                     MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF)) ||
        !checkResult("Start grabbing", MV_CC_StartGrabbing(handle))) {
        cleanup();
        return false;
    }
    grabbing = true;
    if (!checkResult("Run one-shot white balance",
                     MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 2))) {
        std::cerr << "Warning: one-shot white balance was not accepted.\n";
    }
    printCameraFrameRateInfo(handle);

    OutputWorker output(config_);
#ifdef RM_ENABLE_VISION
    // 视觉管线初始化（方案 §9.5）。加载失败必须明确报错并安全退出（验收标准）。
    if (config_.vision.enabled) {
        vision_ = std::make_unique<vision::VisionPipeline>(config_.vision);
        if (!vision_->start()) {
            std::cerr << "Vision pipeline failed to start; aborting.\n";
            vision_.reset();
            cleanup();
            return false;
        }
        if (config_.vision.enableSerial && !config_.vision.serialPort.empty()) {
            serial_ = std::make_unique<vision::SerialOutput>();
            vision::SerialConfig serialConfig;
            serialConfig.port = config_.vision.serialPort;
            serialConfig.baud = config_.vision.serialBaud;
            serialConfig.periodMs = config_.vision.serialPeriodMs;
            serialConfig.maxAgeMs = config_.vision.serialMaxAgeMs;
            if (!serial_->start(serialConfig, [this](vision::VisionResult& r) {
                    return vision_ != nullptr && vision_->getLatestResult(r);
                })) {
                // 串口打不开只告警，不拖死采集（方案 §10）。
                std::cerr << "Warning: serial output failed to start on "
                          << config_.vision.serialPort << '\n';
                serial_.reset();
            }
        }
        output.setSerialProvider([this] {
            return serial_ ? serial_->status() : vision::SerialStatus{};
        });
        output.setVisionProvider([this](vision::VisionResult& r) {
            return vision_ != nullptr && vision_->getLatestResult(r);
        });
        std::cout << "Vision pipeline active: backend=" << config_.vision.backend
                   << '\n';
    }
#endif
    output.start();
    outputSink_ = [&output](const cv::Mat& frame, float fps, double captureMs) {
        return output.submit(frame, fps, captureMs);
    };
    const auto startTime = Clock::now();
    std::uint64_t frameCount = 0;
    bool success = true;
    int consecutiveGrabFailures = 0;  // usbipd/USB 抖动下单次超时不该致命退出
    constexpr int kMaxConsecutiveGrabFailures = 50;

    std::cout << "Camera started; recording for " << config_.recordSeconds
              << " seconds." << '\n';
    while (!output.stopRequested() && (config_.recordSeconds <= 0.0 ||
           std::chrono::duration<double>(Clock::now() - startTime).count() <
               config_.recordSeconds)) {
        const auto captureStart = Clock::now();
        MV_FRAME_OUT frame{};
        const int result = MV_CC_GetImageBuffer(handle, &frame, config_.grabTimeoutMs);
        if (result != MV_OK) {
            // 单次超时（如 0x80000007）可能是 usbipd/USB 抖动：告警并重试，
            // 连续超过阈值才判定相机链路失效。
            ++consecutiveGrabFailures;
            if (consecutiveGrabFailures == 1 || consecutiveGrabFailures % 10 == 0) {
                std::cerr << "Get image buffer failed, error = 0x" << std::hex
                          << result << std::dec << " (consecutive "
                          << consecutiveGrabFailures << ")\n";
            }
            if (consecutiveGrabFailures >= kMaxConsecutiveGrabFailures) {
                std::cerr << "Grab failed " << consecutiveGrabFailures
                          << " times in a row; aborting.\n";
                success = false;
                break;
            }
            continue;
        }
        consecutiveGrabFailures = 0;

        ImageBufferGuard guard(handle, frame);
        const MV_FRAME_OUT_INFO_EX& info = frame.stFrameInfo;
        cv::Mat image;
        if (info.enPixelType == PixelType_Gvsp_BayerRG8) {
            image.create(static_cast<int>(info.nHeight), static_cast<int>(info.nWidth), CV_8UC3);
            MV_CC_PIXEL_CONVERT_PARAM param{};
            param.nWidth = info.nWidth;
            param.nHeight = info.nHeight;
            param.pSrcData = frame.pBufAddr;
            param.nSrcDataLen = info.nFrameLen;
            param.enSrcPixelType = info.enPixelType;
            param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            param.pDstBuffer = image.data;
            param.nDstBufferSize = static_cast<unsigned int>(image.total() * image.elemSize());
            if (MV_CC_ConvertPixelType(handle, &param) != MV_OK) {
                cv::cvtColor(cv::Mat(static_cast<int>(info.nHeight),
                                     static_cast<int>(info.nWidth), CV_8UC1,
                                     frame.pBufAddr), image, cv::COLOR_BayerRG2BGR);
            }
        } else if (info.enPixelType == PixelType_Gvsp_Mono8) {
            image.create(static_cast<int>(info.nHeight), static_cast<int>(info.nWidth), CV_8UC1);
            std::memcpy(image.data, frame.pBufAddr,
                        std::min(image.total() * image.elemSize(),
                                 static_cast<std::size_t>(info.nFrameLen)));
        } else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
            cv::cvtColor(cv::Mat(static_cast<int>(info.nHeight),
                                 static_cast<int>(info.nWidth), CV_8UC3,
                                 frame.pBufAddr), image, cv::COLOR_RGB2BGR);
        } else {
            std::cerr << "Unsupported pixel format: 0x" << std::hex
                      << info.enPixelType << std::dec << '\n';
            success = false;
            break;
        }

        ++frameCount;
        const auto captureEnd = Clock::now();
        const double captureMs = std::chrono::duration<double, std::milli>(
            captureEnd - captureStart).count();
        const float pipelineFps = calculateFPS(startTime, frameCount, captureEnd);
#ifdef RM_ENABLE_VISION
        if (vision_) {
            // 推理线程只拿最新帧：移动所有权避免整帧复制（方案 §5.1/§5.2）。
            // 显示/录像改由 OutputWorker 的 visionLoop 消费管线结果。
            vision_->submitFrame(std::move(image), captureEnd, pipelineFps);
            continue;
        }
#endif
        if (!processFrame(image, startTime, frameCount, pipelineFps, captureMs)) {
            success = false;
            break;
        }
    }

#ifdef RM_ENABLE_VISION
    if (serial_) serial_->stop();
    if (vision_) vision_->stop();
#endif
    output.stop();
    outputSink_ = {};
    cleanup();
    std::cout << "Captured " << frameCount << " frames, FPS="
              << calculateFPS(startTime, frameCount, Clock::now()) << '\n';
    return success;
}
