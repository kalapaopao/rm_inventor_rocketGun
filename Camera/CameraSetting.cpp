#include "CameraSetting.hpp"

#include <MvCameraControl.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cstring>
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

    void start() { worker_ = std::thread(&OutputWorker::loop, this); }

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
    output.start();
    outputSink_ = [&output](const cv::Mat& frame, float fps, double captureMs) {
        return output.submit(frame, fps, captureMs);
    };
    const auto startTime = Clock::now();
    std::uint64_t frameCount = 0;
    bool success = true;

    std::cout << "Camera started; recording for " << config_.recordSeconds
              << " seconds." << '\n';
    while (config_.recordSeconds <= 0.0 ||
           std::chrono::duration<double>(Clock::now() - startTime).count() <
               config_.recordSeconds) {
        const auto captureStart = Clock::now();
        MV_FRAME_OUT frame{};
        const int result = MV_CC_GetImageBuffer(handle, &frame, config_.grabTimeoutMs);
        if (result != MV_OK) {
            std::cerr << "Get image buffer failed, error = 0x"
                      << std::hex << result << std::dec << '\n';
            success = false;
            break;
        }

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
        if (!processFrame(image, startTime, frameCount, pipelineFps, captureMs)) {
            success = false;
            break;
        }
    }

    output.stop();
    outputSink_ = {};
    cleanup();
    std::cout << "Captured " << frameCount << " frames, FPS="
              << calculateFPS(startTime, frameCount, Clock::now()) << '\n';
    return success;
}
