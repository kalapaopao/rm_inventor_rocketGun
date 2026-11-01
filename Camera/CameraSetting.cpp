#include "CameraSetting.hpp"
#include "VideoRecorder.hpp"

#include <MvCameraControl.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>

#ifndef DEFAULT_RECORDING_DIR
#define DEFAULT_RECORDING_DIR "recordings"
#endif

namespace {

bool checkResult(const char* operation, int result) {
    if (result == MV_OK) return true;
    std::cerr << operation << " failed, error = 0x"
              << std::hex << result << std::dec << std::endl;
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
    int release() {
        if (!active_) return MV_OK;
        active_ = false;
        return MV_CC_FreeImageBuffer(handle_, &frame_);
    }
private:
    void* handle_;
    MV_FRAME_OUT& frame_;
    bool active_ = true;
};

}  // namespace

bool CameraConfig::load(const std::string& path) {
    // OpenCV 提供的文件存储类，能够读取 OpenCV 支持的 YAML、XML 等配置格式。
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) return false;
    // 把这个节点保存为 camera 变量。
    const cv::FileNode camera = storage["camera"];
    // 它判断的是 OpenCV 有没有成功打开文件，而不是单独判断字符串是否为空、文件是否存在。
    if (camera.empty()) {
        std::cerr << "Missing 'camera' section in config: " << path << std::endl;
        return false;
    }

    int deviceIndexValue = static_cast<int>(deviceIndex);
    int grabTimeoutValue = static_cast<int>(grabTimeoutMs);
    int bayerQualityValue = static_cast<int>(bayerQuality);
    int adcBitDepthValue = static_cast<int>(adcBitDepth);
    int acquisitionFrameRateEnableValue =
        acquisitionFrameRateEnable ? 1 : 0;
    int displayEnabledValue = displayEnabled ? 1 : 0;

    if (!camera["device_index"].empty()) camera["device_index"] >> deviceIndexValue;
    if (!camera["width"].empty()) camera["width"] >> frameWidth;
    if (!camera["height"].empty()) camera["height"] >> frameHeight;
    if (!camera["exposure_time"].empty()) camera["exposure_time"] >> exposureTime;
    if (!camera["gain"].empty()) camera["gain"] >> gain;
    if (!camera["grab_timeout_ms"].empty()) camera["grab_timeout_ms"] >> grabTimeoutValue;
    if (!camera["bayer_quality"].empty()) camera["bayer_quality"] >> bayerQualityValue;
    if (!camera["adc_bit_depth"].empty()) camera["adc_bit_depth"] >> adcBitDepthValue;
    if (!camera["acquisition_frame_rate_enable"].empty()) {
        camera["acquisition_frame_rate_enable"] >> acquisitionFrameRateEnableValue;
    }
    if (!camera["display_enabled"].empty()) camera["display_enabled"] >> displayEnabledValue;
    if (!camera["display_fps"].empty()) camera["display_fps"] >> displayFps;
    if (!camera["window_name"].empty()) camera["window_name"] >> windowName;
    if (!camera["record_seconds"].empty()) camera["record_seconds"] >> recordSeconds;

    if (deviceIndexValue < 0 || frameWidth <= 0 || frameHeight <= 0 ||
        exposureTime <= 0.0F || gain < 0.0F || grabTimeoutValue <= 0 ||
        bayerQualityValue < 0 || bayerQualityValue > 3 ||
        (adcBitDepthValue != 8 && adcBitDepthValue != 12) ||
        (acquisitionFrameRateEnableValue != 0 &&
         acquisitionFrameRateEnableValue != 1) ||
        (displayEnabledValue != 0 && displayEnabledValue != 1) ||
        (displayEnabledValue == 1 && displayFps <= 0.0) ||
        recordSeconds < 0.0 ||
        windowName.empty()) {
        std::cerr << "Invalid value in camera config: " << path << std::endl;
        return false;
    }

    deviceIndex = static_cast<unsigned int>(deviceIndexValue);
    grabTimeoutMs = static_cast<unsigned int>(grabTimeoutValue);
    bayerQuality = static_cast<unsigned int>(bayerQualityValue);
    adcBitDepth = static_cast<unsigned int>(adcBitDepthValue);
    acquisitionFrameRateEnable = acquisitionFrameRateEnableValue == 1;
    displayEnabled = displayEnabledValue == 1;
    return true;
}

CameraSetting::CameraSetting(CameraConfig config)
    : config_(std::move(config)) {}

void CameraSetting::setFrameSink(FrameSink sink) {
    frameSink_ = std::move(sink);
}

bool CameraSetting::setupCamera(void* handle) {
    bool success = true;
    success &= checkResult("Set ExposureAuto",
                           MV_CC_SetEnumValue(handle, "ExposureAuto", 0));
    success &= checkResult("Set GainAuto",
                           MV_CC_SetEnumValue(handle, "GainAuto", 0));
    success &= checkResult("Set PixelFormat",
                           MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BayerRG8));
    const unsigned int adcBitDepthValue =
        config_.adcBitDepth == 8U ? 0U : 3U;
    success &= checkResult(
        "Set ADC bit depth",
        MV_CC_SetEnumValue(handle, "ADCBitDepth", adcBitDepthValue));
    success &= checkResult("Set Width",
                           MV_CC_SetIntValue(handle, "Width", static_cast<unsigned int>(config_.frameWidth)));
    success &= checkResult("Set Height",
                           MV_CC_SetIntValue(handle, "Height", static_cast<unsigned int>(config_.frameHeight)));
    success &= checkResult("Set ExposureTime",
                           MV_CC_SetFloatValue(handle, "ExposureTime", config_.exposureTime));
    success &= checkResult("Set Gain",
                           MV_CC_SetFloatValue(handle, "Gain", config_.gain));
    success &= checkResult(
        "Set acquisition frame rate enable",
        MV_CC_SetBoolValue(
            handle, "AcquisitionFrameRateEnable",
            config_.acquisitionFrameRateEnable));
    success &= checkResult("Set Bayer conversion quality",
                           MV_CC_SetBayerCvtQuality(handle, config_.bayerQuality));
    return success;
}

void CameraSetting::printCameraFrameRateInfo(void* handle) const {
    bool frameRateEnabled = false;
    int result = MV_CC_GetBoolValue(
        handle, "AcquisitionFrameRateEnable", &frameRateEnabled);
    if (result == MV_OK) {
        std::cout << "[CAMERA] AcquisitionFrameRateEnable="
                  << std::boolalpha << frameRateEnabled << std::noboolalpha
                  << std::endl;
    } else {
        std::cout << "[CAMERA] AcquisitionFrameRateEnable unavailable, error=0x"
                  << std::hex << result << std::dec << std::endl;
    }

    MVCC_FLOATVALUE value;
    std::memset(&value, 0, sizeof(value));
    result = MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &value);
    if (result == MV_OK) {
        std::cout << "[CAMERA] AcquisitionFrameRate current="
                  << value.fCurValue << ", min=" << value.fMin
                  << ", max=" << value.fMax << std::endl;
    } else {
        std::cout << "[CAMERA] AcquisitionFrameRate unavailable, error=0x"
                  << std::hex << result << std::dec << std::endl;
    }

    std::memset(&value, 0, sizeof(value));
    result = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &value);
    if (result == MV_OK) {
        std::cout << "[CAMERA] ResultingFrameRate=" << value.fCurValue << std::endl;
    } else {
        std::cout << "[CAMERA] ResultingFrameRate unavailable, error=0x"
                  << std::hex << result << std::dec << std::endl;
    }
}

void CameraSetting::resetCaptureInterval(Clock::time_point now) {
    captureStats_ = CaptureStats{};
    captureStats_.intervalStart = now;
}

void CameraSetting::updateCaptureStats(Clock::time_point frameTime) {
    ++captureStats_.deliveredFrames;
    const double seconds = std::chrono::duration<double>(
        frameTime - captureStats_.intervalStart).count();
    if (seconds < 1.0) return;

    const float delivered = static_cast<float>(
        static_cast<double>(captureStats_.deliveredFrames) / seconds);
    deliveredFps_.store(delivered, std::memory_order_relaxed);
    std::cout << std::fixed << std::setprecision(1)
              << "[CAPTURE] fps=" << delivered << std::endl;
    resetCaptureInterval(frameTime);
}
bool CameraSetting::captureLoop(
    void* handle,
    std::atomic<bool>& stopRequested,
    std::mutex& frameMutex,
    cv::Mat& latestFrame,
    std::uint64_t& latestFrameSequence) {
    cv::Mat producerFrame;

    while (!stopRequested.load(std::memory_order_relaxed)) {
        MV_FRAME_OUT frame{};
        const int timeoutResult =
            MV_CC_GetImageBuffer(handle, &frame, config_.grabTimeoutMs);
        if (timeoutResult != MV_OK) {
            if (stopRequested.load(std::memory_order_relaxed)) break;
            std::cerr << "Get image buffer failed, error = 0x"
                      << std::hex << timeoutResult << std::dec << std::endl;
            return false;
        }

        ImageBufferGuard guard(handle, frame);
        const MV_FRAME_OUT_INFO_EX& info = frame.stFrameInfo;
        int result = MV_OK;

        if (info.enPixelType == PixelType_Gvsp_BayerRG8) {
            producerFrame.create(static_cast<int>(info.nHeight),
                                 static_cast<int>(info.nWidth), CV_8UC3);
            MV_CC_PIXEL_CONVERT_PARAM param{};
            param.nWidth = info.nWidth;
            param.nHeight = info.nHeight;
            param.pSrcData = frame.pBufAddr;
            param.nSrcDataLen = info.nFrameLen;
            param.enSrcPixelType = info.enPixelType;
            param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            param.pDstBuffer = producerFrame.data;
            param.nDstBufferSize = static_cast<unsigned int>(
                producerFrame.total() * producerFrame.elemSize());

            result = MV_CC_ConvertPixelType(handle, &param);
            if (result != MV_OK) {
                static bool warned = false;
                if (!warned) {
                    warned = true;
                    std::cout << "[CAMERA] MVS Bayer conversion unavailable; "
                                 "using OpenCV fallback (error=0x"
                              << std::hex << result << std::dec << ")" << std::endl;
                }
                cv::cvtColor(cv::Mat(static_cast<int>(info.nHeight),
                                     static_cast<int>(info.nWidth), CV_8UC1,
                                     frame.pBufAddr),
                             producerFrame, cv::COLOR_BayerRG2BGR);
            }
        } else if (info.enPixelType == PixelType_Gvsp_Mono8) {
            producerFrame.create(static_cast<int>(info.nHeight),
                                 static_cast<int>(info.nWidth), CV_8UC1);
            std::memcpy(producerFrame.data, frame.pBufAddr,
                        std::min(producerFrame.total() * producerFrame.elemSize(),
                                 static_cast<std::size_t>(info.nFrameLen)));
        } else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
            const cv::Mat rgb(static_cast<int>(info.nHeight),
                              static_cast<int>(info.nWidth), CV_8UC3,
                              frame.pBufAddr);
            cv::cvtColor(rgb, producerFrame, cv::COLOR_RGB2BGR);
        } else {
            std::cerr << "Unsupported pixel format: 0x"
                      << std::hex << info.enPixelType << std::dec << std::endl;
            return false;
        }

        if (!checkResult("Free image buffer", guard.release())) return false;
        if (frameSink_) frameSink_(producerFrame);

        {
            std::lock_guard<std::mutex> lock(frameMutex);
            std::swap(producerFrame, latestFrame);
            ++latestFrameSequence;
        }
        updateCaptureStats(Clock::now());
    }
    return true;
}
void CameraSetting::displayLoop(
    std::atomic<bool>& stopRequested,
    const std::atomic<bool>& captureFinished,
    std::mutex& frameMutex,
    const cv::Mat& latestFrame,
    const std::uint64_t& latestFrameSequence) {
    const auto interval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config_.displayFps));
    VideoRecorder recorder(DEFAULT_RECORDING_DIR, 30.0);
    cv::Mat displayFrame;
    std::uint64_t displayedSequence = 0;
    auto nextDisplay = Clock::now();

    const bool autoRecord = config_.recordSeconds > 0.0;
    const auto autoRecordDuration = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(config_.recordSeconds));
    Clock::time_point recordDeadline{};
    bool autoRecordStarted = false;

    while (!stopRequested.load(std::memory_order_relaxed) &&
           !captureFinished.load(std::memory_order_acquire)) {
        const auto now = Clock::now();
        if (now < nextDisplay) {
            std::this_thread::sleep_until(nextDisplay);
            continue;
        }
        nextDisplay = std::max(nextDisplay + interval, now);

        bool hasNewFrame = false;
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            if (latestFrameSequence != displayedSequence && !latestFrame.empty()) {
                latestFrame.copyTo(displayFrame);
                displayedSequence = latestFrameSequence;
                hasNewFrame = true;
            }
        }
        if (!hasNewFrame) continue;

        if (autoRecord && !autoRecordStarted) {
            if (recorder.start(displayFrame.size(), displayFrame.channels() != 1)) {
                autoRecordStarted = true;
                recordDeadline = Clock::now() + autoRecordDuration;
                std::cout << "[RECORD] started: " << recorder.outputPath() << std::endl;
            } else {
                std::cerr << "[RECORD] failed to start automatic recording" << std::endl;
                stopRequested.store(true, std::memory_order_relaxed);
                break;
            }
        }
        if (autoRecord && autoRecordStarted && recorder.isRecording() &&
            Clock::now() >= recordDeadline) {
            recorder.stop();
            stopRequested.store(true, std::memory_order_relaxed);
            break;
        }

        recorder.write(displayFrame);
        displayFPS(displayFrame, calculateFPS());
        int key = -1;
        if (config_.displayEnabled) {
            cv::imshow(config_.windowName, displayFrame);
            key = cv::waitKey(1);
        }

        if (key == 'r' || key == 'R') {
            if (recorder.isRecording()) recorder.stop();
            else recorder.start(displayFrame.size(), displayFrame.channels() != 1);
        } else if (key == 27 || key == 'q' || key == 'Q') {
            recorder.stop();
            stopRequested.store(true, std::memory_order_relaxed);
        }
    }
    recorder.stop();
}
bool CameraSetting::import_image() {
    int result = MV_OK;
    void* handle = nullptr;
    // 才把 grabbing 改为 true
    bool grabbing = false;

    // 创建一个设备信息列表变量，后面由 SDK 填充检测到的相机信息。
    MV_CC_DEVICE_INFO_LIST list;
    // 在调用 SDK 之前，把设备列表结构体的内存清零。
    std::memset(&list, 0, sizeof(list));
    
    // 它调用海康 SDK，枚举当前计算机能够发现的工业相机。
    result = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
    if (!checkResult("Enumerate cameras", result) || list.nDeviceNum == 0) {
        std::cerr << "No Hikrobot camera found." << std::endl;
        return false;
    }
    if (config_.deviceIndex >= list.nDeviceNum) {
        std::cerr << "Camera device_index " << config_.deviceIndex
                  << " is out of range; detected " << list.nDeviceNum
                  << " camera(s)." << std::endl;
        return false;
    }
    // SDK 会给 handle 写入有效的句柄值。
    result = MV_CC_CreateHandle(&handle, list.pDeviceInfo[config_.deviceIndex]);
    if (!checkResult("Create camera handle", result)) return false;

    auto cleanup = [&]() {
        if (grabbing) {
            MV_CC_StopGrabbing(handle);
            grabbing = false;
        }
        if (handle != nullptr) {
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            handle = nullptr;
        }
        cv::destroyAllWindows();
    };

    result = MV_CC_OpenDevice(handle);
    if (!checkResult("Open camera", result)) {
        cleanup();
        return false;
    }
    if (!setupCamera(handle)) {
        cleanup();
        return false;
    }
    result = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    if (!checkResult("Disable trigger mode", result)) {
        cleanup();
        return false;
    }
    result = MV_CC_StartGrabbing(handle);
    if (!checkResult("Start grabbing", result)) {
        cleanup();
        return false;
    }
    grabbing = true;

    printCameraFrameRateInfo(handle);
    resetCaptureInterval(Clock::now());

    std::cout << "Camera started: " << config_.frameWidth << "x"
              << config_.frameHeight << ", exposure=" << config_.exposureTime
              << ", gain=" << config_.gain
              << ", bayer_quality=" << config_.bayerQuality
              << ", adc_bit_depth=" << config_.adcBitDepth
              << ", display=" << (config_.displayEnabled ? "on" : "off");
    if (config_.displayEnabled) {
        std::cout << "@" << config_.displayFps << " FPS";
    }
    std::cout << "." << std::endl;

    std::atomic<bool> stopRequested{false};
    std::atomic<bool> captureFinished{false};
    std::atomic<bool> captureSuccess{true};
    std::mutex frameMutex;
    cv::Mat latestFrame;
    std::uint64_t sequence = 0;

    std::thread captureThread([&]() {
        const bool success = captureLoop(
            handle, stopRequested, frameMutex, latestFrame, sequence);
        captureSuccess.store(success, std::memory_order_relaxed);
        if (!success) stopRequested.store(true, std::memory_order_relaxed);
        captureFinished.store(true, std::memory_order_release);
    });

    if (config_.displayEnabled || config_.recordSeconds > 0.0) {
        displayLoop(
            stopRequested, captureFinished, frameMutex, latestFrame, sequence);
    } else {
        while (!captureFinished.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    stopRequested.store(true, std::memory_order_relaxed);
    captureThread.join();
    cleanup();
    return captureSuccess.load(std::memory_order_relaxed);
}

float CameraSetting::calculateFPS() const {
    return deliveredFps_.load(std::memory_order_relaxed);
}

void CameraSetting::displayFPS(cv::Mat& image, float fps) {
    std::ostringstream text;
    text << "Capture FPS: " << std::fixed << std::setprecision(1) << fps;
    cv::putText(image, text.str(), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0),
                2, cv::LINE_AA);
}
