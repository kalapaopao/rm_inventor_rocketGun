#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include <opencv2/core/mat.hpp>

struct CameraConfig {
    unsigned int deviceIndex = 0;
    int frameWidth = 1440;
    int frameHeight = 1080;
    float exposureTime = 2500.0F;
    float gain = 9.0F;
    unsigned int grabTimeoutMs = 100;
    unsigned int bayerQuality = 1;
    unsigned int adcBitDepth = 8;
    bool acquisitionFrameRateEnable = false;
    bool displayEnabled = true;
    double displayFps = 30.0;
    std::string windowName = "Hikrobot Camera";
    double recordSeconds = 0.0;

    // 安全加载yaml
    bool load(const std::string& path);
};

class CameraSetting {
public:
    // Called once per captured BGR/gray frame. Keep the callback non-blocking.
    using FrameSink = std::function<void(const cv::Mat&)>;

    explicit CameraSetting(CameraConfig config);
    void setFrameSink(FrameSink sink);

    bool import_image();
    float calculateFPS() const;
    void displayFPS(cv::Mat& image, float fps);

private:
    using Clock = std::chrono::steady_clock;

    struct CaptureStats {
        Clock::time_point intervalStart{};
        std::uint64_t deliveredFrames = 0;
    };

    bool setupCamera(void* handle);
    void printCameraFrameRateInfo(void* handle) const;
    bool captureLoop(void* handle,
                     std::atomic<bool>& stopRequested,
                     std::mutex& frameMutex,
                     cv::Mat& latestFrame,
                     std::uint64_t& latestFrameSequence);
    void displayLoop(std::atomic<bool>& stopRequested,
                     const std::atomic<bool>& captureFinished,
                     std::mutex& frameMutex,
                     const cv::Mat& latestFrame,
                     const std::uint64_t& latestFrameSequence);
    void resetCaptureInterval(Clock::time_point now);
    void updateCaptureStats(Clock::time_point frameTime);

    CameraConfig config_;
    CaptureStats captureStats_;
    std::atomic<float> deliveredFps_{0.0F};
    FrameSink frameSink_;
};
