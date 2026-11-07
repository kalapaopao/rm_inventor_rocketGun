#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include <opencv2/core/mat.hpp>
#include "VideoRecorder.hpp"

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
    bool displayEnabled = false;
    std::string windowName = "Hikrobot Camera";
    double recordSeconds = 0.0;

    bool load(const std::string& path);
};

class CameraSetting {
public:
    explicit CameraSetting(CameraConfig config);

    // Open, capture, record, and display until record_seconds is reached.
    bool run();

private:
    using Clock = std::chrono::steady_clock;

    bool setupCamera(void* handle);
    void printCameraFrameRateInfo(void* handle) const;
    bool processFrame(const cv::Mat& frame,
                      Clock::time_point startTime,
                      std::uint64_t frameCount,
                      float pipelineFps,
                      double captureMs);
    float calculateFPS(Clock::time_point startTime,
                       std::uint64_t frameCount,
                       Clock::time_point now) const;
    void displayFPS(cv::Mat& image, float pipelineFps, double captureMs) const;

    CameraConfig config_;
    std::function<bool(const cv::Mat&, float, double)> outputSink_;
};
