#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include "VideoRecorder.hpp"
// VisionResult 等结构体为纯头文件定义，不依赖推理库，可无条件包含；
// 只有 VisionPipeline/SerialOutput 的实例化需要 RM_ENABLE_VISION（链接 vision_core）。
#include "VisionPipeline.hpp"
#include "SerialOutput.hpp"

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

#ifdef RM_ENABLE_VISION
    vision::VisionConfig vision;  // 默认 enabled=false，行为与原纯取流一致
#endif

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

#ifdef RM_ENABLE_VISION
    // 视觉管线与串口输出：vision.enabled=1 时才创建（方案 §8/§12）。
    std::unique_ptr<vision::VisionPipeline> vision_;
    std::unique_ptr<vision::SerialOutput> serial_;
#endif
};
