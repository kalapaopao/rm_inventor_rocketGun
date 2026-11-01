#pragma once

#include <chrono>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>

class VideoRecorder {
public:
    explicit VideoRecorder(std::string outputDirectory, double fps = 30.0);
    ~VideoRecorder();

    VideoRecorder(const VideoRecorder&) = delete;
    VideoRecorder& operator=(const VideoRecorder&) = delete;

    bool start(const cv::Size& frameSize, bool isColor = true);
    void write(const cv::Mat& frame);
    void stop();

    bool isRecording() const;
    const std::string& outputPath() const;

private:
    using Clock = std::chrono::steady_clock;

    std::string makeOutputPath() const;

    std::string outputDirectory_;
    std::string outputPath_;
    double fps_;
    Clock::duration frameInterval_{};
    Clock::time_point nextFrameTime_{};
    cv::VideoWriter writer_;
};
