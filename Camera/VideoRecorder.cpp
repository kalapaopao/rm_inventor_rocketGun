#include "VideoRecorder.hpp"

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <system_error>
#include <utility>

VideoRecorder::VideoRecorder(std::string outputDirectory, double fps)
    : outputDirectory_(std::move(outputDirectory)),
      fps_(fps) {}

VideoRecorder::~VideoRecorder() {
    stop();
}

bool VideoRecorder::start(const cv::Size& frameSize, bool isColor) {
    if (isRecording()) return true;
    if (fps_ <= 0.0) {
        std::cerr << "[RECORDING] Invalid FPS: " << fps_ << std::endl;
        return false;
    }
    if (frameSize.width <= 0 || frameSize.height <= 0) {
        std::cerr << "[RECORDING] Invalid frame size." << std::endl;
        return false;
    }

    std::error_code directoryError;
    std::filesystem::create_directories(outputDirectory_, directoryError);
    if (directoryError) {
        std::cerr << "[RECORDING] Create directory failed: "
                  << directoryError.message() << std::endl;
        return false;
    }

    frameInterval_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / fps_));
    outputPath_ = makeOutputPath();
    const int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    writer_.open(outputPath_, codec, fps_, frameSize, isColor);
    if (!writer_.isOpened()) {
        std::cerr << "[RECORDING] Open file failed: "
                  << outputPath_ << std::endl;
        outputPath_.clear();
        return false;
    }

    nextFrameTime_ = Clock::now();
    std::cout << "[RECORDING] Started: " << outputPath_
              << " (MJPG, " << fps_ << " FPS)" << std::endl;
    return true;
}

void VideoRecorder::write(const cv::Mat& frame) {
    if (!isRecording() || frame.empty()) return;

    const auto now = Clock::now();
    if (now < nextFrameTime_) return;

    writer_.write(frame);
    nextFrameTime_ = now + frameInterval_;
}

void VideoRecorder::stop() {
    if (!isRecording()) return;

    writer_.release();
    std::cout << "[RECORDING] Saved: " << outputPath_ << std::endl;
}

bool VideoRecorder::isRecording() const {
    return writer_.isOpened();
}

const std::string& VideoRecorder::outputPath() const {
    return outputPath_;
}

std::string VideoRecorder::makeOutputPath() const {
    const auto nowPoint = std::chrono::system_clock::now();
    const std::time_t now = std::chrono::system_clock::to_time_t(nowPoint);
    std::tm localTime{};
    localtime_r(&now, &localTime);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        nowPoint.time_since_epoch()) % 1000;

    std::ostringstream filename;
    filename << "camera_" << std::put_time(&localTime, "%Y%m%d_%H%M%S")
             << '_' << std::setw(3) << std::setfill('0') << milliseconds.count()
             << ".avi";
    return (std::filesystem::path(outputDirectory_) / filename.str()).string();
}
