#pragma once
#include <sstream>
#include <opencv2/imgproc.hpp>
#include "SerialOutput.hpp"

namespace vision {
inline void drawSerialPanel(cv::Mat& image, const VisionResult& result,
                            bool enabled, int maxAgeMs, const SerialStatus& status) {
    std::int16_t yaw = 0;
    const bool valid = selectYaw(result, std::chrono::steady_clock::now(), maxAgeMs, yaw);
    std::ostringstream candidate, sent, counts;
    candidate << "yaw_offset_px: ";
    if (valid) candidate << std::showpos << yaw;
    else candidate << "N/A (no fresh target)";
    sent << "Serial " << (!enabled ? "OFF" : (status.running ? "ON" : "STOPPED"))
         << " | last TX: ";
    if (!enabled) sent << "N/A";
    else if (!status.lastTargetValid) sent << "TARGET=0 (none)";
    else sent << "TARGET=1 yaw=" << std::showpos << status.lastYaw;
    counts << "TX frames: " << status.sent << " errors: " << status.errors;
    const std::string lines[] = {candidate.str(), sent.str(), counts.str()};
    for (int i=0; i<3; ++i) {
        const cv::Point origin(16, 34+i*32);
        cv::putText(image, lines[i], origin, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0,0,0), 4, cv::LINE_AA);
        cv::putText(image, lines[i], origin, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0,255,255), 2, cv::LINE_AA);
    }
}
} // namespace vision
