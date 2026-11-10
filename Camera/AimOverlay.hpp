#pragma once

#include <algorithm>
#include <opencv2/imgproc.hpp>

// Geometric image centre, not the calibrated optical principal point.
inline void drawCentreCrosshair(cv::Mat& image) {
    if (image.empty()) return;
    const cv::Point centre(image.cols / 2, image.rows / 2);
    const int halfLength = std::max(12, std::min(image.cols, image.rows) / 18);
    const auto draw = [&](cv::Scalar color, int thickness) {
        cv::line(image, centre - cv::Point(halfLength, 0),
                 centre + cv::Point(halfLength, 0), color, thickness, cv::LINE_AA);
        cv::line(image, centre - cv::Point(0, halfLength),
                 centre + cv::Point(0, halfLength), color, thickness, cv::LINE_AA);
    };
    draw(cv::Scalar(0, 0, 0), 4);
    draw(cv::Scalar(0, 255, 255), 2);
}
