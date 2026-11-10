#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "DetectionTypes.hpp"

namespace vision {

// Initial conservative thresholds; tune with real camera footage, not FPS alone.
struct GreenLightConfig {
    int hueMin = 35, hueMax = 85, saturationMin = 60, valueMin = 60;
    double minArea = 8.0, minCircularity = 0.65, minAxisRatio = 0.65;
};

inline bool refineGreenLight(const cv::Mat& image, Detection& detection,
                            const GreenLightConfig& config) {
    if (image.type() != CV_8UC3) return false;
    if (!std::isfinite(detection.x1) || !std::isfinite(detection.x2) ||
        !std::isfinite(detection.y1) || !std::isfinite(detection.y2) ||
        detection.x2 <= detection.x1 || detection.y2 <= detection.y1) return false;
    // Clamp in floating point before integer conversion (malformed model output).
    const cv::Rect roi = cv::Rect(
        cv::Point(static_cast<int>(std::floor(std::clamp(detection.x1, 0.0F, float(image.cols)))),
                  static_cast<int>(std::floor(std::clamp(detection.y1, 0.0F, float(image.rows))))),
        cv::Point(static_cast<int>(std::ceil(std::clamp(detection.x2, 0.0F, float(image.cols)))),
                  static_cast<int>(std::ceil(std::clamp(detection.y2, 0.0F, float(image.rows)))))) &
        cv::Rect(0, 0, image.cols, image.rows);
    if (roi.width < 3 || roi.height < 3) return false;
    cv::Mat hsv, mask;
    cv::cvtColor(image(roi), hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(config.hueMin, config.saturationMin, config.valueMin),
                cv::Scalar(config.hueMax, 255, 255), mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double bestArea = 0.0;
    detection.lightRefined = false;
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        const double perimeter = cv::arcLength(contour, true);
        if (area < config.minArea || perimeter <= 0.0 || contour.size() < 5) continue;
        const auto bounds = cv::boundingRect(contour);
        // Reject clipped shapes: their centre and circularity are unreliable.
        if (bounds.x <= 0 || bounds.y <= 0 || bounds.br().x >= roi.width ||
            bounds.br().y >= roi.height) continue;
        const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        const auto ellipse = cv::fitEllipse(contour);
        const double major = std::max(ellipse.size.width, ellipse.size.height);
        const double minor = std::min(ellipse.size.width, ellipse.size.height);
        if (circularity < config.minCircularity || major <= 0 ||
            minor / major < config.minAxisRatio || area <= bestArea) continue;
        bestArea = area;
        detection.lightX = ellipse.center.x + roi.x;
        detection.lightY = ellipse.center.y + roi.y;
        detection.lightRadius = static_cast<float>((major + minor) * 0.25);
        detection.lightRefined = true;
    }
    return detection.lightRefined;
}

class CameraGeometry {
public:
    bool load(const std::string& path) {
        valid_ = false;
        try {
            cv::FileStorage file(path, cv::FileStorage::READ);
            if (!file.isOpened()) return false;
            file["image_width"] >> size_.width;
            file["image_height"] >> size_.height;
            file["camera_matrix"] >> matrix_;
            file["distortion_coefficients"] >> distortion_;
            if (size_.width <= 0 || size_.height <= 0 || matrix_.rows != 3 ||
                matrix_.cols != 3 || matrix_.channels() != 1 || distortion_.empty() ||
                (distortion_.rows != 1 && distortion_.cols != 1) ||
                distortion_.channels() != 1 ||
                (distortion_.total() != 4 && distortion_.total() != 5 &&
                 distortion_.total() != 8 && distortion_.total() != 12 &&
                 distortion_.total() != 14)) return false;
            matrix_.convertTo(matrix_, CV_64F);
            distortion_.convertTo(distortion_, CV_64F);
            if (!cv::checkRange(matrix_) || !cv::checkRange(distortion_) ||
                matrix_.at<double>(0, 0) <= 0 || matrix_.at<double>(1, 1) <= 0 ||
                std::abs(matrix_.at<double>(2, 2) - 1.0) > 1e-6) return false;
            valid_ = true;
            std::cout << "[Calibration] loaded " << path << " size=" << size_ << '\n';
            return true;
        } catch (const cv::Exception& e) {
            std::cerr << "[Calibration] " << e.what() << '\n';
            return false;
        }
    }

    // Keep detector/tracker/serial in raw pixels. Only convert final points to
    // optical-frame sight angles: right/down positive, radians, not 3D position.
    bool angles(cv::Point2f pixel, cv::Size frameSize, cv::Point2d& angle) const {
        if (!valid_ || frameSize != size_ || !std::isfinite(pixel.x) ||
            !std::isfinite(pixel.y) || pixel.x < 0 || pixel.y < 0 ||
            pixel.x >= size_.width || pixel.y >= size_.height) return false;
        std::vector<cv::Point2f> input{pixel}, normalized;
        cv::undistortPoints(input, normalized, matrix_, distortion_);
        angle = {std::atan(normalized[0].x), std::atan(normalized[0].y)};
        return std::isfinite(angle.x) && std::isfinite(angle.y);
    }
    cv::Size imageSize() const { return size_; }
private:
    bool valid_ = false;
    cv::Size size_;
    cv::Mat matrix_, distortion_;
};
}  // namespace vision
