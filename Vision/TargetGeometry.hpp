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
    int hueMin = 35, hueMax = 85, saturationMin = 60, valueMin = 160;
    double minArea = 8.0, maxArea = 100000.0, minCircularity = 0.65, minAxisRatio = 0.65;
};

// Full-frame traditional detector: green luminous mask -> external contours ->
// area/circularity/ellipse-axis gates -> fitted light centres. No trained model.
inline std::vector<Detection> detectGreenLights(const cv::Mat& image,
                                               const GreenLightConfig& config) {
    std::vector<Detection> lights;
    if (image.type() != CV_8UC3) return lights;
    cv::Mat hsv, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(config.hueMin, config.saturationMin, config.valueMin),
                cv::Scalar(config.hueMax, 255, 255), mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        const double perimeter = cv::arcLength(contour, true);
        if (area < config.minArea || area > config.maxArea || perimeter <= 0 || contour.size() < 5) continue;
        const auto bounds = cv::boundingRect(contour);
        if (bounds.x <= 0 || bounds.y <= 0 || bounds.br().x >= image.cols ||
            bounds.br().y >= image.rows) continue;
        const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (circularity < config.minCircularity) continue;
        const auto ellipse = cv::fitEllipse(contour);
        const double major = std::max(ellipse.size.width, ellipse.size.height);
        const double minor = std::min(ellipse.size.width, ellipse.size.height);
        if (major <= 0 || minor/major < config.minAxisRatio) continue;
        Detection light;
        light.x1 = bounds.x; light.y1 = bounds.y;
        light.x2 = bounds.br().x; light.y2 = bounds.br().y;
        light.score = static_cast<float>(area); // contour area, NOT model confidence
        light.lightX = ellipse.center.x; light.lightY = ellipse.center.y;
        light.lightRadius = static_cast<float>((major+minor)*0.25);
        light.lightRefined = true; light.refinePassed = true;
        lights.push_back(light);
    }
    // First acquisition prefers the largest qualifying light. Tracking associates
    // subsequent frames by position, so contour enumeration order is irrelevant.
    std::sort(lights.begin(), lights.end(), [](const Detection& a, const Detection& b) {
        return a.score > b.score;
    });
    return lights;
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
