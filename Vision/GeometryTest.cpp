#include "TargetGeometry.hpp"
#include "VisionPipeline.hpp"
#include "../Camera/AimOverlay.hpp"
#include <thread>
#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
    try {
        require(argc == 2 || argc == 3, "expected intrinsics path [model path]");
        vision::CameraGeometry geometry;
        require(geometry.load(argv[1]), "calibration load failed");
        cv::Point2d angle;
        require(geometry.angles({698.74054F, 487.45153F}, {1440,1080}, angle), "angle invalid");
        require(std::abs(angle.x) < 1e-5 && std::abs(angle.y) < 1e-5, "principal ray not zero");
        require(!geometry.angles({700,500}, {640,480}, angle), "size mismatch accepted");
        require(!geometry.angles({-1,500}, {1440,1080}, angle), "outside pixel accepted");
        cv::Mat image(200,200,CV_8UC3,cv::Scalar());
        drawCentreCrosshair(image);
        require(image.at<cv::Vec3b>(100,100)[1] > 0, "crosshair centre missing");
        require(image.at<cv::Vec3b>(100,110)[1] > 0 &&
                image.at<cv::Vec3b>(110,100)[1] > 0, "crosshair arms missing");
        image.setTo(cv::Scalar());
        vision::Detection d; d.x1=20; d.y1=20; d.x2=180; d.y2=180;
        vision::GreenLightConfig config;
        cv::circle(image, {90,100}, 15, cv::Scalar(0,255,0), -1);
        require(vision::refineGreenLight(image,d,config), "green circle rejected");
        require(std::abs(d.centerX()-90)<1 && std::abs(d.centerY()-100)<1, "wrong light centre");
        image.setTo(cv::Scalar());
        cv::circle(image, {90,100}, 15, cv::Scalar(255,255,255), -1);
        require(!vision::refineGreenLight(image,d,config), "white glare accepted");
        image.setTo(cv::Scalar());
        cv::rectangle(image, {70,60}, {80,140}, cv::Scalar(0,255,0), -1);
        require(!vision::refineGreenLight(image,d,config), "green strip accepted");
        if (argc == 3) {
            vision::VisionConfig settings;
            settings.modelPath = argv[2];
            settings.calibrationEnabled = true;
            settings.calibrationFile = argv[1];
            vision::VisionPipeline pipeline(settings);
            require(pipeline.start(), "pipeline start failed");
            pipeline.submitFrame(cv::Mat(1080,1440,CV_8UC3,cv::Scalar()),
                                 vision::VisionPipeline::Clock::now(), 1);
            vision::VisionResult result;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (!pipeline.getLatestResult(result) && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            require(result.sequence > 0 && !result.annotated.empty(), "no pipeline result");
            require(!result.currentAnglesValid, "blank frame produced valid target angles");
            pipeline.stop();
            std::cout << "PASS: calibrated ONNX pipeline result publication and clean stop\n";
        }
        std::cout << "PASS: calibration, ray angles, size guard, green circle, glare/strip rejection\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
