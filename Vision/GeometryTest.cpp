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
        require(argc == 2, "expected intrinsics path");
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
        std::vector<vision::Detection> lights;
        vision::GreenLightConfig config;
        cv::circle(image, {90,100}, 15, cv::Scalar(0,255,0), -1);
        lights = vision::detectGreenLights(image,config);
        require(lights.size()==1, "green circle rejected");
        const auto d = lights.front();
        require(std::abs(d.centerX()-90)<1 && std::abs(d.centerY()-100)<1, "wrong light centre");
        image.setTo(cv::Scalar());
        cv::circle(image, {90,100}, 15, cv::Scalar(255,255,255), -1);
        require(vision::detectGreenLights(image,config).empty(), "white glare accepted");
        image.setTo(cv::Scalar());
        cv::rectangle(image, {70,60}, {80,140}, cv::Scalar(0,255,0), -1);
        require(vision::detectGreenLights(image,config).empty(), "green strip accepted");
        // Dim green and clipped lights must not be treated as valid luminous circles.
        image.setTo(cv::Scalar());
        cv::circle(image,{90,100},15,cv::Scalar(0,80,0),-1);
        require(vision::detectGreenLights(image,config).empty(), "dim green accepted");
        image.setTo(cv::Scalar());
        cv::circle(image,{0,100},15,cv::Scalar(0,255,0),-1);
        require(vision::detectGreenLights(image,config).empty(), "clipped circle accepted");
        image.setTo(cv::Scalar());
        cv::circle(image,{50,60},12,cv::Scalar(0,255,0),-1);
        cv::circle(image,{130,120},22,cv::Scalar(0,255,0),-1);
        lights=vision::detectGreenLights(image,config);
        require(lights.size()==2 && lights[0].score>lights[1].score, "multi-circle area ordering");

        vision::VisionConfig settings;
        settings.calibrationEnabled = true;
        settings.calibrationFile = argv[1];
        settings.predictionHorizonSeconds = 5;
        vision::VisionPipeline pipeline(settings);
        require(pipeline.start(), "pipeline start failed");
        std::uint64_t sequence=0;
        vision::VisionResult result;
        const auto submit = [&](const cv::Mat& frame) {
            pipeline.submitFrame(frame.clone(), vision::VisionPipeline::Clock::now(), 50);
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
            do {
                if(pipeline.getLatestResult(result) && result.sequence>sequence) {
                    sequence=result.sequence; return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            } while(std::chrono::steady_clock::now()<deadline);
            throw std::runtime_error("pipeline result timeout");
        };
        cv::Mat frame(1080,1440,CV_8UC3,cv::Scalar());
        submit(frame);
        require(!result.currentAnglesValid && !result.yawOffsetValid,"blank target valid");
        for(int i=0;i<8;++i) {
            frame.setTo(cv::Scalar());
            cv::circle(frame,{760,500},20,cv::Scalar(0,255,0),-1);
            submit(frame);
        }
        require(result.target.confirmed && result.yawOffsetValid, "green light not tracked");
        require(std::abs(result.yawOffsetPixels-40)<1, "pixel offset wrong");
        require(result.currentAnglesValid && result.target.horizonSeconds==5,"geometry/horizon wrong");
        frame.setTo(cv::Scalar()); submit(frame);
        require(!result.yawOffsetValid,"lost target still sends");
        pipeline.stop();
        std::cout << "PASS: traditional pipeline, target tracking, calibrated angles, 5s prediction, yaw invalidation\n";
        std::cout << "PASS: calibration, ray angles, size guard, green circle, glare/strip rejection\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
