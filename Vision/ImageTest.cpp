// Offline debugging only: never opens a camera or serial port.
#include "TargetGeometry.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iostream>

int main(int argc, char** argv) {
    if(argc != 3) {
        std::cerr << "Usage: vision_image_test input.jpg output.png\n";
        return 2;
    }
    try {
        if(std::string(argv[1]) == argv[2]) return 2;
        cv::Mat image = cv::imread(argv[1]);
        if(image.empty()) return 1;
        const auto start=std::chrono::steady_clock::now();
        const auto lights=vision::detectGreenLights(image, {});
        const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        for(const auto& light: lights) {
            std::cout << "light=" << light.lightX << "," << light.lightY
                      << " radius=" << light.lightRadius << " area=" << light.score << '\n';
            cv::circle(image,{cvRound(light.lightX),cvRound(light.lightY)},
                       cvRound(light.lightRadius),cv::Scalar(0,255,255),2);
        }
        std::cout << "traditional detect_ms=" << ms << " circles=" << lights.size() << '\n';
        return cv::imwrite(argv[2],image) ? 0 : 1;
    } catch(const cv::Exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
