// 单图推理测试工具（方案 §9.4）。
// 用途：不接相机，加载 best.onnx 对单张图片做推理，打印检测框与耗时，
// 叠框结果写到 <image>_vision_test.jpg，便于与离线 Python 验证结果对照。
//
// 用法: vision_single_test <model.onnx> <image> [confidence]

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "OnnxRuntimeBackend.hpp"
#include "Preprocess.hpp"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: vision_single_test <model.onnx> <image> [confidence]\n";
        return 2;
    }
    const std::string modelPath = argv[1];
    const std::string imagePath = argv[2];
    float conf = 0.35F;
    if (argc >= 4) conf = std::strtof(argv[3], nullptr);

    vision::OnnxRuntimeBackend backend;
    if (!backend.load(modelPath)) {
        std::cerr << "model load failed: " << modelPath << '\n';
        return 1;
    }

    const cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "cannot read image: " << imagePath << '\n';
        return 1;
    }

    vision::LetterboxParams params;
    const cv::Mat input = vision::letterbox(image, backend.inputWidth(), params);

    std::vector<float> output;
    const auto t0 = std::chrono::steady_clock::now();
    if (!backend.infer(input, output)) {
        std::cerr << "inference failed\n";
        return 1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double inferMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<vision::Detection> detections;
    vision::decodeEnd2End(output, params, conf, detections);

    std::cout << "backend=" << backend.name() << " infer=" << inferMs << " ms, "
              << output.size() << " output floats, " << detections.size()
              << " detections (conf>=" << conf << ")\n";
    for (const vision::Detection& d : detections) {
        std::cout << "  box=(" << d.x1 << "," << d.y1 << "," << d.x2 << "," << d.y2
                  << ") score=" << d.score << " cls=" << d.classId << '\n';
    }

    cv::Mat annotated = image.clone();
    for (const vision::Detection& d : detections) {
        cv::rectangle(annotated,
                      cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
                      cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
                      cv::Scalar(60, 60, 255), 2);
        cv::putText(annotated, std::to_string(d.score).substr(0, 5),
                    cv::Point(static_cast<int>(d.x1),
                              std::max(0, static_cast<int>(d.y1) - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60, 60, 255), 1);
    }
    const std::string outPath = imagePath + "_vision_test.jpg";
    cv::imwrite(outPath, annotated);
    std::cout << "annotated image saved to " << outPath << '\n';
    return 0;
}
