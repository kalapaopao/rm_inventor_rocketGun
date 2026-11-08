#pragma once

// YOLO26 端到端模型的预处理与输出解码。
// 模型契约已通过离线实测确认（docs 方案 §9.2，2026-11-08，Python + ONNX Runtime）：
//   输入  images [1,3,640,640] FP32 NCHW RGB /255
//   输出  output0 [1,300,6] = [x1,y1,x2,y2,score,cls]，输入图(640)像素坐标，
//         端到端内置 TopK+NMS，无需再做 NMS
// 实测 4 张数据集图与 labelme 标注 IoU 0.85~0.95。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "DetectionTypes.hpp"

namespace vision {

// letterbox 参数：原图 -> 640 输入空间的缩放与填充量。
struct LetterboxParams {
    double scale = 1.0;
    int padX = 0;
    int padY = 0;
};

// Ultralytics letterbox：等比缩放 + 灰边(114)填充，输出 RGB CV_32FC3 (0..1)。
// 允许目标尺寸不是 640 以适配后续不同导出的模型。
inline cv::Mat letterbox(const cv::Mat& bgr, int targetSize, LetterboxParams& params) {
    const int w = bgr.cols;
    const int h = bgr.rows;
    params.scale = std::min(static_cast<double>(targetSize) / w,
                            static_cast<double>(targetSize) / h);
    const int nw = static_cast<int>(std::lround(w * params.scale));
    const int nh = static_cast<int>(std::lround(h * params.scale));
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);

    cv::Mat canvas(targetSize, targetSize, CV_8UC3, cv::Scalar(114, 114, 114));
    params.padX = (targetSize - nw) / 2;
    params.padY = (targetSize - nh) / 2;
    resized.copyTo(canvas(cv::Rect(params.padX, params.padY, nw, nh)));

    cv::Mat rgb;
    cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);
    cv::Mat floatRgb;
    rgb.convertTo(floatRgb, CV_32FC3, 1.0 / 255.0);
    return floatRgb;
}

// 把模型输出空间的坐标映射回原图像素坐标。
inline cv::Rect2f mapBoxToOriginal(const float x1, const float y1, const float x2,
                                   const float y2, const LetterboxParams& params) {
    return cv::Rect2f(
        static_cast<float>((x1 - params.padX) / params.scale),
        static_cast<float>((y1 - params.padY) / params.scale),
        static_cast<float>((x2 - x1) / params.scale),
        static_cast<float>((y2 - y1) / params.scale));
}

// 解码 YOLO26 端到端输出 [1,300,6]。output 大小必须为 rows*6；
// 坐标经 letterbox 参数映射回原图，低于 confThreshold 的行被丢弃。
inline void decodeEnd2End(const std::vector<float>& output,
                          const LetterboxParams& params,
                          float confThreshold,
                          std::vector<Detection>& detections) {
    constexpr int kFieldsPerRow = 6;
    if (output.empty() || output.size() % kFieldsPerRow != 0) {
        return;  // 形状不符由调用方在启动阶段校验，这里只防御异常输入。
    }
    const size_t rows = output.size() / kFieldsPerRow;
    detections.clear();
    for (size_t i = 0; i < rows; ++i) {
        const float* row = &output[i * kFieldsPerRow];
        const float score = row[4];
        if (score < confThreshold) continue;
        const cv::Rect2f box = mapBoxToOriginal(row[0], row[1], row[2], row[3], params);
        Detection det;
        det.x1 = box.x;
        det.y1 = box.y;
        det.x2 = box.x + box.width;
        det.y2 = box.y + box.height;
        det.score = score;
        det.classId = static_cast<int>(row[5]);
        detections.push_back(det);
    }
}

}  // namespace vision
