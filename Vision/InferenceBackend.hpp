#pragma once

// 统一推理后端接口（方案 §6，签名与文档一致）。
// 相机与跟踪代码只依赖本接口，不依赖 ONNX Runtime / CVI Runtime 的具体 API，
// 后续替换 GPU/NPU 后端时不修改相机和跟踪代码。

#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace vision {

class IInferenceBackend {
public:
    virtual ~IInferenceBackend() = default;

    // 加载模型；失败时必须返回 false，由调用方决定报错退出。
    virtual bool load(const std::string& modelPath) = 0;

    // 单次推理。输入为预处理后的模型输入图，输出为模型输出张量按行主序
    // 展开的原始数据。输出张量的形状由具体后端在 load 时校验、
    // 由管线在首次推理时按已知格式解码。返回 false 表示推理失败。
    virtual bool infer(const cv::Mat& input,
                       std::vector<float>& output) = 0;

    // 后端名称，用于日志与 vision.backend 配置的匹配。
    virtual std::string name() const = 0;
};

}  // namespace vision
