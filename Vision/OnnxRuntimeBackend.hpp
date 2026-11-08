#pragma once

// ONNX Runtime CPU 后端（方案 §6 实现顺序第一步）。
// 头文件不包含任何 ONNX Runtime 类型，保证包含本文件的代码不依赖 ORT API。

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include "InferenceBackend.hpp"

namespace vision {

class OnnxRuntimeBackend : public IInferenceBackend {
public:
    OnnxRuntimeBackend();
    ~OnnxRuntimeBackend() override;

    OnnxRuntimeBackend(const OnnxRuntimeBackend&) = delete;
    OnnxRuntimeBackend& operator=(const OnnxRuntimeBackend&) = delete;

    bool load(const std::string& modelPath) override;
    bool infer(const cv::Mat& input, std::vector<float>& output) override;
    std::string name() const override { return "onnxruntime_cpu"; }

    // load() 成功后返回模型输入尺寸，管线用于校验预处理输出。
    int inputWidth() const;
    int inputHeight() const;
    int inputChannels() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vision
