#include "OnnxRuntimeBackend.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

namespace vision {

namespace {

// 输入约定（与 VisionPipeline::letterbox 对齐）：
// CV_32FC3、HWC、RGB、数值 0..1。这里只负责 HWC -> NCHW 的排布转换，
// 保证后端不关心任何 YOLO 细节。
void packNchw(const cv::Mat& hwc, float* dst) {
    const int channels = hwc.channels();
    const int rows = hwc.rows;
    const int cols = hwc.cols;
    const size_t plane = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    for (int c = 0; c < channels; ++c) {
        float* planePtr = dst + plane * static_cast<size_t>(c);
        for (int y = 0; y < rows; ++y) {
            const float* row = hwc.ptr<float>(y);
            float* out = planePtr + static_cast<size_t>(y) * static_cast<size_t>(cols);
            for (int x = 0; x < cols; ++x) {
                out[x] = row[static_cast<size_t>(x) * static_cast<size_t>(channels) + static_cast<size_t>(c)];
            }
        }
    }
}

}  // namespace

struct OnnxRuntimeBackend::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "vision-onnxruntime"};
    Ort::Session session{nullptr};
    int width = 0;
    int height = 0;
    int channels = 0;
    std::vector<float> inputBlob;

    bool ready() const { return width > 0 && height > 0 && channels > 0; }
};

OnnxRuntimeBackend::OnnxRuntimeBackend() : impl_(std::make_unique<Impl>()) {}
OnnxRuntimeBackend::~OnnxRuntimeBackend() = default;

int OnnxRuntimeBackend::inputWidth() const { return impl_->width; }
int OnnxRuntimeBackend::inputHeight() const { return impl_->height; }
int OnnxRuntimeBackend::inputChannels() const { return impl_->channels; }

bool OnnxRuntimeBackend::load(const std::string& modelPath) {
    try {
        Ort::SessionOptions options;
        // CPU 后端保持默认线程数即可；OpenCV 采集和显示已在抢核，
        // 过大的 intra-op 并发反而拉高尾延迟。
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        impl_->session = Ort::Session(impl_->env, modelPath.c_str(), options);

        const Ort::TypeInfo inputInfo = impl_->session.GetInputTypeInfo(0);
        const auto tensorInfo = inputInfo.GetTensorTypeAndShapeInfo();
        const std::vector<int64_t> inputDims = tensorInfo.GetShape();
        if (inputDims.size() != 4) {
            std::cerr << "[OnnxRuntimeBackend] unexpected input rank " << inputDims.size()
                      << " for " << modelPath << '\n';
            return false;
        }
        // NCHW 布局；batch 固定为 1 的静态模型。
        impl_->channels = static_cast<int>(inputDims[1]);
        impl_->height = static_cast<int>(inputDims[2]);
        impl_->width = static_cast<int>(inputDims[3]);
        impl_->inputBlob.assign(
            static_cast<size_t>(impl_->channels) * static_cast<size_t>(impl_->height) *
                static_cast<size_t>(impl_->width),
            0.0F);

        std::cout << "[OnnxRuntimeBackend] loaded " << modelPath << ", input NCHW=["
                  << inputDims[0] << "," << impl_->channels << "," << impl_->height << ","
                  << impl_->width << "]\n";
        return true;
    } catch (const Ort::Exception& e) {
        std::cerr << "[OnnxRuntimeBackend] load failed: " << e.what() << '\n';
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[OnnxRuntimeBackend] load failed: " << e.what() << '\n';
        return false;
    }
}

bool OnnxRuntimeBackend::infer(const cv::Mat& input, std::vector<float>& output) {
    if (!impl_->ready()) {
        std::cerr << "[OnnxRuntimeBackend] infer called before load()\n";
        return false;
    }
    if (input.type() != CV_32FC3 || input.rows != impl_->height ||
        input.cols != impl_->width || input.channels() != impl_->channels) {
        // 预处理尺寸不匹配属于管线 bug，宁可失败也不静默 resize 出错误坐标。
        std::cerr << "[OnnxRuntimeBackend] input mismatch: expect " << impl_->width << "x"
                  << impl_->height << " CV_32FC3, got " << input.cols << "x" << input.rows
                  << " type=" << input.type() << '\n';
        return false;
    }

    try {
        packNchw(input, impl_->inputBlob.data());

        const std::array<int64_t, 4> inputShape = {1, impl_->channels, impl_->height,
                                                   impl_->width};
        Ort::MemoryInfo memoryInfo =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo, impl_->inputBlob.data(), impl_->inputBlob.size(),
            inputShape.data(), inputShape.size());

        // 名字指针指向 ORT 分配的字符串，必须先保存 allocated 对象再取 .get()，
        // 避免临时对象析构造成悬垂指针。
        Ort::AllocatorWithDefaultOptions allocator;
        auto inputName = impl_->session.GetInputNameAllocated(0, allocator);
        auto outputName = impl_->session.GetOutputNameAllocated(0, allocator);
        const char* inputNames[] = {inputName.get()};
        const char* outputNames[] = {outputName.get()};

        auto outputs =
            impl_->session.Run(Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1,
                               outputNames, 1);
        if (outputs.empty() || !outputs.front().IsTensor()) {
            std::cerr << "[OnnxRuntimeBackend] inference returned no tensor\n";
            return false;
        }
        const auto outInfo = outputs.front().GetTensorTypeAndShapeInfo();
        const size_t count = outInfo.GetElementCount();
        const float* data = outputs.front().GetTensorData<float>();
        output.assign(data, data + count);
        return true;
    } catch (const Ort::Exception& e) {
        std::cerr << "[OnnxRuntimeBackend] infer failed: " << e.what() << '\n';
        return false;
    }
}

}  // namespace vision
