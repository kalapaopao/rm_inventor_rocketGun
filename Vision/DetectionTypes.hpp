#pragma once

// Vision 管线共用的数据结构：帧、检测框、跟踪结果、统计信息。
// 只放纯数据，不包含任何推理库或相机 SDK 的依赖，
// 让相机采集代码与具体后端（ONNX Runtime / CVI NPU）保持解耦。

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace vision {

// 圆灯检测框，坐标为原图像素坐标。
struct Detection {
    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;
    float score = 0.0F; // contour area, not model confidence
    int classId = 0;
    // 传统视觉复筛是否通过；关闭复筛时恒为 true。
    bool refinePassed = true;

    bool lightRefined = false;
    float lightX = 0, lightY = 0, lightRadius = 0;
    float centerX() const { return lightRefined ? lightX : (x1 + x2) * 0.5F; }
    float centerY() const { return lightRefined ? lightY : (y1 + y2) * 0.5F; }
    float width() const { return x2 - x1; }
    float height() const { return y2 - y1; }
};

// 跟踪器对外的目标状态：当前估计 + 未来位置预测。
// 注意区分（方案 7.1）：current* 来自滤波器对"此刻"的估计，
// future* 只是基于当前状态按 prediction_horizon_seconds 的外推，
// 绝不回写进滤波器。
struct TrackedTarget {
    bool confirmed = false;      // 连续命中 min_confirm_frames 后才为 true
    bool predictionValid = false;
    float currentX = 0.0F;
    float currentY = 0.0F;
    float currentVx = 0.0F;      // 像素/秒
    float currentVy = 0.0F;
    float futureX = 0.0F;
    float futureY = 0.0F;
    double horizonSeconds = 0.0;
    double uncertainty = 0.0;    // 位置标准差(px)，来自卡尔曼协方差
    int missedFrames = 0;        // 连续未匹配检测数
    bool justLost = false;       // 本帧由跟踪转丢失（用于计数）
};

// 四类 FPS + 延迟分位数（方案 9.6 / 11.5），供显示叠层和串口/日志输出。
struct VisionStats {
    float captureFps = 0.0F;     // 采集线程产出帧率
    float detectionFps = 0.0F;   // 传统检测完成帧率
    float displayFps = 0.0F;     // 输出线程消费帧率
    double e2eLatencyP50Ms = 0.0; // 采集完成 -> 结果就绪
    double e2eLatencyP95Ms = 0.0;
    double detectMs = 0.0;        // 最近一帧圆灯检测耗时
    double trackingMs = 0.0;      // 最近一帧跟踪耗时
    std::uint64_t droppedFrames = 0;  // 推理队列覆盖丢弃的旧帧数
    std::uint64_t trackingLostCount = 0;
    std::uint64_t processedFrames = 0;
};

// 推理线程对外发布的"最新结果"：叠框图 + 检测 + 跟踪状态 + 统计。
// 输出线程（显示/录像/串口）只读取这一份快照，不阻塞采集和推理。
struct VisionResult {
    bool yawOffsetValid = false;
    double yawOffsetPixels = 0; // current filtered light x - (raw image centre + offset)
    cv::Mat annotated;           // BGR 叠框图（与被处理帧同一时刻）
    std::vector<Detection> detections;
    TrackedTarget target;
    VisionStats stats;
    bool currentAnglesValid = false, futureAnglesValid = false;
    // Optical frame, radians: right/down positive. Serial remains raw pixels.
    double currentYaw = 0, currentPitch = 0, futureYaw = 0, futurePitch = 0;
    // 该帧完成采集转换的时刻，作为端到端延迟的起点。
    std::chrono::steady_clock::time_point captureTimestamp{};
    // 每发布一次递增。消费者用序号区分"新结果"与"上次读过的同一份快照"，
    // 避免重复录像/统计。
    std::uint64_t sequence = 0;
};

}  // namespace vision
