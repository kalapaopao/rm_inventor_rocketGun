// 录像校验小工具：打印帧数 / 帧率 / 分辨率 / 实际时长，可选导出首帧、末帧。
//
//   ./build/checkvid recordings/xxx.avi                 # 只看信息
//   ./build/checkvid recordings/xxx.avi first.jpg last.jpg   # 顺便导出首末帧
//
// 依赖 OpenCV（与主程序同一套），构建时由 Camera/CMakeLists.txt 的 checkvid 目标产出。

#include <opencv2/opencv.hpp>

#include <cstdio>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: %s <video> [first.jpg] [last.jpg]\n", argv[0]);
        return 2;
    }

    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        std::printf("open failed: %s\n", argv[1]);
        return 1;
    }

    const int frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double duration = fps > 0.0 ? frames / fps : 0.0;

    std::printf("frames=%d  fps=%.2f  size=%dx%d  duration=%.2fs\n",
                frames, fps, width, height, duration);

    cv::Mat frame;
    if (argc > 2) {
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        if (cap.read(frame) && !frame.empty()) {
            cv::imwrite(argv[2], frame);
            std::printf("first frame -> %s\n", argv[2]);
        }
    }
    if (argc > 3) {
        cap.set(cv::CAP_PROP_POS_FRAMES, std::max(0, frames - 1));
        if (cap.read(frame) && !frame.empty()) {
            cv::imwrite(argv[3], frame);
            std::printf("last  frame -> %s\n", argv[3]);
        }
    }

    return 0;
}
