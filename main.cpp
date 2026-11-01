#include "CameraSetting.hpp"
#include "Web/MjpegServer.hpp"

#include <opencv2/core.hpp>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#ifndef DEFAULT_CAMERA_CONFIG_PATH
#define DEFAULT_CAMERA_CONFIG_PATH "camera_config.yaml"
#endif

namespace {

struct Options {
    std::string configPath = DEFAULT_CAMERA_CONFIG_PATH;
    bool recording = false;
    double recordSeconds = -1.0;
    bool noWindow = false;
    bool noStream = false;
};

bool parseNumber(const std::string& text, double& value) {
    if (text.empty()) return false;
    char* end = nullptr;
    value = std::strtod(text.c_str(), &end);
    return end != nullptr && *end == '\0';
}

void printUsage(const char* program) {
    std::cout << "用法: " << program << " [选项]\n\n"
              << "  --recording[=秒]   启动即录像；不带数值时使用配置中的 record_seconds\n"
              << "  --config <路径>    指定 YAML 配置（默认 " << DEFAULT_CAMERA_CONFIG_PATH << ")\n"
              << "  --no-window        不打开 OpenCV 预览窗口\n"
              << "  --no-stream        不启动网页实时流\n"
              << "  -h, --help         显示帮助\n\n"
              << "常驻模式：预览窗口中按 r 开关录像，按 q 或 Ctrl+C 退出。\n";
}

bool parseArgs(int argc, char* argv[], Options& options, bool& showHelp) {
    showHelp = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            showHelp = true;
            return true;
        }
        if (arg == "--recording") {
            options.recording = true;
            if (i + 1 < argc && parseNumber(argv[i + 1], options.recordSeconds)) ++i;
            continue;
        }
        if (arg.rfind("--recording=", 0) == 0) {
            options.recording = true;
            if (!parseNumber(arg.substr(std::strlen("--recording=")), options.recordSeconds)) {
                std::cerr << "无效的录像时长: " << arg << '\n';
                return false;
            }
            continue;
        }
        if (arg == "--config") {
            if (++i >= argc) {
                std::cerr << "--config 需要一个路径参数\n";
                return false;
            }
            options.configPath = argv[i];
            continue;
        }
        if (arg.rfind("--config=", 0) == 0) {
            options.configPath = arg.substr(std::strlen("--config="));
            continue;
        }
        if (arg == "--no-window") {
            options.noWindow = true;
            continue;
        }
        if (arg == "--no-stream") {
            options.noStream = true;
            continue;
        }
        if (!arg.empty() && arg.front() != '-') {
            options.configPath = arg;  // 兼容旧用法：./show config.yaml
            continue;
        }
        std::cerr << "未知参数: " << arg << '\n';
        printUsage(argv[0]);
        return false;
    }
    return true;
}

bool loadStreamConfig(const std::string& path, StreamConfig& config) {
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) return false;
    return config.load(storage["stream"]);
}

}  // namespace

int main(int argc, char* argv[]) {
    Options options;
    bool showHelp = false;
    if (!parseArgs(argc, argv, options, showHelp)) return 1;
    if (showHelp) return 0;

    CameraConfig cameraConfig;
    if (!cameraConfig.load(options.configPath)) {
        std::cerr << "Failed to load camera config: " << options.configPath << '\n';
        return 1;
    }

    StreamConfig streamConfig;
    if (!loadStreamConfig(options.configPath, streamConfig)) {
        std::cerr << "Failed to load stream config: " << options.configPath << '\n';
        return 1;
    }

    if (options.noWindow) cameraConfig.displayEnabled = false;
    if (options.noStream) streamConfig.enabled = false;
    cameraConfig.recordSeconds = options.recording
        ? (options.recordSeconds > 0.0 ? options.recordSeconds
                                       : (cameraConfig.recordSeconds > 0.0
                                              ? cameraConfig.recordSeconds
                                              : 10.0))
        : 0.0;

    if (cameraConfig.displayEnabled && std::getenv("DISPLAY") == nullptr &&
        std::getenv("WAYLAND_DISPLAY") == nullptr) {
        std::cout << "No display detected; running without GUI window.\n";
        cameraConfig.displayEnabled = false;
    }
    if (!cameraConfig.displayEnabled && cameraConfig.recordSeconds <= 0.0 &&
        !streamConfig.enabled) {
        std::cerr << "No output enabled: window, recording and stream are all disabled.\n";
        return 1;
    }

    CameraSetting camera(cameraConfig);
    MjpegServer stream(streamConfig);
    if (streamConfig.enabled) {
        if (!stream.start()) return 1;
        camera.setFrameSink([&stream](const cv::Mat& frame) { stream.submit(frame); });
    }

    if (cameraConfig.recordSeconds > 0.0) {
        std::cout << "Recording for " << cameraConfig.recordSeconds << " seconds.\n";
    } else {
        std::cout << "Live mode: press r to record, q or Ctrl+C to exit.\n";
    }

    const bool ok = camera.import_image();
    stream.stop();
    return ok ? 0 : 1;
}
