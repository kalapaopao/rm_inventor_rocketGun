#include "CameraSetting.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

#ifndef DEFAULT_CAMERA_CONFIG_PATH
#define DEFAULT_CAMERA_CONFIG_PATH "camera_config.yaml"
#endif

namespace {

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " [options] [config.yaml]\n\n"
              << "Options:\n"
              << "  --window       enable the OpenCV preview window\n"
              << "  --recording    record for record_seconds from YAML\n"
              << "  --config PATH  use a specific YAML file\n"
              << "  -h, --help     show this help\n\n"
              << "Without --window, the program runs without a preview window.\n";
}

bool parseArgs(int argc, char* argv[], std::string& configPath,
               int& windowMode, bool& recording, bool& showHelp) {
    configPath = DEFAULT_CAMERA_CONFIG_PATH;
    windowMode = 0;  // no --window means headless mode
    recording = false;
    showHelp = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            showHelp = true;
            return true;
        }
        if (arg == "--window") {
            windowMode = 1;
            continue;
        }
        if (arg == "--recording") {
            recording = true;
            continue;
        }
        if (arg == "--config") {
            if (i + 1 >= argc) {
                std::cerr << "--config requires a path.\n";
                return false;
            }
            configPath = argv[++i];
            continue;
        }
        if (arg.rfind("--config=", 0) == 0) {
            configPath = arg.substr(9);
            continue;
        }
        if (!arg.empty() && arg.front() != '-') {
            configPath = arg;
            continue;
        }
        std::cerr << "Unknown argument: " << arg << '\n';
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string configPath;
    int windowMode = -1;
    bool recording = false;
    bool showHelp = false;
    if (!parseArgs(argc, argv, configPath, windowMode, recording, showHelp)) {
        printUsage(argv[0]);
        return 2;
    }
    if (showHelp) {
        printUsage(argv[0]);
        return 0;
    }

    CameraConfig config;
    if (!config.load(configPath)) return 1;
    config.displayEnabled = windowMode == 1;
    if (!recording) {
        config.recordSeconds = 0.0;
    } else if (config.recordSeconds <= 0.0) {
        config.recordSeconds = 10.0;
    }

    if (config.displayEnabled && std::getenv("DISPLAY") == nullptr &&
        std::getenv("WAYLAND_DISPLAY") == nullptr) {
        std::cerr << "No graphical display detected; disabling imshow.\n";
        config.displayEnabled = false;
    }

    CameraSetting camera(config);
    return camera.run() ? 0 : 1;
}
