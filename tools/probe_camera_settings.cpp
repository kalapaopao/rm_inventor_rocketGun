// 诊断探针：验证 hik_calibration 应用的是模块内 config.yaml，而不是 Camera/camera_config.yaml。
// 流程：打开相机 → 打印“应用前”寄存器值（上个程序留下的旧值）→ 按 hik_calibration
// 的 applyCameraConfig 同样的顺序应用 → 打印“应用后”读回值。不做取流，不弹窗口。
//
// 构建：cmake --build build --target probe_camera_settings
// 运行：需先停掉占用相机的进程（如 dart-vision.service），然后 ./build/hik_calibration/probe_camera_settings
#include <MvCameraControl.h>

#include <opencv2/core.hpp>

#include <iostream>
#include <string>

#ifndef HIK_CALIBRATION_CONFIG_PATH
#define HIK_CALIBRATION_CONFIG_PATH "./config.yaml"
#endif

namespace {

struct Config {
    float exposure_time = -1.0f;
    float gain = -1.0f;
    int image_width = 0;
    int image_height = 0;
    float frame_rate = -1.0f;
};

void warnIfFailed(int ret, const std::string& where) {
    if (ret != MV_OK) {
        std::cerr << "  [FAIL] " << where << " ret=0x" << std::hex << ret << std::dec << "\n";
    }
}

void printFloat(void* handle, const char* name) {
    MVCC_FLOATVALUE v{};
    if (MV_CC_GetFloatValue(handle, name, &v) == MV_OK) {
        std::cout << "  " << name << " = " << v.fCurValue << "\n";
    } else {
        std::cout << "  " << name << " = <不可读>\n";
    }
}

void printInt(void* handle, const char* name) {
    MVCC_INTVALUE v{};
    MVCC_INTVALUE_EX vex{};
    if (MV_CC_GetIntValueEx(handle, name, &vex) == MV_OK) {
        std::cout << "  " << name << " = " << vex.nCurValue << "\n";
    } else if (MV_CC_GetIntValue(handle, name, &v) == MV_OK) {
        std::cout << "  " << name << " = " << v.nCurValue << "\n";
    } else {
        std::cout << "  " << name << " = <不可读>\n";
    }
}

void printEnum(void* handle, const char* name) {
    MVCC_ENUMVALUE v{};
    if (MV_CC_GetEnumValue(handle, name, &v) == MV_OK) {
        std::cout << "  " << name << " = " << v.nCurValue << " (0=Off 1=Once 2=Continuous)\n";
    } else {
        std::cout << "  " << name << " = <不可读>\n";
    }
}

void dumpAll(void* handle, const char* tag) {
    std::cout << "--- " << tag << " ---\n";
    printFloat(handle, "ExposureTime");
    printFloat(handle, "Gain");
    printEnum(handle, "ExposureAuto");
    printEnum(handle, "GainAuto");
    printInt(handle, "Width");
    printInt(handle, "Height");
    printFloat(handle, "ResultingFrameRate");
}

}  // namespace

int main() {
    // 1. 读配置——打印完整路径，证明读的是哪个文件
    std::cout << "[probe] 配置文件 = " << HIK_CALIBRATION_CONFIG_PATH << "\n";
    cv::FileStorage fs(HIK_CALIBRATION_CONFIG_PATH, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[probe] 无法打开配置文件\n";
        return 1;
    }
    Config cfg;
    fs["exposure_time"] >> cfg.exposure_time;
    fs["gain"] >> cfg.gain;
    fs["image_width"] >> cfg.image_width;
    fs["image_height"] >> cfg.image_height;
    fs["frame_rate"] >> cfg.frame_rate;
    std::cout << "[probe] 解析结果: exposure=" << cfg.exposure_time << "us gain=" << cfg.gain
              << " size=" << cfg.image_width << "x" << cfg.image_height
              << " frame_rate=" << cfg.frame_rate << "\n";

    // 2. 打开相机
    MV_CC_DEVICE_INFO_LIST list{};
    if (MV_CC_EnumDevices(MV_USB_DEVICE, &list) != MV_OK || list.nDeviceNum == 0) {
        std::cerr << "[probe] 未找到相机\n";
        return 1;
    }
    void* handle = nullptr;
    if (MV_CC_CreateHandle(&handle, list.pDeviceInfo[0]) != MV_OK) return 1;
    if (MV_CC_OpenDevice(handle) != MV_OK) {
        std::cerr << "[probe] 打开相机失败（是否被 show / MVS 客户端占用？）\n";
        MV_CC_DestroyHandle(handle);
        return 1;
    }

    dumpAll(handle, "应用前（相机寄存器里残留的上个程序的值）");

    // 3. 按 hik_calibration::applyCameraConfig 的同样顺序应用
    if (cfg.image_width > 0) warnIfFailed(MV_CC_SetIntValueEx(handle, "Width", cfg.image_width), "set Width");
    if (cfg.image_height > 0) warnIfFailed(MV_CC_SetIntValueEx(handle, "Height", cfg.image_height), "set Height");
    if (cfg.exposure_time > 0.0f) {
        warnIfFailed(MV_CC_SetEnumValue(handle, "ExposureAuto", 0), "disable ExposureAuto");
        warnIfFailed(MV_CC_SetFloatValue(handle, "ExposureTime", cfg.exposure_time), "set ExposureTime");
    }
    if (cfg.gain >= 0.0f) {
        warnIfFailed(MV_CC_SetEnumValue(handle, "GainAuto", 0), "disable GainAuto");
        warnIfFailed(MV_CC_SetFloatValue(handle, "Gain", cfg.gain), "set Gain");
    }
    if (cfg.frame_rate > 0.0f) {
        warnIfFailed(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true), "enable AcquisitionFrameRate");
        warnIfFailed(MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", cfg.frame_rate), "set AcquisitionFrameRate");
    }

    dumpAll(handle, "应用后（按标定配置覆盖的结果）");

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 0;
}
