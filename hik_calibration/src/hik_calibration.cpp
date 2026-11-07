#include <MvCameraControl.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ===== 需要你确认或修改的路径都放这里 =====
// 默认取模块目录下的 config.yaml（由 CMake 注入绝对路径）；未注入时回退到工作目录。
#ifndef HIK_CALIBRATION_CONFIG_PATH
#define HIK_CALIBRATION_CONFIG_PATH "./config.yaml"
#endif
static const char* kConfigPath = HIK_CALIBRATION_CONFIG_PATH;
// =========================================

namespace fs = std::filesystem;

struct Config {
    int capture_images = 1;
    std::string save_dir = "./calibration_images";
    int board_width = 8;
    int board_height = 11;
    double square_size = 0.45;
    int camera_index = 0;
    int image_width = 0;
    int image_height = 0;
    int offset_x = 0;
    int offset_y = 0;
    float exposure_time = -1.0f;
    float gain = -1.0f;
    float frame_rate = -1.0f;
    std::string window_name = "Hikvision Calibration";
    int min_images = 5;
    std::string output_file = "./camera_intrinsics.yaml";
};

struct SavedImage {
    fs::path path;
    std::vector<cv::Point2f> corners;
    cv::Size image_size;
};

static void checkMv(int ret, const std::string& where) {
    if (ret != MV_OK) {
        std::ostringstream oss;
        oss << where << " failed, ret=0x" << std::hex << ret;
        throw std::runtime_error(oss.str());
    }
}

static Config loadConfig(const std::string& path) {
    Config cfg;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("无法打开配置文件: " + path);
    }
    fs["capture_images"] >> cfg.capture_images;
    fs["save_dir"] >> cfg.save_dir;
    fs["board_width"] >> cfg.board_width;
    fs["board_height"] >> cfg.board_height;
    fs["square_size"] >> cfg.square_size;
    fs["camera_index"] >> cfg.camera_index;
    fs["image_width"] >> cfg.image_width;
    fs["image_height"] >> cfg.image_height;
    fs["offset_x"] >> cfg.offset_x;
    fs["offset_y"] >> cfg.offset_y;
    fs["exposure_time"] >> cfg.exposure_time;
    fs["gain"] >> cfg.gain;
    fs["frame_rate"] >> cfg.frame_rate;
    fs["window_name"] >> cfg.window_name;
    fs["min_images"] >> cfg.min_images;
    fs["output_file"] >> cfg.output_file;

    if (cfg.board_width <= 1 || cfg.board_height <= 1) {
        throw std::runtime_error("board_width和board_height的内角计数必须大于1。");
    }
    if (cfg.square_size <= 0.0f) {
        throw std::runtime_error("square_size必须大于0。");
    }
    if (cfg.min_images < 3) {
        throw std::runtime_error("min_images必须至少为3。");
    }
    return cfg;
}

class HikCamera {
public:
    explicit HikCamera(const Config& cfg) : cfg_(cfg) {
        MV_CC_DEVICE_INFO_LIST device_list{};
        checkMv(MV_CC_EnumDevices(MV_USB_DEVICE, &device_list), "MV_CC_EnumDevices");
        if (device_list.nDeviceNum == 0) {
            throw std::runtime_error("未找到海康威视USB工业相机。");
        }
        if (cfg_.camera_index < 0 || cfg_.camera_index >= static_cast<int>(device_list.nDeviceNum)) {
            throw std::runtime_error("camera_index超出范围。");
        }

        checkMv(MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[cfg_.camera_index]), "MV_CC_CreateHandle");
        checkMv(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");

        MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
        applyCameraConfig();
        checkMv(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
    }

    ~HikCamera() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
    }

    HikCamera(const HikCamera&) = delete;
    HikCamera& operator=(const HikCamera&) = delete;

    cv::Mat read() {
        MV_FRAME_OUT frame{};
        int ret = MV_CC_GetImageBuffer(handle_, &frame, 100);
        if (ret != MV_OK) {
            return {};
        }

        cv::Mat image;
        try {
            image = convertFrame(frame);
        } catch (...) {
            MV_CC_FreeImageBuffer(handle_, &frame);
            throw;
        }

        checkMv(MV_CC_FreeImageBuffer(handle_, &frame), "MV_CC_FreeImageBuffer");
        return image;
    }

private:
    void warnMv(int ret, const std::string& where) {
        if (ret != MV_OK) {
            std::cerr << "Warning: " << where << " failed, ret=0x" << std::hex << ret << std::dec << "\n";
        }
    }

    void applyCameraConfig() {
        if (cfg_.offset_x > 0) {
            warnMv(MV_CC_SetIntValueEx(handle_, "OffsetX", cfg_.offset_x), "set OffsetX");
        }
        if (cfg_.offset_y > 0) {
            warnMv(MV_CC_SetIntValueEx(handle_, "OffsetY", cfg_.offset_y), "set OffsetY");
        }
        if (cfg_.image_width > 0) {
            warnMv(MV_CC_SetIntValueEx(handle_, "Width", cfg_.image_width), "set Width");
        }
        if (cfg_.image_height > 0) {
            warnMv(MV_CC_SetIntValueEx(handle_, "Height", cfg_.image_height), "set Height");
        }
        if (cfg_.exposure_time > 0.0f) {
            warnMv(MV_CC_SetEnumValue(handle_, "ExposureAuto", 0), "disable ExposureAuto");
            warnMv(MV_CC_SetFloatValue(handle_, "ExposureTime", cfg_.exposure_time), "set ExposureTime");
        }
        if (cfg_.gain >= 0.0f) {
            warnMv(MV_CC_SetEnumValue(handle_, "GainAuto", 0), "disable GainAuto");
            warnMv(MV_CC_SetFloatValue(handle_, "Gain", cfg_.gain), "set Gain");
        }
        if (cfg_.frame_rate > 0.0f) {
            warnMv(MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true), "enable AcquisitionFrameRate");
            warnMv(MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", cfg_.frame_rate), "set AcquisitionFrameRate");
        }
    }

    cv::Mat convertFrame(const MV_FRAME_OUT& frame) {
        const int width = static_cast<int>(frame.stFrameInfo.nWidth);
        const int height = static_cast<int>(frame.stFrameInfo.nHeight);
        const auto pixel_type = frame.stFrameInfo.enPixelType;

        if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
            cv::Mat src(height, width, CV_8UC3, frame.pBufAddr);
            return src.clone();
        }
        if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
            cv::Mat src(height, width, CV_8UC3, frame.pBufAddr);
            cv::Mat bgr;
            cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);
            return bgr;
        }
        if (pixel_type == PixelType_Gvsp_Mono8) {
            cv::Mat gray(height, width, CV_8UC1, frame.pBufAddr);
            cv::Mat bgr;
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            return bgr;
        }

        MV_CC_PIXEL_CONVERT_PARAM convert{};
        convert.nWidth = frame.stFrameInfo.nWidth;
        convert.nHeight = frame.stFrameInfo.nHeight;
        convert.pSrcData = frame.pBufAddr;
        convert.nSrcDataLen = frame.stFrameInfo.nFrameLen;
        convert.enSrcPixelType = pixel_type;
        convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

        std::vector<unsigned char> buffer(width * height * 3);
        convert.pDstBuffer = buffer.data();
        convert.nDstBufferSize = static_cast<unsigned int>(buffer.size());
        checkMv(MV_CC_ConvertPixelType(handle_, &convert), "MV_CC_ConvertPixelType");

        cv::Mat bgr(height, width, CV_8UC3, buffer.data());
        return bgr.clone();
    }

    Config cfg_;
    void* handle_ = nullptr;
};

static std::vector<cv::Point3f> makeObjectPoints(const Config& cfg) {
    std::vector<cv::Point3f> points;
    points.reserve(cfg.board_width * cfg.board_height);
    for (int y = 0; y < cfg.board_height; ++y) {
        for (int x = 0; x < cfg.board_width; ++x) {
            points.emplace_back(x * cfg.square_size, y * cfg.square_size, 0.0f);
        }
    }
    return points;
}

static bool findChessboardCorners(const cv::Mat& image, const Config& cfg, std::vector<cv::Point2f>& corners) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    const cv::Size pattern_size(cfg.board_width, cfg.board_height);
    bool found = cv::findChessboardCorners(
        gray,
        pattern_size,
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
    }
    return found;
}

static fs::path makeImagePath(const Config& cfg, int index) {
    std::ostringstream name;
    name << "calib_" << std::setw(4) << std::setfill('0') << index << ".png";
    return fs::path(cfg.save_dir) / name.str();
}

static double computeViewError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const cv::Mat& rvec,
    const cv::Mat& tvec) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected);
    return cv::norm(image_points, projected, cv::NORM_L2) / std::sqrt(static_cast<double>(projected.size()));
}

static void calibrateAndSave(const Config& cfg, const std::vector<SavedImage>& saved) {
    if (saved.size() < static_cast<size_t>(cfg.min_images)) {
        std::cout << "仅 " << saved.size() << " 有效图像。至少需要 " << cfg.min_images << ". 跳过校准.\n";
        return;
    }

    std::vector<std::vector<cv::Point3f>> object_points(saved.size(), makeObjectPoints(cfg));
    std::vector<std::vector<cv::Point2f>> image_points;
    image_points.reserve(saved.size());
    for (const auto& item : saved) {
        image_points.push_back(item.corners);
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    double rms = cv::calibrateCamera(
        object_points,
        image_points,
        saved.front().image_size,
        camera_matrix,
        dist_coeffs,
        rvecs,
        tvecs);

    std::vector<double> per_view_errors;
    per_view_errors.reserve(saved.size());
    double sum_error = 0.0;
    for (size_t i = 0; i < saved.size(); ++i) {
        double error = computeViewError(object_points[i], image_points[i], camera_matrix, dist_coeffs, rvecs[i], tvecs[i]);
        per_view_errors.push_back(error);
        sum_error += error;
    }
    const double mean_error = sum_error / static_cast<double>(per_view_errors.size());

    std::cout << "\nCalibration finished\n";
    std::cout << "Images: " << saved.size() << "\n";
    std::cout << "RMS reprojection error: " << rms << " px\n";
    std::cout << "Mean per-image reprojection error: " << mean_error << " px\n";
    std::cout << "Camera matrix:\n" << camera_matrix << "\n";
    std::cout << "Distortion coefficients:\n" << dist_coeffs.t() << "\n";
    for (size_t i = 0; i < saved.size(); ++i) {
        std::cout << saved[i].path.filename().string() << ": " << per_view_errors[i] << " px\n";
    }

    cv::FileStorage fs(cfg.output_file, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        throw std::runtime_error("无法写入输出文件: " + cfg.output_file);
    }
    fs << "image_width" << saved.front().image_size.width;
    fs << "image_height" << saved.front().image_size.height;
    fs << "board_width" << cfg.board_width;
    fs << "board_height" << cfg.board_height;
    fs << "square_size" << cfg.square_size;
    fs << "rms_reprojection_error" << rms;
    fs << "mean_per_image_reprojection_error" << mean_error;
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << dist_coeffs;
    fs << "images" << "[";
    for (size_t i = 0; i < saved.size(); ++i) {
        fs << "{"
           << "file" << saved[i].path.string()
           << "reprojection_error" << per_view_errors[i]
           << "rvec" << rvecs[i]
           << "tvec" << tvecs[i]
           << "}";
    }
    fs << "]";
    std::cout << "Saved calibration result to: " << cfg.output_file << "\n";
}

// 离线函数，从保存的图片中加载并寻找角点。返回成功找到角点的图片列表。
static std::vector<SavedImage> loadImagesAndFindCorners(const Config& cfg) {
    std::vector<SavedImage> saved;

    for (const auto& entry : fs::directory_iterator(cfg.save_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        fs::path path = entry.path();
        std::string ext = path.extension().string();
        if (ext != ".png" && ext != ".jpg" && ext != ".jpeg" && ext != ".bmp") {
            continue;
        }

        cv::Mat image = cv::imread(path.string());
        if (image.empty()) {
            std::cout << "Skip unreadable image: " << path.string() << "\n";
            continue;
        }

        std::vector<cv::Point2f> corners;
        if (!findChessboardCorners(image, cfg, corners)) {
            std::cout << "未找到角点: " << path.string() << "\n";
            continue;
        }

        saved.push_back({path, corners, image.size()});
        std::cout << "已加载: " << path.string() << " 角点=" << corners.size() << "\n";
    }

    return saved;
}


int main() {
    try {
        Config cfg = loadConfig(kConfigPath);
        fs::create_directories(cfg.save_dir);
        std::vector<SavedImage> saved;

        if(cfg.capture_images) {
            std::cout << "打开摄像头进行采集模式。\n";
        
            std::cout << "保存文件: " << fs::absolute(cfg.save_dir).string() << "\n";
            std::cout << "Keys: s=保存, d=删除上次保存的图像, Esc=完成并校准\n";

            HikCamera camera(cfg);
            int next_index = 1;

        // 缩放窗口以适应屏幕，避免过大无法显示
        // cv::namedWindow(cfg.window_name, cv::WINDOW_NORMAL);
            while (true) {
                cv::Mat frame = camera.read();
                if (frame.empty()) {
                    std::cerr << "无法抓取帧。\n";
                    continue;
                }

                std::vector<cv::Point2f> corners;
                bool found = findChessboardCorners(frame, cfg, corners);
                cv::Mat display = frame.clone();
                cv::drawChessboardCorners(display, cv::Size(cfg.board_width, cfg.board_height), corners, found);

                std::ostringstream status;
                status << "saved=" << saved.size() << " corners=" << (found ? "found" : "not found");
                cv::putText(display, status.str(), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                cv::imshow(cfg.window_name, display);

                int key = cv::waitKey(1) & 0xff;
                if (key == 27) {
                    break;
                }
                if (key == 's' || key == 'S') {
                    if (!found) {
                        std::cout << "未找到角点。图片未保存。\n";
                        continue;
                    }
                    fs::path image_path = makeImagePath(cfg, next_index++);
                    if (!cv::imwrite(image_path.string(), frame)) {
                        std::cerr << "无法保存图片: " << image_path.string() << "\n";
                        continue;
                    }
                    saved.push_back({image_path, corners, frame.size()});
                    std::cout << "Saved: " << image_path.string() << " corners=" << corners.size() << "\n";
                }
                if (key == 'd' || key == 'D') {
                    if (saved.empty()) {
                        std::cout << "没有已保存的图片可以删除。\n";
                        continue;
                    }
                    fs::path image_path = saved.back().path;
                    saved.pop_back();
                    std::error_code ec;
                    fs::remove(image_path, ec);
                    if (ec) {
                        std::cerr << "删除失败 " << image_path.string() << ": " << ec.message() << "\n";
                    } else {
                        std::cout << "删除: " << image_path.string() << "\n";
                    }
                }
            }
            cv::destroyAllWindows();
        }else {
            std::cout << "采集模式已关闭，进入离线模式，从保存目录加载图片。\n";
            saved = loadImagesAndFindCorners(cfg);
        }
        calibrateAndSave(cfg, saved);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << "\n";
        return 1;
    }
}
