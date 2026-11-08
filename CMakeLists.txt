cmake_minimum_required(VERSION 3.16)
project(rm_inventor_fly LANGUAGES CXX)

# Linux/WSL is the only supported C++ build and launch environment.
# Vision 必须先于 Camera 配置：Camera 依据 RM_VISION_ENABLED 决定是否链接 vision_core。
add_subdirectory(Vision)
add_subdirectory(Camera)
add_subdirectory(hik_calibration)
