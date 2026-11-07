cmake_minimum_required(VERSION 3.16)
project(rm_inventor_fly LANGUAGES CXX)

# Linux/WSL is the only supported C++ build and launch environment.
add_subdirectory(Camera)
add_subdirectory(hik_calibration)
