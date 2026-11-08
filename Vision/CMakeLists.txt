cmake_minimum_required(VERSION 3.16)
project(rm_vision LANGUAGES CXX)

# 视觉推理模块（docs/海康相机目标识别接入方案.md §3）。
# ONNX Runtime 使用官方 Linux x64 预编译包，放在 third_party/（不入库），
# 也可以用 -DONNXRUNTIME_ROOT=/path/to/onnxruntime 覆盖。
set(RM_VISION_ENABLED "0" CACHE INTERNAL "Whether the vision pipeline is compiled in")
set(ONNXRUNTIME_LIB_DIR "" CACHE INTERNAL "Directory of libonnxruntime for rpath")

option(RM_ENABLE_VISION "Compile the YOLO vision pipeline and backend" ON)
if(NOT RM_ENABLE_VISION)
    message(STATUS "Vision pipeline disabled by RM_ENABLE_VISION=OFF")
    return()
endif()

if(NOT ONNXRUNTIME_ROOT)
    set(ONNXRUNTIME_ROOT
        "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/onnxruntime-linux-x64-1.31.1"
        CACHE PATH "ONNX Runtime C++ package root")
endif()

find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
    PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT ONNXRUNTIME_LIBRARY)
    message(WARNING
        "ONNX Runtime not found under ${ONNXRUNTIME_ROOT}/lib; "
        "building without vision support (vision.enabled=1 will be rejected at runtime).")
    return()
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Threads REQUIRED)

set(RM_VISION_ENABLED "1" CACHE INTERNAL "Whether the vision pipeline is compiled in" FORCE)
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT}/lib" CACHE INTERNAL
    "Directory of libonnxruntime for rpath" FORCE)

# 静态库：后端 + 管线 + 跟踪 + 串口。相机代码只链接 vision_core，
# 不直接接触 ONNX Runtime（方案 §6 的依赖隔离）。
add_library(vision_core STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/OnnxRuntimeBackend.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/VisionPipeline.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/TargetTracker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SerialOutput.cpp"
)

find_package(OpenCV REQUIRED COMPONENTS videoio video)

target_include_directories(vision_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${ONNXRUNTIME_ROOT}/include"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(vision_core PUBLIC
    "${ONNXRUNTIME_LIBRARY}"
    ${OpenCV_LIBS}
    Threads::Threads
)

target_compile_options(vision_core PRIVATE -Wall -Wextra -Wpedantic)

# 单图推理测试（方案 §9.4）：vision_single_test <model.onnx> <image> [confidence]
add_executable(vision_single_test "${CMAKE_CURRENT_SOURCE_DIR}/SingleImageTest.cpp")
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)
target_link_libraries(vision_single_test PRIVATE
    vision_core
    ${OpenCV_LIBS}
)
set_target_properties(vision_single_test PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib")
target_compile_options(vision_single_test PRIVATE -Wall -Wextra -Wpedantic)
