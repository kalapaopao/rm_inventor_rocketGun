cmake_minimum_required(VERSION 3.16)
project(HikrobotCameraPreview LANGUAGES CXX)

if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "This application targets Linux/WSL. Build and launch it from Linux.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)
find_package(Threads REQUIRED)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK installation")
set(MVS_INCLUDE_DIR "${MVS_ROOT}/include")
set(MVS_LIB_DIR "" CACHE PATH "Hikrobot MVS library directory")

if(NOT MVS_LIB_DIR)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(MVS_LIB_DIR "${MVS_ROOT}/lib/aarch64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")
    else()
        message(FATAL_ERROR
            "Unsupported processor for automatic MVS path detection: "
            "${CMAKE_SYSTEM_PROCESSOR}. Set -DMVS_LIB_DIR=/path/to/lib.")
    endif()
endif()

find_library(MVS_LIBRARY NAMES MvCameraControl PATHS "${MVS_LIB_DIR}" NO_DEFAULT_PATH REQUIRED)

add_executable(show
    "${CMAKE_CURRENT_SOURCE_DIR}/../main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/CameraSetting.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/VideoRecorder.cpp"
)

target_include_directories(show PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/.."
    "${MVS_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(show PRIVATE
    "${MVS_LIBRARY}"
    ${OpenCV_LIBS}
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

target_compile_definitions(show PRIVATE
    DEFAULT_CAMERA_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/camera_config.yaml"
    DEFAULT_RECORDING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../recordings"
)

target_compile_options(show PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(show PROPERTIES BUILD_RPATH "${MVS_LIB_DIR}")
