cmake_minimum_required(VERSION 3.16)
project(hik_calibration LANGUAGES CXX)

if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "This application targets Linux/WSL. Build and launch it from Linux.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgcodecs imgproc)
find_package(Threads REQUIRED)

# Same MVS SDK discovery as the Camera module.
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

add_executable(hik_calibration
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hik_calibration.cpp"
)

target_include_directories(hik_calibration PRIVATE
    "${MVS_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(hik_calibration PRIVATE
    "${MVS_LIBRARY}"
    ${OpenCV_LIBS}
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

# Default to the config next to this module, so the tool also works when
# launched from the repository root via run.sh.
target_compile_definitions(hik_calibration PRIVATE
    HIK_CALIBRATION_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config.yaml"
)

target_compile_options(hik_calibration PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(hik_calibration PROPERTIES BUILD_RPATH "${MVS_LIB_DIR}")
