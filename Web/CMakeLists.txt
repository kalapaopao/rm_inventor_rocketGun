add_library(web_stream STATIC MjpegServer.cpp)

target_include_directories(web_stream
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(web_stream
    PUBLIC
        ${OpenCV_LIBS}
        Threads::Threads
)

target_compile_options(web_stream PRIVATE -Wall -Wextra -Wpedantic)
