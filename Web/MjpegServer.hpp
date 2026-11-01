#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

// Web streaming configuration. Camera acquisition does not depend on this type.
struct StreamConfig {
    bool enabled = true;
    int port = 8080;
    int width = 960;
    double fps = 15.0;
    int quality = 65;

    bool load(const cv::FileNode& node);
};

// Small Linux MJPEG-over-HTTP server.
// The producer only copies the newest frame; JPEG encoding and socket I/O run
// on their own threads so a slow browser cannot block camera acquisition.
class MjpegServer {
public:
    explicit MjpegServer(StreamConfig config = {});
    ~MjpegServer();

    MjpegServer(const MjpegServer&) = delete;
    MjpegServer& operator=(const MjpegServer&) = delete;

    bool start();
    void stop();
    bool running() const { return running_.load(std::memory_order_relaxed); }
    int port() const { return config_.port; }

    void submit(const cv::Mat& frame);

private:
    using Clock = std::chrono::steady_clock;

    void acceptLoop();
    void encodeLoop();
    void clientLoop(int fd);

    static int createListenSocket(int port);
    static bool sendAll(int fd, const void* data, std::size_t size);
    static std::string requestPath(int fd);
    static bool sendPage(int fd);
    static bool sendNotFound(int fd);

    bool encode(const cv::Mat& frame, std::vector<unsigned char>& jpeg) const;
    bool nextFrame(std::uint64_t& sequence, std::vector<unsigned char>& jpeg);
    void closeClient(int fd);

    StreamConfig config_;
    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<int> clients_{0};

    std::thread acceptThread_;
    std::thread encodeThread_;
    std::mutex clientMutex_;
    std::vector<int> clientFds_;
    std::vector<std::thread> clientThreads_;

    Clock::time_point nextSubmit_{};
    cv::Mat pendingFrame_;
    std::mutex pendingMutex_;
    std::condition_variable pendingCv_;
    std::uint64_t pendingSequence_ = 0;
    std::uint64_t consumedSequence_ = 0;

    std::mutex frameMutex_;
    std::condition_variable frameCv_;
    std::vector<unsigned char> latestJpeg_;
    std::uint64_t frameSequence_ = 0;
};
