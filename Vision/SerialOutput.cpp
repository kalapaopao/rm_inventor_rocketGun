#include "SerialOutput.hpp"

#include <chrono>
#include <cstring>
#include <cerrno>
#include <poll.h>
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace vision {

namespace {

speed_t toSpeedT(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return 0;
    }
}

}  // namespace

bool SerialOutput::start(const SerialConfig& config, ResultProvider provider) {
    if (running_.load() || thread_.joinable() || config.port.empty() ||
        config.periodMs <= 0 || config.periodMs > 1000 || config.maxAgeMs <= 0 ||
        toSpeedT(config.baud) == 0 || !provider) return false;
    config_ = config;
    provider_ = std::move(provider);
    if (!openPort()) return false;
    sentFrames_.store(0); sendErrors_.store(0);
    { std::lock_guard<std::mutex> lock(statusMutex_);
      hasLastSent_ = false; lastYaw_ = 0; lastTargetValid_ = false; }
    stopRequested_.store(false);
    running_.store(true);
    thread_ = std::thread(&SerialOutput::loop, this);
    std::cout << "[SerialOutput] started on " << config_.port << " @" << config_.baud
              << ", period " << config_.periodMs << " ms\n";
    return true;
}

void SerialOutput::stop() {
    stopRequested_.store(true);
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
    running_.store(false);
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialOutput::openPort() {
    fd_ = ::open(config_.port.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[SerialOutput] cannot open " << config_.port << '\n';
        return false;
    }
    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
        std::cerr << "[SerialOutput] tcgetattr failed\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    speed_t speed = toSpeedT(config_.baud);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    tty.c_cflag = CLOCAL | CREAD | CS8;  // 只发送：8N1，无流控
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[SerialOutput] tcsetattr failed\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

SerialStatus SerialOutput::status() const {
    SerialStatus out;
    out.running = running_.load(); out.sent = sentFrames_.load();
    out.errors = sendErrors_.load();
    std::lock_guard<std::mutex> lock(statusMutex_);
    out.hasLastSent = hasLastSent_; out.lastYaw = lastYaw_;
    out.lastTargetValid = lastTargetValid_;
    return out;
}

bool SerialOutput::writePacket(const Frame& packet) {
    size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    while (!stopRequested_.load() && std::chrono::steady_clock::now() < deadline) {
        const ssize_t count = ::write(fd_, packet.data()+offset, packet.size()-offset);
        if (count > 0) {
            offset += static_cast<size_t>(count);
            if (offset == packet.size()) return true;
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            break;
        } else {
            pollfd descriptor{fd_, POLLOUT, 0};
            const int ready = ::poll(&descriptor, 1, 5);
            if (ready > 0 && (descriptor.revents & (POLLERR|POLLHUP|POLLNVAL))) break;
        }
    }
    // Never append a new frame after an incomplete write: that corrupts framing.
    sendErrors_.fetch_add(1);
    stopRequested_.store(true);
    std::cerr << "[SerialOutput] incomplete/error write; sender stopped (MCU timeout required)\n";
    return false;
}

void SerialOutput::loop() {
    const auto period = std::chrono::milliseconds(config_.periodMs);
    while (!stopRequested_.load()) {
        const auto next = std::chrono::steady_clock::now() + period;
        VisionResult result;
        bool haveResult = false;
        try {
            haveResult = provider_(result);
        } catch (const std::exception& e) {
            ++sendErrors_;
            std::cerr << "[SerialOutput] provider failed: " << e.what() << '\n';
        }
        bool targetValid = false;
        std::int16_t yaw = 0;
        if (haveResult) {
            targetValid = selectYaw(result, std::chrono::steady_clock::now(),
                                    config_.maxAgeMs, yaw);
        }
        if (!targetValid) yaw = 0;  // 无新鲜目标：发 TARGET=0，绝不伪装有效零
        // 每周期都发一帧：TARGET=0 帧让电控知道链路仍在，只是当前没目标。
        if (writePacket(encodeFrame(targetValid, yaw))) {
            {
                std::lock_guard<std::mutex> lock(statusMutex_);
                lastTargetValid_ = targetValid;
                if (targetValid) { lastYaw_ = yaw; hasLastSent_ = true; }
            }
            sentFrames_.fetch_add(1);
        }
        std::unique_lock<std::mutex> lock(waitMutex_);
        wake_.wait_until(lock, next, [this] { return stopRequested_.load(); });
    }
    running_.store(false);
}

}  // namespace vision
