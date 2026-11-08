#include "SerialOutput.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace vision {

namespace {

std::uint16_t crc16Modbus(const unsigned char* data, size_t length) {
    std::uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 1) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001)
                            : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

speed_t toSpeedT(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B115200;
    }
}

}  // namespace

bool SerialOutput::start(const SerialConfig& config, ResultProvider provider) {
    if (config.port.empty()) return false;
    config_ = config;
    provider_ = std::move(provider);
    if (!openPort()) return false;
    stopRequested_.store(false);
    running_.store(true);
    thread_ = std::thread(&SerialOutput::loop, this);
    std::cout << "[SerialOutput] started on " << config_.port << " @" << config_.baud
              << ", period " << config_.periodMs << " ms\n";
    return true;
}

void SerialOutput::stop() {
    bool expected = false;
    if (stopRequested_.compare_exchange_strong(expected, true)) {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
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

void SerialOutput::buildFrame(const VisionResult& result,
                              unsigned char* buf) const {
    const TrackedTarget& t = result.target;
    buf[0] = 0xA5;
    buf[1] = !t.confirmed ? 0 : (t.predictionValid ? 1 : 2);
    float futureX = t.predictionValid ? t.futureX : 0.0F;
    float futureY = t.predictionValid ? t.futureY : 0.0F;
    std::memcpy(buf + 2, &futureX, 4);
    std::memcpy(buf + 6, &futureY, 4);
    std::memcpy(buf + 10, &t.currentX, 4);
    std::memcpy(buf + 14, &t.currentY, 4);
    const std::uint16_t crc = crc16Modbus(buf, 18);
    buf[18] = static_cast<unsigned char>(crc & 0xFF);
    buf[19] = static_cast<unsigned char>((crc >> 8) & 0xFF);
}

void SerialOutput::loop() {
    const auto period = std::chrono::milliseconds(config_.periodMs);
    VisionResult result;
    unsigned char frame[20];
    while (!stopRequested_.load()) {
        const auto next = std::chrono::steady_clock::now() + period;
        if (provider_ && provider_(result)) {
            buildFrame(result, frame);
            const ssize_t n = ::write(fd_, frame, sizeof(frame));
            if (n == sizeof(frame)) {
                sentFrames_.fetch_add(1, std::memory_order_relaxed);
            } else {
                sendErrors_.fetch_add(1, std::memory_order_relaxed);
            }
        }
        std::this_thread::sleep_until(next);
    }
}

}  // namespace vision
