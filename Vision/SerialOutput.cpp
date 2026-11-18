#include "SerialOutput.hpp"

#include <chrono>
#include <iostream>

namespace vision {

bool SerialOutput::start(const SerialConfig& config, ResultProvider provider) {
    if (running_.load() || thread_.joinable() || config.port.empty() ||
        config.periodMs <= 0 || config.periodMs > 1000 || config.maxAgeMs <= 0 ||
        !provider) return false;
    config_ = config;
    provider_ = std::move(provider);
    if (!openTransport()) return false;
    sentFrames_.store(0); sendErrors_.store(0);
    { std::lock_guard<std::mutex> lock(statusMutex_);
      hasLastSent_ = false; lastYaw_ = 0; lastTargetValid_ = false; }
    stopRequested_.store(false);
    running_.store(true);
    thread_ = std::thread(&SerialOutput::loop, this);
    std::cout << "[SerialOutput] started on " << transportName_ << " @" << config_.baud
              << ", period " << config_.periodMs << " ms\n";
    return true;
}

void SerialOutput::stop() {
    stopRequested_.store(true);
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
    running_.store(false);
    if (transport_) transport_->close();
}

bool SerialOutput::openTransport() {
    std::string error;
    transport_ = createSerialTransport(config_.backend, config_.port, config_.baud, error);
    if (!transport_) {
        std::cerr << "[SerialOutput] no transport for " << config_.port << ": " << error << '\n';
        return false;
    }
    if (!transport_->open(error)) {
        std::cerr << "[SerialOutput] cannot open " << config_.port << ": " << error << '\n';
        transport_.reset();
        return false;
    }
    transportName_ = transport_->describe();
    return true;
}

SerialStatus SerialOutput::status() const {
    SerialStatus out;
    out.running = running_.load(); out.sent = sentFrames_.load();
    out.errors = sendErrors_.load();
    out.transport = transportName_;
    std::lock_guard<std::mutex> lock(statusMutex_);
    out.hasLastSent = hasLastSent_; out.lastYaw = lastYaw_;
    out.lastTargetValid = lastTargetValid_;
    return out;
}

bool SerialOutput::writePacket(const Frame& packet) {
    if (transport_ && transport_->write(packet, writeError_)) return true;
    // Never append a new frame after an incomplete write: that corrupts framing.
    sendErrors_.fetch_add(1);
    stopRequested_.store(true);
    std::cerr << "[SerialOutput] incomplete/error write (" << writeError_
              << "); sender stopped (MCU timeout required)\n";
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
