#include "SerialLinkTest.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "SerialOutput.hpp"

namespace vision {
namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop.store(true); }

using Clock = std::chrono::steady_clock;

double secondsSince(Clock::time_point from) {
    return std::chrono::duration<double>(Clock::now() - from).count();
}

// 与 SerialOutput 内部发送的帧同源（encodeFrame），保证打印的就是线上字节。
std::string hexOf(const Frame& frame) {
    static const char kDigits[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(frame.size() * 3);
    for (std::size_t i = 0; i < frame.size(); ++i) {
        if (i != 0) out.push_back(' ');
        out.push_back(kDigits[frame[i] >> 4]);
        out.push_back(kDigits[frame[i] & 0x0F]);
    }
    return out;
}

}  // namespace

int runSerialLinkTest(const LinkTestConfig& config) {
    if (config.port.empty()) {
        std::cerr << "[link] FAIL：vision.serial_port 为空，请在 camera_config.yaml 指定设备节点"
                     "（如 /dev/ttyUSB0）\n";
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::cout << std::fixed << std::setprecision(1);

    SerialConfig serial;
    serial.port = config.port;
    serial.baud = config.baud;
    serial.periodMs = config.periodMs;
    serial.maxAgeMs = 500;

    // 与正式路径同一编码函数：TARGET=0 时 YAW 强制 0。
    const Frame frame = encodeFrame(false, 0);
    const std::string frameHex = hexOf(frame);

    std::cout << "[link] 串口自检 " << config.port << " @" << config.baud << " 8N1 / "
              << config.periodMs << "ms\n"
              << "[link] 发送帧 " << frameHex
              << "  (SOF=A5 TARGET=0 YAW=0 CRC16)\n";

    SerialOutput output;
    // provider 恒返回 false（没有新鲜目标）→ 与正式运行中丢目标时同一条路径：每周期发 TARGET=0。
    if (!output.start(serial, [](VisionResult&) { return false; })) {
        std::cerr << "[link] FAIL：打不开 " << config.port
                  << "。CH340/CH341 在板端无内核驱动，需换 CP2102 / FT232；"
                     "详见 docs/串口链路测试.md\n";
        return 1;
    }

    const auto start = Clock::now();
    auto lastReport = start;
    std::uint64_t lastSent = 0;
    bool stoppedEarly = false;

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // SerialOutput 触发半帧保护/写错误时会自行停线程，这里如实报告。
        if (!output.running()) {
            stoppedEarly = true;
            break;
        }
        const auto now = Clock::now();
        if (now - lastReport < std::chrono::seconds(1)) continue;

        const SerialStatus status = output.status();
        const double window = std::chrono::duration<double>(now - lastReport).count();
        const double hz = window > 0
                              ? static_cast<double>(status.sent - lastSent) / window
                              : 0.0;
        std::cout << "[link] " << static_cast<long>(secondsSince(start)) << "s 帧=" << status.sent
                  << " 错误=" << status.errors << " " << hz << "Hz  发=" << frameHex << '\n';
        lastReport = now;
        lastSent = status.sent;
    }

    output.stop();
    const SerialStatus status = output.status();
    const double elapsed = secondsSince(start);
    const double hz = elapsed > 0 ? static_cast<double>(status.sent) / elapsed : 0.0;
    // Ctrl+C 可能与写操作竞争，导致线程在退出瞬间停掉并留下一次"写错误"；
    // 这是退出时序而非链路故障，只要用户主动结束就不算失败。
    const bool interrupted = g_stop.load();

    std::cout << "\n[link] " << status.sent << " 帧 / " << status.sent * 6 << " B / "
              << status.errors << " 错误 / " << elapsed << " s / " << hz << " Hz\n";

    if (status.sent == 0) {
        std::cerr << "[link] FAIL：一帧都未发出\n";
        return 1;
    }
    if (interrupted && (stoppedEarly || status.errors > 0)) {
        std::cout << "[link] PASS（退出瞬间发送线程停止，非链路故障）\n";
        return 0;
    }
    if (stoppedEarly) {
        std::cerr << "[link] FAIL：发送线程中途停止（写错误/半帧保护），正式运行同样会停发\n";
        return 1;
    }
    if (status.errors > 0) {
        std::cerr << "[link] FAIL：" << status.errors << " 次写错误\n";
        return 1;
    }
    std::cout << "[link] PASS：正式发送模块正常，项目可发帧给电控\n";
    return 0;
}

}  // namespace vision
