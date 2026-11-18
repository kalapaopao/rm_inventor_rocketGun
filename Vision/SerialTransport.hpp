#pragma once

// 串口传输层：只负责"把一帧字节写到线上"，不含协议、线程与重试策略。
//
// 两个后端：
//   Tty    —— 内核 tty 设备（/dev/ttyUSB0 等）。需要内核驱动，CP2102/FT232 内置，
//             CH340/CH341 在板端内核未编（CONFIG_USB_SERIAL_CH341 is not set）时不会有节点。
//   Ch341  —— libusb 用户态直驱 CH341/CH340，不需要内核驱动。寄存器序列照抄
//             Linux v6.1 drivers/usb/serial/ch341.c。仅在编译期找到 libusb 时可用。
//
// Auto 选择顺序：设备节点存在就用 Tty（正式、可靠）；否则回退 Ch341（本板现状）。
// 换 CP2102/FT232 后无需改配置，会自动走 Tty。

#include <memory>
#include <string>

#include "SerialProtocol.hpp"

namespace vision {

enum class SerialBackend {
    Auto = 0,  // 节点存在用 Tty，否则尝试 Ch341
    Tty,       // 仅内核 tty 设备
    Ch341,     // 仅 libusb 直驱 CH341/CH340
};

// 解析 "auto" / "tty" / "ch341"（大小写不敏感）；未知值返回 false。
bool parseSerialBackend(const std::string& text, SerialBackend& out);

class SerialTransport {
public:
    virtual ~SerialTransport() = default;
    virtual bool open(std::string& error) = 0;
    virtual bool write(const Frame& frame, std::string& error) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    // 形如 "tty:/dev/ttyUSB0" 或 "libusb:ch341"，用于日志。
    virtual std::string describe() const = 0;
};

// 创建并返回传输实例（尚未 open）。返回 nullptr 时 error 说明原因。
std::unique_ptr<SerialTransport> createSerialTransport(SerialBackend backend,
                                                       const std::string& port,
                                                       int baud,
                                                       std::string& error);

}  // namespace vision
