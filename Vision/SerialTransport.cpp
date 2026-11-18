#include "SerialTransport.hpp"

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#ifdef RM_HAVE_LIBUSB
#include <libusb-1.0/libusb.h>
#endif

namespace vision {
namespace {

using Clock = std::chrono::steady_clock;

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

bool nodeExists(const std::string& path) {
    return !path.empty() && ::access(path.c_str(), F_OK) == 0;
}

// ---------------------------------------------------------------- tty 后端

class TtyTransport final : public SerialTransport {
public:
    TtyTransport(std::string port, int baud) : port_(std::move(port)), baud_(baud) {}
    ~TtyTransport() override { close(); }

    bool open(std::string& error) override {
        close();
        fd_ = ::open(port_.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            error = "cannot open " + port_ + ": " + std::strerror(errno);
            return false;
        }
        const speed_t speed = toSpeedT(baud_);
        if (speed == 0) {
            error = "unsupported baud " + std::to_string(baud_);
            close();
            return false;
        }
        termios tty{};
        if (::tcgetattr(fd_, &tty) != 0) {
            error = "tcgetattr failed";
            close();
            return false;
        }
        ::cfsetispeed(&tty, speed);
        ::cfsetospeed(&tty, speed);
        tty.c_cflag = CLOCAL | CREAD | CS8;  // 只发送：8N1，无流控
        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
            error = "tcsetattr failed";
            close();
            return false;
        }
        return true;
    }

    bool write(const Frame& frame, std::string& error) override {
        std::size_t offset = 0;
        const auto deadline = Clock::now() + std::chrono::milliseconds(50);
        while (Clock::now() < deadline) {
            const ssize_t count = ::write(fd_, frame.data() + offset, frame.size() - offset);
            if (count > 0) {
                offset += static_cast<std::size_t>(count);
                if (offset == frame.size()) return true;
            } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
                       errno != EINTR) {
                error = std::strerror(errno);
                return false;
            } else {
                pollfd descriptor{fd_, POLLOUT, 0};
                const int ready = ::poll(&descriptor, 1, 5);
                if (ready > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
                    error = "port hung up";
                    return false;
                }
            }
        }
        error = "write timeout";
        return false;
    }

    void close() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool isOpen() const override { return fd_ >= 0; }
    std::string describe() const override { return "tty:" + port_; }

private:
    std::string port_;
    int baud_ = 115200;
    int fd_ = -1;
};

#ifdef RM_HAVE_LIBUSB
// ------------------------------------------------------------ CH341 后端

class Ch341Transport final : public SerialTransport {
public:
    explicit Ch341Transport(int baud) : baud_(baud) {}
    ~Ch341Transport() override { close(); }

    bool open(std::string& error) override {
        close();
        if (libusb_init(&ctx_) != 0) {
            ctx_ = nullptr;
            error = "libusb_init failed";
            return false;
        }
        libusb_device** list = nullptr;
        const ssize_t count = libusb_get_device_list(ctx_, &list);
        libusb_device_handle* handle = nullptr;
        int openRc = 0;
        for (ssize_t i = 0; i < count && handle == nullptr; ++i) {
            libusb_device_descriptor desc{};
            if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
            if (!isCh341(desc.idVendor, desc.idProduct)) continue;
            openRc = libusb_open(list[i], &handle);
            if (openRc != 0) handle = nullptr;
        }
        libusb_free_device_list(list, 1);
        if (handle == nullptr) {
            if (openRc != 0) {
                error = std::string("找到 CH341 但打不开（") + libusb_error_name(openRc) +
                        "），检查 /etc/udev/rules.d/99-ch341.rules 与设备权限";
            } else {
                error = "未找到 CH341/CH340（lsusb 查 1a86:5523 / 1a86:7523）";
            }
            close();
            return false;
        }
        handle_ = handle;

        if (libusb_kernel_driver_active(handle_, 0) == 1) {
            libusb_detach_kernel_driver(handle_, 0);
        }
        if (libusb_claim_interface(handle_, 0) != 0) {
            error = "claim USB 接口失败（通常需要 root 或 dialout 权限）";
            close();
            return false;
        }
        if (!findEndpoints(error) || !configure(error)) {
            close();
            return false;
        }
        return true;
    }

    bool write(const Frame& frame, std::string& error) override {
        int transferred = 0;
        const int rc = libusb_bulk_transfer(handle_, epOut_,
                                            const_cast<unsigned char*>(frame.data()),
                                            static_cast<int>(frame.size()), &transferred, 200);
        if (rc != 0) {
            error = std::string("bulk write: ") + libusb_error_name(rc);
            return false;
        }
        if (transferred != static_cast<int>(frame.size())) {
            error = "short write (" + std::to_string(transferred) + " B)";
            return false;
        }
        return true;
    }

    void close() override {
        if (handle_ != nullptr) {
            libusb_release_interface(handle_, 0);
            libusb_close(handle_);
            handle_ = nullptr;
        }
        if (ctx_ != nullptr) {
            libusb_exit(ctx_);
            ctx_ = nullptr;
        }
        epOut_ = 0;
        epIn_ = 0;
    }

    bool isOpen() const override { return handle_ != nullptr; }
    std::string describe() const override { return "libusb:ch341"; }

private:
    static bool isCh341(std::uint16_t vid, std::uint16_t pid) {
        if (vid != 0x1a86) return false;
        return pid == 0x5523 || pid == 0x7522 || pid == 0x7523;
    }

    bool findEndpoints(std::string& error) {
        libusb_config_descriptor* config = nullptr;
        if (libusb_get_active_config_descriptor(libusb_get_device(handle_), &config) != 0) {
            error = "读取 USB 配置描述符失败";
            return false;
        }
        const libusb_interface_descriptor& alt = config->interface[0].altsetting[0];
        for (int i = 0; i < alt.bNumEndpoints; ++i) {
            const libusb_endpoint_descriptor& ep = alt.endpoint[i];
            if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK) continue;
            if ((ep.bEndpointAddress & LIBUSB_ENDPOINT_IN) != 0) {
                epIn_ = ep.bEndpointAddress;
            } else {
                epOut_ = ep.bEndpointAddress;
            }
        }
        libusb_free_config_descriptor(config);
        if (epOut_ == 0) {
            error = "未找到 bulk OUT 端点";
            return false;
        }
        return true;
    }

    bool configure(std::string& error) {
        unsigned char version[2] = {0, 0};
        if (controlIn(kCh341ReqReadVersion, 0, 0, version, 2) < 2) {
            error = "读芯片版本失败";
            return false;
        }
        version_ = version[0];
        if (controlOut(kCh341ReqSerialInit, 0, 0) < 0) {
            error = "串口初始化(0xA1)失败";
            return false;
        }
        int divisor = ch341Divisor(baud_);
        if (divisor < 0) {
            error = "波特率 " + std::to_string(baud_) + " 不可用";
            return false;
        }
        if (version_ > 0x27) divisor |= (1 << 7);  // 禁止 32 字节分包缓冲
        const std::uint16_t regs =
            static_cast<std::uint16_t>((kCh341RegDivisor << 8) | kCh341RegPrescaler);
        if (controlOut(kCh341ReqWriteReg, regs, static_cast<std::uint16_t>(divisor)) < 0) {
            error = "写波特率分频失败";
            return false;
        }
        if (version_ >= 0x30) {
            const std::uint16_t lcrRegs =
                static_cast<std::uint16_t>((kCh341RegLcr2 << 8) | kCh341RegLcr);
            if (controlOut(kCh341ReqWriteReg, lcrRegs, kCh341Lcr8N1) < 0) {
                error = "写行控制寄存器失败";
                return false;
            }
        }
        const unsigned char modem = kCh341BitRts | kCh341BitDtr;
        if (controlOut(kCh341ReqModemCtrl, static_cast<std::uint16_t>(~modem & 0xFF), 0) < 0) {
            error = "设置 DTR/RTS 失败";
            return false;
        }
        return true;
    }

    int controlOut(std::uint8_t request, std::uint16_t value, std::uint16_t index) {
        return libusb_control_transfer(handle_, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
                                       request, value, index, nullptr, 0, 1000);
    }

    int controlIn(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                  unsigned char* buffer, std::uint16_t length) {
        return libusb_control_transfer(handle_, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
                                       request, value, index, buffer, length, 1000);
    }

    // 分频值计算，照抄内核 ch341_get_divisor（去掉 quirk 分支）。
    static int ch341Divisor(int speed) {
        constexpr std::uint32_t kClock = 48000000u;
        auto clkDiv = [](int ps, int fact) { return 1u << (12 - 3 * ps - fact); };

        std::uint32_t value = speed < 46 ? 46u : static_cast<std::uint32_t>(speed);
        if (value > 3000000u) value = 3000000u;

        int fact = 1;
        int ps = -1;
        for (int candidate = 3; candidate >= 0; --candidate) {
            if (value > kClock / (clkDiv(candidate, 1) * 512)) {
                ps = candidate;
                break;
            }
        }
        if (ps < 0) return -1;

        std::uint32_t clk = clkDiv(ps, fact);
        std::uint32_t div = kClock / (clk * value);
        if (div < 9 || div > 255) {
            div /= 2;
            clk *= 2;
            fact = 0;
        }
        if (div < 2) return -1;
        if (16u * kClock / (clk * div) - 16u * value >=
            16u * value - 16u * kClock / (clk * (div + 1))) {
            ++div;
        }
        if (fact == 1 && div % 2 == 0) {
            div /= 2;
            fact = 0;
        }
        return static_cast<int>(((0x100u - div) << 8) | (static_cast<unsigned>(fact) << 2) |
                                static_cast<unsigned>(ps));
    }

    static constexpr std::uint8_t kCh341ReqReadVersion = 0x5F;
    static constexpr std::uint8_t kCh341ReqWriteReg = 0x9A;
    static constexpr std::uint8_t kCh341ReqSerialInit = 0xA1;
    static constexpr std::uint8_t kCh341ReqModemCtrl = 0xA4;
    static constexpr std::uint8_t kCh341RegPrescaler = 0x12;
    static constexpr std::uint8_t kCh341RegDivisor = 0x13;
    static constexpr std::uint8_t kCh341RegLcr = 0x18;
    static constexpr std::uint8_t kCh341RegLcr2 = 0x25;
    static constexpr std::uint8_t kCh341Lcr8N1 = 0x80 | 0x40 | 0x03;
    static constexpr std::uint8_t kCh341BitRts = 1 << 6;
    static constexpr std::uint8_t kCh341BitDtr = 1 << 5;

    libusb_context* ctx_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    std::uint8_t epOut_ = 0;
    std::uint8_t epIn_ = 0;
    std::uint8_t version_ = 0;
    int baud_ = 115200;
};
#endif  // RM_HAVE_LIBUSB

}  // namespace

bool parseSerialBackend(const std::string& text, SerialBackend& out) {
    std::string lowered;
    lowered.reserve(text.size());
    for (char ch : text) {
        lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (lowered.empty() || lowered == "auto") {
        out = SerialBackend::Auto;
        return true;
    }
    if (lowered == "tty") {
        out = SerialBackend::Tty;
        return true;
    }
    if (lowered == "ch341" || lowered == "ch340" || lowered == "libusb") {
        out = SerialBackend::Ch341;
        return true;
    }
    return false;
}

std::unique_ptr<SerialTransport> createSerialTransport(SerialBackend backend,
                                                       const std::string& port,
                                                       int baud,
                                                       std::string& error) {
    const bool exists = nodeExists(port);
    switch (backend) {
        case SerialBackend::Tty:
            if (!exists) {
                error = port + " 不存在";
                return nullptr;
            }
            return std::make_unique<TtyTransport>(port, baud);
        case SerialBackend::Ch341:
#ifdef RM_HAVE_LIBUSB
            return std::make_unique<Ch341Transport>(baud);
#else
            error = "编译时未找到 libusb，无法直驱 CH341（apt-get install libusb-1.0-0-dev 后重编）";
            return nullptr;
#endif
        case SerialBackend::Auto:
        default:
            if (exists) return std::make_unique<TtyTransport>(port, baud);
#ifdef RM_HAVE_LIBUSB
            return std::make_unique<Ch341Transport>(baud);
#else
            error = port + " 不存在，且编译时未找到 libusb";
            return nullptr;
#endif
    }
}

}  // namespace vision
