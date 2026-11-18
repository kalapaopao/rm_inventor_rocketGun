/*
 * serial_link_test.c — TL3576 板端独立串口链路测试工具
 *
 * 背景：板端内核 6.1.115-rt16 未编译 ch341 驱动（CONFIG_USB_SERIAL_CH341 is not
 * set），/dev/ttyUSB0 永远不会出现，主视觉程序打不开串口。本工具不依赖内核驱动，
 * 用 libusb 在用户态直接驱动 CH341/CH340 USB-TTL 转接板完成链路测试：
 *   - 芯片初始化/波特率设置序列照抄 Linux v6.1 drivers/usb/serial/ch341.c
 *   - 帧编码（0xA5 | TARGET | int16 YAW LE | CRC16-Modbus LE，定长 6 字节）
 *     与 Vision/SerialProtocol.hpp 逐字节一致，电控端解码程序无需任何改动
 *
 * 用法（板端，root 运行）：
 *   sudo ./serial_link_test probe                 # 探测芯片 + 设 115200 8N1，不发送数据
 *   sudo ./serial_link_test tx [秒]               # 20Hz 发送 v1 无目标帧（默认 10s）
 *   sudo ./serial_link_test tx 30 --yaw 123       # 发送 TARGET=1、YAW=+123 的帧（电控会视为有效目标！）
 *   sudo ./serial_link_test loop [帧数]           # 自回环：发帧读回比对（先把转接板 TX-RX 短接）
 *   sudo ./serial_link_test rx [秒]               # 只收：滑动同步 + CRC 校验解码 v1 帧
 *   通用参数：--baud N（默认 115200）
 *
 * 编译：gcc -O2 -Wall -Wextra -o serial_link_test serial_link_test.c -lusb-1.0
 *
 * 安全：默认只发“无目标帧” A5 00 00 00 22 E8，协议要求电控对此不跟随、不动作；
 * 只有显式 --yaw 才会发送 TARGET=1 的有效目标帧。
 */

#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <libusb-1.0/libusb.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* ---------------- CH341 寄存器协议（源自 Linux v6.1 ch341.c） ---------------- */

#define CH341_REQ_READ_VERSION 0x5F
#define CH341_REQ_WRITE_REG    0x9A
#define CH341_REQ_READ_REG     0x95
#define CH341_REQ_SERIAL_INIT  0xA1
#define CH341_REQ_MODEM_CTRL   0xA4

#define CH341_REG_PRESCALER 0x12
#define CH341_REG_DIVISOR   0x13
#define CH341_REG_LCR       0x18
#define CH341_REG_LCR2      0x25

#define CH341_LCR_ENABLE_RX 0x80
#define CH341_LCR_ENABLE_TX 0x40
#define CH341_LCR_CS8       0x03   /* 8 数据位；无校验、1 停止位不需要额外位 */

#define CH341_BIT_RTS (1 << 6)
#define CH341_BIT_DTR (1 << 5)

#define CH341_CLKRATE            48000000u
#define CH341_CLK_DIV(ps, fact)  (1u << (12 - 3 * (ps) - (fact)))

static const uint16_t ch341_ids[][2] = {
    {0x1a86, 0x5523},  /* CH341 serial（本板实测就是这个） */
    {0x1a86, 0x7522},
    {0x1a86, 0x7523},  /* CH340 */
    {0x2184, 0x0057},
    {0x4348, 0x5523},
    {0x9986, 0x7523},
};

static int ch341_control_out(libusb_device_handle *h, uint8_t request,
                             uint16_t value, uint16_t index)
{
    return libusb_control_transfer(h, 0x40, request, value, index, NULL, 0, 1000);
}

static int ch341_control_in(libusb_device_handle *h, uint8_t request,
                            uint16_t value, uint16_t index, uint8_t *buf, int len)
{
    return libusb_control_transfer(h, 0xC0, request, value, index, buf, len, 1000);
}

/* 波特率分频值：算法照抄内核 ch341_get_divisor（仅去掉 quirk 分支） */
static int ch341_get_divisor(unsigned speed)
{
    unsigned fact, div, clk_div;
    int ps;

    if (speed < 46u) speed = 46u;                /* CH341_MIN_BPS */
    if (speed > 3000000u) speed = 3000000u;      /* CH341_MAX_BPS */

    fact = 1;
    for (ps = 3; ps >= 0; ps--) {
        if (speed > CH341_CLKRATE / (CH341_CLK_DIV(ps, 1) * 512))
            break;
    }
    if (ps < 0)
        return -1;

    clk_div = CH341_CLK_DIV(ps, fact);
    div = CH341_CLKRATE / (clk_div * speed);
    if (div < 9 || div > 255) {
        div /= 2;
        clk_div *= 2;
        fact = 0;
    }
    if (div < 2)
        return -1;

    /* 舍入到更接近的速率 */
    if (16u * CH341_CLKRATE / (clk_div * div) - 16u * speed >=
        16u * speed - 16u * CH341_CLKRATE / (clk_div * (div + 1)))
        div++;

    /* 偶数分频时优先低基准时钟（接收更宽容） */
    if (fact == 1 && div % 2 == 0) {
        div /= 2;
        fact = 0;
    }

    return (int)((0x100u - div) << 8 | fact << 2 | (unsigned)ps);
}

struct ch341_ctx {
    libusb_device_handle *h;
    uint8_t ep_out;
    uint8_t ep_in;
    uint8_t version;
};

/* 芯片初始化：115200 8N1。等价于内核 ch341_configure + ch341_set_termios。 */
static int ch341_init(struct ch341_ctx *c, unsigned baud)
{
    uint8_t buf[2] = {0, 0};
    uint8_t mcr;
    int r, val;

    r = ch341_control_in(c->h, CH341_REQ_READ_VERSION, 0, 0, buf, 2);
    if (r < 2) {
        fprintf(stderr, "[ch341] 读芯片版本失败: %s\n", libusb_error_name(r));
        return -1;
    }
    c->version = buf[0];
    printf("[ch341] 芯片版本 0x%02x\n", c->version);

    r = ch341_control_out(c->h, CH341_REQ_SERIAL_INIT, 0, 0);
    if (r < 0) {
        fprintf(stderr, "[ch341] 串口初始化(0xA1)失败: %s\n", libusb_error_name(r));
        return -1;
    }

    val = ch341_get_divisor(baud);
    if (val < 0) {
        fprintf(stderr, "[ch341] 波特率 %u 不可用\n", baud);
        return -1;
    }
    /* 版本 >0x27 的芯片：置 bit7，禁止 32 字节分包缓冲（否则收不到小块数据） */
    if (c->version > 0x27)
        val |= (1 << 7);
    r = ch341_control_out(c->h, CH341_REQ_WRITE_REG,
                          (uint16_t)(CH341_REG_DIVISOR << 8 | CH341_REG_PRESCALER),
                          (uint16_t)val);
    if (r < 0) {
        fprintf(stderr, "[ch341] 写波特率分频(0x9A/0x1312=0x%04X)失败: %s\n",
                val, libusb_error_name(r));
        return -1;
    }
    printf("[ch341] 波特率 %u -> 分频寄存器 0x%04X (实测约 %u bps)\n",
           baud, val,
           (unsigned)(CH341_CLKRATE / (CH341_CLK_DIV(val & 0x03, (val >> 2) & 1) *
                                       (0x100u - (uint8_t)(val >> 8)))));

    /* 版本 >=0x30：单独写 LCR（8N1 = RX|TX|CS8）。老版本芯片沿用默认格式。 */
    if (c->version >= 0x30) {
        uint8_t lcr = CH341_LCR_ENABLE_RX | CH341_LCR_ENABLE_TX | CH341_LCR_CS8;
        r = ch341_control_out(c->h, CH341_REQ_WRITE_REG,
                              (uint16_t)(CH341_REG_LCR2 << 8 | CH341_REG_LCR), lcr);
        if (r < 0) {
            fprintf(stderr, "[ch341] 写 LCR(0x9A/0x2518=0xC3)失败: %s\n",
                    libusb_error_name(r));
            return -1;
        }
    } else {
        printf("[ch341] 老版本芯片(<0x30)，8N1 沿用芯片默认行格式（同内核行为）\n");
    }

    /* 断言 DTR|RTS，与内核 open() 行为一致 */
    mcr = CH341_BIT_RTS | CH341_BIT_DTR;
    r = ch341_control_out(c->h, CH341_REQ_MODEM_CTRL, (uint16_t)~(uint16_t)mcr, 0);
    if (r < 0) {
        fprintf(stderr, "[ch341] 设置握手(0xA4)失败: %s\n", libusb_error_name(r));
        return -1;
    }

    return 0;
}

/* ---------------- v1 协议帧（与 Vision/SerialProtocol.hpp 一致） ---------------- */

#define V1_SOF       0xA5
#define V1_FRAME_LEN 6

static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001)
                            : (uint16_t)(crc >> 1);
    }
    return crc;
}

static void v1_encode(bool target, int16_t yaw, uint8_t frame[V1_FRAME_LEN])
{
    uint16_t bits = target ? (uint16_t)yaw : 0;
    uint16_t crc;

    frame[0] = V1_SOF;
    frame[1] = target ? 1 : 0;
    frame[2] = (uint8_t)(bits & 0xFF);
    frame[3] = (uint8_t)(bits >> 8);
    crc = crc16_modbus(frame, 4);
    frame[4] = (uint8_t)(crc & 0xFF);
    frame[5] = (uint8_t)(crc >> 8);
}

/* 用协议文档 §8 的已知帧做 CRC 自检，防止本工具编码错误污染测试结论 */
static int v1_selftest(void)
{
    static const struct { bool target; int16_t yaw; const char *hex; } cases[] = {
        { false, 0,    "A5 00 00 00 22 E8" },  /* 无目标 */
        { true,  123,  "A5 01 7B 00 51 D8" },  /* +123px */
        { true,  -123, "A5 01 85 FF 51 F8" },  /* -123px */
        { true,  0,    "A5 01 00 00 73 28" },  /* 有效零 */
        { true,  2000, "A5 01 D0 07 6F 2A" },  /* +2000px */
    };
    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        uint8_t f[V1_FRAME_LEN];
        char hex[32];
        v1_encode(cases[i].target, cases[i].yaw, f);
        snprintf(hex, sizeof hex, "%02X %02X %02X %02X %02X %02X",
                 f[0], f[1], f[2], f[3], f[4], f[5]);
        if (strcmp(hex, cases[i].hex) != 0) {
            fprintf(stderr, "FATAL: 编码自检失败: 得到 %s 期望 %s\n",
                    hex, cases[i].hex);
            return -1;
        }
    }
    printf("[selftest] v1 编码/CRC 与协议文档示例帧一致 (%zu/%zu)\n",
           ARRAY_SIZE(cases), ARRAY_SIZE(cases));
    return 0;
}

struct v1_decoder {
    unsigned frames_ok;
    unsigned crc_err;
    unsigned garbage;
};

/* 滑动同步解码，返回消费的字节数；尾部不足一帧的数据留给下一次。 */
static size_t v1_decode(const uint8_t *data, size_t len, struct v1_decoder *dec)
{
    size_t i = 0;
    while (i < len) {
        const uint8_t *f;
        uint16_t crc, got;
        int16_t yaw;

        if (data[i] != V1_SOF) {
            i++;
            dec->garbage++;
            continue;
        }
        if (len - i < V1_FRAME_LEN)
            break;
        f = data + i;
        crc = crc16_modbus(f, 4);
        got = (uint16_t)(f[4] | ((uint16_t)f[5] << 8));
        if (crc != got || f[1] > 1) {
            dec->crc_err++;
            printf("[rx] 帧校验失败，跳过 1 字节重同步\n");
            i++;
            continue;
        }
        yaw = (int16_t)(uint16_t)(f[2] | ((uint16_t)f[3] << 8));
        printf("[rx] frame #%u: TARGET=%u YAW=%+d (%02X %02X %02X %02X %02X %02X)\n",
               dec->frames_ok, f[1], yaw,
               f[0], f[1], f[2], f[3], f[4], f[5]);
        dec->frames_ok++;
        i += V1_FRAME_LEN;
    }
    return i;
}

/* ---------------- USB 设备发现与端点 ---------------- */

static libusb_device_handle *open_ch341(libusb_context *ctx, struct ch341_ctx *c)
{
    libusb_device **list;
    libusb_device_handle *h = NULL;
    ssize_t n = libusb_get_device_list(ctx, &list);
    int r;

    for (ssize_t i = 0; i < n && !h; i++) {
        struct libusb_device_descriptor d;
        if (libusb_get_device_descriptor(list[i], &d) != 0)
            continue;
        for (size_t k = 0; k < ARRAY_SIZE(ch341_ids); k++) {
            if (d.idVendor != ch341_ids[k][0] || d.idProduct != ch341_ids[k][1])
                continue;
            printf("[usb] 发现 %04x:%04x (bus %03d dev %03d)\n",
                   d.idVendor, d.idProduct,
                   libusb_get_bus_number(list[i]),
                   libusb_get_device_address(list[i]));
            r = libusb_open(list[i], &h);
            if (r != 0) {
                fprintf(stderr, "[usb] 打开设备失败: %s（通常需要 sudo）\n",
                        libusb_error_name(r));
                h = NULL;
            }
            break;
        }
    }
    libusb_free_device_list(list, 1);
    if (!h)
        return NULL;
    c->h = h;

    r = libusb_kernel_driver_active(h, 0);
    if (r == 1) {
        printf("[usb] 内核驱动占用接口 0，尝试 detach\n");
        r = libusb_detach_kernel_driver(h, 0);
        if (r != 0 && r != LIBUSB_ERROR_NOT_SUPPORTED)
            fprintf(stderr, "[usb] detach 失败: %s\n", libusb_error_name(r));
    }
    r = libusb_claim_interface(h, 0);
    if (r != 0) {
        r = libusb_set_configuration(h, 1);
        if (r != 0 && r != LIBUSB_ERROR_BUSY)
            fprintf(stderr, "[usb] set_configuration: %s\n", libusb_error_name(r));
        r = libusb_claim_interface(h, 0);
    }
    if (r != 0) {
        fprintf(stderr, "[usb] claim 接口 0 失败: %s\n", libusb_error_name(r));
        libusb_close(h);
        return NULL;
    }
    return h;
}

static int find_endpoints(struct ch341_ctx *c)
{
    struct libusb_config_descriptor *cfg;
    const struct libusb_interface_descriptor *alt;

    if (libusb_get_active_config_descriptor(libusb_get_device(c->h), &cfg) != 0)
        return -1;
    alt = &cfg->interface[0].altsetting[0];
    for (int i = 0; i < alt->bNumEndpoints; i++) {
        const struct libusb_endpoint_descriptor *ep = &alt->endpoint[i];
        uint8_t addr = ep->bEndpointAddress;
        const char *type = (ep->bmAttributes & 0x03) == LIBUSB_TRANSFER_TYPE_BULK
                               ? "bulk" : "interrupt";
        printf("[usb] 端点 0x%02X (%s, %s, maxpacket %u)\n", addr, type,
               (addr & 0x80) ? "IN" : "OUT", ep->wMaxPacketSize);
        if ((ep->bmAttributes & 0x03) == LIBUSB_TRANSFER_TYPE_BULK) {
            if (addr & 0x80)
                c->ep_in = addr;
            else
                c->ep_out = addr;
        }
    }
    libusb_free_config_descriptor(cfg);
    if (!c->ep_out) {
        fprintf(stderr, "[usb] 未找到 bulk OUT 端点\n");
        return -1;
    }
    return 0;
}

/* ---------------- 主流程 ---------------- */

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static void sleep_ms(long ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static int run_tx(struct ch341_ctx *c, double duration_s, bool target, int16_t yaw)
{
    uint8_t frame[V1_FRAME_LEN];
    unsigned frames = 0, bytes = 0, errors = 0;
    double t0 = now_sec(), last_stat = t0;

    v1_encode(target, yaw, frame);
    printf("[tx] %s：每 50ms 一帧，共 %.0fs，帧内容 %02X %02X %02X %02X %02X %02X\n",
           target ? "有效目标帧（电控会跟随！）" : "无目标帧（安全，电控不动作）",
           duration_s, frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);

    while (!g_stop && now_sec() - t0 < duration_s) {
        int transferred = 0;
        int r = libusb_bulk_transfer(c->h, c->ep_out, frame, V1_FRAME_LEN,
                                     &transferred, 1000);
        if (r == 0 && transferred == V1_FRAME_LEN) {
            frames++;
            bytes += (unsigned)transferred;
        } else {
            errors++;
            fprintf(stderr, "[tx] bulk write 失败: %s\n", libusb_error_name(r));
            if (errors >= 10) {
                fprintf(stderr, "[tx] 连续失败过多，中止\n");
                return 1;
            }
        }
        sleep_ms(50);
        if (now_sec() - last_stat >= 1.0) {
            last_stat = now_sec();
            printf("[tx] %.0fs: %u 帧 / %u 字节 / %u 错误\n",
                   now_sec() - t0, frames, bytes, errors);
        }
    }
    printf("[tx] 结束：共 %u 帧 / %u 字节 / %u 错误，平均 %.1f Hz\n",
           frames, bytes, errors, frames / (now_sec() - t0));
    return errors == 0 && frames > 0 ? 0 : 1;
}

static int run_loop(struct ch341_ctx *c, unsigned count)
{
    uint8_t expected[32 * 64];
    uint8_t rbuf[512];
    size_t sent_bytes = 0, matched = 0, recv_bytes = 0;
    unsigned sent_frames = 0;

    printf("[loop] 自回环测试：发 %u 帧并读回比对。请先把转接板 TX 与 RX 短接！\n",
           count);
    for (unsigned i = 0; i < count && !g_stop; i++) {
        uint8_t frame[V1_FRAME_LEN];
        int transferred = 0;
        int r;

        v1_encode(i & 1, (int16_t)(i * 7 - 200), frame);  /* 多种字节模式 */
        memcpy(expected + sent_bytes, frame, V1_FRAME_LEN);
        sent_bytes += V1_FRAME_LEN;
        sent_frames++;

        r = libusb_bulk_transfer(c->h, c->ep_out, frame, V1_FRAME_LEN,
                                 &transferred, 1000);
        if (r != 0) {
            fprintf(stderr, "[loop] 发送失败: %s\n", libusb_error_name(r));
            return 1;
        }

        /* 读回：0.4s 内应能收到刚发的 6 字节 */
        int got = 0;
        r = libusb_bulk_transfer(c->h, c->ep_in, rbuf, sizeof rbuf, &got, 400);
        if (r == 0 && got > 0) {
            size_t keep = matched;
            for (int k = 0; k < got; k++) {
                if (keep + (size_t)k < sent_bytes &&
                    rbuf[k] == expected[keep + (size_t)k])
                    matched++;
            }
            recv_bytes += (size_t)got;
        } else if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) {
            fprintf(stderr, "[loop] 读取失败: %s\n", libusb_error_name(r));
        }
        if ((i + 1) % 20 == 0)
            printf("[loop] %u/%u 帧，回读 %zu B，匹配 %zu B\n",
                   i + 1, count, recv_bytes, matched);
    }

    printf("[loop] 结果：发送 %u 帧 / %zu B，回读 %zu B，逐字节匹配 %zu B (%.1f%%)\n",
           sent_frames, sent_bytes, recv_bytes, matched,
           sent_bytes ? 100.0 * matched / sent_bytes : 0.0);
    if (matched >= sent_bytes - V1_FRAME_LEN) {
        printf("[loop] PASS：转接板发送与接收链路正常（收到且与发送一致）\n");
        return 0;
    }
    printf("[loop] FAIL：未短接 TX-RX（回读 0B，仅发送链路可用）或回读数据不一致\n");
    return 1;
}

static int run_rx(struct ch341_ctx *c, double duration_s)
{
    uint8_t acc[4096];
    uint8_t rbuf[512];
    size_t acc_len = 0;
    struct v1_decoder dec = {0, 0, 0};
    double t0 = now_sec();

    if (!c->ep_in) {
        fprintf(stderr, "[rx] 该设备没有 bulk IN 端点，无法接收\n");
        return 1;
    }
    printf("[rx] 只收模式 %.0fs：滑动同步 + CRC16 校验解码 v1 帧\n", duration_s);
    while (!g_stop && now_sec() - t0 < duration_s) {
        int got = 0;
        int r = libusb_bulk_transfer(c->h, c->ep_in, rbuf, sizeof rbuf, &got, 1000);
        if (r == 0 && got > 0) {
            size_t used;
            if (acc_len + (size_t)got > sizeof acc) {  /* 防积压 */
                memmove(acc, acc + V1_FRAME_LEN, acc_len - V1_FRAME_LEN);
                acc_len -= V1_FRAME_LEN;
            }
            memcpy(acc + acc_len, rbuf, (size_t)got);
            acc_len += (size_t)got;
            used = v1_decode(acc, acc_len, &dec);
            if (used > 0) {
                memmove(acc, acc + used, acc_len - used);
                acc_len -= used;
            }
        } else if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) {
            fprintf(stderr, "[rx] 读取失败: %s\n", libusb_error_name(r));
            return 1;
        }
    }
    printf("[rx] 结束：有效帧 %u，校验失败 %u，垃圾字节 %u\n",
           dec.frames_ok, dec.crc_err, dec.garbage);
    return dec.frames_ok > 0 ? 0 : 1;
}

static void usage(const char *prog)
{
    printf(
        "用法: sudo %s <mode> [参数] [--baud N] [--yaw N]\n"
        "  probe             探测 CH341/CH340 + 设 115200 8N1（不发数据）\n"
        "  tx [秒]           20Hz 发送 v1 无目标帧（默认 10s，安全）\n"
        "  tx [秒] --yaw N   发送 TARGET=1、YAW=N 的帧（电控视为有效目标）\n"
        "  loop [帧数]       自回环测试（先短接转接板 TX-RX），默认 50 帧\n"
        "  rx [秒]           只收并解码 v1 帧，默认 10s\n"
        "示例: sudo %s probe && sudo %s tx 10\n", prog, prog, prog);
}

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "help";
    long param = 10;
    long yaw = 0;
    bool yaw_set = false;
    unsigned baud = 115200;
    libusb_context *ctx = NULL;
    struct ch341_ctx c = {0};
    int rc = 1;

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            baud = (unsigned)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--yaw") == 0 && i + 1 < argc) {
            yaw = strtol(argv[++i], NULL, 0);
            yaw_set = true;
        } else if (isdigit((unsigned char)argv[i][0])) {
            param = strtol(argv[i], NULL, 0);
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, on_sigint);

    if (strcmp(mode, "help") == 0 || strcmp(mode, "--help") == 0) {
        usage(argv[0]);
        return 0;
    }
    if (strcmp(mode, "probe") && strcmp(mode, "tx") &&
        strcmp(mode, "loop") && strcmp(mode, "rx")) {
        usage(argv[0]);
        return 1;
    }
    if (yaw < -32768 || yaw > 32767) {
        fprintf(stderr, "--yaw 超出 int16 范围\n");
        return 1;
    }

    if (v1_selftest() != 0)
        return 1;

    if (libusb_init(&ctx) != 0) {
        fprintf(stderr, "[usb] libusb 初始化失败\n");
        return 1;
    }

    if (!open_ch341(ctx, &c)) {
        fprintf(stderr, "结论: 未找到/无法打开 CH341 转接板（lsusb 确认 1a86:5523；需要 sudo）\n");
        libusb_exit(ctx);
        return 1;
    }
    if (find_endpoints(&c) != 0)
        goto out;

    printf("[usb] 芯片就绪，开始初始化串口（baud=%u 8N1）\n", baud);
    if (ch341_init(&c, baud) != 0) {
        fprintf(stderr, "结论: 芯片初始化失败——USB 链路异常或不是标准 CH341\n");
        goto out;
    }
    printf("[ch341] 初始化完成（用户态驱动，未依赖内核模块）\n");

    if (strcmp(mode, "probe") == 0) {
        uint8_t st[2] = {0, 0};
        if (ch341_control_in(c.h, CH341_REQ_READ_REG, 0x0706, 0, st, 2) == 2)
            printf("[ch341] 线路状态 0x%02x 0x%02x\n", st[0], st[1]);
        printf("PASS: 设备识别 + 115200 8N1 配置成功。可运行 tx 让电控验证接收。\n");
        rc = 0;
    } else if (strcmp(mode, "tx") == 0) {
        rc = run_tx(&c, (double)param, yaw_set, (int16_t)yaw);
    } else if (strcmp(mode, "loop") == 0) {
        rc = run_loop(&c, (unsigned)param);
    } else if (strcmp(mode, "rx") == 0) {
        rc = run_rx(&c, (double)param);
    }

out:
    libusb_release_interface(c.h, 0);
    libusb_close(c.h);
    libusb_exit(ctx);
    return rc;
}
