#pragma once

// `show --serial-test`：串口链路稳定性测试。
//
// 直接驱动项目**当前正式的发送模块** vision::SerialOutput（不另写一套传输实现），
// 只把目标来源换成"永远没有新鲜结果"，于是每个周期发出协议 v1 的无目标帧
// （`A5 00 00 00 22 E8`），与正式运行中"链路活着但视野内无目标"走的是同一条代码路径。
//
// 因此：**--serial-test 通过 = 项目的发送模块能正常往电控发帧**。
// 它不打开相机、不启动视觉管线，可长时间空跑；电控收到无目标帧应保持不跟随、不动作。
//
// 设备节点不存在时（例如板端内核未编 ch341 驱动、CH341 转接板无 /dev/ttyUSB0）
// 会明确报失败——这正是正式程序同样会遇到的状态，不做任何绕过。
//
// 文档：docs/串口链路测试.md

#include <cstdint>
#include <string>

namespace vision {

struct LinkTestConfig {
    std::string port;   // 取自 vision.serial_port
    int baud = 115200;  // 取自 vision.serial_baud
    int periodMs = 50;  // 取自 vision.serial_period_ms
};

// 阻塞运行，直到 SIGINT/SIGTERM（Ctrl+C）后打印汇总并返回。
// 返回 0：正式发送模块正常发出帧且未中断；1：无法打开端口 / 发送中断 / 一帧未发。
int runSerialLinkTest(const LinkTestConfig& config);

}  // namespace vision
