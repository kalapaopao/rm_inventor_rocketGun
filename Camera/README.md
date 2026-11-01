# RM Inventor Fly：Linux/WSL 相机工程

本工程的 C++ 主路径是 **Linux/WSL**。源码仍放在 Windows 目录中，通过 `/mnt/d/study/rm_inventor_fly` 直接读写；Windows 侧只负责编辑器、浏览器和保留的启动脚本。

## 目录结构

```text
.
├── Camera/                 # 相机适配、采集、显示和录像
│   ├── CameraSetting.*
│   ├── VideoRecorder.*
│   ├── camera_config.yaml
│   └── CMakeLists.txt
├── Web/                    # 独立的 MJPEG over HTTP 实时画面模块
│   ├── MjpegServer.hpp
│   ├── MjpegServer.cpp
│   └── CMakeLists.txt
├── tools/                  # Linux 工具和可选的 WSL usbipd 辅助脚本
├── main.cpp                # 命令行解析和模块组装
├── CMakeLists.txt          # Linux 根构建入口
└── run.sh                  # Linux/WSL 唯一启动入口
```

模块边界：

- `CameraSetting` 只负责 MVS 相机、取帧和可选的本地预览/录像。
- `Web/MjpegServer` 只负责接收最新帧、缩放/JPEG 编码、HTTP 页面和客户端连接。
- `VideoRecorder` 只负责 AVI 写盘。
- `main.cpp` 只负责参数、配置和依赖注入。相机通过 `FrameSink` 把帧交给网页模块，不知道 HTTP 细节。

## WSL 中构建和运行

```bash
cd /mnt/d/study/rm_inventor_fly
./run.sh                 # 构建并启动预览 + 网页流
./run.sh --no-window     # 无 GUI，只提供网页流/录像
./run.sh --recording=30 # 录像 30 秒后退出
```

浏览器打开：`http://localhost:8080/`。

`run.sh` 是唯一启动入口。检测到 WSL 且安装了 `usbipd.exe` 时，才调用 `tools/wsl_usbipd.sh` 做相机 attach 和可选的后台重连。Windows 端只使用浏览器访问 `http://localhost:8080/`，不再负责启动程序。

如果使用全新的构建目录，也可以：

```bash
cmake -S . -B build-linux -DCMAKE_BUILD_TYPE=Release
cmake --build build-linux -j"$(nproc)"
```

默认要求 Hikrobot MVS SDK 位于 `/opt/MVS`。可以用 `-DMVS_ROOT=/path/to/MVS` 覆盖。

## 配置

`Camera/camera_config.yaml` 中的 `camera:` 和 `stream:` 两段互相独立：

```yaml
camera:
  device_index: 0
  width: 1440
  height: 1080
  display_enabled: 1
  display_fps: 60.0
  record_seconds: 10.0

stream:
  enabled: 1
  port: 8080
  width: 960
  fps: 15.0
  quality: 65
```

## 迁移到 Jetson Nano / 树莓派

可复用的核心是 `CameraSetting` 的帧处理接口、`Web/MjpegServer` 和算法模块；不要把 `run.sh` 的 WSL/usbipd 逻辑带到开发板。迁移时只需要替换：

1. 相机 SDK/驱动和 `CameraSetting` 的设备适配；
2. OpenCV、MVS（如果设备支持）和编译器依赖；
3. 启动脚本中的设备发现方式。

网页模块只依赖 Linux socket、线程和 OpenCV 编码，绑定到 `0.0.0.0` 后可由同一局域网设备访问。
