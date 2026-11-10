# Hikrobot camera core (Linux / WSL)

当前程序只保留海康 MVS 相机的核心链路：读取 YAML、打开相机、采集图像、计算 FPS、`putText`、`imshow`，以及可选的 10 秒录像。

## 目录

```text
Camera/
  CameraSetting.hpp/.cpp   # MVS 相机、图像转换、FPS 和显示
  VideoRecorder.hpp/.cpp   # AVI 写盘
  camera_config.yaml       # 相机配置
  CMakeLists.txt
main.cpp                   # 命令行参数和配置覆盖
CMakeLists.txt
```

网页模块、Windows 启动方式和项目启动脚本已经移除。

## WSL 构建

```bash
cd /mnt/d/study/rm_inventor_fly
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

## 运行

进入构建目录后直接运行：

```bash
cd /mnt/d/study/rm_inventor_fly/build
./show                  # 不录像，不显示窗口
./show --window         # 强制显示 OpenCV 窗口
./show --recording      # 按 YAML 的 record_seconds 录像
./show --recording --window
```

也可以指定配置文件：

```bash
./show --config /mnt/d/study/rm_inventor_fly/Camera/camera_config.yaml
```

命令行参数只覆盖当前进程内存中的配置，不会改写 YAML 文件。

## 配置

```yaml
%YAML:1.0
---
camera:
  device_index: 0
  width: 1440
  height: 1080
  exposure_time: 2500.0
  gain: 9.0
  grab_timeout_ms: 100
  acquisition_frame_rate_enable: 0
  adc_bit_depth: 8
  bayer_quality: 0
  window_name: "Hikrobot Camera"
  record_seconds: 10.0
```

不加 `--window` 时不显示窗口；加上 `--window` 后才调用 `imshow`。不加 `--recording` 时不会录像；加上 `--recording` 后才使用 YAML 中的 `record_seconds`。录像输出到项目根目录的 `recordings/`。

## 依赖

- Linux/WSL，CMake 3.16+
- OpenCV：core、imgproc、highgui、videoio
- Hikrobot MVS SDK，默认安装在 `/opt/MVS`

## 视觉管线（可选）

当前使用纯 OpenCV 绿色发光圆灯检测，不加载训练模型，也不依赖 ONNX Runtime/NPU。

- `vision.enabled: 1` 启用传统检测，`0` 保持纯取流。
- `vision.backend` 必须为 `traditional`；旧模型配置需要更新。
- 在 `vision.green_light` 调整 HSV、面积、圆度与轴比门限。
- 保留标定灯心角度、5秒可调预测、长十字准心及8字节像素偏差串口。
- 运行 `./run.sh --window`；串口默认关闭，按 `docs/SerialYawProtocol.md` 对接。
- 单图检查：`build/Vision/vision_image_test input.jpg build/detected.png`。
- 详情见 `docs/TraditionalVision.md`，旧模型交接报告仅作历史参考。
