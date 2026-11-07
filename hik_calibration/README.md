# 海康威视 USB 工业相机标定程序

## 功能

- 使用海康 MVS SDK 打开 USB 工业相机。
- 实时显示棋盘格角点。
- 按 `s` 保存当前图片，保存格式为 PNG。
- 按 `d` 删除刚刚保存的图片。
- 按 `Esc` 结束采图并开始标定。
- 退出时保存 `camera_intrinsics.yaml`，包含相机内参、畸变参数和每张图的重投影误差。
- 相机分辨率、偏移、曝光、增益、帧率可在 `config.yaml` 中设置。

## 依赖

- Jetson Orin Nano
- OpenCV
- 海康 MVS SDK，默认路径 `/opt/MVS`

## 编译

```bash
mkdir -p build
cd build
cmake .. -DMVS_ROOT=/opt/MVS
make -j$(nproc)
```

## 运行

```bash
./hik_calibration
```

如果在工程根目录运行，需要保证当前目录下有 `config.yaml`。
CMake 会把 `config.yaml` 复制到 `build` 目录，所以推荐在 `build` 目录运行。

## 配置

主要修改 `config.yaml`：

```yaml
board_width: 8
board_height: 11
square_size: 45

image_width: 0
image_height: 0
offset_x: 0
offset_y: 0

exposure_time: -1
gain: -1
frame_rate: -1
```

说明：

- `board_width` 和 `board_height` 是棋盘格内角点数量，不是方格数量。
- `square_size` 是单个方格边长，单位自己定，常用毫米。
- `image_width/image_height/offset_x/offset_y` 填 `0` 表示不设置。
- `exposure_time/gain/frame_rate` 填 `-1` 表示不设置。
- 如果设置曝光或增益，程序会先关闭对应的自动模式。

## 按键

- `s`：保存当前检测到角点的图片。
- `d`：删除刚刚保存的图片。
- `Esc`：结束采图，开始标定并保存结果。

## 输出

- 图片保存到 `save_dir` 配置的目录。
- 标定结果保存到 `output_file` 配置的文件，默认 `camera_intrinsics.yaml`。
