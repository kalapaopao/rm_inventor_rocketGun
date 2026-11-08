# Vision 接入实施报告（交接文档）

执行日期：2026-11-08
执行依据：`docs/海康相机目标识别接入方案.md`
执行环境：Windows 主机 + WSL Ubuntu-22.04（工程仅支持 Linux/WSL 构建）
状态：**方案 §9.1~§9.9 已完成并真机验证；§9.10 的 30 分钟稳定性测试已启动
但被 usbipd 链路中断（见 §3 表格与 §4.7 事故记录），链路正常期间的
实测数据稳定，完整 30 分钟重跑待相机恢复后进行。**

---

## 1. 一句话总结

在不改动 MVS 采集方式的前提下，新增 `Vision/` 模块，把
`best.onnx`（YOLO26n 端到端，1×300×6）接入取流程序，完成
推理叠框、ROI 传统视觉复筛、卡尔曼跟踪和可配置的未来位置预测，
并实现了方案要求的全部统计与开关回退。默认 `vision.enabled: 0`，
行为与原纯取流完全一致。

---

## 2. 做了什么 / 怎么做的

### 2.1 只读检查（方案 §9.1）

- 工程：`Camera/`（MVS 取流 + OutputWorker 显示录像）+ `hik_calibration/`，
  仅 Linux/WSL 构建（`run.sh` 负责 usbipd attach + noreset.so + MVS 库路径）。
- 环境：WSL 内 cmake 3.22 / g++ 11 / OpenCV 4.5.4 / MVS SDK `/opt/MVS`；
  无 ONNX Runtime（C++ 与 Python 均无），Windows 侧 Python 3.11 有 ORT 1.31.1。
- §11.3 提到的 `captureMs = captureMs` 遮蔽 bug **在当前代码中不存在**
  （`CameraSetting.cpp` 中已是正确的 `captureMs = captureMs_`），
  但 §11.5 四类 FPS 缺失属实，已补齐。

### 2.2 模型离线验证（方案 §9.2，先实测后编码）

用 Windows Python + onnxruntime 对 `best.onnx` 实测（脚本
`%TEMP%/dsh-vision-probe/verify_model.py`，对数据集 189 张标注图中的前 4 张）：

| 项 | 确认结果 |
|---|---|
| 输入 | `images [1,3,640,640]` FP32 NCHW，RGB，/255 归一化 |
| 输出 | `output0 [1,300,6] = [x1,y1,x2,y2,score,cls]`，**640 letterbox 输入空间坐标** |
| NMS | 端到端内置（TopK 300），无需再做 NMS |
| 类别 | 1 类，cls_id=0 |
| 精度 | 映射回原图后与 labelme 标注 IoU 0.85~0.95 |

坐标映射公式：`orig = (model - pad) / scale`。**这是踩过的坑：**第一次对比
时误用了 640 空间原始坐标和原图标注算 IoU，得到 0.004 的假结果；
修正映射后再算才是 0.85~0.95。

### 2.3 ONNX Runtime C++ 包（方案 §6 的前置）

- 下载官方 `onnxruntime-linux-x64-1.31.1.tgz`（GitHub 直连和镜像均 504，
  最终用 `api.github.com` + `Accept: application/octet-stream` 走 API 域名拿到），
  **SHA256 与 release 元数据一致**：`C849E36A...B16BDC9`。
- 放在 `third_party/onnxruntime-linux-x64-1.31.1/`（.gitignore 已排除整个
  `/third_party/`）。**注意：解压必须在 WSL 内做**，Windows tar 建不了 `.so` 符号链接。
- 链接库是 `libonnxruntime.so.1.31.1`（35 MB），运行需要
  `LD_LIBRARY_PATH` 指到其 lib 目录（`vision_single_test` 和 `show` 都已设
  BUILD_RPATH，直接运行即可）。
- 1.31 的 API 注意点：优化等级枚举是
  `GraphOptimizationLevel::ORT_ENABLE_ALL`（不是 `AllOptimization`）；
  `GetInputNameAllocated` 返回的智能指针必须先保存再取 `.get()`，否则悬垂。

### 2.4 新增文件清单（全部在 `Vision/`）

```text
Vision/
  DetectionTypes.hpp      # Detection / TrackedTarget / VisionStats / VisionResult
  InferenceBackend.hpp    # IInferenceBackend，签名与方案 §6 完全一致
  OnnxRuntimeBackend.hpp/.cpp  # CPU 后端，Pimpl 隐藏 ORT 头依赖
  Preprocess.hpp          # letterbox / decodeEnd2End（内联，无 cpp）
  TargetTracker.hpp/.cpp  # [x,y,vx,vy] 卡尔曼，CV 模型，可配置预测
  VisionPipeline.hpp/.cpp # 推理线程：最新帧覆盖队列+复筛+跟踪+统计
  SerialOutput.hpp/.cpp   # 串口线程，默认关闭，协议见 2.7
  SingleImageTest.cpp     # 单图推理测试工具（方案 §9.4）
  CMakeLists.txt          # vision_core 静态库 + vision_single_test
```

设计要点：

- **队列深度恒为 1**（方案 §5.2）：`submitFrame()` 非阻塞，新帧覆盖旧帧，
  丢弃计数 `droppedFrames`。实测推理 6fps、采集 40fps 时 p50 延迟稳定不增长。
- **帧零拷贝传递**：采集线程把转换后的 BGR Mat `std::move` 给管线，
  只在发布结果时 clone 一份叠框图给显示/录像。
- **复筛**（`refineDetection`）：ROI 内绿色像素占比（HSV 35~85）或
  高亮像素占比（V>200）达到阈值即通过；未通过的框不进跟踪器。
- **跟踪可靠性**（方案 §7.4）：`prediction_valid` 需要
  确认帧数达标 + 有匹配 + horizon≤max + 无速度突变（突变后需连续 5 帧正常才恢复）
  + 位置协方差标准差 ≤ `uncertaintyLimit`。丢失超 `lost_timeout_ms` 直接重置。
  预测只做外推，**绝不回写滤波器**（方案 §7.1）。
- **结果序号**：`VisionResult.sequence` 每次发布递增；消费者（显示/录像）
  用序号去重，避免同一快照被重复消费。这是真机测试中发现的 bug
  （displayFps 一度显示 5.9e6、录像重复写同帧），已修复。

### 2.5 Camera 侧接入（最小改动）

- `CameraConfig` 增加 `vision::VisionConfig vision`，`load()` 解析 `vision:` 块
  （字段与方案 §8 一一对应，另加 `serial_port/baud/period_ms`）；
  `enabled=1` 时校验 model_path 非空、horizon ≤ max。
- **Windows 盘符路径自动转换**：方案示例配置用 `D:/study/...`，程序在 WSL
  运行时自动转成 `/mnt/d/study/...`，同一份配置两端可用。
- `CameraSetting::run()`：`vision.enabled=1` 时创建管线，**模型加载失败
  打印明确错误并退出**（验收标准）；串口打不开只告警不拖死采集。
- 采集循环：vision 模式下 `submitFrame(std::move(image), ...)` 后 continue，
  不走旧的 OutputWorker 提交路径。
- `OutputWorker` 拆成 `legacyLoop()`（原行为，一行没改语义）和
  `visionLoop()`（消费管线快照：录像 + 显示 + 每 2s 打印四类 FPS/延迟/
  丢帧/丢失统计）。§11.1 的解耦要求由此满足。
- 取图超时（如 0x80000007）从"立即致命退出"改为**连续 50 次失败才退出**
  ——usbipd 虚拟链路下偶发超时，之前会直接终止整个程序。
- CMake：根 `CMakeLists.txt` 中 **Vision 必须先于 Camera 配置**
  （Camera 依据 `RM_VISION_ENABLED` 决定是否链接 `vision_core`）。
  缺 ONNX Runtime 时整个 Vision 子目录 `return()`，
  `show` 编译为纯取流程序，只是配置里 vision 段会被忽略并告警。

### 2.6 配置（方案 §8）

`Camera/camera_config.yaml` 已加完整 `vision:` 块，默认 `enabled: 0`。
测试用配置在 `build/camera_config_vision_test.yaml`（enabled:1，不入库）。
**修改 `prediction_horizon_seconds` 只改 YAML，不重编译**（验收标准）。

### 2.7 串口（方案 §9.9）

默认关闭。项目没有正式的 PC→STM32 协议定义，我参照
`reference/2024_Dart_Algorithm` 的 vision_protocol（0xA5+CRC 风格）
实现了默认 20 字节帧（见 `Vision/SerialOutput.hpp` 头注释）：
`0xA5 | state(0/1/2) | float future_x | float future_y | float current_x | float current_y | CRC16-Modbus`。
**⚠️ 需要与下位机确认后再启用**；帧格式改一个 `buildFrame()` 函数即可。

---

## 3. 验证结果（全部真机，非纸面）

| 测试 | 命令 | 结果 |
|---|---|---|
| 单图推理 | `vision_single_test best.onnx <jpg> 0.35` | box/score 与 Python 离线验证一致（0.939 vs 0.943） |
| 真机 vision 开 | `run.sh --recording --config build/camera_config_vision_test.yaml` | 模型加载、叠框录像、STATS 每 2s 输出，p50≈167ms 稳定 |
| 真机 vision 关 | 同上，`enabled: 0` | 664 帧 @82fps，纯取流行为不变 ✅ |
| 30 分钟稳定性 | 后台 `timeout 1800 run.sh --config ...` | **未完成**：运行 ~45s 后 usbipd 链路死亡（50 次连续取图超时→中止；进程退出阶段挂起，kill -9 恢复，见 §4.7）。链路正常期间 cap≈40fps inf≈6.4fps p50≈166ms 稳定。完整 30 分钟需相机重新插拔后重跑，日志 `build/stability_test.log` |

典型 STATS 行：

```text
[STATS] cap=40.5 inf=6.4 disp=6.7 | e2e p50=166.9ms p95=235.6ms | inf_ms=95.3 drop=725 lost=0
```

- **drop=725 但 p50 不变**：证明"推理慢时采集延迟不无限增长"（验收标准）。
- `disp` ≈ `inf`：显示 FPS 跟随推理节拍，正常。
- 相机：MV-CS016-10UC，ResultingFrameRate=164.6（采集 40fps 是 usbipd + CPU 推理抢核的结果，非相机瓶颈）。

---

## 4. 已知限制 / 留给下一位的事

1. **跟踪与预测未在真实目标上验证**：测试时视野内没有靶标，`lost=0` 是
   "没有目标"而不是"跟踪稳定"。需要放一个目标（装甲板+灯条+绿圆灯），
   开 `--window` 看：红框=通过复筛的检测，黄十字=当前位置，青圈=预测位置。
   再改 `prediction_horizon_seconds: 10.0` 验证不重编译生效。
2. **串口协议待确认**（见 2.7），确认后替换 `buildFrame()` 即可。
3. **推理 ~6fps 是 CPU 极限**：下一步提速按方案 §2 顺序：
   CUDA/TensorRT 后端（新增 `CudaOnnxBackend` 实现 `IInferenceBackend`
   即可，相机和跟踪代码不用动），或把模型重新导出 320×320。
4. **LicheeRV 端**：按方案 §2，从 `best.pt` 重导 raw head 再转 `.cvimodel`，
   不要拿当前端到端 ONNX 直接喂 NPU（`DEPLOY_HANDOFF.md` 有失败记录）。
5. `vision_single_test` 会把 `*_vision_test.jpg` 写到图片所在目录
   （测试时我已清理，属临时产物，不入库）。
6. 编译警告：CRLF 警告（Windows 编辑的文件）可忽略；WSL 下偶发
   "Clock skew"（/mnt 时间戳精度），无实害。
7. **usbipd 链路抖动事故记录**（首次稳定性测试时真实发生）：一次测试中
   取图连续 50 次超时触发中止逻辑，但 `show` 进程在退出阶段挂起——
   怀疑 `MV_CC_StopGrabbing/CloseDevice` 在 usbipd 虚拟链路死亡时阻塞。
   处置：`kill -9` 后重新 attach 即恢复。两个对策建议：
   a) 长跑用守护脚本监测 show 进程与 STATS 日志间隔，异常时杀掉重启；
   b) 后续可在 run() 里给清理阶段加看门狗线程（超时 `_exit`），
      属方案外的可靠性增强，未实现以免引入复杂度（方案 §13）。

## 5. 回退方式（方案 §12 已遵守）

- 本次改动 = 新增 `Vision/`（9 个文件）+ `Camera/` 4 个文件的小diff +
  `CMakeLists.txt`/`.gitignore` 各 2 行 + `camera_config.yaml` 追加 vision 块；
- `git checkout -- Camera CMakeLists.txt .gitignore` + 删除 `Vision/` 即完全回退；
- 未覆盖任何模型/录像文件，未执行 git commit/push。

## 6. 复核修正（2026-11-08）

- 修复 `TargetTracker` 的帧间隔更新时间：之前 `lastUpdate_` 只在初始化时写入，
  会使后续 `dt` 累积，导致速度估计和 8 秒预测失真；现在每次更新后刷新时间戳。
- 修复目标匹配初始分数：之前负距离指标从 `-1` 开始，距离超过 1 像素时可能错误地判定为无匹配；
  现在使用负无穷初始化，保证候选框可以正常参与最近目标选择。
- 30 分钟 usbipd 稳定性测试仍未完成；以上修正只针对跟踪正确性，不代表 USB 链路问题已经解决。
