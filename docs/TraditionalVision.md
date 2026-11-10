# 镖架纯传统视觉版本

## 备份与切换

修改算法之前，模型版已提交为 `a1282c4` 并成功推送到
`rm_inventor_rocketGun/dev`，没有操作主分支。
旧模型实施报告和标定精调文档中的 YOLO/ROI 流程属于历史版本；当前以本文为准。

当前源码删除推理接口、ONNX Runtime 后端、模型预处理解码及模型单图测试。
本地 `third_party/`、训练权重、录像、内参、采集图片均保留，没有清理或覆盖。
不再需要下载模型或 RKNN 转换即可运行传统检测；TL3576 仍需 ARM64 MVS SDK 和板端验证。

## 实际流程

MVS 原分辨率图像 → HSV 绿色亮区分割 → 外轮廓面积/圆度筛选
→ 椭圆轴比约束 → 灯心拟合 → 卡尔曼当前灯心与5秒外推
→ 原图水平像素偏差 → 独立串口线程。

参考前面已阅读的 [Misaka21/dart 传统检测](https://github.com/Misaka21/dart/blob/main/detector/detector.cpp)
中“轮廓、面积、圆度、优先大目标”的思路，独立实现；没有复制未知的“海威”项目源码。
与直接绿色通道阈值不同，这里同时要求色相/饱和度，防止白色圆形反光轻易通过。
检测所有合格绿灯，首次跟踪选择面积最大的候选，后续按位置关联。
框上 `area` 是轮廓面积，不是神经网络置信度。

## 参数与运行

`Camera/camera_config.yaml` 中 `vision.backend: traditional`、`vision.enabled: 1`。
`green_light` 中默认 H=35..85、S>=60、V>=160，面积8..100000像素，
圆度>=0.65、短长轴比>=0.65。均为可调初值，并非经过赛场标定的最佳门限。
排除贴图像边缘的截断轮廓，可能暂时漏掉刚进入边缘的目标，避免错误灯心。
过曝白芯仍有绿色外环时可用外轮廓拟合；完全过曝成白色时会漏检，应先调整曝光。

运行 `./run.sh --window`，终端输出 `backend=traditional`、`detect` FPS 和 `detect_ms`。
显示准心、轮廓和灯心、预测点以及左上角发送参数。8字节串口格式不变，仍默认关闭。
标定文件仍为配置相对路径 `../camera_intrinsics.yaml`，不得把旧模型版配置直接覆盖回来。

## 验证与限制

`cmake -S . -B build && cmake --build build -j2`

`ctest --test-dir build/Vision --output-on-failure`

测试覆盖亮绿圆灯、白光排除、绿色灯条排除、暗绿排除、边缘截断、多个圆灯，
以及整条传统管线的跟踪/内参视线角/5秒参数/像素偏差/丢目标停发和虚拟串口。
`build/Vision/vision_image_test input.jpg build/detected.png` 可检查真实单图。
不以合成测试代替真实靶灯精度、30分钟长稳或板端性能验收。
多个相似圆灯仍可能误关联；应使用实拍数据验证误检/漏检及匹配门限需求。

本次执行结果：
- 默认视觉构建成功，CTest 2/2通过（含完整传统管线和PTY串口）。
- 纯取流构建最初暴露原有无条件包含Vision头文件的问题，已加编译开关保护，
  `build-capture-only/` 下 `RM_ENABLE_VISION=OFF` 构建复测成功。
- 历史数据 `frame_fly0400013.jpg` 检出1个绿色圆灯，灯心约(1068.1,327.6)，
  单次算法耗时8.96ms；这是旧GC4653图片的离线测试，不是海康实时FPS或TL3576性能。
- 叠加图 `build/traditional_target_check.png` 已检查，圆圈落在绿色发光靶灯上。
  另一张暗场 `frame_fly0100000.jpg` 检出0个，产物 `build/traditional_frame_check.png`。
- WSL启动仍打印既存Windows PATH转换警告；未改系统设置。
- 未连接海康运行长测、未连接真实电控、未上TL3576；不宣称这些验收通过。

默认 `build/` 中保留此前产物，不做清理；旧 `vision_single_test` 若仍存在，
它不是当前源码生成的工具，请使用 `vision_image_test`。

回退前先保存当前工作区；本次传统算法改动可逐项反向撤销当前差异，恢复到模型备份提交对应源码。
恢复模型版时须同步恢复配置和 ONNX Runtime 构建依赖；不能只换一个后端名称。
