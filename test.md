# ROS2 Smart Follower 小车端 Profiling 测试报告

测试时间：2026-03-18  
测试目的：获取感知链路各阶段耗时，定位当前推理瓶颈，并对比两种 ReID 模型的实际效果。

## 1. 测试环境

- 主控：树莓派小车主控（`192.168.0.100`）
- 运行位置：`ros2` Docker 容器
- ROS 发行版：ROS2 Humble
- 相机：Astra 深度相机
- 图像输入：`640x480 @ 30fps`
- D2C：已开启，深度对齐到彩色
- 工作区：
  - 容器内挂载：`/home/wheeltec/ros2_shared_dir/ros2_smart_follower`
- 感知节点：
  - 可执行文件：`smart_follower_perception/perception_node`
  - 命名空间：`/robot1`

## 2. 测试模型

### 2.1 基线模型

- YOLO：`models/yolo26n.onnx`
- ReID：`models/reid_resnet50_2048.onnx`

### 2.2 轻量 ReID 对照组

- YOLO：`models/yolo26n.onnx`
- ReID：`models/osnet_x0_5_512.onnx`

说明：
- `osnet_x0_5_512.onnx` 由 `E:\osnet_x0_5_traced.pt` 导出得到。
- 当前 C++ 感知链路主线仍以 `2048` 维特征为接口，因此对 `OSNet` 的 `512` 维输出做了零填充，以便在不改消息/缓存结构的前提下完成同链路基准测试。

## 3. 测试前确认

已确认：

- 相机节点正常启动
- 话题存在：
  - `/camera/color/image_raw`
  - `/camera/color/camera_info`
  - `/camera/depth/image_raw`
- 感知节点日志显示：
  - `YOLO ready=1`
  - `ReID ready=1`
- 实际测试中存在有效人体检测，`/robot1/person_pose` 可持续发布

测试中同时观察到一项环境问题：

- `base_footprint <- camera_color_optical_frame` 的 TF 缺失
- 因此 3D 坐标变换/消息组装阶段包含 TF lookup 失败开销
- 这不会改变“谁是主要瓶颈”的结论，但会影响 `message` 阶段绝对值

## 4. Profiling 结果

### 4.1 第一轮：ResNet50-2048 ReID

稳定日志均值如下：

| 阶段 | 平均耗时（ms） |
|---|---:|
| camera_info | 0.00 |
| cv_bridge | 0.73 |
| yolo | 464.31 |
| depth | 0.01 |
| reid | 251.45 |
| recover | 0.00 |
| tracking | 0.23 |
| lock | 0.00 |
| message | 20.52 |
| publish | 0.07 |
| total | 737.75 |

`/robot1/person_pose` 实测发布频率：

- 约 `1.291 Hz`

### 4.2 第二轮：OSNet x0.5-512 ReID

稳定日志均值如下：

| 阶段 | 平均耗时（ms） |
|---|---:|
| camera_info | 0.01 |
| cv_bridge | 0.62 |
| yolo | 465.24 |
| depth | 0.01 |
| reid | 39.67 |
| recover | 0.00 |
| tracking | 0.20 |
| lock | 0.00 |
| message | 20.56 |
| publish | 0.07 |
| total | 526.53 |

`/robot1/person_pose` 实测发布频率：

- 约 `1.793 Hz`

说明：

- 日志中出现 `ReID output dim 512 padded to 2048 for benchmarking compatibility`，属于预期行为。

## 5. 对比汇总

| 指标 | ResNet50-2048 | OSNet x0.5-512 | 变化 |
|---|---:|---:|---:|
| total（ms） | 737.75 | 526.53 | -211.22 ms（-28.6%） |
| yolo（ms） | 464.31 | 465.24 | +0.93 ms（基本不变） |
| reid（ms） | 251.45 | 39.67 | -211.78 ms（-84.2%） |
| person_pose 频率（Hz） | 1.291 | 1.793 | +0.502 Hz（+38.9%） |

## 6. 结论

### 6.1 当前最大瓶颈已经非常明确

在两轮测试里，**YOLO 始终是最大的耗时项**：

- 基线模型下约 `464 ms`
- 更换轻量 ReID 后仍约 `465 ms`

这说明：

- 换轻量 ReID 的收益是真实且明显的
- 但总链路性能上限已经主要受 YOLO 推理限制

### 6.2 轻量 ReID 值得保留为后续优化方向

OSNet x0.5 的收益很明显：

- ReID 耗时下降约 `84.2%`
- 整体总耗时下降约 `28.6%`
- `person_pose` 发布频率提升约 `38.9%`

如果我们当前目标是“先跑起来、先把功能链路做顺”，那么轻量 ReID 是值得继续保留和验证的。

### 6.3 下一步优化优先级建议

建议优先顺序：

1. **YOLO 优化**
   - 更轻模型
   - 更低检测频率
   - 输入分辨率进一步评估
   - ONNX Runtime 配置调优
2. **感知流水线异步化**
   - 检测线程与跟踪/发布解耦
   - 避免单帧总时延直接卡住全链路输出
3. **修复 TF 链**
   - 补齐 `base_footprint <- camera_color_optical_frame`
   - 让 3D 坐标与 `message` 阶段恢复正常
4. **在真实运行工况下继续测**
   - 当前已能反映主瓶颈
   - 后续建议在完整底盘/控制链路联动时再做一次系统级 profiling

## 7. 本轮产出

本轮已完成：

- 小车主控容器内部署项目
- 接入 ARM64 ONNX Runtime SDK 并完成真实推理编译
- 增加感知链路分阶段 profiling
- 完成两轮实机 profiling
- 形成本报告供后续优化参考
