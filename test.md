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

## 8. 第二组测试：异步版感知链路下的旧 / 新 YOLO 对比

测试时间：2026-03-19  
测试目的：在**刚完成的异步感知处理版本**下，对比旧 YOLO ONNX 与新导出的静态 `480x640` YOLO ONNX 的实际表现。

### 8.1 测试约束

本轮仅切换 YOLO 模型，其他条件保持一致：

- 感知代码：异步版（单 worker + latest-frame 覆盖 + 主线程 tracking/publish）
- ReID：`models/reid_resnet50_2048.onnx`
- 命名空间：`/robot1`
- 相机输入：`640x480 @ 30fps`
- 运行位置：小车主控 `ros2` 容器

### 8.2 第一轮：旧模型 + 异步处理

模型配置：

- YOLO：`models/yolo26n.onnx`
- YOLO 输入：`640x640`
- ReID：`models/reid_resnet50_2048.onnx`

稳定日志均值如下：

| 阶段 | 平均耗时（ms） |
|---|---:|
| camera_info | 0.01 |
| cv_bridge | 0.70 |
| yolo | 471.23 |
| depth | 0.03 |
| reid | 511.17 |
| recover | 0.00 |
| tracking | 0.48 |
| lock | 0.00 |
| message | 42.42 |
| publish | 0.22 |
| total | 1031.50 |

`/robot1/person_pose` 实测发布频率：

- 约 `0.944 Hz`

运行观察：

- 有效检测存在，尾部日志稳定看到 `detections=2`
- `persons=2`，说明链路在异步版下可持续工作

### 8.3 第二轮：新静态模型 + 异步处理

模型配置：

- YOLO：`models/yolo26n_static_480x640_simplify_e2e.onnx`
- YOLO 输入：`640x480`
- ReID：`models/reid_resnet50_2048.onnx`

稳定日志均值如下：

| 阶段 | 平均耗时（ms） |
|---|---:|
| camera_info | 0.02 |
| cv_bridge | 0.51 |
| yolo | 339.32 |
| depth | 0.02 |
| reid | 527.70 |
| recover | 0.00 |
| tracking | 0.50 |
| lock | 0.00 |
| message | 41.83 |
| publish | 0.31 |
| total | 915.59 |

`/robot1/person_pose` 实测发布频率：

- 约 `0.982 Hz`

运行观察：

- 有效检测存在，尾部日志看到 `detections=3`
- `persons=2`
- 新模型在当前场景下比旧模型更容易检出更多框，因此 ReID 负担略有上升

### 8.4 对比汇总

| 指标 | 旧 YOLO + 异步 | 新 YOLO + 异步 | 变化 |
|---|---:|---:|---:|
| total（ms） | 1031.50 | 915.59 | -115.91 ms（-11.2%） |
| yolo（ms） | 471.23 | 339.32 | -131.91 ms（-28.0%） |
| reid（ms） | 511.17 | 527.70 | +16.53 ms（+3.2%） |
| person_pose 频率（Hz） | 0.944 | 0.982 | +0.038 Hz（+4.0%） |

### 8.5 结论

1. **新静态 `480x640` YOLO 模型已经明显更快**
   - YOLO 阶段从 `471.23 ms` 降到 `339.32 ms`
   - 下降约 `28.0%`

2. **整体总耗时也有改善，但没有和 YOLO 同比例下降**
   - 原因是当前 ReID 仍然很重，并且新模型在该场景下产生了更多检测框，导致 ReID 总耗时略有增加

3. **异步化已经起到作用，但当前系统新的主瓶颈已转向“YOLO + 重型 ReID 组合”**
   - 如果后续将 ReID 再切回轻量 OSNet，同时继续沿用新 YOLO，整体收益预计会更明显

4. **这一轮结果说明新 YOLO 导出方向是正确的**
   - 静态 `480x640`
   - `simplify=True`
   - `end2end=True`
   - 已经在小车主控实测中体现出收益

## 9. 第三组测试：新 YOLO + 异步处理 + 轻量 OSNet ReID

测试时间：2026-03-19  
测试目的：在已经验证有效的新静态 YOLO 与异步感知链路基础上，再切换到轻量 OSNet ReID，评估组合收益。

### 9.1 模型配置

- YOLO：`models/yolo26n_static_480x640_simplify_e2e.onnx`
- YOLO 输入：`640x480`
- ReID：`models/osnet_x0_5_512.onnx`
- ReID 输入：`128x256`

说明：
- 当前 C++ 主链路仍以 `2048` 维特征接口为主，因此 OSNet 的 `512` 维输出会被补零到 `2048` 维。
- 日志中的 `ReID output dim 512 padded to 2048 for benchmarking compatibility` 属于预期行为。

### 9.2 稳定日志均值

| 阶段 | 平均耗时（ms） |
|---|---:|
| camera_info | 0.01 |
| cv_bridge | 0.63 |
| yolo | 355.76 |
| depth | 0.02 |
| reid | 85.76 |
| recover | 0.00 |
| tracking | 0.61 |
| lock | 0.00 |
| message | 44.77 |
| publish | 0.21 |
| total | 492.47 |

`/robot1/person_pose` 实测发布频率：

- 约 `2.256 Hz`

### 9.3 运行观察

- 有效检测存在，尾部日志常见 `detections=2`，偶尔 `detections=3`
- 已持续看到 `persons=2` 或 `persons=3`
- 感知链路运行稳定，没有出现崩溃或卡死

### 9.4 与第二轮（新 YOLO + 重型 ReID）对比

| 指标 | 新 YOLO + 重型 ReID | 新 YOLO + 轻量 OSNet | 变化 |
|---|---:|---:|---:|
| total（ms） | 915.59 | 492.47 | -423.12 ms（-46.2%） |
| yolo（ms） | 339.32 | 355.76 | +16.44 ms（+4.8%） |
| reid（ms） | 527.70 | 85.76 | -441.94 ms（-83.7%） |
| person_pose 频率（Hz） | 0.982 | 2.256 | +1.274 Hz（+129.7%） |

### 9.5 与第一轮（旧 YOLO + 重型 ReID + 异步）对比

| 指标 | 旧 YOLO + 重型 ReID | 新 YOLO + 轻量 OSNet | 变化 |
|---|---:|---:|---:|
| total（ms） | 1031.50 | 492.47 | -539.03 ms（-52.3%） |
| yolo（ms） | 471.23 | 355.76 | -115.47 ms（-24.5%） |
| reid（ms） | 511.17 | 85.76 | -425.41 ms（-83.2%） |
| person_pose 频率（Hz） | 0.944 | 2.256 | +1.312 Hz（+139.0%） |

### 9.6 结论

1. **当前三轮测试中，综合表现最好的是：新静态 YOLO + 异步处理 + 轻量 OSNet ReID**
2. 新 YOLO 负责把检测阶段压下来，轻量 OSNet 负责把 ReID 阶段大幅压缩，二者叠加效果非常明显
3. 在这套组合下：
   - 总耗时已经从约 `1031.50 ms` 降到 `492.47 ms`
   - `/robot1/person_pose` 频率从约 `0.944 Hz` 提升到约 `2.256 Hz`
4. 这说明我们前面做的两件事方向都正确：
   - 感知异步化
   - YOLO / ReID 模型轻量化
5. 后续如果还要继续优化，优先级建议为：
   - 检测频率自适应
   - ONNX Runtime 线程调优
   - 更细粒度 profiling
   - 若仍不满足，再评估 INT8 或其他推理后端
## 10. 第四组测试：ONNX Runtime 线程调优（S1-S5 第一轮）

测试时间：2026-03-19  
测试目的：在当前推荐组合基础上，初步比较 YOLO ONNX Runtime 不同线程配置的效果，找出第一版较优参数。

### 10.1 测试前提

固定条件：

- 感知实现：异步版 perception
- YOLO：`models/yolo26n_static_480x640_simplify_e2e.onnx`
- ReID：`models/osnet_x0_5_512.onnx`
- ReID ORT：`intra=1 / inter=1 / sequential`
- 相机输入：`640x480 @ 30fps`

本轮仅测试 YOLO ORT：
- `intra_op_num_threads`
- `execution_mode`

### 10.2 重要限制

本轮测试时，**画面中没有稳定人体目标**，因此：

- `detections=0`
- `tracks=0`
- `reid=0.00 ms`

所以这组结果主要反映：

- YOLO 纯推理开销
- 空检测场景下的 pipeline 表现

它**可以用于初步筛选 YOLO ORT 参数**，但还不能作为最终“完整业务场景最优解”的唯一依据。

### 10.3 测试结果

| 组别 | YOLO 配置 | total（ms） | yolo（ms） | person_pose（Hz） |
|---|---|---:|---:|---:|
| S1 | intra=1, sequential | 348.91 | 343.05 | 2.696 |
| S2 | intra=2, sequential | 244.17 | 238.24 | 3.582 |
| S3 | intra=3, sequential | 226.55 | 220.70 | 3.131 |
| S4 | intra=4, sequential | 277.14 | 271.16 | 2.994 |
| S5 | intra=3, parallel | 228.74 | 223.41 | 3.278 |

### 10.4 初步结论

1. `intra=1` 明显偏慢，可排除
2. `intra=4` 相比 `2` / `3` 反而退化，说明线程不是越多越好
3. 当前空检测场景下：
   - **`S3`（YOLO intra=3, sequential）** 的 `total` 和 `yolo` 最低
   - `S5`（YOLO intra=3, parallel）`接近，但略逊于 S3`
4. 因此，**第一版推荐的 YOLO ORT 参数仍保持为：**

```yaml
yolo:
  ort:
    intra_op_num_threads: 3
    inter_op_num_threads: 1
    execution_mode: sequential
```

5. ReID 由于本轮没有目标进入有效检测，尚未被真正压测，因此：
   - `reid intra=1 / sequential` 仍暂时保留
   - 后续建议在**有稳定人体目标**的场景下，再补做 ReID 线程调优

### 10.5 下一步建议

如果后续继续做 ORT 调优，建议顺序为：

1. 维持 `YOLO intra=3 / sequential`
2. 在**有人体进入画面**的条件下，补做 ReID 线程测试
3. 若 ReID 线程测试完成后收益不明显，再决定是否继续尝试 `inter_op_num_threads > 1`
