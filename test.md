> 归档说明（2026-03-26）
>
> 本文件主要保存阶段性实验记录、性能对比和历史排障过程，**不再作为当前主线说明文档**。
> 当前以以下文件为准：
> - `README.md`
> - `try.md`
> - `当前推荐运行组合.md`
>
> 当前固定技术路线：Astra color+depth + `/camera/get_camera_info` + depth compare 主链路。
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


## 11. 第五组测试：小车端单路最终推荐组合实测（含资源占用基线）

测试时间：2026-03-22  
测试目的：在**小车主控 ros2 容器**内，用当前最终推荐组合做一轮真实相机实测，记录**时延 + CPU/内存占用**，作为后续评估“是否值得做 2 路 YOLO+ReID” 的基线。

### 11.1 测试前提

本轮在**小车主控 `192.168.0.100`** 上完成，方式如下：

- 先清空小车端旧工作区，再同步当前本地项目
- 容器：`ros2`（`ros2:wheeltec_V1.1`）
- 感知/控制工作区：`/home/wheeltec/ros2_shared_dir/ros2_smart_follower`
- 相机驱动来源：额外 source 了宿主机工作区
  - `/home/wheeltec/wheeltec_ros2/install/setup.bash`
- ONNX Runtime 路径：
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3`
- GPIO 后端：`libgpiod`

本轮实际启动方式等价于：

- `smart_follower.launch.py`
- `bringup_robot:=false`
- `bringup_camera:=true`

说明：
- 这样可以在容器内直接拉起 Astra 相机节点与当前 smart follower 主链路
- 不启动底盘本体 bringup，避免串口/底盘状态干扰本轮纯感知性能测试

### 11.2 本轮固定配置

- 感知实现：异步版 perception（单 worker）
- YOLO：`models/yolo26n_static_480x640_simplify_e2e.onnx`
- ReID：`models/osnet_x0_5_512.onnx`
- YOLO ORT：`intra=3 / inter=1 / sequential`
- ReID ORT：`intra=1 / inter=1 / sequential`
- `async_worker_count=1`
- `process_every_n_frames=3`
- 相机模式：Astra，`640x480`
- 画面状态：**镜头中存在 1 个稳定人体目标**

日志确认：

- `YOLO ready=1`
- `ReID ready=1`
- 稳定持续看到：
  - `detections=1`
  - `persons=1`
  - `tracks=1`

### 11.3 链路时延结果

稳定日志尾部多次收敛到如下水平：

| 阶段 | 平均耗时（ms） |
|---|---:|
| camera_info | 0.01 |
| cv_bridge | 0.64 |
| yolo | 223.85 |
| └─ yolo_preprocess | 4.18 |
| └─ yolo_run | 219.59 |
| └─ yolo_postprocess | 0.04 |
| depth | 0.02 |
| reid | 40.11 |
| └─ reid_preprocess | 1.59 |
| └─ reid_run | 38.43 |
| recover | 0.00 |
| tracking | 0.41 |
| lock | 0.00 |
| tf_lookup | 22.10 |
| tf_transform | 0.00 |
| msg_fill | 0.04 |
| message | 22.16 |
| publish | 0.56 |
| total | 293.42 |

对应日志样例：

- `profile avg_ms total=293.42 ... yolo=223.85 ... reid=40.11 ... message=22.16 ... publish=0.56`

### 11.4 输出频率与输入频率

实测 `ros2 topic hz` 结果：

- `/camera/color/image_raw`：约 `11.778 Hz`
- `/robot1/person_pose`：约 `2.243 Hz`

说明：
- 本轮观测到的**实际进入 ROS 图的彩色输入频率并不是预期中的 30Hz**，而是约 `11.8Hz`
- 在 `process_every_n_frames=3` 的前提下，感知可调度上限本身就会被进一步压低
- 由于单 worker 推理耗时仍接近 `293ms`，所以最终 `person_pose` 输出稳定在约 `2.24Hz`

### 11.5 资源占用结果（用于双路可行性评估）

本轮对运行中的容器和关键进程做了连续采样。

> 注：首个样本存在一次采样瞬态（`top` 行排序切换），以下均值按**剔除首个瞬态样本后的 11 个稳定样本**统计。

#### 11.5.1 容器级 CPU

| 指标 | 数值 |
|---|---:|
| ros2 容器 CPU 平均占用 | `279.17%` |
| ros2 容器 CPU 峰值 | `350.39%` |

说明：
- 树莓派 5 为 4 大核，这里的 `279%` 可近似理解为**平均占用了 2.79 个 CPU 核**
- 峰值超过 `350%`，说明短时会逼近 **3.5 个核**

#### 11.5.2 perception_node 资源占用

| 指标 | 数值 |
|---|---:|
| perception_node CPU 平均占用 | `196.00%` |
| perception_node CPU 峰值 | `253.00%` |
| perception_node RSS 平均 | `182.09 MiB` |
| perception_node RSS 峰值 | `182.36 MiB` |
| perception_node VIRT 稳定值 | 约 `1352.41 MiB` |

说明：
- 单个 `perception_node` 在真实运行中，已经长期占用**接近 2 个 CPU 核**
- 高峰时会冲到 **2.5 核左右**
- 常驻物理内存（RSS）约 **182 MiB**，内存压力不大，**主要瓶颈是 CPU**

#### 11.5.3 相机进程资源占用（Astra）

| 指标 | 数值 |
|---|---:|
| astra_camera_node CPU 平均占用 | `71.12%` |
| astra_camera_node CPU 峰值 | `77.00%` |
| astra_camera_node RSS 稳定值 | `107.44 MiB` |
| astra_camera_node VIRT 稳定值 | 约 `1314.25 MiB` |

说明：
- 真实相机驱动本身就稳定占用约 **0.7 个核**
- 所以“相机 + 单路感知”组合，已经构成当前小车端 CPU 负载主体

### 11.6 运行中观察到的关键现象

1. **单路链路已经能稳定跟到 1 个目标**
   - `detections=1`
   - `persons=1`
   - `tracks=1`

2. **YOLO 仍然是主瓶颈**
   - `223.85 ms` 明显高于 ReID 的 `40.11 ms`

3. **TF 仍未补齐**
   - 持续出现：
     - `TF lookup failed (base_footprint <- camera_color_optical_frame)`
   - 这会稳定带来约 `22 ms` 的 `tf_lookup/message` 开销

4. **单 worker 已经开始出现积压/丢帧**
   - 日志中出现：
     - `pending queue full, dropped oldest frame ...`
   - 说明当前单路已经不能完全吃下相机输入

5. **OSNet 512 维补零行为仍在**
   - `ReID output dim 512 padded to 2048 for benchmarking compatibility`
   - 属于预期行为，不影响本轮结论

### 11.7 对“双路 YOLO+ReID”可行性的直接判断

这轮数据对后续双路方案非常关键。

当前单路基线大致是：

- 相机驱动：约 `0.7` 核
- perception：约 `2.0` 核，峰值到 `2.5` 核
- 整个 ros2 容器：平均约 `2.8` 核，峰值约 `3.5` 核

因此可以得到一个很直接的判断：

1. **如果简单复制一整套第二路 perception**，大概率会把总负载推到 `4.5~5.0` 核量级
2. 对树莓派 5 的 4 核 CPU 来说，这已经非常危险，容易导致：
   - 系统调度抖动明显变大
   - 两路互相争抢，单路时延反而上升
   - 控制链路被拖慢
3. 也就是说：
   - **“直接双开完整 YOLO+ReID” 目前不算合理默认方案**
   - 真要试双路，应该把它当成**受控实验**，而不是默认部署配置

### 11.8 本轮结论

本轮小车端真实实测可以得出：

1. 当前最终推荐组合已经能在小车上稳定运行，单目标场景下可持续输出
2. 单路最终推荐组合的稳定时延约为：
   - `total ≈ 293 ms`
   - `yolo ≈ 224 ms`
   - `reid ≈ 40 ms`
3. 当前真正的硬瓶颈仍然是 **YOLO CPU 推理**
4. 从资源占用角度看：
   - **单路已经吃掉接近 2 个核的纯感知算力**
   - 再叠一整路完整 perception，风险很高
5. 因此，后续若要探索“双路/多路推理”，建议优先顺序应当是：
   - 先做**统一缓冲区 + 受控双 worker 实验**
   - 同时严密监控 CPU、队列积压、输出频率、控制稳定性
   - 不宜直接把双路方案当作默认量产配置

## 12. 第六组测试：320x256 YOLO + 轻量 ReID 小车端实机 Profiling（当前工作区）

测试时间：2026-03-24（本地时间）  
说明：小车/容器内系统时间仍有漂移，因此运行日志时间显示为 `2026-01-08`，但本轮测试实际执行于 **2026-03-24**。

测试目的：在**当前最新工作区**与**新降分辨率 YOLO 模型**下，重新获取一轮小车端真实相机输入、推理分阶段耗时与资源占用数据，并直接写回本报告。

### 12.1 本轮测试环境

- 主控：树莓派小车主控（`192.168.0.100`）
- 运行位置：小车端 `ros2:wheeltec_V1.1` Docker 容器
- 工作区：`/home/wheeltec/ros2_smart_follower`
- 相机驱动：`wheeltec_ros2` 内置 `astra_camera`
- Astra 相机启动日志确认：
  - `set color video mode Resolution :640x480@30Hz`
  - `set depth video mode Resolution :640x480@30Hz`
- ONNX Runtime：
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3`
- 实机画面状态：**镜头中存在 1 个稳定人体目标**

### 12.2 本轮固定配置

- Launch：`astra_camera astra.launch.xml` + `smart_follower_only.launch.py`
- YOLO：`models/yolo26n_static_256x320_simplify_e2e.onnx`
- ReID：`models/osnet_x0_5_512.onnx`
- YOLO ORT：`intra=3 / inter=1 / sequential`
- ReID ORT：`intra=1 / inter=1 / sequential`
- `process_every_n_frames=3`
- 当前主链路：纯 RGB 单目地面投影，不再走深度主链路

日志确认：

- `YOLO ready=1`
- `ReID ready=1`
- 稳定持续看到：
  - `detections=1`
  - `persons=1`
  - `tracks=1`
- 同时持续看到：
  - `ReID output dim 512 padded to 2048 for benchmarking compatibility`
  - 属于当前 OSNet 兼容策略的预期行为

### 12.3 链路时延结果

取稳定运行后的最新累计 profiling 结果（日志尾部 `profile avg_ms ...`）：

| 阶段 | 平均耗时（ms） |
|---|---:|
| cv_bridge | `0.45` |
| yolo | `45.38` |
| reid | `36.53` |
| recover | `0.00` |
| tracking | `0.22` |
| lock | `0.00` |
| projection | `0.00` |
| msg_fill | `0.02` |
| message | `0.03` |
| publish | `0.19` |
| total | `87.84` |

对应稳定日志样例：

- `profile avg_ms total=87.84 cv_bridge=0.45 yolo=45.38 reid=36.53 ... publish=0.19`

#### 12.3.1 帧间抖动（取最近 8 条 `last_ms`）

| 指标 | 观测范围 |
|---|---:|
| last total | `81.99 ~ 147.37 ms` |
| last yolo | `41.57 ~ 89.11 ms` |
| last reid | `35.63 ~ 48.20 ms` |

说明：

- 平均值已经比较稳定，主抖动仍主要来自 **YOLO 单帧推理波动**
- 但即使按较高的 `last_ms` 看，本轮也明显快于上一版 `480x640` YOLO 组合

### 12.4 输入 / 输出频率

使用 `ros2 topic hz` 在实机运行中直接采样：

| 话题 | 实测频率 |
|---|---:|
| `/camera/color/image_raw` | `14.603 Hz` |
| `/robot1/person_pose` | `6.515 Hz` |

说明：

- Astra 驱动配置仍是 `640x480@30Hz`，但**实际进入 ROS 图链路的彩色输入只有约 `14.6Hz`**
- 在 `process_every_n_frames=3` 的配置下，最终 `person_pose` 仍能稳定到 **约 `6.5Hz`**
- 本轮相比上一版实测，输出频率已经有明显改善，但**彩色输入频率不足**仍然是独立问题

### 12.5 资源占用结果（20 秒 / 1Hz 采样）

采样方式：在容器内对 `/proc` 连续采样 `20` 次，统计系统忙碌度与关键进程 CPU / RSS。

#### 12.5.1 系统级 CPU

| 指标 | 数值 |
|---|---:|
| 4 核系统 busy 平均占用 | `43.63%` |
| 4 核系统 busy 峰值 | `54.18%` |
| 4 核系统 busy 最低 | `34.52%` |

#### 12.5.2 perception_node

| 指标 | 数值 |
|---|---:|
| perception_node CPU 平均占用 | `114.36%` |
| perception_node CPU 峰值 | `153.92%` |
| perception_node CPU 最低 | `83.25%` |
| perception_node RSS | `125.11 MiB` |
| perception_node 线程数 | `16` |

说明：

- 当前单路 perception 实际长期占用约 **1.14 个 CPU 核**
- 峰值约 **1.54 个核**
- 内存占用稳定在 **125 MiB** 左右，当前主瓶颈仍然是 **CPU**，不是内存

#### 12.5.3 astra_camera_node

| 指标 | 数值 |
|---|---:|
| astra_camera_node CPU 平均占用 | `53.45%` |
| astra_camera_node CPU 峰值 | `59.85%` |
| astra_camera_node CPU 最低 | `49.75%` |
| astra_camera_node RSS | `111.47 MiB` |
| astra_camera_node 线程数 | `22` |

说明：

- Astra 相机驱动本身稳定占用约 **0.53 个 CPU 核**
- 因此当前“相机 + 单路感知”组合合计约吃掉 **1.67 个核** 左右

### 12.6 与第 11 组（480x640 YOLO）对比

为便于直接判断本轮收益，和第 11 组“小车端单路最终推荐组合实测”做对比：

| 指标 | 第 11 组（480x640） | 第 12 组（320x256） | 变化 |
|---|---:|---:|---:|
| total | `293.42 ms` | `87.84 ms` | `-205.58 ms`（`-70.1%`） |
| yolo | `223.85 ms` | `45.38 ms` | `-178.47 ms`（`-79.7%`） |
| reid | `40.11 ms` | `36.53 ms` | `-3.58 ms`（`-8.9%`） |
| `/robot1/person_pose` | `2.243 Hz` | `6.515 Hz` | `+4.272 Hz`（约 `+190.5%`） |
| perception CPU | `196.00%` | `114.36%` | `-81.64` 个百分点 |

结论非常明确：

1. **这轮最主要的收益来自 YOLO 分辨率下降**，YOLO 耗时被大幅压缩
2. ReID 变化不大，说明当前主要改进点确实发生在检测端
3. 输出频率已经从 `2.24Hz` 量级提升到 `6.5Hz` 左右，实机可用性明显改善
4. perception CPU 也明显下降，给后续继续优化或做受控并行实验留出了更多空间

### 12.7 本轮结论

本轮小车端真实 profiling 可以得出：

1. 当前 `320x256` 静态 YOLO + 轻量 OSNet ReID 的组合，已经比上一版 `480x640` 组合有**非常明显的实机收益**
2. 当前稳定链路时延约为：
   - `total ≈ 87.84 ms`
   - `yolo ≈ 45.38 ms`
   - `reid ≈ 36.53 ms`
3. 当前主瓶颈仍然是 **YOLO**，但它已经从“绝对压死全链路的大头”下降到了可继续优化的水平
4. 当前单路 perception 的资源占用大致为：
   - 平均约 `1.14` 核
   - 峰值约 `1.54` 核
   - RSS 约 `125 MiB`
5. 从资源角度看，**继续做受控双路 / 双 worker 实验已经比第 11 组时更有现实意义**，但前提依旧是：
   - 先解决彩色输入频率只有 `14.6Hz` 的问题
   - 双路实验时严密监控 CPU、输出频率和控制稳定性
6. 就当前版本而言，这一组数据已经可以作为后续“并行推理是否值得继续推进”的**新基线**


## 13. 彩色输入掉速定位与 YOLO 线程对照（2026-03-24）

### 13.1 目的

本轮围绕两个问题继续定位：

1. 为什么 color-only 后，Astra 明明能发 `30Hz`，但 follower 实际彩色输入仍会掉速
2. YOLO ORT `intra_op_num_threads` 取 `1/2/3` 时，哪一档对整车链路最优

测试环境统一为：

- 小车主控容器：`ros2:wheeltec_V1.1`
- 相机侧临时参数：
  - `color_qos:=sensor_data`
  - `enable_color:=true`
  - `enable_depth:=false`
  - `enable_point_cloud:=false`
  - `depth_registration:=false`
  - `enable_d2c_viewer:=false`
  - `enable_ir:=false`
- 模型组合：
  - `models/yolo26n_static_256x320_simplify_e2e.onnx`
  - `models/osnet_x0_5_512.onnx`
- perception 参数：
  - `process_every_n_frames=3`
  - `detect_every_n_frames=1`

### 13.2 color-only 相机本体能力对照

先只启动 color-only Astra，再用轻量订阅节点验证相机与 DDS 本体能力：

| 场景 | color 频率 | astra_camera CPU | 结论 |
|---|---:|---:|---|
| 仅 Astra + 轻量订阅 | `29.43 Hz`（自写 probe） / `29.59 Hz`（`ros2 topic hz`） | `13.9%` | 相机本体与 DDS 本身可稳定接近 `30Hz` |
| Astra + follower/perception | perception `raw_input color` 约 `15.3 Hz` | `13%` 左右 | 掉速发生在 follower/perception 联动时 |
| Astra + follower/perception + 轻量订阅 | 轻量订阅约 `10.38 Hz`，perception `raw_input color` 约 `12~14 Hz` | `13%` 左右 | perception 跑起来后会把图像接收链路整体挤压变慢 |

结论：

1. **不是 Astra 相机本体发不出 `30Hz`**
2. **也不是 `sensor_data` QoS 之外的单一相机参数问题**
3. 当前彩色图掉速的主因已经收敛为：**perception 运行负载对 ROS2 图像接收 / 调度链路的挤压**

### 13.3 YOLO `intra_op_num_threads` = 3 / 2 / 1 对照

#### 13.3.1 `intra=3`

参考本轮 color-only follower 基线：

| 指标 | 数值 |
|---|---:|
| perception `raw_input color` | 约 `15.3 Hz` |
| `/robot1/person_pose` | 约 `5~6 Hz` |
| profile total | 约 `72.6 ms` |
| profile yolo | 约 `35.4 ms` |
| perception CPU | 约 `68~70%` |

判断：

- 单帧 YOLO 最快，但**线程争抢最明显**
- 图像接收被压制，整条链路综合效果最差

#### 13.3.2 `intra=2`

在节点运行中热更新：

```bash
ros2 param set /robot1/perception_node yolo.ort.intra_op_num_threads 2
```

实测：

| 指标 | 数值 |
|---|---:|
| perception `raw_input color` | 约 `21.5 Hz` |
| `/robot1/person_pose` | 约 `8.4~8.8 Hz` |
| profile total | 约 `75~76 ms` |
| profile yolo | 约 `39~39.5 ms` |
| perception CPU | 约 `74~76%` |

判断：

- YOLO 单帧略慢于 `intra=3`
- 但彩色图实收和发布频率显著改善
- 是一个明显更均衡的配置

#### 13.3.3 `intra=1`

继续热更新：

```bash
ros2 param set /robot1/perception_node yolo.ort.intra_op_num_threads 1
```

实测：

| 指标 | 数值 |
|---|---:|
| `ros2 topic hz /camera/color/image_raw` | 约 `28.8~29.1 Hz` |
| perception `raw_input color` | 约 `25.7 Hz` |
| `/robot1/person_pose` | 约 `9.3~9.5 Hz` |
| profile total | 约 `92~93 ms` |
| profile yolo | 约 `55~57 ms` |
| perception CPU | 约 `81~82%` |

判断：

- YOLO 单帧最慢
- 但**图像接收链路恢复最好**
- 在 `process_every_n_frames=3` 前提下，`/person_pose` 已接近我们目标的 `10Hz`
- 对“整车实际体验”来说，综合效果最好

### 13.4 本轮结论

综合 1/2/3 三档：

| 配置 | YOLO 单帧 | color 实收 | `person_pose` | 综合评价 |
|---|---:|---:|---:|---|
| `intra=3` | 最快 | 最差 | 最差 | 不推荐 |
| `intra=2` | 中等 | 明显改善 | 明显改善 | 可用折中 |
| `intra=1` | 最慢 | 最好 | 最好 | **当前推荐默认值** |

最终结论：

1. 当前平台上，**单帧 benchmark 最优 != 整体链路最优**
2. 主矛盾是“推理线程过多挤压了图像接收/调度”，不是“YOLO 再快 3~5ms”
3. 因此默认值应优先选择：
   - `yolo.ort.intra_op_num_threads = 1`
4. `intra=2` 可作为保守备选；`intra=3` 在当前平台和当前链路上不建议继续作为默认配置

### 13.5 当前默认配置落地

截至本轮结束，工作区默认值已调整为：

```yaml
yolo:
  ort:
    intra_op_num_threads: 1
    inter_op_num_threads: 1
    execution_mode: sequential
```

同时，follower 自己的 full bringup 已收口到 color-only Astra 默认配置：

- `color_qos:=sensor_data`
- `enable_depth:=false`
- `enable_point_cloud:=false`
- `depth_registration:=false`
- `enable_d2c_viewer:=false`
- `enable_ir:=false`
