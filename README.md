# ROS2 Smart Follower

面向差速小车的人体视觉跟随系统，基于 **ROS 2 Humble + C++17** 实现。

主链路：

**感知（YOLO + 跟踪 + 深度 + ReID）→ 跟随控制 → 安全避障 → 仲裁输出**

---

## 1. 当前版本

- **当前发布版本：`alpha-0.1.3`**
- **上一版本：`alpha-0.1.2`**
- **状态：Usable Alpha**

### alpha-0.1.3（2026-03-22）本轮重点

这一版主要聚焦在 **感知性能优化、默认运行组合收口、profiling 细化**：

- 默认感知模型路径已切换到当前实测推荐组合：
  - YOLO：`models/yolo26n_static_480x640_simplify_e2e.onnx`
  - ReID：`models/osnet_x0_5_512.onnx`
- `perception_params.yaml` 默认输入尺寸已收口为相机真实规格：`640x480 @ 30fps`
- 感知异步 worker 链路继续沿用，配套 `pipeline_utils.*` 收口检测任务与消息构建流程
- ReID 裁剪去掉不必要的 `clone()`，减少每个检测框的一次图像拷贝
- `/person_pose` 消息构建阶段新增细粒度 profiling：
  - `tf_lookup_ms`
  - `tf_transform_ms`
  - `message_fill_ms`
- TF 查询由“每个目标查一次”改为“每帧查一次后复用”，减少重复开销
- ONNX Runtime 线程参数已接入感知配置，可继续做实机组合调优
- 新增并更新根目录文档：
  - `test.md`
  - `当前推荐运行组合.md`
  - `YOLO_优化实施清单.md`
  - `ORT_线程调优测试计划.md`

### 当前推荐运行组合（实机验证）

- 感知：异步版 perception
- YOLO：`models/yolo26n_static_480x640_simplify_e2e.onnx`
- ReID：`models/osnet_x0_5_512.onnx`
- YOLO ORT：`intra=3 / inter=1 / sequential`
- ReID ORT：`intra=1 / inter=1 / sequential`

详细数据请看：
- `test.md`
- `当前推荐运行组合.md`

---

## 2. 项目目标

本项目面向带深度相机与超声波的差速底盘，目标是实现：

- 基于 YOLO 的人体检测
- 基于卡尔曼 + 匈牙利匹配 + ReID 的多目标跟踪
- 基于深度图的测距与 3D 坐标变换
- 基于 PID 的机器人跟随控制
- 基于深度 + 超声波的安全避障
- 基于状态机的速度仲裁与人工复位

当前默认系统假设：

- 相机输入：`640x480 @ 30fps`
- 深度：`/camera/depth/image_raw`
- D2C：已开启，深度对齐到彩色
- 感知输出：`/robot1/person_pose`（默认命名空间下）
- 最终底盘输出：全局 `/cmd_vel`

---

## 3. 工程结构

```text
ros2_smart_follower/
├── src/
│   ├── smart_follower_msgs/
│   ├── smart_follower_perception/
│   ├── smart_follower_control/
│   └── smart_follower_bringup/
├── models/
├── scripts/
├── README.md
├── CHANGELOG.md
├── DEPENDENCIES.md
├── test.md
├── 当前推荐运行组合.md
├── YOLO_优化实施清单.md
└── ORT_线程调优测试计划.md
```

---

## 4. 关键特性

### 4.1 感知链路

输入：
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

主要逻辑：
- 普通订阅 + 最近帧缓存 + 手工时间戳匹配
- YOLO 低频检测，跟踪补齐中间帧
- ReID 特征提取与短期记忆恢复
- 深度 5x5 中值采样
- `tf2` 转换到 `base_footprint`
- 显式锁定 / 保持 / LOST 后重识别

### 4.2 控制链路

- 跟随控制：20Hz
- 避障：深度中央扇区 + 左右超声波融合
- 仲裁：`FOLLOW / SEARCH / AVOID / STOP`
- 急停后不自动恢复，需人工 `RESET`

### 4.3 生命周期与热更新

核心节点使用 Lifecycle。
当前已支持多项运行时参数热更新，包括：
- 感知模型路径 / 线程配置
- 跟踪与同步参数
- 控制 / 避障 / 超声波相关参数

---

## 5. 当前默认模型与参数

默认 YAML 已切到当前推荐组合：

```yaml
yolo:
  model_path: models/yolo26n_static_480x640_simplify_e2e.onnx
  input_w: 640
  input_h: 480
  ort:
    intra_op_num_threads: 3
    inter_op_num_threads: 1
    execution_mode: sequential

reid:
  model_path: models/osnet_x0_5_512.onnx
  input_w: 128
  input_h: 256
  ort:
    intra_op_num_threads: 1
    inter_op_num_threads: 1
    execution_mode: sequential
```

说明：
- 主消息接口仍保持 `2048` 维外观特征字段
- OSNet 的 `512` 维输出会补零到 `2048` 维，便于维持当前接口兼容

---

## 6. 运行与测试

### 6.1 构建

```bash
colcon build --symlink-install
```

### 6.2 启动仅本项目链路

```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py
```

### 6.3 完整启动

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

### 6.4 常用观测项

```bash
ros2 topic hz /robot1/person_pose
ros2 topic echo /robot1/person_pose --once
ros2 lifecycle get /robot1/perception_node
ros2 param get /robot1/perception_node yolo.model_path
```

---

## 7. Profiling 与调优建议

当前 profiling 已覆盖：

- `camera_info_ms`
- `cv_bridge_ms`
- `yolo_ms`
- `depth_ms`
- `reid_ms`
- `recover_ms`
- `tracking_ms`
- `lock_ms`
- `tf_lookup_ms`
- `tf_transform_ms`
- `message_fill_ms`
- `message_ms`
- `publish_ms`
- `total_ms`

建议优先优化顺序：

1. YOLO 主耗时
2. 检测频率自适应
3. ReID 在真实有人场景下的线程调优
4. ROI 检测 / 更细粒度流水线优化

---

## 8. 版本轨迹

- `alpha-0.0.1`：首个 Alpha 主链路落地
- `alpha-0.0.1.1`：ReID 切换到 ResNet50-2048
- `alpha-0.0.1.2`：感知同步切换为手工缓存同步
- `alpha-0.0.2`：首个可用版本，修复 D2C、热更新、超声波与仲裁细节
- `alpha-0.0.3`：稳定性与动态参数热更新补强
- `alpha-0.0.4`：引入 uv CPU-only Python 工具链
- `alpha-0.1.0`：感知结构重构 + 单元测试补齐
- `alpha-0.1.1`：控制侧公共骨架、版本统一、消息语义澄清
- `alpha-0.1.2`：P0 稳定性修复、diagnostics 分级、bringup 依赖补齐、lint 实接入
- `alpha-0.1.3`：推荐模型默认化、感知异步链路文档收口、TF/消息 profiling 细化、减少不必要图像拷贝

---

## 9. 备注

如果你要继续做性能优化，建议先从 `test.md` 和 `当前推荐运行组合.md` 开始，看完当前实机数据再决定下一步。
