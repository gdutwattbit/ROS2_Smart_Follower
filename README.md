# ROS2 Smart Follower

面向差速小车的人体视觉跟随系统，基于 **ROS 2 Humble + C++17** 实现。

主链路固定为：

**感知（YOLO + 跟踪 + 深度 + ReID） → 跟随控制 → 避障 → 仲裁 → `/cmd_vel`**

---

## 当前版本

- **当前发布版本：`alpha-0.1.4`**
- **上一版本：`alpha-0.1.3`**
- **状态：Usable Alpha**

### alpha-0.1.4（2026-03-22）更新重点

这一版主要聚焦在 **代码结构整理、控制侧可维护性提升、项目文档补齐**：

- 感知侧继续完成结构收口：
  - 新增 `perception_pipeline.*`
  - `perception_node.cpp` 进一步瘦身，主文件更接近“薄节点入口”
  - 参数与 diagnostics 实现拆到独立 `.cpp`
- 控制侧完成一轮 **P2 轻量重构**：
  - 新增 `control_node_common.*`
  - 统一参数夹紧、成功返回值、publisher 重建辅助逻辑
  - `arbiter / follower / obstacle / ultrasonic` 四个节点减少重复样板代码
- 新增包内说明文档：
  - `src/smart_follower_perception/README.md`
  - `src/smart_follower_control/README.md`
  - `src/smart_follower_bringup/README.md`
- 新增新人文档：
  - `docs/新人上手指南.md`
- 已在虚拟机 `wheeltec@192.168.220.131` 上完成 `smart_follower_perception + smart_follower_control` 联合编译验证

---

## 项目目标

本项目面向带深度相机与超声波的小车平台，目标是实现：

- 基于 YOLO 的人体检测
- 基于卡尔曼 + 匈牙利匹配 + ReID 的多人跟踪
- 基于深度图的测距与 3D 坐标转换
- 基于 PID 的机器人跟随控制
- 基于深度 + 超声波的安全避障
- 基于状态机的速度仲裁与人工复位

当前默认系统假设：

- 相机输入：`640x480 @ 30fps`
- 深度主话题：`/camera/depth/image_raw`
- D2C：已开启，深度语义对齐到彩色坐标
- 感知输出：`/robot1/person_pose`
- 最终底盘输出：全局 `/cmd_vel`

---

## 工程结构

```text
ros2_smart_follower/
├── src/
│   ├── smart_follower_msgs/
│   ├── smart_follower_perception/
│   ├── smart_follower_control/
│   └── smart_follower_bringup/
├── docs/
│   └── 新人上手指南.md
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

## 包职责

### 1. `smart_follower_msgs`
公共消息定义：
- `TrackedPerson.msg`
- `PersonPoseArray.msg`
- `FollowCommand.msg`

### 2. `smart_follower_perception`
感知链路：
- RGB / Depth / CameraInfo 手工同步
- YOLO 检测
- ReID 特征提取
- 多目标跟踪
- 锁定 / 丢失 / 重识别
- TF 转换并发布 `/person_pose`

包内阅读说明见：
- `src/smart_follower_perception/README.md`

### 3. `smart_follower_control`
控制链路：
- 跟随控制
- 避障
- 仲裁
- 键盘命令
- 超声波驱动

包内阅读说明见：
- `src/smart_follower_control/README.md`

### 4. `smart_follower_bringup`
启动与参数装配：
- `smart_follower_only.launch.py`
- `smart_follower.launch.py`
- perception / control YAML

包内阅读说明见：
- `src/smart_follower_bringup/README.md`

---

## 新人推荐上手顺序

建议按下面顺序读：

1. `docs/新人上手指南.md`
2. `src/smart_follower_bringup/launch/*.launch.py`
3. `src/smart_follower_msgs/msg/*.msg`
4. `src/smart_follower_perception/README.md`
5. `src/smart_follower_control/README.md`
6. 根目录的性能与调优文档：
   - `test.md`
   - `当前推荐运行组合.md`
   - `YOLO_优化实施清单.md`
   - `ORT_线程调优测试计划.md`

---

## 当前默认模型与参数

当前默认 YAML 使用推荐运行组合：

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
- 消息接口中的外观特征字段仍保持 `2048` 维
- 当前使用的 OSNet `512` 维输出会补零到 `2048` 维，以维持接口兼容

---

## 构建与运行

### 构建

```bash
colcon build --symlink-install
```

### 只启动本项目链路

```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py
```

### 完整启动（联合底盘与相机）

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

### 常用观测命令

```bash
ros2 topic hz /robot1/person_pose
ros2 topic echo /robot1/person_pose --once
ros2 topic hz /robot1/cmd_vel_follow
ros2 topic hz /robot1/cmd_vel_avoid
ros2 topic hz /cmd_vel
ros2 lifecycle get /robot1/perception_node
```

---

## Profiling 与调优文档

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

建议阅读顺序：

1. `test.md`
2. `当前推荐运行组合.md`
3. `YOLO_优化实施清单.md`
4. `ORT_线程调优测试计划.md`

---

## 版本轨迹

- `alpha-0.0.1`：首个 Alpha 主链路落地
- `alpha-0.0.1.1`：ReID 切换到 ResNet50-2048
- `alpha-0.0.1.2`：感知同步切换为手工缓存同步
- `alpha-0.0.2`：首个可用版本，修复 D2C、热更新、超声波链路细节
- `alpha-0.0.3`：稳定性与动态参数热更新增强
- `alpha-0.0.4`：引入 uv CPU-only Python 工具链
- `alpha-0.1.0`：感知结构重构 + 单元测试补齐
- `alpha-0.1.1`：控制侧公共骨架、版本口径统一、消息语义注释增强
- `alpha-0.1.2`：稳定性修复、diagnostics 分级、依赖与 lint 接入补齐
- `alpha-0.1.3`：推荐模型默认化、异步链路文档收口、TF/消息 profiling 细化
- `alpha-0.1.4`：感知/控制结构继续收口，新增包内文档与新人上手指南

---

## 备注

如果你准备继续做性能优化，建议先从：

- `docs/新人上手指南.md`
- `test.md`
- `当前推荐运行组合.md`

开始，先确认当前链路、当前推荐组合和现有实测数据，再决定下一步优化方向。
