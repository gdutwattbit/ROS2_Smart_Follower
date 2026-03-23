# ROS2 Smart Follower

一个面向树莓派 / ROS 2 Humble 的智能跟随小车项目，主链路采用 **YOLO + ReID + 跟踪** 做人物感知，控制侧负责跟随、避障、仲裁，并最终输出 `/cmd_vel`。

> 当前基线版本：`alpha-0.1.6`
> 
> 当前主线状态：**纯 RGB + 单目位置估计 + 超声波避障**。

---

## 1. 当前架构

```text
彩色图像 → YOLO 检测 → ReID 特征 → Tracker → Lock Manager
       → 单目位置估计 → /person_pose
       → Follower / Obstacle / Arbiter → /cmd_vel
```

当前版本的几个关键点：
- 感知侧只订阅彩色图像，不再依赖深度图和 `camera_info`
- `TrackedPerson.position` 仍然保留，改由 bbox 做单目估计得到
- `TrackedPerson.depth_m` 为兼容字段；当前无真实深度输入时写入 `NaN`
- 避障侧当前使用 **左右超声波**，不再消费深度图

---

## 2. 仓库结构

```text
ros2_smart_follower/
├─ src/
│  ├─ smart_follower_msgs/
│  ├─ smart_follower_perception/
│  ├─ smart_follower_control/
│  └─ smart_follower_bringup/
├─ docs/
├─ models/
├─ scripts/
├─ README.md
├─ CHANGELOG.md
├─ DEPENDENCIES.md
└─ test.md
```

---

## 3. 各包职责

### `smart_follower_msgs`
消息定义：
- `TrackedPerson.msg`
- `PersonPoseArray.msg`
- `FollowCommand.msg`

### `smart_follower_perception`
负责：
- 彩色图像接入
- YOLO 检测
- ReID 特征提取
- 多目标跟踪
- 锁定 / 切人策略
- 单目位置估计
- 发布 `/person_pose`

### `smart_follower_control`
负责：
- 跟随控制
- 超声波采样
- 超声波避障
- 指令仲裁
- 键盘控制

### `smart_follower_bringup`
负责：
- launch 组织
- 默认 YAML 参数

---

## 4. 运行方式

### 仅启动核心跟随链路
```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py
```

### 启动完整 bringup
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

---

## 5. 当前默认模型

推荐默认组合：
- `models/yolo26n_static_480x640_simplify_e2e.onnx`
- `models/osnet_x0_5_512.onnx`

如果模型未放在 `models/` 下，需要同步修改：
- `src/smart_follower_bringup/config/perception_params.yaml`

---

## 6. 当前值得先看的文件

如果你刚接手项目，建议先看这些：
- `src/smart_follower_perception/src/perception_node.cpp`
- `src/smart_follower_perception/src/perception_pipeline.cpp`
- `src/smart_follower_perception/src/tracker.cpp`
- `src/smart_follower_control/src/follower_controller_node.cpp`
- `src/smart_follower_control/src/follower_runtime.cpp`
- `src/smart_follower_control/src/obstacle_runtime.cpp`
- `src/smart_follower_bringup/config/perception_params.yaml`
- `src/smart_follower_control/config/control_params.yaml`

---

## 7. 最近几轮重点演进

- `alpha-0.1.3`
  - 补齐了更细的 profiling 字段
  - 固定了当前推荐模型组合与线程参数
- `alpha-0.1.4`
  - 做了感知 / 控制两侧的可读性重构
  - 增加包内 README 与新人上手文档
- `alpha-0.1.5`
  - 回退了统一缓冲区实验，恢复实时单帧缓冲主线
  - 保留 YOLO / ORT 热路径优化与线程调优结果
- `alpha-0.1.6`
  - 移除深度相机主链路，感知改为纯 RGB + 单目位置估计
  - 移除避障侧深度依赖，保留左右超声波避障
  - 同步清理参数、依赖、测试与文档

---

## 8. 当前状态说明

这份工作区已经完成：
- 深度相机主链路移除
- 避障深度依赖移除
- 相关参数、诊断、测试同步清理

后续如果要继续推进，建议优先做：
1. 再确认单目距离估计参数
2. 做一轮实车跟随回归
3. 再决定是否继续走更激进的推理优化路线
