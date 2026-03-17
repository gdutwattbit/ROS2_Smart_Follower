# ROS2 Smart Follower

> 面向差速小车的人体视觉跟随系统（ROS2 Humble / C++17）。

本项目实现了一条完整的智能跟随链路：

**感知（检测 + 跟踪 + 深度 + ReID） → 跟随控制 → 安全避障 → 仲裁输出**

当前代码已经具备首个可用 Alpha 版本的基本形态，适合作为实机联调、性能分析和后续优化的基础。

---

## 1. 当前版本

- **当前发布版本：`alpha-0.1.1`**
- **上一稳定快照：`alpha-0.0.4`**
- **上一调试快照：`dev-0.0.1.1`**
- **状态：Usable Alpha**

### alpha-0.1.1（2026-03-17）本轮重点

这一版主要补的是**可读性和语义一致性**，重点不是改算法，而是先把“控制侧公共样板、版本号、消息语义说明”收拢到位：

- 控制侧新增最小公共 helper：`constants.hpp` + `lifecycle_utils.hpp`
- P1 继续拆分控制侧：新增 `arbiter_runtime.hpp/cpp` 与 `ultrasonic_runtime.hpp/cpp`，把状态机运行态和超声硬件/滤波逻辑从节点类中抽离
- 控制侧 Lifecycle main 样板、激活判断、频率转周期工具开始复用，降低后续继续拆分时的重复劳动
- 统一感知侧与控制侧运行时版本字符串到 `alpha-0.1.1`
- 为 `TrackedPerson.msg` / `PersonPoseArray.msg` 补充 2D bbox 与 3D pose 的语义注释
- README 补充消息语义说明，避免 `header.frame_id` 与像素 bbox 混淆

### alpha-0.1.0（2026-03-17）上一轮重点

上一版主要完成了感知节点的结构性重构，目标不是改算法，而是把代码从“能跑”整理成“能维护、能测试、能继续迭代”：

- 将超大文件 `perception_node.cpp` 按职责拆分为：
  - `runtime`
  - `frame_sync`
  - `tracker`
  - `lock_manager`
  - `geometry_utils`
  - `params / diagnostics`
- 新增 `smart_follower_perception_core` 复用库，避免测试和主程序各自复制实现。
- 新增感知单元测试：
  - `test_frame_sync`
  - `test_lock_manager`
  - `test_tracker`
- 保留现有外部接口不变：
  - 节点名不变
  - 话题名不变
  - 参数名不变
  - Lifecycle 行为不变
  - launch 用法不变
- 在虚拟机环境完成一轮 build/test 与合成数据链路验证，确认 `/robot1/person_pose` 发布链路未回退。

---

## 2. 项目目标

本项目面向带深度相机和超声波的差速底盘，目标是实现：

- 基于 YOLO26n 的人体检测
- 基于卡尔曼 + 匈牙利匹配 + ReID 的目标保持
- 基于深度图的目标测距与坐标变换
- 基于 PID 的机器人跟随控制
- 基于超声波 + 深度距离的安全避障
- 基于状态机的最终速度仲裁
- 显式锁定 / 解锁 / 复位 / 急停控制

系统默认假设：

- 相机输入：`640x480 @ 30fps`
- 感知处理主频：约 `10Hz`（默认每 3 帧处理一次）
- 控制输出频率：`20Hz`
- 目标跟随距离：`1.0m`
- 底盘类型：差速小车

---

## 3. 工程结构

```text
ros2_smart_follower/
├─ src/
│  ├─ smart_follower_msgs/         # 自定义消息
│  ├─ smart_follower_perception/   # 感知链路（YOLO / ReID / 跟踪 / 深度 / 锁定）
│  ├─ smart_follower_control/      # 跟随控制 / 避障 / 仲裁 / 键盘 / 超声波
│  └─ smart_follower_bringup/      # 参数与 launch
├─ models/                         # ONNX / traced 模型
├─ scripts/                        # 环境安装脚本
├─ pyproject.toml                  # uv 管理的 Python 工具链
├─ CHANGELOG.md
└─ README.md
```

### 3.1 包说明

#### `smart_follower_msgs`
公共消息接口：
- `TrackedPerson.msg`
- `PersonPoseArray.msg`
- `FollowCommand.msg`

消息语义补充：
- `TrackedPerson.bbox`：同步彩色图像上的**像素坐标**
- `TrackedPerson.position / velocity`：`PersonPoseArray.header.frame_id` 对应坐标系下的 **3D 信息**
- `PersonPoseArray.header.stamp`：源图像时间戳
- `PersonPoseArray.header.frame_id`：只约束 3D 字段，**不约束 bbox**

#### `smart_follower_perception`
核心职责：
- 订阅 RGB / Depth / CameraInfo
- 手工同步最近帧缓存
- YOLO26n 人体检测
- ResNet50-ReID 特征提取（2048 维）
- 卡尔曼跟踪与重识别
- 深度采样与 `base_footprint` 坐标变换
- 发布 `/person_pose`

#### `smart_follower_control`
核心职责：
- 跟随控制节点：输出 `/cmd_vel_follow`
- 避障节点：输出 `/cmd_vel_avoid`
- 仲裁节点：输出最终 `/cmd_vel`
- 键盘命令节点：发布锁定/解锁/复位/急停命令
- 超声波量测节点：读取左右 HC-SR04 距离

#### `smart_follower_bringup`
核心职责：
- 统一 launch 入口
- YAML 参数管理
- 对接现有相机/底盘启动流程

---

## 4. 当前感知链路说明

### 4.1 输入

默认订阅：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

注意：
- 输出 `person_pose_topic` 默认为**相对话题名**，因此在 `robot_ns:=robot1` 下，实际发布为：
  - `/robot1/person_pose`
- 输入话题默认是**绝对路径**，不会随命名空间自动变化。

### 4.1.1 `/person_pose` 消息语义

这是当前版本一个容易误读、但现在已经明确写进消息注释和文档的点：

- `PersonPoseArray.header.frame_id` 默认是 `base_footprint`
- 这表示 **`TrackedPerson.position` / `velocity` 的参考坐标系**
- 但 `TrackedPerson.bbox` 仍然是 **图像像素空间**，不是 `base_footprint` 坐标

也就是说，当前 `TrackedPerson` 是一个“2D 检测结果 + 3D 目标状态”的混合消息。
这在工程上是有意为之，但文档上必须写清楚，否则新接手的人很容易误会。

### 4.2 同步方式

当前版本已经从 `message_filters` 切换为：

- 普通订阅
- 最近帧缓存
- 手工时间戳匹配

这样做的目的：

- 更易调试
- 更易记录同步丢帧原因
- 更容易针对真实相机和 synthetic pub 做容错

### 4.3 处理节流

当前默认参数：

- `process_every_n_frames: 3`
- `detect_every_n_frames: 1`
- `sync_cache_size: 6`
- `sync_slop: 0.04`

含义：

- 相机若输入 30fps，同步帧中默认每 3 帧处理 1 次
- 处理后的每一帧都会进入跟踪流水
- 当前默认每个“处理帧”都执行一次检测

### 4.4 检测与 ReID

- YOLO：`models/yolo26n.onnx`
- ReID：`models/reid_resnet50_2048.onnx`
- ReID 特征维度：`2048`
- ReID 输入尺寸：`128 x 256`
- YOLO 输入尺寸：`640 x 640`

### 4.5 深度与 D2C

当前项目假设：

- 深度主话题：`/camera/depth/image_raw`
- 深度单位：**mm**
- `depth_registration=true` 时，`depth/image_raw` 已对齐到彩色光学坐标系
- 开启 D2C 后，depth 的 `frame_id` 会切到 color optical frame

因此，当前测距逻辑是：

1. 在彩色图上取检测框
2. 到对齐后的深度图同像素位置采样
3. 做 5x5 中值滤波
4. 将 `(u, v, d)` 转到相机坐标，再经 `tf2` 转到 `base_footprint`

---

## 5. 控制、避障与仲裁

### 5.1 跟随控制

- 固定 `20Hz`
- 订阅 `/person_pose`
- 根据目标在 `base_footprint` 下的位置计算线速度与角速度
- 支持：
  - 线速度/角速度限幅
  - 加速度限制
  - 抗积分饱和
  - 角度死区
  - 目标超时保护

### 5.2 避障节点

避障输入来自两部分：

- 左右超声波距离
- 深度图中央 ROI 距离

避障逻辑特点：

- 动态安全距离 `d_safe(v)`
- 进入/退出滞回
- danger 阈值
- 输出 `/cmd_vel_avoid`
- 无障碍物时持续输出零速度，不让旧指令“卡住”

### 5.3 仲裁节点

仲裁状态机：

- `FOLLOW_NORMAL`
- `FOLLOW_DEGRADED`
- `SEARCH`
- `AVOID`
- `STOP`

关键规则：

- 避障优先级最高
- `/cmd_vel` 固定 20Hz 持续发布
- STOP 不自动退出，需人工 `RESET`

---

## 6. 键盘控制

默认键位：

- `L`：锁定
- `U`：解锁
- `R`：复位
- `Q`：急停

说明：

- 系统默认不自动跟人
- 需要显式锁定后才进入目标保持逻辑
- STOP 状态需要人工复位

---

## 7. 超声波配置说明

当前项目已经从 `pigpio` 切换到 **libgpiod**。

原因：

- 更适合当前目标环境
- 更容易部署到较新的系统上
- 避免额外维护 pigpio 守护进程

GPIO 通过 YAML 配置，左右模块分别独立设置。

当前联调中验证过的常见配置包括：

- 左侧：`Trig=23 / Echo=24`
- 右侧：`Trig=4 / Echo=14`

> 实机接线请以最新 YAML 为准，不要只看历史记录。

为了降低串扰风险，当前实现已经加入**分时测量**逻辑，避免左右模块同时触发。

---

## 8. Python 工具链（uv）

运行时核心节点仍然是 **C++**。

`uv` 只用于管理离线工具链，例如：

- YOLO 导出
- ReID 训练
- ReID ONNX 导出
- ONNX 校验

### 8.1 依赖组建议

- `validate`：目标机建议优先使用
- `train`：需要在目标机做训练验证时再启用
- `export`：更推荐在本地开发机执行

### 8.2 当前策略

为避免目标机误装 GPU 重型包：

- `torch`
- `torchvision`

已经固定到 **CPU-only** 源。

---

## 9. 快速开始

### 9.1 编译

```bash
cd ~/ros2_smart_follower
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 9.2 仅启动本系统

```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py robot_ns:=robot1
```

### 9.3 联合底盘与相机启动

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py robot_ns:=robot1
```

### 9.4 参数热更新示例

```bash
ros2 param set /robot1/follower_controller_node target_timeout 0.40
ros2 param set /robot1/arbiter_node search_angular_speed 0.40
ros2 param set /robot1/obstacle_avoidance_node depth_sample_stride 4
ros2 param set /robot1/ultrasonic_range_node rate 8.0
```

---

## 10. 模型文件约定

模型默认放置在项目根目录 `models/`：

- `models/yolo26n.onnx`
- `models/reid_resnet50_2048.onnx`
- `models/resnet50_traced.pt`（可选，中间产物）

若从不同工作目录启动，运行时会尽量自动解析相对路径，但仍建议优先使用清晰的部署目录。

---

## 11. 测试与验证

### 11.1 已验证内容

在虚拟机 `wheeltec@192.168.220.131` 上已完成：

- `colcon build --symlink-install`
- `colcon test`
- 感知链路单元测试通过
- 控制状态机测试通过
- 合成相机数据注入后 `/person_pose` 发布链路可工作

### 11.2 当前已知情况

- VM 中用 Python `rclpy` 发布大尺寸 synthetic image 时，发布端本身很容易成为瓶颈。
- 因此在性能分析时，需要把“测试工装瓶颈”和“系统真实瓶颈”分开看。
- 当前控制链路 20Hz 基本稳定，后续真正需要重点 profiling 的仍然是感知侧，尤其是图像搬运、YOLO 推理和 ReID 开销。

---

## 12. 当前版本演进

- `alpha-0.0.1`：首个 Alpha 主链路落地
- `alpha-0.0.1.1`：ReID 切换到 ResNet50-2048
- `alpha-0.0.1.2`：感知同步切换为手工缓存同步
- `alpha-0.0.2`：首个可用版本，修复 D2C、热更新、超声波与仲裁细节
- `alpha-0.0.3`：稳定性和参数热更新补强
- `alpha-0.0.4`：引入 uv CPU-only 工具链
- `alpha-0.1.0`：感知节点结构重构 + 单元测试补齐
- `alpha-0.1.1`：控制侧最小公共骨架、版本统一、消息语义澄清

完整历史请见：[CHANGELOG.md](./CHANGELOG.md)

---

## 13. 后续计划

接下来更值得做的方向：

1. 感知真实瓶颈 profiling
2. YOLO / ReID 推理耗时拆分统计
3. 感知线程与控制线程进一步解耦
4. 更细的性能日志与诊断指标
5. 实机多人场景下的锁定、遮挡恢复与切人策略回归

---

## 14. 说明

本项目当前仍处于 Alpha 阶段，但已经从“概念验证”进入“可联调、可维护、可逐步部署”的阶段。

如果你正在接手这套系统，建议优先按下面顺序理解：

1. `smart_follower_bringup/config/*.yaml`
2. `smart_follower_bringup/launch/*.launch.py`
3. `smart_follower_msgs/msg/*`
4. `smart_follower_control/src/*`
5. `smart_follower_perception/src/*`

这样会比较顺。
