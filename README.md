# ROS2 Smart Follower

> 面向差速小车的人体视觉跟随系统（ROS2 Humble / C++17）。

本项目实现了一个分层、可扩展的智能跟随链路：

**感知（检测 + 跟踪 + 深度 + ReID） → 跟随控制 → 安全避障 → 仲裁输出**

并支持：
- 命名空间化部署（多机器人友好）
- Lifecycle 核心节点
- 键盘锁定/解锁/复位/急停
- 参数 YAML 外部化
- 诊断信息输出（diagnostic_updater）

---

## 1. 当前版本

- **当前开发版本：`alpha-0.0.1.2`**
- **上一调试快照：`dev-0.0.1.1`**
- 状态：In Progress（当前正在将感知同步链路从 `message_filters::ApproximateTime` 重构为“普通订阅 + 最近帧缓存 + 手工时间戳匹配”）

### 1.1 alpha-0.0.1.2 目标

`alpha-0.0.1.2` 替换当前三路 `message_filters` 同步方案，改为：

- 普通订阅（RGB / Depth / CameraInfo）
- 最近帧缓存
- 手工时间戳匹配
- 明确的超时 / 丢帧 / 诊断日志

目标是提高在真实相机与 synthetic pub 环境下的同步可控性、可调试性和稳定性。

### 1.1.1 640x480@30fps 管道审计（当前工作结论）

- 摄像头原始输入按 **640x480 @ 30fps** 接收，但感知主链默认只对 **每 3 个同步帧执行 1 次完整处理**，将主感知负载控制在约 **10Hz**。
- `detect_every_n_frames` 现在按“**已处理帧**”计数，默认值改为 `1`，避免降采样后检测频率被再次误降到约 3.3Hz。
- `sync_cache_size` 默认从 `30` 收紧到 `6`，降低手工同步缓存积压旧帧的概率。
- 避障节点改为“**只缓存最新 depth 帧 + 20Hz 定时器处理**”，并新增 `depth_sample_stride` 参数，避免对 30fps 深度图逐帧全 ROI 扫描。
- YOLO 运行时仍使用 **640x640** 输入，因为当前 ONNX 模型按该尺寸导出；这部分不是无效浪费，而是当前模型输入约束。

### 1.2 dev-0.0.1.1 保留内容

`dev-0.0.1.1` 已作为联调快照保留，主要用于回溯以下日志能力：

- 原始输入探针计数：RGB / Depth / CameraInfo 是否真的收到
- 三路同步回调计数：同步是否真的形成
- `person_pose` 发布计数：同步形成后是否进入发布路径
- 启动参数回显：YOLO / ReID 输入尺寸、topic、`sync_slop`

---

## 2. 仓库结构

```text
ros2_smart_follower/
├─ models/                              # ONNX 模型目录（运行时读取）
├─ src/
│  ├─ smart_follower_msgs/              # 自定义消息
│  │  └─ msg/
│  │     ├─ TrackedPerson.msg
│  │     ├─ PersonPoseArray.msg
│  │     └─ FollowCommand.msg
│  ├─ smart_follower_perception/        # 感知节点（Lifecycle, C++）
│  │  ├─ include/smart_follower_perception/
│  │  ├─ scripts/                       # 导出 ONNX 工具脚本
│  │  ├─ src/perception_node.cpp
│  │  └─ test/
│  ├─ smart_follower_control/           # 控制/避障/仲裁/键盘节点
│  │  ├─ config/control_params.yaml
│  │  ├─ include/smart_follower_control/
│  │  ├─ src/
│  │  │  ├─ follower_controller_node.cpp
│  │  │  ├─ ultrasonic_range_node.cpp
│  │  │  ├─ obstacle_avoidance_node.cpp
│  │  │  ├─ arbiter_node.cpp
│  │  │  └─ keyboard_command_node.cpp
│  │  └─ test/
│  └─ smart_follower_bringup/
│     ├─ config/perception_params.yaml
│     └─ launch/
│        ├─ smart_follower.launch.py
│        └─ smart_follower_only.launch.py
└─ README.md
```

---

## 3. 功能总览

### 3.1 感知（`smart_follower_perception`）
- RGB + Depth + CameraInfo 同步（ApproximateTime）
- YOLO 人体检测（检测帧执行）
- 卡尔曼 + 匈牙利匹配两阶段跟踪
- ReID 特征提取（ResNet50，2048维）与 EMA 更新
- 深度中心窗口中值采样（mm→m）
- 像素坐标 + 深度投影到 3D，并通过 TF 变换到 `base_footprint`
- 发布 `PersonPoseArray`（全目标 + lock_id/lock_state）

### 3.2 跟随控制（`follower_controller_node`）
- 20Hz PID 控制
- 两次感知间目标位置预测（速度外推）
- 线速度/角速度限幅
- 加速度限幅（rate limit）
- PID 抗积分饱和（反算回推）

### 3.3 超声 + 深度避障（`ultrasonic_range_node` + `obstacle_avoidance_node`）
- HC-SR04（pigpio）10Hz 中值滤波
- 深度图中央 ROI 百分位距离
- 与超声融合（取最危险距离）
- 动态安全距离：
  \[ d_{safe}(v)=d_{min}+v t_{react}+\frac{v^2}{2a_{brake}}+margin \]

### 3.4 仲裁（`arbiter_node`）
- 状态机：`FOLLOW_NORMAL / FOLLOW_DEGRADED / SEARCH / AVOID / STOP`
- 避障优先级最高
- STOP 需要人工 RESET 才能退出
- 固定 20Hz 持续发布全局 `/cmd_vel`

### 3.5 键盘命令（`keyboard_command_node`）
- `L`：LOCK
- `U`：UNLOCK
- `R`：RESET
- `Q`：ESTOP

---

## 4. 环境与依赖

## 4.1 基础环境
- Ubuntu 22.04
- ROS2 Humble
- C++17

### 4.2 主要 ROS2 依赖
- `rclcpp`, `rclcpp_lifecycle`, `lifecycle_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `rclcpp` 原生订阅 + 手工时间戳匹配同步
- `cv_bridge`, `image_geometry`
- `diagnostic_updater`

### 4.3 三方库
- OpenCV
- Eigen3
- pigpio（超声波 GPIO）
- ONNX Runtime（可选但推荐，用于 YOLO/ReID 推理）

> 若未检测到 ONNX Runtime，感知节点仍可编译，但会进入**推理桩模式**（日志中 `YOLO ready=0`、`ReID ready=0`）。

---

## 5. 模型文件

运行前建议准备：
- `models/yolo26n.onnx`
- `models/reid_resnet50_2048.onnx`

可使用：
- `src/smart_follower_perception/scripts/export_yolo_onnx.py`
- `src/smart_follower_perception/scripts/train_reid_resnet50.py`
- `src/smart_follower_perception/scripts/export_reid_onnx.py`
- `src/smart_follower_perception/scripts/validate_reid_onnx.py`

> 离线训练/导出工具链基于 Python（参考 `yolo-reid-training-demo-master`），运行时核心节点仍为 C++ + ONNX Runtime。

### 5.1 ReID（ResNet50-2048）离线流程

```bash
# 1) 训练/续训（Market1501）
python3 src/smart_follower_perception/scripts/train_reid_resnet50.py \
  --data-root reid-data --resume log/resnet50/model/model.pth.tar-30

# 2) 导出 ONNX
python3 src/smart_follower_perception/scripts/export_reid_onnx.py \
  --weights log/resnet50/model/model.pth.tar-60 \
  --output models/reid_resnet50_2048.onnx

# 3) 导出验收（输入/输出维度+随机推理）
python3 src/smart_follower_perception/scripts/validate_reid_onnx.py \
  --model models/reid_resnet50_2048.onnx
```

### 5.2 回滚到旧 ReID 模型

若需回滚，可将 `perception_params.yaml` 中：
- `reid.model_path` 改回旧模型路径
- `reid.input_w / reid.input_h` 改回旧输入尺寸
并重新编译消息与节点。

---

## 6. 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

可选测试：

```bash
colcon test
colcon test-result --verbose
```

---

## 7. 启动

### 7.1 仅启动跟随系统（不拉起底盘/相机）
```bash
source install/setup.bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py robot_ns:=robot1
```

### 7.2 联合已有 wheeltec 启动文件
```bash
source install/setup.bash
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 bringup_robot:=true bringup_camera:=true
```

---

## 8. 关键话题（默认）

> 以下均在命名空间下（如 `/robot1/...`），**最终输出 `/cmd_vel` 为全局话题**。

- 输入：
  - `/camera/color/image_raw`
  - `/camera/depth/image_raw`
  - `/camera/color/camera_info`
- 感知输出：
  - `person_pose` (`smart_follower_msgs/PersonPoseArray`)
- 控制链路：
  - `cmd_vel_follow`
  - `cmd_vel_avoid`
  - `/cmd_vel`（最终）
- 命令：
  - `follow_command` (`smart_follower_msgs/FollowCommand`)
- 传感器：
  - `left_ultrasonic/range`
  - `right_ultrasonic/range`

---

## 9. 参数说明

- 感知参数：`src/smart_follower_bringup/config/perception_params.yaml`
  - 模型路径、检测频率、匹配权重、深度范围、锁定策略等
- 控制与安全参数：`src/smart_follower_control/config/control_params.yaml`
  - PID、限幅、动态安全距离、仲裁超时、GPIO 引脚等

参数均支持 ROS2 参数机制，可在 launch 时覆盖。

---

## 10. 自检结果（当前 Alpha）

在未连接底盘主控的 VM 自检中，已验证：
- 节点全部成功启动
- Lifecycle 核心节点均为 `active`
- 关键话题类型正确
- `/cmd_vel`、`cmd_vel_follow`、`cmd_vel_avoid` 达到约 20Hz
- 超声发布约 10Hz
- 合成相机流可触发感知发布链路

---

## 11. 已知事项

1. `vision_msgs` 在部分环境下 apt 安装受镜像限制，本项目当前不依赖它。  
2. 若 ONNX Runtime 未正确安装或模型缺失，将无法进行真实检测/ReID 推理。  
3. 不连接真实传感器时，超声距离常见为 `inf`，属于预期现象。  

---

## 12. 开发建议

- 先完成相机 D2C 对齐与 TF 外参标定，再调跟随参数。
- 先在 `smart_follower_only.launch.py` 做算法联调，再接底盘闭环。
- 先验证仲裁状态机与看门狗，再逐步放开速度上限。

---

## 13. License

暂未指定（可后续补充 MIT / Apache-2.0）。

---

## 14. 维护者

- 项目维护：`gdutwattbit`
- 版本起点：`alpha-0.0.1.1`
