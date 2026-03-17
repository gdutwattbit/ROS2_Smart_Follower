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

- **当前可用版本：`alpha-0.0.4`**
- **上一调试快照：`dev-0.0.1.1`**
- 状态：Usable Alpha（主链路可启动，适合作为首个实机可用版本）
- 注意：实机使用 Astra 深度相机时，外部 `turn_on_wheeltec_robot/wheeltec_camera.launch.py` 需显式传入 `depth_registration=true`，确保 `/camera/depth/image_raw` 对齐到彩色光学坐标系。

### 1.1 alpha-0.0.2 目标

`alpha-0.0.2` 作为首个可用版本，已完成手工时间同步、D2C 对齐验证、/cmd_vel 20Hz 仲裁验证，以及 Pi 5 上超声波 GPIO 后端切换为 libgpiod。

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

### 1.3 alpha-0.0.4 稳定性补丁（2026-03-17）

本轮收尾主要补的是**运行稳定性**和**在线调参能力**，没有改动主链路架构：

- `perception_node`
  - 已支持参数热更新：模型路径、输入尺寸、同步窗口、话题名修改后可在线重建订阅与发布器。
  - 模型路径增加自动解析，避免因启动目录不同导致 `models/*.onnx` 找不到。
- `follower_controller_node`
  - 已支持在线热更新 `target_timeout`、PID 参数、控制频率、限幅参数和输出话题。
- `arbiter_node`
  - 已支持在线热更新状态机阈值、避障防抖参数、搜索角速度、输出话题。
- `obstacle_avoidance_node`
  - 已支持在线热更新深度 ROI、百分位、采样步长、动态安全距离参数和输入/输出话题。
- `ultrasonic_range_node`
  - 已支持在线热更新频率、GPIO、frame、左右量程话题。
  - 为规避 active 状态下直接重建 GPIO 资源导致的阻塞，超声波节点改为**参数回调只登记重配置请求，由下一次定时器 tick 完成重建**。
- 执行器
  - 控制、避障、仲裁、超声波、感知链路统一固定为 `SingleThreadedExecutor`，降低生命周期切换与定时器/回调交错时的不确定性。

### 1.4 本轮虚拟机自检结果

调试机：`wheeltec@192.168.220.131`

已验证：

1. `smart_follower_control` 在虚拟机上可正常编译：
   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/wheeltec/ros2_smart_follower
   colcon build --symlink-install --packages-select smart_follower_control
   ```
2. `smart_follower_only.launch.py` 可正常拉起各核心节点。
3. 以下热更新命令已验证成功：
   ```bash
   ros2 param set /robot1/follower_controller_node target_timeout 0.40
   ros2 param set /robot1/arbiter_node search_angular_speed 0.40
   ros2 param set /robot1/obstacle_avoidance_node depth_sample_stride 4
   ros2 param set /robot1/ultrasonic_range_node rate 8.0
   ```
4. 虚拟机无 GPIO 后端时，超声波节点会降级为 `dry` 模式并持续发布 `inf` 距离；该行为符合预期。

### 1.5 运行时调参注意事项

- 若 `ros2 node list` / `ros2 param ...` 偶发卡住，先执行一次：
  ```bash
  ros2 daemon stop
  ```
  然后再重新执行查询或设参。
- 对超声波节点改 GPIO / 频率时，日志中应出现：
  - `ultrasonic parameters hot-reloaded ...`
- 对感知节点改同步/模型参数时，日志中应出现：
  - `parameters hot-reloaded ...`
- 当前热更新实现目标是**不重启节点完成工程调试**，不是动态无损切换；参数修改时可能清空局部缓存，这是预期行为。

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
- HC-SR04（libgpiod，适配 Raspberry Pi 5）10Hz 中值滤波
- 左右超声波分时触发，降低串扰
- 已验证 GPIO 配置：左侧 `Trig=23 / Echo=24`，右侧 `Trig=4 / Echo=14`
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
- libgpiod（超声波 GPIO，Raspberry Pi 5 实机验证）
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


### 7.3 alpha-0.0.2 超声波接线记录
- 左侧 HC-SR04：`Trig=GPIO23`，`Echo=GPIO24`
- 右侧 HC-SR04：`Trig=GPIO04`，`Echo=GPIO14`
- 当前版本按上述引脚写入 `src/smart_follower_control/config/control_params.yaml`
- 主控环境为 Raspberry Pi 5，超声波 GPIO 后端使用 `libgpiod`，设备节点实测为 `/dev/gpiochip4`

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


## 15. Python 工具链包管理（uv）

`uv` 只用于**模型训练 / 导出 / 校验脚本**的 Python 环境管理，不替代 `rosdep`、`colcon` 或 ROS2 运行时。

### 15.1 推荐使用边界

按当前工程实践，建议这样用：

- `train` 组中的 `torch / torchvision` 现在固定走 **PyTorch CPU-only index**，避免在 Linux 上误拉取 CUDA / NVIDIA 巨型依赖。

- **虚拟机 / 主控**：只保留 `validate`，必要时保留 `train`
- **本地开发机**：执行完整 `export`

原因很直接：

- `export` 组包含 `torch / torchvision / torchreid / ultralytics`
- 依赖解析和轮子下载都比较重
- 在虚拟机上一次性跑完整 `export` 容易耗时很长，且中断后可能留下残留 `uv sync` 进程占住 `.venv/.lock`

因此：

- 日常联调、校验 ONNX：`validate`
- ReID 训练链路：`train`
- YOLO / ReID 完整导出：**建议本地机执行 `export`**

补充注意：

- `train` 组在 Linux 上可能会拉取较大的 `torch / torchvision` 轮子；若使用默认 PyPI 源，存在一次下载数 GB 级依赖的情况。

当前已在仓库根目录增加：

- `pyproject.toml`

dependency groups 约定如下：

- `base`：`numpy`、`onnx`、`onnxruntime`
- `yolo`：`ultralytics`
- `reid`：`torch`、`torchvision`、`torchreid`、`scipy`、`opencv-python-headless`、`gdown`、`tensorboard`
- `export`：`base + yolo + reid`
- `train`：`base + reid`
- `validate`：`base`

常用命令：

```bash
uv sync
uv sync --group validate
uv sync --group train
uv sync --group export

uv run --group export python src/smart_follower_perception/scripts/export_yolo_onnx.py --help
uv run --group train python src/smart_follower_perception/scripts/train_reid_resnet50.py --help
uv run --group validate python src/smart_follower_perception/scripts/validate_reid_onnx.py --help
```

工程边界：

- `uv` 只管 Python 工具链，不接管 C++ / ROS2 主工程。
- 现阶段不把 `uv` 嵌进 `colcon build`。
- 目标是先统一本地机、虚拟机、主控的模型工具链依赖，减少 Python 包漂移。
- ROS2 Humble / Python 3.10 环境下，`onnxruntime` 当前约束为 `<1.24`，避免 `uv` 解析到仅支持 cp311+ 的新版本。


### 15.2 export 组教程（建议本地机执行）

如果你要完整跑导出链路，建议在**本地开发机**执行：

```bash
cd /path/to/ros2_smart_follower
uv sync --group export
```

准备好模型或训练权重后：

```bash
# 导出 YOLO ONNX
uv run --group export python src/smart_follower_perception/scripts/export_yolo_onnx.py --help

# 训练 ReID（ResNet50-2048）
uv run --group train python src/smart_follower_perception/scripts/train_reid_resnet50.py --help

# 导出 ReID ONNX
uv run --group export python src/smart_follower_perception/scripts/export_reid_onnx.py --help

# 校验 ReID ONNX
uv run --group validate python src/smart_follower_perception/scripts/validate_reid_onnx.py --help
```

推荐顺序：

1. `uv sync --group export`
2. 导出 `yolo26n.onnx`
3. 训练或准备 `reid_resnet50` 权重
4. 导出 `reid_resnet50_2048.onnx`
5. 用 `validate_reid_onnx.py` 做输入输出形状校验

若目标机没有外网：

- 建议先在本地机完成 `export`
- 最终只把 `models/*.onnx` 带到目标机
- 目标机仅保留 `validate` 或直接跳过 Python 工具链
