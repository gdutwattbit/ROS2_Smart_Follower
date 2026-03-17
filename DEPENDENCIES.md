# ros2_smart_follower 依赖清单

本文档基于以下文件整理：

- `src/*/package.xml`
- `src/*/CMakeLists.txt`
- `src/smart_follower_bringup/launch/*.launch.py`
- `src/smart_follower_bringup/config/perception_params.yaml`
- `src/smart_follower_control/config/control_params.yaml`
- `src/smart_follower_perception/scripts/*.py`

## 1. 目标环境

- Ubuntu 22.04
- ROS 2 Humble
- C++17

## 2. 必需依赖

### 2.1 系统基础工具

- `build-essential`
- `cmake`
- `git`
- `curl`
- `wget`
- `unzip`
- `pkg-config`
- `python3-pip`
- `python3-rosdep`
- `python3-colcon-common-extensions`
- `python3-vcstool`

### 2.2 C/C++ 三方库

- `libopencv-dev`
- `libeigen3-dev`

### 2.3 ROS 2 依赖

来自 `package.xml` / `CMakeLists.txt` 的核心依赖：

- `ament_cmake`
- `rosidl_default_generators`
- `rclcpp`
- `rclcpp_lifecycle`
- `lifecycle_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `cv_bridge`
- `image_transport`
- `image_geometry`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `diagnostic_updater`
- `diagnostic_msgs`
- `launch`
- `launch_ros`

### 2.4 从 launch 文件额外识别出的依赖

这些依赖没有完整体现在所有 `package.xml` 中，但代码运行会用到：

- `ament_index_python`

## 3. 可选但强烈建议的依赖

### 3.1 推理运行时

- `ONNX Runtime C++`  
  用于 `smart_follower_perception/perception_node` 的真实 YOLO / ReID 推理。  
  若缺失，工程仍可编译，但会进入 `stub inference` 模式。

可用方式：

- 系统包：`libonnxruntime-dev`（若发行版仓库可用）
- 或放到 `third_party/onnxruntime*`
- 或设置环境变量 `ONNXRUNTIME_ROOT`

### 3.2 超声波硬件支持

- `libgpiod`
- `libgpiod-dev`

若缺失，`ultrasonic_range_node` 可编译，但会进入 dry mode。Pi 5 主控当前使用 libgpiod 路线。

## 4. 可选 Python 依赖（模型训练 / 导出 / 校验）

对应 `src/smart_follower_perception/scripts/*.py`：

- `numpy`
- `onnx`
- `onnxruntime`
- `torch`
- `torchvision`
- `torchreid`
- `scipy`
- `opencv-python-headless`
- `gdown`
- `tensorboard`
- `ultralytics`

> 这部分不是运行 ROS 节点的硬性依赖，而是模型训练、导出、验证工具链依赖。

## 5. 仓库外依赖 / 资源

### 5.1 外部 ROS 包

- `turn_on_wheeltec_robot`

说明：

- `smart_follower.launch.py` 依赖该包
- 如果你只想启动本项目自己的节点，可使用：
  - `smart_follower_only.launch.py`

### 5.2 模型文件

运行前建议准备：

- `models/yolo26n.onnx`
- `models/reid_resnet50_2048.onnx`

当前仓库内未包含实际模型文件。

## 6. 一键安装脚本

已生成：

- `scripts/install_dependencies.sh`

默认行为：

- 安装基础工具链
- 安装 OpenCV / Eigen / libgpiod
- 自动执行 `rosdep install`
- 跳过无法通过 rosdep 直接安装的 `turn_on_wheeltec_robot`
- 若仓库中没有模型文件，则给出提示

可选参数：

- `--with-training`：额外安装 Python 训练/导出工具链
- `--without-pigpio`：跳过超声波相关依赖
- `--without-onnxruntime`：跳过 ONNX Runtime 安装尝试
- `--install-ros`：若本机 apt 源已配置 ROS 仓库，则尝试安装 `ros-humble-desktop`



## 7. Python 工具链的 uv 管理

为避免模型训练 / 导出 / 校验脚本的 Python 环境在本地机、虚拟机、主控间漂移，仓库根目录已增加：

- `pyproject.toml`

当前约定：

- `colcon + CMake`：管理 ROS2 / C++ 主工程
- `uv`：只管理 Python 工具链，不接管 ROS2 运行时

### 7.1 dependency groups

- `base`
  - `numpy`
  - `onnx`
  - `onnxruntime`
- `yolo`
  - `ultralytics`
- `reid`
  - `torch`
  - `torchvision`
  - `torchreid`
- `scipy`
- `opencv-python-headless`
- `gdown`
- `tensorboard`
- `export`
  - 包含 `base + yolo + reid`
- `train`
  - 包含 `base + reid`
- `validate`
  - 包含 `base`

### 7.2 常用命令

```bash
# 初始化空环境（默认不拉重型组）
uv sync

# 安装最小 ONNX 校验环境
uv sync --group validate

# 安装 ReID 训练环境
uv sync --group train

# 安装完整导出环境
uv sync --group export

# 直接运行脚本
uv run --group export python src/smart_follower_perception/scripts/export_yolo_onnx.py --help
uv run --group train python src/smart_follower_perception/scripts/train_reid_resnet50.py --help
uv run --group validate python src/smart_follower_perception/scripts/validate_reid_onnx.py --help
```

### 7.3 当前边界

- 暂不把 `uv` 嵌进 `colcon build`。
- 暂不为 ROS2 Python 节点单独打 wheel / 包发布。
- 现阶段只服务于模型工具链，先保证简单、稳定、可复现。


补充说明：ROS2 Humble 默认 Python 3.10，当前 `onnxruntime` 建议约束为 `<1.24`，否则 `uv` 可能解析到仅支持 cp311+ 的新版本。


当前建议按“方案 B”使用：虚拟机 / 主控以 `validate`、`train` 为主；完整 `export` 建议放在本地开发机执行。

补充说明：`train` 组若直接走默认 PyPI 源，Linux 上可能会拉取较大的 PyTorch 相关轮子，体积可达数 GB。

补充说明：`uv` 中的 `torch`、`torchvision` 已固定到官方 PyTorch CPU-only index（https://download.pytorch.org/whl/cpu），避免目标机误装 CUDA 相关轮子。
