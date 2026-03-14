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

- `pigpio`
- `libpigpio-dev`

若缺失，`ultrasonic_range_node` 可编译，但会进入 dry mode。

## 4. 可选 Python 依赖（模型训练 / 导出 / 校验）

对应 `src/smart_follower_perception/scripts/*.py`：

- `numpy`
- `onnx`
- `onnxruntime`
- `torch`
- `torchvision`
- `torchreid`
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
- 安装 OpenCV / Eigen / pigpio
- 自动执行 `rosdep install`
- 跳过无法通过 rosdep 直接安装的 `turn_on_wheeltec_robot`
- 若仓库中没有模型文件，则给出提示

可选参数：

- `--with-training`：额外安装 Python 训练/导出工具链
- `--without-pigpio`：跳过超声波相关依赖
- `--without-onnxruntime`：跳过 ONNX Runtime 安装尝试
- `--install-ros`：若本机 apt 源已配置 ROS 仓库，则尝试安装 `ros-humble-desktop`

