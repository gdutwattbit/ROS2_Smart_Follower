# ros2_smart_follower 依赖清单

本文档按 **beta-0.2.0 当前技术路线** 整理，只保留当前运行主线需要的依赖：
- Astra 彩色 + 深度输入
- YOLO + ReID + Tracker + Lock Manager
- depth compare 定位
- follower / obstacle / arbiter 控制链路

---

## 1. 目标环境

- Ubuntu 22.04
- ROS 2 Humble
- C++17

---

## 2. 必需系统依赖

### 2.1 基础工具

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

来自 `package.xml` / `CMakeLists.txt` / launch 的主线依赖：

- `ament_cmake`
- `rosidl_default_generators`
- `rclcpp`
- `rclcpp_lifecycle`
- `lifecycle_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `cv_bridge`
- `diagnostic_updater`
- `diagnostic_msgs`
- `launch`
- `launch_ros`
- `ament_index_python`

---

## 3. 强烈建议安装的运行依赖

### 3.1 ONNX Runtime C++

用于 `smart_follower_perception/perception_node` 的真实 YOLO / ReID 推理。

可选安装方式：
- 系统包：`libonnxruntime-dev`
- 或放到 `third_party/onnxruntime*`
- 或设置环境变量：`ONNXRUNTIME_ROOT`

若缺失：
- 工程仍可编译
- 但感知节点会退化为 `stub inference`

### 3.2 超声波 GPIO 依赖

- `libgpiod2`
- `libgpiod-dev`
- `gpiod`

若缺失：
- `ultrasonic_range_node` 仍可编译
- 但会进入 dry mode

---

## 4. 仓库外依赖 / 外部 ROS 包

### 4.1 小车底盘包

- `turn_on_wheeltec_robot`

说明：
- `smart_follower.launch.py` 默认会 include 该包
- 若当前机器没有该包，可这样启动本项目主链路：

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 \
  bringup_robot:=false
```

### 4.2 Astra 相机包

- `astra_camera`
- `astra_camera_msgs`

说明：
- 当前主线依赖彩色图、深度图、`/camera/get_camera_info`
- `perception_node` 在 configure / 热更新时会请求真实相机内参

---

## 5. 模型文件

当前主线默认模型：

- `models/yolo26n_static_256x320_simplify_e2e.onnx`
- `models/osnet_x0_5_512.onnx`

说明：
- 仓库通常不直接提交大模型文件
- 启动前请确认这两个文件已放入 `models/`

---

## 6. 一键安装脚本

仓库内提供：

- `scripts/install_dependencies.sh`

它当前会做这些事：
- 安装基础工具链
- 安装 OpenCV / Eigen
- 按需安装 libgpiod
- 按需尝试安装 ONNX Runtime
- 初始化并执行 `rosdep install`

常用示例：

```bash
bash scripts/install_dependencies.sh
bash scripts/install_dependencies.sh --without-gpio
bash scripts/install_dependencies.sh --without-onnxruntime
```

---

## 7. 构建与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch smart_follower_bringup smart_follower.launch.py robot_ns:=robot1
```

若当前机器没有底盘驱动包：

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 \
  bringup_robot:=false
```
