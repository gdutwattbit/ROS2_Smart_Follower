# ROS2 Smart Follower 安装指南

本文档提供详细的环境配置和依赖安装步骤。

---

## 1. 目标环境

- **操作系统**：Ubuntu 22.04 LTS
- **ROS 版本**：ROS 2 Humble
- **编译器**：GCC 11+ (C++17)
- **Python**：Python 3.10+

---

## 2. 系统依赖安装

### 2.1 更新系统包管理器

```bash
sudo apt update
sudo apt upgrade -y
```

### 2.2 安装基础工具

```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  unzip \
  pkg-config \
  python3-pip \
  python3-dev \
  python3-venv
```

---

## 3. OpenCV 安装

### 方案一：从 apt 仓库安装（快速，推荐新手）

**优点**：快速、易维护
**缺点**：版本可能不是最新

```bash
sudo apt install -y libopencv-dev python3-opencv

# 验证安装
python3 -c "import cv2; print(cv2.__version__)"
```

### 方案二：从源码编译（推荐用于生产环境）

#### 步骤 1：下载 OpenCV 源码

```bash
cd ~/workspace
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.8.1.zip
unzip opencv.zip
cd opencv-4.8.1
```

#### 步骤 2：安装必要的依赖

```bash
sudo apt install -y \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libv4l-dev \
  libxvidcore-dev \
  libx264-dev \
  libgtk-3-dev \
  libatlas-base-dev \
  gfortran \
  libeigen3-dev
```

#### 步骤 3：创建编译目录并配置

```bash
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D BUILD_PYTHON_SUPPORT=ON \
  -D PYTHON3_EXECUTABLE=$(which python3) \
  -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_TESTS=OFF \
  ..
```

#### 步骤 4：编译并安装

```bash
# 使用多线程加速编译（根据 CPU 核心数调整）
make -j$(nproc)

# 安装
sudo make install

# 配置库路径
sudo ldconfig
```

#### 步骤 5：验证安装

```bash
python3 -c "import cv2; print('OpenCV 版本:', cv2.__version__)"
pkg-config --modversion opencv4
```

---

## 4. ONNX Runtime 安装（C++）

本项目 C++ 节点推荐使用 ONNX Runtime C/C++ 预编译包（SDK）方式安装。

### 4.1 下载 C++ 预编译包（CPU 版）

```bash
cd ~/workspace

# 按需替换版本号
ORT_VERSION=1.20.1
wget https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz
tar -xzf onnxruntime-linux-x64-${ORT_VERSION}.tgz

# 安装到系统目录（推荐）
sudo mv onnxruntime-linux-x64-${ORT_VERSION} /opt/onnxruntime
```

### 4.2 配置动态库路径

```bash
echo "/opt/onnxruntime/lib" | sudo tee /etc/ld.so.conf.d/onnxruntime.conf
sudo ldconfig

# 验证动态库是否可见
ldconfig -p | grep onnxruntime
```

### 4.3 CMake 集成示例

在 CMake 项目中可按如下方式链接：

```cmake
cmake_minimum_required(VERSION 3.10)
project(ort_cpp_demo)

set(CMAKE_CXX_STANDARD 17)

add_executable(ort_cpp_demo main.cpp)
target_include_directories(ort_cpp_demo PRIVATE /opt/onnxruntime/include)
target_link_libraries(ort_cpp_demo PRIVATE /opt/onnxruntime/lib/libonnxruntime.so)
```

### 4.4 C++ 最小验证程序

```cpp
#include <iostream>
#include <onnxruntime_cxx_api.h>

int main() {
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ort_cpp_check");
  std::cout << "ONNX Runtime C++ 初始化成功" << std::endl;
  return 0;
}
```

编译与运行：

```bash
g++ -std=c++17 main.cpp \
  -I/opt/onnxruntime/include \
  -L/opt/onnxruntime/lib -lonnxruntime \
  -Wl,-rpath,/opt/onnxruntime/lib \
  -o ort_cpp_check

./ort_cpp_check
```

### 4.5 GPU 版本说明（可选）

如需 CUDA 加速，请下载与本机 CUDA/cuDNN 匹配的 ONNX Runtime GPU C++ 包，并沿用同样的 include/link 方式。

发布页：<https://github.com/microsoft/onnxruntime/releases>

### 4.6 从源码编译（高级用户）

如需自定义编译选项（TensorRT、最小化算子等），请参考官方构建文档：
<https://github.com/microsoft/onnxruntime/blob/main/BUILD.md>

---

## 5. ROS 2 Humble 安装

### 步骤 1：设置 ROS 2 apt 仓库

```bash
# 安装必要工具
sudo apt install -y software-properties-common

# 添加 ROS 2 仓库密钥
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加仓库源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros2.org/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新包列表
sudo apt update
```

### 步骤 2：安装 ROS 2 Humble

```bash
sudo apt install -y ros-humble-desktop

# 安装开发工具
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-ament-cmake \
  ros-humble-rosidl-default-generators
```

### 步骤 3：初始化 rosdep

```bash
sudo rosdep init
rosdep update
```

### 步骤 4：配置环境

```bash
# 添加到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 验证
ros2 --version
```

---

## 6. Python 依赖安装

### 6.1 创建虚拟环境（可选但推荐）

```bash
# 在项目目录创建
python3 -m venv venv

# 激活虚拟环境
source venv/bin/activate
```

### 6.2 安装项目依赖

#### 方法一：使用 pip 直接安装

```bash
pip3 install -U pip

# 安装必要库
pip3 install \
  numpy \
  scipy \
  scikit-learn \
  matplotlib \
  Pillow \
  pyyaml \
  lark
```

#### 方法二：使用 requirements.txt（如存在）

```bash
# 如果项目提供了 requirements.txt
pip3 install -r requirements.txt
```

---

## 7. 项目构建与编译

### 7.1 安装系统侧 ROS 2 依赖

在项目根目录执行：

```bash
cd ~/ROS2_Smart_Follower

# 使用 rosdep 自动安装依赖
rosdep install --from-paths src -i -y --rosdistro humble
```

### 7.2 编译项目

```bash
# 使用 colcon 编译
colcon build --symlink-install --parallel-workers $(nproc)

# 或者只编译特定包
colcon build --packages-select smart_follower_perception smart_follower_control
```

### 7.3 设置环境变量

```bash
# 编译完成后，设置环境
source install/setup.bash

# 验证
ros2 pkg list | grep smart_follower
```

---

## 8. 验证安装完整性

### 8.1 OpenCV（C++）验证

```bash
pkg-config --modversion opencv4
```

### 8.2 ONNX Runtime（C++）验证

```bash
ldconfig -p | grep onnxruntime
```

如果上面命令有输出，再执行第 4.4 节的最小 C++ 程序编译与运行，即可完成 ONNX Runtime C++ 侧验证。

### 8.3 ROS 2 环境验证

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_Smart_Follower/install/setup.bash
ros2 pkg list | grep smart_follower
```

---

## 9. 常见问题排查

### Q1：OpenCV 找不到依赖库

```bash
# 重新配置库路径
sudo ldconfig -v
```

### Q2：ONNX Runtime C++ 运行时报找不到 `libonnxruntime.so`

```bash
# 检查库文件
ls -l /opt/onnxruntime/lib/libonnxruntime.so

# 刷新动态链接缓存
sudo ldconfig

# 临时验证（仅当前终端）
export LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH
```

### Q3：ROS 2 命令找不到

```bash
# 确保环境变量已设置
source /opt/ros/humble/setup.bash
source ~/ROS2_Smart_Follower/install/setup.bash
```

### Q4：编译时缺少头文件

```bash
# 重新运行 rosdep
rosdep install --from-paths src -i -y --rosdistro humble
```

---

## 10. 性能优化建议

### CPU 优化

```bash
# 编译时启用优化
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### GPU 优化（如有 NVIDIA GPU）

```bash
# 检查 CUDA 环境
nvidia-smi
nvcc --version

# 若使用 GPU 版 ONNX Runtime，请确认其与 CUDA/cuDNN 版本匹配
```

---

## 11. 后续步骤

安装完成后，请参考：

1. [新人上手指南](./docs/新人上手指南.md) - 快速了解项目框架
2. [依赖清单](./DEPENDENCIES.md) - 详细的依赖说明
3. 各包内 README：
   - `src/smart_follower_perception/README.md`
   - `src/smart_follower_control/README.md`

---

## 12. 支持与反馈

如遇到问题，请检查：

- 操作系统版本：`lsb_release -a`
- Python 版本：`python3 --version`
- ROS 2 版本：`ros2 --version`
- 编译日志：`cat log/latest_build/*/stdout.log`

