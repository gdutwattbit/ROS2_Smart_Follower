# ROS2 Smart Follower

面向树莓派 / ROS 2 Humble 的低位智能跟随车项目。

当前固定技术路线：
- 感知：YOLO + ReID + Tracker + Lock Manager
- 定位：Astra 彩色 + 深度输入，depth compare 主链路
- 控制：20Hz 跟随控制 + 短时预测补帧 + 超声波避障 + 指令仲裁

> 当前发布标签：`beta-0.3.1`

---

## 0. beta-0.3.1 本轮更新

这一轮主线改动已经收口到 **beta-0.3.1**，重点不是再扩功能，而是把实车链路补稳、补清楚：

- 默认 YOLO 模型切换为 `models/yolo26n_static_256x320_simplify_e2e_int8.onnx`
- perception 默认节奏调整为 `process_every_n_frames=2`
- `depth_compare` 默认采样窗口调整为 `9x9`，`min_valid_samples=5`
- 锁定目标深度定位改为 **lower-body 多窗口采样 + median**，不再保留旧单窗兼容链路
- 跟踪模块增加异常打印与 `tracker.reset()` 保护，异常时不再直接把整条感知链路打崩
- follower 控制默认 `target_timeout` 提升到 `1.2s`，短时深度抖动时更容易续上
- perception / follower 补齐了更直接的失效原因日志，便于判断是 `target_timeout`、`locked_track_position_nan`、`bbox_near_image_edge` 还是 `depth_window_no_valid_samples`
- 清理旧技术路线遗留文档、模型和说明，只保留当前 Astra color+depth 主线需要的仓库内容
- perception 构建阶段改为强制依赖 `astra_camera_msgs`，不再保留缺包时的旧兼容分支
- 历史 `monocular.*` 命名已统一收口为 `camera.*`，参数、类型名和内部变量与当前 RGB-D 主线保持一致
- 小车容器内第三方依赖路径、编译方式和调试面板依赖已补齐到仓库文档

---

## 1. 当前链路概览

```text
/camera/color/image_raw + /camera/depth/image_raw
                    ↓
                  YOLO
                    ↓
                  ReID
                    ↓
                 Tracker
                    ↓
               Lock Manager
                    ↓
      depth compare 定位（bbox 下半部窗口 + median）
                    ↓
                /person_pose
                    ↓
Follower / Obstacle / Arbiter
                    ↓
                  /cmd_vel
```

当前设计要点：
- 使用轻量模型组合：`yolo26n_static_256x320_simplify_e2e_int8.onnx + osnet_x0_5_512.onnx`
- `TrackedPerson.position` 由对齐深度图取样得到
- `/person_pose`、控制侧接口、生命周期行为保持稳定
- perception 会请求 Astra 的 `/camera/get_camera_info` 作为真实内参来源

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
└─ try.md
```

---

## 3. 各包职责

### `smart_follower_msgs`
定义项目消息：
- `TrackedPerson.msg`
- `PersonPoseArray.msg`
- `FollowCommand.msg`

### `smart_follower_perception`
负责：
- 彩色图像 + 深度图接入
- YOLO 检测
- ReID 特征提取
- 多目标跟踪
- 目标锁定 / 切人策略
- depth compare 定位
- 发布 `/person_pose`

### `smart_follower_control`
负责：
- 跟随控制
- 20Hz 控制补帧预测
- 左右超声波采样
- 超声波避障
- 指令仲裁
- 键盘控制

### `smart_follower_bringup`
负责：
- launch 组织
- 默认 YAML 参数
- 模型路径覆盖

---

## 4. 当前推荐模型与运行组合

默认模型组合：
- `models/yolo26n_static_256x320_simplify_e2e_int8.onnx`
- `models/osnet_x0_5_512.onnx`

当前推荐线程参数：
- YOLO ORT: `intra_op_num_threads=1`, `inter_op_num_threads=1`, `sequential`
- ReID ORT: `intra_op_num_threads=1`, `inter_op_num_threads=1`, `sequential`

当前推荐节奏：
- 相机输入：约 30fps
- perception 处理：`process_every_n_frames=2`
- follower 控制输出：20Hz
- 中间依靠控制侧短时常速度预测补帧
- follower 默认目标距离：`0.6m`

---

## 5. 启动方式

### 小车容器内第三方依赖实际路径

在小车 `ros2` 容器内实查到：

- **ONNX Runtime 根目录**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3`
- **ONNX Runtime 头文件**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/include`
- **ONNX Runtime 动态库**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/lib/libonnxruntime.so`
- **ONNX Runtime CMake 配置**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/lib/cmake/onnxruntime`
- **ONNX Runtime pkg-config**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/lib/pkgconfig/libonnxruntime.pc`

- **libgpiod 安装根目录**
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod`
- **libgpiod 头文件**
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod/include/gpiod.h`
- **libgpiod 动态库**
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod/lib/libgpiod.so`
- **libgpiod 源码目录**
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod-2.1.3`

> 注意：这两套库目前在容器里是存在的，但**不在本仓库的 `third_party/` 下**。
> 当前主线代码已经同时补上：
> - 显式绝对路径 `/home/wheeltec/wheeltec_ros2/third_party/...`
> - 环境变量路径 `$HOME/wheeltec_ros2/third_party/...`
> 这样即使容器里用 `root` 编译，也不会再因为 `$HOME=/root` 而漏检。

如需在小车容器内显式指定依赖路径，建议先执行：

```bash
export ONNXRUNTIME_ROOT=/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3
export LIBGPIOD_ROOT=/home/wheeltec/wheeltec_ros2/third_party/libgpiod
export LD_LIBRARY_PATH=$ONNXRUNTIME_ROOT/lib:$LIBGPIOD_ROOT/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$ONNXRUNTIME_ROOT/lib/pkgconfig:$PKG_CONFIG_PATH
```

### 启动完整 bringup
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

### 没有底盘驱动包时，仅启动本项目链路
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 \
  bringup_robot:=false
```

---

## 6. 当前值得优先阅读的文件

建议先看：
- `src/smart_follower_bringup/config/perception_params.yaml`
- `src/smart_follower_control/config/control_params.yaml`
- `src/smart_follower_perception/src/perception_node.cpp`
- `src/smart_follower_perception/src/perception_pipeline.cpp`
- `src/smart_follower_perception/src/tracker.cpp`
- `src/smart_follower_control/src/follower_runtime.cpp`
- `src/smart_follower_control/src/obstacle_runtime.cpp`
- `try.md`

其中：
- `try.md`：参数作用 / 单位 / 调参建议

---

## 7. 当前状态

当前主线已经完成并验证过：
- `smart_follower_msgs`
- `smart_follower_perception`
- `smart_follower_control`
- `smart_follower_bringup`

后续工作重点将转向：
- 路线收敛清理
- 参数整理
- 实车调参与稳定性验证
