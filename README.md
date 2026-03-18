# ROS2 Smart Follower

面向差速小车的人体视觉跟随系统，基于 **ROS 2 Humble + C++17** 实现。

主链路：

**感知（YOLO26n + 跟踪 + 深度 + ReID） → 跟随控制 → 安全避障 → 仲裁输出**

---

## 1. 当前版本

- **当前发布版本：`alpha-0.1.2`**
- **上一版本：`alpha-0.1.1`**
- **上一稳定快照：`alpha-0.0.4`**
- **上一调试快照：`dev-0.0.1.1`**
- **状态：Usable Alpha**

### alpha-0.1.2（2026-03-18）本轮重点

这一版主要补的是 **P0 级稳定性与工程化兜底**：

- ReID ONNX 推理异常已做 `try/catch` 兜底，异常时不会再直接带崩感知节点
- 感知 / 跟随 / 避障 / 仲裁 / 超声波 diagnostics 不再永远显示 OK，改为实际输出 `OK / WARN / ERROR`
- `smart_follower_bringup` 已补 `ament_index_python` 运行依赖，避免 launch 在新环境中 import 失败
- CMake / package.xml 的 lint 接入已从“声明未生效”改为**实际执行**
- 当前 lint 策略采用 **`ament_cmake_lint_cmake + ament_cmake_xmllint`**，先保证工程可持续验收，不把历史风格债一次性混入当前稳定性修复
- 已在虚拟机 `wheeltec@192.168.220.131` 上完成清理后的 build/test 回归，结果为 **37 tests passed, 0 failures**

### alpha-0.1.1（2026-03-17）上一轮重点

- 控制侧抽出公共常量、生命周期工具与 runtime 子模块
- `arbiter / follower / obstacle / ultrasonic` 四条控制子链路的核心逻辑进一步从节点类中剥离
- 统一运行时版本字符串与部分公共语义说明
- 澄清 `TrackedPerson` 与 `PersonPoseArray` 的 2D / 3D 字段语义

### alpha-0.1.0（2026-03-17）结构重构重点

- 感知侧将超大文件 `perception_node.cpp` 拆分为：
  - `runtime`
  - `frame_sync`
  - `tracker`
  - `lock_manager`
  - `geometry_utils`
  - `params / diagnostics`
- 新增 `smart_follower_perception_core` 便于单元测试与后续复用
- 新增感知单元测试：`test_frame_sync`、`test_lock_manager`、`test_tracker`

---

## 2. 项目目标

本项目面向带深度相机与超声波的差速底盘，目标是实现：

- 基于 YOLO26n 的人体检测
- 基于卡尔曼 + 匈牙利匹配 + ReID 的目标保持
- 基于深度图的测距与 3D 坐标转换
- 基于 PID 的机器人跟随控制
- 基于超声波 + 深度距离的安全避障
- 基于状态机的最终速度仲裁
- 显式锁定 / 解锁 / 复位 / 急停控制

默认系统假设：

- 相机输入：`640x480 @ 30fps`
- 感知主处理频率：约 `10Hz`（默认每 3 帧做一次检测）
- 控制输出频率：`20Hz`
- 目标跟随距离：`1.0m`
- 底盘：差速小车

---

## 3. 工程结构

```text
ros2_smart_follower/
├─ src/
│  ├─ smart_follower_msgs/         # 自定义消息
│  ├─ smart_follower_perception/   # 感知链路（YOLO / ReID / 跟踪 / 深度 / 锁定）
│  ├─ smart_follower_control/      # 跟随控制 / 避障 / 仲裁 / 键盘 / 超声波
│  └─ smart_follower_bringup/      # launch 与参数
├─ models/                         # ONNX 模型
├─ scripts/                        # 环境安装与辅助脚本
├─ pyproject.toml                  # Python 工具链（uv）
├─ DEPENDENCIES.md
├─ CHANGELOG.md
└─ README.md
```

---

## 4. 功能架构

### 4.1 感知链路

输入：
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

主要逻辑：
- RGB / Depth / CameraInfo 普通订阅 + 最近帧缓存 + 手工时间戳匹配
- YOLO26n 低频检测，仅保留 `person`
- ReID 使用 ResNet50-2048 ONNX
- 跟踪器融合 IoU / 中心距离 / 深度差 / 外观代价
- 深度采用中心 `5x5` 中值采样，单位 `mm -> m`
- 坐标使用 `tf2` 转到 `base_footprint`
- 输出全目标列表 + 当前锁定 ID

输出：
- `/<robot_ns>/person_pose`

### 4.2 控制链路

- `follower_controller_node`：20Hz 输出 `/cmd_vel_follow`
- `obstacle_avoidance_node`：20Hz 输出 `/cmd_vel_avoid`
- `arbiter_node`：20Hz 持续发布最终 `/cmd_vel`
- `keyboard_command_node`：发布锁定 / 解锁 / 复位 / 急停命令
- `ultrasonic_range_node`：左右前超声波测距，支持 dry mode

---

## 5. 消息语义说明

### `TrackedPerson.msg`

- `bbox`：**彩色图像像素坐标系** 下的 2D 框
- `position / velocity`：`PersonPoseArray.header.frame_id` 对应坐标系下的 **3D 信息**
- `appearance_feature`：当前版本为 **2048 维**

### `PersonPoseArray.msg`

- `header.stamp`：原始图像时间戳
- `header.frame_id`：**只约束 3D 字段**（如 `position / velocity`）
- `lock_id`：当前锁定目标 ID，未锁定时为 `-1`

> 注意：`header.frame_id` 不约束 `bbox`，因为 `bbox` 始终是图像像素坐标。

---

## 6. 模型约定

默认模型路径：

- YOLO：`models/yolo26n.onnx`
- ReID：`models/reid_resnet50_2048.onnx`

当前 ReID 约定：

- 输入尺寸：`128 x 256`（W x H）
- 输出维度：`2048`
- 预处理：RGB + resize + ImageNet mean/std 归一化

---

## 7. 依赖

### 7.1 ROS 2 / C++ 运行时

- ROS 2 Humble
- OpenCV
- Eigen3
- ONNX Runtime C++ SDK
- tf2 / tf2_ros / tf2_geometry_msgs
- diagnostic_updater
- libgpiod（超声波 GPIO 后端；不可用时自动 dry mode）

### 7.2 Python 工具链

本项目保留 `uv` 管理的 Python 工具链，主要用于：

- YOLO ONNX 导出
- ReID 训练 / 导出 / 校验

常见命令：

```bash
uv sync
uv run --group validate python src/smart_follower_perception/scripts/validate_reid_onnx.py --help
```

> 当前推荐：训练与校验可以在目标机或虚拟机执行；完整导出链路更推荐在本地开发机执行。

---

## 8. 构建与测试

### 8.1 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 8.2 测试

```bash
source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

### 8.3 当前 lint 策略

当前版本实际启用：

- `ament_cmake_lint_cmake`
- `ament_cmake_xmllint`

说明：
- 之前仓库中存在较多历史 `cpplint / flake8 / uncrustify` 风格债
- `alpha-0.1.2` 先保证稳定性修复、诊断修复和验收链路是绿色的
- 后续再单独开一轮风格清理，不把两类问题混在一起

---

## 9. 启动

### 9.1 仅启动本系统

```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py
```

### 9.2 联合底盘与相机启动

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

说明：
- 系统内部话题使用命名空间
- 最终底盘控制输出保持全局 `/cmd_vel`
- 默认 `robot_ns` 一般为 `robot1`

---

## 10. diagnostics 与降级行为

### 感知侧

- YOLO 未 ready：`ERROR`
- ReID 未 ready：`WARN`
- color / depth / camera_info 长时间无输入：`WARN / ERROR`
- `person_pose` 长时间未发布：`WARN / ERROR`

### 控制侧

- follower 未收到目标或目标超时：`WARN / ERROR`
- obstacle 输入全部失效：`ERROR`
- ultrasonic dry mode：`WARN`
- arbiter stop latched：`ERROR`

---

## 11. 已验证环境

### 虚拟机
- 主机：`wheeltec@192.168.220.131`
- 结果：`alpha-0.1.2` 已完成构建与测试回归
- 本轮结果：**37 tests passed, 0 failures**

### 小车主控容器
- 用途：部署与运行
- 当前建议：开发在本地机 / 虚拟机进行，主控容器只做部署、编译与联调

---

## 12. 版本演进

- `alpha-0.0.1`：首个 Alpha 主链路落地
- `alpha-0.0.1.1`：ReID 切换到 ResNet50-2048
- `alpha-0.0.1.2`：感知同步切换为手工缓存同步
- `alpha-0.0.2`：首个可用版本，修复 D2C、热更新、超声波与仲裁细节
- `alpha-0.0.3`：稳定性与动态参数热更新补强
- `alpha-0.0.4`：引入 uv CPU-only Python 工具链
- `alpha-0.1.0`：感知结构重构 + 单元测试补齐
- `alpha-0.1.1`：控制侧公共骨架、版本统一、消息语义澄清
- `alpha-0.1.2`：P0 稳定性修复、diagnostics 分级、bringup 依赖补齐、lint 实接入

---

## 13. 后续建议

下一轮建议优先级：

1. 补 `ultrasonic_runtime` 单元测试
2. 继续统一控制侧大节点的参数与生命周期样板
3. 单独开一轮 `cpplint / flake8 / uncrustify` 风格债清理
4. 再做感知性能 profiling 与异步化优化
