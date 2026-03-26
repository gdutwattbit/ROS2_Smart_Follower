# ROS2 Smart Follower

面向树莓派 / ROS 2 Humble 的低位智能跟随车项目。

当前固定技术路线：
- 感知：YOLO + ReID + Tracker + Lock Manager
- 定位：Astra 彩色 + 深度输入，depth compare 主链路
- 控制：20Hz 跟随控制 + 短时预测补帧 + 超声波避障 + 指令仲裁

> 当前发布标签：`beta-0.2.0`

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
- 使用轻量模型组合：`yolo26n_static_256x320_simplify_e2e.onnx + osnet_x0_5_512.onnx`
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
├─ test.md
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
- `models/yolo26n_static_256x320_simplify_e2e.onnx`
- `models/osnet_x0_5_512.onnx`

当前推荐线程参数：
- YOLO ORT: `intra_op_num_threads=1`, `inter_op_num_threads=1`, `sequential`
- ReID ORT: `intra_op_num_threads=1`, `inter_op_num_threads=1`, `sequential`

当前推荐节奏：
- 相机输入：约 30fps
- perception 处理：`process_every_n_frames=3`
- follower 控制输出：20Hz
- 中间依靠控制侧短时常速度预测补帧
- follower 默认目标距离：`0.6m`

---

## 5. 启动方式

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
- `test.md`：测试记录与阶段性实验结论

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
