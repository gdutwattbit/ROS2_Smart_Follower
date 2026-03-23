# ROS2 Smart Follower

面向树莓派 / ROS 2 Humble 的低位智能跟随车项目。

当前主线能力：
- 感知侧：YOLO + ReID + 多目标跟踪 + 锁定策略
- 定位侧：**纯 RGB 单目位置估计**（Astra 内参 + bbox 底点地面投影）
- 控制侧：20Hz 跟随控制、短时预测补帧、超声波避障、速度仲裁

> 当前基线版本：`alpha-0.1.7`
> 
> 当前主线形态：**纯 RGB + 单目位置估计 + 左右超声波避障**

---

## 1. 当前链路概览

```text
/camera/color/image_raw
        ↓
      YOLO
        ↓
      ReID
        ↓
     Tracker
        ↓
   Lock Manager
        ↓
单目位置估计（bbox 底点地面投影）
        ↓
    /person_pose
        ↓
Follower / Obstacle / Arbiter
        ↓
      /cmd_vel
```

当前版本的重要设计点：
- perception 主链路只消费彩色图，不再依赖 depth 主链路
- 单目定位优先调用 Astra 的 `/camera/get_camera_info` 获取真实内参
- 若服务不可用，则自动退回到 `horizontal_fov_deg` 的 fallback 近似内参
- `TrackedPerson.position` 为 base frame 下的位置估计结果
- `TrackedPerson.depth_m` 为兼容字段，当前纯 RGB 模式下填 `NaN`

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
- 彩色图像接入
- YOLO 检测
- ReID 特征提取
- 多目标跟踪
- 目标锁定 / 切人策略
- 单目位置估计
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
- `models/yolo26n_static_480x640_simplify_e2e.onnx`
- `models/osnet_x0_5_512.onnx`

当前推荐线程参数：
- YOLO ORT: `intra_op_num_threads=3`, `inter_op_num_threads=1`, `sequential`
- ReID ORT: `intra_op_num_threads=1`, `inter_op_num_threads=1`, `sequential`

当前推荐节奏：
- 相机输入：约 30fps
- perception 处理：`process_every_n_frames=3`，约 10Hz
- follower 控制输出：20Hz
- 中间依靠控制侧短时常速度预测补帧

---

## 5. 启动方式

### 仅启动跟随链路
```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py
```

### 启动完整 bringup
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

常用 launch 参数：
- `robot_ns:=robot1`
- `bringup_robot:=true|false`
- `bringup_camera:=true|false`

---

## 6. 当前值得优先阅读的文件

如果是新同学接手，建议先看：
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

## 7. alpha-0.1.7 重点变更

- 单目位置估计从“bbox 高度 + 人高假设”升级为：
  - **Astra 内参 + bbox 底点地面投影**
- perception 节点在 configure / 热更新时：
  - 优先调用 `/camera/get_camera_info`
  - 失败时自动 fallback 到近似内参
- diagnostics 新增：
  - `intrinsics_ready`
  - `intrinsics_source`
  - `camera_fx/fy/cx/cy`
  - `position_projection_ms`
  - `position_valid_count / position_invalid_count`
- 新增 `test_geometry_utils.cpp`
- 新增根目录 `try.md` 调参说明

---

## 8. 当前状态

当前工作区已经完成并验证：
- VM 端重新部署整个项目工作区
- `smart_follower_msgs + smart_follower_perception + smart_follower_control + smart_follower_bringup` 编译通过
- 相关测试通过：`39 tests, 0 errors, 0 failures, 0 skipped`

---

## 9. 下一步建议

建议优先做这三件事：
1. 实车标定：
   - `camera_height_m`
   - `camera_pitch_deg`
   - `camera_x_offset_m`
   - `camera_y_offset_m`
2. 在真实跟随场景验证 `/person_pose` 的距离与左右偏移稳定性
3. 再决定是否继续推进更激进的推理后端优化
