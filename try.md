# Smart Follower 调参说明（try.md）

本文只面向 **beta-0.2.0 当前固定技术路线**：
- Astra `color + depth`
- `/camera/get_camera_info` 真实内参
- YOLO + ReID + Tracker + Lock Manager
- `depth_compare` 主定位
- follower / obstacle / arbiter 控制链

当前权威来源：
- `src/smart_follower_bringup/config/perception_params.yaml`
- `src/smart_follower_control/config/control_params.yaml`
- `src/smart_follower_bringup/launch/smart_follower.launch.py`

> 说明
> - 默认命名空间：`robot1`
> - launch 会覆盖模型路径，因此运行时最终模型以 launch 为准
> - 角度带 `deg` 为度，带 `rad` 为弧度，时间带 `s/sec` 为秒，距离默认单位为米
> - 本文不再描述旧纯 RGB / 单目 fallback 路线

---

## 1. 当前默认运行组合

### 1.1 模型
- YOLO：`models/yolo26n_static_256x320_simplify_e2e.onnx`
- ReID：`models/osnet_x0_5_512.onnx`

### 1.2 输入与节奏
- color：`/camera/color/image_raw`
- depth：`/camera/depth/image_raw`
- camera info service：`/camera/get_camera_info`
- 相机典型输入：`640x480 @ 30fps`
- 感知处理：`process_every_n_frames=3`，约 `10Hz`
- follower 控制：`20Hz`
- 超声波：`10Hz`
- 避障 / 仲裁：`20Hz`

### 1.3 当前推荐线程
```yaml
yolo:
  ort:
    intra_op_num_threads: 1
    inter_op_num_threads: 1
    execution_mode: sequential

reid:
  ort:
    intra_op_num_threads: 1
    inter_op_num_threads: 1
    execution_mode: sequential
```

---

## 2. launch 层说明

### 2.1 当前唯一正式入口：`smart_follower.launch.py`
路径：`src/smart_follower_bringup/launch/smart_follower.launch.py`

作用：
- 可拉起底盘相关节点
- 可拉起 Astra 相机
- 再拉起 perception + control 全链路

关键 launch 参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `robot_ns` | `robot1` | 机器人命名空间 |
| `bringup_robot` | `true` | 是否拉起车体底层 |
| `bringup_camera` | `true` | 是否拉起相机驱动 |
| `camera_color_qos` | `sensor_data` | color 话题 QoS |
| `camera_enable_depth` | `true` | 是否打开 depth |
| `camera_enable_point_cloud` | `false` | 是否打开点云 |
| `camera_depth_registration` | `true` | 是否打开 depth registration |
| `camera_enable_d2c_viewer` | `false` | 是否打开 d2c viewer |
| `camera_enable_ir` | `false` | 是否打开 IR |

常用命令：

```bash
# 完整 bringup
ros2 launch smart_follower_bringup smart_follower.launch.py

# 没有底盘包时，只起本项目链路
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 \
  bringup_robot:=false

# 没有相机时，只起本项目链路并自行注入测试数据
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 \
  bringup_robot:=false \
  bringup_camera:=false
```

---

## 3. perception_params.yaml 详解

路径：`src/smart_follower_bringup/config/perception_params.yaml`

### 3.1 话题与坐标系

| 参数 | 默认值 | 作用 | 建议 |
|---|---|---|---|
| `color_topic` | `/camera/color/image_raw` | 彩色输入话题 | 相机话题变化时改这里 |
| `depth_topic` | `/camera/depth/image_raw` | 深度输入话题 | 需与实际 Astra 输出一致 |
| `person_pose_topic` | `person_pose` | 感知输出人物列表 | 控制侧默认消费它，通常别改 |
| `follow_command_topic` | `follow_command` | 锁定/解锁命令话题 | 一般不改 |
| `base_frame` | `base_footprint` | 输出坐标系 | 只有底盘主坐标系不同才改 |

### 3.2 YOLO 参数

| 参数 | 默认值 | 作用 | 调参建议 |
|---|---:|---|---|
| `yolo.model_path` | `models/yolo26n_static_256x320_simplify_e2e.onnx` | YOLO ONNX 路径 | 通常由 launch 覆盖 |
| `yolo.input_w` | `320` | YOLO 输入宽 | 必须与模型匹配 |
| `yolo.input_h` | `256` | YOLO 输入高 | 必须与模型匹配 |
| `yolo.person_class_id` | `0` | person 类别 id | COCO 一般就是 0 |
| `yolo.conf_threshold` | `0.25` | 检测阈值 | 漏检多就降，误检多就升 |
| `yolo.ort.intra_op_num_threads` | `1` | 算子内线程数 | 当前推荐值 |
| `yolo.ort.inter_op_num_threads` | `1` | 算子间线程数 | 通常保持 1 |
| `yolo.ort.execution_mode` | `sequential` | ORT 执行模式 | 当前推荐值 |

### 3.3 ReID 参数

| 参数 | 默认值 | 作用 | 调参建议 |
|---|---:|---|---|
| `reid.model_path` | `models/osnet_x0_5_512.onnx` | ReID ONNX 路径 | 通常由 launch 覆盖 |
| `reid.input_w` | `128` | ReID 输入宽 | 与模型匹配 |
| `reid.input_h` | `256` | ReID 输入高 | 与模型匹配 |
| `reid.ema_alpha` | `0.2` | 特征 EMA 平滑系数 | 越大越稳 |
| `reid.recover_threshold` | `0.70` | 记忆库恢复阈值 | 错认多就升，恢复难就降 |
| `reid.ort.intra_op_num_threads` | `1` | 线程数 | 轻量模型通常 1 即可 |
| `reid.ort.inter_op_num_threads` | `1` | 线程数 | 一般不动 |
| `reid.ort.execution_mode` | `sequential` | ORT 模式 | 一般不动 |

### 3.4 感知节奏 / 跟踪基础参数

| 参数 | 默认值 | 作用 | 调参建议 |
|---|---:|---|---|
| `process_every_n_frames` | `3` | 每 N 帧处理 1 帧 | 30fps 相机下约等于 10Hz |
| `detect_every_n_frames` | `1` | 每处理 N 帧做 1 次检测 | 当前保持每次处理都检测 |
| `min_confirm_hits` | `3` | 轨迹确认最少命中数 | 升大更稳但起锁更慢 |
| `max_miss_frames` | `10` | 轨迹最大丢失帧数 | 升大更抗漏检 |
| `feature_buffer_size` | `20` | 特征历史长度 | 常见 10~30 |
| `sync_cache_size` | `6` | color/depth 同步缓存大小 | 深度同步不稳时可适当增大 |
| `sync_slop` | `0.04` | 同步容忍时间差 | 同步不稳定时先小幅加到 0.05~0.06 |
| `memory_sec` | `30.0` | 记忆库保留时间 | 复杂遮挡场景可适当加大 |

### 3.5 tracking 代价参数

| 参数 | 默认值 | 作用 | 调参建议 |
|---|---:|---|---|
| `tracking.low_score_threshold` | `0.1` | 低分框阈值 | 太低会引入噪声 |
| `tracking.high_score_threshold` | `0.5` | 高分框阈值 | 越高越保守 |
| `tracking.assignment_threshold` | `0.7` | 主匹配阈值 | 串人多就降，断轨多就升一点 |
| `tracking.second_stage_threshold` | `0.8` | 二阶段阈值 | 通常略高于主阈值 |
| `tracking.weights.iou` | `0.3` | IOU 权重 | 框稳定时可略增 |
| `tracking.weights.center` | `0.2` | 中心点权重 | 抖动大时别太高 |
| `tracking.weights.appearance` | `0.4` | 外观权重 | ReID 稳时可适当加大 |

### 3.6 相机与 depth_compare 参数

> 当前这组参数已经不再是“单目定位参数”，而是 **相机服务 + 安装偏移 + depth 采样** 参数。

| 参数 | 默认值 | 作用 | 调参建议 |
|---|---:|---|---|
| `monocular.camera_info_service` | `/camera/get_camera_info` | 请求真实相机内参的服务名 | 必须可用 |
| `monocular.camera_x_offset_m` | `0.175` | 相机相对 `base_footprint` 的前后偏移 | 相机在 base 前方为正 |
| `monocular.camera_y_offset_m` | `0.01` | 相机相对 `base_footprint` 的左右偏移 | 相机在车体左侧为正 |
| `depth_compare.min_range_m` | `0.20` | depth 有效最小距离 | 太小会引入近距噪声 |
| `depth_compare.max_range_m` | `4.00` | depth 有效最大距离 | 室内跟随常用 3~4 米 |
| `depth_compare.sample_window_px` | `5` | 采样窗口边长 | 越大越稳，但更易吃到背景 |
| `depth_compare.min_valid_samples` | `3` | 最少有效深度样本数 | 太小会更冒进，太大容易丢目标 |

#### 安装偏移正负号
按 ROS 常见底盘坐标：
- `x` 朝前
- `y` 朝左
- `z` 朝上

因此：
- `camera_x_offset_m > 0`：相机在 base 原点前方
- `camera_y_offset_m > 0`：相机在车体左侧

你当前固化值表示：
- 相机位于 base 原点前方 `17.5 cm`
- 相机位于 base 原点左侧 `1 cm`

#### depth_compare 调参顺序
1. 先保证 `camera_info_service` 可用，`intrinsics_ready=1`
2. 再确认 `camera_x_offset_m / camera_y_offset_m`
3. 如果距离抖动大，先调 `sample_window_px`
4. 如果经常拿不到位置，先看 `min_valid_samples`
5. 如果远处噪声多，再收紧 `max_range_m`

### 3.7 lock 参数

| 参数 | 默认值 | 作用 | 调参建议 |
|---|---:|---|---|
| `lock.stable_frames` | `5` | 连续稳定多少帧才正式锁定 | 升大更稳，降小更快 |
| `lock.hold_sec` | `0.6` | 短时丢目标保持时间 | 短遮挡多可加一点 |
| `lock.switch_sec` | `2.0` | 切换目标前保守时间 | 串人多就加大 |
| `lock.center_roi_ratio` | `0.6` | 中心 ROI 比例 | 越小越偏向画面中心 |
| `lock.target_area_ratio` | `0.04` | 面积筛选参考值 | 太小可能锁远处小人 |

---

## 4. control_params.yaml 详解

路径：`src/smart_follower_control/config/control_params.yaml`

### 4.1 公共顶层参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `person_pose_topic` | `person_pose` | 控制侧消费的人物位置话题 |
| `follow_command_topic` | `follow_command` | 键盘/上层命令话题 |

### 4.2 follower_controller_node

| 参数 | 默认值 | 作用 | 调参建议 |
|---|---:|---|---|
| `cmd_vel_follow_topic` | `cmd_vel_follow` | 跟随控制输出 | 一般不改 |
| `control_rate` | `20.0` | 跟随控制频率 | 当前推荐值 |
| `target_distance` | `0.6` | 期望跟随距离 | 太近/太远就改它 |
| `theta_deadzone` | `0.03` | 转向死区 | 太小会抖头 |
| `target_timeout` | `0.3` | 目标超时失效时间 | 兼顾平顺与安全 |
| `prediction_horizon_s` | `0.25` | 最远预测窗口 | 太大会飘 |
| `velocity_ema_alpha` | `0.70` | 目标速度 EMA 系数 | 越大越稳 |
| `max_target_speed_mps` | `1.50` | 目标估计速度上限 | 防止异常速度拉飞预测 |
| `pid_r.kp/ki/kd` | `0.8/0.0/0.1` | 线速度 PID | 前后跟距主调这组 |
| `pid_t.kp/ki/kd` | `1.2/0.0/0.1` | 转向 PID | 左右跟随主调这组 |
| `pid_i_limit` | `0.5` | 积分限幅 | 防止积分饱和 |
| `pid_kaw` | `0.2` | anti-windup 系数 | 一般少动 |
| `limits.v_max` | `0.6` | 最大线速度 | 当前安全上限 |
| `limits.w_max` | `1.2` | 最大角速度 | 太小转不过来 |
| `limits.dv_max` | `0.5` | 线速度变化率限制 | 越小越平滑 |
| `limits.dw_max` | `1.5` | 角速度变化率限制 | 越小越平滑 |

#### 跟随手感优先调参顺序
1. `target_distance`
2. `pid_t.kp`, `pid_t.kd`
3. `pid_r.kp`, `pid_r.kd`
4. `limits.dv_max`, `limits.dw_max`
5. `prediction_horizon_s`, `velocity_ema_alpha`

### 4.3 obstacle_avoidance_node

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `left_range_topic` / `right_range_topic` | 左右超声波输入 | 左右 range 输入 |
| `cmd_vel_input_topic` | `/cmd_vel` | 原始速度输入 |
| `cmd_vel_avoid_topic` | `cmd_vel_avoid` | 避障输出 |
| `rate` | `20.0` | 避障频率 |
| `d_min` | `0.12` | 最小安全距离基线 |
| `t_react` | `0.20` | 反应时间补偿 |
| `a_brake` | `0.8` | 制动能力估计 |
| `margin` | `0.08` | 进入避障裕量 |
| `exit_margin` | `0.08` | 退出避障裕量 |
| `turn_speed` | `0.5` | 常规避障转向速度 |
| `slow_turn_speed` | `0.25` | 轻微避障转向速度 |
| `back_speed` | `-0.15` | 必要时倒车速度 |

### 4.4 arbiter_node

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `cmd_vel_follow_topic` | `cmd_vel_follow` | 跟随输入 |
| `cmd_vel_avoid_topic` | `cmd_vel_avoid` | 避障输入 |
| `cmd_vel_topic` | `/cmd_vel` | 最终输出 |
| `publish_rate` | `20.0` | 输出频率 |
| `lost_time_normal_max` | `0.2` | 正常态短时丢目标上限 |
| `lost_time_degraded_max` | `0.6` | 降级态时长 |
| `lost_time_search_max` | `2.0` | 搜索态最长时长 |
| `degraded_linear_scale` | `0.5` | 降级态线速度缩放 |
| `search_angular_speed` | `0.3` | 搜索态角速度 |
| `avoid_enter_threshold` | `3` | 进入避障阈值 |
| `avoid_exit_threshold` | `5` | 退出避障阈值 |
| `avoid_exit_hysteresis_time` | `0.2` | 退出避障迟滞 |
| `avoid_cmd_timeout` | `0.2` | 避障命令超时 |
| `avoid_nonzero_epsilon` | `0.001` | 判定非零命令的阈值 |

### 4.5 ultrasonic_range_node

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `rate` | `10.0` | 采样频率 |
| `window_size` | `5` | 中值窗口大小 |
| `min_range` | `0.03` | 最小有效距离 |
| `max_range` | `3.0` | 最大有效距离 |
| `left/right.trig_pin` | `23/4` 等 | GPIO 配置 |
| `frame_left/frame_right` | `ultrasonic_left_link/right_link` | 超声波 frame |

### 4.6 keyboard_command_node

| 参数 | 默认值 | 作用 |
|---|---|---|
| `follow_command_topic` | `follow_command` | 命令输出话题 |
| `key_lock` | `l` | 锁定 |
| `key_unlock` | `u` | 解锁 |
| `key_reset` | `r` | 重置 |
| `key_estop` | `q` | 急停 |

---

## 5. 实车推荐调参顺序

### 第一阶段：先保证链路正确
1. `intrinsics_ready=1`
2. `camera_info_service` 正常返回真实内参
3. `/camera/color/image_raw` 与 `/camera/depth/image_raw` 都稳定输入
4. `/robot1/person_pose` 持续输出

### 第二阶段：先把“能跟”调出来
1. `camera_x_offset_m`
2. `camera_y_offset_m`
3. `target_distance`
4. `pid_t.kp / kd`
5. `pid_r.kp / kd`

### 第三阶段：把“跟得稳”调出来
1. `limits.dv_max`, `limits.dw_max`
2. `velocity_ema_alpha`
3. `prediction_horizon_s`
4. `lock.stable_frames`, `lock.hold_sec`

### 第四阶段：把“复杂场景不串人”调出来
1. `reid.recover_threshold`
2. `tracking.weights.appearance`
3. `tracking.assignment_threshold`
4. `lock.switch_sec`

### 第五阶段：把“安全避障”调出来
1. `d_min`
2. `margin`
3. `turn_speed`
4. `back_speed`

---

## 6. 最值得记录的实车参数

建议单独记录这几项：
- `monocular.camera_x_offset_m`
- `monocular.camera_y_offset_m`
- `depth_compare.sample_window_px`
- `depth_compare.min_valid_samples`
- `target_distance`
- `pid_r.kp`, `pid_r.kd`
- `pid_t.kp`, `pid_t.kd`
- `prediction_horizon_s`
- `velocity_ema_alpha`
- `d_min`, `margin`

---

## 7. 一句话版建议

如果你现在就要开始调：
1. **先确认内参服务和 depth_compare 正常**
2. **再调 target_distance 和跟随 PID**
3. **最后再修预测、锁定和避障**

不要再按旧单目路线去调 `camera_height_m / camera_pitch_deg` 这类已删除参数；当前主线里它们已经不参与实际运行。
