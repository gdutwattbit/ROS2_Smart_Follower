# Smart Follower 调参说明（try.md）

本文基于当前工作区实际使用的配置文件整理：
- `src/smart_follower_bringup/config/perception_params.yaml`
- `src/smart_follower_control/config/control_params.yaml`
- `src/smart_follower_bringup/launch/smart_follower.launch.py`
- `src/smart_follower_bringup/launch/smart_follower_only.launch.py`

> 说明：
> - 本文默认命名空间为 `robot1`。
> - launch 会额外覆盖模型路径，因此最终运行时模型以 launch 里指定的为准。
> - 单位未特别说明时，角度相关若带 `deg` 就是“度”，时间相关若带 `s`/`sec` 就是“秒”，距离默认“米”。

---

## 1. 当前默认运行组合

### 1.1 感知侧模型
launch 中会把模型路径覆盖为：
- YOLO: `models/yolo26n_static_480x640_simplify_e2e.onnx`
- ReID: `models/osnet_x0_5_512.onnx`

### 1.2 主链路频率
- 相机彩色输入：由相机驱动决定
- 感知处理：`process_every_n_frames=3`
- 检测频率控制：`detect_every_n_frames=1`
- 跟随控制输出：`control_rate=20.0`
- 超声波：`rate=10.0`
- 避障 / 仲裁：`20Hz`

如果彩色图是 30fps，则当前 perception 实际处理节奏约为 10Hz。

---

## 2. launch 层说明

### 2.1 `smart_follower.launch.py`
用于整车启动：
- 可同时拉起底盘相关节点
- 可同时拉起相机
- 再拉起跟随感知与控制

关键 launch 参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `robot_ns` | `robot1` | 机器人命名空间 |
| `bringup_robot` | `true` | 是否拉起车体底层 |
| `bringup_camera` | `true` | 是否拉起相机驱动 |

### 2.2 `smart_follower_only.launch.py`
用于只拉起跟随相关节点，不启动底层驱动。

适合：
- 单独调 perception / control
- 已经手工起好相机和底盘驱动

---

## 3. perception_params.yaml 详解

路径：`src/smart_follower_bringup/config/perception_params.yaml`

### 3.1 话题与坐标系

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---|---|---|---|
| `color_topic` | `/camera/color/image_raw` | 话题 | 彩色输入话题 | 相机话题改名时改这里 |
| `person_pose_topic` | `person_pose` | 话题 | 感知输出人物列表话题 | 控制侧默认也读它，通常别改 |
| `follow_command_topic` | `follow_command` | 话题 | 键盘/上层跟随控制命令 | 一般不改 |
| `base_frame` | `base_footprint` | 坐标系 | 输出 `PersonPoseArray` 的 frame_id | 若车体主坐标系不同才改 |

---

### 3.2 YOLO 检测参数

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `yolo.model_path` | launch 覆盖 | 路径 | YOLO ONNX 模型路径 | 通常由 launch 覆盖，不建议 YAML 单独改 |
| `yolo.input_w` | `640` | 像素 | YOLO 输入宽 | 要和导出模型匹配 |
| `yolo.input_h` | `480` | 像素 | YOLO 输入高 | 要和导出模型匹配 |
| `yolo.person_class_id` | `0` | 类别 id | 人类别 id | COCO person 一般是 0 |
| `yolo.conf_threshold` | `0.25` | 无量纲 | 检测置信度阈值 | 漏检多就降到 0.20~0.22，误检多就升到 0.30~0.40 |
| `yolo.ort.intra_op_num_threads` | `3` | 线程 | ORT 算子内线程数 | 当前最佳组之一，树莓派上优先 2~4 内试 |
| `yolo.ort.inter_op_num_threads` | `1` | 线程 | ORT 算子间线程数 | 通常保持 1 |
| `yolo.ort.execution_mode` | `sequential` | 枚举 | ORT 执行模式 | 目前建议 `sequential` |

**优先调的项**：
1. `conf_threshold`
2. `intra_op_num_threads`
3. 模型本身

---

### 3.3 ReID 参数

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `reid.model_path` | launch 覆盖 | 路径 | ReID ONNX 模型路径 | 通常由 launch 覆盖 |
| `reid.input_w` | `128` | 像素 | ReID 输入宽 | 要和模型匹配 |
| `reid.input_h` | `256` | 像素 | ReID 输入高 | 要和模型匹配 |
| `reid.ema_alpha` | `0.2` | 无量纲 | 轨迹外观特征 EMA 平滑系数 | 越大越稳，越小越跟手 |
| `reid.recover_threshold` | `0.70` | 无量纲 | 用记忆库恢复锁定对象的阈值 | 错认多就升，恢复困难就降 |
| `reid.ort.intra_op_num_threads` | `1` | 线程 | ReID ORT 线程数 | 轻量模型一般 1 就够 |
| `reid.ort.inter_op_num_threads` | `1` | 线程 | ReID 算子间线程数 | 一般不动 |
| `reid.ort.execution_mode` | `sequential` | 枚举 | ORT 执行模式 | 通常不动 |

**建议区间**：
- `reid.recover_threshold` 常见调节范围：`0.60 ~ 0.80`

---

### 3.4 感知频率 / 跟踪基础参数

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `process_every_n_frames` | `3` | 帧 | 每收到 N 帧彩色图，处理 1 帧 | 30fps 相机下，3 约等于 10Hz；性能够可降到 2 |
| `detect_every_n_frames` | `1` | 帧 | 每处理 N 帧做 1 次检测 | 现在保持每次处理都检测 |
| `min_confirm_hits` | `3` | 帧 | 轨迹确认所需最少命中次数 | 升大更稳但起锁更慢 |
| `max_miss_frames` | `10` | 帧 | 轨迹允许丢失的最大帧数 | 升大更抗短时漏检，但残影更久 |
| `feature_buffer_size` | `20` | 个 | 每条轨迹保留的特征历史长度 | 通常 10~30 足够 |
| `sync_cache_size` | `6` | 帧 | 彩色帧缓存大小 | 现在纯 RGB，同步压力不大，通常不需要大改 |
| `memory_sec` | `30.0` | 秒 | 记忆库保留时长 | 场景简单可降，跨遮挡恢复需求高可升 |

**常用思路**：
- 想更丝滑、算力又够：先试 `process_every_n_frames: 2`
- 想更稳：可保留 3，不轻易追高频

---

### 3.5 tracking 代价与阈值

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `tracking.low_score_threshold` | `0.1` | 无量纲 | 低分框阈值 | 太低会引入噪声框 |
| `tracking.high_score_threshold` | `0.5` | 无量纲 | 高分框阈值 | 越高越保守 |
| `tracking.assignment_threshold` | `0.7` | 无量纲 | 主匹配阶段拒配阈值 | 错配多就降，断轨多就升一点 |
| `tracking.second_stage_threshold` | `0.8` | 无量纲 | 二阶段匹配阈值 | 通常略大于主阈值 |
| `tracking.weights.iou` | `0.3` | 权重 | IOU 代价权重 | 目标框稳定时可适当增大 |
| `tracking.weights.center` | `0.2` | 权重 | 中心点距离代价权重 | 画面抖动大时别设太高 |
| `tracking.weights.appearance` | `0.4` | 权重 | 外观特征代价权重 | ReID 好用时可加大 |

**经验**：
- 遮挡后容易串人：提高 `appearance` 权重，或提高 `reid.recover_threshold`
- 目标框抖动大但人没变：降低 `center`，别过度依赖中心点

---

### 3.6 monocular 单目定位参数

这一组是这轮最关键的参数。

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `monocular.camera_info_service` | `/camera/get_camera_info` | 服务名 | 启动时获取相机真实内参 | Astra 默认用它 |
| `monocular.person_height_m` | `1.70` | 米 | 兼容保留参数，当前版本**不参与计算** | 暂时可忽略 |
| `monocular.horizontal_fov_deg` | `69.0` | 度 | 当服务拿不到内参时，用于 fallback 估算焦距 | 只有 fallback 才生效 |
| `monocular.min_range_m` | `0.60` | 米 | 输出最近距离截断 | 太小会让近距离估计更激进 |
| `monocular.max_range_m` | `6.00` | 米 | 输出最远距离截断 | 室内跟随可适当降到 3~4 |
| `monocular.camera_height_m` | `0.28` | 米 | 相机光心离地高度 | **必须按实车标定** |
| `monocular.camera_pitch_deg` | `18.0` | 度 | 相机俯仰角，向下为正 | **必须按实车标定** |
| `monocular.camera_x_offset_m` | `0.00` | 米 | 相机相对 base 前后偏移 | 相机不在车体原点时填写 |
| `monocular.camera_y_offset_m` | `0.00` | 米 | 相机相对 base 左右偏移 | 偏左为正/偏右为负要和你车体定义一致验证 |
| `monocular.min_downward_angle_deg` | `2.0` | 度 | 接近地平线时拒绝投影的最小下视角 | 太小容易出离谱远距离，太大容易丢远处目标 |

#### 这组参数怎么理解
当前位置估计不是“按人高猜距离”，而是：
- 取 bbox 底边中心，近似人的脚点
- 结合相机内参
- 再结合相机离地高度与俯仰角
- 把这个像素点投到地面上

所以对结果影响最大的不是 `person_height_m`，而是：
1. `camera_height_m`
2. `camera_pitch_deg`
3. 内参是否真实拿到
4. `min_downward_angle_deg`

#### 实车优先调参顺序
1. **先保证 `camera_info_service` 可用**
2. 标定 `camera_height_m`
3. 标定 `camera_pitch_deg`
4. 若目标整体偏前/偏后，再微调 `camera_x_offset_m`
5. 若目标整体偏左/偏右，再微调 `camera_y_offset_m`
6. 最后再调 `min_downward_angle_deg`

#### 推荐调法
- 车停住
- 让人站在已知距离（如 1m / 1.5m / 2m）
- 看 `/person_pose` 中 `position.x / position.y`
- 若整体都偏远或偏近：优先改 `camera_pitch_deg`
- 若近处对、远处错很多：检查 `camera_height_m` 与真实内参
- 若左右持续有固定偏移：改 `camera_y_offset_m`

---

### 3.7 lock 锁定逻辑参数

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `lock.stable_frames` | `5` | 帧 | 候选目标连续稳定多少帧才正式锁定 | 升大更稳，降小更快 |
| `lock.hold_sec` | `0.6` | 秒 | 短时丢目标后的保持时间 | 短遮挡多就稍微加大 |
| `lock.switch_sec` | `2.0` | 秒 | 切换目标前的保守时间 | 串人多就加大 |
| `lock.center_roi_ratio` | `0.6` | 比例 | 中心 ROI 范围比例 | 越小越偏向画面中心人物 |
| `lock.target_area_ratio` | `0.04` | 比例 | 目标面积筛选参考值 | 太小可能锁远处小人，太大可能只锁近人 |

---

## 4. control_params.yaml 详解

路径：`src/smart_follower_control/config/control_params.yaml`

这个文件分两层：
- `/**.ros__parameters`：公共顶层参数
- 各节点自己的 `node_name.ros__parameters`

### 4.1 顶层公共参数

| 参数 | 默认值 | 单位 | 作用 |
|---|---:|---|---|
| `person_pose_topic` | `person_pose` | 话题 | 控制侧消费的人物位置话题 |
| `follow_command_topic` | `follow_command` | 话题 | 控制命令输入话题 |

---

### 4.2 follower_controller_node 跟随控制

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `cmd_vel_follow_topic` | `cmd_vel_follow` | 话题 | 跟随控制输出 | 一般不改 |
| `control_rate` | `20.0` | Hz | 跟随控制频率 | 当前推荐值 |
| `target_distance` | `1.0` | 米 | 期望跟随距离 | 跟得太近/太远就改它 |
| `theta_deadzone` | `0.03` | 弧度 | 角度死区 | 太小会频繁抖头，太大则转向迟钝 |
| `target_timeout` | `0.3` | 秒 | 目标超时失效时间 | 安全性很关键，不建议随便加太大 |
| `prediction_horizon_s` | `0.25` | 秒 | 控制侧预测补帧的最远外推窗口 | 太大会“飘”，太小则补偿不足 |
| `velocity_ema_alpha` | `0.70` | 无量纲 | 目标速度估计 EMA 系数 | 越大越稳，越小越跟手 |
| `max_target_speed_mps` | `1.50` | m/s | 目标估计速度上限 | 用于防止异常速度把预测拉飞 |
| `pid_r.kp` | `0.8` | 增益 | 线速度 P | 跟距响应强弱 |
| `pid_r.ki` | `0.0` | 增益 | 线速度 I | 通常保持 0 或很小 |
| `pid_r.kd` | `0.1` | 增益 | 线速度 D | 抑制前后冲击 |
| `pid_t.kp` | `1.2` | 增益 | 转向 P | 转向跟随力度 |
| `pid_t.ki` | `0.0` | 增益 | 转向 I | 一般保持 0 |
| `pid_t.kd` | `0.1` | 增益 | 转向 D | 抑制左右摆动 |
| `pid_i_limit` | `0.5` | 控制量 | PID 积分限幅 | 防止积分饱和 |
| `pid_kaw` | `0.2` | 系数 | anti-windup 回算系数 | 一般少动 |
| `limits.v_max` | `0.6` | m/s | 最大线速度 | 步行跟随通常够用 |
| `limits.w_max` | `1.2` | rad/s | 最大角速度 | 太小转不过来，太大容易晃 |
| `limits.dv_max` | `0.5` | m/s² 近似 | 线速度变化率限制 | 越小越平滑 |
| `limits.dw_max` | `1.5` | rad/s² 近似 | 角速度变化率限制 | 越小越平滑 |

#### 跟随手感优先调哪些
1. `target_distance`
2. `pid_t.kp`, `pid_t.kd`
3. `pid_r.kp`, `pid_r.kd`
4. `limits.dv_max`, `limits.dw_max`
5. `prediction_horizon_s`, `velocity_ema_alpha`

#### 典型现象与调法
- **左右摆头明显**：降 `pid_t.kp`，或升 `pid_t.kd`，必要时降 `dw_max`
- **前后窜动**：降 `pid_r.kp`，或升 `pid_r.kd`，必要时降 `dv_max`
- **跟随迟钝**：适当升 `pid_r.kp` / `pid_t.kp`
- **短时漏检就掉速**：看 `target_timeout` 与感知频率是否匹配，但安全前提下微调
- **预测太冲 / 目标一快就飘**：降 `prediction_horizon_s` 或降 `max_target_speed_mps`

---

### 4.3 obstacle_avoidance_node 避障

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `left_range_topic` | `left_ultrasonic/range` | 话题 | 左超声波输入 |
| `right_range_topic` | `right_ultrasonic/range` | 话题 | 右超声波输入 |
| `cmd_vel_input_topic` | `/cmd_vel` | 话题 | 原始速度输入 | 通常来自仲裁后的主速度链 |
| `cmd_vel_avoid_topic` | `cmd_vel_avoid` | 话题 | 避障修正输出 |
| `rate` | `20.0` | Hz | 避障计算频率 |
| `d_min` | `0.12` | 米 | 最小安全距离基线 |
| `t_react` | `0.20` | 秒 | 反应时间补偿 |
| `a_brake` | `0.8` | m/s² | 制动能力估计 |
| `margin` | `0.08` | 米 | 进入避障附加裕量 |
| `exit_margin` | `0.08` | 米 | 退出避障附加裕量 |
| `turn_speed` | `0.5` | rad/s | 常规避障转向速度 |
| `slow_turn_speed` | `0.25` | rad/s | 轻微避障转向速度 |
| `back_speed` | `-0.15` | m/s | 必要时倒车速度 |

**优先调**：
- `d_min`
- `margin`
- `turn_speed`
- `back_speed`

---

### 4.4 arbiter_node 仲裁

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `cmd_vel_follow_topic` | `cmd_vel_follow` | 话题 | 跟随速度输入 |
| `cmd_vel_avoid_topic` | `cmd_vel_avoid` | 话题 | 避障速度输入 |
| `cmd_vel_topic` | `/cmd_vel` | 话题 | 最终输出到底盘 |
| `publish_rate` | `20.0` | Hz | 仲裁输出频率 |
| `lost_time_normal_max` | `0.2` | 秒 | 正常跟随态允许目标短时丢失时长 |
| `lost_time_degraded_max` | `0.6` | 秒 | 降级态持续时长 |
| `lost_time_search_max` | `2.0` | 秒 | 搜索态最长持续时长 |
| `degraded_linear_scale` | `0.5` | 比例 | 降级态线速度缩放 |
| `search_angular_speed` | `0.3` | rad/s | 搜索态转向速度 |
| `avoid_enter_threshold` | `3` | 次数/周期 | 进入避障阈值 |
| `avoid_exit_threshold` | `5` | 次数/周期 | 退出避障阈值 |
| `avoid_exit_hysteresis_time` | `0.2` | 秒 | 退出避障的迟滞时间 |
| `avoid_cmd_timeout` | `0.2` | 秒 | 避障命令超时 |
| `avoid_nonzero_epsilon` | `0.001` | 速度阈值 | 判定避障命令是否“非零” |

**如果你想让丢目标后更温和**：
- 可适当增大 `lost_time_degraded_max`
- 但别一味加大，安全和空跟风险会增加

---

### 4.5 ultrasonic_range_node 超声波

| 参数 | 默认值 | 单位 | 作用 | 调参建议 |
|---|---:|---|---|---|
| `rate` | `10.0` | Hz | 采样频率 | 超声波通常 10Hz 足够 |
| `window_size` | `5` | 个 | 中值/历史窗口大小 | 越大越稳，越大延迟也更大 |
| `min_range` | `0.03` | 米 | 超声最小有效距离 |
| `max_range` | `3.0` | 米 | 超声最大有效距离 |
| `left.trig_pin` | `23` | BCM 引脚 | 左传感器 trig |
| `left.echo_pin` | `24` | BCM 引脚 | 左传感器 echo |
| `left.topic` | `left_ultrasonic/range` | 话题 | 左侧 range 输出 |
| `right.trig_pin` | `4` | BCM 引脚 | 右传感器 trig |
| `right.echo_pin` | `14` | BCM 引脚 | 右传感器 echo |
| `right.topic` | `right_ultrasonic/range` | 话题 | 右侧 range 输出 |
| `frame_left` | `ultrasonic_left_link` | frame | 左超声坐标系 |
| `frame_right` | `ultrasonic_right_link` | frame | 右超声坐标系 |

---

### 4.6 keyboard_command_node 键盘控制

| 参数 | 默认值 | 作用 |
|---|---|---|
| `follow_command_topic` | `follow_command` | 发布命令的话题 |
| `key_lock` | `l` | 锁定当前目标 |
| `key_unlock` | `u` | 解锁 |
| `key_reset` | `r` | 重置状态 |
| `key_estop` | `q` | 急停 |

---

## 5. 推荐调参顺序（实车）

### 第一阶段：先把“能跟”调出来
1. 确认 perception diagnostics 里 `intrinsics_ready=1`
2. 若 `intrinsics_source=service`，优先使用真实内参
3. 标定：
   - `monocular.camera_height_m`
   - `monocular.camera_pitch_deg`
4. 调 `target_distance`
5. 小幅调整 `pid_t.kp / kd` 与 `pid_r.kp / kd`

### 第二阶段：把“跟得稳”调出来
1. `limits.dv_max`, `limits.dw_max`
2. `velocity_ema_alpha`
3. `prediction_horizon_s`
4. `lock.stable_frames`, `lock.hold_sec`

### 第三阶段：把“复杂场景不串人”调出来
1. `reid.recover_threshold`
2. `tracking.weights.appearance`
3. `tracking.assignment_threshold`
4. `lock.switch_sec`

### 第四阶段：把“安全避障”调出来
1. `d_min`
2. `margin`
3. `turn_speed`
4. `back_speed`
5. `avoid_enter_threshold / avoid_exit_threshold`

---

## 6. 我最建议你优先记录的几组实车标定值

建议单独记录在你的实验笔记里：
- `camera_height_m`
- `camera_pitch_deg`
- `camera_x_offset_m`
- `camera_y_offset_m`
- `target_distance`
- `pid_r.kp`, `pid_r.kd`
- `pid_t.kp`, `pid_t.kd`
- `prediction_horizon_s`
- `velocity_ema_alpha`
- `d_min`, `margin`

这几项基本决定了：
- 人在画面里时，车知不知道人在哪
- 车往前跟的时候稳不稳
- 左右转的时候晃不晃
- 遇到障碍时是不是安全

---

## 7. 一句话版建议

如果你现在就要开始调：
1. **先标定 `camera_height_m + camera_pitch_deg`**
2. **再调 `target_distance` 和跟随 PID**
3. **最后再修预测、锁定和避障**

如果这一步顺序反了，后面很多“控制看起来不对”的问题，本质上其实是前面的单目位置估计就已经偏了。
