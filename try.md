# Smart Follower 现场调参与排查速查

本文面向当前工作区的控制链路，重点记录 `follower_controller_node` 的调参与现场命令。

当前默认命名空间：`/robot1`
当前控制参数文件：`src/smart_follower_control/config/control_params.yaml`

## 1. 当前控制参数

当前常用控制参数如下：

```yaml
/robot1/follower_controller_node:
  ros__parameters:
    control_rate: 20.0
    target_distance: 0.6
    steering_kalman:
      process_noise: 10.0
      measurement_noise: 0.1
      initial_covariance: 2.0
    target_timeout: 1.2
    prediction_horizon_s: 0.25
    velocity_ema_alpha: 0.70
    pid_r:
      kp: 5.0
      ki: 1.0
      kd: 2.0
    pid_t:
      kp: 1.5
      ki: 0.5
      kd: 0.5
    pid_i_limit: 1.0
    pid_kaw: 0.2
    limits:
      v_max: 0.6
      w_max: 1.2
      dv_max: 0.5
      dw_max: 1.5
```

## 2. 参数含义

### 2.1 跟随几何与预测

- `target_distance`
  期望跟随距离，单位米。
- `target_timeout`
  目标超时阈值。超过这个时间没有拿到有效目标，控制器会清零输出。
- `prediction_horizon_s`
  目标预测的最大时间窗。
- `velocity_ema_alpha`
  目标速度估计的 EMA 平滑系数。越大越稳，越小越灵。

### 2.2 转向卡尔曼滤波

- `steering_kalman.process_noise`
  越大越灵，越小越稳。
- `steering_kalman.measurement_noise`
  越大越不信当前观测，越平滑。
- `steering_kalman.initial_covariance`
  刚锁定或刚复位时，对当前观测的贴合速度。

### 2.3 PID

- `pid_r.kp / ki / kd`
  控制前后距离。
- `pid_t.kp / ki / kd`
  控制左右转向。
- `pid_i_limit`
  积分限幅。
- `pid_kaw`
  anti-windup 系数，用来抑制积分在饱和时越积越多。

### 2.4 输出限制

- `limits.v_max`
  线速度上限。
- `limits.w_max`
  角速度上限。
- `limits.dv_max`
  线速度变化率上限。
- `limits.dw_max`
  角速度变化率上限。

## 3. 目前已经删掉的旧限制

为了让控制更直接，当前控制链已经删除以下额外限制：

- `theta_deadzone`
- 大转角时的自动降速逻辑 `abs(theta) > 0.5 -> v *= 0.3`
- `max_target_speed_mps` 对目标速度估计的硬截断

因此现在真正保留的“硬限制”主要就是：

- `limits.v_max`
- `limits.w_max`
- `limits.dv_max`
- `limits.dw_max`
- `target_timeout`

## 4. 现场调参顺序

建议按下面顺序调，不要一上来同时改很多项。

### 4.1 直线太慢、太肉

先调：

1. `pid_r.kp`
2. `pid_r.ki`
3. `limits.v_max`
4. `limits.dv_max`

经验：

- 想让车更愿意往前追：先加 `pid_r.kp`
- 想让车长时间误差能顶上去：再加 `pid_r.ki`
- 想让车起步更猛：加 `limits.dv_max`
- 想让车最高速度更快：加 `limits.v_max`

### 4.2 转弯正常但前轮抽搐

先调：

1. `steering_kalman.measurement_noise`
2. `limits.dw_max`
3. `pid_t.kd`
4. `steering_kalman.process_noise`

经验：

- 先增大 `measurement_noise` 抑制抖动
- 再减小 `dw_max` 限制角速度变化率
- 如仍有高频摆动，再补一点 `pid_t.kd`
- 若变得太钝，再适度提高 `process_noise`

### 4.3 反应太慢但不抖

先调：

1. `steering_kalman.process_noise`
2. `steering_kalman.measurement_noise`
3. `pid_t.kp`
4. `limits.dw_max`

经验：

- 提高 `process_noise`
- 或降低 `measurement_noise`
- 再看 `pid_t.kp` 是否偏小
- 最后确认是不是 `dw_max` 压得太死

### 4.4 走一下停一下

优先排查：

1. `person_pose` 是否持续更新
2. 是否频繁出现 `target_timeout`
3. 感知侧是否有 `depth_window_no_valid_samples`
4. `target_timeout` 是否过小

这个问题通常不是 PID 本身造成的，而是上游目标无效或超时。

## 5. 常用调参命令

以下命令默认你已经 `source install/setup.bash`，并且节点运行在 `robot1` 命名空间。

### 5.1 查看参数

```bash
ros2 param list /robot1/follower_controller_node
ros2 param get /robot1/follower_controller_node target_distance
ros2 param get /robot1/follower_controller_node pid_r.kp
ros2 param get /robot1/follower_controller_node pid_t.kp
ros2 param get /robot1/follower_controller_node steering_kalman.process_noise
ros2 param get /robot1/follower_controller_node limits.v_max
```

### 5.2 在线修改参数

```bash
ros2 param set /robot1/follower_controller_node target_distance 0.7
ros2 param set /robot1/follower_controller_node pid_r.kp 6.0
ros2 param set /robot1/follower_controller_node pid_r.ki 1.2
ros2 param set /robot1/follower_controller_node pid_t.kp 1.8
ros2 param set /robot1/follower_controller_node pid_t.kd 0.8
ros2 param set /robot1/follower_controller_node pid_i_limit 1.2
ros2 param set /robot1/follower_controller_node pid_kaw 0.3
ros2 param set /robot1/follower_controller_node steering_kalman.process_noise 12.0
ros2 param set /robot1/follower_controller_node steering_kalman.measurement_noise 0.15
ros2 param set /robot1/follower_controller_node steering_kalman.initial_covariance 2.5
ros2 param set /robot1/follower_controller_node limits.v_max 0.8
ros2 param set /robot1/follower_controller_node limits.w_max 1.5
ros2 param set /robot1/follower_controller_node limits.dv_max 0.8
ros2 param set /robot1/follower_controller_node limits.dw_max 1.0
ros2 param set /robot1/follower_controller_node target_timeout 1.5
ros2 param set /robot1/follower_controller_node prediction_horizon_s 0.30
```

### 5.3 一组常用试调命令

偏灵敏：

```bash
ros2 param set /robot1/follower_controller_node pid_r.kp 6.0
ros2 param set /robot1/follower_controller_node pid_t.kp 1.8
ros2 param set /robot1/follower_controller_node steering_kalman.process_noise 14.0
ros2 param set /robot1/follower_controller_node steering_kalman.measurement_noise 0.08
ros2 param set /robot1/follower_controller_node limits.dv_max 0.8
ros2 param set /robot1/follower_controller_node limits.dw_max 1.8
```

偏平滑：

```bash
ros2 param set /robot1/follower_controller_node pid_t.kd 0.8
ros2 param set /robot1/follower_controller_node steering_kalman.process_noise 6.0
ros2 param set /robot1/follower_controller_node steering_kalman.measurement_noise 0.18
ros2 param set /robot1/follower_controller_node limits.dv_max 0.4
ros2 param set /robot1/follower_controller_node limits.dw_max 0.8
```

### 5.4 保存到 yaml

在线调参满意后，记得把最终值回写到：

`src/smart_follower_control/config/control_params.yaml`

否则下次重启会恢复旧值。

## 6. 常用观测命令

### 6.1 看 person_pose 是否稳定

```bash
ros2 topic hz /robot1/person_pose
ros2 topic echo /robot1/person_pose --once
```

### 6.2 看控制输出是否连续

```bash
ros2 topic hz /robot1/cmd_vel_follow
ros2 topic echo /robot1/cmd_vel_follow
```

### 6.3 看最终底盘速度

```bash
ros2 topic echo /cmd_vel
```

### 6.4 看节点参数是否真的生效

```bash
ros2 param dump /robot1/follower_controller_node
```

## 7. 推荐的单次调参方法

每次只改 1 到 2 个参数，跑一轮，记录现象。

推荐节奏：

1. 先只调 `pid_r`，把直线跟随速度调顺
2. 再调 `pid_t + steering_kalman`，把转向手感调顺
3. 最后用 `limits.dv_max / dw_max` 收口，让动作既跟手又不抽

如果出现“明显停住”，优先查 `person_pose` 和超时，不要先怪 PID。
