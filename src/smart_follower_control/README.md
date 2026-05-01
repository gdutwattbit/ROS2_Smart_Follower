# smart_follower_control

`smart_follower_control` 负责控制侧闭环，当前链路是：

```text
/person_pose -> follower_runtime -> cmd_vel_follow
ultrasonic -> obstacle_runtime -> cmd_vel_avoid
follow + avoid -> arbiter_runtime -> /cmd_vel
```

## Main Modules

- `follower_runtime.*`: 跟随控制核心逻辑
- `obstacle_runtime.*`: 超声波避障逻辑
- `arbiter_runtime.*`: 最终速度仲裁，正式行为为 `STOP / FOLLOW / AVOID`
- `ultrasonic_runtime.*`: 左右超声波采样与 GPIO 后端适配

## Parameter Layout

- 基础参数文件：`src/smart_follower_control/config/control_params.yaml`
- `robot1` 单节点调试覆盖：`src/smart_follower_control/config/control_params_robot1_debug.yaml`

调试时建议显式叠加：

```bash
ros2 run smart_follower_control follower_controller_node \
  --ros-args \
  --params-file src/smart_follower_control/config/control_params.yaml \
  --params-file src/smart_follower_control/config/control_params_robot1_debug.yaml
```

## Notes

- `arbiter` 仍声明 `lost_time_* / degraded_linear_scale / search_angular_speed` 这组旧参数一轮兼容，但运行时已忽略。
- 控制节点热更新遵循统一策略：topic / rate 变化才重建接口，纯运行参数在线更新。
