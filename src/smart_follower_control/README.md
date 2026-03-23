# smart_follower_control

`smart_follower_control` 负责控制侧闭环，当前链路是：

```text
/person_pose -> follower_runtime -> cmd_vel_follow
ultrasonic -> obstacle_runtime -> cmd_vel_avoid
follow + avoid -> arbiter_runtime -> /cmd_vel
```

## 1. 主要模块

- `src/follower_controller_node.cpp`
  - 跟随控制节点
- `src/follower_runtime.cpp`
  - 跟随控制核心逻辑
- `src/obstacle_avoidance_node.cpp`
  - 避障节点
- `src/obstacle_runtime.cpp`
  - 避障核心逻辑
- `src/arbiter_node.cpp`
  - 仲裁节点
- `src/arbiter_runtime.cpp`
  - 仲裁状态机与超时逻辑
- `src/ultrasonic_range_node.cpp`
  - 左右超声波采样

## 2. 当前设计重点

- follower 保持 20Hz 输出，可消费较低频感知输入
- obstacle 当前只依赖左右超声波，不再消费深度图
- arbiter 统一决定最终底盘输出 `/cmd_vel`

## 3. 参数入口

主要看：
- `src/smart_follower_control/config/control_params.yaml`

## 4. 如果你要改哪里

- 改跟随平滑性：看 `follower_runtime.*`
- 改避障阈值 / 行为：看 `obstacle_runtime.*`
- 改丢目标后的退化策略：看 `arbiter_runtime.*`
- 改超声波读数流程：看 `ultrasonic_runtime.*`
