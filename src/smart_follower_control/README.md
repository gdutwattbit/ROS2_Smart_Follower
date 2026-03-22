# smart_follower_control

`smart_follower_control` 负责整条控制链：
**跟随控制 → 避障 → 仲裁 → 键盘命令 → 超声波采样**。

当前这部分代码按“**node + runtime**”的方式组织：
- `*_node.cpp` 负责 ROS2 Lifecycle、订阅/发布、参数热更新、diagnostics
- `*_runtime.cpp` 负责纯业务逻辑与状态机

这套结构的目标是：新人先看 node 了解接口，再看 runtime 理解控制策略，不需要一上来面对一个超大文件。

## 1. 推荐阅读顺序

1. `src/follower_controller_node.cpp`
2. `src/follower_runtime.cpp`
3. `src/obstacle_avoidance_node.cpp`
4. `src/obstacle_runtime.cpp`
5. `src/arbiter_node.cpp`
6. `src/arbiter_runtime.cpp`
7. `src/ultrasonic_range_node.cpp`
8. `src/ultrasonic_runtime.cpp`
9. `src/keyboard_command_node.cpp`
10. `include/smart_follower_control/control_node_common.hpp`
11. `include/smart_follower_control/lifecycle_utils.hpp`

## 2. 各节点职责

| 节点 | 作用 |
|---|---|
| `follower_controller_node` | 根据锁定目标位置计算 `cmd_vel_follow` |
| `obstacle_avoidance_node` | 融合深度/超声波/当前速度，输出 `cmd_vel_avoid` |
| `arbiter_node` | 把跟随、避障、急停、SEARCH/STOP 状态整合成最终 `/cmd_vel` |
| `ultrasonic_range_node` | 轮询左右超声波，发布 `sensor_msgs/Range` |
| `keyboard_command_node` | 发布 `LOCK / UNLOCK / RESET / ESTOP` 命令 |

## 3. 当前文件职责

| 文件 | 作用 |
|---|---|
| `follower_controller_node.cpp` | 跟随控制节点入口 |
| `follower_runtime.cpp` | PID、限幅、超时降级 |
| `obstacle_avoidance_node.cpp` | 避障节点入口 |
| `obstacle_runtime.cpp` | 深度 ROI、动态安全距离、避障输出 |
| `arbiter_node.cpp` | 仲裁节点入口 |
| `arbiter_runtime.cpp` | FOLLOW / SEARCH / AVOID / STOP 状态机 |
| `ultrasonic_range_node.cpp` | 超声波节点入口 |
| `ultrasonic_runtime.cpp` | GPIO backend、交替测量、滤波、dry mode |
| `control_node_common.*` | 控制侧共享小工具：参数夹紧、通用返回值、publisher 重建辅助 |
| `lifecycle_utils.hpp` | 生命周期状态判断、publisher 激活封装、统一 main 启动模板 |

## 4. P2 后建议遵守的边界

- 参数夹紧、`make_ok_result()`、publisher 重建辅助，优先放进 `control_node_common.*`
- 不要把 runtime 的控制公式/状态机写回 node
- node 里只保留：ROS 接口、热更新、diagnostics glue
- 如果某个控制节点又长到看不动，优先继续拆 runtime，而不是引入复杂继承层级

## 5. 常见维护入口

### 想改跟随控制行为
优先看：
- `follower_runtime.hpp/cpp`
- `control_params.yaml` 里 PID / 限幅参数

### 想改避障策略
优先看：
- `obstacle_runtime.hpp/cpp`
- 深度 ROI、percentile、动态安全距离公式

### 想改 STOP / SEARCH / AVOID 切换
优先看：
- `arbiter_runtime.hpp/cpp`

### 想改超声波 GPIO / 交替测量
优先看：
- `ultrasonic_runtime.hpp/cpp`

## 6. 现有测试覆盖

- `test_arbiter_sm.cpp`
- `test_arbiter_runtime.cpp`
- `test_follower_runtime.cpp`
- `test_obstacle_runtime.cpp`

后续建议继续补 `ultrasonic_runtime` 单测，因为它仍然是控制侧状态最多、最容易藏边界问题的一块。
