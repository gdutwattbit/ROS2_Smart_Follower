# smart_follower_bringup

`smart_follower_bringup` 只做两件事：
1. 统一 launch 入口
2. 统一参数文件管理

如果你要“把系统跑起来”，这一包通常是第一站；如果你要“改算法”，这一包通常只需要确认参数与启动方式，不需要在这里写业务逻辑。

## 1. 主要文件

| 文件 | 作用 |
|---|---|
| `launch/smart_follower_only.launch.py` | 只启动本项目相关节点 |
| `launch/smart_follower.launch.py` | 联合现有底盘/相机 bringup 一起启动 |
| `config/perception_params.yaml` | 感知默认参数 |
| `config/control_params.yaml` | 控制、避障、仲裁、超声波默认参数 |

## 2. 什么时候用哪个 launch

### 只验证本项目链路
```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py
```
适合：
- 虚拟机自检
- bag / synthetic 数据注入
- 单独调算法参数

### 联合底盘和相机一起起
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```
适合：
- 实机联调
- 与 `turn_on_wheeltec_robot` 联动

## 3. 命名空间约定

默认命名空间是 `robot1`，所以很多内部话题实际会变成：
- `/robot1/person_pose`
- `/robot1/cmd_vel_follow`
- `/robot1/cmd_vel_avoid`
- `/robot1/follow_command`

但最终底盘控制仍保持全局：
- `/cmd_vel`

这是为了兼容现有底盘驱动。

## 4. 启动前最好确认的事

- 模型文件是否在 `models/` 下
- 相机深度话题是否真的是 `/camera/depth/image_raw`
- D2C 是否已经打开
- TF 链是否包含 `base_footprint -> camera_link -> camera_color_optical_frame`
- GPIO 引脚配置是否与 `control_params.yaml` 一致

## 5. 修改 bringup 时的原则

- launch 里尽量只做装配，不写业务逻辑
- 参数默认值尽量放 YAML，不要散落在 launch 里硬编码
- 如果新增节点，优先保持与现有命名空间风格一致
