# smart_follower_bringup

`smart_follower_bringup` 负责：
1. 组织 launch
2. 提供默认 YAML 参数
3. 固化当前主线模型与相机启动参数

## 1. 主要文件

| 文件 | 作用 |
|---|---|
| `launch/smart_follower.launch.py` | 当前唯一正式启动入口 |
| `config/perception_params.yaml` | 感知默认参数 |
| `config/control_params.yaml` | 控制、避障、仲裁默认参数 |

## 2. 常用启动方式

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

## 3. 当前默认假设

- 感知输入为：
  - `/camera/color/image_raw`
  - `/camera/depth/image_raw`
- 感知主链路依赖：
  - `/camera/get_camera_info`
- 人体位置由 depth compare 生成
- 最终底盘输出仍为全局 `/cmd_vel`

## 4. 当前重点参数

- 感知模型路径在 `config/perception_params.yaml`
- depth compare 参数也在 `config/perception_params.yaml`
- 控制与避障参数在 `config/control_params.yaml`
- launch 可通过参数控制：
  - `robot_ns`
  - `bringup_robot`
  - `bringup_camera`

## 5. 修改建议

- 改 launch 连接关系：优先改 launch 文件
- 改默认参数：优先改 YAML，不要把默认值散落回节点代码
