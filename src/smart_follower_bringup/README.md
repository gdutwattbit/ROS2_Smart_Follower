# smart_follower_bringup

`smart_follower_bringup` 负责：
1. 组织 launch
2. 提供默认 YAML 参数

## 1. 主要文件

| 文件 | 作用 |
|---|---|
| `launch/smart_follower_only.launch.py` | 启动核心跟随链路 |
| `launch/smart_follower.launch.py` | 启动完整 bringup |
| `config/perception_params.yaml` | 感知默认参数 |
| `config/control_params.yaml` | 控制、避障、仲裁默认参数 |

## 2. 常用启动方式

### 仅启动核心跟随链路
```bash
ros2 launch smart_follower_bringup smart_follower_only.launch.py
```

### 启动完整 bringup
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

## 3. 当前默认假设

- 感知输入为彩色图像 `/camera/color/image_raw`
- 感知不再依赖 `camera_info` 与深度图
- 人体位置由单目估计生成
- 最终底盘输出仍为全局 `/cmd_vel`

## 4. 当前重点参数

- 模型默认放在 `models/`
- 感知模型路径在 `config/perception_params.yaml`
- 单目位置估计参数在 `monocular.*`
- 控制与避障参数在 `config/control_params.yaml`

## 5. 修改建议

- 改 launch 连接关系：优先改 launch 文件
- 改默认参数：优先改 YAML，不要把默认值散落回节点代码
