# smart_follower_bringup

`smart_follower_bringup` 负责 launch 组织、默认参数和模型路径装配。

## Files

- `launch/smart_follower.launch.py`: 默认启动入口
- `config/perception_params.yaml`: 感知默认参数
- `smart_follower_control/config/control_params.yaml`: 控制默认参数
- `smart_follower_control/config/control_params_robot1_debug.yaml`: `robot1` 调试覆盖参数

## Common Usage

完整 bringup：

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

仅启动本项目链路：

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 \
  bringup_robot:=false
```

显式覆盖模型：

```bash
ros2 launch smart_follower_bringup smart_follower.launch.py \
  yolo_model:=/abs/path/to/yolo.onnx \
  reid_model:=/abs/path/to/reid.onnx
```

## Notes

- launch 默认加载基础 control 参数和 `robot1` 调试 overlay。
- 模型路径来自 bringup 包安装后的 `models/` 目录，或由 launch 参数显式指定。
