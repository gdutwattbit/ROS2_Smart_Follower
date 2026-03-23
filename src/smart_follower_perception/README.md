# smart_follower_perception

`smart_follower_perception` 是项目的感知包，当前负责 **纯 RGB 人体感知链路**：

```text
color image -> YOLO -> ReID -> Tracker -> Lock Manager -> monocular position -> /person_pose
```

## 1. 主要文件

- `src/perception_node.cpp`
  - ROS2 Lifecycle 节点入口
  - 负责订阅、发布、生命周期与参数热更新 glue code
- `src/perception_pipeline.cpp`
  - 异步检测 worker 主线
  - 连接 runtime / tracker / lock / publish
- `src/runtime.cpp`
  - YOLO / ReID 模型加载与 ONNX Runtime 推理
- `src/tracker.cpp`
  - 多目标跟踪、匹配、记忆库
- `src/lock_manager.cpp`
  - 锁定、解锁、切人策略
- `src/pipeline_utils.cpp`
  - `/person_pose` 消息组装
- `src/geometry_utils.cpp`
  - 单目位置估计

## 2. 当前输入输出

### 输入
- `/camera/color/image_raw`
- `/follow_command`

### 输出
- `/person_pose`

## 3. 当前关键约束

- 不再依赖深度图和 `camera_info`
- `TrackedPerson.position` 由 bbox 做单目估计得到
- `TrackedPerson.depth_m` 保留兼容字段；当前无真实深度输入时写 `NaN`
- 外部消息、话题、launch 用法保持不变

## 4. 默认参数重点

主要看：
- `src/smart_follower_bringup/config/perception_params.yaml`

其中最关键的是：
- `yolo_model_path`
- `reid_model_path`
- `process_every_n_frames`
- `yolo_ort.*`
- `reid_ort.*`
- `monocular.*`

## 5. 如果你要改哪里

- 改模型推理：看 `runtime.*`
- 改跟踪策略：看 `tracker.*`
- 改锁人逻辑：看 `lock_manager.*`
- 改消息发布：看 `pipeline_utils.*`
- 改位置估计：看 `geometry_utils.*`

## 6. 现阶段注意事项

- 单目距离估计对 bbox 高度比较敏感
- `monocular.person_height_m`、`horizontal_fov_deg`、`camera_x/y_offset_m` 建议结合实车再校一次
- 如果模型路径不在 `models/` 目录，要同步修改 YAML
