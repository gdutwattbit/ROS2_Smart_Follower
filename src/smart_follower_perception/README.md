# smart_follower_perception

`smart_follower_perception` 是当前项目的感知包，负责这条纯 RGB 主链路：

```text
color image -> YOLO -> ReID -> Tracker -> Lock Manager -> monocular position -> /person_pose
```

## 1. 主要文件

- `src/perception_node.cpp`
  - ROS2 Lifecycle 节点入口
  - 负责订阅、发布、生命周期与参数热更新 glue code
- `src/perception_pipeline.cpp`
  - 异步 detection worker 主线
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

## 2. 输入输出

### 输入
- `/camera/color/image_raw`
- `/follow_command`

### 输出
- `/person_pose`

## 3. 当前关键约束

- 主链路只依赖彩色图，不再使用深度图
- 节点在 `configure()` / 热更新时会优先调用 `/camera/get_camera_info` 获取真实内参
- 若服务不可用，则自动退回到 `horizontal_fov_deg` fallback 内参
- `TrackedPerson.position` 是 `base_frame` 下的位置估计结果
- `TrackedPerson.appearance_feature` 已收口为当前 ReID 主线真实使用的 `512` 维

## 4. 默认参数重点

主要看：
- `src/smart_follower_bringup/config/perception_params.yaml`

其中最关键的是：
- `yolo.model_path`
- `reid.model_path`
- `process_every_n_frames`
- `yolo.ort.*`
- `reid.ort.*`
- `monocular.*`

## 5. 如果你要改哪里

- 改模型推理：看 `runtime.*`
- 改跟踪策略：看 `tracker.*`
- 改锁人逻辑：看 `lock_manager.*`
- 改消息发布：看 `pipeline_utils.*`
- 改位置估计：看 `geometry_utils.*`

## 6. 现阶段注意事项

- 单目位置估计对 `camera_height_m`、`camera_pitch_deg` 很敏感
- `horizontal_fov_deg` 只在拿不到真实内参时才参与 fallback
- 如果模型路径不在 `models/` 目录，要同步修改 YAML
