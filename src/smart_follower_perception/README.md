# smart_follower_perception

`smart_follower_perception` 是整条智能跟随链路里的“感知入口”。
它负责把 RGB / Depth / CameraInfo 输入整理成同步帧，完成目标检测、ReID、跟踪、锁定逻辑，并发布 `/robot_ns/person_pose`。

## 1. 这个包解决什么问题

- 从 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` 接收输入
- 手工时间戳匹配三路输入，替代 `message_filters`
- 运行 YOLO + ReID ONNX 推理
- 基于深度和 TF 生成 `base_footprint` 下的目标位置
- 维护轨迹、短期记忆库、锁定/丢失/重识别状态
- 发布 `smart_follower_msgs/PersonPoseArray`

## 2. 推荐阅读顺序

如果你是第一次接手，建议按下面顺序读：

1. `src/perception_node.cpp`
   - 看 ROS2 Lifecycle、订阅/发布器、参数回调、异步 worker 入口
2. `src/perception_pipeline.cpp`
   - 看一帧感知任务如何被组织、调度、收尾
3. `src/perception_params.cpp`
   - 看参数声明、加载、热更新时哪些模块要重建
4. `src/perception_diagnostics.cpp`
   - 看 profiling 和 diagnostics 如何组织
5. `src/runtime.cpp`
   - 看 YOLO / ReID 模型加载、预处理、推理与错误兜底
6. `src/tracker.cpp`
   - 看轨迹、匹配、EMA、记忆库
7. `src/lock_manager.cpp`
   - 看显式锁定、短失联保持、切人策略
8. `src/frame_sync.cpp`
   - 看最近帧缓存和手工同步逻辑
9. `src/geometry_utils.cpp`
   - 看深度采样与 TF 坐标变换

## 3. 当前文件职责

| 文件 | 作用 |
|---|---|
| `perception_node.cpp` | 薄节点入口：Lifecycle、接口装配、热更新 glue code |
| `perception_pipeline.cpp` | 串起一帧感知流程与异步任务边界 |
| `perception_params.cpp` | 参数声明、加载、校验、重建策略 |
| `perception_diagnostics.cpp` | diagnostics 与 profiling 汇总 |
| `runtime.cpp` | YOLO / ReID ONNX Runtime 封装 |
| `frame_sync.cpp` | RGB/Depth/CameraInfo 缓存匹配 |
| `tracker.cpp` | 多目标跟踪与记忆库 |
| `lock_manager.cpp` | 锁定/解锁/丢失/恢复逻辑 |
| `geometry_utils.cpp` | 深度与 TF 几何转换 |
| `pipeline_utils.cpp` | 感知异步任务、profiling 辅助与消息拼装细节 |

## 4. 维护时优先遵守的边界

- **不要**把检测、跟踪、TF、锁定逻辑再塞回 `perception_node.cpp`
- 模型输入输出、预处理、ORT 线程设置，优先改 `runtime.cpp`
- 手工同步窗口、缓存策略，优先改 `frame_sync.cpp`
- 锁人策略只在 `lock_manager.cpp` 调整
- 如果只是改 diagnostics / profiling，不要碰主算法路径

## 5. 常见排查入口

### `/person_pose` 没消息
先看：
- 节点是否 `active`
- 实际话题是不是 `/robot1/person_pose`
- 三路输入是否都在发，且时间戳接近
- `frame_sync.cpp` 是否持续成功拼出 triplet

### 模型加载失败
先看：
- `yolo.model_path`
- `reid.model_path`
- ONNX Runtime SDK 是否被 CMake 正确找到
- diagnostics 里 `yolo_ready` / `reid_ready`

### 耗时太高
优先看 profiling：
- `yolo_ms`
- `reid_ms`
- `tf_lookup_ms`
- `tf_transform_ms`
- `message_fill_ms`
- `total_ms`

## 6. 本包测试

- `test_frame_sync.cpp`：同步缓存与超时淘汰
- `test_lock_manager.cpp`：锁定/失联/恢复策略
- `test_tracker.cpp`：跟踪代价矩阵与拒配阈值
- `test_assignment.cpp`：历史分配逻辑回归

如果你要继续做性能优化，建议先从根目录的：
- `test.md`
- `当前推荐运行组合.md`
- `YOLO_优化实施清单.md`
开始。
