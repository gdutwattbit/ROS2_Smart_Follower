# CHANGELOG

本文件记录 `ROS2 Smart Follower` 的主要版本变更。

## Unreleased

## alpha-0.1.6 - 2026-03-23

### Changed
- 感知主链路从 RGB-D 改为纯 RGB，移除 `depth_topic` / `camera_info_topic`
- `FrameSynchronizer` 改为只缓存彩色图像
- `PerceptionPipeline` 改为纯彩色检测链路
- `TrackedPerson.position` 改由 bbox 做单目位置估计生成
- `TrackedPerson.depth_m` 保留兼容字段，无真实深度时写入 `NaN`
- `Tracker` 去掉深度代价与深度 gating，状态维度由 10 维收缩到 8 维
- `obstacle_avoidance_node` / `ObstacleRuntime` 移除深度图依赖，仅保留左右超声波避障
- perception / control / bringup 的 YAML、diagnostics、测试、README 同步清理
- 统一运行时版本字符串到 `alpha-0.1.6`，并将各包 `package.xml` 版本提升到 `0.1.6`

### Verified
- VM 端 `smart_follower_msgs + smart_follower_perception + smart_follower_control + smart_follower_bringup` 编译通过
- VM 端全量相关测试通过：`42 tests, 0 failures`
- `smart_follower_only.launch.py` 烟测可启动

## alpha-0.1.5 - 2026-03-23

### Changed
- 回退统一缓冲区 / 多 worker 实验，恢复 `alpha-0.1.4` 的单 worker 单帧缓冲主线
- 保留 `runtime` 热路径对象复用、YOLO 预处理减拷贝与 profiling 拆分
- 默认推荐 YOLO ORT 线程参数调整为 `intra_op_num_threads = 3`
- 补充测试记录与当前推荐运行组合文档

## alpha-0.1.4 - 2026-03-22

### Added
- 感知侧拆出 `perception_pipeline.*`
- 控制侧拆出 `control_node_common.*`
- 包级 README：
  - `src/smart_follower_perception/README.md`
  - `src/smart_follower_control/README.md`
  - `src/smart_follower_bringup/README.md`
- `docs/新人上手指南.md`
- diagnostics 辅助实现拆分到 `.cpp`

### Changed
- `perception_node.cpp` 收缩为更薄的 Lifecycle 节点入口
- arbiter / follower / obstacle / ultrasonic 节点做了第一轮可读性拆分
- 统一了一批重复的生命周期 / 参数热更新样板

## alpha-0.1.3 - 2026-03-22

### Added
- 更细粒度 profiling：`tf_lookup_ms`、`tf_transform_ms`、`message_fill_ms` 等
- 补充测试记录与 ORT / YOLO 优化文档

### Changed
- 默认模型组合切到：
  - `models/yolo26n_static_480x640_simplify_e2e.onnx`
  - `models/osnet_x0_5_512.onnx`
- YOLO 输入固定到 `640x480`
- 将消息组装与 TF 处理进一步拆分到 `pipeline_utils.*`

## alpha-0.1.2 - 2026-03-18

### Added
- `smart_follower_bringup` 补齐 `ament_index_python` 依赖
- diagnostics 增加更多 ready / timeout / publish 观测

### Changed
- `ReidExtractor::extract()` 增加 ONNX Runtime 异常兜底
- diagnostics 等级改为 `OK / WARN / ERROR`
- CMake / package.xml 补齐 lint 相关配置

## alpha-0.1.1 - 2026-03-17

### Changed
- 修正第一轮版本对齐问题
- 控制侧和感知侧统一到 `alpha-0.1.1` 口径

## alpha-0.1.0 - 2026-03-16

### Added
- 完成 `perception_node.cpp` 的职责拆分重构第一轮交付
- 增加 `runtime / frame_sync / tracker / lock_manager / geometry / params / diagnostics` 模块化结构
