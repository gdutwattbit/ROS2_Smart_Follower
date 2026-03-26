# CHANGELOG

本文档记录 `ROS2 Smart Follower` 的主要版本变更。

## Unreleased

### Changed
- 路线收敛清理继续推进：删除 `smart_follower_only.launch.py`、旧模型导出/训练脚本、`pyproject.toml` 与相关安装入口
- `try.md`、`docs/新人上手指南.md`、`当前推荐运行组合.md` 已按当前 **depth compare 主线** 重写，不再沿用旧纯 RGB / 单目说明
- `scripts/install_dependencies.sh` 收口到当前运行主线，不再包含 uv / 训练 / 导出工具链逻辑
- 运行时版本字符串同步到 `beta-0.2.0`

### Verified
- VM 端重新覆盖部署、重新编译完成
- VM 端 mock color/depth + camera_info 注入烟测通过，确认 `/robot1/person_pose` 可发布
- VM 端 `colcon test` 通过：`48 tests, 0 errors, 0 failures, 0 skipped`

## beta-0.2.0 - 2026-03-24

### Changed
- 从 `dev-0.1.8` 收口到 `beta-0.2.0`：保留轻量模型、控制修复与调试工具链，同时恢复主链路 color+depth 定位用于实车对照
- perception 恢复 `/camera/depth/image_raw` 输入、color/depth 时间同步与 depth 取样定位；`/person_pose` 外部接口保持不变
- `smart_follower.launch.py` 默认打开 depth 与 depth registration，当前默认运行组合为 `yolo26n_static_256x320_simplify_e2e.onnx + osnet_x0_5_512.onnx`
- perception 节点修复 Astra `/camera/get_camera_info` 内参判定逻辑，现可正确加载真实 `fx/fy/cx/cy`，不再误回退到 fallback

### Added
- 新增 `depth_compare.*` 采样参数与 depth diagnostics（`depth_ready`、`depth_samples_valid`、`last_valid_depth_m`、`depth_position_ms` 等）

### Verified
- VM 端完整编译与测试通过（43 tests, 0 failures）
- 小车容器端已完成重新同步、重新编译与启动烟测；确认 ONNX Runtime 真正生效、Astra color/depth 主链路可正常启动
- 实车联调确认当前版本已可运行跟随；YOLO `intra=2` 实测为负优化，已恢复默认 `intra=1`

## dev-0.1.8 - 2026-03-24

### Changed
- 清理纯 RGB 主线下的兼容残留：删除 `monocular.person_height_m` 参数与无效消息字段 `TrackedPerson.velocity/depth_m`
- 将 ReID / `appearance_feature` 接口从历史兼容的 `2048` 维收口到当前真实使用的 `512` 维，移除运行时 padding 兼容链
- `smart_follower.launch.py` / `smart_follower_only.launch.py` 默认模型切换为 `yolo26n_static_256x320_simplify_e2e.onnx + osnet_x0_5_512.onnx`，并统一 color-only Astra 启动路径
- 控制侧默认跟随距离调整为 `0.6m`，同时补齐 `/robot1/follower_controller_node` 的 namespaced YAML，避免运行时回退到默认 `1.0m`
- `ArbiterRuntime` 移除“目标超时后永久 stop_latch”行为；现在仅 `ESTOP` 会锁停，目标恢复后可自动恢复跟随

### Added
- 根目录 `test.md` 补充 320x256 模型、线程组合与车端联调记录
- 根目录 `try.md` / `当前推荐运行组合.md` 同步更新当前推荐模型、线程与调参说明

## alpha-0.1.7 - 2026-03-23

### Changed
- 单目位置估计从“bbox 高度 + 人高假设”切换为 **bbox 底点地面投影**
- perception 节点新增 Astra 内参初始化逻辑：
  - `configure()` / 热更新时优先调用 `/camera/get_camera_info`
  - 服务失败时自动 fallback 到 `horizontal_fov_deg` 近似内参
- `PerceptionPipeline` / `pipeline_utils` 改为显式传递 `MonocularCameraIntrinsics`
- diagnostics 增加：
  - `intrinsics_ready`
  - `intrinsics_source`
  - `camera_fx / camera_fy / camera_cx / camera_cy`
  - `position_projection_ms`
  - `position_valid_count / position_invalid_count`
- `perception_params.yaml` 新增并启用：
  - `monocular.camera_info_service`
  - `monocular.camera_height_m`
  - `monocular.camera_pitch_deg`
  - `monocular.camera_x_offset_m`
  - `monocular.camera_y_offset_m`
  - `monocular.min_downward_angle_deg`
- 根目录新增 `try.md`，整理当前 YAML 参数的单位、作用与调参建议
- 运行时版本字符串统一提升到 `alpha-0.1.7`
- 各包 `package.xml` 版本统一提升到 `0.1.7`

### Added
- `smart_follower_perception/test/test_geometry_utils.cpp`

### Verified
- VM 端整库重新覆盖部署完成
- VM 端 `smart_follower_msgs + smart_follower_perception + smart_follower_control + smart_follower_bringup` 编译通过
- VM 端测试通过：`39 tests, 0 errors, 0 failures, 0 skipped`

## alpha-0.1.6 - 2026-03-23

### Changed
- 感知主链路从 RGB-D 改为纯 RGB，移除 `depth_topic` / `camera_info_topic`
- `FrameSynchronizer` 改为只缓存彩色图像
- `PerceptionPipeline` 改为纯彩色检测链路
- `TrackedPerson.position` 改由 bbox 做单目位置估计生成
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
- 将消息组装与 TF 处理进一步拆到 `pipeline_utils.*`

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
