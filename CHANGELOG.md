# CHANGELOG

本文档记录 `ROS2 Smart Follower` 的主要版本变更。

## Unreleased

## beta-0.3.2 - 2026-03-30

### Changed
- follower 转向侧从简单一阶平滑升级为轻量卡尔曼滤波，并开放 `steering_kalman.process_noise`、`steering_kalman.measurement_noise`、`steering_kalman.initial_covariance` 供现场热调
- 保留 `limits.v_max`、`limits.w_max`、`limits.dv_max`、`limits.dw_max` 作为最终输出硬限制，同时删除 `theta_deadzone`、大转角自动降速和 `max_target_speed_mps` 目标速度硬截断，减少控制链上与当前路线无关的额外束缚
- `try.md` 重写为现场调参与排查速查，补齐当前 `/robot1/follower_controller_node` 参数解释、在线调参命令和观测命令
- README 同步提升到 `beta-0.3.2`，补齐这一轮控制收口说明

### Verified
- VM 工作区 `/home/wheeltec/ros2_smart_follower` 已同步最新控制模块改动，并完成 `smart_follower_control` 单包编译通过

## beta-0.3.1 - 2026-03-29

### Changed
- 在 `beta-0.3.0` 稳定可跑的基础上，继续做技术路线收口：移除旧文档、旧模型与当前主线无关的历史说明，仓库只保留 Astra color+depth + YOLO/ReID/Tracker/Lock Manager 的固定路线
- perception 构建改为强制依赖 `astra_camera_msgs`，删除缺包时的旧兼容逻辑，运行链路统一以真实相机内参服务为准
- 历史 `monocular` 命名统一替换为 `camera` 命名；包括参数键、类型名、函数名和内部成员，减少当前 RGB-D 主线下的理解成本
- README、依赖说明、调参说明和新人文档同步提升到 `beta-0.3.1`，补齐本轮路线收敛与命名收口说明

### Removed
- 删除仓库根目录中与当前固定主线无关的旧说明文件：`test.md`、`install.md`、`当前推荐运行组合.md`、`技术路线收敛代码审查报告_beta-0.2.0.md`
- 删除已不再使用的旧 YOLO 模型 `models/yolo26n_static_256x320_simplify_e2e.onnx`

### Verified
- VM 工作区已完成同步并重新编译通过：`smart_follower_msgs`、`smart_follower_perception`、`smart_follower_control`、`smart_follower_bringup` 4 个包全部构建成功

## beta-0.3.0 - 2026-03-29

### Changed
- 默认运行模型切换为 `yolo26n_static_256x320_simplify_e2e_int8.onnx + osnet_x0_5_512.onnx`，并同步更新 launch / README / try 文档
- perception 默认处理节奏从 `process_every_n_frames=3` 调整为 `2`，更贴近当前车端输入节奏
- `depth_compare` 默认采样参数调整为 `sample_window_px=9`、`min_valid_samples=5`
- 锁定目标的深度定位从单窗采样收口为 **lower-body 多窗口采样 + median**，取消旧兼容采样链路
- follower 默认 `target_timeout` 从 `0.3s` 提升到 `1.2s`，短时无效深度或漏样本时更容易续上跟随
- 根目录 README、依赖说明、调参说明、推荐运行组合与新人文档统一提升到 `beta-0.3.0`

### Added
- perception 增加锁定目标定位失败日志，区分 `depth_frame_unavailable`、`depth_window_no_valid_samples`、`bbox_near_image_edge`、`depth_projection_rejected`
- follower 增加目标失效原因日志和 diagnostics 字段，便于定位 `target_timeout`、`locked_track_position_nan`、`locked_track_not_confirmed` 等状态
- 新增 Textual / 轻量 live dashboard 调试工具，并补齐相关可选依赖说明
- 新增 `调试命令表.md`，汇总编译、启动、话题排查和实车联调常用命令

### Fixed
- 跟踪模块补齐异常打印，并在 tracker 抛出异常时自动 reset，避免 perception 进程直接退出
- perception 启动阶段对 `/camera/get_camera_info` 的等待改为多次重试，降低相机节点略晚启动时的 configure 失败概率
- 小车容器内构建明确兼容 `/home/wheeltec/wheeltec_ros2/third_party/...` 依赖路径，避免因 `$HOME=/root` 导致 ONNX Runtime / libgpiod 漏检

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