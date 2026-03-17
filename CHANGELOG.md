# CHANGELOG

本文件记录 `ROS2 Smart Follower` 的主要版本变化。

## alpha-0.1.0 - 2026-03-17

### Added
- 新增感知侧模块化头文件：`constants.hpp`、`frame_sync.hpp`、`geometry_utils.hpp`、`lock_manager.hpp`、`perception_diagnostics.hpp`、`perception_params.hpp`、`runtime.hpp`、`tracker.hpp`。
- 新增感知侧实现文件：`frame_sync.cpp`、`geometry_utils.cpp`、`lock_manager.cpp`、`runtime.cpp`、`tracker.cpp`。
- 新增感知单元测试：`test_frame_sync.cpp`、`test_lock_manager.cpp`、`test_tracker.cpp`。

### Changed
- 将 `smart_follower_perception/src/perception_node.cpp` 从大而全实现重构为“薄入口 + 多职责模块”结构。
- 保持外部接口不变：节点名、参数名、话题、消息、Lifecycle 行为、launch 用法均未修改。
- `smart_follower_perception` 构建结构调整为可复用核心库 + 可执行文件，方便测试复用实现。
- README 与 CHANGELOG 重写为 UTF-8 文档，统一版本说明与使用说明。

### Verified
- 在虚拟机 `wheeltec@192.168.220.131` 上完成 `colcon build --symlink-install`。
- 在虚拟机 `wheeltec@192.168.220.131` 上完成 `colcon test`，总计 14 项测试通过、0 失败。
- 通过合成相机数据验证 `/robot1/person_pose` 发布链路可工作。

## alpha-0.0.4 - 2026-03-17

### Added
- 新增 `pyproject.toml`，用 `uv` 管理模型训练、导出、校验相关 Python 工具链。
- 补齐 `train` 依赖组：`scipy`、`opencv-python-headless`、`gdown`、`tensorboard`。
- README 增补方案 B：目标机优先使用 `validate` / `train`，完整 `export` 推荐放在本地开发机执行。

### Changed
- `torch` / `torchvision` 固定到官方 PyTorch CPU-only 源，避免目标机误装 GPU 轮子。
- `scripts/install_dependencies.sh` 默认走轻量方案，避免目标机安装过重的 Python 导出依赖。
- 运行时日志版本字符串统一提升到 `alpha-0.0.4`。

### Verified
- 虚拟机已完成 CPU-only `train` 组安装，验证结果为 `torch 2.10.0+cpu`、`torchvision 0.25.0+cpu`。
- 清理旧 GPU-heavy `.venv` 与 uv 缓存后，空间占用明显下降。
- `uv run --group train ... --help` 与 `uv run --group validate ...` 可正常运行。

## alpha-0.0.3 - 2026-03-17

### Added
- 控制链四个核心节点补齐运行时参数热更新：`follower_controller_node`、`arbiter_node`、`obstacle_avoidance_node`、`ultrasonic_range_node`。
- 新增项目级 `CHANGELOG.md`，统一记录版本演进。
- README 补充稳定性说明、虚拟机自检结果和运行时调参注意事项。

### Changed
- `uv` 工具链按“方案 B”落地：目标机优先 `validate` / `train`，完整 `export` 改为推荐本地执行。
- 超声波节点参数热更新改为“参数回调登记 + 下一次 timer tick 重建资源”，避免 active 状态下阻塞。
- 运行时日志版本字符串统一修正到 `alpha-0.0.3`。

### Verified
- 在虚拟机上重新编译 `smart_follower_control` 通过。
- 已验证多项运行时参数热更新命令可成功返回并生效。

## alpha-0.0.2 - 2026-03-16

### Added
- 感知节点运行时参数热更新：支持在线重建订阅、发布器与模型配置。
- 控制、避障、仲裁、超声波节点运行时参数热更新。
- 超声波节点延迟重配置机制，降低在线改 GPIO 时的阻塞风险。
- 模型路径自动解析逻辑，兼容不同工作目录启动。

### Changed
- 控制、感知、避障、仲裁、超声波节点统一使用 `SingleThreadedExecutor`，降低生命周期与定时器交错风险。
- 避障深度处理保留“最新帧缓存 + 20Hz 定时处理”模式，避免 30fps 深度图逐帧全量处理。
- 感知链路中非检测帧不再计入 miss，锁定保持/切换时序按参数真正生效。
- 超声波节点默认引脚保持已验证配置：左 `Trig=23 / Echo=24`，右 `Trig=4 / Echo=14`。

### Fixed
- 修复感知节点相对模型路径在不同启动目录下失效的问题。
- 修复 ReID 记忆恢复后未复用原 track id 的闭环问题。
- 修复运行时改参数后旧订阅器、旧定时器不重建的问题。
- 修复超声波节点在线修改参数时可能导致 `ros2 param set` 卡住的问题。

### Verified
- 在虚拟机上完成 `smart_follower_control` 编译通过。
- `smart_follower_only.launch.py` 启动后，控制/仲裁/避障/超声波/感知节点均可拉起。
- 已验证多项热更新命令可成功返回。

## alpha-0.0.1.2 - 2026-03-15

### Changed
- 感知同步链路从 `message_filters` 切换为“普通订阅 + 最近帧缓存 + 手工时间戳匹配”。
- 完成 `640x480@30fps` 管道审计，收紧缓存与检测频率配置。

### Added
- 增加 `dev-0.0.1.1` 调试快照说明与关键日志定位信息。

## alpha-0.0.1.1 - 2026-03-14

### Changed
- ReID 主线从 MobileNetV2-128 切换为 ResNet50-2048。
- `TrackedPerson.msg` 外观特征维度升级为 `2048`。

### Added
- 接入训练、导出、ONNX 校验脚本，形成 ResNet50-ReID 工具链。

## alpha-0.0.1 - 2026-03-13

### Added
- 首个 Alpha 主链路落地：感知 → 跟随控制 → 避障 → 仲裁。
- 引入自定义消息、Lifecycle 节点、统一 bringup、键盘控制与基础文档。
