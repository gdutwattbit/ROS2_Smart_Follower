# CHANGELOG

## alpha-0.0.4 - 2026-03-17

### Added
- 新增 `pyproject.toml`，用 `uv` 管理模型训练/导出/校验 Python 工具链。
- `reid/train` 依赖组补齐 `scipy`、`opencv-python-headless`、`gdown`、`tensorboard`，保证 `torchreid` 训练脚本可直接启动。
- README 增补方案 B：目标机优先使用 `validate` / `train`，完整 `export` 放本地开发机执行。

### Changed
- `uv` 中的 `torch` / `torchvision` 固定到官方 PyTorch CPU-only index，避免虚拟机或主控误装 CUDA / NVIDIA 轮子。
- `scripts/install_dependencies.sh` 默认按方案 B 安装 Python 工具链：`--with-training` 仅安装 `train` 组，不再默认拉完整 `export`。
- 运行时日志版本字符串统一提升到 `alpha-0.0.4`。

### Verified
- 虚拟机 `wheeltec@192.168.220.131` 已完成 CPU-only `train` 组安装，验证结果为 `torch 2.10.0+cpu`、`torchvision 0.25.0+cpu`。
- 清理旧 GPU-heavy `.venv` 和 uv 缓存后，虚拟机空间从 `.venv 7.4G / cache 2.4G` 收敛到 `.venv 1.4G / cache 56M`。
- `uv run --group train python src/smart_follower_perception/scripts/train_reid_resnet50.py --help` 与 `uv run --group validate ...` 均已验证可用。

## alpha-0.0.3 - 2026-03-17

### Added
- `uv` 的 `reid/train` 依赖补齐 `tensorboard`，修复 `torchreid` 引擎初始化阶段缺少 `SummaryWriter` 后端的问题。
- `uv` 的 `reid/train` 依赖补齐 `gdown`，修复 `torchreid` 数据集模块导入时缺少 `gdown` 的问题。
- `uv` 的 `reid/train` 依赖补齐 `opencv-python-headless`，修复 `torchreid` 导入时缺少 `cv2` 的问题。
- `uv` 的 `reid/train` 依赖补齐 `scipy`，修复 `torchreid` 导入时因缺少 `scipy.io` 导致脚本连 `--help` 都无法运行的问题。
- 控制链四个核心节点补齐运行时参数热更新：`follower_controller_node`、`arbiter_node`、`obstacle_avoidance_node`、`ultrasonic_range_node`。
- README 补充本轮稳定性说明、虚拟机自检结果和运行时调参注意事项。
- 新增项目级 `CHANGELOG.md`，统一记录版本演进。

### Changed
- `uv` 中的 `torch` / `torchvision` 固定到官方 PyTorch CPU-only index，避免虚拟机或主控误装 CUDA / NVIDIA 轮子。
- 补充说明 `uv train/export` 组在 Linux 下可能拉取较大的 PyTorch 轮子，文档中已明确推荐目标机优先使用 `validate` / `train`，完整 `export` 放本地机执行。
- 运行时日志版本字符串统一修正到 `alpha-0.0.3`。
- `uv` 工具链按“方案 B”落地：目标机默认只保留 `validate` / `train`，完整 `export` 改为推荐在本地开发机执行。
- 超声波节点参数热更新改为“参数回调登记 + 下一次 timer tick 重建资源”，避免 active 状态下直接重建 GPIO 资源导致的服务阻塞。
- 本轮发布版本提升为 `alpha-0.0.3`，保留 `alpha-0.0.2` 作为上一可用快照。

### Verified
- 在虚拟机 `wheeltec@192.168.220.131` 上重新编译 `smart_follower_control` 通过。
- 已验证以下运行时热更新命令返回成功并生效：
  - `ros2 param set /robot1/follower_controller_node target_timeout 0.40`
  - `ros2 param set /robot1/arbiter_node search_angular_speed 0.40`
  - `ros2 param set /robot1/obstacle_avoidance_node depth_sample_stride 4`
  - `ros2 param set /robot1/ultrasonic_range_node rate 8.0`

## alpha-0.0.2 - 2026-03-16

### Added
- 感知节点运行时参数热更新：支持在线重建订阅/发布接口与模型配置。
- 控制节点运行时参数热更新：`follower_controller_node`、`arbiter_node`、`obstacle_avoidance_node`、`ultrasonic_range_node`。
- 超声波节点延迟重配置机制：active 状态下参数回调只登记请求，在下一次定时器 tick 完成 GPIO / publisher / timer 重建，避免服务阻塞。
- 模型路径自动解析逻辑，兼容从不同工作目录启动时的 `models/*.onnx` 查找。

### Changed
- 控制、感知、避障、仲裁、超声波节点统一使用 `SingleThreadedExecutor`，降低生命周期和定时器交错风险。
- `follower_controller_node` 增加目标超时失效保护，丢失目标时主动清零并重置 PID。
- `arbiter_node`、`obstacle_avoidance_node`、`ultrasonic_range_node` 在 deactivate 时先输出安全状态，再停用 publisher。
- 超声波节点默认引脚保持已验证配置：左 `Trig=23 / Echo=24`，右 `Trig=4 / Echo=14`。
- 避障深度处理保留“最新帧缓存 + 20Hz 定时处理”模式，避免 30fps 深度图逐帧全量处理。
- 感知链路中非检测帧不再计入 miss，跟踪输出速度改为 0，锁定保持/切换时序按参数真正生效。

### Fixed
- 修复感知节点相对模型路径在不同启动目录下失效的问题。
- 修复 ReID 记忆恢复后未复用原 track id 的闭环问题。
- 修复运行时改参数后旧订阅器/旧定时器不重建的问题。
- 修复超声波节点在线修改参数时可能导致 `ros2 param set` 卡住的问题。

### Verified
- 在虚拟机 `wheeltec@192.168.220.131` 上完成 `smart_follower_control` 编译通过。
- `smart_follower_only.launch.py` 启动后，控制/仲裁/避障/超声波/感知节点均可拉起。
- 已验证以下热更新命令可成功返回：
  - `ros2 param set /robot1/follower_controller_node target_timeout 0.40`
  - `ros2 param set /robot1/arbiter_node search_angular_speed 0.40`
  - `ros2 param set /robot1/obstacle_avoidance_node depth_sample_stride 4`
  - `ros2 param set /robot1/ultrasonic_range_node rate 8.0`

## alpha-0.0.1.2 - 2026-03-15

- 感知同步链路从 `message_filters` 切换为“普通订阅 + 最近帧缓存 + 手工时间戳匹配”。
- 完成 640x480@30fps 管道审计，收紧缓存与检测频率配置。
- 加入 `dev-0.0.1.1` 调试快照日志说明。

## alpha-0.0.1.1 - 2026-03-14

- ReID 主线从 MobileNetV2-128 切换为 ResNet50-2048。
- `TrackedPerson.msg` 外观特征维度升级为 2048。
- 完成训练/导出脚本接入与 ONNX 验证脚本。

## alpha-0.0.1 - 2026-03-13

- 首个 Alpha 主链路落地：感知 → 跟随控制 → 避障 → 仲裁。
- 引入自定义消息、Lifecycle 节点、统一 bringup、键盘控制与基础文档。
