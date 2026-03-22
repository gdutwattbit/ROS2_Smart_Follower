# CHANGELOG

本文档记录 `ROS2 Smart Follower` 的主要版本变化。

## alpha-0.1.3 - 2026-03-22

### Added
- 新增感知消息阶段细粒度 profiling 字段：`tf_lookup_ms`、`tf_transform_ms`、`message_fill_ms`。
- 新增并整理本轮性能相关文档：`当前推荐运行组合.md`、`YOLO_优化实施清单.md`、`ORT_线程调优测试计划.md`。

### Changed
- `perception_params.yaml` 默认模型路径切换到当前实机推荐组合：
  - `models/yolo26n_static_480x640_simplify_e2e.onnx`
  - `models/osnet_x0_5_512.onnx`
- 默认 YOLO 输入尺寸收口为 `640x480`，与相机真实输入规格一致。
- 感知异步检测任务与消息构建流程进一步收口到 `pipeline_utils.*`。
- ReID 裁剪流程移除不必要的 `clone()`，减少检测框图像拷贝。
- TF 使用改为“每帧 lookup 一次、每目标复用同一变换”，降低消息构建阶段重复开销。
- 感知日志与 diagnostics 现在可直接观察 TF 查询、点变换、消息填充的耗时分布。

### Verified
- 在虚拟机 `wheeltec@192.168.220.131` 上完成 `smart_follower_perception` 单包重新编译通过。
- 当前推荐运行组合与多轮 profiling 结果已汇总到根目录 `test.md`。

## alpha-0.1.2 - 2026-03-18

### Added
- 为 `smart_follower_bringup` 补充运行依赖 `ament_index_python`。
- 为感知 diagnostics 增加运行状态采样字段：输入年龄、最后一次 `person_pose` 发布时间、YOLO/ReID ready 状态。
- 为控制侧 runtime snapshot 增加目标年龄、超声波/深度输入年龄等字段，便于 diagnostics 使用。

### Changed
- `ReidExtractor::extract()` 增加 ONNX Runtime 异常兜底，推理失败时返回空特征并保持节点存活。
- 感知、跟随、避障、仲裁、超声波 diagnostics 改为实际输出 `OK / WARN / ERROR`，不再固定显示健康。
- CMake / package.xml 中的 lint 接入改为实际执行 `ament_cmake_lint_cmake` 与 `ament_cmake_xmllint`。
- lint 策略暂时不强行引入 `cpplint / flake8 / uncrustify`，避免历史风格债阻塞当前稳定性修复。

### Verified
- 在虚拟机 `wheeltec@192.168.220.131` 上完成 build/test 回归，结果为 **37 tests passed, 0 failures**。

## alpha-0.1.1 - 2026-03-17

### Added
- 控制侧新增公共头文件：`constants.hpp`、`lifecycle_utils.hpp`。
- 新增控制侧拆分模块：`arbiter_runtime.hpp/cpp`、`ultrasonic_runtime.hpp/cpp`、`follower_runtime.hpp/cpp`、`obstacle_runtime.hpp/cpp`。
- 新增 `test_arbiter_runtime.cpp`、`test_follower_runtime.cpp`、`test_obstacle_runtime.cpp`。
- 为 `TrackedPerson.msg` 和 `PersonPoseArray.msg` 补充 2D bbox / 3D pose 语义注释。

### Changed
- 控制侧四个 Lifecycle 节点开始复用最小公共骨架：版本常量、激活判断、频率转周期工具、统一 `main` 启动模板。
- `arbiter_node.cpp`、`ultrasonic_range_node.cpp`、`follower_controller_node.cpp`、`obstacle_avoidance_node.cpp` 进一步瘦身。
- 感知侧与控制侧运行时版本字符串统一提升到 `alpha-0.1.1`。

## alpha-0.1.0 - 2026-03-17

### Added
- 新增感知侧模块化头文件：`constants.hpp`、`frame_sync.hpp`、`geometry_utils.hpp`、`lock_manager.hpp`、`perception_diagnostics.hpp`、`perception_params.hpp`、`runtime.hpp`、`tracker.hpp`。
- 新增感知侧实现文件：`frame_sync.cpp`、`geometry_utils.cpp`、`lock_manager.cpp`、`runtime.cpp`、`tracker.cpp`。
- 新增感知单元测试：`test_frame_sync.cpp`、`test_lock_manager.cpp`、`test_tracker.cpp`。

### Changed
- 将 `smart_follower_perception/src/perception_node.cpp` 从大而全实现重构为“薄入口 + 多职责模块”。
- 保持外部接口不变：节点名、参数名、话题、消息、Lifecycle 行为、launch 用法均未修改。
- README 与 CHANGELOG 重写为 UTF-8 文档。

## alpha-0.0.4 - 2026-03-17

### Added
- 新增 `pyproject.toml`，用 `uv` 管理训练、导出、校验相关 Python 工具链。
- 补齐 `train` 依赖组：`scipy`、`opencv-python-headless`、`gdown`、`tensorboard`。

### Changed
- `torch` / `torchvision` 固定到官方 CPU-only 源，避免目标机误装 GPU 轮子。
- `scripts/install_dependencies.sh` 默认走轻量方案，避免目标机安装过重的 Python 导出依赖。

## alpha-0.0.3 - 2026-03-17

### Added
- 控制链路四个核心节点补齐运行时参数热更新。
- 新增项目级 `CHANGELOG.md`。

### Changed
- `uv` 工具链按“方案 B”落地：目标机优先 `validate / train`，完整 `export` 推荐本地执行。
- 超声波节点参数热更新改为“下一次 timer tick 重建资源”。

## alpha-0.0.2 - 2026-03-16

### Added
- 感知节点运行时参数热更新：支持在线重建订阅、发布器与模型配置。
- 控制、避障、仲裁、超声波节点运行时参数热更新。
- 超声波节点延迟重配置机制。

### Changed
- 感知、控制、避障、仲裁、超声波节点统一使用 `SingleThreadedExecutor`。
- 避障深度处理保留“最新帧缓存 + 20Hz 定时处理”模式。
- 超声波节点默认引脚保持已验证配置：左 `Trig=23 / Echo=24`，右 `Trig=4 / Echo=14`。

## alpha-0.0.1.2 - 2026-03-15

### Changed
- 感知同步链路从 `message_filters` 切换为“普通订阅 + 最近帧缓存 + 手工时间戳匹配”。
- 完成 `640x480@30fps` 管道审计，收紧缓存与检测频率配置。

## alpha-0.0.1.1 - 2026-03-14

### Changed
- ReID 主线从 `MobileNetV2-128` 切换为 `ResNet50-2048`。
- `TrackedPerson.msg` 外观特征维度升级为 `2048`。

## alpha-0.0.1 - 2026-03-14

### Added
- 初始 ROS2 Smart Follower Alpha 主链路：感知、控制、避障、仲裁、bringup、消息定义与基础文档。
