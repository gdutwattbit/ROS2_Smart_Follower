# 技术路线收敛代码审查报告（beta-0.2.0）

## 1. 审查目的

本报告用于在 **beta-0.2.0** 之后，对仓库进行“路线收敛”清理：

- 以当前已验证可运行的主线为唯一目标
- 识别与当前技术路线**无关、重复、历史兼容、实验性残留**的代码/资源/文档
- 为后续“删代码而不伤主链”提供分批执行清单

本报告**不直接删文件**，只给出审查结论、删除优先级与风险说明。

---

## 2. 当前固定技术路线（作为清理基准）

当前建议视为唯一主线：

1. **感知模型**
   - YOLO：`models/yolo26n_static_256x320_simplify_e2e.onnx`
   - ReID：`models/osnet_x0_5_512.onnx`
   - ORT：YOLO `intra=1 / inter=1 / sequential`

2. **感知输入链路**
   - color：`/camera/color/image_raw`
   - depth：`/camera/depth/image_raw`
   - 定位：**depth compare**（bbox 下半部窗口采样 + median）
   - Astra `/camera/get_camera_info` 为真实内参主来源

3. **控制链路**
   - `/person_pose -> follower_runtime -> cmd_vel_follow`
   - 超声波避障保留
   - arbiter 保留
   - 当前控制侧预测补帧保留

4. **发布入口**
   - 主入口：`smart_follower.launch.py`
   - 当前版本已验证可运行跟随

凡是不服务于上述主线的内容，都应视为收敛清理对象。

---

## 3. 总体审查结论

当前仓库里的“非主线残留”主要分成 4 类：

1. **纯单目时代的代码残留**
   - 仍保留了旧的 bbox 底点地面投影 API、参数、测试与文档描述
   - 其中一部分已经不再参与当前主运行链路

2. **模型导出/训练实验残留**
   - 仓库内仍保留训练、导出、验证脚本与历史模型资产
   - 它们不属于当前实车运行必需项

3. **历史实验资产/文档残留**
   - 例如 480x640 模型、旧线程调优文档、旧结论记录
   - 对新人理解当前主线会产生噪声

4. **说明文档与版本字符串漂移**
   - 部分包级 README 仍写着“纯 RGB / 单目”
   - 代码内运行时版本字符串仍停留在 `alpha-0.1.7`

结论：

> 当前仓库已经具备“运行主线”，但还没有完成“仓库收敛”。
> 如果目标是让新人快速上手、减少误读，下一步应该先删 **确定无用** 的内容，再收缩 **条件性保留** 的兼容逻辑。

---

## 4. 建议保留的核心模块（不要删）

以下内容明确属于当前主线，**不建议动**：

### 4.1 perception 主链路
- `src/smart_follower_perception/src/runtime.cpp`
- `src/smart_follower_perception/src/perception_node.cpp`
- `src/smart_follower_perception/src/perception_pipeline.cpp`
- `src/smart_follower_perception/src/pipeline_utils.cpp`
- `src/smart_follower_perception/src/frame_sync.cpp`
- `src/smart_follower_perception/src/tracker.cpp`
- `src/smart_follower_perception/src/lock_manager.cpp`
- `src/smart_follower_perception/src/geometry_utils.cpp` 中的 **depth compare 路径**

### 4.2 control 主链路
- `follower_runtime.*`
- `obstacle_runtime.*`
- `arbiter_runtime.*`
- `ultrasonic_runtime.*`
- 对应 node 文件与测试

### 4.3 bringup 主入口
- `src/smart_follower_bringup/launch/smart_follower.launch.py`
- `src/smart_follower_bringup/config/perception_params.yaml`
- `src/smart_follower_control/config/control_params.yaml`

### 4.4 当前模型资产
- `models/yolo26n_static_256x320_simplify_e2e.onnx`
- `models/osnet_x0_5_512.onnx`

---

## 5. P0：可以直接删除的内容（低风险、强建议）

这部分几乎不影响当前主运行链路，建议优先清理。

| 路径 | 现状判断 | 建议动作 | 风险 |
|---|---|---|---|
| `yolo26n_static_480x640_simplify_e2e.onnx`（仓库根目录） | 当前运行完全不引用；属于历史模型残留，且位置也不在 `models/` 主目录 | **删除** | 低 |
| `osnet_x0_5_traced.pt`（仓库根目录） | 当前运行不使用；属于训练/导出中间资产 | **删除** | 低 |
| `src/smart_follower_perception/scripts/__pycache__/` | Python 缓存文件，不应入库 | **删除** | 低 |
| `src/smart_follower_bringup/launch/__pycache__/` | Python 缓存文件，不应入库 | **删除** | 低 |
| `src/smart_follower_perception/src/geometry_utils.cpp` 中 `estimate_person_position_from_bbox(...)` | 当前主链路已不再调用；仅旧单目测试仍在使用 | **删除函数与声明** | 低 |
| `src/smart_follower_perception/include/smart_follower_perception/geometry_utils.hpp` 中对应旧单目 API 声明 | 与上条绑定 | **删除声明** | 低 |
| `src/smart_follower_perception/test/test_geometry_utils.cpp` 中仅覆盖旧单目投影的测试用例 | 已不覆盖主链路 | **删除旧单目测试，保留 depth compare 测试** | 低 |

### P0 说明
P0 的目标不是“重构”，而是先把 **真正没被当前主链路调用** 的内容拿掉，降低仓库噪声。

---

## 6. P1：建议删除或收缩的内容（需要先确认路线是否彻底硬绑定）

这部分不是完全无用，但已经偏向“兼容逻辑”而非“主线逻辑”。

### 6.1 单目 fallback 内参链路
涉及位置：
- `perception_params.yaml` 中：
  - `monocular.horizontal_fov_deg`
  - `monocular.min_downward_angle_deg`
  - `monocular.min_range_m`
  - `monocular.max_range_m`
- `perception_node.cpp` 中：
  - `make_fallback_camera_intrinsics(...)`
  - `apply_fallback_intrinsics(...)`
  - `update_fallback_intrinsics_from_image(...)`
  - fallback 相关日志与 diagnostics
- `geometry_utils.*` 中的单目投影公式相关逻辑

#### 审查意见
如果后续路线明确为：

> **只支持 Astra + depth compare + `/camera/get_camera_info`**

那么上述 fallback 逻辑可以考虑大幅收缩，甚至删除。

#### 但当前不建议一刀切删除的原因
当前 depth compare 仍复用：
- `camera_x_offset_m`
- `camera_y_offset_m`
- 内参结构体

所以更合理的做法是：

**不是直接删整个 `monocular` 结构，而是重命名并瘦身。**

建议后续改成例如：
- `camera_projection.camera_info_service`
- `camera_projection.camera_x_offset_m`
- `camera_projection.camera_y_offset_m`

把“旧单目语义”从参数命名上清掉。

#### 建议动作
- **保留**：`camera_info_service`、`camera_x_offset_m`、`camera_y_offset_m`
- **评估后删除**：`horizontal_fov_deg`、`min_downward_angle_deg`、旧单目投影路径、相关旧文档

风险：中

---

### 6.2 `smart_follower_only.launch.py`
路径：
- `src/smart_follower_bringup/launch/smart_follower_only.launch.py`

#### 审查意见
它是一个“核心链路快捷启动入口”，不是历史废件；但如果你们决定：

> 仓库只保留一个官方入口，避免新人分不清用哪个 launch

那么它可以删除。

#### 建议动作
二选一：
1. **保留**，但明确标注为“调试/最小链路入口”
2. **删除**，统一只保留 `smart_follower.launch.py`

风险：低

---

### 6.3 模型导出/训练工具链
涉及路径：
- `src/smart_follower_perception/scripts/export_yolo_onnx.py`
- `src/smart_follower_perception/scripts/export_reid_onnx.py`
- `src/smart_follower_perception/scripts/train_reid_resnet50.py`
- `src/smart_follower_perception/scripts/validate_reid_onnx.py`
- `pyproject.toml`
- `DEPENDENCIES.md` 中与 export/train/validate 相关部分

#### 审查意见
这些脚本对“当前车上跑起来”没有帮助，属于模型生产工具链。
如果当前路线已经固定，且模型不会在这个仓库里继续训练/导出，那么它们会持续制造认知负担。

#### 建议动作
- 若后续不在本仓库维护模型生产：**整体移除**
- 若还想保留：建议迁移到 `tools/modeling/` 或单独仓库，而不是继续挂在 perception 包里

风险：中

---

## 7. P2：建议归档或重写的内容（不一定删代码，但必须收敛）

### 7.1 文档中仍指向旧路线的内容
当前明显漂移的内容：

| 路径 | 问题 |
|---|---|
| `src/smart_follower_perception/README.md` | 仍写“RGB 主链路 / monocular position”，与当前 depth compare 主线不符 |
| `src/smart_follower_bringup/README.md` | 仍写“不依赖深度图”，已过时 |
| `try.md` | 大量 `monocular.*` 说明仍带旧路线语义，需要按“depth compare + 安装外参”重写 |
| `DEPENDENCIES.md` | 仍有 480x640 模型与训练/导出工具链描述 |
| `ORT_线程调优测试计划.md` | 属于阶段性实验文档，不应与当前主线文档并列 |
| `test.md` | 历史实验完整保留可以，但建议分段归档，不应让新人把它当主说明文档 |

#### 建议动作
- 包级 README：**重写，不建议继续沿用旧文本**
- 阶段性实验文档：迁移到 `docs/archive/`
- `try.md`：收口为“当前唯一调参说明”

风险：低

---

### 7.2 运行时版本字符串未更新
涉及：
- `src/smart_follower_perception/include/smart_follower_perception/constants.hpp`
- `src/smart_follower_control/include/smart_follower_control/constants.hpp`

当前仍为：
- `alpha-0.1.7`

#### 审查意见
这不是“删代码”问题，但会直接干扰排查。
日志显示版本号落后，会让线上联调非常混乱。

#### 建议动作
- 更新为 `beta-0.2.0`
- 后续改成统一单点版本来源，避免 perception/control 各自手写

风险：低

---

## 8. 建议保留但需要改名/瘦身的内容

这部分不是删除，而是“避免继续误导”。

### 8.1 `monocular` 参数块建议重命名
当前 `monocular.*` 里混着两类东西：

1. 旧单目路线参数
2. 当前 depth compare 仍需要的相机安装/投影参数

建议未来拆成：
- `camera_intrinsics.*`
- `camera_mount.*`
- `depth_compare.*`

这样新人不会误以为主线还是“纯单目”。

---

## 9. 建议删除顺序（执行方案）

### 第一步：P0 立即清理
目标：删掉确定无用、无争议的内容。

建议顺序：
1. 删除根目录旧模型/中间资产
2. 删除所有 `__pycache__`
3. 删除旧单目 API `estimate_person_position_from_bbox(...)`
4. 删除对应旧测试
5. 跑 perception/control 全测试回归

### 第二步：P1 路线硬绑定
目标：确认是否彻底放弃“纯单目 fallback 路线”。

建议顺序：
1. 确认 Astra + depth compare 是否作为唯一支持方案
2. 若确认，则删除旧单目 fallback 参数和逻辑
3. 重命名 `monocular` 参数块
4. 更新调参文档

### 第三步：P2 文档与工具链归档
目标：让新人只看到当前主线。

建议顺序：
1. 重写包级 README
2. 把历史测试/线程调优文档移到 `docs/archive/`
3. 决定是否保留模型训练/导出脚本

---

## 10. 最终结论

如果目标是：

> **让仓库只围绕“当前能跑、准备调参、准备继续实车收敛”的这条路线服务**

那么当前最应该优先清掉的是：

### 必删（优先级最高）
- 根目录旧模型资产
- `__pycache__`
- 已无主链调用的旧单目估计 API 与测试

### 次删（确认后动）
- 单目 fallback 逻辑与旧参数
- 训练/导出工具链
- 第二启动入口 `smart_follower_only.launch.py`

### 必重写（不是删，但比删更重要）
- perception / bringup 包级 README
- `try.md` 中旧单目描述
- 运行时版本字符串

一句话总结：

> **当前仓库的主要问题已经不是“跑不起来”，而是“历史路线还没收干净”。**
> 下一步最合理的动作不是再加功能，而是做一次“路线收敛式减法”。
