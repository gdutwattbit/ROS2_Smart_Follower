# smart_follower_perception

`smart_follower_perception` 是当前项目的感知主包，负责这条固定主链路：

```text
/camera/color/image_raw + /camera/depth/image_raw
                    ↓
                  YOLO
                    ↓
                  ReID
                    ↓
                 Tracker
                    ↓
               Lock Manager
                    ↓
      depth compare 定位（bbox 下半部窗口 + median）
                    ↓
                /person_pose
```

## 1. 主要文件

- `src/perception_node.cpp`
  - Lifecycle 节点入口
  - 负责参数加载、相机内参服务请求、订阅/发布与热更新
- `src/perception_pipeline.cpp`
  - 异步感知主线
  - 连接 runtime / tracker / lock / publish
- `src/runtime.cpp`
  - YOLO / ReID 模型加载与 ONNX Runtime 推理
- `src/tracker.cpp`
  - 多目标跟踪、匹配、记忆
- `src/lock_manager.cpp`
  - 锁定 / 解锁 / 切人策略
- `src/pipeline_utils.cpp`
  - `/person_pose` 消息组装
- `src/geometry_utils.cpp`
  - depth compare 取样与位置求解

## 2. 输入输出

### 输入
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/follow_command`
- `/camera/get_camera_info`（服务）

### 输出
- `/person_pose`

## 3. 当前关键约束

- 主链路默认依赖 **彩色图 + 深度图**
- 人体位置由 **depth compare** 求解，不再走旧单目 bbox 投影主路径
- 节点在 `configure()` / 热更新时会请求 `/camera/get_camera_info`
- `TrackedPerson.position` 输出在 `base_frame` 下
- `TrackedPerson.appearance_feature` 当前固定为 OSNet 对应的 `512` 维

## 4. 默认重点参数

重点看：
- `src/smart_follower_bringup/config/perception_params.yaml`

其中最关键的是：
- `yolo.model_path`
- `reid.model_path`
- `process_every_n_frames`
- `yolo.ort.*`
- `reid.ort.*`
- `camera.info_service`
- `camera.x_offset_m`
- `camera.y_offset_m`
- `depth_compare.*`

## 5. 修改入口建议

- 改推理与预处理：看 `runtime.*`
- 改跟踪策略：看 `tracker.*`
- 改锁人逻辑：看 `lock_manager.*`
- 改消息发布：看 `pipeline_utils.*`
- 改 depth compare 定位：看 `geometry_utils.*`

## 6. 当前注意事项

- `info_service` 现在属于必需路径，服务不可用会导致 configure 失败
- depth compare 对深度图对齐质量、采样窗口和有效样本数比较敏感
- 当前默认模型在 `models/` 目录：
  - `yolo26n_static_256x320_simplify_e2e_int8.onnx`
  - `osnet_x0_5_512.onnx`
