# smart_follower_perception

`smart_follower_perception` 负责固定感知主链路：

```text
/camera/color/image_raw + /camera/depth/image_raw
                    ->
                  YOLO
                    ->
                  ReID
                    ->
                 Tracker
                    ->
               Lock Manager
                    ->
      depth compare positioning
                    ->
                /person_pose
```

## Main Structure

- `perception_node.cpp`: lifecycle、参数、订阅发布、内参服务与热更新分发
- `perception_pipeline.*`: 异步 worker 调度与结果队列
- `perception_processing.*`: 单帧处理结果整合、tracking、lock、消息发布
- `runtime.*`: YOLO / ReID 模型加载与推理
- `pipeline_utils.*`: 检测结果与 `/person_pose` 消息组装

## Notes

- 参数文件现在使用显式节点名 `perception_node`，不再使用 `/**`。
- `camera.info_service` 仍是必需路径，服务不可用会导致 configure 失败。
- 默认模型路径由 bringup 显式传入；节点内部仍支持相对路径解析与环境变量覆盖。
