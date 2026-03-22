# ONNX Runtime 线程调优测试计划

更新时间：2026-03-19
适用对象：`smart_follower_perception`（异步版感知链路）

## 1. 目标

本轮测试目标不是改模型，而是为现有最佳组合继续挖掘 CPU 推理性能：

- 感知实现：异步版 perception
- YOLO：`yolo26n_static_480x640_simplify_e2e.onnx`
- ReID：`osnet_x0_5_512.onnx`
- 相机输入：`640x480 @ 30fps`

重点观察：
- `yolo` 平均耗时
- `reid` 平均耗时
- `total` 平均耗时
- `/robot1/person_pose` 频率
- 是否出现明显抖动 / 尖峰

---

## 2. 已接入参数

当前代码已支持以下参数：

```yaml
yolo:
  ort:
    intra_op_num_threads: 3
    inter_op_num_threads: 1
    execution_mode: sequential

reid:
  ort:
    intra_op_num_threads: 1
    inter_op_num_threads: 1
    execution_mode: sequential
```

说明：
- `execution_mode` 允许值：`sequential` / `parallel`
- 当前默认值是我给出的第一版建议起点：
  - YOLO：`3 / 1 / sequential`
  - ReID：`1 / 1 / sequential`

---

## 3. 测试原则

### 3.1 一次只改少量变量

建议先固定：
- 新 YOLO
- 轻量 ReID
- 异步版 perception
- `inter_op_num_threads = 1`

第一轮只测：
- `intra_op_num_threads`
- `execution_mode`

### 3.2 先测 YOLO，再测 ReID

原因：
- 当前 YOLO 仍然是主要耗时项之一
- ReID 已经明显变轻
- 先把 YOLO 调顺，更容易看清收益

### 3.3 每组测试建议至少 30s

每组统一：
- 运行 30s
- 记录 profiling 稳定均值
- 再取一次 `ros2 topic hz /robot1/person_pose`

---

## 4. 推荐测试顺序

# 阶段 A：先固定 ReID，只调 YOLO

固定：
- `reid.ort.intra_op_num_threads = 1`
- `reid.ort.inter_op_num_threads = 1`
- `reid.ort.execution_mode = sequential`

测试矩阵：

| 组别 | yolo intra | yolo inter | yolo mode |
|---|---:|---:|---|
| A1 | 1 | 1 | sequential |
| A2 | 2 | 1 | sequential |
| A3 | 3 | 1 | sequential |
| A4 | 4 | 1 | sequential |
| A5 | 2 | 1 | parallel |
| A6 | 3 | 1 | parallel |

### 阶段 A 的目标
- 找出 YOLO 最合适的线程数
- 判断 `parallel` 是否值得保留

### 我对结果的预判
- 最优点大概率在 `2` 或 `3`
- `4` 不一定更好
- `parallel` 不一定比 `sequential` 稳

---

# 阶段 B：固定最优 YOLO，再调 ReID

固定：
- 使用阶段 A 中最佳 YOLO 参数

测试矩阵：

| 组别 | reid intra | reid inter | reid mode |
|---|---:|---:|---|
| B1 | 1 | 1 | sequential |
| B2 | 2 | 1 | sequential |
| B3 | 3 | 1 | sequential |
| B4 | 2 | 1 | parallel |

### 阶段 B 的目标
- 判断 ReID 是否值得分配更多线程
- 验证轻量 ReID 在多线程下是否真的受益

### 我对结果的预判
- ReID 最优值很可能仍然是 `1`
- 少数情况下 `2` 可能略好
- `3` 和 `parallel` 的边际收益大概率不高

---

# 阶段 C：只在必要时再测 inter-op

只有当阶段 A/B 差异不明显时，再做下面这组：

| 组别 | yolo intra | yolo inter | yolo mode | reid intra | reid inter | reid mode |
|---|---:|---:|---|---:|---:|---|
| C1 | 最优 | 1 | 最优 | 最优 | 1 | 最优 |
| C2 | 最优 | 2 | 最优 | 最优 | 1 | 最优 |
| C3 | 最优 | 1 | 最优 | 最优 | 2 | 最优 |

### 阶段 C 的目标
- 验证 inter-op 是否带来收益
- 若没有明显收益，就固定回 `1`

我的建议是：
- **inter-op 默认尽量保持 1**
- 不要一开始就把变量开太多

---

## 5. 每组需要记录的指标

建议每轮至少记录：

1. `profile avg_ms total`
2. `profile avg_ms yolo`
3. `profile avg_ms reid`
4. `profile avg_ms message`
5. `/robot1/person_pose` 频率
6. 运行中是否出现：
   - 明显卡顿
   - 检测突然长时间断更
   - 日志尖峰异常

---

## 6. 建议判断标准

### 第一优先级
- `total` 更低
- `/robot1/person_pose` 更高

### 第二优先级
- `yolo` 更低
- `reid` 不因抢线程而明显恶化

### 第三优先级
- 抖动更小
- 日志更稳定
- 没有明显 spike

如果出现：
- 平均值稍微变好
- 但 topic hz 更抖
- 或最大延迟变差很多

那我会认为这组参数**不值得作为默认值**。

---

## 7. 我推荐先跑的最小集合

如果今天只想先试一轮，不想把所有组合全跑完，建议先跑这 5 组：

| 组别 | yolo intra | yolo mode | reid intra | reid mode |
|---|---:|---|---:|---|
| S1 | 1 | sequential | 1 | sequential |
| S2 | 2 | sequential | 1 | sequential |
| S3 | 3 | sequential | 1 | sequential |
| S4 | 4 | sequential | 1 | sequential |
| S5 | 3 | parallel | 1 | sequential |

这 5 组通常就足够看出趋势了。

---

## 8. 当前推荐起点

如果要先给一个默认起点，我建议：

```yaml
yolo:
  ort:
    intra_op_num_threads: 3
    inter_op_num_threads: 1
    execution_mode: sequential

reid:
  ort:
    intra_op_num_threads: 1
    inter_op_num_threads: 1
    execution_mode: sequential
```

这也是我已经写进参数文件的当前默认值。

---

## 9. 建议执行方式

建议每组按以下顺序：

1. 修改参数
2. 启动 perception
3. 跑 30s
4. 采集 profile 日志
5. 跑 `ros2 topic hz /robot1/person_pose`
6. 停止 perception
7. 记录结果到对照表

---

## 10. 预期产出

完成这轮测试后，我们应能得到：

- 一组更可信的 YOLO ORT 参数
- 一组更可信的 ReID ORT 参数
- 是否需要继续测 inter-op / parallel
- 下一轮是否值得继续做：
  - ORT profiling
  - 检测频率自适应
  - INT8 量化
