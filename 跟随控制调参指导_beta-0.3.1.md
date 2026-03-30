# 跟随控制调参指导（beta-0.3.1）

本文只针对当前主线的 `follower_controller_node`，重点解决两个现象：

1. 直线跟随偏慢，有迟钝感
2. 转弯时前轮抽搐，但整体轨迹转向幅度基本正常

先给结论：

- 第一个问题大概率不是单一 PID 问题，而是 **线速度上限、线速度爬升限幅、距离环增益** 一起偏保守
- 第二个问题更像是 **转向通道对横向误差太敏感**，叠加感知抖动后，方向命令高频来回修正

---

## 1. 先看当前控制逻辑

当前默认参数在 [control_params.yaml](d:/Program/ros2_smart_follower/src/smart_follower_control/config/control_params.yaml)：

- `target_distance = 0.6`
- `theta_deadzone = 0.03`
- `prediction_horizon_s = 0.25`
- `velocity_ema_alpha = 0.70`
- `pid_r.kp = 0.8`
- `pid_r.kd = 0.1`
- `pid_t.kp = 1.2`
- `pid_t.kd = 0.1`
- `limits.v_max = 0.6`
- `limits.w_max = 1.2`
- `limits.dv_max = 0.5`
- `limits.dw_max = 1.5`

核心控制逻辑在 [follower_runtime.cpp](d:/Program/ros2_smart_follower/src/smart_follower_control/src/follower_runtime.cpp)：

- 小角度死区：[`theta_deadzone`](d:/Program/ros2_smart_follower/src/smart_follower_control/src/follower_runtime.cpp#L139)
- 距离环输出线速度：[`pid_r_.update(...)`](d:/Program/ros2_smart_follower/src/smart_follower_control/src/follower_runtime.cpp#L147)
- 角度环输出角速度：[`pid_t_.update(...)`](d:/Program/ros2_smart_follower/src/smart_follower_control/src/follower_runtime.cpp#L148)
- 当 `|theta| > 0.5 rad` 时，线速度直接乘 `0.3`：[`v *= 0.3`](d:/Program/ros2_smart_follower/src/smart_follower_control/src/follower_runtime.cpp#L151)
- 线速度和角速度都还要过一层加速度限幅：[`dv_max`](d:/Program/ros2_smart_follower/src/smart_follower_control/src/follower_runtime.cpp#L154)、[`dw_max`](d:/Program/ros2_smart_follower/src/smart_follower_control/src/follower_runtime.cpp#L155)

所以现在车“肉”和“抽”，都很容易从这里解释出来。

---

## 2. 先讲两个问题分别是什么

### 2.1 直线跟随太慢、迟钝

最常见的 4 个原因：

1. `pid_r.kp` 偏小，距离误差转成速度的力度不够
2. `limits.v_max` 太保守，速度上不去
3. `limits.dv_max` 太小，速度爬升太慢
4. 感知本身只有 `4~5Hz`，控制再快也会显得钝

其中第 3 点最容易被忽略。

当前：

- `control_rate = 20Hz`
- `dv_max = 0.5`

这意味着每个控制周期线速度最多只能变化：

- `0.5 * 0.05 = 0.025 m/s`

如果目标速度想从 `0` 拉到 `0.6 m/s`，理论上就要大约：

- `0.6 / 0.025 = 24` 个周期
- 也就是约 `1.2s`

所以你感觉“迟钝”，这非常符合当前参数。

### 2.2 转弯时前轮抽搐，但轨迹幅度又正常

这通常说明：

- 平均转向量没有大问题
- 但角速度命令里夹着高频抖动

常见来源：

1. `pid_t.kp` 偏大，横向角误差一有波动就立刻修
2. `pid_t.kd` 偏大，对测量噪声很敏感
3. `theta_deadzone` 太小，近零角度附近也在不停修方向
4. 感知位置在左右轻微跳，控制层被动跟着来回打方向
5. `prediction_horizon_s` 偏大，预测把抖动也提前放大了

你描述“轨迹大体是对的，但前轮抽搐”，我最优先怀疑：

- `theta_deadzone` 太小
- `pid_t.kp / kd` 略偏激进

---

## 3. 调参原则

这次不要同时乱动很多项，按下面原则来：

1. 先把“直线速度”调顺
2. 再把“转向平顺性”调顺
3. 每次只改 1 到 2 个参数
4. 每轮测试都做“站定、直行、转向”三段

---

## 4. 对这两个问题，我的直接建议

## 4.1 先解决“直线太慢”

我建议先按这个顺序试：

### 第一步：先加线速度爬升能力

优先改：

- `limits.dv_max: 0.5 -> 0.8`

如果还是肉，再试：

- `limits.dv_max: 0.8 -> 1.0`

原因：

- 这项直接决定车加速是不是“发闷”
- 它通常比先改 PID 更直观

### 第二步：再加距离环比例

再改：

- `pid_r.kp: 0.8 -> 1.0`

如果还偏慢，再试：

- `pid_r.kp: 1.0 -> 1.2`

建议：

- `pid_r.kd` 先不动，保持 `0.1`
- 只有当直线开始出现明显前后窜动，再把 `kd` 降到 `0.05`

### 第三步：必要时再放开线速度上限

如果前两步后仍明显偏慢，再试：

- `limits.v_max: 0.6 -> 0.7`

谨慎一些的话最多先试到：

- `limits.v_max: 0.8`

不建议一上来就开太大，因为底盘和避障还要一起工作。

---

## 4.2 再解决“转弯抽搐”

这个问题我建议优先按下面顺序压。

### 第一步：先增大角度死区

优先改：

- `theta_deadzone: 0.03 -> 0.05`

如果仍抽，再试：

- `theta_deadzone: 0.05 -> 0.06`

原因：

- 这能直接抑制“目标几乎在正前方时还不停左右小修”
- 对“前轮抖但整体轨迹没坏”的情况很有效

### 第二步：把转向比例略降一点

再改：

- `pid_t.kp: 1.2 -> 1.0`

如果还抽，再试：

- `pid_t.kp: 1.0 -> 0.9`

理由：

- 现在转向幅度已经基本正常，说明不是“转不动”
- 问题更像是“修得太勤”

### 第三步：把转向微分减弱

如果还有高频抽动，再试：

- `pid_t.kd: 0.1 -> 0.05`

再不行可以试：

- `pid_t.kd: 0.05 -> 0.02`

甚至短测一下：

- `pid_t.kd = 0.0`

如果 `kd` 一降，前轮明显不抖了，那就说明你的噪声主要被微分项放大了。

### 第四步：必要时再减小角速度变化率

如果上面几步都做了，前轮仍然“快速摆”，再试：

- `limits.dw_max: 1.5 -> 1.0`

这个会让方向变化更柔和，但也会让转向显得更钝一点，所以我把它放在后面。

---

## 5. 我建议你今晚先试的一组参数

这组是我觉得最可能同时改善两个问题、又比较稳的一组：

```yaml
theta_deadzone: 0.05
pid_r:
  kp: 1.0
  kd: 0.1
pid_t:
  kp: 1.0
  kd: 0.05
limits:
  v_max: 0.7
  dv_max: 0.8
  w_max: 1.2
  dw_max: 1.0
```

这一组的思路是：

- 直线更肯给速度
- 加速更快
- 转向不过度兴奋
- 前轮动作更柔一点

---

## 6. 如果只允许改最少参数

如果你晚上想快速定位，我建议只试这 4 个：

1. `limits.dv_max: 0.5 -> 0.8`
2. `pid_r.kp: 0.8 -> 1.0`
3. `theta_deadzone: 0.03 -> 0.05`
4. `pid_t.kd: 0.1 -> 0.05`

这 4 个最有可能直接命中你说的两个问题。

---

## 7. 现场测试顺序

每改一轮参数，固定做这 3 段：

### 7.1 直线逼近

人站在车正前方，慢慢拉开一点距离，再停住。

观察：

- 车是不是比以前更愿意往前走
- 起步是不是还发闷
- 接近目标距离时会不会前后窜

### 7.2 小角度转向

人从车正前方稍微偏左、稍微偏右移动。

观察：

- 前轮是否还快速左右抖
- 车身是否转得过猛

### 7.3 连续弯道

人沿着一个缓弯轨迹走。

观察：

- 前轮是否抽搐
- 整体轨迹是否仍跟得上
- 是否出现“方向顺了，但转不过去”

---

## 8. 动态调参命令

如果你是在线热调，直接用下面这些：

```bash
ros2 param set /robot1/follower_controller_node theta_deadzone 0.05
ros2 param set /robot1/follower_controller_node pid_r.kp 1.0
ros2 param set /robot1/follower_controller_node pid_t.kp 1.0
ros2 param set /robot1/follower_controller_node pid_t.kd 0.05
ros2 param set /robot1/follower_controller_node limits.v_max 0.7
ros2 param set /robot1/follower_controller_node limits.dv_max 0.8
ros2 param set /robot1/follower_controller_node limits.dw_max 1.0
```

先读当前值也可以：

```bash
ros2 param get /robot1/follower_controller_node theta_deadzone
ros2 param get /robot1/follower_controller_node pid_r.kp
ros2 param get /robot1/follower_controller_node pid_t.kp
ros2 param get /robot1/follower_controller_node pid_t.kd
ros2 param get /robot1/follower_controller_node limits.v_max
ros2 param get /robot1/follower_controller_node limits.dv_max
ros2 param get /robot1/follower_controller_node limits.dw_max
```

---

## 9. 一个很重要的前提

你现在感知输出大约只有 `4.5Hz`，这一点会直接影响调参体感。

所以要有一个预期：

- 如果感知更新本身偏慢，控制再怎么调，也不会变成“特别丝滑”
- 你现在能优先做到的是：
  - 直线不那么肉
  - 转向不那么抽
  - 整体更顺一点

但如果想再上一个台阶，后面还是要回到感知侧，把输出频率和稳定性再抬一点。

---

## 10. 我对你这两个问题的最终判断

如果只凭当前代码和你描述来判断，我的意见是：

### 对“直线太慢”

优先动：

1. `limits.dv_max`
2. `pid_r.kp`
3. `limits.v_max`

### 对“前轮抽搐”

优先动：

1. `theta_deadzone`
2. `pid_t.kd`
3. `pid_t.kp`
4. `limits.dw_max`

也就是说：

- “慢”先从线速度限幅和距离环入手
- “抽”先从转向灵敏度和噪声放大入手

这比一上来乱改全部 PID 更稳，也更容易看出因果。
