# Observation, Reward, and Action Reference

## 1. 先明确 RL 的输入输出

每 12 ms：

```text
34D observation -> normalized/clipped PPO MLP -> 1D Gaussian action
1D action -> target wave -> rate-limited realized wave
realized wave -> gait map -> CPG/PD -> physics
```

Policy 看不到：terrain ID、terrain wavelength/amplitude、world position、手工 roughness label、heightmap lookup、future contact 或 failure label。

## 2. 34 个维度逐项表

所有 index 为 0-based，与 checkpoint 的 34-wide running statistics 顺序一致。

| Index | 名称 | 计算 | 物理含义 | 主要限制 |
|---:|---|---|---|---|
| 0 | `realized_wave` | controller 当前 wave | 动作执行状态 | 是自身状态，不是 terrain evidence |
| 1 | `cpg_sin` | `sin(body_phase[0])` | 1Hz cycle phase | 仅相位 |
| 2 | `cpg_cos` | `cos(body_phase[0])` | 消除 phase wrap 的另一坐标 | 仅相位 |
| 3 | `root_vx` | body-frame root linear velocity x | 前后运动响应 | 受 gait/action 与 terrain 共同影响 |
| 4 | `root_vy` | body-frame root linear velocity y | 侧滑 | 不能单独识别 roughness |
| 5 | `root_vz` | body-frame root linear velocity z | 垂向颠簸 | 速度估计可能含接触瞬态 |
| 6 | `root_wx_scaled` | body-frame angular velocity x *0.25 | roll rate | scale 只为数值尺度 |
| 7 | `root_wy_scaled` | angular velocity y *0.25 | pitch rate | 同上 |
| 8 | `root_wz_scaled` | angular velocity z *0.25 | yaw rate | 同上 |
| 9 | `gravity_x` | projected gravity in body frame | body tilt x component | 姿态，不是地形几何直接测量 |
| 10 | `gravity_y` | projected gravity y | body tilt y component | 同上 |
| 11 | `gravity_z` | projected gravity z | upright/flip state | failure gate 也使用相关值 |
| 12 | `contact_std_dct0` | 19 段 contact 时间 std 的 spatial DCT mode0 | 全身平均 contact variability | 1s window 未满时为0 |
| 13 | `contact_std_dct1` | 同上 mode1 | 前后低频梯度/不对称 | DCT sign 与 body 顺序有关 |
| 14 | `contact_std_dct2` | 同上 mode2 | 更高阶纵向分布变化 | 只保留前3 mode |
| 15 | `contact_spike_dct0` | 19 段 spike fraction 的 mode0 | 全身平均冲击频率 | spike 是相对各段 median 的3倍 |
| 16 | `contact_spike_dct1` | 同上 mode1 | 冲击前后梯度 | 高度依赖 1s 历史 |
| 17 | `contact_spike_dct2` | 同上 mode2 | 冲击空间起伏 | 丢弃更高 spatial modes |
| 18 | `omega_crest_log` | `log1p(max omega / RMS omega)` | 突发角速度峰相对背景 | 只用 magnitude，不含方向 |
| 19 | `omega_spike_rate` | `mean(omega > 3*median omega)` | 姿态冲击频率 | median 接近0时较敏感 |
| 20 | `total_load_std` | 1s total foot load std | 全身载荷波动 | load 已按0.05N缩放 |
| 21 | `total_load_spike_rate` | `mean(load > 3*median load)` | 全身大冲击频率 | 仍是相对阈值 |
| 22 | `spine_y_dct0` | chord-detrended lateral spine residual mode0 | 全身平均侧弯偏移 | root-frame、chord-normalized |
| 23 | `spine_y_dct1` | lateral mode1 | 前后侧弯梯度 | 不是 joint angle 直接和 |
| 24 | `spine_y_dct2` | lateral mode2 | 中阶 lateral shape | 只保留前4 modes |
| 25 | `spine_y_dct3` | lateral mode3 | 更局部 lateral shape | 高频 geometry 被丢弃 |
| 26 | `spine_z_dct0` | vertical residual mode0 | 全身平均拱起/下垂 | 去除了首尾直线趋势 |
| 27 | `spine_z_dct1` | vertical mode1 | 前后拱起梯度 | 同上 |
| 28 | `spine_z_dct2` | vertical mode2 | 中阶垂直形状 | 同上 |
| 29 | `spine_z_dct3` | vertical mode3 | 更局部垂直形状 | 同上 |
| 30 | `curvature_y_rms` | lateral residual 二阶差分 RMS | lateral bending intensity | 离散曲率 proxy，不是 SI curvature |
| 31 | `curvature_z_rms` | vertical 二阶差分 RMS | vertical bending intensity | 同上 |
| 32 | `edge_cv` | 20 个相邻 body edge length 的 std/mean | 躯干局部压缩/伸展不均 | USD link geometry/pose 也影响 |
| 33 | `tortuosity` | spine polyline length / head-tail chord length | 躯干路径弯曲程度 | chord 很短时 clamp 到1e-4 |

维度总和：

```text
1 realized wave
+ 2 CPG phase
+ 9 root kinematics/orientation
+ 10 one-second contact/time response
+ 12 instantaneous spatial body shape
= 34
```

## 3. DCT 压缩到底做了什么

对沿身体排序的 N 个量 `x_j`，第 k 个正交 DCT-II mode 近似为：

```text
c_k = scale_k * sum_j x_j cos(pi * (j+0.5) * k / N)
```

- mode0 是全身平均/共同成分。
- mode1 表示最慢的头尾梯度。
- mode2、mode3 表示逐渐更细的纵向结构。
- contact 用 N=19、保留3 modes；spine geometry 用 N=21、保留4 modes。

它避免把 19 或 21 个高度相关 segment 全塞给 policy，同时保留“扰动发生在身体什么位置”的低频结构。代价是局部高频足碰撞可能被压掉。

## 4. 为什么既要 time-domain 又要 spatial-domain

### Time-domain 10D

1s 内 contact/omega/load 的 variability 和 spike 描述机器人被地形怎样持续激励。优点是对瞬时噪声更稳；缺点是启动后第一秒全为0，并且不能直接表达当前身体空间形状。

### Spatial-domain 12D

当前 21-link 躯干在 root frame 中的 chord-detrended shape 描述此刻身体怎样被 terrain/contact 塑形。它立即可用，更接近空间相互作用；缺点是只是一帧，可能把 gait phase 与 terrain response 混合。

### Root 9D

全局运动响应和姿态提供低维动力学结果。它们对控制有效，但单独不一定能区分不同 roughness 来源。

组合的设计目标不是“重建 heightmap”，而是给 policy 足够的闭环身体响应，让它有可能学会 terrain-conditioned wave action。

## 5. 31D scientific subset 是什么

Classifier audit 排除 index 0-2：

```text
all_no_wave_phase31 = observation[3:34]
```

这样做是为了避免 classifier 通过 commanded/realized wave 或固定 CPG phase 猜标签。31D 仍包含 root、contact/time 和 spatial geometry。

## 6. Terrain 信息证据

冻结 audit：

| 项 | 值 |
|---|---:|
| 成功 boots | 30；seed 8510/9601 各15 |
| Primary snapshot | 1.44 s |
| 任务 | roughness low/mid/high 3-class |
| 特征 | no-wave/no-phase 31D |
| split | simulator seed exclusive + held wave exclusive |
| primary unit | 每个 held-out boot 聚合一个 prediction |
| balanced accuracy | 0.9444 |
| minimum class recall | 0.8333 |
| exact terrain-label permutation p | 0.0333 |
| phase-only negative control | 0.3333 |
| scientific minus control | 0.6111 |

### `whole-boot balanced accuracy` 的含义

1. 同一个 simulator boot 内的 32 env 先聚合成一个 boot-level prediction。
2. 每类 recall 分别计算。
3. balanced accuracy 是各类 recall 的平均，因此不被 low/high 样本多、mid 样本少直接支配。
4. seed-exclusive 表示一个 terrain/simulator seed 不同时出现在 train/test。
5. wave-exclusive 进一步要求测试 wave 未在对应训练 fold 中出现，降低 classifier 记住 gait response 的风险。

它不是“32个env投票就有32个独立样本”。统计单位仍是 boot。

### 这份证据允许说什么

- 在测试 terrain panel 和时间点，31D response 中存在强的、可线性读取的 3-class roughness 信息。
- phase-only control 不能完成任务，因此结果不是简单 CPG phase leakage。
- 一个含 MLP 的 PPO policy 在表示能力上至少包含线性 classifier 这种简单函数。

### 不允许说什么

- 不能说 34D 完整推断了任意 terrain geometry。
- 不能说 0.9444 是新地形上的普适准确率。
- 不能说 PPO 必然利用了这些信息；PPO 优化 return，不优化 terrain label。
- 不能说 classifier 的人工 low/mid/high 标签是自然界唯一 roughness ordering。
- 不能说一个 boot 内的 32 env 是32个独立统计样本。
- 不能排除 nonlinear policy 在 classifier 较弱的 feature group 上仍找到有用函数。

## 7. Action law 手算

### 映射

```text
action -1 -> target wave 1.0
action  0 -> target wave 2.5
action +1 -> target wave 4.0
```

### 为什么命令不会立刻到 target

reset wave=2.5。假设第一帧 action=-1，误差1.5大于 `rate*tau=0.875`，先进入 rate-saturated 区：

```text
delta wave = 3.5 wave/s * 0.012 s = 0.042
first realized wave = 2.5 - 0.042 = 2.458
```

后续逐步下降，接近 target 后转指数响应。因此短 episode 即使 action 一直=-1，也可能很晚才接近1.0。这是过去“policy 为什么碰不到1.5”讨论的核心检查点：必须看 realized wave trace，而不是只看 raw action。

## 8. Gait map 与 action 的绑定

realized wave 不仅改变 CPG 空间相位；它同时决定：

1. body yaw amplitude scale；
2. leg yaw amplitude scale；
3. body yaw Kp；
4. body yaw Kv；
5. legbase yaw Kp；
6. legbase yaw Kv。

因此 policy 的一个 action 实际选择的是一条一维 gait manifold 上的 operating point，而不是只改变一个 wave number。科研表述应写“wave-conditioned gait operating point”，不能把因果全部归给 wave number。

## 9. Reward 公式

令 61 个 spine link 的 world x 坐标为 `x_i(t)`：

```text
x_spine(t) = (1/61) * sum_i x_i(t)
r_t = -(x_spine(t+12ms) - x_spine(t)) / 0.1025
```

canonical forward 是 world -x，所以有负号。reward 单位是“本 policy step 前进了多少 body length”。它没有再除以 12ms；episode return 是净 body-length displacement 的近似和。

terminal/failure/time-limit 当步 reward 被置0，防 reset 或 terminal 状态跳变混入。

### Reward 中没有什么

- 没有 survival bonus。
- 没有 energy/torque penalty。
- 没有 belly/contact penalty。
- 没有 yaw/roll penalty。
- 没有 target wave 或 low-wave preference。
- 没有 terrain classification auxiliary loss。

因此若 policy 选择较高 wave，只能来自它在训练采样和优化过程中的 expected pure-forward return 更有利，或来自探索/优化/command-boundary 限制；不能归因于隐藏 shaping 项。

## 10. Failure 与 reward 要分开

Failure 条件只含严重 tipping、root z<0 和非有限 state。一个失败 episode 会终止，间接减少可获得的未来 forward reward，但没有独立 survival reward。

科研分析必须并列报告：

1. survival rate；
2. conditional full-duration speed among survivors；
3. first-done timing/missingness。

没有 survivor 的 cell 没有合法 conditional speed。不要填0后和正常速度一起平均。

## 11. PPO 能否“隐式包含 classifier”

网络第一层是 34->256，之后256->128，再输出 policy mean/value。线性 classifier 只是 `W x + b`；MLP 第一层显然可以表示这样的线性投影，所以 representation capacity 足够。

但能力不等于学习结果：

- classifier 有 terrain label supervision；PPO 只有 delayed return/advantage。
- PPO 的 data distribution 由当前 policy 改变。
- observation normalization、command lag、episode horizon 和 exploration sigma 都影响能否利用 terrain information。
- terrain-related features 可能与 wave/gait phase 纠缠，policy 未必学到 PI 期望的 monotone rule。

证明闭环使用 terrain 信息仍需 policy intervention：固定同一 policy/seed，改变 terrain response，查看 realized wave 分布、return 与 counterfactual/ablation，而不是只看 classifier。

