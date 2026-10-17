# Clean RL 编程课程：从源码到一条完整 rollout

## 0. 课程边界与版本事实

本课程只讲 `Centipede_IsaacLab_RL_Clean` 的冻结主线，不讲旧仓中的历史分支。
运行代码冻结于 2026-08-17；高保真皮肤、三地形 demo、波数俯视 panel、
120 fps structure-reveal 范式在 2026-08-18 至 2026-08-23 继续同步。当前
`SOURCE_LOCK.json`、`python scripts/check_project.py --quick` 和 `pytest -q`
共同定义可接手版本。课程文档是教学层，不改变运行时 hash。

当前科学边界也必须一起接手：

- Open loop 的 P13 结论是 `CONDITIONAL_LOCAL_OR_CALIBRATION_DEPENDENT`，不是普适定律。
- 31D scientific observation subset 在测试 panel 中包含线性可读的 terrain 信息；这不证明 PPO 必然使用它。
- 三个 epoch-24 checkpoint 可运行，但迁移时 closed-loop 矩阵尚未完成，不能宣称 RL 已证明 rougher -> higher wave。

## 1. 你最后应该能独立回答什么

完成课程后，你应该能在不看旧仓的情况下回答：

1. USD、`ModelIndex`、controller、environment、RL-Games 分别负责什么。
2. 一个 PPO action 怎样在 12 ms 内变成 12 组 1 ms 关节 reference。
3. 34D observation 的每一组怎样从仿真状态构造。
4. pure-forward reward、survival、speed 和 return 为什么不能混为一谈。
5. tri terrain 为什么用于训练，homogeneous terrain 为什么用于评估。
6. fixed-wave、closed-loop policy 和 open-loop sweep 三条入口怎样复用同一个 environment。
7. 怎样从 `metrics.json`、`traces.npz` 和 world-pose trace 判断一次运行是否可信。
8. 怎样修改一个层而不同时污染模型、控制器、observation、reward 和训练配置。

## 2. 全项目只保留这一条调用链

```text
USD assets
  -> centipede/model.py              exact joint/body indices
  -> centipede/env_cfg.py            Isaac scene and frozen timing
  -> centipede/terrain.py            source height -> collision mesh -> spawn height
  -> centipede/gait.py               action law and wave-conditioned gait map
  -> centipede/controller.py         1 ms CPG position/velocity references
  -> centipede/env.py                34D obs, 1D action, reward, done, reset
  -> scripts/train.py                PPO training on tri terrain
  -> scripts/evaluate.py             one policy or one fixed wave
  -> scripts/open_loop_sweep.py      isolated fixed-wave cells
  -> scripts/render_*.py             post-simulation visualization only
```

第一次读源码按这个顺序：

```text
model.py -> gait.py -> terrain.py -> controller.py -> env_cfg.py -> env.py
         -> rl_games.json -> train.py -> evaluate.py -> open_loop_sweep.py
```

`docs/01_RUNTIME_LINE_BY_LINE.md` 和
`docs/02_RL_CONFIG_AND_TOOLS_LINE_BY_LINE.md` 已覆盖所有有行为意义的源码行。
本课程不重复逐行翻译，而是把这些行组织成可执行的心智模型。

## 3. Lesson 1：先把数字 21、61、211 分清

### 阅读

- `centipede/model.py:8-74`
- `docs/01_RUNTIME_LINE_BY_LINE.md` 第 1-2 节

### 核心代码

```python
self.main_body = np.asarray(
    [body(f"link_body_{i}") for i in range(N_BODY_SEGMENTS)], dtype=int
)
self.spine = np.asarray([...], dtype=int)
if len(self.spine) != 61:
    raise ValueError(...)
```

### 老师解释

`ModelIndex` 不是运动学模型，它是 USD 命名到整数 index 的唯一翻译层。21 个
`main_body` 用于描述可见主躯干的空间形状；61 个 `spine` link 用于计算质心式
forward metric；211 个 active joints 接收 position/velocity reference。三组数字的
物理角色不同。这里使用 exact name 和硬检查，是为了让错误模型尽早失败，而不是
让正则表达式找一个“差不多”的 joint 继续跑。

### 你要能沿着代码指出

- 为什么 body yaw 从 1 开始，而 pitch/roll 从 0 开始。
- `legs` 为什么是 `(19, 2, 4)`。
- 为什么 211 unique 检查同时防止漏 joint 和重复 joint。
- 为什么 high-fidelity Blender mesh 不出现在这里。

## 4. Lesson 2：地形不是一张图，而是三个坐标层

### 阅读

- `centipede/terrain.py:11-145`
- `centipede/env.py:136-160, 361-409`

### 三层含义

1. `source heightmap`：窄频带空间波叠加后的浮点地形。
2. `collision heightmap`：经过 uint8 round-trip 和 MuJoCo vertical convention 后的非负高度。
3. `USD triangle mesh`：Isaac/PhysX 真正碰撞的顶点和三角面。

### 核心代码

```python
source = homogeneous_terrain(lambda_mm, amplitude_mm, seed)
self._terrain_collision = build_mesh(stage, "/World/CentipedeTerrain", source)
z_m = footprint_max(self._terrain_collision, x_m, y_m, radius_m) + clearance
```

### 老师解释

spawn 必须使用 collision 坐标，而不是 source map。机器人 footprint 半径内只要有
一个更高的地形点，单点采样的 spawn z 就可能导致初始 penetration。当前实现取
75 mm footprint 最大碰撞高度，再加 25 mm clearance。这样把“出生就翻转”和
“运动过程中因接触失稳”分开。

`tri_terrain` 在同一 3 m 方形里放 flat、L90/A8、L60/A12 三条纵向 strip，供训练
reset 随机选择。`homogeneous_terrain` 覆盖整个 arena，供科学评估，避免一个 boot
内混用 terrain label。

### 必须保持的坐标纪律

- 数组索引是 `height[row=y, col=x]`。
- 世界坐标映射是 `z[row, col] -> world(y[row], x[col])`。
- `L90/A8` 的单位都是 mm；A 是 source 的最终 peak magnitude。
- `A/L` 不是唯一 terrain-complexity 定义。

## 5. Lesson 3：RL 并不直接控制 211 个关节

### 阅读

- `centipede/gait.py:12-83`
- `config/gait_map.json`
- `centipede/env.py:179-201`

### Action 的三种数值

```text
raw policy action a in [-1, 1]
  -> target wave = 2.5 + 1.5a
  -> rate-limited and first-order realized wave in [1, 4]
  -> six interpolated gait/controller parameters
```

target wave 不是 realized wave。误差较大时每个 12 ms 最多变化
`3.5 * 0.012 = 0.042`；接近 target 后进入 tau=0.25 s 的指数响应。

### 1 对 6 绑定

每个 realized wave 同时决定：

1. body yaw amplitude scale；
2. legbase yaw amplitude scale；
3. body yaw Kp；
4. body yaw Kv；
5. legbase yaw Kp；
6. legbase yaw Kv。

非 knot wave 在相邻 0.25-wave knot 之间线性插值。`leg_pitch_amp=1.15` 被锁定，
不属于六个变化参数。这张 map 是 calibration contract，不是物理定律，也不代表每个
knot 都是独立全局最优点。

### 你要会手算

- `a=-1,0,+1` 的 target wave。
- 从 realized=2.5、action=-1 开始，下一步 realized wave 的变化量。
- realized=2.625 时 gait-map interpolation 的 alpha。
- `body_amp` 和 `leg_yaw_amp` 为什么是 scale 而不是 rad。

## 6. Lesson 4：CPG 负责 reference，PhysX PD 负责力

### 阅读

- `config/controller.yaml`
- `centipede/controller.py:18-330`
- `centipede/env.py:203-225`

### 一次 physics substep

```python
self.controller.step(self._sim_time, 0.001)
self.robot.write_joint_stiffness_to_sim(...)
self.robot.write_joint_damping_to_sim(...)
self.robot.set_joint_position_target(...)
self.robot.set_joint_velocity_target(...)
self.robot.set_joint_effort_target(zeros, ...)
```

CPG 每 1 ms 更新 phase 和 target。PPO 只每 12 ms 改一次 realized-wave command；这不
意味着关节只以 12 ms 更新。explicit effort 永远写 0，唯一反馈和力路径是 implicit PD。

### Controller 的四部分

1. 19 个 body oscillator 构成 Kuramoto neighbor chain。
2. 19 个 leg oscillator 跟随节段中心相位：对应 body-yaw phase 加半个相邻 phase lag。
3. settle 后使用 staggered cosine blend 依次启动 body 和 legs。
4. 目标角有限差分得到目标速度，再填入 211-joint tensor。

模型中 body-yaw joint 位于两个身体节段之间，而腿根装在节段上。因此它们不能共用同一个
空间采样点。代码使用 `leg_anchor = body_phase + 0.5 * delta`；这等价于取前后 body-yaw
相位的中点。这个 `0.5*delta` 是沿身体方向的空间偏移，不是 YAML 中 leg yaw/pitch 的
摆动时序 offset。后者保持不变，所以 leg yaw 与 leg pitch 的相对动作关系没有被改写。

### 当前实际动作

- body yaw angle amplitude = `0.4 * body_amp(wave)` rad。
- legbase yaw angle amplitude = `0.6 * leg_yaw_amp(wave)` rad。
- legbase pitch amplitude = `0.4 * 1.15 = 0.46` rad。
- legbase roll target = 0，不 swing。
- knee target = pi/6，不 swing。
- head pitch joint target = -0.57 rad；它不等于世界 head pitch。

Kuramoto coupling gain、joint Kp/Kv 和 amplitude scale 是三类不同参数。阅读时发现
“gain”必须先问它作用在 phase、reference amplitude 还是 PhysX impedance 上。

## 7. Lesson 5：DirectRLEnv 的一帧怎样流动

### 阅读

- `centipede/env_cfg.py`
- `centipede/env.py:78-225`

### Policy step 的真实顺序

```text
action
  -> _pre_physics_step
       save reward-start centroid
       update realized wave
       interpolate gait map
       update yaw Kp/Kv
  -> 12 x _apply_action
       1 ms CPG target
       implicit-PD gains and targets
       physics advance
       contact-window sample
  -> reward / done / next observation
```

`decimation=12` 和 `sim.dt=0.001` 共同定义 12 ms RL period。episode 9.6 s 对应
800 policy steps。`is_finite_horizon=False` 只控制 time-limit 的 value bootstrap 语义，
不是无限 episode。

### 初始化为什么有两阶段

Isaac `DirectRLEnv` 的父类在子类字段尚未全部创建时可能请求 observation。因此
`_get_observations` 在 controller 尚不存在时返回合法 34D 零向量。这是 framework
初始化顺序的兼容，不是另一个 observation branch。

## 8. Lesson 6：34D observation 是怎样拼出来的

### 阅读

- `centipede/env.py:27-45, 227-327`
- `docs/03_OBSERVATION_REWARD_ACTION_REFERENCE.md`

### 分组

```text
1D  realized wave
2D  CPG sin/cos
9D  root body-frame linear/angular/gravity
10D one-second contact and angular/load history
12D current 21-link spine geometry
----
34D
```

### Time-domain 10D

每个 physics substep 把 19 对脚的合力、root angular-speed 和 total load 写入 1000-step
ring buffer。只有完整积累 1 s 后这 10D 才启用。19 个 segment 的 contact standard
deviation 和 spike rate 各取前三个 spatial DCT mode，再加入四个全局 crest/load 指标。

这里既有时间统计，也有沿身体的空间压缩。不要把“contact10”误称为纯时间特征。

### Spatial-domain 12D

取 21 个主 body 的世界位置，以头部为原点，反旋转到 root frame；减去首尾 chord，
并按 chord length 归一化。随后计算 lateral/vertical residual 的前四个 DCT mode、
两个 curvature RMS、edge-length CV 和 tortuosity。

这 12D 不直接读取地形高度。它读取机器人被地形塑造成的瞬时空间姿态，因此仍然是
合法的 embodied terrain-response signal。

### 证据边界

排除 wave 和 phase 后的 31D subset 在测试 panel 的 1.44 s snapshot 上达到 0.9444
whole-boot balanced accuracy，phase-only control 为 0.3333。这证明存在可线性访问的
terrain-related information；它不证明完整 terrain reconstruction、任意地形泛化，
也不证明 PPO 实际使用了这些维度。

## 9. Lesson 7：Reward、failure、speed、return 是四件事

### 阅读

- `centipede/env.py:329-409`
- `scripts/evaluate.py:150-204`

### Reward

```python
reward = -(post_spine_centroid_x - pre_spine_centroid_x) / BODY_LENGTH_M
reward[terminal] = 0
```

canonical forward 是 world `-x`。reward 使用 61-link unweighted spine centroid 的
逐 policy-step净位移，除以 0.1025 m。没有 belly、energy、yaw、survival 或 smoothness
shaping。

### Failure

failure 只包含严重倾覆、root z<0 和非有限状态。慢、belly contact、leg crossing 不在
failure mask 中。time limit 是 truncation，不是物理 failure。

### Evaluation speed

只为完整跑完全时长的 survivor 计算：

```text
-(final_x - initial_x) / duration / body_length
```

无 survivor 的 cell 没有合法 speed。`episode_return` 是每步 reward 求和，会受 terminal
mask、wrapper reset 和时长影响，因此不是 speed 的同义词。

## 10. Lesson 8：PPO 到底学什么

### 阅读

- `config/rl_games.json`
- `scripts/train.py`
- `docs/02_RL_CONFIG_AND_TOOLS_LINE_BY_LINE.md` 第 3-4 节

### 网络

```text
normalized/clipped 34D observation
  -> shared MLP 256 ELU -> 128 ELU
  -> Gaussian action mean (1D)
  -> global trainable log sigma
```

Actor 和 critic 共享 trunk；action 在 wrapper/environment 处限制到 `[-1,1]`。PPO 不包含
人工 terrain classifier，也没有“rough 就升 wave”的公式。只有当 observation 中的
terrain response 与更高 pure-forward return 相关时，网络才可能学到条件策略。

### 一轮 epoch 的样本账本

- 32 env x 128 steps = 4096 transition。
- minibatch 256，因此每 mini-epoch 有 16 个 minibatch。
- 同一 rollout 优化 4 mini-epochs。
- gamma≈0.99749，GAE lambda≈0.98726，PPO clip epsilon=0.2。
- entropy coefficient=0，但 Gaussian sigma 仍提供探索。

训练 terrain 是三条 strip 的 tri terrain；每次 reset 选择一个 strip。terrain label、
absolute position、oracle roughness 不在 observation 中。

## 11. Lesson 9：训练、评估和 sweep 的角色必须分开

### `scripts/train.py`

创建 tri-terrain environment，注册 RL-Games vector wrapper，加载 JSON 配置并训练。
CLI 只覆盖 seed、epoch、env count 和输出路径，不改原始 JSON。

### `scripts/evaluate.py`

创建一个 homogeneous terrain。提供 `--fixed-wave` 时走 open loop，不加载 checkpoint；
否则加载三个冻结 seed 之一。它同时产生：

```text
metrics.json
traces.npz
all_link_world_poses.npz   only with --video
video/offline_45deg.mp4    only with --video
video/offline_45deg.audit.json
```

### `scripts/open_loop_sweep.py`

每个 `(terrain, terrain_seed, wave)` 都启动独立 Isaac 进程。这样慢，但一个 cell 的
PhysX/GPU 状态不会泄漏到下一个 cell。默认是 6 x 2 x 13 = 156 boots。每个 boot 内
32 env 是并行确定性条件，不是 32 个独立 replicate。

## 12. Lesson 10：视频是状态的显示，不是第二套动力学

### 内部核验

`evaluate.py --video` 在 Isaac 中不启动相机，只记录 251-link exact world poses。退出
simulation 后，`render_offline.py` 用冻结 MJCF/OBJ 几何在 CPU 离线渲染并审计帧。

### 高保真展示

`render_high_fidelity.py` 和 Blender scene builder 把 105 个视觉 mesh 绑定到 97 个已
记录刚体。它只改变显示层：头壳抬升、触须方向、腿段延长、尾壳缩短都不会改变 USD、
collision、controller、reward 或 metrics。

### Structure reveal

`render_structure_reveal_demo.py` 是唯一公开入口。120 fps 是对同一物理轨迹的展示插值，
不是 120 Hz physics/contact evidence。

## 13. 一条 closed-loop rollout 的完整追踪

当你执行：

```powershell
python scripts/evaluate.py --terrain l60a12 --policy-seed 161803 --num-envs 1 --headless
```

依次发生：

1. `AppLauncher` 启动 Isaac。
2. `CentipedeEnvCfg` 被改成 homogeneous L60/A12。
3. `CentipedeEnv` 构建 terrain mesh、robot、controller、gait map 和 buffers。
4. reset 用 collision footprint 计算 spawn z，并把 wave 重置到 2.5。
5. RL-Games 从 checkpoint 恢复 34D normalization、actor、critic 和 sigma。
6. policy 读取 34D observation，输出 1D deterministic mean action。
7. action law 更新 realized wave；gait map 插值六个参数。
8. 12 个 physics substep 内，CPG 和 implicit PD 驱动 211 joints。
9. environment 计算 pure-forward reward、failure、next observation。
10. evaluator 记录 wave、reward、alive、first_done 和 survivor speed。
11. 800 steps 后写入新输出目录并关闭唯一 GPU owner。

## 14. 修改项目时的代码纪律

1. 冻结 Clean 目录先保持只读；练习在复制目录或独立分支完成。
2. 一次实验只改一层：terrain、gait/controller、observation/reward 或 PPO config。
3. 每次修改先写 hypothesis、expected observable 和 falsification condition。
4. 修改后先跑 `check_project --quick` 和 CPU tests，再启动 Isaac。
5. 新运行永远写新 `--out`，不覆盖 canonical demo 或 retained evidence。
6. 模型/terrain/controller 变动必须建立新 hash/golden，不能把新值登记成旧值。
7. 分开报告 survival 和 survivor-conditioned speed。
8. 任何结果都保留 terrain seed、policy seed、wave trace 和 checkpoint parent。

## 15. 建议的七节课顺序

| 课次 | 内容 | 完成标志 |
|---|---|---|
| 1 | `model.py`、USD layer、项目完整性 | 能解释 21/61/211，并跑过 CPU checks |
| 2 | `terrain.py`、spawn | 能手画 source/collision/world 坐标转换 |
| 3 | `gait.py`、gait map | 能手算 action、realized wave 和六参数插值 |
| 4 | `controller.py` | 能追踪一个 body yaw 和一条腿的 target |
| 5 | `env_cfg.py`、`env.py` | 能复述一个 12 ms policy step 的调用链 |
| 6 | 34D observation、reward、done | 能从代码重建 34D 分组和 metric 公式 |
| 7 | PPO、train/evaluate/sweep | 能运行并审计一个小型 closed-loop 对比 |

每节课完成后再进入 `07_PI_REPRODUCTION_WORKBOOK.md` 对应题组。课程不提供题目答案；
完成结果交回后按源码、数值结果和实验纪律三部分评分。
