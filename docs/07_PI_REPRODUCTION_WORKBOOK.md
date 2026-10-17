# Clean RL 复现作业册

## 使用规则

本作业册不附答案。完成每题时提交：

1. 你的解释或推导；
2. 运行命令；
3. 关键输出文件完整路径；
4. 你认为结果支持什么、不支持什么；
5. 是否修改代码，以及修改文件的 diff。

不得在冻结 Clean 源码中直接练习。概念题只读源码；编程题使用项目副本或独立分支；
运行结果写入新的 `outputs/course_*`。任何 GPU 题先确认只有一个 Isaac owner。

## Level A：不启动 Isaac 的代码阅读题

### A1. 项目拓扑口试

不用查看文档，只看 `centipede/model.py`，回答：

1. 21、61、211 分别是什么集合？
2. 一共有多少条腿、每条腿几个被索引的 DOF？
3. 为什么 `body_yaw_0` 不在 body-yaw 数组中？
4. 哪个 index 集合进入 reward，哪个进入 spatial observation？
5. 如果 USD 少一个 knee joint，程序会在哪一层、以什么方式失败？

验收：所有答案必须引用源码位置，不能只复述数字。

### A2. Action law 手算

不运行代码，推导下列序列：

```text
initial realized wave = 2.5
actions = [-1, -1, -1, 0, +1]
dt = 0.012 s
```

写出每一步 target wave、所处分支（rate-saturated 或 exponential）和新的 realized wave。
再写一个最小 Python 脚本调用 `WaveActionLaw.step` 验证手算。

验收：手算与程序误差小于 `1e-12`，并解释为什么短时间内到不了 wave=1。

### A3. Gait-map 插值

对 realized wave `1.625`、`2.625`、`3.875`：

1. 找到左右 knot。
2. 写出 alpha。
3. 手算 `body_amp` 和 `legbase_yaw_kp`。
4. 用 `GaitMap.at` 验证。
5. 把 scale 换算成 body-yaw 和 leg-yaw 的实际 rad amplitude。

验收：说明 `leg_yaw_amp=0.x` 为什么不是 0.x rad。

### A4. Terrain 坐标题

只使用 `terrain.py` 的纯 NumPy 函数：

1. 生成 flat、L90/A8、L60/A12 的 source 和 collision map。
2. 比较 source/collision 的 min、max、mean。
3. 在同一 `(x,y)` 上比较 `height_at` 与半径 75 mm 的 `footprint_max`。
4. 解释为什么 flat collision map 不是全零仍表示平面。
5. 画出 row/col 与 world x/y 的对应关系。

验收：不得启动 Isaac；结果必须包含 terrain seed。

### A5. 34D observation 账本

从 `OBSERVATION_NAMES` 出发，建立一张 34 行表：index、name、单位或尺度、数据源、
时间窗口、是否直接依赖 realized wave、是否属于合法 terrain-response signal。

验收：总维数必须严格等于 34；必须说明 contact10 中哪些是沿身体的 DCT 特征。

## Level B：源码追踪与小型实现题

### B1. 手画一次 12 ms 调用链

从 policy action 开始，画出 `_pre_physics_step`、12 次 `_apply_action`、reward、done、
observation 的顺序。每个节点标注：调用频率、读什么状态、写什么状态。

验收：必须区分 policy frequency、CPG frequency 和 PhysX integration frequency。

### B2. 追踪一个关节

任选：`body_yaw_10` 或 `legbase_yaw_10_L`。从 wave=2.5 出发，追踪它在一个 physics
substep 中使用的：index、phase、amplitude、blend、position target、velocity target、
Kp、Kv 和最终执行路径。

验收：明确指出哪些值来自 YAML、gait-map JSON、runtime state 和硬编码 impedance。

### B3. Observation 重建器

在独立练习文件中实现两个不依赖 Isaac 的函数：

1. 给定 19-segment x 1000-step contact array，重建 contact std/spike 的前三个 DCT mode。
2. 给定 21-link body-frame positions，重建 spatial 12D。

用合成数据建立至少三类测试：直线、单峰弯曲、左右交替弯曲。

验收：输出维数分别为 6 和 12；说明 DCT mode 对均匀变化与局部变化的响应差异。

### B4. Reward 与 speed 对账

构造一条 10-step spine-centroid x 序列，其中包含一次 terminal。手算每步 reward、
episode return 和 survivor speed 的合法性。再写代码验证。

验收：必须说明 terminal step 为什么置零，以及无 survivor 时 speed 为什么应为缺失而非 0。

## Level C：单环境 Isaac 实验

### C1. 最小 smoke

运行 1 env、flat、fixed wave 2.5、20 steps，输出到 `outputs/course_smoke`。

你需要报告：

- survivors；
- first_done_step；
- duration；
- metrics/traces 路径；
- 这个实验能证明什么、不能证明什么。

验收：不能用 0.24 s smoke 比较 locomotion performance。

### C2. 平地三波数对比

在同一 terrain seed 上分别运行 wave 1.5、2.5、3.5，1 env、800 steps。只给一个预先
定义的 run root，不覆盖 canonical demo。

提交：survival、speed、return、净位移和 realized-wave trace。解释 speed 与 return 的
数量关系。

验收：运行前写出你的预期；运行后不得根据结果重新定义成功标准。

### C3. Spawn 与动力学 failure 鉴别

选择一个较 rough terrain，录制一条 9.6 s world-pose trace。根据 `first_done_step`、
初始 root/terrain clearance、姿态时间序列，把 failure 分成：spawn-time invalid、early
settling、motion-induced、survived。

验收：必须使用 collision-coordinate footprint，而不是只凭视频主观判断。

## Level D：RL 入口复现

### D1. 读取一个 checkpoint

不训练，检查三个 checkpoint 的：epoch、first-layer input width、action-head width、
running observation statistics。解释为什么它们只能加载 34D/1D contract。

验收：使用安全的 `weights_only=True` 路径；不修改 checkpoint。

### D2. Open loop 与 closed loop 公平比较

在 flat 和 L60/A12 上，比较：

- fixed wave 1.5；
- fixed wave 2.5；
- retained policy seed 161803。

每个条件使用相同 terrain seed、env count、steps 和 metric definition。绘制 realized wave
随时间曲线，并将 survival 与 survivor speed 分成两个 panel。

验收：不能把 32 env 称为 32 independent replicates；不能把 policy raw action 当 realized wave。

### D3. 一 epoch 训练接口复现

在独立输出目录运行 seed 314159、1 epoch、32 env。记录：总 transition 数、rollout horizon、
minibatch 数、mini-epochs、checkpoint/log 输出和 GPU owner 的退出状态。

验收：解释为什么 1 epoch 只证明接口和数据流，不证明策略学会 terrain adaptation。

### D4. PPO 代码口试

回答：

1. `rho_t` 比较哪两个策略？
2. `A_t>0` 和 `A_t<0` 时 clipped objective 分别限制什么？
3. input normalization 参数存在哪里？
4. `fixed_sigma=true` 是否表示没有探索？
5. entropy coefficient=0 对探索意味着什么？
6. PPO 为什么“有能力”利用 terrain information，但不保证真的利用？

## Level E：小型科学复现

### E1. 3 x 3 open-loop pilot

运行 terrain `{flat,l90a8,l60a12}` x wave `{1.5,2.5,3.5}`，一个 terrain seed。提前写出：

- primary metric；
- survival gate；
- 允许缺失 speed 的规则；
- 你预测的 speed/robustness pattern；
- 哪些结果会推翻预测。

运行后生成 speed、survival 两张图，不做 scalar weighted reward。

验收：结论只能约束到这个 seed 和这九个 cell。

### E2. 第二 terrain seed

只在 E1 最有信息的三个 cell 上增加第二 terrain seed。比较结果是否跨 seed 保持，不把
新增 seed 当成事后挑选。

验收：明确什么是 independent evidence，什么只是同一 boot 内 parallel condition。

### E3. Closed-loop 最小趋势检查

选两个 policy seed、三个 terrain、两个 terrain seed，预注册 wave summary、survival 和
speed 的报告方法。检查 roughness 增加时 policy realized-wave distribution 是否变化。

验收：即使 wave 变化方向符合预期，也必须单独确认 performance；不能只凭 command trend
宣称策略有效。

## Final：从空白页复现一条完整研究链

不看运行手册，完成以下任务：

1. 从冻结版本验证开始。
2. 解释模型、terrain、controller、observation、action、reward 和 PPO contract。
3. 运行一个 fixed-wave baseline 和一个 retained policy。
4. 生成 metrics、trace 和一条核验视频。
5. 审计 spawn、survival、speed、return 和 realized wave。
6. 写一页结论，分开列出“证据支持”“证据不支持”“下一实验”。

## 评分标准

| 项目 | 分值 |
|---|---:|
| 能从真实源码还原调用链 | 25 |
| 数值推导和输出对账正确 | 20 |
| 能区分 action target 与 realized wave | 10 |
| 能区分 reward、return、survival 和 speed | 15 |
| seed、replicate、缺失值和 provenance 纪律 | 15 |
| 命令可复现、输出路径清晰 | 10 |
| 结论没有越过证据边界 | 5 |

总分 100。85 分以上才进入“修改 observation/controller 或继续 closed-loop 研究”的课程；
此前以读懂和复现冻结主线为主。

## 提交模板

```text
题号：
我的预测：
源码依据：
推导或实现：
运行命令：
输出路径：
关键数值：
结果支持：
结果不支持：
我仍不确定：
```

第一次建议提交 A1、A2 和 B1。它们不需要 GPU，但能快速判断你是否已经真正掌握
Clean RL 的骨架。完成后由老师逐题追问并给出下一组，不直接公布完整答案。
