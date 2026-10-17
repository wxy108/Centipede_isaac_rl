# Runtime Line-by-Line Companion

本文按 clean source 的真实行号解释运行时。空行、`from __future__` 和普通 import 合并说明；所有有行为意义的代码块均覆盖。源码不含这些教学注释。

## 1. 模型索引：`centipede/model.py`

### 文件目的

USD 是物理模型，`ModelIndex` 是 Python 对 USD joint/body 命名的唯一解释器。它不允许“找差不多的名字”，因此换错模型会立刻失败。

| 行 | 解释 |
|---|---|
| 1-5 | 文件说明、类型注解兼容和 NumPy import。这里只处理离散 index，不启动 Isaac。 |
| 8 | 主 body segment 数为 21，即 `link_body_0..20`。 |
| 9 | body yaw joint 为 19 个，命名从 `body_yaw_1` 到 `body_yaw_19`。 |
| 10 | body pitch joint 为 20 个，命名从 `body_pitch_0` 到 `body_pitch_19`。第 0 个在 env 中单独作为 head pitch。 |
| 11 | body roll joint 为 20 个。 |
| 12 | 有 19 对腿，每个主躯干间隔对应左右腿。 |
| 13 | 每条腿 4 DOF。 |
| 14 | canonical body length = 0.1025 m，用来把位移归一化成 body length。不是 USD 自动测量值。 |
| 16 | 每条腿 4 个 family 的固定顺序：yaw、pitch、roll、knee。这个顺序决定 `legs[..., dof]` 的含义。 |
| 19-20 | `ModelIndex` 定义；强调 exact names。 |
| 22-24 | 从 Isaac articulation 的 `joint_names` 和 `body_names` 建立 name->index 字典。 |
| 26-34 | 两个 fail-loudly lookup。缺一个 joint/body 就抛异常，不悄悄跳过。 |
| 36-44 | 按固定命名构建三条 body joint index 数组。注意 yaw 从 1 开始，pitch/roll 从 0 开始。 |
| 45-51 | 构建形状 `(19,2,4)` 的腿 index tensor：segment、L/R、DOF family。 |
| 53 | `link_body_0` 是 head body，用于读取 head world yaw。 |
| 54-56 | `main_body` 精确包含 21 个主 body，用于 spatial geometry observation。 |
| 57-66 | `spine` 用命名规则收集所有非 leg、非 tarsus 的 link。它用于 reward/metric，共应有 61 个。此处规则只服务已经冻结的唯一 USD。 |
| 67-68 | 61-link metric contract 硬检查。 |
| 70-74 | 合并全部主动 joint，要求总数和 unique 数都为 211，防漏项或重复。 |

### PI 应记住

- `main_body=21` 服务空间形状 observation。
- `spine=61` 服务 forward reward/速度。
- 两者不是同一个集合，也不能互换。
- 211 是 actuator target 数，不代表模型里只有 211 个 joint/prim。

## 2. USD 模型层

5 个 USD 是 USDC 二进制，不适合文本逐行阅读。其职责按 composition layer 分开：

| 文件 | 角色 |
|---|---|
| `assets/centipede_rebuild_v1_renamed.usd` | 唯一入口 wrapper，组合下面四层。 |
| `configuration/..._base.usd` | 基础结构、prim hierarchy 和几何/质量基础。 |
| `configuration/..._robot.usd` | robot link/joint 拓扑与命名。 |
| `configuration/..._physics.usd` | PhysX articulation、drive、collision 等物理属性。 |
| `configuration/..._sensor.usd` | 传感相关 layer。 |

PI 不应直接用文本编辑器改 USDC。需要改模型时，在 Isaac/USD 工具里另存新版本，并同步更新 `ModelIndex` contract、5 个 asset hash、211 joint 和 61 spine tests。

## 3. 环境配置：`centipede/env_cfg.py`

| 行 | 解释 |
|---|---|
| 1-15 | 文件说明与 Isaac Lab config 类型 import。这个文件被 import 时需要 Isaac Lab，但还不会启动 app。 |
| 18-21 | 从文件位置推导项目根目录，随后定义唯一 USD、controller YAML、gait map JSON 的绝对路径。没有旧项目硬编码。 |
| 24-29 | `ROBOT_CFG` 指定每个 env 下 robot prim，并从唯一 USD spawn；打开 contact sensor 数据。 |
| 30 | 配置层初始 z=0.025 m，但真正 reset 时会按 terrain footprint 重新计算 z。 |
| 31-36 | 只创建一个名为 `active` 的 implicit actuator，匹配全部 body/leg/knee joint。 |
| 37-38 | actuator 的 effort/velocity 上限。它们是执行边界，不是 RL action bounds。 |
| 39-41 | 初始 stiffness/damping/armature 为 0；运行时 `env.py` 写入各 family 的 Kp/Kv。 |
| 47-50 | scene 默认 32 env，origin 间距 1.5 m。 |
| 51-53 | 安全 ground plane。rough mesh 另外创建；ground plane 不是 roughness 来源。 |
| 54-56 | 远光源，只影响显示。 |
| 57 | 把 robot cfg 安装进 InteractiveScene。 |
| 58-65 | contact sensor 匹配 head subtree 下的所有 `foot_*`，每 physics step 更新，保留 Isaac 自身 6 帧 history；clean observation 另有独立 1000-step ring buffer。force threshold=0.005 N。 |
| 68-76 | `CentipedeEnvCfg`：decimation=12、episode=9.6s、1 action、34 observation、0 critic state、physics dt=1ms。 |
| 72 | `is_finite_horizon=False` 让 time-limit truncation 可以 bootstrap value；不是说 episode 无限。 |
| 77-82 | 默认 viewer 相机位置、look-at 和 1280x720。只影响画面。 |
| 83 | 使用上面的 scene。 |
| 85-90 | terrain mode 与 homogeneous 参数。训练脚本设为 tri；评估脚本设 homogeneous。label 是元数据，policy 看不到。 |
| 91 | tri reset 选择 strip 的随机种子。它和 terrain surface seed 不是同一个概念。 |
| 92 | spawn footprint 顶部再加 25 mm clearance。 |
| 93 | 计算 spawn 支撑最高点时检查半径 75 mm。 |
| 94 | contact/time-domain feature window 固定 1 s，即 1000 physics substeps。 |
| 95 | `None` 表示 RL 控 wave；数值表示 fixed open-loop wave。 |

## 4. Terrain/arena：`centipede/terrain.py`

### 为什么没有 `arena.py`

旧项目的 `arena.py` 混合过多代 transition、block 和 sweep 逻辑。clean 项目把唯一需要的 terrain construction 放进 `centipede/terrain.py`。因此本文中的 arena 指这个文件，不应再寻找旧 `arena.py`。

| 行 | 解释 |
|---|---|
| 1-8 | NumPy 地形生成和 SciPy Gaussian smoothing。 |
| 11 | terrain 方形边长 3 m。 |
| 12 | heightmap 1536 x 1536。单 cell 约 1.955 mm。 |
| 13-14 | tri terrain 三类：flat、L90/A8、L60/A12。L/A 单位均为 mm。 |
| 15 | 三条 strip 的 y 中心为 -0.95、0、0.95 m。 |
| 16 | tri reset 的固定 x=0.65 m。 |
| 17 | 每条 strip 半宽 0.43 m。 |
| 20-23 | `_spectral_band` 建立世界坐标网格和 double-precision height array。 |
| 24-30 | 随机抽取若干窄频带二维正弦：频率、方向、相位都由 seed RNG 决定。 |
| 31-32 | 把合成场标准化到目标标准差 amplitude。 |
| 35-40 | `single_wavelength` 接收 mm，转 m；A=0 时严格返回全零。 |
| 41-45 | 中心 spatial frequency=`1/lambda`，实际采样 0.9-1.1 倍，共 8 个成分。 |
| 46-47 | 按每个周期像素数进行 Gaussian smoothing，避免高频离散噪声。 |
| 48-50 | 去均值，再按最大绝对峰值缩放到指定 amplitude。注意这里 amplitude 最终是 peak magnitude，不再是标准差。 |
| 53-58 | 创建全零 1536 方阵，计算 strip 宽和中心。 |
| 59-68 | 对三个 y strip 逐一生成：flat 保持零；rough 从同一完整随机场抽对应宽度、去 strip 均值并写入大图。 |
| 69 | 返回 source heightmap；它还没经过 MuJoCo PNG/collision convention。 |
| 72-73 | homogeneous terrain 就是覆盖整个 arena 的 single-wavelength surface。 |
| 76-81 | `to_mujoco_collision` 计算 source 的对称 peak 和至少 5 mm 的 vertical range。 |
| 82-85 | 平地变成常数 `0.5*z_max`；rough 则模拟 uint8 PNG quantization 后映射到 `[0,z_max]`。所以 collision 全部非负并有 z shift。 |
| 88-100 | `height_at` 把世界 x/y 映射到像素并双线性插值单点高度。 |
| 103-111 | `footprint_max` 不看单点，而看半径内所有像素最高值；spawn 使用它避免躯体 footprint 穿地。 |
| 114-121 | `build_mesh` 只在 Isaac app 内运行；先转 collision height，再计算网格 cell/half size。 |
| 122-125 | 为每个 height pixel 创建 `(x,y,z)` 顶点，最终转 float32。 |
| 127-135 | 每个方格拆成两个三角形，建立 vertex indices 和每面 3 顶点计数。 |
| 137-143 | 在 USD stage 创建无 subdivision 的 mesh，并应用 collision API。 |
| 144 | 返回实际 collision heightmap，reset/spawn 必须用它而不是 source map。 |

### 地形名怎么读

- `L90/A8`：主要波长 90 mm，height source 峰值幅度 8 mm。
- 不能仅用 `A/L` 当唯一 complexity；旧实验已经显示 wavelength、amplitude、局部坡度和机器人尺度共同作用。
- `flat` 在 collision convention 下不是 z=0 array，但它是几何平面；spawn 用同一 collision array，因此一致。

## 5. Action 与 gait map：`centipede/gait.py`

| 行 | 解释 |
|---|---|
| 1-9 | 文件包含 1D command law 和 wave-conditioned 6-parameter map。 |
| 12-20 | 冻结常数：wave 1-4、reset 2.5、tau 0.25s、rate 3.5/s、policy dt 12ms。 |
| 22-24 | 原始 action 先 clamp 到 [-1,1]，线性映射为 target wave=`2.5+1.5a`。 |
| 27-33 | `step` 比较 target 与上一时刻 realized wave，保留方向与误差大小。 |
| 34 | `release_error=rate*tau=0.875`，是从 rate-saturated 区切换到一阶区的误差。 |
| 35-36 | 小误差区使用精确离散一阶响应，不是 Euler approximation。 |
| 37-40 | 大误差先按 3.5/s 匀速接近；若一个 12ms step 尚未离开饱和区，增量正好 0.042。 |
| 41-44 | 若在本 step 内跨入一阶区，解析计算剩余指数衰减。 |
| 45 | 最后 clamp realized wave 到 [1,4]。因此 target=1 不等于这一帧就 realized=1。 |
| 48-52 | gait map 每个 wave knot 的 6 个变化参数：两个 amplitude scale、body yaw Kp/Kv、legbase yaw Kp/Kv。 |
| 54-57 | 读取 JSON，按 wave 排序。 |
| 58-61 | 要求 wave knot 恰好是 1.0 到 4.0、间隔 0.25，共 13 点。 |
| 62-64 | 每个 knot 必须有全部 6 个 field。 |
| 65 | leg pitch scale 是全 wave 共享常数，不属于 6 个变化参数。 |
| 66-72 | 把 knot 和数值常驻到目标 torch device。 |
| 74-78 | 对任意 realized wave 找相邻 knot，并计算线性插值系数 alpha。 |
| 79-82 | 六参数逐项线性插值，另补固定 leg pitch scale。 |
| 83 | 返回字典给 env；不存在“最近 knot 取整”。 |

## 6. CPG controller：`centipede/controller.py`

### 总原则

Controller 只输出 joint position/velocity reference。它不直接施加 torque；反馈力全部来自 PhysX implicit PD。不要把 CPG coupling gain 与 joint Kp/Kv 混为一谈。

| 行 | 解释 |
|---|---|
| 1-15 | 文件目的、math/torch/yaml 和唯一 model index import。 |
| 18-20 | 从 world quaternion 提取 yaw，供 head-yaw reference。 |
| 23-33 | `dct_blend` 是按 joint/segment 顺序错开的半余弦 startup blend；`settle` 前 0，ramp 完成后 1。这里名字沿用历史，但它不是 DCT transform。 |
| 25 | 用 float64 计算 blend，减少 startup parity 漂移，最终再转 float32。 |
| 28-33 | 每个后续 segment 稍晚启动，持续时间为 `ramp*0.5`。 |
| 36-45 | body Kuramoto chain：每个 oscillator 有同一 intrinsic omega，相邻 phase error 通过 forward/backward sine coupling 修正。 |
| 48-49 | leg oscillator 与传入的 segment-center anchor 耦合，不是 leg-to-leg chain。 |
| 52-67 | `ReferenceGenerator` 初始化，读取 YAML，禁止旧 `impedance` key，并建立 exact ModelIndex。 |
| 69-76 | 把 NumPy index 转到 torch device，缓存 body/pitch/roll/leg/head body index。 |
| 78-82 | 读取 base body wave：0.4 rad base amplitude、1 Hz、base wave number 3、speed scale 1。 |
| 84-88 | CPG 必须启用。YAML coupling 分别乘 40 和 80，这是冻结 runtime override。 |
| 90-97 | `reference_targets` 只允许 7 个 key；出现历史残留 key 会失败。 |
| 98-104 | 读取 settle/ramp、head pitch、head yaw gain/clip/rate。 |
| 106-108 | 20 个 pitch target 默认 0，仅前 `head_pitch_joints` 个设 offset。当前只有第 0 个。 |
| 110-114 | 读取四腿 DOF 的 base amplitude、phase offset、DC offset 和 active set。 |
| 116-119 | 左右腿 sign 交替，形成镜像摆动。 |
| 120-127 | 把 4-DOF 数值重复到 19x2 条腿，并形成 active mask。当前 active 只有 yaw/pitch；roll 和 knee 无 CPG swing。 |
| 121 | 单独保留原始 Python float yaw amplitude 作为 leg coupling，避免 float32 round-trip 破坏 golden parity。 |
| 128-129 | 为 flatten 后每个腿 DOF 保存 segment index 与 dof index。 |
| 131-135 | 构造 base wave=3 在 19 body-yaw oscillators 上的空间 phase。 |
| 137-147 | 每 env runtime scale、CPG phase、head reference 和 previous targets 的状态。 |
| 149-151 | realized wave=`wave_scale*base_wave_number`。 |
| 153-163 | `set_gait` 强制每个输入形状为 `(num_envs,)`，保存 wave 与三个 amplitude scale。Kp/Kv 不在 controller 内写，而在 env 写。 |
| 165-169 | temporal angular frequency 固定 `2*pi*1Hz`，wave number 不改变 temporal frequency。 |
| 171-176 | `_spatial` 与 `_delta` 把 per-env wave scale 应用到绝对 spatial phase 和相邻期望 phase lag。 |
| 178-180 | `_leg_phase_anchor` 使用 `body_phase + 0.5*delta`。腿根在两个 body-yaw joint 之间，所以采样节段中心；这不是 YAML 的腿内 phase offset。 |
| 182-185 | 第一次过 settle 时初始化 body oscillator，并以 segment-center anchor 初始化 leg oscillator。 |
| 187-217 | reset 指定 env 的 phase、head reference、previous target 与所有 scale。scale reset 为 1，随后下一次 policy step 会由 gait map 覆盖。 |
| 218-221 | 每个 1ms physics step 进入 controller；至少一个 env 过 settle 时初始化 CPG。 |
| 222-237 | CPG 初始化后更新 body chain；每条腿跟随对应 body sample 加半个相邻 lag 的中心 anchor。注意这里每 physics step 更新，不是每 12ms。 |
| 239-246 | 读取 head world yaw；reset 后第一帧把当前 yaw 作为 reference。 |
| 247-252 | 如配置非零 head yaw rate，settle 后 reference 会匀速旋转。当前为 0。 |
| 253-256 | yaw error wrap 到 [-pi,pi]，乘 gain 并 clip。当前 gain=0，所以首个 body yaw target 为 0。 |
| 258-273 | body-yaw 其余 18 个 joint 按 staggered blend、per-wave body scale、base amplitude 和 oscillator sine 生成 target。 |
| 274-277 | target velocity 由相邻 physics step target 的有限差分得到。 |
| 279-287 | head/body pitch 只做一次 startup blend，目标是固定 head pitch offset 向量。 |
| 289-306 | 为 19 个腿 segment 建 blend；CPG 与非 CPG 路径都使用同一 segment-center phase；yaw DOF 乘 gait-map yaw scale，pitch DOF 乘固定 pitch scale。 |
| 307-315 | 再叠加左右镜像、amplitude、YAML 腿内 phase offset 与 DC offset；inactive DOF 始终保持 DC。当前 knee DC=pi/6。 |
| 316-317 | 计算腿 target velocity 并保存 previous。 |
| 319-330 | 建立完整 `(env,joint)` position/velocity target，按 exact indices 填 body yaw、pitch 和所有腿；body roll target 保持 0。 |
| 332-336 | 第一个 target 或异常小 dt 时 velocity=0，否则 `(current-previous)/dt`。 |

### 当前 controller YAML 对动作的实际影响

- body yaw 实际角度幅值 = `0.4 * body_amp_scale(wave)` rad。
- legbase yaw 实际角度幅值 = `0.6 * leg_yaw_amp_scale(wave)` rad，左右符号相反。
- legbase pitch 实际角度幅值 = `0.4 * 1.15 = 0.46` rad。
- legbase roll 不 swing，target=0。
- knee 不 swing，target=pi/6。
- head pitch target=-0.57 rad 是 joint target，不等同于世界 head pitch；世界姿态还取决于全身弯曲和接触。

## 7. 环境：`centipede/env.py`

### 常量和 helper

| 行 | 解释 |
|---|---|
| 1-24 | 导入 DirectRLEnv 和唯一 controller/config/gait/model/terrain。 |
| 27 | contact feature 有 19 个 longitudinal segment，每段左右脚合并。 |
| 28 | 每段力除以 0.05 N 做数值尺度归一化。 |
| 29 | spike threshold 是对应时间 median 的 3 倍。 |
| 30 | spatial geometry 使用 21 个 main body。 |
| 31 | root angular velocity observation 乘 0.25。 |
| 32 | failure tip threshold `abs(projected_gravity_z)<0.259`，约等于躯干竖直方向与重力夹角超过 75 度。 |
| 34-45 | 34 个 observation 名称及固定顺序；checkpoint 输入顺序依赖它。 |
| 47-56 | 8 个 joint family 的 Kp/Kv。body/leg yaw 的值会被 gait map 每个 policy step覆盖，其余固定。 |
| 59-67 | 创建正交 DCT-II basis。contact 19->3 modes，spine 21->4 modes。 |
| 70-75 | quaternion inverse rotation，把 world relative vectors转换到 root body frame。 |

### 初始化与 scene

| 行 | 解释 |
|---|---|
| 78-88 | 强制 34D/1D 和 1ms/12ms contract，再调用 Isaac `DirectRLEnv` 初始化。 |
| 90-95 | scene 创建后建立 model index、controller、gait map 和 action buffer。 |
| 96-102 | fixed wave 或 2.5 初始化 wave state；非法 wave 立即失败；建立 sim clock、reward pre-position 和 tri strip RNG。 |
| 104-116 | 1s 必须等于 1000 substeps；为 contact、omega、total load 建 ring buffers 和 DCT bases。 |
| 118-130 | 建 8 family index、填 Kp/Kv、合并 active ids 并硬检查 211。 |
| 132-134 | 录视频模式先 render 30 帧，让 camera/sensor 初始化。 |
| 136-143 | `_setup_scene` 在 articulation 初始化前停用 USD 转换残留的 `Robot/worldBody` subtree，再取得 scene robot。 |
| 145-155 | 根据 cfg 只允许 tri 或 homogeneous 两种 terrain source。 |
| 156-160 | source -> USD collision mesh，并把 ground/terrain 作为 global collision prim。 |
| 162-177 | 把 211 joints 划成 8 个不重叠 family；第一个 pitch 单独作为 head。 |

### 每个 policy step 与 physics substep

| 行 | 解释 |
|---|---|
| 179-182 | policy action clamp；在物理推进前保存 61-link spine centroid x，供本 step reward。 |
| 184-191 | RL 模式通过 `WaveActionLaw.step` 更新；open-loop 模式强制固定 wave。 |
| 192-196 | realized wave 在 13-knot gait map 插值，并传 amplitude scale 给 controller。 |
| 197-201 | body yaw 与 legbase yaw Kp/Kv 同 wave 联动，标记下一 physics step 写入 simulator。 |
| 203-205 | 每个 1ms substep先让 CPG 生成完整 targets。 |
| 206-213 | gait 变化时写 stiffness/damping 到 PhysX 与 actuator runtime buffer；避免只改 tensor 不生效。 |
| 214-223 | 写 joint position/velocity target；explicit effort target 为 0，确保只有 implicit PD force path。 |
| 224-225 | sim clock +1ms，并采集 contact window。 |

### Contact/time-domain observation

| 行 | 解释 |
|---|---|
| 227-242 | 第一次使用时把 sensor body names 解析成 19x2 足 index。任一足名缺失则暂不采集。 |
| 244-252 | 取每只脚 net force magnitude，左右合并；归一化并 clip contact，另算全身 total load 和 root angular speed。 |
| 253-258 | 按每 env ring position 写入 1s buffer，循环覆盖；记录已填长度。 |

### 34D observation 组装

| 行 | 解释 |
|---|---|
| 260-262 | Isaac 构造早期 controller 尚不存在时返回合法 34D 零向量，避免 init order 崩溃。 |
| 264 | 第 0 维是 realized wave，不是 raw action/target wave。 |
| 265-270 | 第 1-2 维是第一个 body oscillator 的 sin/cos；CPG 未启动时设 `(0,1)`。 |
| 271-275 | 第 3-11 维：root body-frame linear velocity、0.25-scaled angular velocity、projected gravity。 |
| 277-281 | 从 1s buffers 计算中位数；只有 ring 填满后 contact10 才启用。 |
| 282-285 | 每 segment contact 标准差和超过 3x median 的 spike fraction。 |
| 286-292 | global 4 特征：angular crest log ratio、angular spike rate、total-load std、total-load spike rate。 |
| 293-297 | 19 segment 的 std/spike 各压缩成前 3 DCT mode，加 global4，得到 10D。未满 1s 时整体乘 0。 |
| 299-304 | 取 21 body world position，以 head 为原点并 inverse-rotate 到 root frame。 |
| 305-310 | 建首尾 chord，减去线性 chord 得到形状 residual，再按 chord length 归一化。 |
| 311-313 | 二阶 spatial difference 近似 curvature；相邻 body edge length 及均值。 |
| 314-321 | lateral/vertical residual 各前 4 DCT mode，加 y/z curvature RMS、edge CV、tortuosity，得到 12D。 |
| 322 | 拼成 `1 wave + 2 phase + 9 root + 10 contact + 12 spatial = 34`。 |
| 323-327 | 硬检查形状和 finite，返回 RL-Games 所需 `{"policy": obs}`。 |

### Failure、reward、reset

| 行 | 解释 |
|---|---|
| 329-338 | failure 条件：严重倾覆、root z<0、root pos/vel 非有限。它不包含 belly-drag、慢速或 leg crossing。 |
| 340-345 | reward 计算当前 61-link centroid x，取 `-(post-pre)/0.1025`。canonical forward 是 -x。 |
| 346-347 | failure 或 episode terminal 的当步 reward 置零，防 reset/terminal 跳变污染 forward reward。 |
| 348-352 | 输出训练 log：forward reward、realized wave、failure rate。没有额外 shaping reward。 |
| 355-359 | terminated=failure，truncated=time limit。 |
| 361-367 | reset env ids；先调用 Isaac reset，再取得 default root 并加 env origin。 |
| 368-374 | tri 模式每 env 随机选三条 strip；homogeneous 模式交替 y=-1/+1，减少完全共位但保持同 terrain。 |
| 375-384 | 用 collision heightmap 的 75mm footprint maximum +25mm clearance 算 z，再写 exact xyz。 |
| 385-392 | 写 root pose/velocity 和 default joint state。 |
| 394-409 | 清零 sim time、wave、controller phase、1s ring buffers，标记 gains 下一步重写。 |

## 8. 一帧到底发生什么

一个 policy step 的顺序可简化为：

1. PPO/fixed mode 给 1D action。
2. `_pre_physics_step` 保存 reward 起点，更新 realized wave，插值 gait map，更新 yaw Kp/Kv。
3. 连续 12 次 `_apply_action`：CPG phase 走 1ms、写 211 targets、PhysX implicit PD 推进、采 contact。
4. `_get_rewards` 用 12ms 前后 spine centroid 差给纯 forward reward。
5. `_get_dones` 判断翻转/非法状态/9.6s time limit。
6. `_get_observations` 组装下一帧 34D。

这也是为什么“RL update 12ms”不表示 CPG/PD 只每 12ms 更新。CPG target 和 physics 仍是 1ms。
