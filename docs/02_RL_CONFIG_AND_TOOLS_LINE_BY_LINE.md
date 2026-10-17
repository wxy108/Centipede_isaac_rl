# RL, Configuration, and Tools Line-by-Line Companion

## 1. Controller 数值：`config/controller.yaml`

| 行 | 字段 | 解释 |
|---|---|---|
| 1-5 | `body_wave` | base body yaw wave：amplitude 0.4 rad、temporal frequency 1 Hz、base wave number 3、spatial speed scale 1。runtime wave 通过 scale 改变空间相位，不改变 1 Hz。 |
| 6-8 | `head_hold/ramp_*` | 明确保留为 0，阻止旧 controller 默认值偷偷抑制 head 相关 joints。这三个值在 clean generator 中不参与计算，但保留用于 numeric-lineage/parity 审计。 |
| 9-13 | `leg_wave` | 四 DOF base amplitude `[0.6,0.4,0,0]`，phase offset、DC offset 和 active DOF `[yaw,pitch]`。knee 无 swing、DC=pi/6。 |
| 14-17 | `cpg` | CPG 启用；YAML coupling -2.5，Python 再分别乘 40/80。负号决定 coupling phase-error 方向。 |
| 18-25 | `reference_targets` | head yaw 控制关闭；head pitch joint target -0.57 rad；启动前 settle 1s，随后 0.25s staggered cosine ramp。 |

重要：这个 YAML 不含 Kp/Kv。固定 Kp/Kv 在 `env.py::IMPEDANCE`，wave-dependent yaw Kp/Kv 在 `gait_map.json`。

## 2. 13-knot gait map：`config/gait_map.json`

| 行 | 解释 |
|---|---|
| 1-4 | JSON 根、版本、固定 leg pitch scale=1.15、wave map 开始。 |
| 5-17 | wave=1.0..4.0、步长 0.25 的 13 行。每行绑定 6 个数：body amplitude scale、leg yaw amplitude scale、body yaw Kp/Kv、legbase yaw Kp/Kv。 |
| 18-19 | 数组与 JSON 结束。 |

### 六参数到底怎样落到实际仿真

| map 字段 | 最终作用 |
|---|---|
| `body_amp` | 乘 controller base body amplitude 0.4 rad。 |
| `leg_yaw_amp` | 乘 controller base leg yaw amplitude 0.6 rad。它是 scale，不是 rad。 |
| `body_yaw_kp/kv` | 直接写 body yaw implicit-PD stiffness/damping。 |
| `legbase_yaw_kp/kv` | 直接写 legbase yaw implicit-PD stiffness/damping。 |

非 knot wave 用线性插值。例如 realized wave=2.625 会在 2.5 与 2.75 两行之间取 alpha=0.5。

这张 map 是 frozen calibration，不代表每个点是全局最优，也不应被解释为独立优化得到的物理定律。它把 wave 与六参数绑定，后续实验选 wave 时必须同时选对应插值参数。

## 3. PPO 配置：`config/rl_games.json`

### 算法与网络，行 1-10

| 行 | 解释 |
|---|---|
| 1-2 | RL-Games 顶层 `params`。 |
| 3 | `a2c_continuous` 是 RL-Games 的 continuous PPO/A2C implementation 名。真正开启 PPO 见 line 19。 |
| 4 | Gaussian continuous action model，学习 mean，使用 log standard deviation。 |
| 5-7 | shared actor-critic trunk，而不是 actor/critic 完全分离网络。 |
| 8 | action mean/sigma 无末端 activation；fixed_sigma=true 表示 sigma 是可训练全局参数而非 observation-dependent head；初始 log sigma=0。环境 wrapper 最后 clip action 到 [-1,1]。 |
| 9 | MLP 两层 `[256,128]`，ELU，无 D2RL、无额外 regularizer。checkpoint 验证第一层 `(256,34)`，mean head `(1,128)`。 |

### Seed、clip 与 device，行 11-20

| 行 | 解释 |
|---|---|
| 11 | JSON 默认 seed 4242；`train.py --seed` 会覆盖。 |
| 12 | observation clip 到 [-5,5]，action clip 到 [-1,1]。input normalization 在 clip 前后由 wrapper/RL-Games contract处理。 |
| 14-18 | run 名、env registry 名、单 GPU `cuda:0`、不启用 multi-GPU。 |
| 19 | PPO clipping objective 开启。 |
| 20 | mixed precision 关闭，使用常规 float32。 |

### Normalization 与 reward，行 21-24

| 行 | 解释 |
|---|---|
| 21 | observation 使用 running mean/std normalization；checkpoint 保存 34-wide统计量。 |
| 22 | value target normalization。 |
| 23 | advantage normalization。 |
| 24 | reward scale=1，没有额外 reward shaping。环境 reward 本身只有 pure forward displacement。 |

### PPO dynamics，行 25-42

| 行 | 解释 |
|---|---|
| 25 | gamma≈0.99749。每步 12ms，对长期 reward 衰减较慢。 |
| 26 | GAE lambda/tau≈0.98726。 |
| 27-29 | learning rate 2e-4，constant schedule。 |
| 30 | KL threshold 0.008；constant LR 下主要作为算法配置/监控边界，不要当 hard trust region 保证。 |
| 31 | PPO ratio clip epsilon=0.2。 |
| 32 | value loss clipping 开启。 |
| 33 | critic/value loss coefficient 2。 |
| 34 | entropy coefficient 0；没有显式 entropy bonus，探索主要来自 Gaussian policy sigma。 |
| 35 | action bounds loss coefficient 1e-4。 |
| 36-37 | gradient norm clip 1.0 并启用 truncation。 |
| 38 | 每 env 每 epoch rollout 128 policy steps=1.536s。 |
| 39 | minibatch 256。32 env x128=4096 samples/rollout，即 16 minibatches。 |
| 40 | 每批数据优化 4 mini-epochs。 |
| 41 | sequence length 4；当前网络非 RNN，主要由 RL-Games data loader contract使用。 |
| 42 | time-limit truncation 做 value bootstrap。 |

### Run length，行 43-47

| 行 | 解释 |
|---|---|
| 43 | 默认 24 epochs；`train.py --epochs` 可覆盖。 |
| 44-45 | epoch1 后可保存 best，每 6 epoch定期保存。 |
| 46 | `score_to_win` 设极大，防提前停止。 |
| 47 | 默认 32 actors/envs；train CLI 可覆盖。 |

### PPO objective 直觉

对旧策略产生的样本，定义概率比：

```text
rho_t = pi_new(a_t | o_t) / pi_old(a_t | o_t)
```

优势 `A_t>0` 表示该 action 比 critic 预期好，训练希望提高它的概率；`A_t<0` 则降低。clipped objective 为：

```text
min(rho_t * A_t, clip(rho_t, 0.8, 1.2) * A_t)
```

它限制单次 update 利用同一批 rollout 时的概率变化，减少策略骤变。它不直接分类地形，也没有写入“rough 就用高 wave”的公式。若 34D observation 中的 terrain response 有助于提高 pure forward return，MLP 才可能学到条件策略。

## 4. 训练入口：`scripts/train.py`

| 行 | 解释 |
|---|---|
| 1-10 | 训练脚本说明、CLI/JSON/path 和 AppLauncher。必须先启动 Kit，再 import大部分 Isaac runtime。 |
| 13-20 | CLI：seed、epochs、num-envs、输出目录，加 Isaac 标准参数如 `--headless`/`--device`。 |
| 21-22 | 真正启动 Isaac app。 |
| 24 | 把 clean root 放到 import path。 |
| 26-32 | app 启动后 import RL-Games wrapper、runner、clean env/config。 |
| 35-42 | 读 frozen JSON；用 CLI 覆盖 seed、epochs、actors、train dir、experiment name。不会原地改 JSON。 |
| 44-48 | 建 env cfg，写 seed/env count，强制 terrain mode=`tri`，实例化 environment。 |
| 49 | RL-Games wrapper：device cuda:0，observation clip 5，action clip 1。 |
| 50-56 | 在 RL-Games registry 注册 vector env；creator 返回已构造的唯一 wrapped env。 |
| 57-59 | Runner 加 Isaac observer，加载配置并 reset algorithm。 |
| 60-63 | 执行 train；无论成功失败都关闭 wrapper。 |
| 66-70 | 脚本级 finally 确保 simulation app 关闭，避免残留 GPU owner。 |

训练命令：

```powershell
python scripts\train.py --seed 161803 --epochs 24 --num-envs 32 --headless --out outputs\train
```

## 5. 评估入口：`scripts/evaluate.py`

### CLI 与 app，行 1-51

| 行 | 解释 |
|---|---|
| 1-11 | 评估 fixed open-loop 或 checkpoint policy；`subprocess` 仅用于仿真后的 CPU 离线渲染。 |
| 14-22 | homogeneous terrain 字典。flat 的 lambda=1000 只是 label-compatible placeholder，A=0 时严格平地。 |
| 24-40 | terrain/seed/policy/checkpoint/fixed-wave/envs/steps/out/video 参数。`fixed-wave` 一旦提供，就不会加载 policy。video 还可设 fps/width/height。 |
| 41-44 | 加 Isaac 参数并启动 camera-free app；`--video` 不设置 `enable_cameras`。 |
| 46-56 | app 后 import NumPy、Torch、RL-Games 和 clean runtime。 |

### Policy 装载，行 59-81

| 行 | 解释 |
|---|---|
| 59-61 | `_spine_x` 使用和 reward 同一 61-link unweighted centroid。 |
| 64-68 | `_policy` 读取 frozen config，切换 checkpoint load，设置当前 actor 数。 |
| 69-75 | 注册 eval vector env；仍复用已构造 wrapper。 |
| 76-81 | 构造 player、恢复 checkpoint、reset。 |

### Environment 与输出，行 84-148

| 行 | 解释 |
|---|---|
| 84-93 | terrain label 解成 lambda/amplitude；强制 homogeneous；写 seed/fixed wave。 |
| 94-98 | 若请求时长正好等于/超过默认 episode，自动多留一个 control step，避免最后一步 timeout 被误报为 locomotion failure 或录到 reset 帧。 |
| 99-101 | 无论是否 video 都创建 camera-free env；这是隔离 Windows RTX race 的关键。 |
| 103-108 | 输出目录编码 mode、terrain 和 seed。相同目录重跑仍可能覆盖，所以正式实验应使用新 `--out`。 |
| 110-117 | closed-loop：选 explicit checkpoint 或 retained seed checkpoint，包装 env，reset observation，建立 player batch size。 |
| 118-121 | open-loop：不建 player，直接 reset raw env。 |
| 123-135 | 初始化 metric/time trace；video 额外初始化 world-pose trace。 |
| 137-148 | `capture_pose` 只记录 env0 的 251-link position/quaternion、alive、realized wave 和真实时间；按目标 fps 采样。 |

### Rollout、metric 与视频，行 150-260

| 行 | 解释 |
|---|---|
| 150-158 | 每 step inference；open-loop action tensor 为 0，但 env 因 `fixed_wave` bypass action law；closed-loop 用 deterministic policy mean。 |
| 159-174 | 统一 reward/done device；只为仍 alive 的 env 更新 final_x；记录首次 done；累加 trace；到达下一个 video 时间点时记录 pose。 |
| 176-178 | 总时长=`steps*12ms`；只对 full-duration survivor 计算净速度。 |
| 179-196 | 组装 metrics。明确写 `parallel_conditions_not_independent_replicates=true`。 |
| 197-204 | 写 human-readable JSON 和压缩 wave/reward trace。 |
| 205-229 | video 模式写 `all_link_world_poses.npz`，包含 geometry、terrain 与时间重建所需字段。 |
| 230-234 | 打印 JSON 并关闭 wrapper/env。 |
| 235-260 | 在 Kit fast shutdown 前调用 `render_offline.py`；audit 非零就 fail loudly。 |
| 263-267 | finally 关闭 Isaac app。 |

## 6. 离线视频：`scripts/render_offline.py`

| 行 | 解释 |
|---|---|
| 1-29 | 纯 CPU/第三方依赖、geometry 路径和固定配色；不 import Isaac Sim。 |
| 31-50 | 文本数值解析和 Isaac/MJCF `wxyz` quaternion 到旋转矩阵。 |
| 52-141 | 从保留的 MJCF+OBJ 重建每个 body 的 visual mesh、sphere/capsule/foot primitive；body name 映射，不按 index 猜。 |
| 143-165 | 把每帧记录的 link world pose 直接作用到 visual geometry；`__split` massless body 可无 visual，其余未知名 fail。 |
| 167-195 | VTK face 编码和地形裁剪；地形复用 clean runtime 的 generator 与 MuJoCo collision Z，保持 row/col 方向。 |
| 198-258 | color-agnostic frame audit：black/white/blank/partial-white/strobe/outlier/robot presence/motion。 |
| 260-359 | 解析 trace，验证 21 main body 与 251-body mapping，建立 45° chase，逐帧更新 mesh、编码 MP4 并写 provenance/audit JSON。 |
| 366-367 | audit pass 返回0，否则返回2。 |

### 速度与 return 为什么不完全相等

`episode_return` 是每 12ms 的 reward 和。reward 本质是逐步 centroid displacement/body length，因此无 reset/failure 时接近总净 body-length displacement。`survivor_speed` 还除以总 duration。两者差异来源包括 terminal step reward 被置零、done 后 wrapper auto-reset 行为、速度只统计从未 done 的 env，以及一个是位移、一个是位移/时间。

## 7. 高保真视频：`scripts/render_high_fidelity.py`

| 代码块 | 解释 |
|---|---|
| 常量与 CLI | 定位 Blender、packed `.blend`、105-entry skin map、Blender scene builder 和输入 `all_link_world_poses.npz`。 |
| `target_body_extents` | 从冻结 MJCF+21 OBJ 读取当前 body 外形尺度，只用来贴合皮肤，不改变 USD。 |
| `prepare_terrain` | 使用 clean terrain generator 和 collision-coordinate Z，裁剪轨迹附近的真实地形。 |
| `audit_video` | 检查精确帧数、黑/白/空/strobe、运动、有色皮肤，并与旧 MuJoCo 色相参考做定量比较。 |
| `--view-name` / `--camera-offset` | 为同一条物理 trace 选择可复现机位；默认文件名保持兼容，其他机位使用独立后缀，避免相互覆盖。 |
| `main` | 写临时 runtime/terrain，后台调用 Blender，随后用 libx264 编码并写 provenance-bound audit。 |

`scripts/blender_high_fidelity_scene.py` 只在 Blender 内执行：保留原 UV/材质/packed images，隐藏全部 `col_*`，把 105 个 visual mesh 绑定到 97 个当前刚体，逐帧应用 Isaac 记录的 `wxyz` 世界位姿。21 个 body shell 先消除旧 rig 最大 9.920 mm 的 mesh-center 偏移，并使用 `[1.08, 1.04, 1.04]` 的展示尺度余量。源头壳前端轴线原为 `-5.838°`；渲染器以其后缘中心为支点，将头壳、颚肢、口器和触须根部整体旋转 `+16.5°`，得到 `+8.094°` 可见抬头，随后把触须最终方向保持在外展 18°、上扬 22°，不重复叠加角度。头后第一节只在视觉层把 world-Z 对齐到相邻两壳平均中心线，轨迹 X/Y、姿态与物理状态不变。76 个腿段按真实关节端点定长，每端延长 0.50 mm、直径 1.05 mm，使膝部连续。末节只在轴向按 `[0.67, 1.00, 1.00]` 缩短并向头侧平移 2.107 mm，保留相邻体节的宽高连续性；成对终足与末节共享同一尺度，并额外向头侧嵌入 1.10 mm，不再使用会把两根终足挤成色块的独立各向异性缩放。几何审计会拒绝非零 body-center residual、leg-span drift、头部组件缺失、头壳抬升不在 `8.0±1.0°`、支点漂移、触须方向错误或终足根部嵌入不足 2.0 mm。所有修正都只作用于视觉 mesh，不新增仿真关节，不改变碰撞或 metrics。

## 8. Sweep 入口：`scripts/open_loop_sweep.py`

| 行 | 解释 |
|---|---|
| 1-8 | 用独立子进程运行每个 cell，避免 persistent Isaac 状态泄漏。 |
| 11-18 | 默认 6 terrain、2 terrain seed、13 wave、输出目录和可选 video。 |
| 21-24 | 三层循环，cell key=(terrain, terrain_seed, wave)。 |
| 25-32 | 调当前 Python 的 `evaluate.py`，传固定 wave、输出和 headless。 |
| 33-34 | `--video` 会让每一个 cell 都录视频，成本很高；仅小 panel 使用。 |
| 35 | 每 cell 启动前打印进度。 |
| 36 | `check=True`：任何 cell 非零退出立即停止，不悄悄继续产生残缺 panel。 |
| 39-40 | CLI 入口。 |

默认运行 156 boots。首次只跑小 panel，命令见 master runbook。

## 9. Integrity checker：`scripts/check_project.py`

| 行 | 解释 |
|---|---|
| 1-15 | 定义 root 和 `--quick`；quick 只跳过大 terrain rebuild，不跳过 file/checkpoint contract。 |
| 18-23 | 流式 SHA-256，避免把 checkpoint 整个读进额外 buffer。 |
| 26-28 | `require` 统一 fail-loudly。 |
| 31-37 | 读 `SOURCE_LOCK.json`，验证每个 locked release file 存在且 hash 一致。lock 自身不 hash 自己。 |
| 39-41 | 拒绝 archive/quarantine/attempt/recovery/guardian 等历史运行目录回流。 |
| 42-47 | 物理资产必须刚好 5 USD；高模目录必须只有一份 hash-locked `.blend`，其 105 visual/97 target 映射、四类 topology、视觉隔离和颜色契约必须吻合；checkpoint 必须刚好三份指定文件。 |
| 49-55 | import Torch/gait，验证 13 knots 和 action endpoints。 |
| 56-61 | `weights_only=True` 安全加载三 checkpoint，要求 epoch24、34D input、1D output。 |
| 63-78 | 非 quick 时重建 tri source/collision，包含 dtype+shape+bytes 的 hash 必须匹配 frozen contract。 |
| 79 | 唯一成功标志。 |
| 82-83 | CLI 入口。 |

## 10. Environment file：`environment.yml`

| 行 | 解释 |
|---|---|
| 1-8 | 最小 Conda 层：Python、NumPy、SciPy、PyYAML、pytest。 |
| 10-11 | Isaac Sim、Isaac Lab、Torch、isaaclab_rl、RL-Games 是外部 GPU framework，按注明版本另装，不 vendor 进 6MB 项目。 |

这个文件不是完整的 NVIDIA/Isaac 一键安装器。原因是 Isaac Sim 安装与驱动/CUDA/Omniverse 环境相关，把整套 framework 复制进项目反而破坏 clean boundary。当前机器先使用 `isaaclab_50`；新机器严格安装 pinned versions。

## 11. Pytest import config：`pyproject.toml`

| 行 | 解释 |
|---|---|
| 1 | 打开 pytest 专用配置表。 |
| 2 | 把 clean project root 加入 test import path，使 `pytest -q` 和 `python -m pytest -q` 都能 import `centipede`。这只影响测试收集，不改变仿真 runtime import。 |

## 12. Binary checkpoints

| Seed | SHA-256 | 接口 |
|---|---|---|
| 141421 | `1cfd2508c60a5c215206f183a1070b8073ef9a218fdb5fbe8b3f298f3970183d` | epoch24, 34D -> 1D |
| 161803 | `427e58e1fae4353ac244104f3d6165c9671f147e046e7ad2d293309c96974297` | epoch24, 34D -> 1D |
| 173205 | `b0358fb6336540d8c2b0dc168b9a2aee67cc368cb34e3f5cdd2185521037e8de` | epoch24, 34D -> 1D |

checkpoint 是 binary state dict，不做文本逐行注释。`check_project.py` 验证关键 tensor shape，`SOURCE_LOCK.json` 验证完整 bytes。
