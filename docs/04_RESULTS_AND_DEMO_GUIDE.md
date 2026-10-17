# Results and Demo Guide

## 1. Retained evidence inventory

| 路径 | 用途 |
|---|---|
| `evidence/P13_OPEN_LOOP_FINAL_REPORT.md` | open-loop 预注册判决和解释边界，第一阅读入口 |
| `evidence/data/p13_open_loop_long_form_quiescent.csv` | per-env 原始审计表；用于重分析，不作为32独立replicates |
| `evidence/data/p13_cell_results.json` | 每 cell 曲线、robust choice、bootstrap draws |
| `evidence/data/p13_trend_tests.json` | 预注册 trend test 输出 |
| `evidence/data/observation_classifier_audit.json` | 34D/31D terrain-information classifier 完整 audit |
| `evidence/figures/*.png` | 已筛选的最终科研图 |

clean release 没有保留任何历史 demo 视频。旧视频包含多代模型、camera、head pitch、spawn 和参数 contract，混放会使 PI 无法判断视频对应哪套系统。交付目录只保留由本 clean runtime 重新生成并完整审计的一组三条件 panel：flat/wave1.5、L90/A8/wave2.0、L60/A12/wave3.0。PI 第一核验入口是各 run 的 `video/high_fidelity/high_fidelity_45deg.mp4`，统一索引见 `outputs/demo/TERRAIN_DEMO_PANEL_20260818.json`。

## 2. 六张 P13 图逐张读

### `p13_open_loop_speed_curves.png`

看每个 terrain cell 中 survivor-conditioned full-duration speed 随 wave 的变化。先确认该 wave 有 survivor，再比较 speed。曲线高不代表 survival 好；无 survivor 的点不应填成0速度。

### `p13_open_loop_survival_curves.png`

看完整 episode 的 survivor fraction。低 wave 在 rough terrain failure、而高 wave 保留 survival，本身是允许且有科学意义的 phenotype，不是必须“修复”的异常。

### `p13_open_loop_premeasurement_validity_curves.png`

识别 warmup/测量前 termination。P13 有31行 premeasurement-warmup-invalid，占1.345%。它们不能作为正常 locomotion score；也不能仅凭现有 artifact断言是 spawn penetration 还是初始动力学。

### `p13_robust_wave_preference_heatmap.png`

每个 `(lambda, amplitude)` cell 的 robust wave choice。它最适合找局部 trend 和非单调反例，不适合从视觉上事后挑一条最漂亮路径再宣称全局规律。

### `p13_near_optimal_interval_span.png`

展示多个 wave 是否性能近似，而不是强迫每个 terrain 只有一个精确 optimum。区间宽说明 operating point 不可精确识别或多个 wave 近似等价。

### `p13_preregistered_geometry_path.png`

唯一预先注册的跨地形 path：

| 顺序 | Terrain | Robust wave |
|---:|---|---:|
| 1 | Flat | 1.75 |
| 2 | L120/A4 | 2.0 |
| 3 | L90/A8 | 2.0 |
| 4 | L60/A12 | 3.0 |
| 5 | L45/A16 | 1.75 |
| 6 | L30/A16 | 3.75 |

endpoint 从1.75到3.75，shift=+2.0；但第5点回落到1.75，所以 path 不单调。这正是最终 verdict 只能是 conditional 的直接原因。

## 3. P13 判决拆解

| Test | 观察 | 预注册要求 | Pass |
|---|---:|---:|---|
| path nondecreasing | False | True | No |
| endpoint shift | +2.0 | >=+0.25 | Yes |
| fixed-lambda positive | 3/5 | >=4/5 | No |
| fixed-amplitude positive | 2/4 | >=3/4 | No |
| pairwise nonnegative dominance | 0.669 | >=0.75 | No |
| endpoint-positive bootstrap | 0.806 | >=0.95 | No |

因此规范表述是：

> 在冻结 controller/gait calibration 下观察到若干局部 rougher-to-higher wave 偏好和显著 endpoint shift，但预注册全局稳健性条件未满足；结果依赖 terrain path 和 calibration。

不要表述为“假说完全失败”，也不要表述为“已经证明 rougher terrain 必然选择更高 wave”。

## 4. Observation classifier 图怎么读

`observation_classifier_evidence.png` 应与 `observation_classifier_audit.json` 一起看。

Primary gate 在1.44s、31D no-wave/no-phase features、roughness 3-class、seed+wave double holdout 下 balanced accuracy 0.9444。phase-only control 0.3333。

图的用途是回答：身体响应中是否存在 policy representation 有可能读取的 terrain-related information。它不回答：PPO 是否已经学会使用、policy 是否能在任意新 terrain 上估计 roughness、roughness 是否一维。

## 5. 推荐 demo matrix

Flat/wave1.5、L90/A8/wave2.0 和 L60/A12/wave3.0 已作为 canonical panel 保留，无需重复生成。需要扩展比较时再生成表中其余条件；每个只用1 env、9.6s。这里的 `--video` 是无 RTX 的内部核验视频：由真实 251-link 世界位姿离线渲染，无标签、无黑边：

| Demo | 目的 | 命令关键参数 |
|---|---|---|
| Flat, wave1.5 | 低波平地表现 | `--terrain flat --fixed-wave 1.5` |
| Flat, wave3.5 | 高波平地对照 | `--terrain flat --fixed-wave 3.5` |
| L90/A8, wave2.0 | mild rough operating point | `--terrain l90a8 --fixed-wave 2.0` |
| L60/A12, wave1.5 | rough 低波 failure/slow phenotype | `--terrain l60a12 --fixed-wave 1.5` |
| L60/A12, wave3.0 | preregistered robust choice | `--terrain l60a12 --fixed-wave 3.0` |
| L60/A12, policy161803 | closed-loop行为检查，不作最终trend结论 | `--terrain l60a12 --policy-seed 161803` |

逐条运行：

```powershell
python scripts\evaluate.py --terrain flat --fixed-wave 1.5 --num-envs 1 --steps 800 --video --headless --out outputs\pi_demo
python scripts\evaluate.py --terrain flat --fixed-wave 3.5 --num-envs 1 --steps 800 --video --headless --out outputs\pi_demo
python scripts\evaluate.py --terrain l90a8 --fixed-wave 2.0 --num-envs 1 --steps 800 --video --headless --out outputs\pi_demo
python scripts\evaluate.py --terrain l60a12 --fixed-wave 1.5 --num-envs 1 --steps 800 --video --headless --out outputs\pi_demo
python scripts\evaluate.py --terrain l60a12 --fixed-wave 3.0 --num-envs 1 --steps 800 --video --headless --out outputs\pi_demo
python scripts\evaluate.py --terrain l60a12 --policy-seed 161803 --num-envs 1 --steps 800 --video --headless --out outputs\pi_demo
```

每条命令只有在 `video\offline_45deg.audit.json` 的 `audit.pass=true` 时才算完成。正式组会 Omniverse 视频不能直接复用这条命令；其单条件单进程、Vulkan、显式相机、早停、重试与逐帧审计规则见 `05_VIDEO_RENDERING_PLAYBOOK.md`。

## 6. 每个 demo 按这个 checklist 核验

### 启动 0-1.5s

- spawn 时腹部/腿是否已经穿地。
- robot 是否在第一帧就侧翻；若是，不能标为 locomotion buckle。
- 1s settle 内不应有 full swing。
- 1.0-1.25s staggered ramp 后 body/leg amplitude 应逐步释放。
- head world pitch 是否合理；joint target -0.57 rad 不等于世界角。
- 高保真皮肤的 `+8.094°` 头壳角只是展示装配角；不能拿它替代 trace 中的 head world pitch 或 collision pose。

### 中段 1.5-8s

- canonical forward 是否朝 world -x。
- 是否持续前进，而不是原地大幅 roll/yaw。
- 腹部是否长期拖地。
- 左右腿是否在 top projection 严重交叉。
- rough 中低 wave failure 是不是运动中逐渐发生，而非初始 penetration。

### 末段 8-9.6s

- 是否完整 survival。
- 是否因为离开有效 terrain 区而产生边界效应。
- policy demo 的 realized wave 是否仍变化，还是收敛在近常数。

## 7. 快速看 metrics

例如查看一个 demo：

```powershell
$m = Get-Content outputs\pi_demo\l60a12\terrain_seed_244949\open_loop_w3\metrics.json -Raw | ConvertFrom-Json
$m | Select-Object terrain,fixed_wave,num_envs,survivors,survival_rate,survivor_speed_median_bl_cycle
```

查看 RL demo：

```powershell
$m = Get-Content outputs\pi_demo\l60a12\terrain_seed_244949\policy_seed_161803\metrics.json -Raw | ConvertFrom-Json
$m | Select-Object terrain,policy_seed,survivors,survival_rate,survivor_speed_median_bl_cycle
```

`survivor_speed_bl_cycle` 实际计算为 body-length/second。因为当前 temporal CPG frequency 恰好1Hz，所以数值上也等于 body-length/cycle。若未来 frequency 不再是1Hz，字段名必须修正或显式乘 cycle period。

## 8. 查看 realized wave 数值

不画图也可以先检查范围：

```powershell
python -c "import numpy as np; p=r'outputs\pi_demo\l60a12\terrain_seed_244949\policy_seed_161803\traces.npz'; w=np.load(p)['wave']; print('shape',w.shape,'min',w.min(),'median',np.median(w),'max',w.max(),'start',w[0,0],'end',w[-1,0])"
```

如果 raw action 看似到边界而 realized wave 没到，先回看 command tau/rate limit；不要立即归因于 policy 学不到。

## 9. Demo 与科研结果不要混为一谈

- 1-env video 是 mechanism/qualitative evidence，不是统计重复。
- 一个漂亮视频不能替代 terrain-seed/policy-seed matrix。
- 一个失败视频应标明是 spawn、warmup、运动中 buckle、离开 terrain 还是数值异常。
- 当前 clean demo 用于确认 runtime 与现象；closed-loop rougher-to-higher 的最终统计仍未完成。

## 10. 从这里继续科研的最小步骤

1. 完成 clean runtime 的1-env smoke。
2. 生成上面6个 contract-consistent videos，逐个核验 spawn 与运动 failure。
3. 对3个 retained policy，各自评估 flat/L90A8/L60A12，至少两个 terrain seeds，分别报告 survival、conditional speed、realized-wave distribution。
4. 做 observation ablation：full34、去contact、去spatial、去root；保持训练预算和 seeds 一致。
5. 只有闭环 matrix 完整后，才判断 policy 是否真正随 terrain response 调整 wave。
