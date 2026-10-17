# Visual rendering close-up chase pipeline (validated 2026-05-10)

This is the **canonical** way to produce visually-judgable centipede gait videos
for this project. The earlier far-oblique videos are no longer sufficient because
the centipede silhouette was too small to evaluate gait quality.

## Why this pipeline exists

- Default arena: 2.2 m wide; centipede body length ≈ 5 cm.
- Far oblique camera (eye 1.9 m, target origin) makes the centipede a ~30-pixel
  smudge on a 1280×720 frame. Body wave / leg motion / contact behavior cannot
  be judged from these videos.
- num_envs=16 group view also fails inside this renderer because the rl_games
  `vecenv.register` path interferes with the replicator camera annotator at
  multi-env scale (all-black frames; documented failure).
- A single-robot close-up chase camera + rough_b spawn enforcement does work
  reliably at num_envs=1 and produces visually-judgable gait videos.

## Hardware / software footprint

- Renderer: `scripts\sanity\render_policy_visual_audit.py` (this project, this
  session).
- Frame audit: `scripts\sanity\frame_audit_closeup.py`.
- Spawn-seed helper: `scripts\sanity\find_spawn_seed.py`.
- Watchdog: `scripts\sanity\run_isaaclab_with_watchdog.ps1` (already includes
  `[viz] report ->` as a success marker for clean Omniverse-shutdown handling).

## Working settings (these are NOT defaults — keep them in sync with this doc)

| flag | value | why |
|---|---|---|
| `--num-envs` | **1** | single-robot view |
| `--seed` | 42 | episode randomness |
| `--spawn-seed 21` | rough_b deterministic spawn | found by `find_spawn_seed.py` |
| `--chase` | enabled | chase camera tracks env=0 root each frame |
| `--chase-offset -0.25 -0.20 0.18` | (m) | larger eye distance + height; smaller offsets caused **all-black frames on rough heightmap** (RTX raytrace did not converge with eye too close to bumpy mesh) |
| `--chase-alpha 0.85` | EMA smoothing | smooth camera even when robot turns |
| `--n-warmup 30` | renders before capture | smaller warmup left raytrace unconverged on rough_b |
| `--duration-sec 10` | 10 s = 200 policy steps @ 20 fps | covers multi-cycle gait |
| `--width 1280 --height 720` | 720p | reasonable filesize ~8 MB |
| `--video-fps 20` | matches policy step (0.048 s) | near-real-time playback |

## Reward flags (when rendering RL policies)

Match the policy's training flags. For Pass 12 B4 RELAXED policies:

```
--pass11-cot --pass12-cot-bounded --pass12-functional-guard
```

For zero-action baseline (open-loop default CPG): no Pass-N flags needed; use
`--zero-action`.

## Spawn-seed strategy

The transition arena has 6 terrain blocks (rough_a, rough_b, medium,
intermediate, long_wave, flat). For num_envs=1, the agent lands on ONE of 24
stratified cells, determined by `transition_spawn_seed`.

To force a specific block, run `scripts\sanity\find_spawn_seed.py`. As of the
2026-05-10 close-up audit:

| target block | smallest spawn-seed | label |
|---|---:|---|
| rough_a | 11 | most rough |
| **rough_b** | **21** | **rough, used in this audit** |
| medium | 1 | mild rough |
| intermediate | (any) | gentle rough |
| long_wave | 100 | very gentle |
| flat | 1234 | default flat |

## Exact commands used in the 2026-05-10 close-up audit

```powershell
# A — default baseline gait
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/sanity/run_isaaclab_with_watchdog.ps1 `
  -Cmd '.\IsaacLab\isaaclab.bat -p scripts\sanity\render_policy_visual_audit.py `
       --zero-action --policy-name baseline_zero_action `
       --camera oblique --chase --chase-offset -0.25 -0.20 0.18 --chase-alpha 0.85 `
       --duration-sec 10 --n-warmup 30 --spawn-seed 21 `
       --out outputs\visual_audit\pass12_B4_2026-05-10\single_follow_closeup\A_baseline_zero_action_chase_rough_b.mp4' `
  -LogFile 'outputs\analysis\overnight_session_2026-05-10\12_chase_A_rough_b.log' `
  -ExpectedDurationMin 6 -ExpectedReport '<output mp4>'

# C — Pass 12 B4 pilot ep_15 BEST
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/sanity/run_isaaclab_with_watchdog.ps1 `
  -Cmd '.\IsaacLab\isaaclab.bat -p scripts\sanity\render_policy_visual_audit.py `
       --checkpoint outputs\rl_smoke\promoted\pass12_B4_pilot_ep15_BEST_safe_candidate.pth `
       --policy-name pass12_B4_pilot_ep15_BEST `
       --camera oblique --chase --chase-offset -0.25 -0.20 0.18 --chase-alpha 0.85 `
       --duration-sec 10 --n-warmup 30 --spawn-seed 21 `
       --pass11-cot --pass12-cot-bounded --pass12-functional-guard `
       --out outputs\visual_audit\pass12_B4_2026-05-10\single_follow_closeup\C_pass12_B4_pilot_ep15_BEST_chase_rough_b.mp4' ...

# D — PPO-E val ep_55 BEST
... --checkpoint outputs\rl_smoke\stage_b_transition_global6d_pass12_B4_ppo_E_validation_overnight_2026-05-10\2026-05-10_04-18-21\nn\last_centipede_stage_b_transition_global6d_smoke_ep_55_rew_-15.021125.pth ...
```

## Frame audit (mandatory after every render batch)

```powershell
& C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe scripts\sanity\frame_audit_closeup.py
```

Writes:
- `<video_dir>\frame_audit.json` — per-policy stats
- `<video_dir>\frame_audit.md` — pass/fail summary
- `<video_dir>\closeup_snapshots.png` — N policies × M time-points grid

Pass criteria (all five must hold):
1. `no_black_frames` — 0 frames with mean<5 AND std<1
2. `no_white_frames` — 0 frames with mean>250 AND std<1
3. `no_frozen_pairs` — 0 consecutive bit-identical frames
4. `robot_visible_majority` — >50 % of frames have >0.1 % reddish pixels
   (`R > G+25 AND R > B+25 AND R > 60`); the centipede body is dark red,
   the terrain is light gray
5. `motion_present` — mean inter-frame absolute difference > 0.5

If any check fails, do NOT report the audit as complete; investigate (likely
chase offset too small / warmup too short / wrong spawn seed).

## Known failure modes and fixes

| symptom | cause | fix |
|---|---|---|
| all-black mp4 (file ~10 KB) | chase offset too close to rough heightmap; RTX raytrace did not converge | use `--chase-offset -0.25 -0.20 0.18` minimum, `--n-warmup 30` |
| centipede invisible at oblique view | camera too far for body scale | use chase camera (this pipeline) |
| black mp4 at num_envs=16 with this renderer | rl_games vecenv.register interferes with replicator camera | skip num_envs=16 with this renderer; either fix or use scripts\render_centipede_demo.py for n=16 |
| centipede walks off-frame fast | chase EMA too laggy | reduce `--chase-alpha` (e.g. 0.7); or increase `--chase-offset z` |
| `[viz] report ->` not detected by watchdog | watchdog success_markers list incomplete | already added in `scripts\sanity\run_isaaclab_with_watchdog.ps1` |
| `unexpected_exit` from watchdog despite success | bracket-class bug in PowerShell `-like` | already fixed: watchdog uses `.Contains()` |

## When NOT to use this pipeline

- For batch evaluation across many policies, use `audit_stage_b_latest_policy.py`
  (computes traces + metrics) — much faster, no rendering cost.
- For multi-policy comparison, render with this pipeline only the 2-3 policies
  worth visual judgment; use trace-based metrics for the rest.
- For evaluation across all 6 terrain blocks at once, use num_envs=16 stratified
  spawns in `audit_stage_b_latest_policy.py` (NOT this renderer).

## Compatibility / rollback

The renderer was extended this session with `--chase`, `--chase-offset`,
`--chase-alpha`, `--spawn-seed`. All flags are opt-in; without them the
renderer falls back to the previous static-camera behavior (which the
oblique videos used). To rollback: omit the flags.
