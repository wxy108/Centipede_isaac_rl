# ChatGPT Review Packet — Phase 1 sweep repair complete  (2026-05-11)

> Composed manually because `scripts/rl/make_chatgpt_review_packet.ps1`
> is blocked by the local PowerShell execution policy. Same schema as
> the auto-generated packet. Previous packet preserved at
> `docs/review_packets/phase12_2026-05-10_packet.md`.
>
> Paste this entire file into ChatGPT for external architectural review,
> per `docs/CHATGPT_SUPERVISION_PROTOCOL.md`.

**1. Timestamp:**     2026-05-11 00:20 EDT
**2. Project root:**  C:\Users\wxy22\Documents\Centipede_IsaacLab_RL
**3. Current branch:** main

---

## 4. `git status --short`

```
 M assets/config.yaml
 M controllers/impedance_controller_torch.py
 M envs/centipede_env.py
 M envs/centipede_env_cfg.py
 M scripts/optimization/arena.py
?? .claude/
?? CLAUDE.md
?? IsaacLab/
?? assets/centipede.usd.legacy_abs_path_backup
?? assets/config.yaml.legacy_abs_path_backup
?? configs/optimizer_archive/
?? controllers/impedance_controller_torch.py.before_global_6d_2026-05-07
?? docs/                                ← NEW Phase 1 docs added below
?? envs/centipede_env.py.before_global_6d_2026-05-07
?? envs/centipede_env.py.before_pass12_2026-05-09
?? envs/centipede_env.py.before_pass12_relaxed_2026-05-09
?? envs/centipede_env_cfg.py.before_global_6d_2026-05-07
?? envs/centipede_env_cfg.py.before_pass12_2026-05-09
?? envs/centipede_env_cfg.py.before_pass12_relaxed_2026-05-09
?? envs/centipede_env_cfg.py.before_step2_2026-05-07
?? scripts/render_centipede_demo.py
?? scripts/rl/
?? scripts/sanity/                       ← NEW Phase 1 sanity scripts
?? tasks/
?? outputs/                              ← NEW Phase 1 analyses
```

## 5. `git diff --stat`

```
 assets/config.yaml                        |   10 +-
 controllers/impedance_controller_torch.py |  471 +++++++++-
 envs/centipede_env.py                     | 1415 ++++++++++++++++++++++++++++-
 envs/centipede_env_cfg.py                 |  341 ++++++-
 scripts/optimization/arena.py             |  171 +++-
 5 files changed, 2336 insertions(+), 72 deletions(-)
```

**No protected-file diff added by Phase 1.** The five modified files
above are unchanged since the Phase-12 packet (same SHAs, same line
counts). Phase 1 work is entirely in new (untracked) files under
`docs/`, `scripts/sanity/`, and `outputs/analysis/`.

## 6. SHA256 + LastWriteTime fingerprints (Phase 1 file set)

| file | SHA256 (16 chars) | bytes | LastWriteTime |
|---|---|---:|---|
| `envs\centipede_env.py` | `FA3320B0CE96E58A` | 81 771 | 2026-05-09 20:12:42 (UNCHANGED since Phase 12) |
| `envs\centipede_env_cfg.py` | `10EB9E0A471D3FCE` | 21 823 | 2026-05-09 20:12:02 (UNCHANGED) |
| `controllers\impedance_controller_torch.py` | `A14EF7FFE60CE9A0` | 45 760 | 2026-05-07 04:16:04 (UNCHANGED) |
| `tasks\agents\rl_games_stage_b_smoke_cfg.yaml` | `580AD395B6E31EE7` | 6 820 | 2026-05-09 13:21:17 (UNCHANGED) |
| `scripts\sanity\sweep_global_parameters.py` | `0027F5BA7AA023E9` | 17 562 | 2026-05-10 16:00:33 (Phase-13 sweep runner; UNCHANGED) |
| `scripts\sanity\sweep_repair_focused.py` | `2B67715DBB57D4D6` | 13 123 | 2026-05-10 23:00:13 (NEW Phase 1) |
| `scripts\sanity\analyze_sweep_fixed_freq.py` | `CE2C4E8D246F3E06` | 23 282 | 2026-05-10 22:30:14 (NEW Phase 1) |
| `scripts\sanity\calibrate_roughness_sensor.py` | `EACADDEEFC55E490` | 21 594 | 2026-05-10 22:56:50 (NEW Phase 1) |
| `scripts\sanity\merge_sweep_csvs.py` | `826BC8E4D08E23D7` | 2 039 | 2026-05-10 23:00:25 (NEW Phase 1) |
| `scripts\sanity\run_phase1_postsweep_pipeline.py` | `F48D81DEB5A223BB` | 2 629 | 2026-05-10 23:02:50 (NEW Phase 1) |
| `outputs\analysis\global_parameter_sweep_stageA.csv` | `BDA205F51C810BD5` | 100 467 | 2026-05-10 16:49:22 (Phase-13 source, preserved) |
| `outputs\analysis\global_parameter_sweep_repair_2026-05-10.csv` | `5C0FF68BCC7869E9` | 168 373 | 2026-05-11 00:14:16 (NEW — repair output) |
| `outputs\analysis\global_parameter_sweep_stageA_repaired.csv` | `A6EC341151F21EDC` | 268 605 | 2026-05-11 00:15:20 (NEW — merged) |
| `outputs\analysis\PHASE1_SWEEP_REPAIR_SUMMARY.md` | `55DE4C48CD8A50FD` | 14 368 | 2026-05-11 00:18:08 (NEW — synthesis) |
| `outputs\analysis\global_parameter_sweep_coverage_matrix_repaired.md` | `7FB6ED229B37399E` | 3 396 | 2026-05-11 00:15:20 (NEW) |
| `outputs\analysis\global_parameter_sweep_fixed_freq_analysis_repaired.md` | `289626BF28B33523` | 16 363 | 2026-05-11 00:15:20 (NEW) |
| `outputs\analysis\roughness_sensor_calibration_repaired.md` | `AA2062DC95C39A3D` | 13 383 | 2026-05-11 00:15:21 (NEW) |
| `docs\SCRIPTED_ADAPTIVE_CONTROLLER_DESIGN.md` | `E7CCE9B82871F142` | 14 183 | 2026-05-10 22:32:40 (NEW) |
| `docs\PHASE_1_NEXT_STEPS_AND_NON_ACTIONS.md` | `DD0725E5A6005959` | 7 141 | 2026-05-10 22:33:44 (NEW) |
| `docs\PHASE_1_SWEEP_REPAIR_PLAN.md` | `8C58EEFDB59CF87C` | 5 459 | 2026-05-10 22:58:53 (NEW) |

## 7. Pass-status reference (unchanged from Phase 12)

| Pass | Status |
|---|---|
| Pass 8 / 9 | Validated reference; checkpoint at `outputs\rl_smoke\stage_b_transition_global6d\2026-05-08_15-27-26\nn\` (UNCHANGED) |
| Pass 10 | Failed ablation; flag default False (UNCHANGED) |
| Pass 11 | Windowed-CoT branch (UNCHANGED) |
| Pass 12 B4 RELAXED + PPO-E | promoted to `outputs\rl_smoke\promoted\pass12_B4_pilot_ep15_BEST_safe_candidate.pth` (UNCHANGED). **NOT used in Phase 1 work.** |
| **Phase 1 (NEW)** | Scripted-adaptive-controller workstream. No PPO. No reward changes. No action-bound changes. No env / controller code changes. Only sweep + analysis + design docs. |

---

## 8. What changed since the Phase-12 packet

Nothing in `envs/`, `controllers/`, `assets/`, `tasks/agents/`, or any
checkpoint directory. All Phase 1 changes are NEW files (additive,
untracked).

The Phase-12 packet's "proposed next command" was *PROMOTE pilot ep_15
BEST + ADOPT variant E PPO config*. After the user's directive on
2026-05-10 (re-prioritize from RL/PPO to scripted adaptive control
with `freq` pinned), Phase 1 was executed as a pure analysis +
design phase. The Phase-12 promotion remains in the `promoted/`
folder, not undone, but is no longer the operating point for
Phase 1.

## 9. Phase 1 work executed (all read-only or sanity-script category)

1. **Fixed-freq=1.0 re-analysis** of original 384-row sweep
   (`scripts/sanity/analyze_sweep_fixed_freq.py`).
   - Output: `outputs/analysis/global_parameter_sweep_fixed_freq_analysis.md`,
     `..._fixed_freq_per_value.csv`, `..._coverage_matrix.md`.
   - Finding: with freq held at 1.0, body_amp ↓ / leg_yaw ↓ / wave_num ↑
     trends emerged; coverage gaps flagged (11 cells n=0, 59 n∈{1,2}).
2. **Roughness-sensor calibration** from existing per-env aggregates
   (`scripts/sanity/calibrate_roughness_sensor.py`).
   - Output: `outputs/analysis/roughness_sensor_calibration.md`.
   - Tested 4 candidate `R_raw` formulas; C3 (vz + force-spike + contact-loss)
     won at 68.8 % 3-band accuracy with cuts (0.257, 0.380).
3. **Focused sweep repair** to fill low-coverage cells
   (`scripts/sanity/sweep_repair_focused.py`).
   - 20 non-freq settings × 16 envs × 2 repeats = 640 rows in one
     IsaacLab boot. ~70 min wallclock. Freq pinned at 1.0
     throughout.
   - Used canonical `controller.apply_global_6d_modulation(...)`
     API; same monkey-patch pattern as the Phase-13 sweep runner.
   - **Did not** modify `envs/` / `controllers/` / `assets/` /
     `tasks/agents/` / reward / action bounds.
4. **Re-merge + regenerate analyses** on the repaired CSV.
   - 1024 rows total. 126 cells now n ≥ 3 (was 74).
   - Calibration C3/C4 both 66.6 % (slight drop with more data);
     C4 selected for better rough recall (83 %).

## 10. Headline scientific findings (READ THE SUMMARY DOC, this is just a teaser)

Full details: `outputs/analysis/PHASE1_SWEEP_REPAIR_SUMMARY.md`.

Key reversals vs the pre-repair view:

- **`body_amp` is no longer monotone↓.** Once `baseline × rough_b`
  exists (n=5, v_fwd 19.8 mm/s), `body_amp = 1.30` wins on rough_b
  at v_fwd 23.5 mm/s. Pre-repair "body_amp = 0.20 wins on rough_b"
  was an n=0-baseline artifact.
- **No need to expand `_G6_BODY_AMP_LO`** — the Phase-13 argument
  for action-bound expansion does not survive.
- `leg_yaw_amp` and `wave_num` monotonic trends preserved (each only
  flips on rough_b).
- `wave_num > 2.0` confirmed harmful on every block.
- `leg_pitch_amp` confirmed safest at 1.0.
- `body_imp` confirmed too noisy to script per-band.

Revised scripted-controller target map (replacing
`docs/SCRIPTED_ADAPTIVE_CONTROLLER_DESIGN.md` §6):

| param | smooth | mid | rough |
|---|---:|---:|---:|
| body_amp_scale | 1.20 | 1.00 | **1.20** ← was 0.55, sign flip |
| leg_yaw_amp_scale | 1.00 | 1.00 | 0.55 |
| leg_pitch_amp_scale | 1.00 | 1.00 | **1.00** ← was 0.85 |
| wave_number_scale | 1.00 | 1.00 | **1.50** ← was 1.80, conservative |
| body_imp_scale | 1.00 | **1.00** | **1.00** ← was 1.20, drop per-band |
| freq_scale | 1.00 | 1.00 | 1.00 (PINNED) |

Roughness-sensor cuts (C4): **c_low = 0.286, c_high = 0.365**.
EMA τ_R = 1.5 s, hysteresis h = 0.07. Mid → rough Cohen's d = +1.15 (large).

## 11. Proposed next command

Implement the read-only scripted-adaptive-controller runner +
analyzer per the (revised) design:

```
# 1. Implement
scripts/sanity/run_scripted_adaptive_controller.py
scripts/sanity/analyze_scripted_adaptive_results.py

# 2. Run (in IsaacLab, same boot pattern as the sweep)
.\IsaacLab\isaaclab.bat -p scripts\sanity\run_scripted_adaptive_controller.py `
    --num-envs 16 --steps-rollout-phys 1500 --base-seed 142 `
    --rraw-formula C4 --c-low 0.286 --c-high 0.365 `
    --tau-r 1.5 --tau-u 0.6 --hysteresis 0.07 `
    --target-map outputs/analysis/PHASE1_SWEEP_REPAIR_SUMMARY.md `
    --out-csv outputs/analysis/scripted_adaptive_eval_2026-05-11.csv

# 3. Analyze (head-to-head vs baseline + per-band winners)
python scripts/sanity/analyze_scripted_adaptive_results.py `
    --in-csv outputs/analysis/scripted_adaptive_eval_2026-05-11.csv `
    --baseline-rows-from outputs/analysis/global_parameter_sweep_stageA_repaired.csv `
    --out-md outputs/analysis/scripted_adaptive_eval_2026-05-11.md
```

Constraints applied:

- Runner monkey-patches `_apply_global_6d_action`; **does NOT
  modify** `envs/`, `controllers/`, `assets/`, `tasks/agents/`,
  reward constants, or action bounds.
- Target params clamped inside existing `_G6_*` bounds
  (no expansion needed).
- `freq_scale` asserted == 1.0 every step.
- Output low-pass τ_u = 0.6 s; sensor EMA τ_R = 1.5 s.
- Promotion criterion: scripted controller ≥ all-1.0 baseline on
  every block AND ≥ +20 % v_fwd on rough_a AND rough_b. **No
  promotion** if it can't beat baseline (it would mean the band
  map is wrong, not that we should escalate to RL).

## 12. Question for ChatGPT

**Should we CONTINUE / STOP / ROLLBACK / AUDIT-MORE / REDESIGN?**

Specific decisions requested:

A. **Adopt the revised target map** in `PHASE1_SWEEP_REPAIR_SUMMARY.md`
   §"Revised target parameter map"? (Replaces `SCRIPTED_ADAPTIVE_CONTROLLER_DESIGN.md` §6.)

B. **Adopt the C4 R_raw formula + cuts (0.286, 0.365)**? Or prefer
   C3 with (0.220, 0.368)? (Both 66.6 % accuracy; trade-off is C4
   has better smooth/rough recall, C3 has better mid recall and
   larger mid→rough Cohen's d.)

C. **Approve implementation** of `run_scripted_adaptive_controller.py`
   and `analyze_scripted_adaptive_results.py` per §11? (Sanity scripts;
   no env/controller/reward changes.)

D. **Revert action-bound expansion plan** explicitly in
   `docs/PHASE_1_NEXT_STEPS_AND_NON_ACTIONS.md`? (Phase 1 data shows
   it's unjustified.)

E. **No further sweep repair** before §C? Phase 1 left only 5 non-freq
   cells at n < 3, none of which affect the scripted-controller
   decision.

F. **Defer reward-calibration discussion** until after the scripted
   controller has produced trustworthy metrics? (Per the user's brief.)

G. Anything Phase 1 missed?

The most actionable answer is whatever you have for A–C. D–F are
follow-up directives. G is your call.

## 13. What this packet does NOT request

- Does not request reward changes.
- Does not request PPO training.
- Does not request action-bound expansion.
- Does not request env / controller / asset / task code changes.
- Does not request checkpoint promotion or deletion.
- Does not adopt any specific (R_raw, c_low, c_high) tuple — only
  asks ChatGPT to evaluate the recommendation.

---

## 14. Reference: the existing protocol (unchanged)

Per `docs/CHATGPT_SUPERVISION_PROTOCOL.md`:

- Allowed without packet: read-only post-processing of CSVs, new
  files under `docs/` and `outputs/analysis/`, sanity scripts under
  `scripts/sanity/` that monkey-patch at runtime.
- Requires packet: anything in `envs/`, `controllers/`, `assets/`,
  `tasks/agents/`; any reward / action-bound / PPO change; any
  PPO run.

Phase 1 stayed entirely on the "allowed" side. The §11 proposed
next command is on the EDGE — implementing a new runner that
monkey-patches the env at runtime is in the same family as the
existing `sweep_global_parameters.py` and `sweep_repair_focused.py`
(both approved). I'm asking ChatGPT to confirm that classification.
