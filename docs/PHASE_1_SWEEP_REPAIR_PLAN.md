# Phase 1 — focused sweep repair plan

> Drafted 2026-05-10. Companion to
> `docs/PHASE_1_NEXT_STEPS_AND_NON_ACTIONS.md` §2.1.
> Status: ready to launch. No env / controller / asset / reward /
> action-bound / RL changes.

## 1. Goal

Fill the n=0 / n=1 / n=2 cells in the original Stage A sweep
coverage matrix (`outputs/analysis/global_parameter_sweep_coverage_matrix.md`)
so that the fixed-freq=1.0 trends and the roughness-sensor
calibration rest on n ≥ 3 in every cell that actually drives a
Phase 1 conclusion.

Specifically we need:

- `baseline × rough_b` (n=0 → n ≥ 3) — cleanest "no-perturbation" reference for rough_b.
- `leg_pitch_amp_{0.30, 0.60} × rough_b` (n=0 / n=2 → n ≥ 3).
- `wave_num_{1.50, 3.50} × rough_b` (n=0 → n ≥ 1) and `wave_num_{2.0, 2.5} × rough_b` (n=1 → n ≥ 3).
- `wave_num_4.00 × flat` (n=0 → n ≥ 1) — corroborates the "high wave_num always hurts" finding.
- `body_amp_0.20 × long_wave` (n=0 → n ≥ 1) — the body_amp ↓ trend's left tail.
- `body_imp_{0.30 × medium}` (n=0 → n ≥ 1) and `body_imp × {rough_a, rough_b}` low-conf cells.
- `leg_yaw_amp_{0.30, 0.60} × {flat, long_wave}` LOW-CONF cells.

## 2. Mechanism

The original `scripts/sanity/sweep_global_parameters.py` runs each
setting once. The transition-spawn-block sampler RNG advances
with each `inner.reset()` so different settings hit different
spawn distributions, but with only one shot per (setting × seed)
we get 0–6 envs per cell.

Two ways to add coverage:

1. **Re-seed strategy.** Boot IsaacLab once, run the same
   (non-freq) settings list multiple times, re-seeding the env
   RNG between repeats so each repeat draws a fresh spawn
   distribution. Same setting_id across repeats → analyzers
   aggregate naturally.
2. **Many-seed strategy.** Run the script multiple times with
   different `--seed`. Boots IsaacLab N times (~60–90 s overhead
   each).

We use **(1)**: lower boot overhead, simpler merging.

## 3. Settings list — explicitly excludes freq

Per project directive, freq stays pinned at 1.0. We do **not**
re-run any `freq_*` setting in the repair. The original freq rows
(`freq_{0.6, 0.8, 1.2, 1.4}`) remain in the source CSV; the
repair augments only the non-freq rows.

20 settings to re-run per repeat:

| axis | values |
|---|---|
| `baseline` | (all 1.0) |
| `body_amp` | 0.00, 0.20, 0.40, 0.70, 1.30 |
| `leg_yaw_amp` | 0.30, 0.60 |
| `leg_pitch_amp` | 0.30, 0.60 |
| `wave_num` | 1.50, 2.00, 2.50, 3.00, 3.50, 4.00 |
| `body_imp` | 0.30, 0.60, 1.50, 2.00 |

Total = 1 + 5 + 2 + 2 + 6 + 4 = **20 settings**.

## 4. Repeat count

Two repeats per setting → 40 setting-runs total.
Each run ≈ 75 s rollout + ~3 s overhead. Plus one boot ≈ 60–90 s.
Estimated wallclock: **45–55 min**.

Two repeats roughly **doubles every cell's n_envs** in
expectation. The original baseline row is n=16 across 6 blocks
(0–6 per cell); after two repeats baseline is n=48 across 6 blocks
(0–~12 per cell). Probability that any specific (setting × block)
cell stays at n=0 after two repeats is small but non-zero.

If after the run any priority cell is still n < 3, a third repeat
of just that setting (single setting in a fresh boot) is the
escalation path — ~5 min each.

## 5. Output — non-destructive

Original kept untouched:
- `outputs/analysis/global_parameter_sweep_stageA.csv` (preserved)
- `outputs/analysis/global_parameter_sweep_stageA.md` (preserved)
- `outputs/analysis/global_parameter_sweep_stageA_analysis.md` (preserved)

New files:
- `outputs/analysis/global_parameter_sweep_repair_2026-05-10.csv`
  — repair-only rows (rows from this run).
- `outputs/analysis/global_parameter_sweep_stageA_repaired.csv`
  — concatenation of original CSV + repair CSV (same schema).
- `outputs/analysis/global_parameter_sweep_coverage_matrix_repaired.md`
- `outputs/analysis/global_parameter_sweep_fixed_freq_analysis_repaired.md`
- `outputs/analysis/global_parameter_sweep_fixed_freq_per_value_repaired.csv`
- `outputs/analysis/PHASE1_SWEEP_REPAIR_SUMMARY.md` — synthesis
  answering the 10 interpretation questions in the user's brief.

## 6. Watchdog

Launch via `scripts/sanity/run_isaaclab_with_watchdog.ps1`.
Success marker: `[sweep-repair] report -> done`. The watchdog
will auto-kill the python process when the marker prints, freeing
the GPU.

## 7. Failure modes and rollback

- **IsaacLab boot fails** (CUDA OOM, asset missing, etc.):
  zero rows produced. Rollback = nothing to roll back; original
  CSV untouched. Fix the boot issue and retry.
- **Mid-run OOM or seg-fault**: partial CSV written. Merge with
  caution; flag any setting with < N_REPEATS completed reps.
- **All cells fill but trends invert**: report it in
  `PHASE1_SWEEP_REPAIR_SUMMARY.md`; treat as a STOP signal and
  request ChatGPT review before any next step.
- **Specific cell stays n=0 after both repeats**: escalate (see §4).

## 8. Strict non-actions (still in force)

This run does **not**:
- modify `envs/`, `controllers/`, `assets/`, `tasks/agents/`,
- change reward constants,
- expand action bounds,
- launch any PPO or training,
- move / delete / promote any checkpoint,
- implement the scripted adaptive controller,
- adopt any `R_raw` formula or `(c_low, c_high)` cuts.

Only after this completes and the synthesis report is reviewed —
and a fresh ChatGPT review packet is approved — does the next
code-touching step (the scripted adaptive controller runner)
proceed.
