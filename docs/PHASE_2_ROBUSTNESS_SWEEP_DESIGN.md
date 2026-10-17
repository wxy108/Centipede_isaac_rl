# Phase 2 — robustness-focused sweep — design

> Drafted 2026-05-11. Status: ready to launch (auto-mode OK granted in
> the brief). No protected-file edits. No PPO. No reward / action-bound
> changes. Single IsaacLab session, ~2.5 h wallclock.

## 1. Scientific question

Does **higher body wave number and/or body impedance** improve
**robustness** across increasing terrain severity, even if it does
not maximize absolute forward speed on flat terrain?

This sweep is **not** trying to find the fastest gait. It is trying
to test whether some `(wave_num, body_imp)` choices show *flatter*
performance curves as severity increases — i.e. they trade peak
speed for stability.

## 2. Why a new sweep is needed

The Phase 1 repair (1024 rows) tightened the trends but used a
one-factor design with low n per cell. Phase 1 also changed the
question: it ranked single-axis winners. For robustness, we need a
**multi-seed two-factor (`wave_num × body_imp`) grid evaluated at a
sequence of controlled severity levels**.

## 3. Terrain severity definition (replaces ordinal block labels)

The existing transition arena is built from
`scripts/optimization/arena.py::generate_flat_to_rough_transition_heightmap`
with per-block `(wavelength λ_mm, amplitude amp_mm, seed)` triples
already exposed (defaults `(18, 18, 30, 50, 60)` mm wavelengths and
`(10, 10, 10, 8, 6)` mm amplitudes; one auto-appended flat block).

Define **physical severity** per block as the dimensionless ratio:

```
S = amp_mm / λ_mm    (≈ average single-wavelength surface slope)
```

S is the natural per-block scalar that controls how steep the
heightmap is at the dominant spatial frequency. The 6 existing
blocks map onto 5 distinct severity values:

| block_idx | label (legacy) | λ_mm | amp_mm | S = amp/λ |
|---:|---|---:|---:|---:|
| 0 | rough_a | 18 | 10 | **0.556** |
| 1 | rough_b | 18 | 10 | **0.556** (same physical S; independent random phase) |
| 2 | medium | 30 | 10 | **0.333** |
| 3 | intermediate | 50 | 8 | **0.160** |
| 4 | long_wave | 60 | 6 | **0.100** |
| 5 | flat | —  | 0  | **0.000** |

→ 5 controlled severity levels: **{0.000, 0.100, 0.160, 0.333, 0.556}**.

Blocks 0 and 1 are physically identical (same λ, same amp, different
seed). For robustness analysis they are pooled into the single
S=0.556 bin, doubling its sample size. So we have 5 severity levels
with extra coverage on the toughest end.

**The legacy labels (`rough_a`, `flat`, ...) are retained in the CSV
schema for compatibility but the primary x-axis on every Phase 2
plot is `S` (or `S × 1000` displayed as a percent for readability).**

This approach uses the existing terrain construction (no monkey-patch
of the heightmap generator, no edit to `envs/`) — it is purely a
re-interpretation of which physical severity each per-env rollout
landed in.

## 4. Parameter grid

**Primary variables** (the focus of the question):

- `wave_number_scale` ∈ {1.0, 1.5, 2.0, 2.5} (4 values).
  Capped at 2.5 because Phase 1 showed `wave_num > 2.0` is uniformly
  harmful — including 2.5 lets us see the harm gradient on this
  grid, but we don't probe deeper into the harmful zone.
- `body_imp_scale` ∈ {0.6, 1.0, 1.5, 2.0} (4 values).
  Skips the known-harmful 0.3 (Phase 1: −15 mm/s on smooth blocks);
  the [0.6, 2.0] range is the "non-harmful" plateau Phase 1
  identified.

→ 16 conditions per repeat.

**Fixed parameters** (controlled — not the story):

- `freq_scale = 1.0` — project directive (no high-freq exploit).
- `body_amp_scale = 1.0` — Phase 1 finding: stay near 1.0 on every
  block; not the operating axis here.
- `leg_yaw_amp_scale = 1.0` — Phase 1 weak evidence; not enough
  sampled values to script confidently.
- `leg_pitch_amp_scale = 1.0` — Phase 1 strong evidence: keep at 1.0.

These are asserted at every step in the runner; any deviation halts.

## 5. Seed strategy

Five seeds per condition, advanced via the spawn-block RNG drift
mechanism (the same mechanism the Phase 1 sweep_repair used). Each
"seed" is one full sweep over the 16 conditions; subsequent seeds
re-enter the RNG further along its sequence so spawn-block
distributions differ between seeds.

→ 5 seeds × 16 conditions = **80 rollouts total**.

Per rollout: 16 envs × 1500 physics steps (6 s sim, ~125 policy
steps at dt_pol=0.048 s). Per-env per-block aggregate metrics same
schema as the existing sweep CSV.

Per (severity × wave × imp) cell expected n_envs:
- 5 seeds × 16 envs / 5 severity bins (rough doubled) ≈ **16 envs per cell**

That is roughly 4–8× the per-cell n of the Phase 1 view, satisfying
the "multi-seed averages and error bars" requirement.

## 6. Wallclock estimate

Empirical from Phase 1 sweep_repair: **~105 s wallclock per 16-env
× 1500-phys-step rollout** (averaged across 40 rollouts in 70 min
runtime).

| component | seconds |
|---|---:|
| boot (cmd → sim_app ready) | ~90 |
| 80 rollouts × 105 s | 8 400 |
| post-loop CSV write | ~5 |
| **total** | **~8 500 s ≈ 142 min ≈ 2.4 h** |

Comfortably under the 4–5 h budget with ~1.5 h safety margin. If
something goes wrong on the first rollout we have time to iterate.

## 7. Output schema

CSV at `outputs/analysis/phase2_robustness_sweep_2026-05-11.csv`.
Same columns as `global_parameter_sweep_stageA.csv` plus four:

- `seed_repeat` (int 0–4)
- `wave_num_scale` (float, 1.0–2.5)
- `body_imp_scale` (float, 0.6–2.0)
- `severity_S` (float, 0.0–0.556) — derived from block's (λ, amp)
- `block_lambda_mm`, `block_amp_mm` — physical block parameters

Existing columns kept: `setting_id`, `axis`, the 6 controller scales,
`env`, `block_label`, `n_steps_valid`, `duration_s`, `net_forward_m`,
`mean_v_fwd_mps`, `z_mean_m`, `z_p2p_m`, `vz_rms_mps`, `F_max_N`,
`F_p95_N`, `F_mean_N`, `n_contact_mean`, `dx_jump_p95_m`, `n_dones`.

`setting_id` will be encoded as `wN.NN_iN.NN_repK` (e.g. `w1.50_i1.00_rep2`).

## 8. Robustness metrics (computed in analysis stage, not in CSV)

For each `(wave_num, body_imp)` cell, aggregate across seeds and
across envs within each severity bin, then compute:

1. **Retention** R(S) = mean_v_fwd(S) / mean_v_fwd(S=0)
2. **Decline slope** dV/dS via linear regression of mean_v_fwd vs S.
3. **CoV across severity** σ(mean_v_fwd over S bins) / mean(mean_v_fwd over S bins)
4. **Worst-case** min over S of mean_v_fwd
5. **Convergence** = (max_wave_num v_fwd at S=0.556) − (min_wave_num v_fwd at S=0.556),
   compared to the same gap at S=0. Decreasing gap means wave_num
   matters less on rough terrain.

Plus per-cell:
- mean v_fwd ± std (across seeds)
- mean F_max, vz_rms, z_p2p, n_contact_mean
- locomotion_score (same formula as Phase 1)
- effort/m proxy: (mean_F_total × duration) / net_forward_m if
  trace lets us compute it, else `(F_mean × n_steps × dt) / net_forward_m`
  per env, averaged

## 9. Plots (saved under `outputs/analysis/figures/phase2/`)

1. `vfwd_vs_severity_per_wavenum.png` — 4 panels (one per body_imp);
   each panel: x = severity S, y = mean_v_fwd, one curve per wave_num,
   error bars (±1 std across seeds).
2. `retention_vs_severity_per_wavenum.png` — same layout, y =
   v_fwd / v_fwd(S=0).
3. `robustness_summary_by_wavenum.png` — 3 panels: decline slope,
   worst-case, CoV across S.
4. `heatmap_per_severity.png` — 5 subplots (one per S level), each
   a (wave_num × body_imp) heatmap of mean_v_fwd.
5. `heatmap_robustness.png` — 3 subplots, (wave_num × body_imp)
   heatmaps of decline slope, worst-case, CoV.
6. `tradeoff_vfwd_vs_fmax.png` — scatter; each marker = one cell
   (wave_num, body_imp); colored by severity.
7. `tradeoff_vfwd_vs_vz_rms.png` — same, vs vz_rms.
8. `convergence_gap_at_S0_vs_S_max.png` — bar chart showing the
   wave_num spread at each end of severity, per body_imp.

## 10. Runner script

`scripts/sanity/sweep_robustness_phase2.py` — sanity-script class
(monkey-patches `_apply_global_6d_action` exactly like the existing
sweep_global_parameters.py and sweep_repair_focused.py). Asserts
freq=1.0 and body_amp/leg_yaw/leg_pitch=1.0 every step. No env /
controller / asset / reward / action-bound / RL changes.

## 11. Strict non-actions (still in force)

- No edit to `envs/`, `controllers/`, `assets/`, `tasks/agents/`.
- No reward / action-bound / RL change.
- No PPO.
- No checkpoint move.
- **No** monkey-patch of the heightmap generator (we use the
  existing 6-block layout and reframe x-axis as physical severity).
- **No** scripted-adaptive-controller implementation yet.

## 12. Acceptance criteria for the analysis report

The downstream report
`outputs/analysis/PHASE2_ROBUSTNESS_SWEEP_REPORT.md` must directly
answer:

1. Does higher wave_num produce flatter performance curves across S?
2. Does higher wave_num reduce sensitivity (decline slope, CoV)?
3. Does higher wave_num improve robustness while costing speed?
4. Does body_imp interact with wave_num?
5. Is there an intermediate wave_num that balances both?
6. Are some wave_nums consistently harmful?
7. Are results stable across seeds, or dominated by single biased samples?
8. What should the future RL action space be?

The report must NOT propose RL or reward changes — only justify
the recommended action-space reduction for a future RL design.

## 13. Stop conditions

- Runner aborts if any rollout sets `freq_scale != 1.0` or any
  fixed parameter drifts from 1.0.
- Runner aborts on > 5 consecutive rollouts with v_fwd < 5 mm/s on
  S=0 — would indicate environment regression.
- Analysis report flags STOP if seed-to-seed std on any cell > 50 %
  of the cell mean — would mean the sample size is insufficient.

## 14. Files this run will create / change

Created (all sanity-script / read-only category):

- `scripts/sanity/sweep_robustness_phase2.py` (new runner)
- `scripts/sanity/analyze_robustness_phase2.py` (analysis + metrics)
- `scripts/sanity/plot_robustness_phase2.py` (figures)
- `outputs/analysis/phase2_robustness_sweep_2026-05-11.csv` (raw)
- `outputs/analysis/phase2_robustness_sweep_2026-05-11.log` (run log)
- `outputs/analysis/PHASE2_ROBUSTNESS_SWEEP_REPORT.md` (synthesis)
- `outputs/analysis/figures/phase2/*.png` (8 figures)
- `outputs/analysis/figures/phase2/PHASE2_FIGURE_INDEX.md`
- `docs/review_packets/latest_chatgpt_review_packet.md` (replaces
  Phase-1 packet; Phase-1 packet preserved at
  `docs/review_packets/phase1_2026-05-11_packet.md` already)

NO changes in `envs/`, `controllers/`, `assets/`, `tasks/agents/`,
or any checkpoint directory.
