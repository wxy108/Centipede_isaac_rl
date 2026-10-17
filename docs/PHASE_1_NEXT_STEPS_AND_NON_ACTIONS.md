# Phase 1 — sweep repair + scripted adaptive controller — next steps

> Drafted 2026-05-10. Replaces the "Option C" recommendation in
> `outputs/analysis/GLOBAL_PARAMETER_SWEEP_AND_NEXT_STRATEGY_REPORT.md`
> per project re-direction. **Option C is cancelled.** No PPO, no
> reward edits, no action-bound expansion until Phase 1 completes.

## 1. Why we are here

The Phase-13 report recommended Option C (warm-start + Phase-8
reward + PPO-E). On review, this was premature:

- The headline finding `freq=1.40` dominates was driven by the
  freq-axis perturbations alone. The **fixed-freq=1.0 re-analysis**
  (`outputs/analysis/global_parameter_sweep_fixed_freq_analysis.md`)
  shows the other 5 axes contain interpretable monotonic structure
  that was masked by freq's outsized score.
- Coverage gaps make several "winners" rest on n=1 or n=2 spawns
  (see `global_parameter_sweep_coverage_matrix.md`). Acting on
  those without a focused re-sweep risks training to noise.
- We have not yet demonstrated that a *scripted* terrain-conditioned
  policy beats baseline. RL should be the *last* intervention, not
  the first — once the scripted controller exists, RL has a clean
  target.

## 2. Phase 1 goals (sequenced)

### 2.1 — sweep repair (this session, optional)

Re-run the sweep with focused settings to fill specific gaps. Same
mechanism as `scripts/sanity/sweep_global_parameters.py`; new
target list rather than the original 24-setting one-factor grid.

Cells to fill (ranked by impact on Phase 1 conclusions):

| target | reason | minimum n_envs needed |
|---|---|---|
| **baseline × rough_b** | currently n=0; every rough_b Δ vs baseline above is "n/a" | 4 |
| **leg_pitch_amp_{0.30, 0.60} × rough_b** | non-monotone trend rests on no rough_b sample | 3 each |
| **wave_num_{2.0, 2.5} × rough_b** | rough_b winner = 2.0 with n=1 | 3 each |
| **body_amp_{0.20, 0.40, 0.70} × rough_b** | the monotone↓ trend's tail is n=2/2/4 | 3 each |
| **body_imp_{0.6, 1.5, 2.0} × {rough_a, rough_b}** | non-monotone reading rests on n=1 | 3 each |
| **leg_yaw_amp_{0.30, 0.60} × {flat, long_wave}** | trend reads `decreasing` only because flat side is n=2 | 3 each |

A spawn-RNG seed loop (run the same setting with N reset seeds)
will hit any target cell with high probability — much cheaper than
re-running the full 24-setting grid. Estimated wallclock: ~6 min
for ~10 targeted settings.

The same script is reusable — only the `SETTINGS` list changes.
That's a small enough modification to count as a sanity-script
edit (allowed without packet) provided the controller / env are
not touched. **Confirm with user before running.**

### 2.2 — roughness-sensor calibration (this session, read-only)

Use the existing sweep traces to compute the proposed `R_raw`
formula (see `SCRIPTED_ADAPTIVE_CONTROLLER_DESIGN.md` §4) on the
baseline rows and verify per-block separation. No simulation
needed; pure CSV post-processing. Output: a small markdown report
under `outputs/analysis/roughness_sensor_calibration.md`.

### 2.3 — scripted adaptive controller — implementation (FUTURE session)

Implement `scripts/sanity/run_scripted_adaptive_controller.py` and
`scripts/sanity/analyze_scripted_adaptive_results.py` per the
design doc. The runner monkey-patches `_apply_global_6d_action` —
**no env / controller code changes**. This is at the edge of
"sanity script" vs "major action" because it inserts a new control
loop. **Generate a ChatGPT review packet before implementation**:

```
.\scripts\rl\make_chatgpt_review_packet.ps1 `
  -ProposedNextCommand "implement scripts/sanity/run_scripted_adaptive_controller.py per docs/SCRIPTED_ADAPTIVE_CONTROLLER_DESIGN.md"
```

### 2.4 — head-to-head evaluation

Once the runner exists, evaluate it on the same 6-block stratified
spawn lattice used by the sweep, with the same metrics and
locomotion_score. Promotion criterion: scripted controller ≥ all-1.0
baseline on every block (within sweep noise) AND ≥ +20 % v_fwd on
rough_a / rough_b vs baseline. Report under
`outputs/analysis/scripted_adaptive_eval.md`.

### 2.5 — only if §2.4 passes — Phase 2 RL design

A separate design doc will specify an RL policy whose action space
is the **band-map deltas** (not the per-step parameters). This
keeps the RL search interpretable and bounds blast radius.
Triggered only after §2.4 completes; out of scope for this session.

## 3. Explicit non-actions (for Phase 1)

Do **not**, without a fresh ChatGPT review packet:

- adopt the Phase-8 proposed reward,
- change `_REWARD_W_*` weights or any `_PASS12_*` constant in
  `envs/centipede_env.py`,
- expand `_G6_BODY_AMP_LO` (or any other `_G6_*` action bound),
- enable any new `passN_*_enabled` flag,
- launch any PPO run (smoke or overnight),
- change `kl_threshold`, `mini_epochs`, `e_clip`, `entropy_coef`,
  or any other PPO hyperparameter in any
  `tasks/agents/rl_games_*.yaml`,
- modify `controllers/impedance_controller_torch.py`,
- modify `assets/centipede.usd` or `assets/config.yaml`,
- rename / move / delete any checkpoint under
  `outputs/rl_smoke/promoted/` or
  `outputs/rl_smoke/stage_b_transition_global6d/2026-05-08_15-27-26/`.

Allowed without a packet:

- read-only post-processing of existing CSVs,
- new files under `docs/` and `outputs/analysis/`,
- new sanity scripts under `scripts/sanity/` that **do not** import
  `controllers/` or `envs/` (the existing sweep runner is at the
  edge — it monkey-patches the env at runtime; the scripted
  adaptive runner is in the same family).

## 4. Per-axis evidence summary (one-liner per axis)

For quick reference, from §2 of
`global_parameter_sweep_fixed_freq_analysis.md`:

- `body_amp`     — **decrease with roughness** (1.30 flat → 0.20 rough_b). Strongest signal.
- `wave_num`     — **increase with roughness** (1.0 → 2.0 on rough). Going > 2.0 always hurts.
- `leg_yaw_amp`  — **decrease with roughness** (1.0 → 0.30 rough_b). Only 2 perturbation values tested.
- `leg_pitch_amp`— keep at 1.0 except mild reduction on rough_b. Reductions on mid blocks cost ~13–20 mm/s.
- `body_imp`     — anything in [0.6, 2.0] is roughly equivalent; 0.30 hurts everywhere.
- `freq`         — **pinned at 1.0** per project directive. The freq=1.40 ceiling is reference only.

## 5. Files this phase will touch

Created:

- `outputs/analysis/global_parameter_sweep_fixed_freq_analysis.md` ✅
- `outputs/analysis/global_parameter_sweep_fixed_freq_per_value.csv` ✅
- `outputs/analysis/global_parameter_sweep_coverage_matrix.md` ✅
- `scripts/sanity/analyze_sweep_fixed_freq.py` ✅
- `docs/SCRIPTED_ADAPTIVE_CONTROLLER_DESIGN.md` ✅
- `docs/PHASE_1_NEXT_STEPS_AND_NON_ACTIONS.md` ✅ (this file)

Pending (later sessions, gated on user / ChatGPT approval):

- `outputs/analysis/roughness_sensor_calibration.md`
- `scripts/sanity/sweep_repair_focused.py` (only if user OKs §2.1)
- `scripts/sanity/run_scripted_adaptive_controller.py` (after packet)
- `scripts/sanity/analyze_scripted_adaptive_results.py`
- `outputs/analysis/scripted_adaptive_eval.md`

No changes pending in `envs/`, `controllers/`, `assets/`, or
`tasks/agents/`.
