# Scripted terrain-conditioned adaptive global-parameter controller — design

> Drafted 2026-05-10. Status: **DESIGN ONLY**. No code edits to
> `controllers/` or `envs/` yet. Per `docs/CHATGPT_SUPERVISION_PROTOCOL.md`,
> any in-place change to those directories is a major action and requires
> a fresh ChatGPT review packet first.

## 1. Purpose

Demonstrate, *without RL*, that the global locomotion parameters
already exposed by the controller (`body_amp`, `leg_yaw_amp`,
`leg_pitch_amp`, `wave_num`, `body_imp`, `freq`) admit a
**terrain-conditioned scripted policy** that:

- improves on the all-1.0 baseline on rough blocks,
- does so via **biologically interpretable monotonic adaptation**
  (smaller body wave + smaller leg yaw + more wave nodes on rougher
  ground), and
- maintains `freq` near 1.0 (no high-frequency exploit).

This baseline is a precondition for the *next* RL phase. Once a
scripted controller exists that exploits the trends measured in
`outputs/analysis/global_parameter_sweep_fixed_freq_analysis.md`,
RL has a target to match or beat.

## 2. What the data justifies (fixed-freq=1.0)

From `global_parameter_sweep_fixed_freq_analysis.md` §2 and §3, with
`freq` pinned at 1.0:

| axis | trend with R (roughness) | confidence | comment |
|---|---|---|---|
| `body_amp`     | **monotone↓** 1.30 → … → 0.20 | high on flat/long_wave (n≥3); LOW on rough_b (n=2, no baseline) | Largest, clearest effect. |
| `wave_num`     | **monotone↑** 1.0 → 1.0 → 1.0 → 1.0 → 2.0 → 2.0 | OK on rough_a (n≥2); LOW on rough_b (n=1) | Going above 2.0 *always* hurts v_fwd. |
| `leg_yaw_amp`  | **monotone↓** 1.0 → 1.0 → 1.0 → 1.0 → 0.6 → 0.3 | only 2 perturbation values (0.30, 0.60) tested | Direction is consistent with body_amp. |
| `leg_pitch_amp`| non-monotone: 0.30 wins flat; baseline mid; 0.60 rough_b | LOW (n≤4) | Reducing this on rough hurts v_fwd up to −20 mm/s. **Keep at 1.0** unless re-measured. |
| `body_imp`     | non-monotone; values in [0.6, 2.0] are roughly v-neutral; 0.30 hurts everywhere | mixed | Treat as a small free knob in [0.6, 1.5]. Avoid going below 0.6. |

Coverage gaps that downgrade confidence (per
`global_parameter_sweep_coverage_matrix.md`): 11 zero-evidence cells,
59 cells with n∈{1,2}. In particular **no baseline sample on
`rough_b`**, and `leg_pitch_amp` and `wave_num_3.5` not sampled on
`rough_b`.

## 3. Architecture

```
                                           ┌───────────────────┐
  obs ─► roughness_estimator ─► R_raw  ─►  │ EMA (τ_R seconds)  │ ─► R_smooth
                                           └───────────────────┘
                                                       │
                                                       ▼
                                           ┌───────────────────┐
                                           │ band classifier    │ ─► band ∈ {flat, mid, rough}
                                           │  + hysteresis (h)  │
                                           └───────────────────┘
                                                       │
                                                       ▼
                                           ┌───────────────────┐
                                           │ band → target u*   │ ─► u_target ∈ ℝ⁵
                                           │  (5 params, freq=1)│
                                           └───────────────────┘
                                                       │
                                                       ▼
                                           ┌───────────────────┐
                                           │ low-pass on u      │ ─► u_applied
                                           │  (τ_u seconds)     │
                                           └───────────────────┘
                                                       │
                                                       ▼
                                  controller.apply_global_6d_modulation(
                                      body_amp_scale     = u_applied[0],
                                      leg_yaw_amp_scale  = u_applied[1],
                                      leg_pitch_amp_scale= u_applied[2],
                                      wave_number_scale  = u_applied[3],
                                      body_imp_scale     = u_applied[4],
                                      freq_scale         = 1.0,           # PINNED
                                  )
```

Per-env state is kept in tensor buffers shaped `(num_envs,)` so the
script multiplexes naturally over the 16-env stratified spawn lattice.

## 4. Roughness estimator

The robot already publishes (via Stage-B `latest_policy_audit`):

- `F_total_sensor` — sum of foot contact force magnitudes (N).
- `vz_rms` — vertical velocity RMS over a window.
- `proj_g_z_abs` — body up-vector alignment.
- `n_contact_mean` — fraction of feet in contact.

Choose a single dimensionless roughness proxy that combines these:

```
R_raw = w₁ · clip(σ_F / F̄, 0, 1)               # contact-force coefficient of variation
      + w₂ · clip(vz_rms / 0.020, 0, 1)         # vertical-velocity bounce
      + w₃ · clip((1 − proj_g_z_abs) / 0.02, 0, 1)   # tilt fraction
```

with provisional weights `w₁ = 0.5, w₂ = 0.3, w₃ = 0.2`. All three
sub-signals are rectified and clipped to [0, 1] so `R_raw ∈ [0, 1]`
even before averaging. The `σ_F / F̄` term is computed over a
rolling window of `H_R = 21` policy steps (~1.0 s at 0.048 s
control), matching the Pass-11 windowed-CoT buffer.

`R_smooth` is an exponential moving average of `R_raw`:

```
R_smooth ← α_R · R_raw + (1 − α_R) · R_smooth_prev
α_R = dt_pol / (τ_R + dt_pol)            with τ_R = 1.5 s
```

A long τ_R is intentional: the robot must commit to a posture for
several gait cycles before a band switch.

## 5. Band classifier with hysteresis

Three operating bands map to fixed target parameter sets:

| band   | R_smooth range (enter / exit)   | rationale (R values from the sweep) |
|---|---|---|
| flat   | enter ≤ 0.18; exit > 0.25     | covers `flat (0.05)` + `long_wave (0.20)` |
| mid    | enter [0.25, 0.65]; exit < 0.18 or > 0.72 | covers `intermediate (0.35)` + `medium (0.55)` |
| rough  | enter ≥ 0.72; exit < 0.65       | covers `rough_a (0.80)` + `rough_b (0.95)` |

The hysteresis half-band is `h = 0.07` (asymmetric edges above).
Without it, an `R_smooth` riding the boundary would flap between
two parameter sets and chatter through the low-pass.

## 6. Target parameter map

Derived from the fixed-freq table; values inside the sampled range
of each axis, rounded to safe interior values:

| param            | flat | mid | rough | safety bound | source |
|---|---:|---:|---:|---|---|
| `body_amp_scale` | 1.10 | 1.00 | 0.55 | clamp to [0.20, 1.30] | sweep best 1.30 (flat) → 0.20 (rough_b); pulled toward interior to stay measurable |
| `leg_yaw_amp_scale` | 1.00 | 1.00 | 0.55 | clamp to [0.30, 1.00] | sweep best 1.0 → 0.6 (rough_a) → 0.3 (rough_b); pulled toward 0.55 to avoid the 0.30 LOW-CONF cell |
| `leg_pitch_amp_scale` | 1.00 | 1.00 | 0.85 | clamp to [0.60, 1.00] | reductions hurt v_fwd in mid (−13 mm/s). Conservative. |
| `wave_number_scale` | 1.00 | 1.00 | 1.80 | clamp to [1.00, 2.00] | sweep best stays 1.0 until rough; 2.0 wins on rough; >2.0 always hurts. |
| `body_imp_scale` | 1.00 | 1.20 | 1.20 | clamp to [0.60, 2.00] | non-monotone but flat in [0.6, 2.0]; mid-range 1.2 is safe |
| `freq_scale`     | 1.00 | 1.00 | 1.00 | **pinned** | project directive — freq stays near 1.0 |

The `mid` row is essentially the all-1.0 baseline. The `flat` row
allows a mild body-amp boost (from the 1.30 sweep observation, which
gave +3.77 mm/s on flat at zero F-cost). The `rough` row applies
all four sweep-measured monotonic reductions/increases.

## 7. Output low-pass

Each of the 5 outputs (excluding the pinned freq) passes through an
independent first-order low-pass:

```
u_applied[k] ← α_u · u_target[k] + (1 − α_u) · u_applied_prev[k]
α_u = dt_pol / (τ_u + dt_pol)            with τ_u = 0.6 s
```

`τ_u = 0.6 s` corresponds to roughly one gait cycle at the
baseline frequency, so a band switch propagates to the controller
over ~1 cycle — slow enough to avoid leg-collision artifacts, fast
enough to react inside one terrain block.

The runtime smoothing layer inside `_apply_global_6d_action` (the
Stage-B 0.30 s smoother) is **independent** of this low-pass and
will further attenuate any residual transient.

## 8. Safety guardrails

- Per-axis hard clip after low-pass (the `clamp` column in §6).
  These bounds are inside the existing controller action-bounds (no
  expansion of `_G6_*` constants — that would be a major action).
- `freq_scale` is asserted to remain == 1.0 every step. A debug
  flag `assert_freq_pinned=True` raises if any other module
  perturbs `freq_scale_runtime`.
- If `R_smooth` is NaN or saturates at 1.0 for >5 s (sensor failure
  mode), output reverts to all-1.0 baseline. (Logged.)
- All buffers initialized to 1.0; first 5 policy steps use baseline
  parameters before the EMA warms up.

## 9. Update rate

The controller runs at `dt_pol = 0.048 s` (= 12 × physics).
- `R_raw` updates every step.
- `R_smooth` updates every step (cheap EMA).
- Band classifier evaluated every step but only fires a `band`
  change when the hysteresis condition holds → at most one
  band-change per ~τ_R seconds in practice.
- `u_target` updates only on band change.
- `u_applied` updates every step (low-pass).

This is the "slow modulation" the project mandate calls for: the
band can change at most a few times per minute on long terrains;
within a band, parameters drift smoothly over ~τ_u seconds toward
the target; per-step parameter jumps are O(α_u · Δu_target) ≈ a
few percent.

## 10. Implementation surface (proposed, NOT implemented)

A standalone read-only script (no env mutation):

```
scripts/sanity/run_scripted_adaptive_controller.py
```

Mirrors `scripts/sanity/sweep_global_parameters.py`:
- monkey-patches `base_env._apply_global_6d_action` to compute
  `u_target` from the band map and call `controller.apply_global_6d_modulation(...)`,
- runs N policy steps × M envs across the 6 transition blocks,
- writes a CSV identical in schema to the sweep CSV plus columns
  `R_raw`, `R_smooth`, `band`, and the 6 `u_applied[*]` values,
- emits `[scripted-adaptive] report -> done` for the watchdog.

A second script:

```
scripts/sanity/analyze_scripted_adaptive_results.py
```

post-processes that CSV and produces a head-to-head:
- vs **fixed all-1.0 baseline** on every block,
- vs **freq=1.40 ceiling** (informational, not a target),
- vs **best per-block one-factor** sweep value.

If — and only if — the scripted controller demonstrably matches or
beats the all-1.0 baseline on rough_a/rough_b without RL, then the
*next* phase is to design an RL agent whose action space is the
band-map deltas (i.e. RL adjusts the lookup, not the per-step
parameters). That's a future design doc.

## 11. What this design does NOT do

- Does **not** modify `envs/centipede_env.py` or `controllers/`.
- Does **not** change the reward.
- Does **not** expand action bounds.
- Does **not** train PPO.
- Does **not** rely on `freq` ≠ 1.0.
- Does **not** assume the existing roughness sensor is calibrated;
  §13 specifies how to calibrate it.

## 12. Pre-launch validation checklist (read-only)

Before any code is written:

1. Run `analyze_sweep_fixed_freq.py` — verify the trends in §2 hold
   (already produced; reproducible).
2. Run a 1-shot diagnostic script that reads the sweep trace CSVs
   and computes the proposed `R_raw` for each setting × env, per
   block, then verifies the bands separate the 6 blocks correctly
   (i.e. `flat` rows fall in band `flat`, etc.). If not, retune
   `(w₁, w₂, w₃)` and the band cutoffs.
3. Confirm the sweep coverage gaps (rough_b baseline, leg_pitch on
   rough_b, etc.) won't be hit by the scripted controller in an
   unexpected way; if they will, generate a *focused* re-sweep
   first (Stage A repair).

## 13. Calibration protocol for the roughness sensor

Use the existing 6-block stratified spawn lattice:

```
for setting in [baseline]:                          # only the all-1.0 control
    for env in 0..15:
        record per-step: F_total, vz_rms, proj_g_z_abs, block_label
        compute R_raw with provisional weights
    per-block: report mean(R_raw), std(R_raw)
```

Acceptance: per-block mean(R_raw) is monotone in the ground-truth R
column (`flat 0.05 → rough_b 0.95`), and adjacent bands' R_raw
distributions don't overlap by more than 30 %.

If acceptance fails, retune weights using a least-squares fit of
mean(R_raw) → ground-truth R.

## 14. Stop conditions

Halt all downstream work and request a ChatGPT review packet if:

- the scripted controller decreases `v_fwd` by more than 10 % vs the
  all-1.0 baseline on **any** block — would indicate the trend table
  is corrupted by the coverage gaps;
- bands flap more than once per 10 s sustained on a single block —
  hysteresis tuning insufficient;
- `R_smooth` saturates at 0 or 1 for >50 % of any block's duration
  — sensor weighting wrong;
- per-step `|Δu_applied|` exceeds 5 % of the parameter range —
  low-pass tuning wrong.

## 15. Open items deferred to a follow-up doc

1. Scheduled re-sweep to repair coverage gaps (esp. baseline on
   rough_b; leg_pitch on rough_b; high-wave_num × rough_a).
2. Stage B sweep (interactions: body_amp × wave_num on rough_a/b).
3. Reward-calibration discussion (forbidden until the scripted
   adaptive controller has produced trustworthy metrics).
4. RL design (forbidden until the above).
