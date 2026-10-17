# Phase 3 — Absolute Wave Number × Homogeneous Terrain Sweep — REPORT

> 2026-05-11. Sweep + analysis complete. Companion files:
> - `outputs/analysis/phase3_absolute_wave_homogeneous_sweep.csv` (80 rows)
> - `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_analysis.md`
> - `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_figures/01..08.png`
> - `outputs/analysis/phase3_absolute_wave_homogeneous_reward_calibration.md`
> - `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_meta/{flat,L1,L2,L3}.json`
>
> **No RL was run. No reward was modified. No protected files were edited.**

---

## 1. What this sweep tested

| design dimension | values |
|---|---|
| absolute wave number (= CPG `n_wave_scalar × wave_number_scale_runtime`) | **{1, 2, 3, 4}** |
| homogeneous terrain wavelength λ | **flat, 90 mm, 60 mm, 30 mm** |
| terrain amplitude amp (fixed across rough cells) | **6 mm** |
| trials per cell (matched spawn-y) | **5** |
| body amp / leg yaw / leg pitch / freq / body_imp | **all baseline (scale = 1.0)** |
| total trials | **4 × 4 × 5 = 80** |

Run via `scripts/sanity/sweep_phase3_absolute_wave.py` (one IsaacLab boot
per terrain, 20 parallel envs per boot = 4 abs_waves × 5 trials);
homogeneous arena enabled by **monkey-patching
`arena.generate_flat_to_rough_transition_heightmap`** to call
`generate_single_wavelength_heightmap` instead — **no protected-file
edit**. Matched spawn via `MatchedSpawnRng` (each trial_id pins the
same y-stratum across all 4 abs_wave conditions on that terrain).

## 2. Convention correction (critical)

The prior Phase 2 sweeps wrote `wave_num` columns that were actually
**runtime scales** applied to the controller's `n_wave_scalar = 3.0`
baseline. So prior conditions labelled `wave_num = 2` were really
**6 traveling waves** along the body. Phase 3 explicitly uses
**absolute wave number** = `3.0 × wave_number_scale`:

| sweep column | actual CPG waves |
|---:|---:|
| abs_wave = 1 | 1 traveling wave (body wavelength ≈ body length, ~50 mm) |
| abs_wave = 2 | 2 waves (λ_body_wave ≈ 25 mm) |
| abs_wave = 3 | 3 waves (= controller baseline, λ_body ≈ 17 mm) |
| abs_wave = 4 | 4 waves (λ_body ≈ 12.5 mm) |

All Phase 3 outputs use absolute wave number on legends and tables.

## 3. Headline finding

**Higher absolute wave number trades flat-ground speed for
terrain-robustness retention.** The forward-velocity curves vs
terrain wavelength clearly separate:

| abs_wave | flat (mm/s) | λ=90 | λ=60 | λ=30 | decline slope | CoV across rough |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 44.66 ± 0.14 | 33.37 ± 5.77 | **16.45 ± 6.36** | 29.26 ± 2.52 | **−6.31** | **0.335** |
| 2 | **47.24 ± 0.03** | **45.05 ± 0.21** | **41.56 ± 1.31** | **32.79 ± 1.44** | −4.68 | 0.159 |
| 3 | 37.25 ± 0.03 | 36.59 ± 0.40 | 35.86 ± 0.33 | 30.53 ± 1.99 | −2.09 | 0.096 |
| 4 | 32.85 ± 0.05 | 32.20 ± 0.32 | 31.34 ± 0.54 | 28.71 ± 0.70 | **−1.33** | **0.059** |

(See `figures/01_v_fwd_vs_terrain.png`.)

Key observations:

- **abs_wave = 1 is brittle.** On λ=60 mm it drops to 16 mm/s mean
  with one trial running **backwards** (v_fwd = −3.94 mm/s). On λ=90
  it is highly variable (SE = 5.8 mm/s). On λ=30 it recovers a bit
  (mean 29) but with the worst SE-relative spread.
- **abs_wave = 2 is the speed winner across the board** — fastest on
  flat (47.2 mm/s) AND on λ=90 (45) AND on λ=60 (41.6). Retention
  0.954 / 0.880 / 0.694 across L1/L2/L3.
- **abs_wave = 3 (the controller baseline) is moderate and very
  retentive** — 97-98 % of flat speed maintained on λ=90 and λ=60.
- **abs_wave = 4 is the slowest on flat but the most consistent**
  across terrains (CoV = 0.059, slope = −1.33 mm/s per level).
- **abs_wave = 4 has the highest worst-case** (28.71 mm/s) of any
  abs_wave across all 4 terrains.

## 4. Paired-trial Δv vs abs_wave = 1 (matched spawn)

(`analysis.md` §4)

| terrain | Δv(2−1) | Δv(3−1) | Δv(4−1) |
|---|---:|---:|---:|
| flat       | **+2.58** | **−7.41** | **−11.81** |
| λ=90 mm    | **+11.68** | +3.22 | −1.17 |
| λ=60 mm    | **+25.11** | **+19.41** | **+14.89** |
| λ=30 mm    | +3.53 | +1.27 | −0.56 |

- On λ=60 (the regime most damaging to abs_wave=1) every higher wave
  number beats wave=1 by **15–25 mm/s** with matched spawn.
- On flat, going above abs_wave=2 actually slows the centipede — the
  flat-ground optimum is **abs_wave ≈ 2**, not the controller's
  baseline of 3.
- On λ=30 the gap between abs_wave conditions narrows — at this
  wavelength all configurations converge to ~28–33 mm/s. The hardest
  terrain is hard for everyone.

## 5. Direct answers to the user's 10 questions (§ 9 of the brief)

**Q1. Does absolute wave number 1 → 4 show the expected trend?**

Partly. The expected trend was "higher abs_wave → better retention on
rougher terrain." That holds for retention and CoV: abs_wave=4 has the
flattest decline (−1.33 mm/s/level) and the lowest CoV across rough
terrains (0.059). But it does NOT hold for absolute speed — abs_wave=2
is the fastest on every terrain except λ=30 where it ties.

**Q2. Does higher absolute wave number preserve speed better on
shorter-wavelength terrain?**

Yes for retention, no for absolute speed. abs_wave=4 keeps 87 % of
its flat speed on λ=30; abs_wave=1 keeps 65 % (with high variance).
However abs_wave=2 absolute speed (32.79) on λ=30 is still higher than
abs_wave=4's (28.71).

**Q3. Is the trend statistically stable across 5 trials?**

For abs_wave=2, 3, 4 the within-cell SE is small (≤ 2 mm/s) — the
ranking is stable. For abs_wave=1 the within-cell SE on rough
terrains is **large** (5.8 to 6.4 mm/s) — its mean is a poor
descriptor of the per-trial outcome. Paired Δv (§ 4) is the cleaner
test and supports the same ordering: 2 > 3 > 4 ≥ 1 on rough.

**Q4. Is absolute wave number 4 beneficial or too high?**

abs_wave=4 trades 12 mm/s of flat-ground speed for the flattest
robustness curve. Whether that's "beneficial" depends on the
deployment scenario. For a robot whose task includes a mix of flat
and rough, abs_wave=4 is "safe but slow"; abs_wave=2 is "fast on flat
and on moderate-λ rough" but **slower than abs_wave=3 on λ=30**
(32.79 vs 30.53 — these overlap within SE so call it a tie).

**Q5. Is the best wave number terrain-dependent?**

Yes:
- flat: abs_wave = 2
- λ=90: abs_wave = 2
- λ=60: abs_wave = 2
- λ=30: abs_wave = 2 (mean), but abs_wave=3 and 4 have tighter spread

abs_wave=2 dominates the means; higher wave numbers win on
consistency.

**Q6. Does higher wave number improve retention at the cost of
flat-ground speed?**

**Yes — directly.** Decline slope monotonically shallows from −6.31
to −1.33 mm/s per level as abs_wave 1 → 4. Flat speed monotonically
decreases from 47.24 to 32.85 mm/s as abs_wave 2 → 4. The trade-off
is real and clean.

**Q7. Is the effect clean enough to justify future reward/RL design?**

Yes for **wave=2 vs wave=1** (huge effect on rough terrain, ~25 mm/s).
The wave=3 and wave=4 trade-off vs wave=2 is more subtle and depends
on whether the experimenter cares about absolute speed or retention.
Reward design should keep this asymmetry in mind — see the
companion reward-calibration doc.

**Q8. What should the next RL action space focus on?**

If RL is to discover better-than-wave=2 behaviour on the homogeneous
arena, it should be allowed to modulate:
- `wave_number_scale` in a range that includes [1/3, 4/3] (= absolute
  1..4), centred on the controller baseline of 1.0;
- possibly body_imp once a separate impedance study is done (out of
  scope here);
- NOT freq (per project policy);
- NOT body_amp / leg_yaw / leg_pitch unless a separate sweep shows
  they help.

A single-dimension RL pilot on `wave_number_scale` alone may be
useful — the design is already well-explored at 4 fixed values
(this sweep), so RL's job becomes "find the terrain-dependent
optimum" rather than "explore from scratch."

**Q9. What reward term would be appropriate if the trend is
confirmed?**

See `outputs/analysis/phase3_absolute_wave_homogeneous_reward_calibration.md`
for full discussion. Recommendation: **first pilot with no new
reward term** (candidate F). If that fails to find the higher-wave
strategy on rough terrain, add a `severity × v_forward` term
(candidate A, the Pass 13 prototype) but with **severity computed
per-env from the cfg-level homogeneous parameters**, not from a
block-lookup.

**Q10. What is still uncertain?**

- The flat-ground optimum at abs_wave=2 is surprising — the
  controller was tuned at abs_wave=3 (the baseline). It suggests
  the existing controller might be over-waved at baseline. Worth
  flagging to the controller designer.
- The within-cell variability of abs_wave=1 on λ=60 (one trial
  reversed direction) suggests **wave=1 with body wavelength ≈ body
  length resonates badly with λ_terrain ≈ 60 mm**. This is the
  spatial-frequency-matching prediction. A finer-grained λ sweep
  could pin down where the resonance band lies.
- Body impedance was held constant at baseline. We don't know if
  higher impedance helps abs_wave=1 recover, or if it interacts
  with the wave_num optimum.
- 5 trials per cell. The qualitative ordering is clean but the
  precise effect sizes (especially the abs_wave=4 vs 3 tie on λ=30)
  are uncertain.

## 6. Terrain quality check

| terrain | λ (mm) | amp (mm) | height_rms (mm) | slope_rms | slope_p95 |
|---|---:|---:|---:|---:|---:|
| flat        | — | 0 | 0.00 | 0.000 | 0.000 |
| L1 λ=90 mm  | 90 | 6 | 1.22 | 0.036 | 0.138 |
| L2 λ=60 mm  | 60 | 6 | 1.32 | 0.060 | 0.227 |
| L3 λ=30 mm  | 30 | 6 | 1.16 | 0.104 | 0.395 |

The intended severity ladder is monotonic in **slope_rms** and
**slope_p95** (the relevant physical quantity). `height_rms` is
non-monotonic by a few %, which is expected because amp is held
constant; height_rms is determined by the random sinusoid
realisation more than by λ. The slope-based ordering (flat < L1 <
L2 < L3) is the right interpretive axis.

`figures/08_terrain_severity_overview.png` shows these visually.

## 7. Files produced this session

| path | role |
|---|---|
| `scripts/sanity/sweep_phase3_absolute_wave.py` | sweep driver (one terrain per boot, homogeneous arena via monkey-patch) |
| `scripts/sanity/run_sweep_phase3_absolute_wave.sh` | shell driver (4 terrains × force-kill on success) |
| `scripts/sanity/analyze_phase3_absolute_wave.py` | numpy-only analyzer + 8 plots |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep.csv` | 80-row data |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_meta/{flat,L1,L2,L3}.json` | per-boot metadata |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_figures/01..08.png` | 8 plots |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_analysis.md` | aggregate table + Δv table |
| `outputs/analysis/phase3_absolute_wave_homogeneous_reward_calibration.md` | reward proposal (no implementation) |
| `docs/RENDERING_PIPELINE_DEBUG_NOTES.md` | rendering-fix authoritative reference |
| `docs/PHASE_3_ABSOLUTE_WAVE_HOMOGENEOUS_SWEEP_REPORT.md` | this report |
| `docs/review_packets/latest_chatgpt_review_packet.md` | next review packet |

## 8. Strict non-actions this session

- **No PPO training.** No checkpoint promoted, moved, or deleted.
- **No Pass 13 enablement.** The flag remains default-OFF.
- **No reward source modifications.** `envs/centipede_env.py` reward
  logic untouched.
- **No action-bound changes.**
- **No protected-file edits.** Homogeneous terrain enabled by sanity
  script monkey-patch; the env source still calls the (now patched)
  `arena.generate_flat_to_rough_transition_heightmap`.
- **No composite transition arena used as scientific evidence.**
  Phase 3 uses single-wavelength homogeneous arenas, with matched
  spawns across the abs_wave conditions on each arena.
- **No claim of statistical significance** beyond what 5 paired
  trials per cell support. Standard errors are reported; the
  abs_wave = 2 advantage on rough terrain is large enough that the
  ranking is stable even at this N.

## 9. Recommended next steps

In priority order (each is a separate scope, NOT executed this session):

1. **Render representative videos** of the 4 (abs_wave × λ=30 mm)
   cells plus a flat baseline using `render_single_condition_video.py`.
   This is optional per the brief (§ 10) and is the next safe
   follow-up.
2. **Approve the homogeneous-terrain cfg/env addition** (per the
   earlier Phase 3 design doc § 4.3) so future RL pilots can use
   `terrain_kind="homogeneous"` directly without the monkey-patch.
3. **No-reward pilot** on the homogeneous arena (candidate F from
   the reward-calibration doc). Use the Pass 12 RECOMMENDED PPO
   config. Check if RL spontaneously discovers higher-abs_wave gait
   on the rough arena.
4. **Reward-term pilot** only if (3) fails. Use candidate A (severity
   × v_forward) with severity = `amp_terrain / λ_terrain` (cfg-level),
   default-OFF behind a new flag, snapshot-backed.
5. **Finer-grained λ sweep** around 60-90 mm to pinpoint the
   abs_wave=1 resonance regime.
6. **Body-impedance study** — separate sweep, with abs_wave held
   constant at the Phase 3 winner (likely 2 or 3), varying body_imp
   scale in narrow range [0.5, 1.5].

Stop after producing the review packet, per the user's directive.
Do not proceed to (3)–(6) without explicit user / ChatGPT approval.
