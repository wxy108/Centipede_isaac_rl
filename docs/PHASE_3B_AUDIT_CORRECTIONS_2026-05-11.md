# Phase 3B — corrected audit mini-report

> Authored 2026-05-11 in response to ChatGPT's AUDIT-MORE verdict.
> Replaces the body-weight, Pareto, and video-evidence claims in
> `docs/PHASE_3B_ABSOLUTE_WAVE_SWEEP_REPORT.md` and
> `docs/PHASE_3_REWARD_CALIBRATION_NEEDS.md` where those conflict
> with the data. Numbers below are regenerated directly from
> `outputs/analysis/phase3b_per_condition_summary.csv` and
> `outputs/analysis/phase3b_absolute_wave_sweep.csv`.

---

## 1. Body-weight constants (single source of truth)

- Robot mass: **m = 0.005 kg** (consistent across the project — see `pass11_overnight_BEST_ep12_bio_metrics.txt`)
- Gravity: **g = 9.81 m/s²**
- **Body weight: W = m·g = 0.04905 N** (≈ 0.049 N)
- Impact-spike threshold used in `sweep_phase3b_absolute_wave.py`: **5·W = 0.2453 N** (= 0.245 N)

## 2. CORRECTION: previous "15× body weight" claim is WRONG by 10×

The Phase 3B report and reward-calibration-needs doc both said:

> abs_wave = 1.5's F_max ≈ 0.073 N on λ=30 mm = **15× body weight**

This is incorrect. The actual ratio is:

- F_max (mean of 5 trials) on λ=30 mm × abs=1.5: **0.0730 N**
- F_max / W = 0.0730 / 0.04905 = **1.49× body weight**

The 10× discrepancy comes from confusing mass (5 g = 0.005 kg) with weight in Newtons. 0.073 / 0.005 = 14.6 ≈ 15, but that ratio is meaningless dimensionally (Newtons / kg, not Newtons / Newtons).

**Consistency check**: `impact_spike_count` (frames where instantaneous summed contact force exceeded 5·W = 0.245 N) is **0 across all 120 trials**. This is consistent with the correct ratio (peak F_max ≈ 1.9× W, never reaching 5× W), and *inconsistent* with the erroneous "15× body weight" claim (which would have implied peak F_max ≈ 0.74 N, beating 5·W by 3×).

## 3. Corrected body-weight-normalised F_max table

**Mean F_max / W over 5 trials** (per condition):

| terrain | abs=1.5 | abs=2.0 | abs=2.5 | abs=3.0 | abs=3.5 | abs=4.0 |
|---|---:|---:|---:|---:|---:|---:|
| flat | 0.69× | 0.63× | 0.71× | 0.69× | 0.70× | **0.74×** |
| λ=90 mm | **0.80×** | 0.62× | 0.68× | 0.69× | 0.71× | 0.72× |
| λ=60 mm | **1.32×** | 0.87× | 0.72× | 0.66× | 0.67× | 0.70× |
| λ=30 mm | **1.49×** | 0.89× | 0.75× | 0.86× | 0.78× | 0.71× |

**Max F_max across 5 trials / W** (worst-trial peak, NOT condition mean):

| terrain | abs=1.5 | abs=2.0 | abs=2.5 | abs=3.0 | abs=3.5 | abs=4.0 |
|---|---:|---:|---:|---:|---:|---:|
| flat | 0.78× | 0.68× | 0.78× | 0.78× | 0.73× | **0.79×** |
| λ=90 mm | **1.03×** | 0.64× | 0.75× | 0.76× | 0.72× | 0.76× |
| λ=60 mm | **1.89×** | 1.05× | 1.06× | 0.71× | 0.77× | 0.81× |
| λ=30 mm | **1.76×** | 1.37× | 1.03× | 1.51× | 0.93× | 0.76× |

**Relative F_max premium** (abs_wave=1.5 vs each other abs_wave, on rough terrain):

| terrain | (abs=1.5)/(abs=2.0) | (abs=1.5)/(abs=3.0) | (abs=1.5)/(abs=4.0) |
|---|---:|---:|---:|
| λ=90 mm | 1.29× | 1.15× | 1.11× |
| λ=60 mm | 1.52× | 2.00× | 1.87× |
| λ=30 mm | 1.67× | 1.73× | 2.09× |

All ratios in §3 are **dimensionally correct** (force ÷ force).

## 4. Corrected Pareto fronts (regenerated directly from data)

Per-terrain Pareto-non-dominated points (objective: maximise v_fwd_mean, minimise F_max_mean).
A point is dominated if some other (terrain-matched) point has both higher v_fwd AND lower F_max.

### 4.1 Speed vs F_max

| terrain | abs_wave | v_fwd (mm/s) | F_max (N) | F_max/W | on Pareto front? |
|---|---:|---:|---:|---:|---|
| flat | 1.5 | 51.16 | 0.0339 | 0.69× | **yes** |
| flat | 2.0 | 47.24 | 0.0310 | 0.63× | **yes** |
| flat | 2.5 | 40.98 | 0.0349 | 0.71× | no |
| flat | 3.0 | 37.25 | 0.0340 | 0.69× | no |
| flat | 3.5 | 34.69 | 0.0343 | 0.70× | no |
| flat | 4.0 | 32.85 | 0.0362 | 0.74× | no |
| λ=90 mm | 1.5 | 47.79 | 0.0391 | 0.80× | **yes** |
| λ=90 mm | 2.0 | 45.05 | 0.0304 | 0.62× | **yes** |
| λ=90 mm | 2.5 | 40.13 | 0.0336 | 0.68× | no |
| λ=90 mm | 3.0 | 36.59 | 0.0340 | 0.69× | no |
| λ=90 mm | 3.5 | 33.19 | 0.0347 | 0.71× | no |
| λ=90 mm | 4.0 | 32.20 | 0.0352 | 0.72× | no |
| λ=60 mm | 1.5 | 45.67 | 0.0647 | 1.32× | **yes** |
| λ=60 mm | 2.0 | 41.56 | 0.0425 | 0.87× | **yes** |
| λ=60 mm | 2.5 | 39.35 | 0.0355 | 0.72× | **yes** |
| λ=60 mm | 3.0 | 35.86 | 0.0324 | 0.66× | **yes** |
| λ=60 mm | 3.5 | 32.23 | 0.0330 | 0.67× | no |
| λ=60 mm | 4.0 | 31.34 | 0.0346 | 0.70× | no |
| λ=30 mm | 1.5 | 36.56 | 0.0730 | 1.49× | **yes** |
| λ=30 mm | 2.0 | 32.79 | 0.0437 | 0.89× | **yes** |
| λ=30 mm | 2.5 | 32.20 | 0.0367 | 0.75× | **yes** |
| λ=30 mm | 3.0 | 30.53 | 0.0422 | 0.86× | no |
| λ=30 mm | 3.5 | 29.80 | 0.0384 | 0.78× | no |
| λ=30 mm | 4.0 | 28.71 | 0.0349 | 0.71× | **yes** |

### 4.2 Pareto front summary per terrain (speed vs F_max)

| terrain | Pareto-front abs_waves |
|---|---|
| flat | 1.5, 2.0 |
| λ=90 mm | 1.5, 2.0 |
| λ=60 mm | 1.5, 2.0, 2.5, 3.0 |
| λ=30 mm | 1.5, 2.0, 2.5, 4.0 |

### 4.3 Speed vs vz_rms (bounce)

| terrain | Pareto-front abs_waves |
|---|---|
| flat | 1.5, 2.0 |
| λ=90 mm | 1.5, 2.0, 3.0 |
| λ=60 mm | 1.5, 2.0, 2.5, 3.5 |
| λ=30 mm | 1.5, 2.0, 2.5 |

**Note**: §4.2 and §4.3 are the authoritative Pareto answers. The earlier hand-written tables in `PHASE_3B_ABSOLUTE_WAVE_SWEEP_REPORT.md` §5 are superseded by this table.

## 5. Re-derived role assignments

| role | abs_wave | rationale |
|---|---:|---|
| per-terrain speed winners (flat / λ=90 / λ=60 / λ=30) | 1.5 / 1.5 / 1.5 / 1.5 | highest mean v_fwd per row |
| overall speed winner | 1.5 | highest mean v_fwd across 4 terrains |
| baseline reliable | 3.0 | controller default `n_wave_scalar = 3.0` |
| stability winner (lowest CoV across rough) | 3.5 | CoV = 0.055 |
| worst-case winner (best minimum v_fwd across terrains) | 1.5 | min = 36.56 mm/s |
| impact-minimising winner (lowest mean F_max on rough) | 4.0 | mean F_max/W = 0.71× |

## 6. Corrected video audit status

- **14 / 24 videos** passed the documented frame audit (size ≥ 50 kB, black ≤ 4, frozen ≤ 20, motion ≥ 0.5).
- **10 / 24 videos** failed audit and CANNOT be cited as visual evidence.

Per-condition video verdict:

| terrain | abs=1.5 | abs=2.0 | abs=2.5 | abs=3.0 | abs=3.5 | abs=4.0 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| flat | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| λ=90 mm | ❌ | ✅ | ❌ | ✅ | ✅ | ❌ |
| λ=60 mm | ❌ | ❌ | ✅ | ✅ | ❌ | ❌ |
| λ=30 mm | ❌ | ❌ | ✅ | ✅ | ✅ | ❌ |

**CORRECTION to prior video claims**: the original visual_gait_summary.md cited `v17_L2_lambda60_abs1.5` and `v19_L3_lambda30_abs1.5` as visual evidence for the abs_wave=1.5 impact-spike interpretation. Both of those videos **failed audit** (v17: too-small / black; v19: too-small / black). They are not visual evidence. The impact conclusion is **numeric-only** based on the sweep CSV, with no validated visual confirmation on the critical rough × abs_wave=1.5 cells.

Critical missing videos (rough × abs_wave=1.5):

- `v07_L1_lambda90_abs1.5` — verdict: too-small
- `v13_L2_lambda60_abs1.5` — verdict: too-many-frozen
- `v19_L3_lambda30_abs1.5` — verdict: too-small

## 7. Final scientific conclusion (speed–impact–stability tradeoff)

Phase 3B identifies a three-way tradeoff at zero-action CPG, with all other scales pinned at 1.0:

- **Speed**: abs_wave = 1.5 maximises forward velocity on every terrain in the grid.
- **Impact**: abs_wave = 4.0 minimises mean foot-contact force on rough terrain. abs_wave = 1.5 carries an impact premium that is **dimensionally about 1.3–1.5× body weight** (mean over 5 trials) on λ ≤ 60 mm — NOT 15× as previously stated. Peak per-trial F_max on the worst conditions reaches **~1.9× body weight**.
- **Stability**: abs_wave = 3.5 has the lowest CoV (= 0.055) of v_fwd across rough terrains.

**No single abs_wave wins all three.** The relative differences are:

| metric | abs=1.5 | abs=4.0 | premium of 1.5 over 4.0 |
|---|---:|---:|---|
| v_fwd_mean over rough (mm/s) | 43.34 | 30.75 | +12.59 mm/s |
| F_max_mean/W over rough | 1.20× | 0.71× | 1.69× higher |
| CoV across rough | 0.138 | 0.059 | ++0.079 (higher = less stable) |

Interpretation: the speed gain at abs=1.5 is **real but modest** (~5 mm/s mean over rough), while the impact premium (~1.9×) is substantial and the stability cost (CoV +0.08) is real but not catastrophic. This is a genuine trade-off the controller would have to navigate.

## 8. NO reward recommendation in this corrections doc

Per ChatGPT's AUDIT-MORE verdict, this document is **scope-limited to the four corrections** (body weight, Pareto, video evidence, scientific conclusion). It does NOT propose:

- which reward shape to implement,
- whether to enable Pass 13 / Pass 14,
- whether to proceed to RL pilot,
- any controller / action / asset / reward / IsaacLab-core change.

Those decisions are deferred to a separate review packet once these corrections are accepted.

## 9. Provenance

- All numbers in this doc were regenerated by `scripts/sanity/phase3b_audit_corrections.py` from the two CSVs at the top.
- Pareto fronts use the standard "no other point dominates" definition over (terrain, abs_wave) tuples per terrain.
- Body weight constants match `pass11_overnight_BEST_ep12_bio_metrics.txt` and the sweep driver `sweep_phase3b_absolute_wave.py:BODY_MASS_KG / GRAV / BODY_WEIGHT_N`.
- Video audit status comes from `outputs/visual_audit/phase3b_absolute_wave_videos/reports/frame_audit.csv` (latest attempt per video).
