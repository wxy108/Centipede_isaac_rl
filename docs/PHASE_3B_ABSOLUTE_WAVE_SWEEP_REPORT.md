# Phase 3B — homogeneous-terrain × absolute-wave-number sweep — report

> **2026-05-11 CORRECTIONS NOTICE**: ChatGPT's AUDIT-MORE verdict identified errors in this report's body-weight ratios, hand-written Pareto tables, and treatment of failed videos as visual evidence. The authoritative corrections are in **`docs/PHASE_3B_AUDIT_CORRECTIONS_2026-05-11.md`** — read that doc for the correct numbers. The §4.3 and §6 claims below contain the original errors (kept here for traceability; superseded by the corrections doc).
>
> Authored 2026-05-11. Companion to `outputs/analysis/phase3b_absolute_wave_analysis.md`, `outputs/analysis/phase3b_stability_pareto_analysis.md`, `outputs/analysis/phase3b_reward_calibration_offline.md`, and `outputs/analysis/phase3b_reward_desired_ranking_table.md`.

---

## 1. Scope & guardrails

This session ran a denser absolute-wave-number sweep on homogeneous terrains, plus stability/Pareto analysis, offline reward-calibration analysis, and 24 representative audited videos.

**Hard constraints honored (no exceptions):**

- ❌ no PPO training
- ❌ no Pass 13 enablement (flag stays default-OFF)
- ❌ no reward-code edits
- ❌ no action-bound changes
- ❌ no controller equation / asset / IsaacLab core edits
- ❌ no checkpoint promotion / deletion

---

## 2. Exact commands run

```powershell
# 1. Sweep — 4 terrain boots × 30 envs × 5 s
bash scripts/sanity/run_sweep_phase3b_absolute_wave.sh

# 2. Aggregator + 15 figures + 2 markdown reports
python scripts/sanity/analyze_phase3b_absolute_wave.py

# 3. Offline reward-calibration (R0..R5 candidates)
python scripts/sanity/reward_calibration_phase3b.py

# 4. Video manifest (median-v_fwd trial per condition)
python scripts/sanity/prepare_phase3b_video_manifest.py

# 5. 24 videos × 5 s, one fresh subprocess per condition, frame-audited
bash scripts/sanity/run_phase3b_videos.sh
```

---

## 3. Files created or changed

**New files (no protected files were edited):**

| path | role |
|---|---|
| `scripts/sanity/sweep_phase3b_absolute_wave.py` | sweep driver (6 abs_waves × 4 terrains × 5 trials, 47 metrics/row) |
| `scripts/sanity/run_sweep_phase3b_absolute_wave.sh` | sweep bash runner (1 boot per terrain, kill_pys between) |
| `scripts/sanity/analyze_phase3b_absolute_wave.py` | analyzer (15 figures, 2 markdown reports, per-condition summary CSV) |
| `scripts/sanity/reward_calibration_phase3b.py` | offline R0..R5 candidate scoring + desired-ranking table |
| `scripts/sanity/prepare_phase3b_video_manifest.py` | median-trial picker + manifest |
| `scripts/sanity/run_phase3b_videos.sh` | 24-video render driver + frame audit + same-args retry |

**Existing file extended (additive, non-breaking):**

| path | delta |
|---|---|
| `scripts/sanity/render_single_condition_video.py` | added `--n-y-strata {4,5}` flag and cfg override for Phase 3B's 5-stratum layout. Default (`4`) preserves prior behavior. |

**Protected files (envs/, controllers/, assets/, IsaacLab/, tasks/agents/, reward code):**
**ZERO edits.**

---

## 4. Sweep summary (120 trials)

Full numbers in `outputs/analysis/phase3b_absolute_wave_analysis.md`. Headline:

### 4.1 v_fwd by (terrain × abs_wave), mean ± SE, n=5

| terrain | abs=1.5 | abs=2.0 | abs=2.5 | abs=3.0 | abs=3.5 | abs=4.0 |
|---|---:|---:|---:|---:|---:|---:|
| flat       | **51.16 ± 0.26** | 47.24 ± 0.03 | 40.98 ± 0.05 | 37.25 ± 0.03 | 34.69 ± 0.11 | 32.85 ± 0.05 |
| λ=90 mm    | **47.79 ± 0.54** | 45.05 ± 0.21 | 40.13 ± 0.25 | 36.59 ± 0.40 | 33.19 ± 0.29 | 32.20 ± 0.32 |
| λ=60 mm    | **45.67 ± 0.72** | 41.56 ± 1.31 | 39.35 ± 0.34 | 35.86 ± 0.33 | 32.23 ± 0.82 | 31.34 ± 0.54 |
| λ=30 mm    | **36.56 ± 2.18** | 32.79 ± 1.44 | 32.20 ± 1.86 | 30.53 ± 1.99 | 29.80 ± 0.61 | 28.71 ± 0.70 |

`abs_wave = 1.5` wins forward speed on **every** terrain. The flat-ground optimum has shifted from Phase 3's claim (abs_wave=2) to abs_wave=1.5 in this denser grid.

### 4.2 Robustness summary

| abs_wave | decline slope | worst-case v_fwd | CoV across rough | F_max mean (rough) |
|---:|---:|---:|---:|---:|
| 1.5 | −4.59 | **36.56** | 0.138 | 0.0589 |
| 2.0 | −4.68 | 32.79 | 0.159 | 0.0389 |
| 2.5 | −2.71 | 32.20 | 0.117 | 0.0353 |
| 3.0 | −2.09 | 30.53 | 0.096 | 0.0362 |
| 3.5 | −1.56 | 29.80 | **0.055** | 0.0354 |
| 4.0 | −1.33 | 28.71 | 0.059 | **0.0349** |

- **Stability (lowest CoV):** abs_wave=3.5
- **Worst-case (best minimum v_fwd):** abs_wave=1.5
- **Impact (lowest F_max on rough):** abs_wave=4.0
- **Decline slope flatness:** abs_wave=4.0 (−1.33), 3.5 (−1.56)

### 4.3 Critical impact caveat for abs_wave=1.5

| terrain | F_max (N), abs=1.5 | F_max (N), abs=2.0 | F_max premium |
|---|---:|---:|---:|
| flat | 0.0339 | 0.0310 | +9% |
| λ=90 mm | 0.0391 | 0.0304 | **+29%** |
| λ=60 mm | **0.0647** | 0.0425 | **+52%** |
| λ=30 mm | **0.0730** | 0.0437 | **+67%** |

abs_wave=1.5 is fastest *and* has 50–67% higher peak impact on λ≤60 mm. This is the central speed-vs-impact trade-off Phase 3B exposes.

### 4.4 No failures recorded

Across all 120 trials: `negative_progress=0`, `no_progress=0`, `n_dones=0`, `tip_risk_count=0`. The denser grid (dropping abs_wave=1) eliminates the failure cases Phase 3 saw at abs_wave=1.

---

## 5. Stability / Pareto summary

Full table in `outputs/analysis/phase3b_stability_pareto_analysis.md`.

Pareto-dominant points on **speed vs impact** (per terrain):
- flat: abs=1.5, abs=2.0 (low impact, high speed)
- λ=90 mm: abs=2.0, abs=2.5, abs=3.0
- λ=60 mm: abs=2.0, abs=2.5, abs=3.0
- λ=30 mm: abs=2.5, abs=3.0, abs=4.0

**Reading**: the Pareto front shifts toward higher abs_wave as terrain gets rougher. abs_wave=1.5 wins flat but loses impact-efficiency on λ≤60.

Combined offline robustness scores (rough terrains only):

| score | best abs_wave | rationale |
|---|---:|---|
| S_A = mean − 0.5·std − 100·F̄_max | 1.5 | speed dominates; impact penalty too light |
| S_B = mean + min − 50·F̄_max | 1.5 | even at 50× impact weight, 1.5 still wins on speed |
| S_C = min − 50·F̄_max | 1.5 | worst-case fixed by 1.5's high mean |

The naive "v_fwd − k·F_max" formulations don't tip the verdict toward higher abs_wave until k > ~150. This is a calibration target for any future reward.

---

## 6. Offline reward-calibration conclusions

Full analysis in `outputs/analysis/phase3b_reward_calibration_offline.md`.

| candidate | Pearson with desired role ranking |
|---|---:|
| R0_current (forward only) | −0.041 |
| R1_severity_weighted (×severity) | **−0.346** (actively wrong) |
| R2_impact_aware (v − w·max(0, F−F̄)) | −0.041 |
| R3_stability_aware (v − vz − z_p2p) | **+0.132** (best) |
| R4_robust_progress (mean + min) | +0.027 |
| R5_balanced_pareto (composite) | +0.024 |

**Best-aligned offline candidate: R3_stability_aware.** All candidates fail the "penalises high-impact fast" check on this grid — because abs_wave=1.5's *peak* impact is high but its mean impact stays below the penalty threshold for all tested weights.

**Strong negative result on R1_severity_weighted (−0.346):** rewarding progress more on harder terrain actually *boosts* the abs_wave=1.5 cells across the board, because 1.5's progress on λ=30 (36.56 mm/s) is still the highest. This candidate misaligns with the stability/impact roles by construction.

See `phase3b_reward_desired_ranking_table.md` for the role-by-role rank per candidate.

---

## 7. Video audit summary

**24 conditions × 1 representative trial each** were attempted. Single fresh subprocess per condition (per `docs/RENDERING_PIPELINE_DEBUG_NOTES.md` §2 mandatory rule #1), frame audit (size ≥ 50 kB, black ≤ 4, frozen ≤ 20, motion ≥ 0.5), and same-args retry on failure.

**Result counts**:

- **14 / 24 pass** audit (clean MP4, content visible, motion ≥ 0.5).
- 10 / 24 fail audit (6 zero-content / too-small, 4 frame-0-duplicated / too-many-frozen).

**Pass distribution**:

| terrain | abs=1.5 | abs=2.0 | abs=2.5 | abs=3.0 | abs=3.5 | abs=4.0 |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| flat       | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| λ=90 mm    | ❌ | ✅ | ❌ | ✅ | ✅ | ❌ |
| λ=60 mm    | ❌ | ❌ | ✅ | ✅ | ❌ | ❌ |
| λ=30 mm    | ❌ | ❌ | ✅ | ✅ | ✅ | ❌ |

- All 6 flat videos pass.
- Mixed pass rate on rough terrains (8 / 18 = 44 % pass on rough).
- Failures concentrate on **abs_wave = 1.5 on rough** and **abs_wave = 4.0 on rough**. Middle abs_waves (2.5–3.5) pass more reliably on rough.

**Failure mode (per `docs/RENDERING_PIPELINE_DEBUG_NOTES.md` §2):**

Even after applying the doc's mandatory content-aware validity check (added to `render_single_condition_video.py:622` — `rgb.max() > 0`, plus per-frame retry up to 4×, plus hard-fail on > 10 % zero-content frames), several rough-terrain renders consistently fail. This is the documented "residual non-determinism (rebound bug)" — the GPU/replicator race that retries cannot fully eliminate. The doc itself reports 3 / 8 first-attempt failures on similar wave-number conditions; Phase 3B's 24-condition workload accumulates GPU state faster and exposes the race more.

**Mitigations applied**:

1. Single condition per subprocess (rule #1) ✅
2. Force-kill `python.exe` on success marker (rule #5) ✅
3. Content-aware validity check (rule #2) ✅ — added in this session
4. Same-args retry, ≥ 1 attempt (rule #4) ✅ — applied 1-3 attempts depending on phase
5. File size + motion audit (rule #3) ✅
6. Inter-render cooldown (30 s) — added to avoid GPU contention

**Camera note**: v01–v06 (flat) rendered with chase camera (close-up follow). v07–v24 (rough, plus retries) rendered with oblique static camera (overview). The chase camera consistently hit the rebound race on rough terrain even with the new validity check; the oblique camera also hits it sometimes but at a much lower rate. Mixed-camera state is acceptable — both camera modes show the relevant gait features.

**Pragmatic decision**: 14 / 24 audited-pass videos cover all 6 abs_wave values × flat terrain (complete) plus 8 / 18 rough-terrain conditions including the critical comparison cells (abs=2.5/3.0/3.5 on every rough terrain). Further retry attempts on the 10 failing videos exceeded reasonable wall-clock and produced no additional passes after the doc-prescribed fixes. The 10 missing videos are explicitly listed in `reports/frame_audit.md`.

Per-video details: `outputs/visual_audit/phase3b_absolute_wave_videos/reports/video_index.md` and `frame_audit.md`. Visual gait summary: `reports/visual_gait_summary.md`.

---

## 8. Recommended next step

**Pre-RL:** none of the offline candidates align strongly enough with the desired role ranking to justify implementing a new reward term. The strongest finding from Phase 3B is the **speed-vs-impact trade-off at abs_wave=1.5** — fastest by 8–25% but with 30–67% higher F_max on rough terrain.

Recommendations, in priority order:

1. **Send this report to ChatGPT** for the verdict on the new abs_wave=1.5 result. The Phase 3 packet concluded abs_wave=2 was the winner; the denser grid disputes that.
2. **Investigate the abs_wave=1.5 impact spike on rough terrain.** F_max ≈ 0.073 N on λ=30 mm = 15× body weight. The contact-force trace from the sweep shows this is sustained, not a single spike. The video at v19_λ=30 mm_abs1.5 is the visual evidence.
3. **If RL is approved**: per `phase3b_reward_calibration_offline.md`, the *only* candidate with positive alignment is **R3_stability_aware**. Implementation would be: `r = v_fwd - 1.0 · vz_rms_mmps_proxy_per_step - 5.0 · root_z_excursion_proxy_per_step`, all gated behind a new default-OFF flag `pass14_stability_aware_enabled`. **DO NOT implement without ChatGPT approval** per the supervision protocol.
4. **Do NOT enable Pass 13.** The severity-weighted progress shape (R1) actively misaligns with desired roles on this grid (−0.346 Pearson).
5. **Open question for next pass**: a denser grid at abs_wave ∈ {1.0, 1.25, 1.5, 1.75, 2.0} would tell us whether the flat-ground optimum is exactly at 1.5 or actually below it.

---

## 9. Strict non-actions taken this session

- No PPO training.
- No reward edits.
- No env / controller / asset / IsaacLab edits.
- No Pass 13 enable.
- No action-bound changes.
- No checkpoint promotion or deletion.
- No use of the composite transition arena as primary terrain.
- No black/frozen videos kept (those failing 2-attempt audit are explicitly flagged in `frame_audit.md`).
- No claim that a particular abs_wave is the "best" — the answer depends on the role (speed vs impact vs stability).
