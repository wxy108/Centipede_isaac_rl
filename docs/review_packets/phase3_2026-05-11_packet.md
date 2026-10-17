# ChatGPT Review Packet — Phase 3 homogeneous-terrain × absolute-wave-number sweep (2026-05-11)

> Composed manually (auto-generator `scripts/rl/make_chatgpt_review_packet.ps1`
> is blocked by local PowerShell execution policy). Same schema as the
> auto-generated packet.
>
> Previous packets preserved:
> - `docs/review_packets/phase1_2026-05-11_packet.md`
> - `docs/review_packets/phase12_2026-05-10_packet.md`
> - `docs/review_packets/phase2_2026-05-11_packet.md`
>
> Companion docs:
> - `docs/PHASE_3_ABSOLUTE_WAVE_HOMOGENEOUS_SWEEP_REPORT.md`
> - `docs/RENDERING_PIPELINE_DEBUG_NOTES.md`
> - `docs/PHASE_3_HOMOGENEOUS_TERRAIN_ROBUSTNESS_SWEEP_DESIGN.md` (prior design)
> - `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_analysis.md`
> - `outputs/analysis/phase3_absolute_wave_homogeneous_reward_calibration.md`

---

**1. Timestamp:**    2026-05-11
**2. Project root:** C:\Users\wxy22\Documents\Centipede_IsaacLab_RL
**3. Branch:**       main

---

## 4. Workflow summary

| step | status | output |
|---|---|---|
| Document the rendering-fix experience | DONE | `docs/RENDERING_PIPELINE_DEBUG_NOTES.md` |
| Audit env terrain wiring | DONE | env passes only `rough_*` scalars to the arena fn — confirmed I cannot enable homogeneous terrain through cfg alone |
| Build homogeneous-terrain sweep driver | DONE | `scripts/sanity/sweep_phase3_absolute_wave.py` + `run_sweep_phase3_absolute_wave.sh` — homogeneous arena via monkey-patch of `arena.generate_flat_to_rough_transition_heightmap`; matched-spawn via `MatchedSpawnRng`; per-env absolute_wave assignment |
| Run sweep (4 boots × 20 envs × 5 s rollout) | DONE | `outputs/analysis/phase3_absolute_wave_homogeneous_sweep.csv` (80 rows) |
| Analysis + 8 plots | DONE | `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_analysis.md`, `figures/01..08.png` |
| Reward-calibration proposal (no implementation) | DONE | `outputs/analysis/phase3_absolute_wave_homogeneous_reward_calibration.md` |
| **RL pilot** | **NOT RUN** — gated per the user's directive |
| **Pass 13 enablement** | **NOT TOUCHED** — flag remains default-OFF |
| **Protected-file edits** | **NONE** |

## 5. Convention correction (critical, applies retroactively to Phase 2)

The prior Phase 2 reports wrote `wave_num` columns that were really the
**runtime scale** applied to the controller's `n_wave_scalar = 3.0`
baseline. Therefore the Phase 2 "wave_num=2.0" condition was actually
**6 traveling waves** along the body, NOT 2. Phase 3 explicitly uses
`absolute_wave_number = 3.0 × wave_number_scale_runtime` on all axes and
legends. The CSV exposes both columns side-by-side.

## 6. Headline result

`figures/01_v_fwd_vs_terrain.png` (and `analysis.md` § 1):

| abs_wave | flat | λ=90 | λ=60 | λ=30 | decline slope | CoV across rough |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 44.66 ± 0.14 | 33.37 ± 5.77 | **16.45 ± 6.36** | 29.26 ± 2.52 | **−6.31** | **0.335** |
| 2 | **47.24 ± 0.03** | **45.05 ± 0.21** | **41.56 ± 1.31** | **32.79 ± 1.44** | −4.68 | 0.159 |
| 3 | 37.25 ± 0.03 | 36.59 ± 0.40 | 35.86 ± 0.33 | 30.53 ± 1.99 | −2.09 | 0.096 |
| 4 | 32.85 ± 0.05 | 32.20 ± 0.32 | 31.34 ± 0.54 | 28.71 ± 0.70 | **−1.33** | **0.059** |

Findings:

- **abs_wave = 1 is brittle on rough terrain.** Mean drops to 16 mm/s on
  λ=60 mm with one trial running **backwards** (−3.94 mm/s).
- **abs_wave = 2 is the speed winner across all terrains** AND has good
  retention. Beats abs_wave=1 by +25.11 mm/s in paired comparison on
  λ=60 (`analysis.md` § 4).
- **abs_wave = 3 (controller baseline) is moderate, very retentive.**
  98 % retention on λ=90 and λ=60.
- **abs_wave = 4 is the slowest on flat but the most consistent** —
  flattest decline slope (−1.33), lowest CoV (0.059), highest
  worst-case (28.71 mm/s).
- The flat-ground optimum is **abs_wave ≈ 2**, NOT the controller
  baseline of 3. Worth flagging to the controller designer.

## 7. Paired-trial Δv (matched spawn-y across 4 abs_wave conditions)

| terrain | Δv(abs=2 − 1) | Δv(abs=3 − 1) | Δv(abs=4 − 1) |
|---|---:|---:|---:|
| flat       | +2.58 | −7.41 | −11.81 |
| λ=90 mm    | **+11.68** | +3.22 | −1.17 |
| λ=60 mm    | **+25.11** | **+19.41** | **+14.89** |
| λ=30 mm    | +3.53 | +1.27 | −0.56 |

The largest matched-paired wins are on λ=60. On λ=30 the spread
collapses — at the roughest wavelength every abs_wave value scores
similarly.

## 8. Implementation details for review

### Homogeneous arena enabled without protected-file edits

`scripts/sanity/sweep_phase3_absolute_wave.py::install_homogeneous_arena_patch`
replaces `arena.generate_flat_to_rough_transition_heightmap` (the
function the env imports and calls at line 393-405 of `envs/centipede_env.py`)
with a wrapper that calls `terrain.generate_single_wavelength_heightmap`
instead. The env's USD-mesh build is unchanged — it receives a fresh
homogeneous heightmap on every boot. **Pure monkey-patch; no env
source edit.**

This works because the env's `_build_transition_terrain` (around
line 388-425) reads only `arena_size_m`, `rough_wavelength_mm`,
`rough_amplitude_m`, `seed`, `image_size` from cfg and passes them to
the arena fn. The per-block (λ, amp) tuples are baked into the arena
fn's signature defaults — not cfg-driven — so the only override path
was the monkey-patch.

### Matched-spawn cycling

`MatchedSpawnRng` (`sweep_phase3_absolute_wave.py`) deterministically
maps env_i → (block_idx=2, y_strat_idx = i % 5). After the initial
spawn-RNG bug fix (the `uniform` method also needed to increment the
call counter so block_idx didn't desync), all 20 envs spawn at
`x=−0.183` with `y ∈ {−0.7, −0.35, 0, +0.35, +0.7}` based on
trial_id. Same trial_id across the 4 abs_wave cells → same xyz spawn.

### What was NOT done

- No env / controller / asset / IsaacLab / tasks/agents edits.
- No reward modifications. The Pass 13 default-OFF flag is unchanged.
- No `terrain_kind="homogeneous"` cfg/env branch implemented (the
  earlier design doc § 4.3 still proposes this as the eventually
  clean approach; the current sweep used monkey-patch instead).

## 9. Decisions for ChatGPT

A. **Accept the convention correction** and rename `wave_num` →
   `absolute_wave_number` / `wave_number_scale` in any future
   docs / reports / plots. **CONTINUE / REJECT?**

B. **Accept Phase 3 as scientifically clean evidence** for the
   robustness trend: higher abs_wave → flatter velocity decline,
   lower CoV across terrains; trade-off is reduced flat-ground
   speed. **CONTINUE / AUDIT-MORE / REDESIGN?**

C. **Approve the homogeneous-terrain cfg/env addition** (per the
   earlier design doc § 4.3) so future runs do not require a
   monkey-patch? Diff ≤ 80 lines, default-OFF, snapshot-backed.

D. **Reward-design verdict**: per
   `reward_calibration.md` § 4-5, recommendation is **candidate F
   (no new reward term) for the first homogeneous-arena RL pilot**.
   Fall back to candidate A (severity × v_forward) only if F fails.
   **APPROVE / REDESIGN?**

E. **RL pilot scope**: if approved, would test a single-dimension
   wave_number_scale action only (other 5 scales pinned). PPO config:
   `tasks/agents/rl_games_stage_b_smoke_pass12_B4_RECOMMENDED.yaml`.
   Warmstart from Pass 8 best smoke ckpt. **APPROVE A SMOKE PILOT
   (e.g. 10 epochs) BEFORE ANY OVERNIGHT TRAINING?**

F. **Pass 13 status**: keep default-OFF as is, OR roll back via
   `.before_pass13_2026-05-11` snapshots. The Phase 3 evidence makes
   Pass 13's severity × v_forward reward shape *more plausible* than
   the Phase 2 spawn-biased data suggested. Recommendation: **keep
   default-OFF for now**; reconsider only after the no-reward pilot.

G. **Anomaly: abs_wave = 2 is fastest on flat (47.24 mm/s) vs
   abs_wave = 3 baseline (37.25 mm/s).** This says the controller's
   `n_wave_scalar = 3.0` is not the flat-ground optimum. Should this
   be re-tuned, or left alone because the project is moving to RL?

H. **Optional priority videos** of {flat, λ=30} × {1, 2, 3, 4} are
   not rendered yet. Worth the GPU time?

I. Anything I've missed.

## 10. Files added this session

| path | role |
|---|---|
| `docs/RENDERING_PIPELINE_DEBUG_NOTES.md` | rendering-fix authoritative reference |
| `docs/PHASE_3_ABSOLUTE_WAVE_HOMOGENEOUS_SWEEP_REPORT.md` | this experiment's final report |
| `scripts/sanity/sweep_phase3_absolute_wave.py` | sweep driver |
| `scripts/sanity/run_sweep_phase3_absolute_wave.sh` | shell driver |
| `scripts/sanity/analyze_phase3_absolute_wave.py` | numpy analyzer + 8 plots |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep.csv` | 80-row data |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_meta/*.json` | per-boot metadata |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_figures/01..08.png` | 8 plots |
| `outputs/analysis/phase3_absolute_wave_homogeneous_sweep_analysis.md` | aggregate analysis |
| `outputs/analysis/phase3_absolute_wave_homogeneous_reward_calibration.md` | reward proposal (no implementation) |

## 11. Strict non-actions this session

- No PPO training; no Pass 13 enable; no reward edits.
- No action-bound changes; no IsaacLab/asset/protected-file edits.
- No checkpoint promotion / deletion / overwrite.
- No use of the composite transition arena as scientific evidence —
  all 4 terrains in this sweep are homogeneous single-λ.
- No claim that a particular abs_wave is the "best" — the answer is
  terrain-dependent (see § 5 / § 6).

## 12. Backups in case of rollback

No protected-file edits were made, so no new `.before_*` snapshots
are needed. The Pass 13 backups from prior sessions remain in place
(`envs/*.py.before_pass13_2026-05-11`,
`scripts/rl/train_stage_b_smoke.py.before_pass13_2026-05-11`).
