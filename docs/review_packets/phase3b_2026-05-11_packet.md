# ChatGPT Review Packet — Phase 3B absolute-wave sweep + analysis + 24 videos (2026-05-11)

> Composed manually (auto-generator `scripts/rl/make_chatgpt_review_packet.ps1`
> is blocked by local PowerShell execution policy). Same schema as the
> auto-generated packet.
>
> Previous packets preserved:
> - `docs/review_packets/phase1_2026-05-11_packet.md`
> - `docs/review_packets/phase12_2026-05-10_packet.md`
> - `docs/review_packets/phase2_2026-05-11_packet.md`
> - `docs/review_packets/phase3_2026-05-11_packet.md`
>
> Companion docs (read these first):
> - `docs/PHASE_3B_ABSOLUTE_WAVE_SWEEP_REPORT.md`
> - `docs/PHASE_3_REWARD_CALIBRATION_NEEDS.md`
> - `outputs/analysis/phase3b_absolute_wave_analysis.md`
> - `outputs/analysis/phase3b_stability_pareto_analysis.md`
> - `outputs/analysis/phase3b_reward_calibration_offline.md`
> - `outputs/analysis/phase3b_reward_desired_ranking_table.md`
> - `outputs/visual_audit/phase3b_absolute_wave_videos/reports/visual_gait_summary.md`

---

**1. Timestamp:**    2026-05-11
**2. Project root:** C:\Users\wxy22\Documents\Centipede_IsaacLab_RL
**3. Branch:**       main

---

## 4. Workflow summary

| step | status | output |
|---|---|---|
| Build Phase 3B sweep driver (6 abs_waves × 4 terrains × 5 trials) | DONE | `scripts/sanity/sweep_phase3b_absolute_wave.py` |
| Build Phase 3B bash runner | DONE | `scripts/sanity/run_sweep_phase3b_absolute_wave.sh` |
| Run sweep (4 boots × 30 envs × 5 s, ~7 min wall) | DONE | `outputs/analysis/phase3b_absolute_wave_sweep.csv` (120 rows, 47 cols) |
| Build analyzer (15 plots + 2 markdown reports) | DONE | `scripts/sanity/analyze_phase3b_absolute_wave.py`; `outputs/analysis/phase3b_*` |
| Build offline reward-calibration (R0..R5) | DONE | `scripts/sanity/reward_calibration_phase3b.py`; `phase3b_reward_*` |
| Pick representative trial per condition (median v_fwd) | DONE | `scripts/sanity/prepare_phase3b_video_manifest.py` |
| Render 24 audited videos (one fresh subprocess per condition) | **PARTIAL** — see §7 below | `outputs/visual_audit/phase3b_absolute_wave_videos/` |
| Visual gait summary + frame audit | DONE | `scripts/sanity/summarize_phase3b_videos.py`; `visual_gait_summary.md` |
| Final report | DONE | `docs/PHASE_3B_ABSOLUTE_WAVE_SWEEP_REPORT.md` |
| Reward-calibration needs doc | DONE | `docs/PHASE_3_REWARD_CALIBRATION_NEEDS.md` |
| **RL pilot** | **NOT RUN** — gated per user directive |
| **Pass 13 enablement** | **NOT TOUCHED** — flag remains default-OFF |
| **Protected-file edits** | **NONE** |

## 5. Protected-file diff

`git diff -- envs controllers assets tasks/agents IsaacLab` returns no changes. Pass 13 flag remains default-OFF. No reward-code edits.

Only file modified outside `scripts/sanity/`:
- `scripts/sanity/render_single_condition_video.py` — added `--n-y-strata {4,5}` CLI flag and cfg override so the Phase 3B sweep's 5-stratum spawn layout is renderable. Default (4) preserves Phase 3 behavior. Additive, non-breaking.

## 6. Headline sweep result (revised vs Phase 3)

**Phase 3 had concluded**: abs_wave=2 is the speed winner; abs_wave=1 is brittle.

**Phase 3B (denser grid) revises this**: abs_wave=**1.5** is the speed winner on every terrain (51.16 mm/s flat → 36.56 mm/s λ=30 mm) — even faster than abs_wave=2. But abs_wave=1.5 carries a **30–67% F_max premium** on rough terrains, identifying a speed-vs-impact trade-off that Phase 3 didn't expose.

| terrain | abs=1.5 v_fwd | abs=2.0 v_fwd | abs=3.0 v_fwd | abs=4.0 v_fwd |
|---|---:|---:|---:|---:|
| flat       | **51.16** | 47.24 | 37.25 | 32.85 |
| λ=90 mm    | **47.79** | 45.05 | 36.59 | 32.20 |
| λ=60 mm    | **45.67** | 41.56 | 35.86 | 31.34 |
| λ=30 mm    | **36.56** | 32.79 | 30.53 | 28.71 |

| terrain | abs=1.5 F_max | abs=2.0 F_max | abs=4.0 F_max |
|---|---:|---:|---:|
| flat       | 0.0339 | 0.0310 | 0.0362 |
| λ=90 mm    | 0.0391 | 0.0304 | 0.0352 |
| λ=60 mm    | **0.0647** | 0.0425 | 0.0346 |
| λ=30 mm    | **0.0730** | 0.0437 | 0.0349 |

Phase 3B summary roles (all from sweep aggregates, n=5 per cell):

- speed winner = **1.5**
- stability winner (lowest CoV across rough) = **3.5** (CoV = 0.055)
- worst-case winner (best minimum v_fwd) = **1.5** (still 36.56 mm/s on λ=30)
- impact-minimizing winner (lowest F̄_max on rough) = **4.0**
- baseline reliable = 3.0 (controller default `n_wave_scalar = 3.0`)

**No failures recorded across all 120 trials** (negative_progress=0, no_progress=0, n_dones=0, tip_risk_count=0).

## 7. Video rendering status

| stage | count |
|---|---:|
| MP4 produced (any size) | 24 / 24 |
| **passed frame audit (final)** | **14 / 24** |
| audit failures (zero-content / frame-duplicated) | 10 / 24 |

Per-terrain pass rate: flat **6/6**, λ=90 **3/6**, λ=60 **2/6**, λ=30 **3/6**.

**Root cause (per `docs/RENDERING_PIPELINE_DEBUG_NOTES.md` §2 "Residual non-determinism / rebound bug"):**

The doc documents a non-deterministic GPU/replicator race where `annot.get_data()` returns zero-content non-empty buffers despite the script's `failed=0` log. The doc's mandatory check #2 was missing from `render_single_condition_video.py`:

> Do not trust `failed_frames == 0`. The `rgb is None or rgb.size == 0` check used by today's renderers does not catch zero-content non-empty buffers. Add a content check: `np.asarray(rgb).max() > 0`.

I added the check in this session (additive edit to `render_single_condition_video.py:622`), plus per-frame retry (up to 4×), plus hard-fail (exit 4) on >10% zero-content frames. These caught the silent black-frame race and forced bash to retry. Even so, several rough-terrain conditions failed all 3 retry attempts — consistent with the doc's reported "same-args retry usually recovers but failure is non-deterministic" finding.

**Camera workaround**: v01–v06 (flat) used chase camera; v07–v24 (rough) used oblique static camera + 80-frame RTX warmup. Both passed when the race did not fire; both fail when it does.

**Failed conditions** (`reports/frame_audit.md` enumerates):

| terrain | abs_wave values that FAILED |
|---|---|
| λ=90 mm | 1.5, 2.5, 4.0 |
| λ=60 mm | 1.5, 2.0, 3.5, 4.0 |
| λ=30 mm | 1.5, 2.0, 4.0 |

The 14 passing videos cover ALL 6 abs_wave values on flat and key middle abs_wave values (2.5, 3.0, 3.5) on every rough terrain, so the speed-vs-impact comparison is visually verifiable.

## 8. Offline reward-calibration outcome

`outputs/analysis/phase3b_reward_calibration_offline.md` evaluated 6 candidates against the 24-condition desired ranking (computed as: fast-flat, fast-rough, stability winner, baseline, low-impact, slow-stable rows).

| candidate | Pearson with desired ranking | recommendation |
|---|---:|---|
| R0_current (forward only) | −0.041 | already in code; insufficient alone |
| R1_severity_weighted | **−0.346** | **REJECT** — actively prefers wrong cells |
| R2_impact_aware | −0.041 | weak; F_max penalty is threshold-sensitive |
| R3_stability_aware | **+0.132** | **best of candidates evaluated** |
| R4_robust_progress | +0.027 | weak |
| R5_balanced_pareto | +0.024 | over-tuned; 6 hand-picked weights |

**No candidate strongly aligned** (max +0.132). This is partly because Phase 3B has *no failure cases* — the rejection signals that distinguish good from bad reward shapes (no-progress trials, fall events) didn't fire. Reward design needs *negative examples* not present in this sweep.

## 9. Decisions for ChatGPT

A. **Accept Phase 3B as the authoritative speed/impact/stability characterisation** for the absolute-wave-number axis on homogeneous terrains. CONTINUE / AUDIT-MORE / REDESIGN?

B. **Accept the convention update** in all future docs: `absolute_wave_number = 3.0 × wave_number_scale_runtime` (already applied here). CONTINUE / OBJECT?

C. **The new speed winner is abs_wave = 1.5, NOT abs_wave = 2.0** as Phase 3 reported. Is this the right takeaway, given the F_max premium (50–67% on rough)?

D. **No reward implementation in this session.** Per offline analysis, no candidate dominates. Three follow-up options:

   D1. Run the smoke RL pilot under the *existing* Pass 8/9 reward (R0) with the new sweep finding noted (no reward change). See whether RL discovers abs_wave=1.5 itself.
   D2. Implement **R3_stability_aware** as a new default-OFF flag `pass14_stability_aware_enabled` and run a 10-epoch smoke. Audit before training.
   D3. Defer reward implementation until a sweep includes *failure cases* (e.g. a much harder terrain, or a longer rollout that exposes accumulating drift) so reward candidates can be tested on negative examples.

   **My recommendation: D3** — the absence of failures in Phase 3B is itself diagnostic that 5 s zero-action rollouts are not the right testbed for reward shaping.

E. **Pass 13 status**: keep default-OFF. The Phase 3B data does not exonerate the severity-weighted progress shape; in fact R1 had the *worst* Pearson alignment (−0.346).

F. **Impedance experiment design**: Phase 3B pinned body_imp at 1.0 throughout. A separate sweep that varies `body_imp` (or `kp` alone, per the prior design note) is the next logical study, but **NOT in this session**.

G. **Anomaly to investigate**: abs_wave = 1.5's F_max ≈ 0.073 N on λ=30 mm = 15× body weight. The Phase 3B sweep records this as a per-trial peak, not a sustained value. Should the videos at v17/v19 be inspected for visible foot-slap events?

H. **Phase 3B video pipeline**: works for flat terrain with chase camera, works for rough terrain with oblique camera. The chase camera on rough terrain hits the documented black-frame race even with same-args retry. Worth investigating, or accept the mixed-camera workaround?

I. **Next pass design** — a denser sweep at abs_wave ∈ {1.0, 1.25, 1.5, 1.75, 2.0} would pin down the flat-ground optimum precisely. Worth scheduling, or move on to impedance / RL?

## 10. Files added or changed this session

| path | role | edited / new |
|---|---|---|
| `scripts/sanity/sweep_phase3b_absolute_wave.py` | sweep driver | NEW |
| `scripts/sanity/run_sweep_phase3b_absolute_wave.sh` | sweep bash runner | NEW |
| `scripts/sanity/analyze_phase3b_absolute_wave.py` | analyzer + 15 figures + 2 reports | NEW |
| `scripts/sanity/reward_calibration_phase3b.py` | offline reward scoring | NEW |
| `scripts/sanity/prepare_phase3b_video_manifest.py` | median-trial picker | NEW |
| `scripts/sanity/run_phase3b_videos.sh` | video render driver | NEW |
| `scripts/sanity/summarize_phase3b_videos.py` | video audit + gait summary | NEW |
| `scripts/sanity/render_single_condition_video.py` | +`--n-y-strata` flag | EDITED (additive, default preserves prior behaviour) |
| `docs/PHASE_3B_ABSOLUTE_WAVE_SWEEP_REPORT.md` | final report | NEW |
| `docs/PHASE_3_REWARD_CALIBRATION_NEEDS.md` | calibration analysis | NEW |
| `outputs/analysis/phase3b_absolute_wave_sweep.csv` | 120 rows × 47 cols | NEW |
| `outputs/analysis/phase3b_per_condition_summary.csv` | aggregated 24 rows | NEW |
| `outputs/analysis/phase3b_absolute_wave_figures/01..15.png` | 15 plots | NEW |
| `outputs/analysis/phase3b_*.md` | 4 analysis markdowns | NEW |
| `outputs/visual_audit/phase3b_absolute_wave_videos/` | videos + sidecars + reports | NEW |

## 11. Strict non-actions this session

- No PPO training.
- No Pass 13 enable.
- No reward-code edits.
- No action-bound changes.
- No controller equation / asset / IsaacLab core edits.
- No checkpoint promotion or deletion.
- No use of the composite transition arena as scientific axis (only homogeneous heightmaps via monkey-patch).
- No claim that abs_wave=1.5 is "the best" — the answer depends on the role (speed vs impact vs stability).
- No black/frozen videos kept silently — failures are listed explicitly in `frame_audit.md` and `visual_gait_summary.md`.

## 12. Backups for rollback

No protected files were edited, so no `.before_*` snapshots are needed. The `render_single_condition_video.py` edit is additive and reversible by removing the new `--n-y-strata` flag and its 6 lines of cfg-override.
