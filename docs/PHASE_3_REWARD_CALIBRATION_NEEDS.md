# Phase 3 + 3B — reward-calibration needs

> **2026-05-11 CORRECTIONS NOTICE**: ChatGPT's AUDIT-MORE verdict identified an error in §1 of this doc (the "67% F_max premium" and "15× body weight" claims). The body-weight-normalised numbers are corrected in **`docs/PHASE_3B_AUDIT_CORRECTIONS_2026-05-11.md`** — the actual ratio is ~1.5× body weight (mean) or ~1.9× peak across trials, NOT 15×. The reward analysis itself (R0..R5) is unaffected because it operates on the per-condition CSV directly, not on the body-weight ratio text.
>
> Companion to `outputs/analysis/phase3b_reward_calibration_offline.md` and `docs/PHASE_3B_ABSOLUTE_WAVE_SWEEP_REPORT.md`.
> Authored 2026-05-11. **Analysis only. NO env reward edits. NO Pass 13 enable.**

---

## 1. What the sweeps showed

Phase 3 (abs_wave ∈ {1, 2, 3, 4}) and Phase 3B (denser {1.5..4.0}) together:

- **Speed**: abs_wave=1.5 wins on every terrain (51 mm/s flat → 36 mm/s λ=30 mm).
- **Impact**: abs_wave=1.5 has 30–67% higher F_max on rough terrain. abs_wave=4 has lowest impact.
- **Stability (CoV across terrains)**: abs_wave=3.5 (CoV 0.055) and 4.0 (CoV 0.059) are most consistent.
- **Worst-case**: abs_wave=1.5 still highest absolute min (36.56 mm/s on λ=30); abs_wave=4.0 highest *relative* min (28.71 mm/s with 87% retention).
- **Failure modes**: none observed in Phase 3B (all 120 trials had v_fwd > 25 mm/s and no `done` events). The denser grid eliminates the abs_wave=1 failure mode from Phase 3.

These results define a **3-dimensional trade-off**: speed × impact × stability. There is no single abs_wave that wins all three.

---

## 2. Why a single fixed wave number is insufficient

If the goal is the centipede locomotes on multiple terrains with high speed, low impact, and low variance:

- abs_wave=1.5 maximizes speed but pays 50%+ in impact on rough terrain.
- abs_wave=3.5/4.0 minimizes variance and impact but loses 30–40% of flat-ground speed.
- abs_wave=2.0–2.5 are middle compromises but don't decisively win any single criterion.

Future RL should learn to *adapt* abs_wave (or related controller scales) to terrain. The reward must express the multi-dimensional trade-off, not just speed.

---

## 3. What the env / sim currently logs (available without reward edits)

These are the metrics Phase 3B's sweep extracted from existing public env state. None of them require modifying reward code:

| metric | source | per-step or rollout? |
|---|---|---|
| v_fwd | `robot.data.root_pos_w[:, 0]` displacement | rollout |
| F_max, F_mean, F_p95, F_p99 | `scene["contact_sensor"].data.net_forces_w` | per-step trace → percentiles |
| impact_spike_count | F > 5× body weight threshold | per-step trace |
| vz_rms | `robot.data.root_lin_vel_w[:, 2]` | per-step trace → RMS |
| root_z_p2p | `robot.data.root_pos_w[:, 2]` | per-step trace → max−min |
| roll_rms, pitch_rms | `robot.data.projected_gravity_b` → atan2/asin | per-step trace → RMS |
| proj_g_z_min | `robot.data.projected_gravity_b[:, 2]` | per-step trace → min |
| tip_risk_count | proj_g_z > −0.85 | per-step trace |
| omega_b_rms, _p95 | `robot.data.root_ang_vel_b` magnitude | per-step trace |
| tau_abs, power, mech_work | `robot.data.applied_torque`, `joint_vel` | per-step trace |
| contact_n_mean | count of bodies with net_force > 1e-4 N | per-step trace |

**Conclusion**: a stability/impact-aware reward could be implemented **entirely from public env state**. No protected-file edits to `_get_rewards` would be required to feed it — only the reward-term arithmetic itself.

---

## 4. Candidate reward shapes evaluated offline

Full Pearson-vs-desired-ranking table in `phase3b_reward_calibration_offline.md`. Headline:

| candidate | formula (offline aggregate proxy) | Pearson | recommendation |
|---|---|---:|---|
| R0_current | v_fwd | −0.041 | already in code; insufficient alone |
| R1_severity_weighted | (1+2·s)·v_fwd | **−0.346** | **REJECT** — actively prefers high-impact 1.5 |
| R2_impact_aware | v_fwd − 100·max(0, F_p95 − 2W) | −0.041 | weak — threshold/weight insensitive on this grid |
| R3_stability_aware | v_fwd − vz_rms − 5·z_p2p | **+0.132** | **best of candidates evaluated** |
| R4_robust_progress | v_fwd + v_min − 50·F_max | +0.027 | weak |
| R5_balanced_pareto | composite | +0.024 | over-tuned; weights need recalibration |

---

## 5. What's missing from the offline analysis

These need to be addressed before any reward is implemented:

1. **Per-step vs per-rollout magnitudes.** Phase 3B aggregates over 5-second rollouts. The actual reward is per-step. A 0.01 N·m/step torque penalty integrated over 100 steps ≠ a 1.0 N·m·s rollout-level penalty. Future reward implementations must be calibrated against per-step magnitudes of the existing terms (`forward ≈ 0.025/step`, `alive ≈ 0.020/step`).
2. **Reward-shape interaction with PPO entropy/KL.** R3's `−vz_rms` term penalises bouncy gaits across the board, including legitimately fast flat-ground gaits. PPO with high entropy may not converge to a corner that minimizes vz_rms.
3. **Trial-distribution under RL.** Offline rankings use zero-action CPG rollouts. Under RL, the action distribution shifts — the same reward may rank conditions differently.
4. **Threshold sensitivity.** Phase 3B uses F_safe = 2× body weight, no_progress = 5 mm/s, tip_risk = −0.85. These are post-hoc choices. The reward should expose them as cfg knobs.

---

## 6. Recommendation for the next reward step (if any)

**Do nothing on the env reward in this session.** This document records what was learned. The decisions about whether to implement a new reward and which candidate to pick belong with ChatGPT per `docs/CHATGPT_SUPERVISION_PROTOCOL.md`.

If ChatGPT later approves a reward implementation:

1. **Start with R3_stability_aware** (best offline alignment, simplest formulation).
2. **Gate behind a new default-OFF flag** `pass14_stability_aware_enabled` (or equivalent — per supervision protocol §5).
3. **Calibrate per-step**: target `r_stability` magnitude ≈ −0.005 / step at baseline gait, so it's not larger than forward + alive (≈ +0.045 / step combined).
4. **Audit before training**:
   - flag=False reproduces Pass 8/9 exactly (G3)
   - finite components (G1), total=sum (G2)
   - zero-action policy gets r_stability ≈ 0 (G5)
   - Phase 3B's abs_wave=1.5 condition gets r_stability < r_stability(abs_wave=4.0) on rough terrain (the sign-of-effect check, G4)

5. **Run a Pass-14 smoke** (10 epochs, warmstart Pass 8 ckpt), check whether the new term drags the policy toward higher abs_wave on rough terrain.

---

## 7. What this document does NOT recommend

- **No new term that depends on body impedance.** Impedance pinned at baseline in Phase 3B; impedance design is a separate study.
- **No term that depends on `wave_number_scale` directly.** All reward terms should reference functional outcomes (progress, stability, impact), not the controller knob.
- **No term that uses block_idx lookups.** Phase 3B's homogeneous arenas have a single block; the per-block severity tuple in `_BLOCK_SEVERITY_S` is not the right severity definition for homogeneous arenas.
- **No re-enable of Pass 10 effort penalty.** Prior ablation showed it dominates the reward and degrades the trained policy.
- **No re-enable of Pass 11 windowed-CoT** without re-audit. Pass 11's overnight run achieved Pearson ≈ ? against the Phase 3B desired ranking — *not measured in this session*. Worth measuring next time.

---

## 8. Sign-off

- All numbers in this document and the linked `phase3b_*` files come from `outputs/analysis/phase3b_absolute_wave_sweep.csv` (120 rows). No simulation re-runs required to reproduce.
- No env / reward / controller / asset / IsaacLab file was edited.
- No checkpoint was promoted, deleted, or overwritten.
- No PPO training was run.
- This document is *proposal-only* per `docs/CHATGPT_SUPERVISION_PROTOCOL.md` §2.
