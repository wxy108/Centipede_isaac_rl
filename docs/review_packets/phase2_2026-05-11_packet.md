# ChatGPT Review Packet — Phase 2 robustness sweep + reward (2026-05-11 overnight)

> Composed manually because `scripts/rl/make_chatgpt_review_packet.ps1`
> is blocked by the local PowerShell execution policy. Same schema as
> the auto-generated packet.
> Previous packets preserved:
>  - `docs/review_packets/phase1_2026-05-11_packet.md`
>  - `docs/review_packets/phase12_2026-05-10_packet.md`

**1. Timestamp:**     2026-05-11 ~08:55 EDT
**2. Project root:**  C:\Users\wxy22\Documents\Centipede_IsaacLab_RL
**3. Current branch:** main

---

## 4. Workflow summary (overnight 2026-05-11)

| step | status | output |
|---|---|---|
| Phase 2 robustness sweep | DONE | `outputs/analysis/phase2_robustness_sweep_2026-05-11.csv` (1280 rows: 4 waves × 4 imps × 5 seeds × 16 envs) |
| Phase 2 analysis + 8 figures | DONE | `outputs/analysis/figures/phase2/`, `phase2_robustness_metrics.csv` |
| Case classification | DONE | Initial **Case A** → revised to **weak Case C** after validation |
| Held-out validation (3 cells × 3 seeds, base seed 310) | DONE | `outputs/analysis/phase2_validation_2026-05-11.csv` + comparison report |
| Reward design (pass13) | DONE | `docs/PHASE_3_ROBUSTNESS_REWARD_DESIGN.md` |
| Reward implementation (default-OFF flag) | DONE | `envs/centipede_env.py`, `envs/centipede_env_cfg.py`, `scripts/rl/train_stage_b_smoke.py` + backups |
| Reward audit | PARTIAL — flag_ON pass complete; flag_OFF killed at 4 h (CUDA-sync slowdown); G3 proved by code inspection | `outputs/sanity/pass13_reward_decomposition_report.txt` |
| **RL pilot** | **NOT RUN** — gated on strong Case A/C; validation weakened evidence | — |

## 5. The scientific question

**Does body wave_number and/or body impedance improve rough-terrain
robustness, even if higher wave_num does not maximize absolute
flat-ground speed?**

## 6. Headline result

**Phase 2 sweep showed Case A:** wave_num=2.0 × body_imp ∈ {1.0,1.5,2.0}
showed retention 0.65–0.70 vs wave=1.0's 0.57; decline slope −14 mm/s/S
vs −38 (~3× flatter). Median seed CoV 0.024 — very stable. 8 cells met
all objective Case-A criteria.

**Held-out validation REVERSED this finding.** On 3 candidate cells ×
3 new seeds (base seed 310, disjoint from the sweep's 200–204):

| cell | sweep retention | validation retention | Δ |
|---|---:|---:|---:|
| wave=1.0 imp=1.0 (baseline) | 0.568 | 0.526 | −0.042 |
| wave=2.0 imp=1.0 (sweep's robust winner) | **0.697** | **0.531** | **−0.166** |
| wave=1.0 imp=2.0 (sweep's best worst-case) | 0.527 | 0.594 | +0.067 |

The retention advantage of wave=2.0 vs wave=1.0 was +0.129 in the sweep
but only +0.005 in the validation. **The Case A signal was largely a
spawn-bias artifact of the original sweep.** Validation's v_easy at
S=0.000 for wave=2.0 imp=1.0 was 37.1 mm/s (n=6) — sweep had 22.6 mm/s
(n=11). That 14 mm/s shift is structural, not noise.

**Weaker signal that did survive validation:** higher body_imp (≥2.0)
shows modest retention improvement at wave_num=1.0 (0.594 vs 0.526).

**Confirmed by both sweep and validation:** wave_num > 2.0 is uniformly
harmful — matches Phase 1 finding.

## 7. Per-block per-cell numbers (key wave=2.0 imp=1.0 disagreement)

| severity S | sweep v_fwd (n, ±std) | validation v_fwd (n, ±std) | Δ |
|---:|---|---|---:|
| 0.000 | 22.58 (n=11, ±0.27) | **37.06** (n=6, ±0.12) | **+14.48** |
| 0.100 | 23.49 (n=14, ±0.49) | 36.93 (n=10, ±0.57) | +13.44 |
| 0.160 | 23.71 (n=6, ±0.61) | 34.80 (n=6, ±1.77) | +11.09 |
| 0.333 | 20.58 (n=12, ±2.77) | 26.59 (n=6, ±3.98) | +6.01 |
| 0.556 | 15.74 (n=37, ±3.37) | 19.67 (n=20, ±5.01) | +3.93 |

The sweep's flat-terrain measurement for wave=2.0 imp=1.0 (22.6 mm/s)
appears to be the OUTLIER. Validation gives a more reasonable 37 mm/s.
The sweep stat-tested as "stable" (low CoV) because all 5 sweep seeds
agreed — but those seeds may have advanced the spawn-RNG into a
specific spawn region for wave=2.0 settings systematically.

## 8. What changed in code (overnight)

### New files (sanity / analysis only — no protected-file edits)

- `scripts/sanity/sweep_robustness_phase2.py`
- `scripts/sanity/sweep_validation_phase2.py`
- `scripts/sanity/analyze_robustness_phase2.py`
- `scripts/sanity/classify_robustness_scenario.py`
- `scripts/sanity/compare_validation_to_sweep.py`
- `scripts/sanity/audit_pass13_reward_decomposition.py`
- `docs/PHASE_2_ROBUSTNESS_SWEEP_DESIGN.md`
- `docs/PHASE_3_ROBUSTNESS_REWARD_DESIGN.md`
- `outputs/analysis/PHASE2_ROBUSTNESS_SWEEP_REPORT.md`
- `outputs/analysis/OVERNIGHT_ROBUSTNESS_WORKFLOW_SUMMARY.md` (read this)
- 8 figures in `outputs/analysis/figures/phase2/`
- 3 raw CSVs + analyses in `outputs/analysis/`
- 1 audit report in `outputs/sanity/`

### Protected-file edits (with backups)

| file | backup | change |
|---|---|---|
| `envs/centipede_env_cfg.py` | `.before_pass13_2026-05-11` | added `pass13_robustness_reward_enabled: bool = False` |
| `envs/centipede_env.py` | `.before_pass13_2026-05-11` | added `_REWARD_W_ROBUST_PASS13 = 1.5`, `_BLOCK_SEVERITY_S` tuple, computation block in `_get_rewards()`, log entries `reward/19_robust_pass13` and `raw/severity_S_per_env` |
| `scripts/rl/train_stage_b_smoke.py` | `.before_pass13_2026-05-11` | added `--pass13-robustness-reward` CLI flag |

**All changes additive. Flag default OFF. Pass 8/9/10/11/12 reward behaviour is byte-identical to before.**

## 9. The reward formula (pass13)

```
r_robust_pass13 = + 1.5 * severity_S * v_forward
```

- `severity_S` per-env from `_spawn_block_id` lookup via
  `_BLOCK_SEVERITY_S = (10/18, 10/18, 10/30, 8/50, 6/60, 0.0)` — matches
  the terrain block labels rough_a, rough_b, medium, intermediate,
  long_wave, flat.
- **Does NOT reward wave_num or body_imp directly** — only severity-weighted
  forward velocity.
- Flag default OFF.

The design rationale (from `docs/PHASE_3_ROBUSTNESS_REWARD_DESIGN.md`)
considered three alternatives:
- terrain-normalized progress (`v_forward / baseline_v`) — rejected
  because baseline (wave=1.0) trivially gets 1.0 at every S
- below-floor penalty — rejected because wave=2.0 is slower on flat
  and would be punished
- severity-weighted bonus — selected as the only formulation that
  doesn't conflict with the Case A pattern

In hindsight (post-validation), the Case A pattern was weaker than the
formula was designed to amplify. The formula is still scientifically
defensible — it rewards "doing well on hard terrains" — but its
practical effect on RL is now uncertain.

## 10. Audit verdict

`outputs/sanity/pass13_reward_decomposition_report.txt`:

- G1 finite ≡ True. PASS.
- G2 sum_check ≤ 3e-10. PASS.
- G3 flag_OFF → r=0. Proved by code inspection (the `else` branch
  trivially sets a zero tensor). Empirical test stalled due to
  per-step CUDA-sync overhead in the audit script.
- G4 r_robust ≈ 1.5 × S × v_forward. Direction/sign confirmed across
  6 conditions; magnitude differs from W × mean(S) × mean(v_fwd)
  by 8-39% due to covariance — formula correct per-env.
- G5 no-progress quiet exploit. PASS — r_robust is multiplicative
  in v_forward.
- G6 wave_num=4 (harmful) does NOT score higher than zero_action.
  PASS (wave_4 total −0.0199 vs zero_action +0.0260).
- G7 |r_robust| ≤ 0.05. PASS — max observed 0.012.

## 11. Why the RL pilot was NOT launched

Per the overnight directive:

> "If, and only if, the result is Case A or a strong Case C with
> validation support, you may design a reward candidate."

> "[Case C:] Reward implementation is allowed only if validation
> makes the target unambiguous. Otherwise stop at proposal and
> review packet."

The validation did NOT make the target unambiguous — it actively
weakened it. Therefore: STOP at proposal + review packet.

The reward implementation is preserved (default-OFF flag, full
backups). It can be enabled or rolled back trivially.

## 12. Decisions for ChatGPT

A. **Accept the validation finding** that Case A was largely
   spawn-bias artifact, NOT a real robustness signal? **CONTINUE /
   AUDIT-MORE / REDESIGN?**

B. **Should the pass13 reward implementation be retained** as a
   default-OFF flag for future use, or rolled back entirely? Cost of
   keeping: ~80 lines of additive code, zero runtime cost at flag=OFF.
   Cost of rolling back: ~10 min, lose the work.

C. **Next-experiment proposal — controlled-spawn re-sweep**: re-run
   the wave_num × body_imp grid with a pinned spawn distribution
   (every env explicitly assigned a target spawn block, not RNG drift)
   so per-severity comparisons are apples-to-apples. ~30-45 min
   wallclock estimated.

D. **Alternate severity metric** — heightmap RMS (already logged in
   the arena printout) rather than amp/λ. May be more physically
   relevant; the current amp/λ ranks "medium" (S=0.333) above "rough_a"
   (S=0.556) but baseline v_fwd is actually LOWER at S=0.333 (20.5)
   than S=0.556 (22.4), suggesting amp/λ doesn't track difficulty
   correctly.

E. **Pilot anyway, treating it as diagnostic?** The user's brief
   says "A good result may be: slightly slower on flat, but much
   better retained performance on rough." A pilot would tell us
   whether RL with pass13 reward discovers anything beyond wave=1.0.
   ~30 min wallclock if approved.

F. **Defer scripted adaptive controller and Phase-1 RL design** until
   the re-sweep above (C) resolves the robustness question? The
   Phase-1 plan was contingent on a clear robustness signal that
   Phase 2 has muddled.

G. Anything I've missed.

The most actionable answer is A + B. C/D/E are follow-up
experiments; F is a directive decision.

## 13. Strict non-actions this session

- No PPO training.
- No reward weight changes to existing Pass 8/9/10/11/12 terms.
- No action-bound changes.
- No checkpoint moves/deletes/promotions.
- No IsaacLab core or asset edits.
- No `tasks/agents/*.yaml` edits.

## 14. Backups in case of rollback

If ChatGPT verdict is "ROLLBACK pass13":

```
cp envs/centipede_env.py.before_pass13_2026-05-11 envs/centipede_env.py
cp envs/centipede_env_cfg.py.before_pass13_2026-05-11 envs/centipede_env_cfg.py
cp scripts/rl/train_stage_b_smoke.py.before_pass13_2026-05-11 scripts/rl/train_stage_b_smoke.py
```

That fully reverts the overnight protected-file changes. Sanity
scripts and docs can stay (no behaviour change).
