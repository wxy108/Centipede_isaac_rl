# ChatGPT Review Packet — Observation enrichment + offline K=12 retraining (2026-05-20)

> Composed manually (the auto-generator `scripts/rl/make_chatgpt_review_packet.ps1`
> errored on git stderr warnings — same blocker noted in
> `docs/review_packets/phase3b_2026-05-11_packet.md` header). Schema mirrors
> the auto-generated packet.
>
> Previous packet preserved:
> - `docs/review_packets/phase3b_audit_corrections_2026-05-11_packet_archived.md`
> - `docs/review_packets/phase3b_2026-05-11_packet.md`
> - `docs/review_packets/phase3_2026-05-11_packet.md`
> - `docs/review_packets/phase2_2026-05-11_packet.md`
> - `docs/review_packets/phase1_2026-05-11_packet.md`
>
> Companion docs (read these in order):
> - `outputs/analysis/online_sensor_severity_observability_2026-05-18/REPORT.md` — 2026-05-18 Class C observability verdict
> - `outputs/analysis/extended_trace_collection_2026-05-18/REPORT.md` — Scope-A trace audit; per-foot timing features at best individual r=+0.254 still Class C
> - `outputs/analysis/overnight_estimator_pass16_forensic_2026-05-19/REPORT.md` — K=12 forensic; LOSO r=+0.45 from K=12 deployable
> - `outputs/analysis/ep30_lockin_diagnostic_2026-05-19/REPORT.md` — ep6 save_best lock-in confirmed; ep30 changes ranking
> - `outputs/analysis/pass18_directionB_swc_v1_w05_step1b_2026-05-19/REPORT.md` — Dir B w_B=0.05 (orig target) FAIL r=−0.77, wrong-signed
> - `outputs/analysis/pass18_directionB_swc_v1_w05_targetB_step1b_2026-05-20/REPORT.md` — Dir B w_B=0.05 (NEW target [2.0,4.5]) FAIL r=−0.31; pre-commit pivot to obs enrichment

---

**1. Timestamp:**     2026-05-20 (late evening; closes the 2026-05-19 → 2026-05-20 reward-redesign branch)
**2. Project root:**  `C:\Users\wxy22\Documents\Centipede_IsaacLab_RL`
**3. Current branch:** main

---

## 4. `git status --short`

```
 M assets/config.yaml
 M controllers/impedance_controller_torch.py
 M envs/centipede_env.py
 M envs/centipede_env_cfg.py
 M scripts/optimization/arena.py
?? (many .before_<feature>_<date> snapshots in envs/, controllers/, scripts/)
?? .claude/, 0-System_initialization/, IsaacLab/, etc.
```

Working-tree modifications listed below in §7 (protected-file diff).

## 5. `git diff --stat` (protected-file summary)

```
 assets/config.yaml                        |   10 +-
 controllers/impedance_controller_torch.py |  711 ++++++++-
 envs/centipede_env.py                     | 2459 ++++++++++++++++++++++++++++-
 envs/centipede_env_cfg.py                 |  597 ++++++-
 scripts/optimization/arena.py             |  171 +-
 5 files changed, 3860 insertions(+), 88 deletions(-)
```

The diff vs commit `c339983` (the Stage A baseline) is the full accumulated history of Passes 8-18. **All Pass-N feature flags are default False** in `envs/centipede_env_cfg.py`; the env's runtime behaviour at default is byte-identical to the pre-feature baseline at flag-OFF.

## 6. `git diff --name-only`

```
assets/config.yaml
controllers/impedance_controller_torch.py
envs/centipede_env.py                        (HEAD of all reward + Pass-N additions; OPTION-B target tweak applied today)
envs/centipede_env_cfg.py
scripts/optimization/arena.py
```

## 7. Protected-file diff — incremental changes since the most-recent packet (2026-05-18 15:09)

Two protected files were touched between the 2026-05-18 packet and now:

**(a) `envs/centipede_env.py`** — Pass 17 K=12 plumbing + Direction A/B reward terms + OPTION-B target recalibration (today):

- Lines ~1900-2045: K=12 R_belief estimator (12 windowed sensor features → ridge → smoothstep ∈ [0, 1])
- Lines ~2014-2026: Direction B r_26 imitation reward. Today's OPTION-B parameter recalibration:

```diff
-    _applied_abs_target = 1.5 + 2.0 * _R_belief                 # ∈ [1.5, 3.5]
+    _applied_abs_target = 2.0 + 2.5 * _R_belief                 # ∈ [2.0, 4.5]
```

Snapshot at `envs/centipede_env.py.before_target_formula_mod_2026-05-20`. The user explicitly authorized this constant change as parameter recalibration (no fresh ChatGPT packet). Bit-identity at default flag-OFF is preserved because the change is inside the `if self.cfg.pass18_k12_imitation_enabled:` branch and `pass18_k12_imitation_enabled` defaults to False.

**(b) `scripts/sanity/audit_r0_40_80_step1b.py`** — `--prefer-last-epoch` standing change (sanity script under `scripts/sanity/`, which is allowed-without-packet per supervision protocol §2). Auto-redirects `*_smoke.pth` paths to highest-ep `last_*_ep_N_rew_*.pth` siblings. Snapshot at `.before_eval_last_epoch_2026-05-20`. Verified via 4-unit test.

No other protected file was modified.

## 8. SHA256 + LastWriteTime for key files (current working tree)

| file | SHA256 | last-edited context |
|---|---|---|
| `envs/centipede_env.py` | `1c3f22430fcff415604539a4feb7cfcd50b0e6b7582b71d7c9f3ea73ccfd0df9` | post-OPTION-B target recalibration (2026-05-20) |
| `envs/centipede_env.py.before_target_formula_mod_2026-05-20` | (snapshot of pre-edit) | rollback target |
| `envs/centipede_env_cfg.py` | `4f86d4e694a4133f32fa6968d74744d0f6e82402b462d2d8973f4ba548620b63` | unchanged today |
| `tasks/agents/rl_games_stage_b_smoke_cfg.yaml` | `580ad395b6e31ee7a98e21f952c78ec3992ccbe487d140b841c13b6ed317c568` | unchanged today |
| `scripts/rl/train_stage_b_smoke.py` | `07fc94b402b601b6c57df01852c41664b8de4f5f267b22c3288deff8d6a0d624` | unchanged today |
| `scripts/sanity/audit_r0_40_80_step1b.py` | `b49387dd5536d650cea9f5c962cbfe199ba40006e53415420695e6706a11059c` | post eval-last-epoch standing change |
| `outputs/analysis/overnight_estimator_pass16_forensic_2026-05-19/data/k12_deploy.json` | (3588 B, 2026-05-19 05:35) | current K=12 coefs (deployed) |

## 9. PPO knobs (unchanged from 2026-05-18 packet)

From `tasks/agents/rl_games_stage_b_smoke_cfg.yaml`:

```
learning_rate: 0.0003
lr_schedule: adaptive
kl_threshold: 0.005
save_best_after: 5
save_frequency: 5
entropy_coef: 0.005
e_clip: 0.2
horizon_length: 32
mini_epochs: 4
bounds_loss_coef: 0.005
```

This packet does NOT propose any PPO-config change.

## 10-12. Pass-N flag state (grep verified; all defaults False except R3 mitigations)

| flag | default | this packet's value at deploy time |
|---|:---:|---|
| pass10_effort_enabled | False | unchanged |
| pass11_cot_enabled | False | unchanged |
| pass12_ang_split_enabled | True (R3 L11) | unchanged |
| pass12_functional_guard_enabled | True (R3 L10) | unchanged |
| pass13_robustness_reward_enabled | False | unchanged |
| pass14_obs_v2_enabled | False | unchanged |
| pass15_v_fwd_floor_enabled | False | ON via CLI at training (mirror) |
| pass15_wave_softcap_enabled | False | ON via CLI at training |
| pass16_attitude_var_enabled | False | unchanged |
| pass16_force_peak_enabled | False | unchanged |
| pass17_k12_estimator_enabled | False | ON via CLI at training |
| pass17_severity_weighted_speed_enabled | False | unchanged |
| pass18_k12_imitation_enabled | False | ON via CLI at training |
| pass18_k12_imitation_weight | 0.02 | overridden to 0.05 via CLI |
| **NEW (proposed):** pass19_extended_trace_enabled | False (new default) | controls trace logging |
| **NEW (proposed):** pass19_k12_v2_enabled | False (new default) | controls K=12 coefficient set |

## 13. Pass 8 reference checkpoint

- Path: `outputs/rl_smoke/stage_b_transition_global6d/2026-05-08_15-27-26/nn/centipede_stage_b_transition_global6d_smoke.pth`
- **Untouched** (NOT to be overwritten per supervision protocol §2 item #10).

## 14. Target output directories for proposed work

After ChatGPT CONTINUE, this packet's plan produces:

- `outputs/analysis/obs_enrichment_trace_collection_2026-05-XX/` — Phase 1 trace data
- `outputs/analysis/k12_v2_offline_refit_2026-05-XX/` — Phase 2 retrained K=12
- `outputs/rl_smoke/pass18_directionB_obs_enriched_{10iter,30iter}_2026-05-XX/` — Phase 3 training
- `outputs/analysis/pass18_directionB_obs_enriched_step1b_2026-05-XX/` — Phase 3 Step1B

## 15. Existing checkpoints from the closed reward-redesign branch

All `outputs/rl_smoke/pass18_directionB_swc_v1_*_2026-05-{19,20}` runs are preserved. Per-epoch ep30 checkpoints retained for forensic comparison (the ep6 lock-in diagnostic relies on them). Best-pth checkpoints retained.

## 16. Proposed next command

(Three-phase plan; see §17-26 for full proposal. After CONTINUE, the first sub-command is the env-edit phase, then trace collection.)

```
# Phase 1 (env-edit, packet-gated):
#   snapshot envs/centipede_env.py.before_obs_enrichment_2026-05-20
#   snapshot envs/centipede_env_cfg.py.before_obs_enrichment_2026-05-20
#   add cfg flags: pass19_extended_trace_enabled (default False), pass19_k12_v2_enabled (default False)
#   add trace logging for the 4 new feature families (see §18) — gated behind pass19_extended_trace_enabled
#   add cfg field pass19_k12_v2_coef_path (default points at the new JSON after Phase 2)
#   run audit_terrain_bit_identity.py to verify flag-OFF bit-identity
#   run audit_pass19_static.py to verify shapes / no NaN / Inf
```

---

## 17. Context — what we just learned, and why the proposal

Today (2026-05-19 → 2026-05-20) the project closed the reward-shape branch on sweep_consistent_v1. Five Step1B's were run against the same arena with different reward shapes; **none pass the primary bio-prior gate (Spearman r(severity, applied_abs) ≥ +0.3 with rough_a > flat).**

| config | epoch | Spearman r | rough_a vs flat | omega_x rough/flat | primary |
|---|---|---:|---:|---:|:---:|
| Pass 15-only (baseline)                | ep30 | −0.2571 | 3.177 < 3.218 | 1.52× | FAIL |
| Pass 17 Direction A (w_A=0.5)          | ep30 | −0.2000 | 2.940 < 2.983 | 4.86× | FAIL |
| Pass 18 Direction B (w_B=0.02, tgt A)  | ep30 | **+0.0286** | **2.996 > 2.977** | 9.58× | FAIL |
| Pass 18 Direction B (w_B=0.05, tgt A)  | ep30 | −0.7714 | 3.093 < 3.136 | 1.85× | FAIL |
| **Pass 18 Direction B (w_B=0.05, tgt B) [today's latest]** | ep30 | **−0.3143** | 3.171 < 3.314 | **1.45×** | FAIL |

**The pattern across all five runs:** the policy moves applied_abs uniformly across blocks, not block-conditionally. The reward shape *is* working at the per-step gradient level (today's per-step check confirmed |Δ_rough|/|Δ_flat| = 8.34× in the bio-prior direction), but the 30-epoch trained policy fails to produce a block-conditional applied_abs trend. This is consistent with the **2026-05-18 Class C observability verdict**: per-step from the current 86-D obs, severity_S is not learnable (best single-feat r = +0.185, multi-ridge r = +0.153, 3-class logistic balanced acc = chance).

The 2026-05-18 STATUS_REPORT had already proposed obs trace enrichment as the next step, but the user gated it pending a final reward-shape attempt. Today's OPTION-B closed the reward branch with the directive's explicit pre-commit: "FAIL goes to obs enrichment without further reward retries."

**This packet executes that pre-commit.**

## 18. Proposed action — three phases

### Phase 1: env trace logging (protected-file edit; flag-gated default False)

Add four new feature families to `env.extras["log"]` per policy step. **Logged for offline analysis only — NOT added to the policy observation space.** Gated behind `cfg.pass19_extended_trace_enabled` (default False); flag-OFF behavior is byte-identical to the current env.

| family | new log key | shape | description |
|---|---|---|---|
| Per-foot force vector | `log["raw/foot_force_vector"]` | `(num_envs, 38, 3)` | `contact_sensor.data.net_forces_w` per foot (already exists in env; this exposes the per-foot 3-vector instead of just F_max / F_total scalars) |
| Per-foot force magnitude | `log["raw/foot_force_mag"]` | `(num_envs, 38)` | `||foot_force_vector||_2` per foot (derived; redundant but cheap and useful for retraining) |
| Per-segment pitch | `log["raw/seg_pitch"]` | `(num_envs, 19)` | per-body-segment pitch angle (radians) from `body_quat_w[:, seg_id]` → quat_to_pitch |
| Per-segment roll | `log["raw/seg_roll"]` | `(num_envs, 19)` | per-body-segment roll angle |
| Per-foot contact state | `log["raw/foot_contact_state"]` | `(num_envs, 38)` | binary float: 1 if `||foot_force||_2 > eps` else 0 |
| Per-foot contact duration | `log["raw/foot_contact_dur"]` | `(num_envs, 38)` | rolling per-foot frames-in-contact counter; resets on contact break |

Total new state needed in env: 1 buffer for the contact-duration counter, shape `(num_envs, 38)`, int32. ~5 KB.

Compute cost: dominated by `contact_sensor.data.net_forces_w` slicing (already in env hot path) and 38 × 3 quaternion-to-Euler conversions per env per step. Estimated overhead < 1% of current per-step time.

CLI flag wiring: `--pass19-extended-trace` in both `train_stage_b_smoke.py` and `scripts/sanity/audit_r0_40_80_step1b.py`. Default False; mirror to `cfg.pass19_extended_trace_enabled`.

**Static audit (no PPO):** `scripts/sanity/audit_pass19_static.py` to verify:
- with flag OFF, env produces byte-identical first-5-step obs + reward + extras["log"] keys (set diff) as the pre-edit snapshot
- with flag ON, the 6 new log keys are present; all tensors finite (no NaN/Inf) across 100 random-action steps; shapes match the spec above
- bit-identity invariant at default arena AND sweep_consistent_v1 arena

### Phase 2: offline K=12 v2 retraining

The current K=12 estimator (`outputs/analysis/overnight_estimator_pass16_forensic_2026-05-19/data/k12_deploy.json`) was fitted on a limited windowed feature pool from the 86-D obs. Today's evidence shows it has Class B LOSO r=+0.45 on flat-vs-stress separation — too noisy for the policy to learn a block-conditional applied_abs.

The new feature pool from Phase 1 includes:
- 38 per-foot force magnitudes
- 38 binary contact states
- 38 contact durations
- 19 per-segment pitches
- 19 per-segment rolls
- Per-foot force vector (3D directional info)

Each can be aggregated over W={5, 10, 20, 40} windows with {mean, std, p95, switch_rate, entropy} reductions, producing a feature pool of ~500-1000 candidates per window size.

**Methodology (sanity-only; pure offline, no env edit):**

1. Use the **Pass 18 Dir B w_B=0.05 targetB ep30 policy** (today's latest trained policy) as the "rollout policy" for trace collection. With `pass19_extended_trace_enabled=True`, run `audit_r0_40_80_step1b.py` at N=128 envs × 8 s × 2 seeds (42, 6868) on sweep_consistent_v1. Total ~512 envs × 167 steps ≈ 85,500 logged rows × ~600 candidate features. Wall ~10 min.

2. **Feature selection.** For each window size W, compute per-feature point-biserial correlation against `spawn_block_id` (6-class) and Pearson r against `severity_S`. Drop features with |r| < 0.15 (noise floor from 2026-05-18 audit). Apply L1-regularized logistic regression with class-balanced weights to identify the top-50 candidates, then run forward selection with LOSO cross-validation on `spawn_block_id` to identify the optimal 12-feature subset.

3. **Class A target.** Per the directive's pre-commit, the new K=12 must reach **Class A observability**: LOSO mean r(severity_S, R_belief) ≥ +0.7 AND 3-class balanced accuracy ≥ 0.6 (the project's standing Class A threshold per 2026-05-18 observability audit).

4. **Output:** new JSON `outputs/analysis/k12_v2_offline_refit_2026-05-XX/data/k12_v2_deploy.json` with the 12 selected features, their (mu, sigma, coef_z, intercept), and per-feature provenance (W, family, reduction). Also produces a LOSO scatter plot and per-fold r table.

5. **Acceptance gate** (sanity-only verdict; no env hot-swap unless gate passes): 
   - LOSO r ≥ +0.7 ⇒ Class A confirmed ⇒ proceed to Phase 3
   - LOSO r ∈ [+0.5, +0.7) ⇒ Class B (improved over current +0.45 but below Class A) ⇒ STOP, report, pre-committed branch to architectural investigation
   - LOSO r < +0.5 ⇒ Class B-or-worse, ditto

### Phase 3: deploy K=12 v2 + Direction B re-training

Conditional on Phase 2 hitting Class A:

1. Hot-swap the new K=12 coefs into env code (constants inside the `_R_belief` computation block). Gated behind `cfg.pass19_k12_v2_enabled`; default False. Snapshot before edit.

2. **Static audit:** confirm `pass19_k12_v2_enabled=False` reproduces original K=12 R_belief on a deterministic rollout (bit-identity for the existing pass17/18 path).

3. **Reward shape unchanged.** Direction B target = `2.0 + 2.5 · R_belief ∈ [2.0, 4.5]`, w_B = 0.05. The OPTION-B parameter recalibration is preserved.

4. Run the standard cycle: per-step magnitude check → 10-iter SMOKE → 30-iter PPO → Step1B on explicit ep30 (using the standing eval-on-last-epoch fix).

5. **Pre-commit gates** (FINAL DECISION at Step1B):
   - **PASS** = primary gate (Spearman r ≥ +0.6 AND no inversions AND rough_a > flat) AND secondary gates all PASS → working baseline + generalisation packet next (held-out seed, 100-iter PPO with packet, video render).
   - **FAIL by any gate** ⇒ architectural investigation, NOT another obs-shape iteration. The directive is explicit on this point.

## 19. Why these 4 trace families specifically

The four families correspond to the four orthogonal sources of terrain information available to a rigid-body sensor suite:

- **Per-foot force vector** carries the IMMEDIATE contact force per leg. Today's 86-D obs only exposes F_total (sum over feet) and F_max_link (per body, not per foot). Per-foot force gives per-leg load profile, which carries severity information that summary statistics destroy.
- **Per-segment pitch/roll** carries the BODY POSTURE per segment. Currently the env's obs has root attitude (`projected_gravity_z`) and a single roll/pitch summary. Per-segment exposes the body's wave shape and how it conforms to terrain — directly relevant to severity.
- **Per-foot contact state** carries the GAIT structure (which feet are loaded vs swinging at each instant). The 86-D obs has no per-foot contact bit; only aggregate `n_active_feet`-style summaries are derivable.
- **Per-foot contact duration** carries TEMPORAL gait variability (duty per foot, switch_rate per foot, asymmetric stance times under terrain stress).

Together: per-foot/per-segment spatial granularity × temporal granularity (raw + rolling counter) covers the contact+attitude+timing space that the 2026-05-18 Scope-A audit identified as the family with the strongest residual signal (best per-feat r = +0.254 for `duty_std_w5`, but still Class C because the data was per-foot-aggregated rather than per-foot resolved).

## 20. Standing-rule compliance

| rule | status |
|---|:---:|
| Protected files edited only behind default-OFF flags | YES (proposed pass19_extended_trace + pass19_k12_v2 both default False) |
| `.before_<feature>_<date>` snapshots of all touched protected files | YES (proposed) |
| Bit-identity invariant at flag-OFF | YES (proposed verifier: `scripts/sanity/audit_pass19_static.py`) |
| Sanity scripts (under `scripts/sanity/`) | YES (audit + feature-selection + LOSO scripts are sanity-only) |
| No reward-code edit in this packet | YES (Direction B reward stays at today's OPTION-B target) |
| No PPO config change | YES |
| No `arena.py` edit | YES |
| No `IsaacLab/` core edit | YES |
| No checkpoint overwrite | YES |
| `make_chatgpt_review_packet.ps1` was attempted | YES (failed on git stderr warning; manual composition per phase3b convention) |
| Auto-kill watcher co-launched on every IsaacLab launch | YES (proposed for Phase 1 audit + Phase 2 trace collection + Phase 3 PPO + Step1B) |

## 21. Rollback path

If ChatGPT verdict is ROLLBACK at any phase:

```
# Phase 1 rollback (env trace logging):
Copy-Item envs/centipede_env.py.before_obs_enrichment_2026-05-XX     envs/centipede_env.py     -Force
Copy-Item envs/centipede_env_cfg.py.before_obs_enrichment_2026-05-XX envs/centipede_env_cfg.py -Force
# Phase 2 has no protected-file edit (offline-only).
# Phase 3 rollback (K=12 v2 hot-swap):
Copy-Item envs/centipede_env.py.before_k12_v2_2026-05-XX envs/centipede_env.py -Force
# At any phase, run audit_terrain_bit_identity.py to confirm restoration.
```

The current OPTION-B target formula change from today is preserved across rollbacks (separate snapshot at `before_target_formula_mod_2026-05-20`).

## 22. Risk analysis

| risk | mitigation |
|---|---|
| **Per-foot force / per-segment pitch logging adds compute overhead that bloats PPO step time** | Phase 1 audit measures wall-time overhead; if > 5% per step, gate aggregates (compute only mean/p95, not full vector) |
| **K=12 v2 retraining overfits to the trace-collection policy** | Use Pass 18 Dir B trained policy (not Pass 8) for traces; cross-validate LOSO with a HELD-OUT seed (6868) different from the training seed (42); also bootstrap CI on the LOSO r |
| **Class A K=12 (LOSO r ≥ +0.7) is reachable but Step1B still FAILS** | This is the directive's "architectural investigation" branch — packet handoff documents next-level questions (CPG-policy interaction, action-space dimensionality, reward shape revisitation under richer obs) |
| **Hot-swap of K=12 coefs accidentally breaks R_belief computation** | `pass19_k12_v2_enabled=False` returns the original code path bit-for-bit; Phase 3 static audit verifies before any PPO |
| **Direction B's per-step penalty magnitude shifts because R_belief distribution changes under v2** | Pre-SMOKE per-step magnitude check is part of Phase 3 (same as today's standing protocol); if |r_26|/|r_01| > 5× a recalibration step would be flagged and packet-gated |
| **Gait-gaming of the new R_belief** | The R_belief gaming audit (3-test suite from 2026-05-19 Pass 17 Dir A) is re-run on the new K=12; if borderline gaming detected the user/auditor decides whether to proceed |

## 23. Pre-commit (re-statement)

Per user directive:

- **Success** = Class A K=12 (LOSO r ≥ +0.7) AND Step1B Spearman r ≥ +0.6 AND rough_a > flat in applied_abs.
- **Failure** under either gate ⇒ architectural investigation. NOT another obs-shape iteration. NOT another reward retry.

The user has bound the next two decisions in advance.

## 24. What this packet does NOT propose

- No reward-code edit (Direction B stays at today's OPTION-B target).
- No PPO config change (no `kl_threshold` / `lr_schedule` / `mini_epochs` / `save_best_after` tuning, despite today's evidence that save_best_after=5 contributes to the ep6 lock-in).
- No new arena / terrain change.
- No 100-iter PPO (10/30 SMOKE+train pattern unchanged).
- No video render.
- No checkpoint promotion. Today's all-FAIL Step1B's are preserved for the obs-enrichment training to replace.

## 25. Question for ChatGPT

**Should we CONTINUE, STOP, ROLLBACK, AUDIT-MORE, or REDESIGN?**

Specific sub-questions for ChatGPT to weigh in on:

1. **Are the 4 trace families the right set?** Particularly: is per-foot force VECTOR (3D) necessary, or is per-foot force MAGNITUDE (1D) plus per-foot direction-of-max sufficient? Logging magnitude reduces compute and feature pool size by 3× without losing severity-relevant signal in our experience.
2. **K=12 fixed at 12, or open to K ∈ {8, 12, 16, 20}?** The 2026-05-19 K-sweep showed elbow at K≈10-12; the directive bakes in K=12. ChatGPT may prefer a sweep at the new feature pool.
3. **Class A threshold = LOSO r ≥ +0.7.** Is this the right operationalization? Alternative: 3-class severity balanced accuracy ≥ 0.6 (also called Class A in the 2026-05-18 audit).
4. **Trace policy = Dir B w_B=0.05 targetB ep30.** Is this the right choice? Alternatives: Pass 8 baseline (NOT trained on this arena; OOD risk per `PASS8_NAN_OOD_DIAGNOSTIC_FILED.md`); the OPTION-B ep27 best.pth (different policy state); a held-out spawn policy.
5. **Should K=12 v2 retraining use trace data from MULTIPLE policies** (Pass 15 ep30, Pass 17 Dir A ep30, Pass 18 Dir B w_B=0.05 ep30, w_B=0.05 targetB ep30) to reduce overfitting to a single policy's gait? Each adds ~10 min of trace collection.
6. **Should the env CONSUME the new features as part of the OBSERVATION (e.g., 86-D → ~140-D obs), or stay LOG-ONLY?** The 2026-05-18 STATUS_REPORT proposal was log-only; the directive's wording allows either. Adding to obs changes the policy's input dimensionality and breaks reuse of any earlier Pass 18 checkpoint as warm-start.
7. **Pre-commit on "architectural investigation":** what should that branch actually look like? Options surfaced today: (a) PPO-config tuning of `save_best_after` to escape ep6 lock-in; (b) CPG-policy interaction audit (the action mapping currently allows the policy to set wave_number_scale_runtime arbitrarily, but the body's response to that command is the CPG dynamics — maybe the lever is on the CPG side); (c) action-space dimensionality change (currently 6-D global_6d; maybe per-block action gating).

Per `docs/CHATGPT_SUPERVISION_PROTOCOL.md` §3 step 6, please return **one verdict** + (optionally) specific guidance for any of sub-questions 1-7.

## 26. Architectural note (informational, not actionable in this packet)

The 30-iter PPO runs on this arena show a robust **ep6 save_best lock-in pattern** — except today's Pass 18 Dir B w_B=0.05 (target A) run, where rl_games updated the running-best at ep27 to −33.66 (vs ep6's −34.10). At higher w_B the reward signal is strong enough to occasionally beat the ep6 anchor; at w_B=0.02 and w_B=0.05/targetB it does not. This suggests `save_best_after=5` may be effectively capturing the policy at its earliest moderately-good state. **No PPO change is proposed in this packet** — but ChatGPT may flag this as part of the architectural investigation branch if Phase 3 fails.

---

**End of packet.** Paste this entire file into ChatGPT. Report verdict back to Claude.
