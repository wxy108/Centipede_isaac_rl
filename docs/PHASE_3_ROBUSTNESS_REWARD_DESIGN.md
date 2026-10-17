# Phase 3 — robustness reward design (pass13_robustness_reward_enabled)

> Drafted 2026-05-11 ~04:30 EDT, after Phase-2 sweep Case A verdict.
> Status: DESIGN — implementation gated on validation-sweep agreement.

## 1. Goal

A minimal, functional reward term that **encourages forward progress on
harder terrains without rewarding specific gait parameters**. Goes
behind `pass13_robustness_reward_enabled` (default OFF).

## 2. Why this reward (not other ones)

Phase 2 sweep (Case A verdict) showed:
- wave_num = 2.0, body_imp ∈ {1.0, 1.5, 2.0}: retention 0.65–0.70,
  decline slope −14 mm/s/S (vs wave=1.0's −38).
- But: wave=2.0 cells are slower **everywhere** in absolute terms
  (flat 22.6 vs 39.4 mm/s, rough 15.7 vs 22.4 mm/s). The robustness
  story is about **decline slope**, not absolute speed.

Per-step retention is hard to compute directly (it's a ratio of two
across-episode means). A practical per-step proxy that pushes the
policy toward "progress on hard terrain" is:

```
r_robust_pass13 = + W_ROBUST * severity_S * v_forward
```

This term:
- is **zero on flat** (S=0) — no incentive to slow down on easy terrain;
- scales linearly with **per-env severity** (S=amp/λ, in [0, 0.556]);
- is proportional to **v_forward** (must move to earn it);
- combines additively with the existing `r_v = 1.0 * v_forward`.

## 3. Why this reward is NOT a direct parameter incentive

`r_robust_pass13` does NOT reference `wave_num`, `body_imp`, or any
controller scale. It only sees:
- per-env terrain severity (derived from `_spawn_block_id`),
- forward velocity (already in the reward).

The policy can satisfy it via any gait it chooses; the Phase 2 data
just tells us that wave_num=2.0 happens to be one such gait.

## 4. Why not a below-floor penalty

A below-floor penalty (`-W * max(0, floor - v_forward)`) was
considered. Rejected because:
- the Phase-2-robust regime (wave=2.0) is actually SLOWER on flat
  than wave=1.0 (22.4 vs 39.4 mm/s), and would itself trip a
  retention-based floor (set at 50–60 % of baseline);
- this would PUNISH the very behaviour we want to encourage.

A floor centered on per-terrain v=0 (no-progress floor) is already
in `_get_rewards` via `r_v` (gain through zero) plus existing alive
reward; adding another floor would double-count.

## 5. Why not "v_forward / baseline_v_at_S"

Normalized progress (`v_forward / baseline_v_at_S`) was considered.
Rejected because:
- the baseline (wave=1.0, imp=1.0) trivially achieves ratio = 1.0
  at every severity, so it actually *out-scores* wave=2.0 under
  this reward (which earns 0.57 on flat, 0.70 on rough);
- would push RL back toward wave=1.0, the opposite of intent.

## 6. The implementation

### 6.1 cfg field

`envs/centipede_env_cfg.py`:
```python
# pass13_robustness_reward_enabled:
#     when True, adds a severity-weighted forward-progress bonus
#         r_robust_pass13 = + W_ROBUST * severity_S * v_forward
#     where severity_S is per-env from BLOCK_PHYSICS_S[spawn_block_id]
#     (computed once, cached on env).
#     Default OFF; flag=False reproduces Pass 8/9 reward exactly.
pass13_robustness_reward_enabled: bool = False
```

### 6.2 env constants

`envs/centipede_env.py`:
```python
_REWARD_W_ROBUST_PASS13 = 1.5        # severity boost weight
# Per-block severity S = amp_mm / λ_mm, indexed by block_label order:
_BLOCK_SEVERITY_S = (10.0/18.0,      # rough_a
                     10.0/18.0,      # rough_b
                     10.0/30.0,      # medium
                     8.0/50.0,       # intermediate
                     6.0/60.0,       # long_wave
                     0.0)            # flat
```

### 6.3 per-step computation

Inside `_get_rewards()`:
```python
r_robust_pass13 = torch.zeros_like(self._v_forward)
if self.cfg.pass13_robustness_reward_enabled:
    if not hasattr(self, "_pass13_S_lut"):
        self._pass13_S_lut = torch.tensor(self._BLOCK_SEVERITY_S,
                                          dtype=self.dtype, device=self.device)
    S_per_env = self._pass13_S_lut[self._spawn_block_id]   # (E,)
    r_robust_pass13 = self._REWARD_W_ROBUST_PASS13 * S_per_env * s["v_forward"]
```

Add to total:
```python
total = ... + r_robust_pass13
```

Log:
```python
log["reward/15_robust_pass13"] = r_robust_pass13.detach()
log["raw/severity_S_per_env"]  = (S_per_env if self.cfg.pass13_robustness_reward_enabled
                                  else torch.zeros_like(self._v_forward)).detach()
```

### 6.4 CLI flag

`scripts/rl/train_stage_b_smoke.py` adds `--pass13-robustness-reward`
matching the existing `--pass11-cot`, `--pass12-cot-bounded`, etc.
pattern.

## 7. Expected per-step magnitude

With W_ROBUST = 1.5:
- Flat (S=0): `r_robust = 0` (always).
- Long-wave (S=0.10): `r_robust = 0.15 × v_forward` (≈ +0.0058 at v=0.039 m/s).
- Medium (S=0.333): `r_robust = 0.50 × v_forward` (≈ +0.010 at v=0.020 m/s).
- Rough (S=0.556): `r_robust = 0.83 × v_forward` (≈ +0.019 at v=0.022 m/s).

Compare to existing `r_v = 1.0 × v_forward` (~0.039 on flat, ~0.022
on rough). The new term doubles the rough-progress incentive at most,
while leaving flat-progress unchanged.

## 8. Safety / non-exploit properties

- **No-progress quiet exploit**: r_robust × 0 = 0, so r_robust offers
  no incentive to stand still. The existing alive reward already
  provides the standard "be alive" baseline.
- **High-impact speed exploit**: r_robust is gated by v_forward (must
  move forward). The existing F_max, vz_rms, and ang penalties remain
  unchanged — they still penalize impact.
- **Flat-terrain regression**: r_robust = 0 on flat, so no incentive
  to slow on flat.
- **Direct parameter exploit**: r_robust never references wave_num /
  body_imp / freq. The policy can satisfy it with any gait.

## 9. Audits required before RL pilot

`scripts/sanity/audit_pass13_reward_decomposition.py` (new), modeled
after `audit_pass11_reward_decomposition.py`:

1. **Zero-action baseline**: r_robust_pass13 = ?
2. **Wave=2.0 imp=1.0 fixed-parameter** (the Case-A robust winner):
   r_robust_pass13 = ?
3. **Wave=1.0 imp=1.0 fixed-parameter** (the Phase-1 baseline):
   r_robust_pass13 = ?
4. **Wave=4.0 imp=1.0** (known-harmful from Phase 1; should score
   poorly even with r_robust):
   r_robust_pass13 = ? (expected: small because v_forward is small)
5. **Quiet no-progress** (zero gait + zero RL action): r_robust_pass13
   should be 0 (no v_forward).
6. **G1**: all finite.
7. **G2**: total = sum(components) to FP precision.
8. **G3**: flag=False reproduces Pass 8/9 reward exactly.
9. **G4**: r_robust = 0 in every cell when flag=False.

If G1–G4 fail, halt before any RL.

## 10. Pilot training settings (if audits pass)

- task: `Isaac-Centipede-Stage-B-Transition-v0` (same as Pass 12)
- num_envs: 16
- max-iterations: 30 (small pilot — gate on completion + clean BEST + clean FINAL)
- warm-start: Pass 8 best ckpt (or Pass 12 B4 promoted)
- PPO config: PPO-E (`rl_games_stage_b_smoke_pass12_B4_RECOMMENDED.yaml`)
- new flag: `--pass13-robustness-reward`
- existing flags kept: `--pass11-cot --pass12-cot-bounded --pass12-functional-guard`
  (so the change vs Pass 12 B4 RELAXED is ISOLATED to the new term)
- out-dir: `outputs/rl_smoke/pass13_robustness_pilot_2026-05-11`

## 11. Stop conditions during pilot

- Any NaN in reward or loss → stop, save log
- F_max > 0.20 N sustained → likely impact exploit, stop
- Audit shows policy converges to wave=4.0 or freq>>1.0 (parameter
  exploit despite intent) → stop
- v_forward < 5 mm/s sustained across all blocks → no-progress exploit,
  stop

## 12. Non-actions reaffirmed

This reward design does NOT:
- modify action bounds,
- modify any existing reward term,
- change `freq` constraint,
- expand the action space,
- modify env / asset / task-cfg layout,
- promote any checkpoint.

## 13. Pre-edit backups (required before §6 implementation)

Before touching `envs/centipede_env.py` and `envs/centipede_env_cfg.py`:

```
cp envs/centipede_env.py     envs/centipede_env.py.before_pass13_2026-05-11
cp envs/centipede_env_cfg.py envs/centipede_env_cfg.py.before_pass13_2026-05-11
```

And before touching `scripts/rl/train_stage_b_smoke.py`:

```
cp scripts/rl/train_stage_b_smoke.py scripts/rl/train_stage_b_smoke.py.before_pass13_2026-05-11
```
