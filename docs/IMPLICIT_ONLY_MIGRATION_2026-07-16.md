# CentipedeEnv is IMPLICIT-ONLY (2026-07-16) — what changed, and how to migrate

> ## ⚠️ PARTIALLY SUPERSEDED THE SAME DAY — read this box first
>
> This page was written after **S1** (the explicit-path deletion). **S2a/S2b landed hours later** and changed
> the parameter model underneath it (DISPATCH_LOG `2026-07-16T20:5xZ` and `22:05Z`). Sections **2–4 are still
> accurate** — they are the real "why implicit ≠ explicit" story. **Section 1 and Section 5 are NOT:**
>
> | this page says | actually, now |
> |---|---|
> | `controller.set_export_pd(True)` (§1, step 2) | **DELETED by S2b.** That call raises `AttributeError`. Targets are the controller's only output; there is no export flag. |
> | "same controller, same YAML, same gains-json" (§1) | **All three are false.** The controller lost its entire gain/torque chain (it holds no kp/kv, computes no torque, reads no joint state, `step()` returns `None`). The YAML is gain-free and its `impedance:` block was renamed **`reference_targets:`**. gains-json **raises** on any impedance/effort key. |
> | "What is coming next (S2)" (§5) | **S2 is DONE.** §5 is a record of intent, not a plan. |
>
> **THE CURRENT ARCHITECTURE, in one paragraph.** One joint family = exactly **one (Kp, Kv) in physical units**
> (N·m/rad, N·m·s/rad), written straight to PhysX via `env.set_joint_impedance({family: (Kp, Kv)})` —
> families = `env.s2_family_ids()` (8 families = all 211 DOF). Torque caps: `env.set_joint_effort_limit({family:
> tau_max_Nm | None})`, family-exact. The CPG is the **sole reference generator**: it emits position/velocity
> **targets only**, and **all feedback closes inside PhysX's implicit PD solver**. The passive spring,
> `suppress_active_pitch`, the YAML-base × gains-json-SCALE chain and the explicit torque path are **deleted**;
> callers of the deleted members hit a directed `RuntimeError` tombstone (`env._S2_DELETED_MEMBERS`).
>
> **Impedance is no longer a per-step action channel.** The 6D action's ch-4 (`body_imp_scale`) and the legacy
> body_21d ch-20 no longer reach physics and **raise** if moved off 1.0 — see `_S2_DEAD_ACTION_NOTE` at the top
> of `envs/centipede_env.py`. To modulate impedance, modulate the **table**, not a scale.
>
> **Source of truth:** `envs/centipede_env.py` (`set_joint_impedance`, `set_joint_effort_limit`,
> `_apply_action_implicit_pd`, `_S2_DEAD_ACTION_NOTE`) · `outputs/CURRENT_RESULT.md` ·
> `DISPATCH_LOG.md` 2026-07-16T19:34Z → 22:05Z.

**PI-signed** (王熙原, 2026-07-16; `DISPATCH_LOG.md` 2026-07-16T19:34Z). The **explicit torque path is deleted**.
If you have a script that booted `CentipedeEnv` before this date and never set `_implicit_pd`, **it will now raise a
`RuntimeError` on the first step.** That is deliberate. This page tells you why, and how to fix it in two lines.

> **Why it fails loudly instead of just working.** 169 of the 182 env-using scripts never set the flag. Silently running
> them through the implicit path would have changed their physics *without a word* — and implicit ≠ explicit is
> **measured**, not theoretical (see below). A loud error costs you five minutes; silently different physics costs you a
> retracted result. The PI's stated goal for this change was *"以免后面的simulation出错"*.

---

## 1. The two-line migration — ⛔ OBSOLETE (S1-era; do not copy-paste)

```python
# ⛔ DOES NOT WORK since S2b (2026-07-16). Kept to show what the S1-era recipe was.
base_env._implicit_pd = True             # ✅ still the flag that routes the CPG through PhysX implicit PD
base_env.controller.set_export_pd(True)  # ⛔ DELETED by S2b -> AttributeError. Targets are the only output now.
```

**What to do instead.** `_implicit_pd = True` is still correct and is still the flag; the *second* line is gone,
and one thing is now **required** that was not before — the impedance table:

```python
base_env._implicit_pd = True
base_env.set_joint_impedance({fam: (Kp, Kv) for fam in base_env.s2_family_ids()})   # PHYSICAL units, REQUIRED
base_env.set_joint_effort_limit({fam: None for fam in base_env.s2_family_ids()})    # optional; None = uncapped
```

There is **no default gain to fall back on**: stepping without a table raises (deliberately — a silent default is
exactly what S2 removed). Reference wiring: `scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py`
(`--implicit-pd`, `--impedance-json`, `--effort-json`); the PI-facing entry point is
`scripts/analysis/tune_panel_reset_2026-07-16.py` (`--dry-run` validates a config without booting Isaac).

~~Nothing else changes: same controller, same YAML, same gains-json, same CPG.~~ **False as of S2b** — see the box
at the top. Only the **CPG** is unchanged (it is frozen).

---

## 2. What actually changed (the mechanism)

Both schemes use the *same* CPG and the *same* Kp/Kv numbers. The difference is **who computes the force**.

| | EXPLICIT (deleted) | IMPLICIT (the only path now) |
|---|---|---|
| who computes τ | our Python/PyTorch controller | **PhysX's solver** |
| the law | `τ = Kp·(q_ref−q) + Kv·(q̇_ref−q̇)` (`pd_torque`, controller:69-71) | same law, evaluated **inside the solver** |
| how it reaches the sim | `set_joint_effort_target(τ)` — an external torque | `write_joint_stiffness/damping_to_sim(Kp,Kv)` + `set_joint_position/velocity_target(q_ref,q̇_ref)`; effort feed-forward is **zeroed** |
| stability | explicit force in semi-implicit Euler — can lag/overshoot the step | solver-side, unconditionally stable |
| the CPG's role | reference generator | **unchanged — still the sole reference generator** |

The CPG is untouched. `controller.step()` is still called every physics step (it is stateful — it integrates the
Kuramoto phase — and it populates the `_pd_*` stash the implicit path reads). Only its returned `tau` is discarded.

---

## 3. ⚠️ Your old numbers do NOT carry over

**This is a real physics change, not a refactor.** Measured (marker `RESULT-2026-07-16T0350Z-cap2x2`, same config /
gains / terrain / seed, only the scheme varied):

| amp_mult | explicit `bl_per_cycle` | implicit `bl_per_cycle` | difference |
|---|---|---|---|
| **1.0** | 0.396893 | 0.249389 | **−37.2 %** |
| 0.15 | — | — | −0.1 % (parity) |
| 0.05 | — | — | +0.6 % (parity) |

⇒ **Parity holds only at small amplitude.** At the operating amplitudes the project used for most of its history, the
two schemes differ by more than a third. Do not assume a pre-2026-07-16 number reproduces under implicit — re-measure.

### Three specific traps

1. **Effort limits were INERT under explicit and BITE under implicit.**
   `write_joint_effort_limit_to_sim` → `set_dof_max_forces` limits the PhysX **drive** force only. Under explicit
   torque actuation it does nothing (proven: the entire `envs[]` record was **byte-identical** with the cap ON vs OFF,
   3/3, at a cap **441× below** the running torque). Under implicit the same cap torque-kills the robot
   (`bl 0.249389 → −0.000288`). ⇒ **Any "we tested a force limit" conclusion from an explicit run is vacuous — the cap
   never reached the physics.** (This retired the FORCE arm of `compliance_rescue_2026-07-13`.)

2. **`applied_torque` is not a solver readback.** PhysX does not expose the implicit-drive torque.
   `ImplicitActuator.compute` (`actuator_pd.py:117-138`) *recomputes* `clip(Kp·e_q + Kv·e_v + τ_ff, ±effort_limit)` in
   Python, using `actuator.effort_limit` = **10.0 N·m** (`envs/centipede_env_cfg.py:135`), which
   `write_joint_effort_limit_to_sim` never updates (IsaacLab #128). Read it as **estimated drive effort**, not measured
   joint torque. With no live PhysX cap it equals unclipped demand and is fine; **under a live cap it reads far too
   high** (measured 3764.3× over a 0.01 mN·m cap).

3. **The passive spring is folded, not added.** Under explicit it was a separate torque
   (`τ_passive = k(q0−q) − c·q̇`). Under implicit it is folded into the drive at `envs/centipede_env.py:1186-1195`:
   ```
   K = Kp + k_p ;  C = Kv + c_p ;  target = (Kp·pos + k_p·q0)/K ;  vel = Kv·vel/(Kv + c_p)
   ```
   This is an exact algebraic identity (two parallel springs = one spring at the stiffness-weighted-average rest
   point), so the physics matches — but note the **consequence**: if a joint's active `Kp` is 0, its position target
   collapses to `q0` and the commanded wave survives **only** through the velocity channel. That is the live situation
   for the 19 spine-yaw joints (`build_gains` hard-codes `body_kp: 0.0`). *(Scheduled for removal in S2 — see §5.)*

---

## 4. The error you'll see

```
RuntimeError: CentipedeEnv is IMPLICIT-ONLY as of 2026-07-16 (PI-signed; DISPATCH_LOG 2026-07-16T19:34Z).
The explicit torque path (set_joint_effort_target of the controller's tau) has been DELETED, so this caller --
which never set `_implicit_pd` -- would otherwise run SILENTLY DIFFERENT physics (implicit vs explicit is measured
at -37.2% bl_per_cycle at amp_mult 1.0). Failing instead of corrupting the run. TO MIGRATE: ...
```

**If you are reproducing a historical explicit result**, do not migrate — check out the pre-surgery code instead:
`outputs/analysis/implicit_unify_2026-07-16/backup_pre_surgery/` holds the exact pre-change files (md5s in
`DISPATCH_LOG.md`), and `envs/` + `controllers/` are git-tracked. Migrating the script will **not** reproduce the old
number; it will produce the implicit number, which is a different physical result.

---

## 5. ~~What is coming next (S2)~~ — DONE 2026-07-16; kept as the record of intent

> **Status: SHIPPED**, hours after this page was written (S2a `20:5xZ`, S2a-effort `21:35Z`, S2b `22:05Z`).
> Everything below in this section is written in the future tense and describes what S2 *would* do; it did.
> For what actually exists now, read the box at the top of this page and `_S2_DEAD_ACTION_NOTE` in
> `envs/centipede_env.py`. One correction worth carrying: the S2 result marker
> (`outputs/CURRENT_RESULT.md`) **partially retracted** the first S2 headline — the body-yaw torque change was
> **Kv-dominated**, not evidence of the Kp channel doing work. Do not cite the "+71.9% ⇒ Kp tracks the wave"
> inference; read the marker's RETRACTED section before quoting any S2 number.

The PI's ruling: **"不要有所谓的被动弹簧 ... 一个关节就两个，kp和kv，直接喂进implicit参数，作为controller唯一的参数源"**
— and, importantly, *"即使数值用一样的，也要分开，后面我们肯定还要单独调整"* (keep the families **separate** even when
the values coincide; per-family tuning must stay possible).

S2 will remove:
- the passive spring (`enable_passive_spring`, `--bio-pitch-k/--bio-yaw-k/--bio-roll-k`, `--passive-c-scale`) — its
  role is replaced by the joint's own implicit `Kp`, with the CPG's own target (which is already `0` for the
  suppressed spine-pitch joints, i.e. identical physics via one parameter instead of two channels);
- `suppress_active_pitch`;
- the gains-json **scale-on-YAML-base** indirection (physical units go straight to PhysX);
- the head's **double** parameterisation (`head_Kp_eff = head_pitch_kp 0.05 × pitch_kp_scale 0.596 = 0.0298` → one
  `Kp_head_pitch`). Head pitch stays its **own family** with its own target (`head_pitch_offset = −0.25`).

And it will **fix** the body-yaw wiring the PI caught: `Kp_body_yaw` currently arrives on the **passive** channel
(rest point `q0`), so it *fights* the wave instead of tracking it. It moves to the active channel so that **Kp tracks
the CPG wave exactly like Kv does**.

**FARMS reference** (verified from source, `github.com/wxy108/FARMISM_Nov_6`, full clone, positive controls pass):
its centipede runs **one** parameter set across **all 172 joints** — `limits_torque ±2e-4 N·m` (0.2 mN·m),
`gains [0,0]`, `equation: ekeberg_muscle` with `α=5e-5, β=1e-8, γ=1e4, δ=1e-6`, `dt=1 ms` ⇒ effective per-joint
**Kp ≈ 1e-4 N·m/rad, Kv = 1e-6 N·m·s/rad, τ_max = 2e-4 N·m**. Our current values sit **58×–6788×** above that on
stiffness/damping and **50,000×** above on the torque limit — every one of them tuned on the 1.27e6× fake armature.
FARMS's uniformity is a fact about FARMS, **not** a structure we copy: our families stay separate per the PI's ruling.
