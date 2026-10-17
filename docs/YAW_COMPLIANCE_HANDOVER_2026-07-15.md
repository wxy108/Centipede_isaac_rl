# Yaw-Compliance Handover — 2026-07-15

> ## ⚠️ STALE IN PART — the parameter mechanism changed on 2026-07-16 (read this first)
> 
> This page was written **before** the implicit-only unification (S1/S2a, PI-signed; `DISPATCH_LOG.md`
> 2026-07-16T19:34Z). The **problem statement and the physics findings still stand**, but every parameter name below that refers to
> the passive spring, the active-pitch suppressor, or gains-as-scales **no longer exists**. In particular the
> "param taxonomy" list further down still names `k_*_passive`, `c_scale_passive` and `suppress_active_pitch`
> as tunables: they are gone. The yaw knob is now simply `Kp`/`Kv` of the `body_yaw` family.
> 
> | this page says | reality since 2026-07-16 |
> |---|---|
> | `--passive-spring`, `--bio-*-k`, `k_*_passive`, `c_scale_passive`, `enable_passive_spring()` | **DELETED.** There is no passive spring. The LOCKED bio pitch `k=6e-4` is simply the **`Kp` of the `body_pitch` family**. |
> | `--suppress-active-pitch`, `--keep-head-up`, `keep_head_up`, `suppress_active_pitch()` | **DELETED.** `head_pitch` is now its own family with its own `(Kp, Kv)`, so `keep_head_up` is subsumed. |
> | gains-json values are **SCALES** on `configs/farms_controller.yaml` | **DELETED.** One family -> exactly one **`(Kp, Kv)` in physical units**, written straight to PhysX via `env.set_joint_impedance()`. Nothing scales it. |
> | `effort` in a gains-json / `set_freesearch_runtime(enable_effort=...)` | **DEAD under implicit** (it clamps a `tau` the implicit path discards, so a run believing it was capped was silently UNCAPPED) and now **raises**. The real cap is `env.set_joint_effort_limit({family: tau_max_Nm | None})`. |
> | the explicit torque path | **DELETED.** `_apply_action` raises unless `_implicit_pd` is set. |
>
> **Full guide: `docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md`.** Calling any deleted member now raises a *directed*
> `RuntimeError` from the env tombstone (`envs/centipede_env.py`, `_S2_DELETED_MEMBERS`) rather than a bare
> `AttributeError`. To reproduce a historical explicit/passive-spring number, do **not** migrate the script --
> implicit != explicit is MEASURED (-37.2% bl/cycle at amp_mult 1.0); restore
> `outputs/analysis/implicit_unify_2026-07-16/backup_pre_surgery/` instead.


**Reader: new auditor picking up the horizontal (yaw) body-compliance problem.** Read this + CLAUDE.md +
`memory/MEMORY.md` + `docs/AUDITOR_HANDOVER_2026-07-15.md`. The PI (王熙原) is hand-tuning the compliant centipede via
the one-stop panel and driving this line personally.

---

## 0. THE ONE OPEN PROBLEM (this is the goal — everything else is prerequisite)

**Make body YAW compliant/conforming to the terrain + leg-contact forces in the horizontal (XY) plane — the way body
PITCH already conforms to terrain height in Z — WHILE the centipede still walks forward.**

- Body **PITCH** already conforms (drapes over bumps vertically). ✅
- Body **YAW** does **NOT** conform in XY. ❌  ← **UNSOLVED. This is the job.**

The PI's vision (verbatim intuition, correct): the legs during stance trace arcs/circles (their own sine); those circles
do **not** match the body's commanded yaw wave; a **rigid** body wave fights the legs → feet slip / lose contact. We want
the body's yaw wave to **yield ("屈服过去")** to the leg/terrain forces so the body's horizontal curve conforms to where the
legs actually plant on the rough terrain — i.e. horizontal compliance, emergent from the leg-ground interaction, not a
rigidly imposed serpentine template.

---

> **⛔ RULE 1 — DATA FIRST, NO JUDGMENT WITHOUT DATA (PI, non-negotiable): before ANY analysis or claim, INDEPENDENTLY run
> the sim yourself (a 3 s test is enough: `--duration 3`), dump ALL joint data (per-joint q, q_ref, applied torque, contact),
> and reason FROM that data. Everything below is reference/hypothesis to be CONFIRMED against your own run — not taken as
> fact. 用数据说话 — let the data speak.**

> **⛔ RULE 2 — VIDEO ONLY AT KEY / REPRESENTATIVE POINTS (PI refinement): do NOT render a video for every config. Render the
> 3-VIEW clip (top=YAW / side=PITCH / chase=45°) only at the KEY, representative parameter sets that decide a conclusion, and
> inspect those one-by-one. [[feedback_multiview_sweep_video_2026-07-15]]**

> **⚠️ RULE 3 — almost NOTHING below is "settled/correct."** Most is the PI's REASONING/REFERENCE or an untested hypothesis.
> Do not restate a reference as a fact. Call something settled only if you measured it or it is physically airtight — and
> re-derive if in doubt. The PI corrected repeated "this is correct/sound" over-claims this session; do not repeat them.

## 0.5 ⭐ FIRST TASK — AUDIT THE ARMATURE / "is 0.2 mN·m really enough?" (do this BEFORE any yaw work)

**What the PI reports (NOT measured — an eyeball impression):** at an effort cap of **0.2 mN·m** the legs barely swing.
**Back-of-envelope CALCULATION (arithmetic, NOT data, NOT confirmed):**
- `configs/sim_params.yaml:31  armature: 0.0001` → **every joint carries a 1e-4 kg·m² rotor inertia.**
- A leg is nearly massless: proximal links 1e-9 kg, whole leg ≈ **3.9 µg**, physical inertia about legbase ≈ **1e-10 kg·m²**
  (`models/farms/centipede_rebuild_v1_renamed.xml`). So armature/leg ≈ **10⁶×**, and armature/**whole-robot**
  (2.5 g × 5 cm² ≈ 6e-6) ≈ **16×** — IF that armature really applies to the leg joints at runtime (audit it, below).
- ⇒ `α = τ/I_eff = 0.2e-3 / 1e-4 = 2 rad/s²`. Swinging 0.15 rad @ 1 Hz needs `A·ω² = 5.9 rad/s²` (≈0.59 mN·m), so the
  calc says 0.2 mN·m would give only ~⅓ the accel needed. **The PI notes this calculated 2 rad/s² looks ROUGHLY like the slow
  motion seen by eye — but that is a loose impression, it does NOT confirm anything.** LEADING HYPOTHESIS (to TEST, not to
  assert): the armature — not the tiny robot — is throttling the leg. Could also be an effort-path bug, higher runtime
  armature, or a bigger opposing load. MEASURE it.

**FIRST TASK for the new auditor (data-first, RULE 1):**
1. **Audit** the ACTUAL runtime per-joint armature + effort_limit + stiffness/damping (`scripts/analysis/audit_A_inventory_2026-07-13.py`)
   — confirm armature is really 1e-4 on the LEG joints (not just the yaml default), and that the leg actually receives the
   0.2 mN·m (rule out an effort-path bug).
2. **Isolated torque test, SUSPENDED IN AIR** (PI-specified — ground contact adds disturbance, so remove it): hang the
   centipede (the cell supports airborne mode, `--airborne-z`), apply a KNOWN torque (e.g. 0.2 mN·m) to a leg joint and,
   separately, to a body joint, and MEASURE the actual angular acceleration / q(t). **Test the legs, test the body,
   separately.** Answer directly: is 0.2 mN·m really enough to move each, free of contact? (Predicted: leg α ≈ 2 rad/s² =
   too slow; but MEASURE it, don't take the prediction.)
3. **Fix to TEST if the armature is confirmed as the throttle:** drop the leg (and maybe body) armature toward a physical
   value (≪1e-4, e.g. 1e-7–1e-8) so a given torque produces the accel the PI's intuition expects. ⚠️ armature is padded for
   numerical stability — 1e-9 has NaN'd before ([[project_body_yaw_calibration_2026-07-14]]); step DOWN carefully, watch
   stability. Alternatively raise the cap, but that trades compliance — the RIGHT fix is almost certainly the armature.

This gates everything else: if the legs can't be driven because of the armature, no amount of yaw-cap/gain tuning matters.

---

## 1. THE PITCH-JOINT vs YAW-JOINT COMPARISON — the 4 conditions that make PITCH compliant, and how YAW violates each

This is the roadmap: the pitch joint conforms because of 4 things at once. To make the yaw joint conform we must replicate
their EFFECT — but yaw is the propulsion, so it cannot copy them literally. (Mechanism-grounded reasoning; the new auditor
must still CONFIRM each against real joint data per the data-first rule above — don't take these as measured.)

| # | condition | PITCH (compliant ✅) | YAW (not compliant ❌) |
|---|---|---|---|
| **1** | active drive | **OFF** — `suppress_active_pitch=True`, no active PD fighting displacement | **ON** — actively drives the CPG wave; the drive fights any push off the wave |
| **2** | holding stiffness | a **soft passive spring** only (`k_pitch_passive`=6e-4) | **active kv-tracking** (`body_kp=0`, `body_kv≈1.5e-3`); its passive spring `k_yaw_passive`=7.2e-5 is even softer but is overridden by the active drive |
| **3** | reference it holds around | a **FIXED** target (q0 = level) → a soft spring to a fixed point **drapes** | a **MOVING** target (`q_ref=amp·sin(ωt−k·i)`) → tracking a moving trajectory can't yield to terrain |
| **4** | load to yield to | **gravity** — persistent, pulls each segment onto the terrain → drapes | **none persistent** — a heightmap z=f(x,y) exerts NO lateral force; the ONLY horizontal load is the **leg-contact reaction**, so yaw has nothing to conform to unless the legs engage the ground and push |

**Roadmap implication:** replicate condition **2** (make the hold soft), but conditions **1** (drive off) and **3** (fixed
target) **CONFLICT with propulsion** — yaw MUST actively drive a moving wave. So the design must deliver pitch-like YIELD
*while keeping* the active moving-wave drive (that's the §5 levers: saturation / soft position-tracking around the moving
wave / feedforward+feedback decouple), and condition **4** requires the legs to actually LOAD the yaw laterally (so leg
engagement is a precondition, not optional).

**THE TENSION (the crux):** yaw IS the propulsion wave. It cannot go fully passive like pitch or the robot stops walking.
So yaw compliance must be **PARTIAL** — yield to disturbance (leg/terrain) while still driving the wave. That is the whole
difficulty; a trivial "make it soft" kills locomotion.

---

## 2. THE PI'S DESIGN RATIONALE (a REFERENCE for the tuning intent — NOT a validated/"correct" architecture)

The PI offered this only as the *reasoning* behind the current gain/cap choices. **Treat it as design INTENT / a reference,
NOT as an established or "correct/sound" architecture** — it has NOT produced yaw compliance, so nothing here is proven:
- **cap = the torque lock** = the intended compliance/force ceiling (above it the joint is meant to yield, FARMS-muscle-like).
- **PD gains = the track-quality knob** = within the capped budget, how tightly the joint is meant to chase the command;
  the gains were set high FOR THIS STATED REASON (commit the capped torque toward tracking).

**Do NOT assert either "high gains are correct" OR "lower the gains" — BOTH are unmeasured.** (An earlier auditor asserted
"lower the gains"; that was unverified. Do not swing to the opposite unverified claim that "high gains are right." The PI
gave the cap/gains split as *reference only*.) The only way to know how gains-vs-cap actually shape tracking/yield is to
MEASURE (per-DOF saturation fraction + tracking error, §6). `body_yaw` computes to ~3.6× over-cap (a rough calc, unverified);
regardless, **yaw does not conform to terrain** — the rationale has not delivered the goal.

---

## 3. CURRENT HAND-TUNED OPERATING POINT (fragile; "还不错" per PI) — the live CONFIG

`scripts/analysis/tune_control_panel_2026-07-15.py` CONFIG (2026-07-15, PI hand-tuned):
- `implicit_pd=True`
- `body_kp=0`, `body_kv=1.532e-3` (yaw driven by kv)
- `leg_kp=[5.559, 4.465, 2.4e-4, 5.86e-3]`  ← legbase_yaw/pitch = **100× env4** (0.0556/0.0447), high ON PURPOSE (track hard
  inside the 0.8 mN·m cap). `leg_kv=[5.8e-6, 1.2e-12, 2.9e-8, 4.3e-4]` (legbase_pitch damping ≈0).
- amps: `body_amp=0.30`, `legbase_yaw/pitch=0.15/0.15`
- passive: `k_pitch_passive=6e-4`, `k_yaw_passive=7.2e-5`, `c_scale_passive=1.5`; `suppress_active_pitch=True`, `keep_head_up=True`
- **per-class caps [N·m]**: body_yaw 8e-4, body_pitch 5e-4, body_roll 5e-4, legbase_yaw 8e-4, legbase_pitch 8e-4,
  legbase_roll 5e-4, knee 5e-4. `actuator_velocity_limit=None`.
- terrain l75a12 seed4242, 3.0 m arena, dt 4 ms / decim 12, 8 s.

**env4 per-class PEAK desired torque [mN·m]** (reference for where caps bite; from `metrics_ampscan.json` env-index 4):
body_yaw **1.085**, body_pitch 0.113, body_roll 0.124, legbase_yaw 0.609, **legbase_pitch 0.729**, legbase_roll 0.014, knee 0.047.

**KEEP the env4 reference parameter set (do NOT lose it).** It is the clean, verified baseline the panel reproduces
bit-identically; the current hand-tuned point above was tuned OUT of it. env4 WALKS + low torque (~1 mN·m) + conforms
vertically (pitch), BUT its **leg motion is insufficient — under-driven / 欠驱动** (small leg amps + base gains), which is
exactly why the PI raised leg gains/amps. Still valuable as reference (the low-torque compliant anchor). env4 values:
- `implicit_pd=False` (explicit); `body_kp=0`, `body_kv=1.532e-3`
- `leg_kp=[0.0556, 0.0447, 2.4e-4, 5.86e-3]` (base, NOT ×100), `leg_kv=[5.8e-6, 1.2e-12, 2.9e-8, 4.3e-4]`
- `body_amp_rad=0.30`, `legbase_yaw/pitch_amp = 0.09 / 0.06`
- passive `k_pitch_passive=6e-4`, `k_yaw_passive=7.2e-5`, `c_scale_passive=1.5`; `suppress_active_pitch=True`, `keep_head_up=True`
- `actuator_effort_limit_Nm=None` (NO cap), `actuator_velocity_limit=None`; wave_number 3.0, freq 1.0
- result: bl/cycle ≈ 0.166, walks, upright; peaks as listed above. (To rerun: set the panel CONFIG to these = env4 repro.)

---

## 4. A BOUNDARY OBSERVATION at a SPECIFIC over-large leg amplitude (NOT a blocker on the working point)

The current working point WALKS ("还不错"). The 上扬 is **not** a blocker on it — it is simply what happens at ONE specific,
too-large `legbase_*_amp` setting: push the leg amplitude up and **the LEGS pitch up** (legbase_pitch), ground contact drops,
it stops walking. Recorded as the EDGE of the usable amp range so the next auditor knows where it breaks — not a fault that
must be fixed before proceeding. (Separately, `body_amp=0.1` → body ~straight = a stick, tiny wave → no thrust → can't walk =
amplitude-too-small, NOT a stiffness result.)

**Hypothesis (UNVERIFIED) for the 上扬:** a fixed cap gives the trackable amplitude a ceiling (required torque grows with
amplitude; past the cap the joint saturates more of the cycle and lags — gain can't beat the cap). The binding cap is
*plausibly* `legbase_pitch` (lift), but that is a GUESS — the §6.1 diagnostic must confirm which cap binds and at what amp
before anyone acts on it.

---

## 4b. OPEN PUZZLE (measure it — do NOT guess): wave_number 3.0 → legs move well; 1.5 → legs barely move

PI observation on the current setup: at `wave_number=3.0` the legs move well and the gait works; drop to `wave_number=1.5`
and the legs **barely move**. Unexplained — a prime case for RULE 1: run both (3 s test each), dump per-joint leg
q(t)+q_ref(t)+applied-torque+contact, and SEE what actually differs before saying anything.

Tentative hypotheses ONLY (UNVERIFIED — do not assert any):
- wave_number sets the per-segment PHASE gradient of the body wave, and the legs are CPG-coupled to that phase; at 1.5 the
  inter-segment / inter-leg phase lag may collapse toward in-phase → no metachronal leg wave → little net leg motion;
- or the 1.5 body shape changes per-segment loading so the capped leg torque can no longer move the legs;
- or a coupling/phase resonance specific to 3.0.
The mechanism is unknown; the data will show it. This is exactly why the data-first rule exists.

## 5. CANDIDATE LEVERS FOR YAW COMPLIANCE (all UNTESTED — hypotheses)

1. **body_yaw effort_limit saturation** — drive a walking-amplitude wave but cap body_yaw near flat-ground demand
   (~1–1.5 mN·m vs env4 peak 1.09); flat → wave drives; leg/terrain excess → yaw yields/conforms. (Currently 0.8 mN·m =
   ~3.6× over → mild yield; a promising spot but not yet delivering conformance.)
2. **Soft `body_kp` position-tracking** — switch yaw from pure-kv-drive to a SMALL kp tracking the wave POSITION, soft
   enough to be pushed off it → a soft spring around a MOVING target (the true pitch analog; pitch = soft spring around a
   FIXED target).
3. **Decouple feedforward + feedback** (controller change, the "right" fix): command the wave as a **feedforward torque**
   (open-loop propulsion) + a separate **soft feedback impedance** for compliance. Today `body_kv` does both jobs → they
   fight. Splitting them removes the tension.
4. **Make the legs engage** (leg amp / knee bend `leg_dc_offsets[3]`=30° / lift phase) so there ARE lateral leg forces for
   yaw to yield to — precondition, since yaw has no other horizontal load.
5. **Per-DOF cap split** (tool already supports it): raise legbase_pitch (lift) cap so legs clear at higher amp, keep
   legbase_yaw + body_yaw caps low for compliance.

**Important caveat carried from §1–2:** in IMPLICIT mode the passive bio springs are folded into the PhysX drive
(K=Kp+k_passive) and are therefore ALSO clamped by effort_limit; once a joint saturates there is NO residual
stiffness/damping (a saturated drive = constant torque = zero incremental K,C) — motion is then bounded only by inertia +
`actuator_velocity_limit` + position limits/contact, NOT by a spring. So a too-tight cap makes the yaw (and the passive
posture) run away rather than settle. `actuator_velocity_limit` is the knob that bounds post-saturation runaway (currently
None).

---

## 6. DIAGNOSTICS TO BUILD/RUN FIRST (measure, don't guess)

1. **Per-DOF saturation diagnostic**: for each joint class, the fraction of the cycle spent AT the cap, the tracking error
   |q−q_ref|, and the foot-contact timing — swept over leg amplitude (e.g. 0.15 that works vs a 0.25 that breaks). Pinpoints
   which cap (hypothesis: legbase_pitch) binds and when the leg falls behind. This turns §4 from hypothesis to fact.
2. **Disturbance-probe (external impedance)**: FREEZE the gait (amp→0 or freq→0), apply a KNOWN external torque to a
   link/DOF, measure steady δq and the decay transient → K_eff = δτ/δq, C_eff from the transient, yield point from where it
   stops resisting. Sweep ONE param at a time: δq moves ⇒ that param sets external impedance; δq unchanged ⇒ it only shapes
   the reference q_ref(t) (see the param taxonomy below). This directly answers "which knob touches external-force
   compliance and which is inert."

**Param taxonomy — impedance vs reference-only (mechanism-grounded PREDICTION; CONFIRM with the disturbance-probe, don't take as measured):**
- **Sets external impedance** (enters the force-balance δτ=K·δq+C·δq̇+I·δq̈, gated by effort_limit): kp, kv, k_*_passive,
  c_scale_passive, actuator_effort_limit_Nm (yield ceiling), actuator_velocity_limit (post-yield speed bound),
  suppress_active_pitch (toggles whether active PD contributes). (+ mass/armature/joint-friction = intrinsic, not tunable
  in panel.)
- **Reference-only (NO effect on external impedance, only move q_ref)**: body_amp/legbase_amp, freq_hz, wave_number,
  wave_speed, coupling_body/leg, coupling_omega, leg_phase_offsets, leg_dc_offsets, head_* servo, settle_time, ramp_time.
- **Only cross-term**: amplitude interacts with effort_limit — big amp eats the torque budget → less headroom to resist
  external before saturating — but it does not change the linear K/C.
- **Inert in THIS setup**: `body_kp`=0 (spine-yaw impedance = k_yaw_passive + body_kv, NOT body_kp); `pitch_kp/pitch_kv`
  suppressed (pitch impedance = k_pitch_passive + c_scale, NOT pitch_kp/kv). Don't tune the inert ones expecting a change.

**Verification discipline (CLAUDE.md):** data first (RULE 1), differential > absolute, falsify > confirm, measure at the
consumer's interface, one axis at a time from the working point (this operating point is fragile — small steps), render the
3-view only at KEY/representative points (RULE 2) so the collapse MODE is visible where it matters, cite CSV+script:line,
recompute headline via a disjoint path.

---

## 7. TOOLS ALREADY BUILT (ready to use)

- **`scripts/analysis/tune_control_panel_2026-07-15.py`** — one-stop panel. ALL params in one CONFIG (effective physical
  units, each annotated). Run: `conda activate isaaclab_50; cd <proj>; python scripts\analysis\tune_control_panel_2026-07-15.py
  [--duration 3|8] [--run-name X] [--no-video] [--max-frames N]`. Output → `outputs/analysis/tune_panel_2026-07-15/<run_name>/`.
  Features: per-joint effort caps (dict keyed by joint-name substring body_yaw/legbase_*/knee), live `[SIMTIME]` sim-clock
  streamed to the window (`sim_time_report_dt`), parallel 3-view render (top=YAW/side=PITCH/chase=45°), applied-vs-desired
  torque report when a cap is set. Reproduces env4 bit-identically when set to env4 values. Detail: memory
  [[project_tune_control_panel_2026-07-15]].
- **cell `scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py`** — has `--sim-time-report-dt` (SIMTIME counter,
  default off = byte-identical) and per-joint effort/vel/pos limits via `--physx-override-json` (`_mk_limit`, dict-substring
  matched, snapshot `.before_simtimer_2026-07-15`).
- **`scripts/analysis/render_multiview_2026-07-15.py`** — parallel 3-view renderer (`--max-frames` optional).
- Migration: `--implicit-pd` (env `_apply_action_implicit_pd`, controller `set_export_pd`/`_pd_kp`); implicit≡explicit
  parity proven. [[project_farms_implicit_pd_result_2026-07-15]].

---

## 8. STATUS LEDGER

- **OPEN (the job):** yaw horizontal compliance/conformance while walking — UNSOLVED. The current point walks but yaw does
  not conform.
- **PHYSICS-GROUNDED (defensible; re-derive if in doubt, don't treat as dogma):** pitch-vs-yaw asymmetry (§1); the
  external-impedance param taxonomy (§6); implicit≡explicit parity (measured); the effort-limit saturation physics (§5 caveat).
- **UNVERIFIED — PI reference or hypothesis; do NOT state as fact:** the cap=force-lock / gains=track-quality split is the
  PI's RATIONALE, not a proven/"correct" architecture (§2); the legbase_pitch-cap-binds-first guess for 上扬 (§4); every
  lever in §5; the ~3.6× body_yaw over-cap number.
- **Do NOT assert in EITHER direction (both unmeasured):** "lower the gains" and "high gains are correct." Also: "small
  amplitude is the compliance answer" (small amp can't walk). Body-amp 0.1 = stick = amplitude-too-small, not a stiffness result.

**Recommended order for the new auditor:** (0) **§0.5 FIRST — audit the armature + the suspended-in-air 0.2 mN·m leg/body
torque test**; this gates everything (if the armature throttles the legs, no yaw tuning matters). Then (1) §6.1 per-DOF
saturation diagnostic (which cap binds; replace the §4 guess with data) + the §4b wave_number 3.0-vs-1.5 case. Then (2) §6.2
disturbance-probe for yaw's external impedance and test a §5 lever for actual conformance. Always RULE 1 (run + dump data
yourself first), ONE axis at a time from the working point, **3-view video only at KEY/representative points (RULE 2).**
