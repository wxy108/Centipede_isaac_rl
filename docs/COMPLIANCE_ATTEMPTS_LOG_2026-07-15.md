# Compliance / Terrain-Conformance — full attempts log (→ FARMS migration)

Complete chronological record of the compliance push (why we tried each thing, what happened, dead-ends), so a new
auditor can follow the reasoning end-to-end. Companion to `docs/AUDITOR_HANDOVER_2026-07-15.md` (the entry point) and
`outputs/RUN_INDEX.md` (the raw run log). Every run has a dated `outputs/analysis/<name>/` dir + memory topic file.

## The question
Make the centipede's body **compliant** — soft enough to drape over / conform to undulating terrain — **without buckling
or going numerically unstable**, while still walking. This is the whole arc below; it CONVERGED on: our controller can't
do it stably because it uses an EXPLICIT-torque PD, and the fix is to migrate to FARMS-style IMPLICIT-PD position control.

## Self-pace / working rules in force (the PI grants short autonomous cycles)
- WORLD-FIRST: inventory the actual runtime world (colliders/actuators/limits) BEFORE analyzing data; a violated
  invariant means an assumption is false. (memory `feedback_selfpaced_autonomous_session_rules_2026-07-12`,
  `feedback_verify_function_not_artifact_2026-07-12`, CLAUDE.md WORLD-FIRST).
- Every result: recompute headline via a DISJOINT path (V2), cite CSV + script:line; multi-spawn before any spawn-
  sensitive claim; honest scoping; retraction-free; no forbidden inputs (severity/block_id/x-pos/LUT in obs/reward).
- Video verification of EVERY step; FULL absolute paths always; timestamp (local+UTC) each status.
- ONE GPU job at a time; detached boots + Monitor; safe_write for protected/large edits; one result marker project-wide.

## Chronology of attempts (each = a dated run dir + memory file)

| # | Attempt (hypothesis) | Method | Result / verdict |
|---|---|---|---|
| 1 | Controller-dominance: does the controller (kp/kv) set effective stiffness/damping? | isolated 1-DOF ring-down, all 7 joint families | YES — k_eff=kp, c_eff=kv, ζ_eff/ζ_cmd≈1.02 over 3.5 decades; armature 1e-5 dominates inertia; continuous, no switch. So compliance IS a controller knob. `project_controller_dominance_pitch_2026-07-13`. |
| 2 | Titration: lower gains until it can't walk, then rescue with amplitude | gain 1.0→0.01, then amp 1→8× | ~5–6× compliance headroom (walks to gain~0.18); amplitude CANNOT rescue a dead-low gain (excess amp buckles). `project` (titration). |
| 3 | Overnight rescue: can solver / dt / joint-limits rescue low-kp+high-amp? | isolated axes on 9 buckling configs, dt 1ms→0.01ms, multi-seed | **VELOCITY LIMIT (1.2× a004) rescues** (8/9, multi-seed); solver/force/position DON'T; **dt 1ms==0.5ms buckle (dt NOT the lever, buckle is PHYSICAL)**. `project_compliance_rescue_overnight_2026-07-13`. |
| 4 | body_yaw bio-calibration (make yaw SOFT like pitch) | side-lying gravity droop, fixedbase weld, sweep k to hit 6-tail 126.87° | **k_body_yaw = 7.2e-5 Nm/rad** (both analytic candidates WRONG — PI right). Then amp sweep: peak body_amp≈1.5. `project_body_yaw_calibration_2026-07-14`. |
| 5 | Amp sweep REDO (PI 4 fixes: freq1.0, lower kv, knee bent, rough terrain) | kv_mult×amp grid, multi | Lower kv BETTER: kv×0.15 rises to 0.413 bl/cyc (beats viscous a004 0.358); knee locked bent (stiffen leg_kp[3]=8 + raise knee vel-cap). `project_body_yaw_calibration` (redo section). |
| 6 | "body 上扬" — head-up too much? | head_pitch_offset sweep −0.25→+0.08; all-joint torque-cap sweep (16 levels) | head-up = fixed head_pitch_offset (−0.25→+12.5°); **torque cap does NOT change it** (flat +12.5° across all caps). Not the whole-body issue the PI meant. |
| 7 | Whole-body terrain conformance = LEG stiffness | leg-compliance sweep, multi-spawn, corr(body↔terrain) metric | **Soft legs → body drapes** (corr 0.46→0.88); but all-soft breaks walking. **Vertical-only (hip+knee ×0.08, legbase_yaw stiff) → corr 0.67, walk 100%** (decouples conform vs walk). `project_terrain_conformance_leg_stiffness_2026-07-14`. |
| 8 | Front pitch-up = body PITCH kv (was it 0?) | READ code: pitch = suppress_active + passive spring c=c_scale·k, c_scale=0 → kv literally 0. Sweep c_scale | **pitch kv was 0 (undamped) → front overshoots on bumps**; c_scale≈1.5 cuts it 8× (+0.98→+0.12mm); U-shape (>4 over-damps). Same memory. |
| 9 | Fully-undamped (body_kv=0 + pitch c_scale=0) video at fine dt 0.1ms | 1 boot | STABLE at 0.1ms (upright 0.98, no NaN). BUT 5s@0.1ms ≈ 60min (physics-bound). `reference_finedt_cost_and_ctrl_decim_2026-07-15`. |
| 10 | Open-loop speedup: `--controller-update-decim` (recompute ctrl every K substeps, hold) | cell flag, safe_write, A/B | Only ~1.75× (controller was ~40% of per-step cost; **PhysX solver dominates**). Confirmed 1ms is stable + same behavior → iterate at 1ms. Same memory. |
| 11 | **WHY can FARMS be compliant and we can't?** (PI: read the FARMS repo) | read FARMS `mjcf.py`/`task.py`/`fix_motors.py` + our controller + actuator cfg | **THE ANSWER:** FARMS uses MuJoCo **implicit** position+velocity actuators (position target → solver-side PD → unconditionally stable → arbitrarily soft). We push **explicit** torque (`set_joint_effort_target`, actuator stiffness=damping=0 → conditionally stable → soft kp buckles + needs fine dt). `project_farms_position_controller_migration_2026-07-15`. |

## Two distinct failure modes (do NOT conflate)
1. **NUMERICAL** — explicit `kv·q̇` (held over the substep) injects energy → needs tiny dt. This is what forced 0.1ms +
   velocity limits. **Implicit PD kills this.**
2. **PHYSICAL** — at genuinely low kp the soft springs can't hold the slender column vs gravity+wave → Euler-buckling;
   and the CPG is OPEN-LOOP (no contact/posture feedback) so nothing recovers. FARMS operates at soft-but-sufficient kp.

## The current sweet-spot gait (what the migration must reproduce/beat)
passive soft body_yaw k=7.2e-5 + active body_kp=0; body_kv 0.15×a004 (=0.413 scale); body_amp 5.0 (=2.0 rad); freq 1.0Hz;
keep_head_up (head_pitch_offset −0.25); knee stiff bent 30° (leg_kp[3]=8); pitch kv c_scale≈1.5; vel-1.0×; rough l75a12.

## THE PLAN (PI 2026-07-15) — FARMS controller migration
Migrate actuation from explicit torque to implicit-PD position control (flag-gated, default OFF, snapshot):
set active-joint `ImplicitActuatorCfg.stiffness=kp, damping=kv` and drive via `set_joint_position_target(q_ref)` instead
of `set_joint_effort_target(tau)`. **A/B at LOW kp: if buckling/instability vanishes at 4ms with NO velocity limit, proven.**
Watch-outs: re-express passive bio-springs, suppress_active_pitch/keep_head_up/head_pitch_offset, forced-wave, effort/vel
limits, coupling scales as position-targets + per-joint stiffness/damping. Full spec:
`memory/project_farms_position_controller_migration_2026-07-15.md`, `memory/reference_farms_repo_2026-07-15.md`.

## Dead-ends / do-NOT-re-open
- Torque cap (leg or all-joint) does NOT fix conformance, head-up, or ride-height while walking.
- Solver type / iterations, force limit, position limit — none rescue low-kp buckling (only the velocity limit does).
- dt is NOT the lever (1ms==0.5ms buckle) — buckle is physical, not numerical.
- head_pitch_offset controls head-up but that's ONE joint, not the whole-body conformance.
- The fling arc + emergence/basin/P4 arc are CLOSED (see MEMORY.md archive sections).
