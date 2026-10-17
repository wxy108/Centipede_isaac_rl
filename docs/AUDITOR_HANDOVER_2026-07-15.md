# AUDITOR HANDOVER — Centipede IsaacLab RL (2026-07-15)

> ## ⚠️ STALE IN PART — the parameter mechanism changed on 2026-07-16 (read this first)
> 
> This page was written **before** the implicit-only unification (S1/S2a, PI-signed; `DISPATCH_LOG.md`
> 2026-07-16T19:34Z). Its description of the runtime parameter chain is **superseded**: the "gains-json values are SCALES on
> `configs/farms_controller.yaml`" indirection is deleted, and the migration listed under "Watch-outs" has
> since been EXECUTED (S2a). The rest (people, rules, how we work) still holds.
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


Entry point for a **new auditor**. Read this top-to-bottom once, then keep it beside `CLAUDE.md`. It answers: who/what,
the rules, how we actually work, where the science stands **right now**, and the **current plan (FARMS controller migration)**.

---

## 0. Who & what
- **PI:** 王熙原 / Wang Xiyuan (middle char 熙, not 西). Email xiyuanwang2235@gmail.com. He drives fast, iterates in
  short cycles, wants **video verification of every step** and **full absolute paths** for every artifact, and gets
  frustrated by stalls or unverified claims. Answer his physical-absurdity signals FIRST, from the raw config/render.
- **Project:** a segmented **centipede** in **IsaacLab / PhysX**, driven by a **Kuramoto-CPG + per-joint impedance (PD)**
  controller ported from **FARMS/MuJoCo**. Long-running goal: biologically-plausible, terrain-conforming, compliant
  locomotion. Repo `C:\Users\wxy22\Documents\Centipede_IsaacLab_RL`, git branch `main`, conda env `isaaclab_50`.
- **Review chain:** auditor (Claude) drafts directives → PI (Xiyuan) signs off in the directive's `DISPATCH_LOG.md` →
  worker executes and returns at ⛳ gates. **No external/ChatGPT gate** (retired 2026-05-21).

## 1. The canonical files to read (in order)
1. **`CLAUDE.md`** — the INVARIANTS: RESULT MARKER PROTOCOL, STANDING WORKER RULES (metrics/analysis, training/GPU,
   forbidden inputs, protected files + the **mount-truncation guard**, WORLD-FIRST verification, video workflow, output
   organization). These OVERRIDE defaults. Do not re-litigate closed branches.
2. **`memory/MEMORY.md`** — the memory index (one line per fact). ~120 memory files under `memory/`. The `## Project`
   section is newest-first; the `## Feedback` section is the standing lessons. **Start with the top of Project + all of
   Feedback.**
3. **This file**, then **`docs/COMPLIANCE_ATTEMPTS_LOG_2026-07-15.md`** (the full attempt-by-attempt journey:
   hypothesis → result → dead-ends → the FARMS conclusion — READ THIS to understand our current thinking), then
   `docs/HANDOVER/README.md` (16 KB overview) + `docs/HANDOVER/STEP_LOG.md` (running log) + `docs/HANDOVER/HOW_TO_SWEEP.md`.
4. `STARTUP_GUIDANCE.md` (how to launch), `docs/PROJECT_STRUCTURE.md` (layout), `docs/OUTPUT_ORGANIZATION.md`
   (every artifact → one dated run dir), `docs/RENDERING_PIPELINE_DEBUG_NOTES.md` (video recipe).
5. **`outputs/CURRENT_RESULT.md`** — the single result marker (what to verify NOW). **Exactly one exists project-wide.**
6. **Skills** (`.claude/skills/`): `centipede-init` (session init — run first), `centipede-state-check` (grep live
   flags/bounds before any edit), `centipede-render-video`, `centipede-sweep`, `world-first-forensics`.

## 2. How we actually work (operational workflow)
- **ONE GPU job at a time.** IsaacLab boots hang 10–22 min on Omniverse teardown → co-launch a watchdog / force-kill
  python on the completion marker. Preserve the pre-existing python PID (a leftover kit process) when killing.
- **Detached long boots + Monitor.** Background Bash tasks get killed erratically. Launch GPU orchestrators **detached**
  via PowerShell `Start-Process` (survives), and watch with the **Monitor** tool (poll a log for the completion marker /
  npz). See `memory/reference_long_run_resume_detached_2026-06-26`. Fine-dt boots are ~10× slower — see
  `reference_finedt_cost_and_ctrl_decim_2026-07-15`.
- **The compliance cell** = `scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py` (boots N envs in one Isaac
  run, per-env gains from a gains-json, dumps `_alllink.npz` poses). Drivers under `scripts/analysis/*_2026-07-*.py`
  wrap it (build the gains grid, boot, measure, aggregate). `a004_gains()` (in `compliance_sweep_2026-07-13.py`) is the
  base config; gains-json values are SCALES on `configs/farms_controller.yaml`.
- **Offline video** = `scripts/sanity/offline_render_from_log_2026-06-22.py` (CPU/MuJoCo, from `_alllink.npz`). Gotchas
  (cmd eats commas → call isaaclab python directly; top-view needs dist-k ~5.5; full-body = dist-k 3.5) in
  `reference_offline_render_gotchas_2026-07-15`. Every clip → per-frame audit JSON.
- **Protected edits** (`envs/`, `controllers/`, `assets/`, large cell scripts): NEVER via Edit/Write (mount-truncation
  silently truncates). Use a python patcher (insertion-only) → `scripts/sanity/safe_write.sh` (bash temp → AST + checks →
  atomic mv, `--snapshot TAG`). safe_write's `python3` AST check fails on UTF-8/Chinese → verify AST yourself with
  `io.open(...,encoding='utf-8')` and pass `--no-ast`. See `reference_safe_write_utf8_shim_2026-07-06`.
- **Result marker discipline:** producer overwrites `outputs/CURRENT_RESULT.md` on EVERY new result; auditor verifies per
  IMPORTANCE (P0 full Rule-1 / P1 disjoint recompute + CSV+script:line / P2 spot-check), then STATUS VERIFIED|RETRACTED.
- **When in doubt, use a Workflow** (ultracode) for fan-out research/review; but GPU sweeps are serial, so orchestrate
  the CPU-side (reading, rendering, analysis) and run the one GPU boot inline.

## 3. Where the science stands NOW (the compliance → conformance → FARMS arc, this session)
The recent thread is all about making the body **compliant + terrain-conforming** without buckling:
- **body_yaw bio-calibrated** to k=7.2e-5 Nm/rad (side-lying gravity droop, 6-tail bend 126.87°); amp sweep peaks at
  body_amp≈1.5. `project_body_yaw_calibration_2026-07-14`.
- **Whole-body terrain conformance = LEG stiffness** (multi-spawn): soft VERTICAL legs (hip+knee ×0.08, legbase_yaw
  stiff) let the body drape (corr body↔terrain 0.46→0.67) while still walking; all-legs ×0.1 conforms best (0.88) but
  breaks walking. **Front pitch-up = body PITCH kv (passive c_scale), which was 0 (undamped)** → c_scale≈1.5 cuts it 8×
  (U-shaped; >4 over-damps). Torque cap does NOT fix conformance or head-up.
  `project_terrain_conformance_leg_stiffness_2026-07-14`.
- **THE PIVOTAL FINDING → the current plan:** we struggle to be compliant because we push an **EXPLICIT torque PD**
  (`set_joint_effort_target`, actuator stiffness=damping=0), which is only conditionally stable — soft kp buckles /
  needs fine dt + velocity limits. **FARMS is compliant because it uses an IMPLICIT solver-side position PD** (MuJoCo
  position+velocity actuators, position targets), unconditionally stable → arbitrarily soft.
  → **`project_farms_position_controller_migration_2026-07-15`** + `reference_farms_repo_2026-07-15`.

## 4. THE PLAN — FARMS controller migration (PI-stated 2026-07-15)
**Migrate our actuation from explicit torque to FARMS-style implicit-PD position control**, behind a default-OFF flag:
- Set the active-joint `ImplicitActuatorCfg` `stiffness=kp`, `damping=kv` and drive the CPG wave via
  `set_joint_position_target(q_ref)` instead of `set_joint_effort_target(tau)`. PhysX then solves the PD implicitly.
- **The direct test:** A/B at LOW kp — if buckling/instability vanishes at 4 ms with NO velocity limit, hypothesis proven.
- **Watch-outs:** re-express passive bio-springs (pitch k=6e-4, yaw k=7.2e-5), suppress_active_pitch / keep_head_up /
  head_pitch_offset, forced-wave, effort/vel limits, coupling scales as position-targets + per-joint stiffness/damping.
  Controller is FROZEN — flag-gated, snapshot, protected-file rules. Full spec in the migration memory.

## 5. Open threads / immediate next steps
1. **Build + A/B the implicit-PD migration flag** (the plan above) — the current top priority.
2. Reproduce the sweet-spot gait under implicit PD; confirm compliance without the fine-dt / velocity-limit crutches.
3. Then re-run the conformance + pitch-kv sweeps under implicit PD (they should be cleaner + faster).
- Deferred: `outputs/` cleanup (task #8); the broader Stage-E feedback-rate RL is on hold behind the compliance work.

## 6. Standing corrections (never cite the retracted versions)
See CLAUDE.md → "Known retractions". Highlights: per-block tilt = +0.143 (not +0.83); impedance = velocity-error damping
τ=kp(q_ref−q)+kv(q̇_ref−q̇); rank ≠ magnitude (require wave-span ≥0.30 with any Spearman); the INVISIBLE-FLOOR lesson
(inventory the runtime world before analyzing data; a violated invariant means an assumption is false).
