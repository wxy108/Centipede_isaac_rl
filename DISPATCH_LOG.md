
## 2026-07-04 — GO_BIO_PITCH_RETEST_AND_SWEEP (AUTONOMOUS, NO SIGN-OFF)
- PI-approved in-session 2026-07-04 (Xiyuan): self-paced, run all 3 stages back-to-back, NO sign-off / NO gate returns this round; on any unsolvable blocker LOG + SKIP + continue, never stop.
- Changes from draft: bio-pitch-k = 9.93e-3 (real-dims install pitch_down); Stage 2 ALWAYS re-optimizes (warm-start from v13 grid results), fallback to best prior design only if none meets no-collapse+small-fling; Stage 3 terrains lambda = 120,90,60,30.
- Directive: outputs/directives/GO_BIO_PITCH_RETEST_AND_SWEEP_2026-07-04/DIRECTIVE.md

## 2026-07-04 (later) — GO_BIO_PITCH_OPT_AND_FORCE_SWEEP (AUTONOMOUS, NO SIGN-OFF)
- Completes the prior run gaps: (1) REAL optimization with the bio spring ON (prior only re-scored 6 v13 winners, no new params); (2) force-capturing sweep (prior saved kinematics only). Cell repaired by auditor first. Same no-sign-off self-paced policy + NEW mandatory write-verification (bash py_compile after every edit) and force-key assertion.
- Directive: outputs/directives/GO_BIO_PITCH_OPT_AND_FORCE_SWEEP_2026-07-04/DIRECTIVE.md

## 2026-07-04 (later) — GO_GROUP_MEETING_PACKAGE (AUTONOMOUS, NO SIGN-OFF)
- Produce a comprehensive group-meeting deliverable: curated video set + full figure set + slide-by-slide narrative report covering the whole project arc (goal → wave-optimum/force → fling root cause → bio-pitch negative → A005 frontier). Mandatory video discipline (canonical pipeline, per-frame audit, offline-first, time-ramp control). Honest caveats kept throughout.
- Directive: outputs/directives/GO_GROUP_MEETING_PACKAGE_2026-07-04/DIRECTIVE.md

## 2026-07-04 (later) — GO_DEFERRED_BACKLOG (AUTONOMOUS, NO SIGN-OFF, SELF-PACED)
- Worker double-checks the auditor-synthesized deferred-experiment backlog against the LIVE repo (STAGE 0, resolve the knee/contact-compliance contradiction), then works P1..P11 in priority order self-paced to completion. Top: P1 info-theory (data ready) + P2 adaptive wave dynamics (the parked thesis). Protected-edit / human-input items = design+flag only. Session hard-lessons baked in (write-verify, force-key assert, one-GPU+watchdog, N_ENVS cliff).
- Directive: outputs/directives/GO_DEFERRED_BACKLOG_2026-07-04/DIRECTIVE.md

## 2026-07-04 (later) — GO_DEFERRED_BACKLOG SUPERSEDED by v2 (adversarial review)
- Two independent adversarial red-teams found v1 unshippable: P2 "wave emergence" un-testable (no endogenous adaptation; Pattern-A = hand-set reflex; RL reset), P1 cited a non-existent spec + normal-only sensor + N=5 → artifact, P6 already done (knee weak −5.4
## 2026-07-05 - GO_A008_OVERNIGHT_PACKAGE (AUTONOMOUS, OVERNIGHT, NO SIGN-OFF)
- Expanded overnight campaign built on the C1-verified finding (A008 Pareto-dominates d0086). Blocks: A A008 multi-seed lock validation + variance budget; B A008 terrain-wave FORCE sweep (real candidate); C A008 video package; D bio completion (kv damping + drop + clamped droop + video); E design specs for the deferred protected-edit items (asym spring / duty-factor / friction-PhysX). Adversarial-review CUT items stay cut. Per-block boot caps + ~120-boot session ceiling; no shared-cell edit; force-key assertion; multi-seed + variance budget; marker hygiene.
- Directive: outputs/directives/GO_A008_OVERNIGHT_PACKAGE_2026-07-05/DIRECTIVE.md

## 2026-07-05 - GO_WEEK_ARCHIVE_AND_GROUPMEETING_UPDATE (AUTONOMOUS, NO SIGN-OFF)
- Archive the week (2026-06-29..07-05) + assemble a complete, video-heavy, results-heavy, HONEST group-meeting UPDATE package. PART1 archive FILE_MAP (status-tagged); PART2 hero video gallery (fling event, free-body wobble, terrain locomotion, wave-number sweep, A005-vs-d0086; droop blocked -> bio GoPro instead); PART3 figure/results gallery (fling=external 3-way, cancellation, detector, bio-calib, wavelength wave-optimum + force-why design-general, fling<->fidelity front + 3-seed no-clean-winner correction); PART4 slide-by-slide honest update report; PART5 index. .pptx assembled after by auditor/PI. Video discipline + write-verify + no shared-cell edit + marker hygiene baked in.
- Directive: outputs/directives/GO_WEEK_ARCHIVE_AND_GROUPMEETING_UPDATE_2026-07-05/DIRECTIVE.md

## 2026-07-05 - GO_CALIBRATION_COMPLETE_AND_CLEAN (AUTONOMOUS, NO SIGN-OFF; PI authorizes the minimal gated protected edit)
- PI: complete + clean the joint-stiffness calibration (esp. backbone body-pitch) with a real-sim VIDEO comparison, plus the other loose ends. PART A real-sim clamped-droop + side-by-side bio-GoPro video + resolve model-geom 3.92e-4 vs real-dims 9.93e-3 k; PART B kv/damping settle + 25cm splat-drop (gated --spawn-height) + asymmetric Up!=Down spring install (gated); PART C one clean CALIBRATION_DOSSIER (all joint stiffnesses, installed-vs-default, reconcile stale numbers); PART D fold into the group-meeting update. Protected edits ONLY behind default-OFF flag + snapshot + flag-OFF byte-identity; no shared-cell edit; video discipline.
- Directive: outputs/directives/GO_CALIBRATION_COMPLETE_AND_CLEAN_2026-07-05/DIRECTIVE.md

## 2026-07-05 - GO_GROUPMEETING_UPDATE_MASTER (INTEGRATED; supersedes the 2 standalone drafts)
- Merged GO_CALIBRATION_COMPLETE_AND_CLEAN (->STAGE 1) + GO_WEEK_ARCHIVE_AND_GROUPMEETING_UPDATE (->STAGE 2+3) into ONE chain: calibration completion (real-sim droop video + dossier + loose ends, gated protected harness authorized) -> week archive -> the SINGLE group-meeting update package that INCLUDES the calibration. No double-packaging. Both standalone drafts marked folded.
- Directive: outputs/directives/GO_GROUPMEETING_UPDATE_MASTER_2026-07-05/DIRECTIVE.md

## 2026-07-05 - GO_DROOP_CALIBRATION_FIX (AUTONOMOUS, NO SIGN-OFF)
- Completes/corrects the STAGE-1 calibration: the "sim rigid, can't droop" was an ARMATURE artifact (armature=1e-4 ~110,000x the ~9e-10 physical inertia; stiffness=damping=0 so not rigid). Corrected droop test = physical armature (droop-test-only gated override) + TRUE fixed-base clamp (not airborne re-pin) + damping for a FAST settle; success = reproduce the bio calibration angles (139/128/96 deg) AND settle fast (real centipede is sub-second, not 10s); discriminate k 3.92e-4 vs 9.93e-3; the real droop video + dossier + deck SL-D update. BASH-VERIFY every edit (the truncation just corrupted a protected env). Broader flag: armature dwarfs physical inertia ~1e5x -> model review.
- Directive: outputs/directives/GO_DROOP_CALIBRATION_FIX_2026-07-05/DIRECTIVE.md

## 2026-07-05 - GO_DROOP_CALIBRATION_FIX amended: INDEPENDENT re-modeled droop fixture
- PI: the droop sim must be an INDEPENDENT, re-modeled FIXTURE (a true fixed-base clamp), not a hack on the locomotion env. Route A = fixed-root IsaacLab articulation variant (fix fix_root_link / weld proximal to a world anchor); Route B = standalone rigid-body cantilever integrator (pure-yaw precedent) with REAL per-segment inertia (~9e-10, NOT the 1e-4 locomotion armature). Passive law k(q0-q)-c*qdot; calibrate c for the ~3-4s clay-like overdamped creep (cold-anesthetized data); success = reproduce bio angles 139/128/96 AND the ~3-4s creep; discriminate k. Root cause reframed to MULTIPLE candidates (no-true-clamp likely dominant for k>0, armature for k=0) - not "just armature" (auditor overshoot, corrected).

## 2026-07-06 - GO_DROOP_CEILING_DECIDER (PENDING_PI_SIGNOFF)
- Adjudicate the droop-ceiling cause: -closeSF "k_eff floor" disputed by BOTH auditors (independent chains). 3 causes killed offline (pendulum d=0.1-0.2mm; creep — settled k7e-5 cell still capped; twist — planar 0.00mm). Standing prediction (both): locked-leg-forest SELF-COLLISION wall. T0 runtime probes (self-collision flag/limits/drive readback + direct contact observation per auditor#1) -> T1 one-factor ablation (self-collision OFF / legs folded / settle-hardened / optional MuJoCo contype=0). Prediction ledger recorded in the directive.
- Directive: outputs/directives/GO_DROOP_CEILING_DECIDER_2026-07-06/DIRECTIVE.md

## 2026-07-06 - GO_FRICTION_DIRECT_V2 (PENDING_PI_SIGNOFF; supersedes GO_FRICTION_DIRECT_CLOSE)
- HARD RULE (PI-ratified): friction measurement is WHOLE-BODY or nothing — the COM-level residual has no per-body decomposition, so feet-only has no valid rough comparator (and misses the ~30% non-foot contact); feet-only = smoke test only, never reported. T1 re-apply preserved hook via safe_write (Edit tool forbidden), filter BOTH global prims, assert filter_count>=1, 120s watchdog os._exit(3). T2 flat-then-rough step-match vs B0 residual. T3 Coulomb backstop absorbed from auditor#1 (mu=1.6 from sim_params.yaml, bound+slide-equality+anti-alignment). Binary outcome; no middle state.
- Directive: outputs/directives/GO_FRICTION_DIRECT_V2_2026-07-06/DIRECTIVE.md

## 2026-07-06 - GO_INVISIBLE_FLOOR_FORENSIC SIGNED + DISPATCHED (PI)
- Invisible MJCF-inherited CollisionPlane in worldBody (purpose=guide, collidable, z=0): Block 0 confirm all 3 USDs; Block 1 prove locomotion/friction clean (env deactivation); Block 2 droop-rig floor probe + floor-removed decisive rerun vs auditor#2's pre-registered P-A..P-E; Block 3 cleaned USD (no deploy swap without PI). Incident postmortem -> CLAUDE.md WORLD-FIRST VERIFICATION + .claude/skills/world-first-forensics.

## 2026-07-07 - GO_TARSUS_FIX_AND_DEMO — EXECUTED; premise (roots=2) REFUTED, returned at blocker
- Worker ran Task 1a (confirm the auditor's roots=2 fix). REFUTED by 2 direct env boots (worldBody deactivated, still hung
  at articulation-init, GPU 100%, no completion; n=2 flat hangs same as n=16 → intrinsic single-articulation pathology, not
  spawn-harness). Task 1b dropped (roots=2 refuted), Task 2/3 BLOCKED. Elimination re-diagnosis + recommendation in
  outputs/deliverables/tarsus_fix_and_demo_2026-07-07/MANIFEST.md. Marker RESULT-2026-07-07T1944Z-tarhang (UNVERIFIED).
  Side-fix (worked): renamed USD copy for resolve_asset_override. Controller 256c8732 INTACT; no protected edit. Recalib
  k=6e-4 STANDS. NEEDS PI: model-level redo decision (weld-distal bisection / leaner distal design) — out of "fix-spawn" scope.

## 2026-07-08T02:37Z — DISPATCH: GO omega-terrain sweep (temporal-frequency ω × terrain)
PI handed the auditor-drafted "Temporal-frequency (omega) × terrain sweep" directive to the worker (Claude Code) for
autonomous execution. Chain: auditor drafts → PI signs (this entry) → worker executes, returns at ⛳ gates.
Scope: ω=`--freq-hz` (NO protected edit) × terrain × wave on the a008 apparatus (USD=centipede_rebuild_v1_renamed.usd
= the 4-DOF model I proved G1 bit-identical for earlier today — controller now b724eff9, was 256c8732; ⛳1 parity is the
end-to-end G1 confirmation). Phase-1 CORE 8 boots + ceiling ≤3 = ≤11. STOP at ⛳2 before any expansion. All speed bl/cycle.
⛳1 grep-first PASS (CELL applies --freq-hz via set_frequency :492/:570; emits bl_per_cycle/body_amp_ratio/track_err).

## 2026-07-08T05:56Z — ⛳2 RETURN: omega-terrain sweep complete
9 boots on the a008 4-DOF apparatus. ⛳1 parity PASS bit-identical (blockA G1 confirmed end-to-end). Ceiling mild 2.5Hz /
rough 2.0Hz; optimal-ω rough ~1.5Hz (bracketed), mild edge; ANOVA terrain-dominated, ω separable from wave. HANDED TO
AUDITOR at ⛳2 — no grid expansion / no low-freq follow-up until sign-off. Deliverables in omega_terrain_sweep_2026-07-07/.

## 2026-07-16T19:34Z — PI SIGN-OFF: IMPLICIT-ONLY UNIFICATION (supersedes the pending GO_ARCHITECTURE_RESET P1 scope)
PI (王熙原) signed off IN CHAT, explicitly: **"我签字 ... 删掉死分支 ... PI指令高于claude.md ... 要删就删除干净,再做test确定
bit identical ... 正式转向implicit不再使用任何explicit的controller ... 以后调参都用统一的implicit参数入口 ... 我给你最高
权限,做好备份放手去做 ... 一个关节就两个,kp和kv,直接喂进implicit参数,作为controller唯一的参数源"**.
PI ALSO RULED: **"不要有所谓的被动弹簧"** (no passive spring — every joint's impedance is the implicit Kp/Kv only) and
**"为什么Kp body yaw是被动弹簧?不是应该和kv一样tracking wave吗?"** (Kp must track the CPG wave, not q0).

SCOPE (PI-authorised protected-file surgery on envs/ + controllers/; CLAUDE.md's flag-OFF-bit-identity rule is
EXPRESSLY overridden by the PI for this change — recorded here per the CLAUDE.md chain):
  S1. DELETE the explicit torque path (env `_apply_action` explicit branch + the dead suppress/passive tau compute +
      `_passive_spring_torque`). Implicit becomes the ONLY force path. TEST: implicit results BIT-IDENTICAL before/after.
  S2. UNIFY the parameter entry: ONE joint family -> exactly (Kp, Kv) in PHYSICAL units, written straight to PhysX.
      Kills: the passive spring (`enable_passive_spring`/`--bio-*-k`/`--passive-c-scale`), `suppress_active_pitch`,
      the gains-json SCALE-on-YAML-base indirection, and the head's DOUBLE parameterisation
      (`head_Kp_eff = head_pitch_kp 0.05 x pitch_kp_scale 0.596` -> ONE `Kp_head_pitch`).
      Body yaw Kp moves from the PASSIVE channel (rest = q0, fights the wave) to the ACTIVE channel (rest = CPG wave).
      CPG stays the SOLE reference generator (targets only), FROZEN.
  S3. SWEEP toward the FARMS operating point (small Kp + small Kv + small tau_max).

BLAST RADIUS SURFACED TO PI BEFORE CUTTING (auditor): 182 env-using scripts, only 13 implicit-aware => **169 would
silently flip explicit->implicit**, and implicit != explicit is MEASURED (bl -37.2% at amp_mult 1.0, marker
RESULT-2026-07-16T0350Z-cap2x2). Includes the droop rigs that produced the LOCKED bio_pitch_k=6e-4, the fling arc,
Stage E, scripts/rl. DECISION (auditor, pending PI objection): delete explicit COMPLETELY but make a non-implicit caller
**FAIL LOUDLY** rather than silently run implicit — satisfies "删除干净" AND "以免后面的simulation出错".

FARMS REFERENCE (verified from source this session; full clone, positive controls PASS 241 src / 5 animat.yaml):
  centipede `limits_torque: [-2e-4, +2e-4]` = **0.2 mN*m**, the UNIQUE torque limit in the file; motor block is
  `control_types:[velocity,torque]`, **`gains:[0,0]`**, `equation: ekeberg_muscle` => ALL force from the muscle, saturated
  at +-2e-4. dt=0.001. Ekeberg: alpha=5e-5 beta=1e-8 gamma=1e4 **delta(damping)=1e-6**.
  => muscle stiffness ~1e-4 ~= our Kp_body_yaw 7.2e-5 (MATCH); muscle damping 1e-6 vs our Kv_body_yaw 1.532e-3 = **1532x**.
  PI CONFIRMED CORRECT on "0.2mNm 就够驱动" — it is FARMS's exact cap.

BACKUP: outputs/analysis/implicit_unify_2026-07-16/backup_pre_surgery/ (6 files, md5 recorded); envs/ + controllers/ are
also git-tracked. The cell is UNTRACKED — the backup is its only safety net.

## 2026-07-16T20:40Z — S2 PRE-GATE DISCHARGED (q0 probe): verdict S2_SAME_PHYSICS (rest pose only)
The one path by which S2 could silently change physics (marker RESULT-2026-07-16T2020Z-s1imp OPEN/NEXT) is now
MEASURED, not assumed. `scripts/analysis/q0_probe_2026-07-16.py` (read-only, no stepping; runtime boot of the live
USD `centipede_rebuild_v1_renamed.usd`, 211 DOF) -> `outputs/analysis/implicit_unify_2026-07-16/q0_probe.json`.
  (a) MEASURED (runtime `robot.data.default_joint_pos`): **q0 == 0 on ALL 211 DOF** (`n_nonzero_dofs: 0`), every
      family incl. the 19 suppressed spine-pitch. The USD bakes NO non-zero default joint angle.
      Independent corroboration: knee q0 = 0 agrees with the LOCKED memory that the 30 deg knee bend is ACTIVE
      (`leg_dc_offsets[3]`), not baked geometry.
  (b) SOURCE-READ (not a runtime measurement — flagged as such): the CPG's target for those same 19 joints is
      identically 0 in BOTH pos and vel. `pitch_target_final = zeros(n_p)` with only the first `head_pitch_joints`
      (runtime-verified = 1) set to `head_pitch_offset` (controller:455-457); the per-step modulation is a
      MULTIPLICATIVE blend (`pitch_targets = head_blend * pitch_target_final`, :1289) so zero stays zero; the vel
      target is the derivative of a constant zero (:1293) = 0. => no pitch wave is hidden behind today's Kp=0 mask.
      (This mattered: today Kp=0 MULTIPLIES the CPG pitch target away (:1183); S2's Kp>0 would UNMASK any wave there.
      There is none.)
  => Both channels rest at the SAME point (0). S2 does NOT shift the rest pose and does NOT unmask a hidden wave.
SCOPE LIMIT — what this gate does NOT establish: S2 still CHANGES those joints' STIFFNESS by design (today
  K = 0 + bio_pitch_k 6e-4, C = 0 + c_scale*6e-4 via the fold; after S2 they get Kp/Kv_body_pitch as free params).
  Same rest point != same stiffness. The equivalence must still be MEASURED S1-style (before/after differential with
  matched numbers), not argued.
FIX en route: the probe's first run (16:31) died `ModuleNotFoundError: No module named 'tasks'` — it did `os.chdir(ROOT)`
  but never `sys.path.insert(0, ROOT)` (the idiom every working script uses). Path bug, not physics. Fixed + rerun clean.

## 2026-07-16T20:5xZ — S2a EXECUTED: the single impedance table (env + cell migrated)
PI (in-chat, this session): **"我不是说了全部清掉，一个关节只留一组kp kv作为PD gains，其他全去掉"** + **"每一阶段跑完之后
独立新开一个fresh memory的subagent审计"** (clarified: **大的阶段** — major stages, not every patch).

WHAT S2a DID (protected-file surgery, PI-signed 19:34Z; snapshots + safe_write + independent bash verification):
  envs/centipede_env.py  3858 -> 3934 lines  (snapshot .before_S2a_impedance_table_2026-07-16, md5 pre d5486a40)
    ADDED: `S2_FAMILIES` (8) · `s2_family_ids()` · `set_joint_impedance(table)` — THE single entry: one family ->
      exactly (Kp, Kv) in PHYSICAL units, straight to PhysX. Per-family value may be a scalar (all envs) OR a
      length-E vector (**this preserves the cell's vectorized per-env sweep — nearly broke it; caught by reading
      gains.json's actual structure, which is a per-env LIST**). Missing/unknown family RAISES (no hidden default).
    DELETED: `enable_passive_spring()` + `_passive_{k,c,q0,spring_enabled}` + the K=Kp+k_p / weighted-average-target
      FOLD · `suppress_active_pitch()` + `_suppress_{active_pitch,keep_head_up}` + the pitch zeroing · the env's
      consumption of `ctrl._pd_kp/_pd_kv`. VERIFIED by AST (not grep): 0 live attribute nodes, 0 getattr-by-string;
      the 2 surviving textual hits are a comment + a docstring. AST OK / tail `_t_host = 0.0` intact / 3934 lines.
    GUARD (S1 doctrine, fail-loudly): `_S2_DEAD_SCALES` — if ANY of the 10 `ctrl.*_scale_runtime` != 1.0 the step
      RAISES. Reason: the 6D action's ch-4 `body_imp_scale` reached physics through those; the table is now the only
      gain source, so that modulation would VANISH SILENTLY. (Campaign pins it via --design-a/--design-c; Stage E is
      wave-only => costs nothing today, cannot rot tomorrow.)
  scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py 1433 -> 1438 (snapshot .before_S2a_2026-07-16)
    NEW `--impedance-json` (one table per env). DELETED --passive-spring/--bio-{pitch,yaw,roll}-k/--passive-c-scale/
    --suppress-active-pitch/--keep-head-up (their env methods are gone; --keep-head-up is SUBSUMED — head_pitch is
    its own family now). gains-json carrying any impedance key now RAISES (it would be silently ignored).
    set_freesearch_runtime no longer receives kp/kv (only effort).

THE ACTUAL PHYSICS CHANGE (this is a CHANGE, NOT a refactor — it is NOT bit-identical, by design):
  The S2 table reproduces the baseline fold's total (K, C) number-for-number (arithmetic in the generator), so the
  ONLY variable is the PI's own catch: **body_yaw's Kp (7.2e-5) moves from the PASSIVE channel to the ACTIVE one.**
  Old fold, computed: pos_target = (Kp_act*wave + k_p*q0)/K = (0*wave + 7.2e-5*0)/7.2e-5 = **0**.
  i.e. the spine-yaw spring's rest point was q0 = STRAIGHT — it FOUGHT the wave, and the wave reached yaw ONLY
  through Kv (because gains `body_kp` = 0.0, the CMA-ES trial#167 zero that has stood for months). After S2 the SAME
  7.2e-5 is the joint's own Kp, so its rest point IS the CPG wave. PI: "P 就是 Kp_body_yaw". This is the S2 headline
  and it must be MEASURED, not asserted.
  Also collapsed: the head's DOUBLE parameterisation (head_Kp_eff = head_pitch_kp 0.05 x pitch_kp_scale 0.596) ->
  ONE `head_pitch` Kp = 0.0304.
DIFFERENTIAL RUN (launched 20:5xZ): S2_after vs the S1_after walking baseline, byte-same cell args except the
  swapped flags. seed 4242 / 4 s / dt 1 ms / armature 0 / FLAT. Baseline to beat/compare: bl 0.19792454255039577.
  -> outputs/analysis/implicit_unify_2026-07-16/S2_after/
FLAGGED, NOT FIXED (out of S2a scope, real): under implicit the controller's effort clamp is DEAD — it clamps `tau`,
  which the implicit path DISCARDS. The real cap is PhysX `effort_limit_nm` (--physx-override-json). True since S1.
OWED: the **fresh-memory sub-agent audit** of the whole S2 stage (PI standing rule), + panel migration
  (tune_panel_reset_2026-07-16.py still emits gains-scales + the dead --bio-* CLI => it is BROKEN until migrated).

## 2026-07-16T21:35Z — S2a-EFFORT FIX (PI: "先修再进") — both audit findings CLOSED, refactor PROVEN INERT
PI ordered the effort defects fixed BEFORE S3 (S3's stated lever IS tau_max). Both were REAL and are now fixed.

FIXED #1 — the controller's effort clamp was DEAD under implicit (it clamps `tau`, which the implicit path
  DISCARDS => a run believing it had a torque cap was SILENTLY UNCAPPED). `_apply_action_implicit_pd` now RAISES on
  `ctrl.effort_limit_enabled`. The cell no longer calls `set_freesearch_runtime` AT ALL (0 live nodes, AST-verified),
  and the now-dead gains-json `effort` key RAISES.
FIXED #2 — `--physx-override-json effort_limit_nm` selected joints by NAME SUBSTRING with an
  `unmatched -> max(spec.values())` fallback: the head is named `body_pitch_0`, so head_pitch could NOT be capped
  alone, a `body_pitch` cap ALSO capped the head, and any unmatched joint silently got the LOOSEST bound. That key
  now RAISES outright.
NEW ENTRY — `env.set_joint_effort_limit({family: tau_max_Nm | None})` (envs/centipede_env.py, snapshot
  `.before_S2effort_2026-07-16`; 3934 -> 4000 lines, AST OK, tail intact). FAMILY-EXACT (same enumeration as the
  impedance table), no substring, no fallback; writes ONLY the capped families' joints via joint_ids, so
  `None`/omitted = that family UNCAPPED = nothing written = the walking baseline stays bit-exact.
  **head_pitch is now separately cappable** — the panel's previous refusal is WITHDRAWN.
  Cell: new `--effort-json` (1438 -> 1458). Panel: `build_effort()`, `build_physx` reduced to armature-only,
  gains-json `effort` dropped, CLI emits `--effort-json` (801 lines; --dry-run PASSES end-to-end).
SELF-CAUGHT en route (worth recording): removing the `set_freesearch_runtime` CALL left the `eff7`/`eff_by`/`eff_leg`
  CONSTRUCTION still reading `gains["effort"]` -> KeyError after a full 4 s sim. Dead code that still reads a
  now-forbidden key. Deleted; 0 residual reads.
PROOF THE REFACTOR IS INERT (S1-style, the standard S2a itself could not meet): same cell args, effort all-null
  (uncapped), S2_after vs S2_effort_verify -> **20/20 metric keys IDENTICAL, 0 differing**;
  bl_per_cycle **0.13824056344538743 == 0.13824056344538743** (all 17 digits).
  -> outputs/analysis/implicit_unify_2026-07-16/S2_effort_verify/
STILL OWED before/with S3: hoist the `_S2_DEAD_SCALES` guard out of the per-substep path (10 CUDA syncs/substep);
  guard the YAML-base/direct-attribute false-negative; run_meta/MANIFEST for the P0 runs; the 4 repo-wide live
  callers of the deleted methods; stale `envs/centipede_env_cfg.py:995-996`.
S3 IS NOW UNBLOCKED. The honest lead (from the 4-env sweep, marker RESULT-2026-07-16T2105Z-s2tbl): body_yaw
  Kv 1.6403e-3 -> 1e-6 (FARMS's value) = +23% bl and net_yaw -29.1 -> -7.9 deg, monotonic over 4 points.

## 2026-07-16T22:05Z — OWED ITEMS CLOSED (PI: "先把欠着的做完，已经删除的就删彻底") — S2b + cleanup, both PROVEN INERT
S2b (controllers/impedance_controller_torch.py 1445 -> 1247, snapshot .before_S2b_2026-07-16): deleted the entire DEAD
  gain/torque chain -- `pd_torque` + every pd_torque call + the `tau` tensor + the 10 gain `*_scale_runtime` + their
  setters + every kp/kv vector/scalar + all effort_* clamps + `_pd_kp`/`_pd_kv` + the ROLL block (torque-only) +
  `set_export_pd`/`_export_pd` (targets are now the sole output; the flag gated nothing). `step()` returns **None** --
  there IS no torque; an un-migrated caller dies instead of applying a phantom force. Beyond brief: `joint_pos`/
  `joint_vel` are now dead reads (they were the deleted PD law's feedback terms) -- **the controller reads NO joint
  state at all**; it is open-loop in q and feedback closes in PhysX (2 fewer GPU reads/substep).
CLEANUP: the "4 live callers" was an **11x UNDERCOUNT** -- an AST scan (941 files, attribute nodes AND the
  getattr/setattr/hasattr STRING form) found **44 files / 92 refs**. Fixed at the ROOT, not one-by-one: a
  `CentipedeEnv.__getattr__` tombstone over `_S2_DELETED_MEMBERS` raising **RuntimeError, deliberately NOT
  AttributeError** -- because `getattr(env,"_suppress_active_pitch",None)` SWALLOWS AttributeError and returns the
  default, which is exactly the line that printed `suppress=None`. Guard hoisted to a host-only fingerprint
  (id + torch `_version` + the effort flag) => 0 CUDA syncs on the hot path, one fused sync only when something
  actually changed. Stale cfg:995-996 rewritten; STALE banners on the 3 reader-entry docs (2 further false claims
  found there: "gains are scales" and "`effort` is an absolute cap, always on" -- both now false).
⚠️ MY PREMISE TO S2b WAS WRONG, and it caught it: the YAML gain keys are **NOT** dead. `controllers/
  impedance_controller.py` (the numpy legacy ref) reads the SAME `configs/farms_controller.yaml`, still computes
  torque, and SILENTLY DEFAULTS -- `body_kp` -> 5.0 vs the tuned 0.14396 (**35x**), leg tibia kp **168x**. Stripping
  the shared file would have silently corrupted 7 live scripts. It SPLIT instead: farms_controller.yaml is now
  gain-free (7 target/timing keys) + new `configs/farms_controller_numpy_legacy.yaml` (gains byte-identical,
  7 consumers repointed); BOTH controllers now RAISE on a dead/missing key instead of defaulting.
PROOF (mine, GPU, S1-standard): outputs/analysis/implicit_unify_2026-07-16/S2b_verify/ vs S2_effort_verify, same
  args -> **20/20 metric keys IDENTICAL, 0 differing**. Chain: **S2_after == S2_effort_verify == S2b_verify**,
  bl_per_cycle **0.13824056344538743** across THREE code states. Deleting 198 lines of controller moved nothing.
STANDING-RULE FINDING (upgrades CLAUDE.md's mount-truncation guard): **`ast.parse` is NOT a sufficient syntax check
  -- it ACCEPTED a file `compile()` rejects** (text inserted before `from __future__ import annotations`; a
  future-import must precede all statements). Every "AST OK" in this project's history carries that blind spot. Use
  `compile(src, path, 'exec')`. All 6 protected/live files re-verified with compile() @22:00Z: ALL OK, tails intact.
PI DECISIONS OWED (from S2b): (1) `scripts/render_centipede_demo.py` QUARANTINED not migrated -- it drives
  set_joint_effort_target with its OWN articulation and no env, so it has no gain source; picking one is a PI call
  (Stage-A-era, last touched 2026-05-06, likely superseded). (2) the `impedance:` YAML block name is now historical
  (targets only) -- rename? (3) env `imp_term` reward + `raw/body_imp_scale` log are now structurally always 0 (kept:
  bit-identical, key read downstream) -- they regularize nothing. (4) 50 legacy scripts tombstoned, not migrated.
=> S3 GRID SEARCH LAUNCHED (next entry).

## 2026-07-16T22:35Z — ⛔ S3 GRID INVALID: the torque cap does NOT reach the sim (RED FLAG, not a result)
The 12-cell grid (body_yaw Kv x body_yaw tau_max, 3 boots x 4 envs) ran clean and must NOT be cited. **All three
tau columns are BIT-IDENTICAL** on every metric and every row:
  Kv 1.6403e-3 -> bl 0.13459 / net_yaw -29.139 / peak yaw tau 0.0021343 / upright 0.98302  -- IDENTICAL at
  tau_max = uncapped AND 2e-3 AND 2e-4. Same for all 4 Kv rows.
PHYSICS SAYS THEY MUST DIFFER: row 1's peak yaw torque is **2.13e-3 vs the 2e-4 cap = 10x OVER**. A cap that binds
by 10x cannot leave the trajectory bit-identical. Per CLAUDE.md ("Bit-identical metrics across configs that physics
says must differ = pipeline bug, investigate before accepting") the GRID IS INVALID.
=> `env.set_joint_effort_limit()` writes, but the cap is NOT enforced. The log DOES echo it
   (`body_yaw n=19 tau_max=0.0002`), so the call fires -- the value is not reaching / not surviving to PhysX.
**MY OWN 20/20 BIT-IDENTITY "PROOF" OF THE EFFORT FIX WAS THE FIRST SYMPTOM, AND I READ IT BACKWARDS.** I verified
  the effort refactor ONLY in the all-uncapped configuration, where "inert" is the CORRECT answer -- so it could not
  distinguish "the plumbing is clean" from "the cap does nothing". A positive control needs a cap that BINDS.
  The S2a-effort fix therefore closed the SELECTION bugs (family-exact, no substring, no loosest-fallback) but did
  NOT establish that a cap is enforced at all. The DISPATCH_LOG 21:35Z claim must be read with that scope.
HYPOTHESIS (NOT a conclusion -- untested): `IsaacLab/source/isaaclab/isaaclab/assets/articulation/articulation.py:1707`
  does `self.write_joint_effort_limit_to_sim(actuator.effort_limit_sim, joint_ids=actuator.joint_indices)` --
  IsaacLab re-writes the same quantity from the ACTUATOR's own `effort_limit_sim`, which would CLOBBER our write
  (same class as the impedance path's "write ONCE, persists across reset" note). IsaacLab 5.0 splits
  `effort_limit` (actuator-model clip; inert for an ImplicitActuator, whose torque PhysX computes internally) from
  `effort_limit_sim` (the PhysX drive maxForce). We may be writing the wrong one, or writing it too early.
CONTRADICTS (must be reconciled, do not hand-wave): memory `project_reset_tuning_panel_2026-07-16` records that tiny
  caps (0.5-0.8 mN*m) DID starve the implicit drive (gamma<<1, bang-bang, no walk) via the OLD
  `--physx-override-json effort_limit_nm` path -- i.e. a cap DID bite once. Either that path differed from this one
  (it wrote ALL joints, unfiltered, no `joint_ids=`), or that memory's mechanism is wrong. RESOLVE BY MEASUREMENT.
NEXT (the honest order): (1) read back `robot.data.joint_effort_limit` (or the actuator's) AFTER reset and compare to
  what was written -- that is the disjoint check that settles clobber-vs-wrong-quantity in one boot; (2) try
  `effort_limit_sim` / the actuator cfg; (3) POSITIVE CONTROL FIRST: a cap so small it must visibly kill the gait
  (e.g. 1e-6) -- if THAT changes nothing, the cap is definitively not reaching PhysX. Only after a cap is proven to
  bite is the Kv x tau grid meaningful.
STILL VALID from this run (single column, cap inert => it is just a 4-point Kv sweep, and it REPLICATES the earlier
  one on a fresh boot): body_yaw Kv 1.6403e-3 -> 1e-4 -> 1e-5 -> 1e-6 gives bl 0.13459 / 0.16107 / 0.16742 / 0.16571
  and net_yaw -29.1 / -21.7 / -8.3 / -7.9, upright ~0.982-0.983 throughout. Kv DOWN = faster + much straighter, and
  it now looks like it SATURATES around 1e-5 (0.16742) rather than improving all the way to 1e-6 (0.16571) --
  consistent with the earlier boot (0.1657 at 1e-6). n=1 seed; cross-boot absolutes carry the ~2.6% batch effect.

## 2026-07-16T22:55Z — PI decisions EXECUTED (rename / annotate / compile-rule) + 3 more silent-default traps found
PI rulings this round: "改名impedance变成implicit内置的求解器了" · "env的奖励项是RL环境训练里的，做好标注和注释就可以，
确保agent在新会话里可以拿到并迅速fully understand发生了什么，直接指向我们的controller变更和源码" · "改成compile()可以" ·
render demo: "如果被替代且确认没被调用过直接删。留墓碑就可以".

RENAME: `impedance:` -> **`reference_targets:`** in configs/farms_controller.yaml. The agent REJECTED my suggested
  `cpg_targets:` on the PI's OWN ruling: head_pitch_offset is a STATIC head-up hold (脊柱pitch根本没有cpg信号 --
  controller writes pitch_target_final[0] only, [1:] identically 0), head_yaw_* is a heading SERVO, settle/ramp is a
  clock => NONE of the 7 keys are CPG-generated; `cpg_targets` would have been a NEW false name, the same disease.
  Guard moved into `_validate_targets_yaml`, called BEFORE the articulation (fails free + offline-testable), 4/4
  proven without Isaac. This MATTERED: `cfg.get()` would have silently defaulted head_pitch_offset 0.0 (head-up hold
  VANISHES) and head_yaw_gain 1.0 (a servo meant to be OFF switches ON). `farms_controller_numpy_legacy.yaml`
  untouched (md5 verified) -- its `impedance:` name is still accurate; it carries real gains.
ANNOTATE: `_S2_DEAD_ACTION_NOTE` (one canonical block, referenced from 11 sites) at imp_term / r_imp /
  raw/body_imp_scale / the ch-4 rate slot / both action decoders. Each says what it USED to do, why it is now
  identically 0, that it is RL-side kept for bit-identity + downstream-log compat, and that a future run must
  **modulate the TABLE, not a scale** -- pointing at CURRENT_RESULT.md, this log, the migration doc, and the source.
  **Proved semantically inert: AST identical modulo docstrings, zero code changed.**
CLAUDE.md: the mount-truncation guard now mandates **`compile()`, NOT `ast.parse`** + the 3 deletion-proof lessons
  (repo-wide AST scan incl. the getattr-string form; same-name != same-symbol; tombstone with RuntimeError).
3 MORE SILENT-DEFAULT / DEAD-PATH TRAPS FOUND (all real):
  1. **`browse_top_trials.py` was MISSED by S2b's repoint** (the 8th numpy-family consumer): it read the now
     gain-free yaml and `leg.get("kp",[defaults])` would SILENTLY inject gains **35-168x off**. Repointed + fails loudly.
  2. **`docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md` was S1-era and ACTIVELY WRONG** -- its headline "two-line
     migration" calls `set_export_pd()`, which **S2b DELETED** (0 hits; raises AttributeError), and "same controller,
     same YAML, same gains-json" is false on all three. Correction banner + working recipe added (11 annotations
     point there).
  3. `global_6d` without `--design-a-fixed-imp-freq` now RAISES (ch-4 -> the dead-gain assert) => that flag is
     load-bearing for NOT CRASHING, not just axis isolation. `action_mode="body_21d"` is UNRUNNABLE (ch-20 ->
     body_kp_scale). `r_act` still taxes the dead ch-4 (act_term sums the RAW action; Design A pins the decoded
     scale, not the action) -> a real gradient on a dead axis, W_ACT=0.002. All left alone for bit-identity + annotated.
⚠️ DEFECT CAUSED AND SELF-REPORTED BY THE AGENT: its (my-mandated) `--dry-run` OVERWROTE a completed run's
  `outputs/analysis/tune_panel_reset_2026-07-16/reset_default/meta/run_meta.json` with a dry_run stub. The dir is
  untracked => **the original argv/config is NOT recoverable.** Bounded: it was a PRE-S2a run (its gains.json carries
  dead impedance scales, its controller.yaml the old block) so it is not reproducible under the current architecture
  anyway; metrics.json/npz/cell.log/inputs all survive -- only the provenance wrapper is lost. It removed its stub
  rather than leave a fake, and did NOT fabricate a reconstruction. ROOT CAUSE = a PRE-EXISTING panel bug (data/
  routes to _dryrun/ but meta/ was written unconditionally -- contradicting the panel's own contract "a completed
  run's provenance is NEVER clobbered"). Fixed + regression-tested.
PI DECISIONS OWED: (a) `reference_targets:` -- accept or prefer bare `reference:`? (b) `r_act` taxing the dead ch-4 --
  leave for bit-identity or fix behind a default-OFF flag? (c) the dead ch-4/ch-20 decode paths -- delete the
  channels (action space 6->5, breaks checkpoint compat) or keep them failing loudly? (d) attempt a run_meta
  reconstruction from the surviving cell.log/inputs, or let it go?

## 2026-07-16T23:05Z — PI RULINGS on the 4 owed decisions
PI, verbatim: "a:接受，b:default off做好备注，留痕，c:我们最新的动作空间只有wave number那一族你去看看，确认后删死解码路径，(d)接受".
  (a) **`reference_targets:` ACCEPTED** as the block name (was `impedance:`). Settled.
  (b) `r_act` taxing the dead ch-4 -> fix behind a **default-OFF flag**, thoroughly annotated, with a TRACE (留痕):
      the comment must record that this term taxed a dead axis from 2026-07-16, why the default is OFF (bit-identity
      with every pre-flag run), and what flipping it ON changes. IN FLIGHT.
  (c) PI states the CURRENT action space is **wave-number only** -> **VERIFY FROM SOURCE FIRST, then delete the dead
      decode paths.** The gate is the confirmation, not the PI's statement: the agent is instructed to STOP and report
      if the live code disagrees (e.g. still 6-wide with the rest merely pinned by a default-OFF flag). Also
      instructed to keep "delete the dead DECODE PATHS" separate from "narrow the action WIDTH" -- different blast
      radii; narrowing the width **breaks existing policy checkpoints** and needs an explicit PI call. IN FLIGHT.
  (d) **Do NOT reconstruct** the clobbered `reset_default/meta/run_meta.json`. ACCEPTED, and it is the right call:
      that run is PRE-S2a (dead impedance scales in its gains.json, the old yaml block) so it is NOT reproducible
      under the current architecture -- a reconstructed provenance file would falsely imply it could be re-run.
      The surviving metrics.json / npz / cell.log / inputs remain; only the wrapper is gone. The panel's
      unconditional-meta-write bug that caused it is FIXED + regression-tested.

## 2026-07-16T23:15Z — (c) DELETION REFUSED: the "wave-only action space" premise is NOT confirmed
PI stated "我们最新的动作空间只有wave number那一族...确认后删死解码路径". The confirmation gate FAILED, so nothing
was deleted. The live env action space is **6-wide `global_6d`** (`centipede_env_cfg.py:421-422`; action tensor
(num_envs, 6) at `centipede_env.py:417`). Wave is the only FREE axis **in the as-run campaign**, but that is achieved
by flags that **default OFF**, not by the code:
  ch0/1/2 body_amp / leg_yaw_amp / leg_pitch_amp -> LIVE wires, decoded at :1579-1581, CONSUMED at controller
    :1195/:1264/:1266; pinned only by designC (`cfg:651` = **False** by default).
  ch3 wave_number -> LIVE, the free axis.  ch4 body_imp -> STRUCTURALLY DEAD (raises; `body_kp_scale_runtime` = 0
    hits after S2b).  ch5 freq -> LIVE wire (controller:567 `_omega()`), pinned only by designA (`cfg:628` = False).
**THE AMBIGUITY THAT CAUSED THE PREMISE:** Stage E's "wave-only" is a **WRAPPER, not an env change** --
  `stage_e_train_wave_v5_2026-07-13.py:80-93` docstring: "The env is global_6d (action_space=6) but only dim 3
  (wave)..."; it exposes 1-D to rl_games and ZERO-PADS to the env's 6. Memory
  `project_stage_e_feedback_rate_rl_2026-07-12`'s "action = wave# only" is true of the POLICY INTERFACE and false of
  the ENV ACTION SPACE. => memory corrected.
INDEPENDENT CONFIRMATION (disjoint path, not a doc claim): all **1317** checkpoints under `outputs/**/*.pth` were
  loaded and their policy action-head widths read -> **1245 are 6-wide**, 72 are 1-wide (exactly the Stage E wrapper
  runs). The 6-wide space is what 1245 trained policies actually emit.
THIRD REASON TO REFUSE (doctrinal, and the strongest): ch-4 currently RAISES on misuse. Deleting its decode would
  convert a loud failure into a **SILENT NO-OP** -- inverting the exact doctrine this whole session established.
PI DECISION OWED: (a) keep ch-4 failing loudly [agent's recommendation, and mine]; (b) delete the decode but KEEP
  width 6 -- needs an `apply_global_6d_modulation` signature change in `controllers/`, so it must be sequenced;
  (c) narrow width 6->5 -- **breaks all 1245 6-wide checkpoints** (the 72 Stage E ones survive: dropping ch-4 leaves
  wave at index 3 and the wrapper only writes full[:,3]). "Delete the decode path" and "narrow the width" are TWO
  different decisions with different blast radii.

## 2026-07-16T23:15Z — (b) DONE: `--exclude-dead-action-channels-from-r-act`, default OFF, bit-identity PROVEN
`cfg.exclude_dead_action_channels_from_r_act` (default **False**); zeroes ch-4 (global_6d) / ch-20 (body_21d) before
the square-sum; unknown action_mode -> KeyError (fails loudly). Files: envs/centipede_env.py 4257->4313,
centipede_env_cfg.py 1028->1042, scripts/rl/train_stage_b_smoke.py 1027->1049; snapshots *.before_S2actions_2026-07-16;
all compile()-verified in bash, tails intact.
PROOF (CPU, no GPU): OFF branch is **AST-identical** to the pre-flag statement (`_get_rewards` AST-identical across
  179 statements, 0 differing -- the only executable change is the wrapping); behaviour 205/205 `torch.equal`;
  **DIFFERENTIAL, exact**: flag-ON is torch.equal-invariant to arbitrary ch-4 values (50/50) while flag-OFF CHANGES
  with ch-4 (50/50) -- the defect DEMONSTRATED, not asserted.
THE TRAP, in the agent's own annotation (:2731-2786, ~45 lines): "`--design-a-fixed-imp-freq` pins the DECODED scale
  to 1.0; it does NOT zero the RAW action, and this term reads the RAW action. So the pin does not remove the tax.
  Both statements are true at once and that is exactly the trap: the run looks 'isolated to wave' while still paying
  for ch-4." Default OFF because "every checkpoint, reward curve and eval CSV in this project's history was produced
  with the dead channel TAXED, and silently un-taxing it would make new numbers incomparable to all of them."
  => turning it ON is a NEW EXPERIMENT needing its own baseline, not a bug fix to compare against existing runs.

## 2026-07-16T23:20Z — PI RULING: ch-4 keeps FAILING LOUDLY (option (a)) — SETTLED, do not re-open
PI, verbatim: "保持响亮报错". The dead `body_imp` decode path STAYS and keeps raising. NOT deleted, width NOT
narrowed. Consequences, all intended: the 6-wide `global_6d` action space is unchanged; all 1245 6-wide checkpoints
stay valid; `global_6d` without `--design-a-fixed-imp-freq` continues to RAISE (that flag remains load-bearing for
not crashing, not merely for axis isolation); a caller that tries to modulate impedance through ch-4 gets a loud
error pointing at the table, never a silent no-op. Rationale (agent's, and it is the right one): deleting the decode
would convert a loud failure into a SILENT NO-OP, inverting the doctrine this session established.

## 2026-07-16T23:35Z — ⛔ MY 22:35Z RED FLAG WAS WRONG: the cap WORKS; the S3 grid is a VALID NULL. Root cause NAMED.
POSITIVE CONTROL (one boot, 4 physically identical envs, per-env tau on body_yaw only, pinned spawn => internal
comparison, no batch effect) -- outputs/analysis/effort_cap_debug_2026-07-16/:
  | tau_max | bl_per_cycle          | vs env0 (metrics) | vs env0 (raw spine_xy, DISJOINT) |
  | 1.0     | 0.13458946692935347   | ref               | ref                              |
  | **2e-4**| 0.13458946692935347   | **0 differing**   | **exactly 0.000e+00**            |
  | 1e-5    | 0.11735491502960070   | 18 differ         | 1.1 cm                           |
  | 1e-6    | 0.03292451398527529 (-76%) | 18 differ    | 5.9 cm                           |
  All-8-families @1e-6 -> gait DESTROYED (bl -0.00061). **`env.set_joint_effort_limit()` reaches PhysX and enforces
  correctly. There is NO plumbing bug.** Also: env0 (tau=1.0) is bit-identical to the s3 grid's uncapped env0 to all
  17 digits -- which independently validates the reconstructed argv.
**ROOT CAUSE (named, not filed as an anomaly): `peak_body_yaw_torque` MEASURES THE WRONG QUANTITY.** It reads
  `robot.data.applied_torque` = IsaacLab's **explicit-product APPROXIMATION** `stiffness*err_pos + damping*err_vel`,
  computed in PYTHON (`actuator_pd.py:133-139`) and clipped at the ACTUATOR CFG `effort_limit` (10.0). Its own
  docstring: "computes the **approximate** torques ... since PhysX does not compute this quantity explicitly."
  It is **structurally BLIND to `set_dof_max_forces`** and **OVERSTATES the true implicit-PD drive torque by >=10.7x**.
  => my premise "peak 2.13e-3 vs cap 2e-4 = 10x over, must clip" compared the cap against a quantity that is NOT the
  drive torque. The tell I misread: **the measured torque ROSE as the cap tightened** (6.79e-3 @1e-5, 1.29e-2 @1e-6) --
  an enforced limit cannot raise the quantity it limits; that alone proved the metric wasn't the capped quantity.
  **THE S3 GRID IS A VALID NULL, NOT A PIPELINE BUG. Its "3 bit-identical tau columns" = correct physics.**
  MEASURED BOUND: 2e-4 inert => true peak <= 2e-4 ; 1e-5 bites => > 1e-5. **True peak body_yaw drive torque
  ∈ (1e-5, 2e-4].** (So FARMS's ±2e-4 cap does NOT bind our body_yaw at these gains -- itself a real finding.)
NO CODE FIX: the plumbing is correct. The fix is METHODOLOGICAL -- never use `applied_torque` to predict whether a
  PhysX cap binds; bracket the threshold empirically. A meaningful Kv x tau grid needs tau BELOW 2e-4.
CONTRADICTION WITH THE OLD PATH -- RECONCILED BY MEASUREMENT, and the memory's mechanism is CORRECT: both paths
  reach PhysX. On-disk prior (`armature_differential_2026-07-16`, same armature/amps, old path, all joints, cap 1e-5):
  implicit capOFF bl 0.2494 -> capON **-0.0003**; while EXPLICIT capON 0.3969 (inert -- under explicit actuation the
  drive force is not the clipped quantity). Reproduced through the new `joint_ids=` path. Only the GRID'S
  INTERPRETATION (mine) was wrong.
⚠️ THREE FLAGS RAISED BY THE AGENT (reported, not concluded -- these bite the NEXT step):
  1. **THE FARMS LOAD-LINE RESTS ON `applied_torque`.** The panel's "Kv*omega*A = 4.8 mN*m = **24x OVER** a 0.2 mN*m
     cap" and "knee = 23-291x the FARMS cap" are ALL `applied_torque` numbers = the quantity just shown to overstate
     by >=10.7x. **The "Kv must come down FIRST; the cap is not the lever" reasoning quoted all session is built on
     it. RE-DERIVE before steering S3 by it.** (The Kv sweep's MEASURED result -- Kv down = faster + straighter --
     stands on its own; it is the load-line ARGUMENT that is compromised, not the observation.)
  2. `tune_panel_reset_2026-07-16.py` blames the SAME `amp_ratio 1.97->0.66` on BOTH the tau cap (~:210) and a
     6.7x-too-small body wave (~:256) -- that run looks CONFOUNDED. UNRESOLVED; not reproduced.
  3. **`upright` is `nan` in every env of both boots.** The grid's quoted "upright ~0.982" came from a DIFFERENT
     aggregation -- I quoted it as if it were this run's. Do not cite those upright numbers.

## 2026-07-16T23:45Z — PI STANDING RULE SHARPENED: 3-view video coverage
PI, verbatim: "多生成3view demo视频给PI核验，每一个干净的备选parameterset或者足够具有代表性（太软太硬，根本没法走之类的）
都要生成对应视频". This DEFINES what the 2026-07-15 "关键/代表性" refinement meant, and it WINS:
  (1) EVERY clean candidate parameter set gets a 3-view clip -- not just the winner;
  (2) the representative EXTREMES (太软 / 太硬 / 根本没法走 / buckles / beaches) get one too -- **failure modes are
      DELIVERABLES, not noise**: winners-only clips hide whether the search bracketed the real range.
  Bias toward MORE clips ("多生成"). The only prohibition remains auto-rendering *literally every* sweep point.
OPERATIONAL CONSEQUENCE FOR S3: the sweep driver **MUST pass `--dump-raw-alllink`** or the multiview render has no
  input and clips cannot be made without re-booting. **PLAN IT BEFORE LAUNCH.** The S3 grid boots so far did NOT all
  carry it (the 3-boot rebuild dropped `--dump-raw-alllink`; only `--dump-raw`), so those runs cannot be rendered
  retroactively -- the S3 relaunch must restore it.
Memory `feedback_multiview_sweep_video_2026-07-15` updated (+ MEMORY.md index).

## 2026-07-16T23:5xZ — NEW AUDITOR (fresh context) — P3 OPENED; the `upright=nan` flag RESOLVED (it was a DRIVER bug, not a metric bug)
Init: marker `RESULT-2026-07-16T2105Z-s2tbl` read (PARTIALLY RETRACTED = already audited => SETTLED, not re-opened).
Machine clean at 23:39Z (0 python/kit, GPU 976 MiB idle). Live directive = `GO_ARCHITECTURE_RESET_2026-07-16.md`.

**STATE CORRECTION (verified from SOURCE, not the log):** the marker's `KNOWN-BROKEN` entry
"`tune_panel_reset_2026-07-16.py` ... BROKEN until migrated" is **STALE** — the migration LANDED (the only
`--bio-*`/`--suppress-active-pitch` hits in that file are COMMENTS/docstrings recording what died).

**WHY P3 AND NOT S3:** the directive's §8 checklist puts **P3 "Torque instrumentation"** BEFORE any tuning
("must precede any number"; "we can measure τ and sat_frac correctly before we tune against them"), and the
marker's own 23:35Z FLAG says the FARMS load-line steering S3 rests on `applied_torque`. Verified from source
that **P3 is NOT started**: `estimated_drive_effort` and a live `sat_frac` do not exist in the pipeline
(the only `sat_frac` hits are `outputs/_archive_pre_reset/...` and `audit_analyze_2026-07-13.py:101`, where
`sat_frac = round(float(tau_max > EFFORT), 3)` is a **BOOLEAN**, not a time fraction — a separate mislabel).
=> S3 cannot be steered honestly until the torque metric is fixed. P3 first.

### ⚠️ THE OPENING (WORLD-FIRST rule 1: a comment is TESTIMONY, not evidence)
IsaacLab's `ImplicitActuator.compute` docstring — *"since PhysX does not compute this quantity explicitly"* — is a
DOC-CLAIM. **The PhysX tensors binding contradicts it.** `_physicsTensors.cp311-win_amd64.pyd` EXPORTS, and
`omni/physics/tensors/impl/api.py` fully IMPLEMENTS: **`get_dof_actuation_forces` (:2034)**,
**`get_dof_projected_joint_forces` (:2062)** ("projects the links incoming joint forces in the motion direction and
hence is the **active component of the force**"), **`get_link_incoming_joint_force` (:2376)**. IsaacLab's
`articulation.py` calls the **`set_` side ONLY** — it never reads any of them. The quantity may be available and
merely unused. **THAT IS A HYPOTHESIS, and this boot tests it — the getters' docstrings do not settle it either.**

### THE RUNTIME WORLD, INVENTORIED (source, before touching anything)
- **PhysX cap** = `_data.joint_effort_limits` -> `root_physx_view.set_dof_max_forces` (`articulation.py:761-794`).
  IsaacLab's own note there: *"This function isn't setting the values for actuator models. (#128)"*.
- **Diagnostic clip** = `actuator.effort_limit`, a **SEPARATE runtime buffer** (`actuator_base.py:199`), used ONLY by
  `_clip_effort` (:364) -> `applied_effort` -> `data.applied_torque`. `ImplicitActuator.compute` **returns
  `control_action` UNCHANGED** (`actuator_pd.py:140`) => under implicit PD this number **never feeds physics**.
- `actuator.effort_limit_sim` IS written to sim, but ONLY in `_process_actuators_cfg` (:1707), called from
  `:1518` = **init only, NOT on reset** => our `set_joint_effort_limit()` is not re-clobbered. (Independent
  sub-agent adversarially auditing this claim + what the 3 getters MEASURE; result pending.)

### P3-A LAUNCHED — torque metrology, calibrated against an INDEPENDENTLY BRACKETED truth
Instrument: cell **1464 -> 1530 lines**, new flag **`--dump-physx-forces`** (default OFF, IMPLIES --dump-torque),
records per step `physx_tau_act` = `get_dof_actuation_forces()` and `physx_tau_proj` =
`get_dof_projected_joint_forces()` alongside `tau_all` (= the suspect `applied_torque`). READ-ONLY; allocation/
capture/dump all flag-guarded. Protected-edit discipline: python patcher (7 anchors, each asserted UNIQUE) ->
`safe_write.sh --no-ast` (non-ASCII `§` false-fails its GBK AST check) -> **independent bash `compile()` OK**,
1530 lines, tail `os._exit(int(rc))` intact, snapshot `.before_P3physxforces_2026-07-16_2026-07-16`
**md5 7a9b9f3869820ad27103fdb2c5ac819f == the pre-edit md5**.
Driver `scripts/analysis/p3_torque_truth_2026-07-16.py`: reuses `effort_cap_debug`'s **PC_bodyyaw config VERBATIM**
(one boot, 4 physically identical envs, body_yaw-only tau ladder **[1.0, 2e-4, 1e-5, 1e-6]**) — a config whose
answer is ALREADY KNOWN from a **disjoint** method (trajectory bit-identity), so the metric is scored against a
truth it cannot fudge. Carries `--dump-raw-alllink` (PI 23:45Z: plan the render input BEFORE the boot).
**5 falsifiable criteria** (each a way to FAIL): C1 env0 bl == `0.13458946692935347` @17 digits (ALSO proves the
read-only instrumentation is physics-inert) · C2 env1 (2e-4) bit-identical · **C3 peak <= tau_max in envs 2/3
— `applied_torque` FAILS this by ~4 orders (1.29e-2 vs 1e-6); this is the discriminator** · C4 env0 peak in
(1e-5, 2e-4] · C5 monotone-nonincreasing. **A ~ZERO read = the user-set actuation buffer, never written under
implicit PD => hypothesis REFUTED**, and that null is reported as the result, NOT patched over with applied_torque.

### ✅ RESOLVED (owed item, marker 23:35Z FLAG 3): "`upright` is nan in every env of both debug boots"
**It is NOT a metric bug and the runs are NOT missing uprightness data.** `upright` was **never a cell metric** —
no such key exists (`grep -n upright` on the cell: the emitted keys are `min_upright` :1311, `min_upright_head`,
`sustained_upright` :1280). `effort_cap_debug_2026-07-16.py:124` lists `"upright"` in its `KEYS` tuple, so
`e.get(k, float('nan'))` printed **nan for a key that never existed**. A DRIVER key-name bug.
**The valid, live numbers for those exact runs** (`data/summary_PC_bodyyaw.json`) — and note these are THREE
DIFFERENT AGGREGATIONS (CLAUDE.md: never quote one as another):
| env | tau_max | `min_upright` (body-median, worst frame = the tip-over gate) | `sustained_upright` (v7 steady-window median) | `min_upright_head` (legacy) | `buckled` |
|---|---|---|---|---|---|
| 0 | 1.0  | 0.9830187559127808 | 0.9867879450321198 | 0.958958089351654 | **False** |
| 1 | 2e-4 | 0.9830187559127808 | 0.9867879450321198 | — | **False** |
| 2 | 1e-5 | 0.9822700023651123 | 0.9861587285995483 | — | **False** |
| 3 | 1e-6 | 0.9799842834472656 | 0.9853209257125854 | — | **False** |
=> **"it never fell" IS supportable from these runs' own data** (`buckled=False` all 4; gate = cos45 = 0.707).
The marker's "do NOT quote an upright figure for these runs" was **over-cautious / mis-diagnosed**: the data is
there and valid. What the flag got RIGHT is the discipline — a bare "upright ~0.982" names no aggregation, and
0.9830 (min) vs 0.9868 (sustained) vs 0.9590 (head) are three different numbers. **Cite `min_upright` (0.980-0.983,
buckled=False), by name.** Marker + memory to be corrected on close-out.

## 2026-07-17T00:5xZ — P3-A/B/C DONE: the true drive torque is MEASURED (5e-5, 7e-5]; TWO of my own claims RETRACTED
**THE NUMBER (P3-C, threshold bisection — the project's FIRST direct measurement of the true drive torque):**
`bl_per_cycle` vs the uncapped reference **0.13458946692935347** (INERT == bit-identical to all 17 digits):
| tau_max | 1.4e-4 | 1.0e-4 | **7e-5** | **5e-5** | 3e-5 | 2.2e-5 | 2.0e-5 | 1.5e-5 | 1e-5 | 1e-6 |
|---|---|---|---|---|---|---|---|---|---|---|
| verdict | INERT | INERT | **INERT** | **BITES** | BITES | BITES | BITES | BITES | BITES | BITES |
⇒ **TRUE PEAK body_yaw DRIVE TORQUE ∈ (5e-5, 7e-5] N·m.** Consequences, all tightened BY MEASUREMENT:
  * `applied_torque` (2.134339e-03) **OVERSTATES the drive 30.5×–42.7×** (supersedes the weak ">=10.7x" lower
    bound, which was taken off the NON-BINDING 2e-4 point).
  * `get_dof_projected_joint_forces` (2.182928e-05) **UNDERSTATES it 2.3×–3.2×.** **BOTH candidates are wrong, in
    OPPOSITE directions.** proj ≠ drive: the cap bites at 3e-5 = **1.4× ABOVE proj's entire peak**.
  * `get_dof_actuation_forces` ≡ **0.0** (0 of 70052 nonzero, bitwise) = the user-set FF buffer.
  ⇒ **NO readback measures the drive. Bracketing is the ONLY method — and it WORKS.** That is the P3 answer.
**MY P3-B PREDICTION WAS FALSIFIED AND I REPORT IT AS SUCH.** Pre-registered: "if proj is the drive, caps above
  2.183e-05 must be INERT." 3e-5 and 2.2e-5 BOTH BITE. The "proj peak lands within 0.02% of the cap at 2.2e-5 /
  2.0e-5" clamping signature I flagged CANNOT be clamped drive — the cap bites well above proj's peak. **NOT
  explained. Do not build on it.**
**FARMS: 2e-4 cap = 2.9×–4× ABOVE the true drive ⇒ it does NOT bind our body_yaw** — and this **no longer rests on
  the 2e-4 point at all** (the verifier's stated SINGLE POINT OF FAILURE): **7e-5 is INERT in a DIFFERENT boot**.
  The per-env write is now EMPIRICALLY PROVEN to reach PhysX with DISTINCT values: in ONE boot env2 (7e-5) is
  bit-identical to uncapped while env3 (5e-5) BITES; a failed/broadcast write would make all four envs identical.
  **RE-SCOPED:** what is refuted is that *in OUR implicit-PD sim* the drive is 24× over the cap. Whether FARMS's own
  MuJoCo/explicit actuator demands 4.8 mN·m is a DIFFERENT question this run cannot answer.

### ⛔ TWO OF MY OWN CLAIMS RETRACTED (independent verifier; every measured number reproduced EXACTLY by a disjoint path)
1. **"`applied_torque` FEEDS THE REWARD ⇒ IT BITES THE RL SIDE" — RETRACTED.** The CHAIN is real and every link
   verified (`:2445` W=0.003 → `:2651` → **`:2681` `"tau": tau`** (the link I omitted) → `:2822` → `:3059` →
   `:3527`; `r_tau` UNCONDITIONAL, no gate). **BUT I NEVER COMPUTED ITS SIZE.** `tau_term` = 4.71e-4 ⇒
   **|r_tau| = 1.41e-06 vs r_v = 0.0138 ⇒ r_tau/r_v ≈ 1.0e-04 = ~0.01% of the forward term.** "M3 fix ① would make
   every future run incomparable to 1245+ checkpoints" is **REFUTED AS STATED**. **The "PI DECISION OWED"
   escalation is WITHDRAWN.** ⇒ **THE STANDING RULE BIT ME**: every "negligible" label needs natural-scale
   arithmetic — **and so does every "it bites".** Same error class this project catalogues, in the session that
   catalogued it.
2. **"proj exceeds a binding cap ⇒ it is the TOTAL generalized force" — RETRACTED (mechanism).** Arithmetic verified;
   mechanism asserted while my own scope-limits admitted it was unmeasured. Excluded by enumeration: joint friction
   (none authored on body_yaw), unit error (link_wrench agrees 0.95%), batch effects (env0/env1 bit-identity).
   **NOT excluded, never checked: THE JOINT LIMIT** — body_yaw authored at **±36°**, measured max|q−q0| = 0.9535 rad
   = **75.9% of range width**; my statistic is *peak over all (t,joint)* = the one most contaminated by limit contact
   (env3: 45.3% of samples over cap). **proj ≠ drive STANDS (different reason); the REASON WHY is UNRESOLVED.**

### Other corrections accepted from the verifier
- **"applied_torque RISES ⇒ not the capped quantity": conclusion right, ARGUMENT WEAK.** A rise is what a *pre-cap
  DEMAND* does (the cap makes the joint lag: max|qd| 3.365 → 6.305 rad/s env0→env3, so the explicit product grows).
  **The airtight argument is C3 alone** + `articulation.py:777` *"this function isn't setting the values for
  actuator models (#128)"*.
- **"act reads 0 ⇒ user FF buffer": verdict stands, my REASON was unproven.** The verifier CAN distinguish a genuine
  zero from a never-written allocation (I could not): act (`cell:1129`) and proj (`:1131`) are written on ADJACENT
  lines at the same `fi`, and proj is 100% nonzero with 83/83 unique slices ⇒ the block ran every step.
- **THE CELL LOG IS ACTIVELY MISLEADING and I cited it wrongly.** `centipede_env.py:1490` does
  `torch.as_tensor(_v).reshape(-1)[0]` ⇒ the effort echo prints **env0's value only**, as a scalar (`tau_max=1`),
  reading as if ALL FOUR envs got 1.0. Armature prints per-env WITH a read-back (`log:57`); effort has none.
  **OWED (one line): a per-env READ-BACK of `joint_effort_limits` after reset.**
- `link_wrench` (the 4th candidate) **ANALYSED**: max|T| = 3.0289e-05 vs proj absmax 3.0004e-05 (**0.95%**) ⇒
  corroborates proj's units and that proj reads a **REACTION** torque, a different quantity from drive.
- ⛔ **DO NOT CITE "97.8×"** (a ratio between two DIFFERENT physical quantities) **or ">=10.7×"** (superseded).
- Fixed en route: the analyzer's `.0e` tau format rendered 2.2e-5 / 2.0e-5 / 1.5e-5 as the SAME string "2e-05" —
  three distinct taus, one label. Now `.3e`.

**MARKER `RESULT-2026-07-17T0015Z-p3tau` → STATUS: VERIFIED with CORRECTIONS** (banners added at the top so the
superseded 00:15Z headline cannot be read straight; KEY_NUMBERS rewritten post-correction).
**P3 IS STILL NOT COMPLETE:** `sat_frac` (directive §B: the ONLY admissible saturation number) is unimplemented —
and this result proves it CANNOT be computed from `applied_torque`. It needs the bracketing method, or a
per-timestep quantity nobody has yet identified. **That is the next P3 step, and it is now well-posed.**

## 2026-07-17T02:10Z — P5 STARTED AND IS PRODUCING: body_roll Kv is a NEW +100% lever; best +129.7%. **THE SEED IS INERT.**
PI asked "接着做，我们还要多久可以sweep参数，优化model的controller". **ANSWER: the sweep is UNBLOCKED and RUNNING —
4 boots done this session.** Throughput measured: **16 configs / ~6 min boot ≈ 150-160 configs/hour** (boot time is
Omniverse startup, ~5 min; N=16 costs the same wall-clock as N=4). A few-hundred-config campaign = one overnight.

**WHY P5 COULD START (P3 gate satisfied BY MEASUREMENT, not bypassed):** P3 gated P5 on "measure tau and sat_frac
before tuning against them". Measured: true drive = (5e-5, 7e-5]; the directive's `sat_frac` is defined on the
EXPLICIT PRODUCT = the quantity that overstates 30.5-42.7x => **unfixable as specified**. BUT `gamma = tau_max/(Kp*A)`
=> **UNCAPPED => gamma = inf => saturation IMPOSSIBLE BY CONSTRUCTION** => gamma/sat_frac MOOT. All P5 boots run
UNCAPPED. FARMS-faithful (2e-4 measured not to bind). And this sweep only LOWERS Kv => lower drive => moves further
below the cap. GUARD: any future config RAISING Kp or A must have its drive BRACKETED vs 2e-4 before shipping.

### THE RESULTS (4 boots; every boot's env0 control PASSED at 17 digits: bl = 0.13458946692935347)
POSITIVE CONTROL (screen envs 1-3) **PASSED**: body_yaw Kv/100 -> +25.5%, net_yaw -8.84 — reproduces the
2-boot-replicated prior lead (+24%, -29.1 -> -8.3). The rig is honest, so the rest of the boot is admissible.
**⭐ body_roll Kv is a NEW lever and the BIGGEST one — /10 ALONE = +100.1% (bl 0.269325), net_yaw -2.28 (the
STRAIGHTEST in the boot), min_upright 0.9954 (the most upright). 4x the body_yaw lead that has steered this
project.** It was never tested because attention was on body_yaw.
| config | bl/cycle | d_bl | net_yaw | min_upr |
|---|---|---|---|---|
| baseline | 0.134589 | — | -29.14 | 0.9830 |
| **roll/5+yaw/100+pitch/1000** | **0.309086** | **+129.7%** | +9.77 | 0.9980 |
| roll/10+yaw/100+pitch/1000 | 0.303720 | +125.7% | +6.37 | **0.9990** |
| roll/10 alone | 0.269325 | +100.1% | **-2.28** | 0.9954 |
16/16 survive in every boot. body_roll is a **BROAD PLATEAU** (/2 +42% | /3 +71% | /5 +94% | /10 +100% | /20 +102%
| /50 +92%) — the optimum's LOCATION is robust (that is NOT robustness across conditions).
**⭐ INTERACTIONS ARE SUB-ADDITIVE**: roll/10+yaw+pitch observed **+125.7%** vs sum-of-singles **+159.7%**
(also roll+yaw +110.3 vs +125.7; roll+pitch +119.2 vs +134.1; yaw+pitch ~additive). ⇒ **the three body families
are NOT three independent levers — they share ONE mechanism.** Likely "total body damping" — **HYPOTHESIS, not a
finding**; the arithmetic only shows they do not add. Do not model them as additive gains.

### ⭐⭐ THE METHOD FINDING — worth more than the number, and it KILLS a control we were about to run
**THE SEED IS INERT: 16/16 configs BIT-IDENTICAL between `--terrain-seed 4242` and `777`.**
FLAT (`--amp-mm 0`) + `--central-spawn --pin-spawns --n-strata 1` = **there is NO RNG left to vary.**
- ⇒ **"run >=3 seeds" HERE IS THEATRE** — it returns 3 identical numbers and reads as a robustness PASS. This
  project's rule *"1-seed Pareto = LOTTERY"* (`project_a008_overnight_2026-07-05`) **DOES NOT APPLY to this
  config** — there is no lottery. It applies where spawn/terrain RNG is live.
- ⇒ **THE COROLLARY IS THE POINT: +129.7% is PERFECTLY REPEATABLE AND COMPLETELY UNGENERALIZED.** One flat
  terrain, one spawn, one pose, 4 s. **Repeatability != robustness.** The real test is VARYING THE CONDITIONS:
  rough terrain (`--amp-mm > 0` — which is also what makes `--terrain-seed` bite, and only THEN is multi-seed
  meaningful and required), multiple spawn strata, longer episodes.
- This is the session's own lesson turned on itself: *a test that passes equally under "it works" and "it does
  nothing" tests nothing.* A 3-seed table here would have been exactly that — and I would have shipped it.
- USEFUL COROLLARY: env0 reproduced 0.13458946692935347 at **N=4 AND N=16 AND on both seeds** ⇒ **no batch effect
  in this spawn config** ⇒ cross-boot absolutes ARE comparable here (the ~2.6% batched-vs-unbatched drift in
  `feedback_pinned_spawn_rng_batch_drift_2026-06-24` is about a DIFFERENT, RNG-live spawn config).

### OWED (priority order — #1 can kill the result)
1. **ROUGH-TERRAIN test of the finalists.** THE #1 RISK, and not hypothetical: low body damping that doubles speed
   on FLAT is exactly what can fail on terrain (damping is what rejects disturbances). This robot is FOR terrain.
2. **3-VIEW VIDEO — OWED TO THE PI** (standing rule 23:45Z): clean candidates (roll/5+yaw+pitch fastest;
   roll/10+yaw+pitch straightest/most upright; roll/10 alone best yaw) + extremes (baseline slow/-29deg;
   roll/50 past the plateau). **All 4 boots carried `--dump-raw-alllink` => NO re-boot needed.** Not yet rendered.
3. Energy/effort NEVER reported — speed alone is not an optimisation target. net_yaw DEGRADES for the fastest
   configs (-29.14 -> -2.28 at roll/10 -> **+9.77** at the fastest: it overshoots). Real speed-vs-straightness
   trade-off that "+129.7%" hides.
4. **Kp is COMPLETELY UNSWEPT** (only Kv touched); `A` unswept.
5. "+129.7%" is vs THIS baseline, NOT the project's best gait (the panel default walks at bl ~0.3031 with body_amp
   scale 5.0 — a DIFFERENT config). It lands ~LEVEL with the panel default. **Not a like-for-like comparison; do
   not read it as "2.3x the state of the art".**

**MARKER: `RESULT-2026-07-17T0210Z-p5kv` (P1, STATUS: UNVERIFIED).** Supersedes `RESULT-2026-07-17T0015Z-p3tau`
(VERIFIED with CORRECTIONS; archived verbatim + tagged DO-NOT-CITE-AS-LIVE at
`outputs/analysis/p3_torque_truth_2026-07-16/MARKER_p3tau_CLOSED.md`). P5 marker DEPENDS on P3's measurement.

## 2026-07-17T03:2xZ — ⛔⛔ THE VIDEO RULE CAUGHT A PROJECT-LEVEL METRIC DEFECT: `bl_per_cycle` conflates STRAIGHTNESS with SPEED
PI: "先渲视频，然后做rough terrain". Rendered 5 configs x 3 views (top=yaw / side=pitch / chase=45deg) + composites via
the OFFLINE path (`render_multiview_2026-07-15.py` -> `offline_render_from_log_2026-06-22.py`; CPU, deterministic,
no RTX race, NO re-boot needed — every P5 boot carried `--dump-raw-alllink`). **All 15 view-clips PASS the per-frame
audit** (black/white/blank/partial-white/strobe/outlier all 0.00, motion 0.71-1.00, mean_lum 142-146, red-blob
present every frame). Full paths in the run's `videos/` + per-clip `*_audit.json` + MULTIVIEW_LEDGER.md.

**THEN THE MANDATED HUD<->CSV CROSS-CHECK (skill Phase 3) FAILED — and it found a PROJECT-LEVEL defect.**
The HUD's `bl_per_cycle` disagreed with the cell's. **THE TELL: the ratio was NOT CONSTANT** (1.026 / 1.031 / 1.050 /
1.075 / **1.3605**) — a units/normaliser bug gives a CONSTANT ratio; a config-dependent one means **the two measure
DIFFERENT THINGS**. Recomputed BOTH from the raw `body_pos_w` by a DISJOINT path (no reuse of either script):
  * **cell `bl_per_cycle` = ROOT/HEAD-x-displacement** (recomputed 0.1374 vs cell 0.1346)
  * **renderer HUD = SPINE-COM-x-displacement** (recomputed 0.1846 vs HUD 0.1831)
**MEASURED: `r(|net_yaw|, root/COM ratio) = -0.887` (n=16)**; root/COM 0.744 @ yaw -29.1 deg -> 0.980 @ +9.8 deg.
**A body that YAWS swings its HEAD relative to its COM, and the root metric charges that to forward progress.**
⇒ **`bl_per_cycle` CONFLATES GOING STRAIGHTER WITH GOING FASTER.**

### ⛔ RETRACTED — mine
"+129.7%" -> **+72.7% by COM**. ~HALF my headline was the artifact (the baseline yaws -29.1 deg, so its ROOT-based
speed is depressed 26%, inflating everything measured against it).

### ⛔⛔ RETRACTED — NOT mine: the PROJECT'S STANDING LEAD
**`body_yaw Kv 1.64e-3 -> 1e-5 = "+24% speed"` IS A METRIC ARTIFACT. By COM it is -3.5% — NOT a speed lever at all.**
It is a **STRAIGHTNESS lever** (net_yaw -29 -> -8 deg: real, measured, useful) that was mislabelled speed. It was
2-boot replicated, sits in standing memory, and **was the stated reason S3/P5 was being steered toward low body_yaw
Kv**. **The replication is real — it replicated the ARTIFACT.** Replication tests the RIG, not the metric's MEANING.
| config | root | **COM** | net_yaw |
|---|---|---|---|
| body_yaw Kv/100 | **+25.0%** | **-3.5%** | -8.84 |
| **body_roll Kv/10 ALONE** | +98.6% | **+53.4%** | -2.28 |
| roll/5+yaw+pitch (best) | +127.4% | **+72.7%** | +9.77 |

### ✅ SURVIVES
**body_roll Kv is a REAL speed lever and still the BIGGEST (+53.4% COM), and it was never tested before.**
SUB-ADDITIVITY holds under COM (+68.9% vs sum-of-singles +75.3%). SEED-INERTNESS stands. All env0 controls at 17
digits. **HUD net_yaw matches the cell EXACTLY on all 5 rendered configs** => right envs, right data; only the bl
LABELS baked into the clip filenames are root-based (noted, not re-rendered).

### THE LESSON (it is the session's third instance of the same disease)
**Two independent ESTIMATORS of the same quantity are worth more than ten REPLICATIONS of one.** No seed, no
replication, no in-boot bit-identity would have caught this — the rig was perfect; the METRIC was wrong. Same class
as the `applied_torque` finding hours earlier: **a number read as measuring what we wanted.**
**PI DECISION OWED:** CLAUDE.md's "Forward = WORLD-x: v_forward = -root_lin_vel_w[:,0]" IS the declared convention
and the cell follows it CORRECTLY — but that rule was written for the RL obs/reward channel and is the WRONG
estimator for cross-config locomotion PERFORMANCE. Report both / lead with COM? **Do NOT silently switch** (it
breaks comparability with the project's entire history).
**⛔ BLAST RADIUS NOT AUDITED: every historical `bl_per_cycle` is root-based** (A004 lock, wave-roughness sweet-spot,
compliance/armature arcs, the panel default's 0.3031). Scope an audit before citing any as a SPEED result.
Minor, real: `L_body` cell-MEASURES 0.101184 m vs renderer-HARDCODES 0.1025 m (~1.3%, constant, cancels in ratios).
`render_multiview` prints `audit=None` — it reads top-level `"PASS"` but the renderer nests it at `audit.PASS`;
the audits DO run and DO pass. Marker + memory (`project_bl_metric_yaw_artifact_2026-07-17`) updated.
**ROUGH TERRAIN IS NEXT** (PI-ordered), and it must report COM alongside root.

## 2026-07-17T04:0xZ — ✅ PI RULING: THE CANONICAL LOCOMOTION METRIC (settled + implemented + self-tested)
PI: *"以后都用COM做BL/Cycle的指标, COM会左右摆吗, 可以接受作为指标吗... 要不要把标准统一定为COM在XY plane上的速度BL/Cycle?"*
**⭐ AND THE PI CORRECTED MY FRAMING — THE DATA AGREES: "不是走得直就是走得快而是走得扭就是走得快".**
The TWISTED baseline (net_yaw -29.1 deg) has a HIGHER COM speed (0.1846) than the straightened yaw/100 (0.1782).
**Lateral undulation IS the thrust.** My "root rewards straightness" described the METRIC's scoring illusion and
inverted the physics. The PI read my own data correctly and I did not.
**IMPLEMENTED: `scripts/analysis/locomotion_metrics_2026-07-17.py`** (import it; never re-derive):
  speed = |NET XY displacement of the MASS-WEIGHTED COM| / T_alive [BL/cycle, L=0.1025 m FIXED]
  straightness = net_yaw_deg, SEPARATE, never folded in (one number encoding two things caused this incident)
SELF-TEST **PASS** vs the hand-computed values by a disjoint path. Under the canonical metric:
| config | BL/cycle | d | net_yaw |
|---|---|---|---|
| baseline | 0.1872 | — | -29.14 |
| **body_yaw Kv/100** | 0.1786 | **-4.6% (NOT a speed lever)** | -8.84 |
| **body_roll Kv/10** | 0.2831 | **+51.2% (THE REAL LEVER)** | -2.28 |
| roll/5+yaw+pitch | 0.3218 | +71.9% | +9.77 |
ANSWERS TO THE PI'S QUESTIONS, ALL MEASURED:
 * **COM 会左右摆吗** — YES but small: 1.24 mm (roll/10) to 5.02 mm (baseline) = **1.2-4.9% BL** after removing the
   linear drift; scales with net_yaw. XY-net absorbs it.
 * **COM == 关节 PCA?** — **EXACT, BY CONSTRUCTION.** PCA subtracts the mean, so the unweighted centroid IS the PCA
   centroid. (The 1.5-2.3 mm I measured is the body's BEND about its principal axis — a different quantity.)
 * **mass-weighted COM vs unweighted centroid** — **MEASURED EQUIVALENT**: max separation 0.68 mm, bl <=0.46%,
   conclusions identical. `mb_mass` ALREADY on disk (18 npz) with IDENTICAL link ordering (251/251) => no boot
   needed. Total 2.5361 g == the PI-measured 2.536051 g. Spine = 94.1% of mass.
 * **XY-net vs XY-path** — **NET.** path/net measured 1.002-1.017 (does not bite today) but XY-path is gameable by
   wriggling in place (net=0, path>0) = literally "走得扭就是走得快". Rejected ON PRINCIPLE before RL finds it.
 * **XY vs XYZ** — XY. Heading drifts on compliant terrain (the reason to drop x-only), but climbing != progress.
ACCEPTED COST (stated, not slid through): XY-net is DIRECTION-AGNOSTIC (sideways scores); honest under an open-loop
CPG with no heading command, with net_yaw carrying direction. A circular path over a LONG episode under-reads —
flag it there; do NOT silently switch to path length.
⚠️ NEW, UNRESOLVED: `mb_mass` shows **0/251 massless links** — the "116 massless dummies" claim in
`project_armature_root_cause_2026-07-16` does NOT reproduce (mass floor in that run? or the claim means something
else?). Flagged, not chased.
⛔ STILL OWED: CLAUDE.md's "Forward = WORLD-x: v_forward = -root_lin_vel_w[:,0]" is now the LEGACY convention for
cross-config SPEED comparison (it remains correct for the RL obs/reward channel). **Blast radius of every
historical root-based bl_per_cycle NOT audited.** Report BOTH during the transition (`bl_root_legacy` provided).
**NEXT: ROUGH TERRAIN (PI-ordered), reporting the canonical COM metric alongside root.**

## 2026-07-17T05:xxZ — 3-ACTIVE-AXIS GRID DONE (43 cfg, 0/43 buckled) + ROUGH VIDEOS DELIVERED
PI: grid search, INTEGER form, amplitudes fixed at default, sweep the 3 active axes, lock the rest, ~1 h.
**AMPLITUDES CONFIRMED FROM SOURCE** (the PI recalled 2 of 3 correctly): body_yaw 0.4x1.25 = **0.500 rad** ✓ ·
legbase_yaw 0.6x0.5 = **0.300 rad** ✓ · legbase_pitch 0.40x1.0 = **0.400 rad** (PI recalled 0.3).
`leg_wave amplitudes[2]=[3]=0.0` ⇒ **legbase_roll + knee carry NO CPG signal — the config CONFIRMS the PI's
"只有body yaw和leg base pitch和yaw才有cpg信号"**. ⇒ the 3 active axes = body_yaw, legbase_yaw, legbase_pitch.
GRID (`scripts/analysis/p5_grid3axis_2026-07-17.py`): INTEGER percent-of-baseline (`body_yaw|kp30|z10`), **Kp/Kv
PAIRED** `Kv = Kv_base*sqrt(kp/100)*(z/100)` (c_crit=2*sqrt(kI) ⇒ at fixed damping ratio Kv MUST track sqrt(Kp);
sweeping Kp at fixed Kv silently changes the damping ratio). kp∈{3,10,30,100,300}% x z∈{1,10,100}% per axis.
Rough l75a12, seed 4242, UNCAPPED, armature 0, dt 1 ms. 43 cfg / 3 boots / ~19 min. **0/43 buckled.**

### ⭐ THE RESULT IS A SURPRISE AND IT INVERTS THE SEARCH PLAN
| config | speed | d% | fwd | net_yaw | upr |
|---|---|---|---|---|---|
| baseline kp100\|z100 | 0.1588 | — | 0.1573 | -30.7 | 0.9615 |
| **body_yaw\|kp300\|z1** | **0.1741** | **+9.7%** | 0.1706 | **-3.1** (STRAIGHTEST) | 0.9653 |
| legbase_pitch\|kp3\|z100 | 0.1719 | +8.3% | 0.1665 | -21.5 | 0.9656 |
| body_yaw\|kp300\|z10 | 0.1694 | +6.7% | 0.1673 | -24.4 | 0.9639 |
**THE 3 CPG-ACTIVE AXES YIELD AT MOST +9.7%** — with Kp swept 100x and the damping ratio swept 100x — **while the
LOCKED, NON-CPG-DRIVEN body_roll + body_pitch yielded +92.6% on the SAME terrain.**
⇒ **THE SPEED LEVER IS NOT IN THE DRIVEN AXES. It is in the "passive" axes' damping.** Self-consistent reading:
the DRIVEN axes set the WAVE SHAPE; the PASSIVE axes decide whether that wave can PROPAGATE down the body.
⇒ **RECOMMENDATION: UNLOCK body_roll + body_pitch into the grid.** The PI's lock (a reasonable a-priori) locked
away the biggest lever — this is now measured, not argued.
⇒ **OBJECTIVE 1 (平地走直线) HAS A CANDIDATE**: `body_yaw|kp300|z1` = Kp x3, damping ratio x0.01 -> **net_yaw -3.1
deg** (from -30.7) AND simultaneously the fastest. **NOTE: measured on ROUGH; the PI's objective 1 is on FLAT —
UNTESTED there.**

### VIDEOS — DELIVERED (rough l75a12), all audited
First attempt SILENTLY DIED (`nohup ... &` never survived) and I reported it as "rendering" — WRONG. Re-run in the
FOREGROUND. **9/9 view-clips PASS** (black/white/strobe 0.00, motion 0.95-1.67, lum 140-152), asserted NON-EMPTY
(my first audit printed "ALL PASS = True" over an EMPTY list — a vacuous check, the exact disease this session
keeps cataloguing; now fail-loud). Paths:
`outputs/analysis/p5_kv_refine_2026-07-17/videos_ROUGH/{ROUGH_BASELINE...,ROUGH_BEST_roll10+yaw+pitch...,ROUGH_yaw100-alone_...NOT-a-speed-lever}_3view.mp4`
(flat set, COM-labelled: `videos_COM/`).

### OWED — see `outputs/directives/OBJECTIVE_4WAY_2026-07-17.md` (the PI's 4 objectives + 4 gaps)
**A. `yaw compliance` HAS NO MEASURABLE DEFINITION** (the standing OPEN PROBLEM) — a grid CANNOT rank an
undefined quantity. 3 candidate metrics proposed, all computable from the npz we already dump; **PI to choose**,
and it needs a MAGNITUDE gate (rank != magnitude).
**B. per-joint max torque UNMEASURABLE absolutely.** The (5e-5, 7e-5] = 0.05-0.07 mN*m bracket is **body_yaw ONLY,
the SOFTEST family** (legbase_yaw Kp is **772x** body_yaw's). **"We are under 0.2 mN*m" is UNSUPPORTED.**
**C. the baseline is from the WRONG PLANT** (armature 1e-4 gains on an armature-0 model; PI-identified, accepted).
This grid CORROBORATES it: the active axes barely move the result ⇒ the mistuning is elsewhere.
**D. head_pitch sign flip REFUTED by measurement** (-0.25 -> +0.25 drives the head MORE DOWN: -2.66 -> -14.26 deg,
while the first pitch LINK goes UP). The offset is a JOINT-RELATIVE angle, not head world pitch. **NOT applied.**
A 1-D `head_pitch_offset` sweep measuring the HEAD LINK pose is owed before it can be locked.
