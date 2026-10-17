# PROJECT HAND-OVER — START HERE

> ## ⚠️ STALE IN PART — the parameter mechanism changed on 2026-07-16 (read this first)
> 
> This page was written **before** the implicit-only unification (S1/S2a, PI-signed; `DISPATCH_LOG.md`
> 2026-07-16T19:34Z). Its **file map and mechanism sections are partly superseded**. Two specific items below are now FALSE:
> "Gains are *scales*, not absolutes" and "`effort` is an *absolute* torque cap and is **always on**".
> Also, the "workhorse" cell `scripts/sanity/stage_a_vec_sweep_cell_2026-06-17.py` is **RETIRED** (it refuses to
> run); today's cell is `scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py`.
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

**For: 王熙原 (PI). Purpose: take over a project that has been agent-driven from day one.**
_Living document. Started 2026-07-13. Read top-to-bottom once, then use as reference._

> How to use this folder:
> - **`README.md` (this file)** = the map + the mental model + how to operate everything.
> - **`STEP_LOG.md`** = a running, hand-held log of *every* code change / experiment from 2026-07-13 onward:
>   what changed, why, the exact command to run it, how to read the result. New entry per step.
> - When I (the agent) do anything from now on, I add a STEP_LOG entry so you can reproduce it yourself.

---

## 0. One-page TL;DR — how to take over *today*

**What this project is.** A simulated centipede robot (IsaacLab / PhysX soft-body) walks on rough terrain. Its
gait is a **CPG (central pattern generator)** body-wave + leg pattern, driven through an **impedance (PD) controller**.
We study how to make it walk *fast* and *stable* by tuning the controller. The current research question (2026-07-13,
from Jonathan in Auke Ijspeert's group) is: **can we cure the "fling" / instability by driving the CPG with very small
PD gains + very large commanded amplitude, so the body only *roughly* tracks the signal but stays richly *compliant*?**

**The 4 things you actually do in this project:**
| Task | Tool | Where |
|---|---|---|
| Run a **parameter sweep** (open-loop, many configs at once) | a "cell runner" + an "orchestrator" script | `scripts/analysis/*_sweep_*.py` |
| Train an **RL policy** (closed-loop) | IsaacLab + rl_games PPO | `scripts/analysis/stage_e_train_*.py` |
| **Render a video** (to see what happened) | PyVista **offline** renderer (not Omniverse) | `scripts/sanity/offline_render_from_log_2026-06-22.py` |
| **Analyze** results | plain python + numpy on the saved `.json`/`.npz` | ad-hoc scripts |

**To check what's running right now:**
```powershell
Get-Process python | Select Id,StartTime          # any live IsaacLab job?
nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv   # GPU busy?
```
**The live experiment (2026-07-13)** is the *compliance sweep*. See §4 and `STEP_LOG.md` entry 1 to run it yourself.

**The golden rules** (full list in `CLAUDE.md` at repo root — read it):
1. **ONE GPU job at a time.** Always kill leftover `python.exe` when a job finishes (Omniverse hangs on teardown 10–22 min).
2. **Never edit big/protected files with a normal editor** — they silently truncate. Use `scripts/sanity/safe_write.sh` or a python patcher. Protected: `envs/ controllers/ assets/ scripts/rl/ IsaacLab/` (see CLAUDE.md).
3. **Never put terrain "cheats" in the policy's observation or reward** (severity, block id, x-position, amplitude, wavelength, lookup-tables). It's a hard rule — the whole point is the robot must *sense* the terrain, not be told.
4. Every result number is re-checked by a second, independent computation before it's trusted.

---

## 1. The mental model (the science in 5 minutes)

**The robot.** A centipede: one long trunk of ~19 body segments + legs. The trunk joints bend in **yaw** (left-right,
the traveling wave you see from above), **pitch** (up-down / droop), and **roll**. Each leg has up to 4 joints
(hip-yaw, hip-pitch, tibia, tarsus). The physical model lives in the USD asset `assets/centipede_rebuild_v1_renamed.usd`
(and a matching MuJoCo XML the renderer uses).

**The gait = a CPG.** A ring of coupled oscillators (Kuramoto) produces a **traveling wave** down the body. You don't
command joint angles directly; you command the *shape* of the wave. The controller file is
`controllers/impedance_controller_torch.py` — it is **FROZEN** (we never change its code; sha256 `b724eff9…`). We only
change its **runtime inputs** (the numbers below).

**The knobs (this is the whole vocabulary):**
| Knob | Meaning | Base value |
|---|---|---|
| **wave_number** | how many wave crests fit along the body (spatial frequency of the undulation) | 3.0 |
| **frequency** | gait cycles per second (the "cycle" in bl/cycle) | 1.0 Hz |
| **amplitude** (`body_amp`, `leg_*_amp`) | how big the commanded joint swing is. **A *scale* on a base** (body base = 0.4 rad). | scale 1.0 |
| **PD gains** (`body_kp/kv`, `leg_kp/kv`, `pitch_*`, `roll_*`) | stiffness (kp) + damping (kv) of the impedance controller. Torque law: `τ = kp·(target−q) + kv·(targeṫ−q̇)`. Also *scales* on YAML bases. | see `configs/farms_controller.yaml` |
| **decimation / dt** | physics timestep (`sim.dt`) and how many physics steps per control step. Default 4 ms × 12 = 48 ms control. | 4 ms / 12 |

**The metric.** Speed, expressed in **body-lengths per gait cycle (bl/cycle)** — a scale-free speed.
`bl/cycle = v[m/s] / (frequency × body_length)`, with body_length ≈ 0.102 m and frequency = 1.0 Hz, so `bl/cycle ≈ v[m/s] × 9.8`.
(Standing rule: *all* speeds are reported in bl/cycle.) The current task uses **total distance over the full window**
(net forward displacement ÷ total time) so that a robot that falls halfway is scored low.

**Forward = world −x.** The robot walks in the −x direction; `v_forward = −root_lin_vel_w[:,0]`.

**The terrain.** A heightmap: a single-wavelength rough surface parameterized by **λ (wavelength, mm)** and
**amplitude (mm)**. Roughness rises as amplitude/λ rises. "Flat" = amplitude 0. We test three: **flat / mild (λ90,amp8) /
rough (λ60,amp16)**.

**The current research question (compliance).** Historically we optimized the *PD gains* to death (grid search,
Bayesian opt) and never cured the fling. Jonathan's insight: we never touched **amplitude**. If we make the gains
*tiny* and the commanded amplitude *huge*, the joints are pulled *softly* toward a big command — they track only a
compliant fraction, so the body is soft (absorbs terrain/sudden motion) yet still driven. The compliance sweep tests
this along a coordinated ladder: gains ↓ while amplitude ↑. See §4.

---

## 2. Project map — where everything is

```
Centipede_IsaacLab_RL/
├── CLAUDE.md                 ← THE RULES. Read this. (protected files, forbidden inputs, output org, video workflow)
├── configs/
│   └── farms_controller.yaml ← the BASE controller numbers (amplitudes, gains, freq, wave, dt). §1 knobs live here.
├── controllers/
│   └── impedance_controller_torch.py  ← the CPG + impedance controller. FROZEN. Torque law at pd_torque(); the
│                                          modulation entry is apply_global_6d_modulation() + set_freesearch_runtime().
├── envs/
│   ├── centipede_env.py       ← the IsaacLab environment (obs, reward, reset, done). PROTECTED (safe_write only).
│   └── centipede_env_cfg.py   ← the env config dataclass. dt/decimation live here (sim.dt, decimation).
├── assets/                    ← the robot USD models. Current: centipede_rebuild_v1_renamed.usd (4-DOF legs).
├── scripts/
│   ├── sanity/
│   │   ├── stage_a_vec_sweep_cell_2026-06-17.py   ← THE WORKHORSE "cell runner": boots IsaacLab, applies per-env
│   │   │                                             gains from a JSON, runs one terrain, writes metrics + raw arrays.
│   │   ├── stage_a_vec_sweep_cell_compliance_2026-07-13.py  ← my copy of it + a --physics-dt/--decimation flag (§4).
│   │   ├── offline_render_from_log_2026-06-22.py  ← THE video renderer (CPU, PyVista, reads an _alllink.npz).
│   │   └── safe_write.sh                          ← safe editor for protected/large files.
│   ├── analysis/              ← sweep ORCHESTRATORS + analysis. e.g. a004_wave_roughness_sweep_2026-07-11.py,
│   │   │                         compliance_sweep_2026-07-13.py (§4), stage_e_train_wave_v5_2026-07-13.py (RL).
│   └── optimization/          ← the gain-scale helpers (stage_a_v9_grid_2026-06-24.py = grid_combo, amp10_baseline).
├── outputs/                   ← ALL results. One dated dir per experiment (MANIFEST.md + data/ logs/ figures/ videos/ meta/).
│   ├── analysis/  rl/  optimization/   ← by class
│   ├── RUN_INDEX.md           ← index of runs
│   ├── CURRENT_RESULT.md      ← the single "please verify this" marker (see CLAUDE.md RESULT MARKER PROTOCOL)
│   └── directives/            ← GO_*.md dispatch directives + their DISPATCH_LOG.md (the work trail)
├── docs/                      ← documentation. OFFLINE_RENDERER_PIPELINE.md, RENDERING_PIPELINE_DEBUG_NOTES.md, this HANDOVER/.
└── IsaacLab/                  ← vendored IsaacLab. Run jobs via IsaacLab/isaaclab.bat -p <script.py> ...
```

**The two "memory" systems that carry project state (important — this is *why* it's been agent-driven):**
- `CLAUDE.md` (repo root) — the standing rules, invariants, and known retractions. Loaded every session.
- `C:\Users\wxy22\.claude\projects\…\memory\` — the agent's long-term memory (one fact per file, indexed by `MEMORY.md`).
  This holds hard-won lessons (e.g. "the invisible-floor incident", "bent-leg is default", parameter locks). To take
  over, skim `MEMORY.md` — it's the table of contents of everything learned.

---

## 3. How to operate it (the toolchain)

**Environment.** Python lives in the conda env `isaaclab_50`:
`C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe`. IsaacLab jobs are launched through `IsaacLab\isaaclab.bat -p <script> <args>`
(that batch file sets up the Omniverse/kit python internally). Plain CPU analysis can use the conda python directly.

**Pattern A — a parameter sweep (open-loop).** Two layers:
1. an **orchestrator** (`scripts/analysis/…_sweep_….py`) — a plain CPU script that loops over configs/terrains and, for
   each, writes a `gains.json` (the per-env knob values) and launches…
2. the **cell runner** (`scripts/sanity/stage_a_vec_sweep_cell…py`) — boots IsaacLab once, reads the `gains.json`,
   applies each env's knobs, simulates, and writes `metrics_<tag>.json` (+ optional raw `.npz`).
   Many configs run *in parallel envs in one boot* (fast).
- One GPU job at a time. The orchestrator kills the cell's python between boots.
- Run the orchestrator **detached** (it drives boots for a while) and watch its `AUTORUN_LEDGER.md` for progress.

**Pattern B — RL training (closed-loop).** `IsaacLab/isaaclab.bat -p scripts/analysis/stage_e_train_wave_v5_2026-07-13.py …`
trains a PPO policy whose action is the wave-number. (This was the *previous* direction; paused in favor of compliance.)

**Pattern C — render a video (always the OFFLINE renderer, never Omniverse unless told):**
1. produce an `_alllink.npz` from a cell boot (add `--dump-raw-alllink`);
2. `python -X utf8 scripts/sanity/offline_render_from_log_2026-06-22.py --alllink-npz <..>_alllink.npz --env-index <i> --out-mp4 <..>.mp4 --report-json <..>_audit.json --camera chase --fps 20 --terrain`
3. the renderer runs an inline 8-check audit into the `--report-json`; PASS = all checks true. Use `--terrain` (feet rest ~0.45 mm on the surface), never `--terrain-raw` (robot floats ~17 mm).
- Full details: `docs/OFFLINE_RENDERER_PIPELINE.md`, `docs/RENDERING_PIPELINE_DEBUG_NOTES.md`.

**Watching a long job without babysitting it.** Background bash watchers get killed erratically on this machine; the
reliable pattern is: run the job **detached** (PowerShell `Start-Process … -PassThru`), then watch its log/ledger.

---

## 4. The live experiment, hand-held: the COMPLIANCE SWEEP (2026-07-13)

**Hypothesis (Jonathan / PI).** Cure the fling by *compliance*: tiny PD gains + huge commanded amplitude → the body
roughly tracks the wave while staying soft. Coordinated ladder: all gains × `gain_scale` (down), all amplitudes ×
`amp_scale` (up); **keep** the bio-calibrated pitch spring (k=6e-4, via `--passive-spring --suppress-active-pitch`),
**lower** the active damping (kv); **fix** wave = 3.0 (isolate gain×amp); run physics at **1 ms** (finer than the
usual 4 ms). Metric = **total distance over the full window** in bl/cycle (penalizes falling), plus compliance readouts
(`track_err` = how loosely it tracks, `body_amp_ratio` = realized ÷ commanded swing, fling = yaw-rate peak).

**What code I created and why** (full line-level walk-through in `STEP_LOG.md` entry 1):
- `scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py` — a **copy** of the frozen cell runner with two new
  optional flags `--physics-dt` and `--decimation-override` (default off → byte-identical). I copied instead of editing
  the shared workhorse so nothing else breaks.
- `scripts/analysis/compliance_sweep_2026-07-13.py` — the **orchestrator**: defines the (amp_scale, gain_scale) ladder,
  the three terrains, builds each env's `gains.json`, launches the cell at 1 ms, parses metrics, computes total-distance.

**How to run it yourself** (copy-paste):
```powershell
cd C:\Users\wxy22\Documents\Centipede_IsaacLab_RL
$CPY='C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe'
& $CPY -u scripts/analysis/compliance_sweep_2026-07-13.py --seeds 4242 --terrains flat mild rough
# watch progress:
Get-Content outputs/analysis/compliance_sweep_2026-07-13/AUTORUN_LEDGER.md -Wait
```
**How to read the result.** When it finishes, `outputs/analysis/compliance_sweep_2026-07-13/data/compliance_sweep.json`
holds, per terrain, a row per ladder rung with: `total_bl_cycle` (the headline — bigger = walks further), `track_err`
(higher = more compliant/loose), `bar_real_over_cmd` (realized swing ÷ commanded — small = "roughly tracks"), `fling`
readouts, and `sus_upright` / `buck` (did it stay up). The question: **does a low-gain/high-amp rung beat the a004
baseline (rung E0) on total distance while reducing fling?**

---

## 5. How to change something safely (the traps that have bitten us)

1. **Big/protected files silently truncate** if edited with the normal tool. Use `scripts/sanity/safe_write.sh` or a
   python patcher that reads → edits a string → writes → `ast.parse` verifies. Always AST-check after.
2. **Gains are *scales*, not absolutes.** `body_kp: 0.003` in a gains-json means 0.003 × 0.144 (the YAML base). Same for
   amplitude (× 0.4). If you think in absolutes you'll be off by the base factor.
3. **`effort` is an *absolute* torque cap and is always on.** The baseline value is tiny (~0.002 Nm) — if you don't
   overwrite it to something large (we use 1e3), a big-amplitude command gets clipped to nothing and the robot won't move.
4. **dt/decimation are not CLI flags in the stock cell** — they're hard-coded in `centipede_env_cfg.py`. Override them
   at runtime on the `env_cfg` object (that's what my `--physics-dt/--decimation-override` copy does).
5. **Kill only the job's python, never the driver.** A `Get-Process python | Stop-Process` kills your own orchestrator.
   Snapshot the driver PID first and exclude it (the compliance orchestrator's `kill_py` shows the pattern).
6. **Never add terrain ground-truth to obs/reward** (§0 rule 3). This is the scientific integrity line.

---

## 6. How to keep learning / pick it up

**Reading order for a full take-over:**
1. This file → `CLAUDE.md` (the rules) → `memory/MEMORY.md` (the lesson index; skim the one-liners).
2. `configs/farms_controller.yaml` (the knobs) → skim `controllers/impedance_controller_torch.py`'s `pd_torque`,
   `apply_global_6d_modulation`, `set_freesearch_runtime` (how a knob becomes a torque).
3. One worked example end-to-end: read `scripts/analysis/a004_wave_roughness_sweep_2026-07-11.py` (orchestrator) +
   its `MANIFEST.md`, then `scripts/sanity/stage_a_vec_sweep_cell_2026-06-17.py` (the cell it drives).
4. `docs/OFFLINE_RENDERER_PIPELINE.md` for video.
5. Then `STEP_LOG.md` from entry 1 onward — every change I make, hand-held, in reproducible form.

**Glossary of the jargon you'll see:**
- **CPG** — central pattern generator (the oscillator gait). **fling** — a sudden yaw lurch/instability we've chased.
- **a004** — a specific locked set of gain scales (`outputs/analysis/a004_wave_roughness_sweep_2026-07-11/meta/A004_LOCK.json`).
- **bl/cycle** — body-lengths per gait cycle (the speed unit). **applied_abs** — the absolute wave number (= scale × 3.0).
- **cell / boot** — one IsaacLab launch that simulates a batch of envs. **orchestrator / driver** — the CPU script that loops boots.
- **track_err** — 1 − correlation(actual joint, commanded joint): how loosely the plant follows the command (compliance readout).

_When something here goes stale (a file is renamed, a value re-locked), fix it in place — this doc is meant to stay true._
