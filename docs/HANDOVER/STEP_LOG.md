# STEP LOG — every code change / experiment, hand-held
_A reproducible diary so the PI can take over. Newest entries appended at the bottom. Read `README.md` first._

Format per entry: **what & why** · **exact code change** · **how to run** · **how to read the result** · **traps hit**.

---

## Entry 1 — 2026-07-13 — The COMPLIANCE pivot + the compliance sweep

### What & why
We stopped the RL wave-conditioning campaign. After the PI talked to Jonathan (Auke Ijspeert's group), the new
hypothesis is: the fling was never cured because every past search only tuned **PD gains** (grid search + Bayesian
opt exhausted that axis). The untouched axis is **amplitude**. Drive the CPG with **tiny gains + huge amplitude** →
the joints are pulled *softly* toward a large command, so the body only *roughly* tracks the wave but stays richly
**compliant** (soft, absorbs terrain and sudden motion). Test it as a **coordinated sweep**: all gains × `gain_scale`
(down) while all amplitudes × `amp_scale` (up), across flat/mild/rough terrain, at 1 ms physics, wave fixed at 3.0.
Metric = **total distance over the full window** (bl/cycle) so falling is penalized.

### Exact code change 1 — a copy of the cell runner with a dt override
The physics timestep (`sim.dt`) and `decimation` are **hard-coded** in `envs/centipede_env_cfg.py` (no CLI flag). I
needed 1 ms instead of 4 ms. Rather than edit the shared workhorse `scripts/sanity/stage_a_vec_sweep_cell_2026-06-17.py`
(which many scripts depend on), I made a **copy** and added two optional flags that default to "no change" (so it stays
byte-identical when unused). The copy: `scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py`.

Two edits, both applied with a small python patcher (never a blind editor — protected-file rule):
- **New argparse flags** (inserted after `--gravity-scale`):
  ```python
  parser.add_argument("--physics-dt", type=float, default=None,   # None = cfg default 0.004 (4ms), byte-identical
                      help="override sim physics dt (s); 0.001 for 1ms")
  parser.add_argument("--decimation-override", type=int, default=None,  # None = cfg default 12
                      help="override decimation; pair 0.001 with 48 to keep the 48ms control period")
  ```
- **The runtime override**, injected just before the line `_dt_pol = int(env_cfg.decimation) * float(env_cfg.sim.dt)`
  (so the episode-length auto-sizing sees the new dt):
  ```python
  if getattr(args, "physics_dt", None) is not None:
      env_cfg.sim.dt = float(args.physics_dt)
  if getattr(args, "decimation_override", None) is not None:
      env_cfg.decimation = int(args.decimation_override)
  ```
  This mirrors the existing gravity-override pattern (`env_cfg.sim.gravity = …` a few lines above) — overriding a
  config field at runtime is allowed; editing the frozen controller/env code is not.

**How to reproduce this edit yourself:** copy the file, then in the copy add the two `add_argument` lines and the two
`if getattr(...)` blocks, then verify it parses:
```powershell
$CPY='C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe'
& $CPY -c "import ast; ast.parse(open(r'scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py',encoding='utf-8').read()); print('AST OK')"
```

### Exact code change 2 — the orchestrator
New file `scripts/analysis/compliance_sweep_2026-07-13.py`. It:
1. Loads the a004 gain **scales** (`a004_gains()` → `grid_combo(amp10_baseline(), A004_LOCK tuple)`).
2. Builds a 10-rung **ladder** (`build_layout`): for each `(amp_scale, gain_scale)` rung it deep-copies the a004 gains
   and multiplies amplitudes up / gains down, **keeps `pitch_kp`** (bio-cal), lowers `pitch_kv`, sets `effort=[1e3]×7`
   (so huge-amp torque isn't clipped), and fixes `wave_abs=3.0`.
   - Ladder rungs: `E0` a004 baseline; `E1,E2` amp-alone (amp↑, gain normal); `E3–E7` the *matched* diagonal
     (amp↑ and gain↓ together, product ≈ const → same peak torque, falling stiffness): (3,0.3)…(200,0.005);
     `E8` gain-alone (tiny gain, normal amp — collapse test); `E9` an off-diagonal point.
3. Runs one **boot per terrain** (flat/mild/rough) via the compliance cell at `--physics-dt 0.001 --decimation-override 48`,
   with `--dump-raw-alllink` (for video), and the locked bio-cal flags (`--passive-spring --bio-pitch-k 6.0e-4
   --suppress-active-pitch --cpg-body-coupling-scale 40 --cpg-leg-coupling-scale 80`).
4. Parses `metrics_<terrain>.json` into a per-rung table and writes `data/compliance_sweep.json`.

### How to run
```powershell
cd C:\Users\wxy22\Documents\Centipede_IsaacLab_RL
$CPY='C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe'
& $CPY -u scripts/analysis/compliance_sweep_2026-07-13.py --seeds 4242 --terrains flat mild rough
Get-Content outputs/analysis/compliance_sweep_2026-07-13/AUTORUN_LEDGER.md -Wait   # progress
```
Each boot is one IsaacLab launch (10 envs = 10 ladder rungs in parallel). 1 ms physics ≈ 4× the wall-time of 4 ms.

### How to read the result
`outputs/analysis/compliance_sweep_2026-07-13/data/compliance_sweep.json`, `results[<terrain>_s4242]` = list of rungs:
- **`total_bl_cycle`** — headline: net forward distance ÷ full window, in bl/cycle. Higher = walks further. Compare each
  rung to `E0` (a004 baseline).
- **`track_err`** — 1 − corr(actual leg, commanded leg). Higher = looser tracking = more compliant. We *want* the
  compliant rungs to still move.
- **`bar_real_over_cmd`** — realized body swing ÷ commanded. Small on the high-amp rungs = literally "roughly tracks".
- **`sus_upright` / `buck` / `nan`** — did it stay upright / buckle / blow up.
- **`ny_deg`** (net yaw) as a fling proxy.
**Success looks like:** a low-gain/high-amp rung matches or beats E0 on `total_bl_cycle` while showing lower fling.

### Traps hit (and how they were fixed — learn from these)
1. **`kill_py()` killed the driver.** First launch died instantly. Cause: the orchestrator's between-boot cleanup ran
   `Get-Process python | Stop-Process -Force`, which kills *all* python — including the orchestrator itself (it runs on
   the same python). **Fix:** snapshot the driver PID (and any pre-existing python) at start into a `PRESERVE` set and
   kill only PIDs *not* in it. Pattern is in `compliance_sweep_2026-07-13.py::kill_py` / `_snapshot_preserve`.
2. **A background log-watcher's crash-grep force-killed a healthy job** (earlier, RL run). A too-broad grep matched the
   benign boot line `Warp CUDA error … cuDeviceGetUuid` (it appears in *every* IsaacLab run — it is NOT a crash) and
   killed the job at epoch 5. **Rule:** a crash detector must match only a real python `Traceback (most recent call
   last)` + process death, never a bare "CUDA error"/"Error" substring.
3. **The baseline `effort` is tiny.** If you forget to overwrite `effort` to a large value, the impedance torque is
   clamped to ~0.002 Nm and the robot won't move regardless of amplitude. We set `effort=[1e3]×7`.

### Files touched this entry
- NEW `scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py` (cell copy + dt flags)
- NEW `scripts/analysis/compliance_sweep_2026-07-13.py` (orchestrator)
- NEW `docs/HANDOVER/README.md`, `docs/HANDOVER/STEP_LOG.md` (this hand-over)
- Output dir `outputs/analysis/compliance_sweep_2026-07-13/`

---

## Entry 2 — 2026-07-13 — Compliance sweep RESULTS + a scoping fix (a lesson)

### What & why
Ran the compliance ladder on flat / mild / rough, analyzed it, and hit an important **scoping lesson** worth recording
for the hand-over: I picked the *wrong* "rough". The result is (so far) mostly negative, and that is fine — negative
results are results. Reported honestly.

### The result (flat / mild / rough = λ60a16, roughness 0.267)
- **The compliance mechanism works** as designed: as amplitude↑ and gain↓, tracking loosens (`track_err` 0.33→0.65) and
  the body realizes as little as **0.7%** of a huge command while staying upright (rung E7). The knobs do what Jonathan
  described.
- **But the a004 baseline never flings on these terrains** — net-yaw ≈ 1°, upright 0.97, it walks fine. So there is
  nothing to cure, and compliance mostly *costs* forward distance. The huge-amplitude rungs go so floppy (legs included)
  they stop propelling or fall. Only a *modest gain reduction* (rung E8: gain×0.1, normal amplitude) helped, and only on
  mild (+12%). Compliance can even *introduce* fling (E8 on rough: net-yaw 64° vs baseline 1°).

### THE LESSON (why this matters for taking over the project)
I set "rough" = λ60a16 (roughness 0.267). But **a004 was optimized on exactly that grid** (the a004 sweep covered
λ{120,90,60,30}×amp{8,12,16,20}) — so a004 is comfortable there. The fling/instability the whole hypothesis targets
lives at the **roughest** terrains (λ30a16 = 0.533, λ30a20 = 0.667). **Testing an intervention on a terrain where the
baseline isn't broken can't tell you if the intervention fixes the breakage.** Always test where the problem actually
manifests. Fix: I added `vrough` (λ30a16) and `xrough` (λ30a20) and re-ran.

### Exact code changes this entry
- NEW `scripts/analysis/compliance_analyze_2026-07-13.py` — prints the per-terrain ladder table, writes the headline
  figure (`figures/compliance_ladder.png` + `docs/figures/…`) and `data/compliance_verdict.json` (per-terrain best rung +
  whether compliance beats baseline + the roughness→best trend).
- NEW `scripts/analysis/compliance_render_2026-07-13.py` — renders each terrain's baseline (E0) + best rung to mp4 via
  the offline renderer with the inline 8-check audit; writes `meta/render_summary.json`.
- EDIT `compliance_sweep_2026-07-13.py`: added `("vrough",30,16),("xrough",30,20)` to `TERRAINS`.
- EDIT `compliance_analyze_2026-07-13.py`: added those two to `TERR_ORDER`/`ROUGHNESS`/figure colors.

### How to run (yourself)
```powershell
$CPY='C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe'
# 1) the sweep (already ran flat/mild/rough; this adds the roughest two):
& $CPY -u scripts/analysis/compliance_sweep_2026-07-13.py --seeds 4242 --terrains vrough xrough
# 2) analysis + figure + verdict (reads the ckpt, all terrains present):
& $CPY -X utf8 scripts/analysis/compliance_analyze_2026-07-13.py
# 3) videos (run AFTER any Isaac boot finishes — render competes for RAM):
& $CPY -X utf8 scripts/analysis/compliance_render_2026-07-13.py --terrains mild vrough xrough
```

### How to read it
`data/compliance_verdict.json` → per terrain: `best_tag`, `best_total` vs `baseline_total`, `compliance_beats_baseline`,
`gain_improvement_pct`. The TREND print shows optimal (amp,gain) vs roughness. Question: does any compliant rung beat
baseline on `total_bl_cycle` on the *roughest* terrains, where the baseline is actually stressed?

### Traps hit
- **Rendering is slow (~1.5 min/clip)** and can be hard-killed if it runs while an Isaac sim is squeezing RAM. Render
  only when no boot is running; batch clips; give it a long timeout / run it detached.
- **A monitor's exit-grep matched a *stale* log line** ("wrote compliance_sweep" from the previous run's DONE), so it
  fired "done" 1 min in. Lesson: make completion patterns specific to the *current* run (e.g. the last boot's exact tag),
  not a phrase that also appears in old log lines.

### (pending) roughest verdict — appended when vrough/xrough finish

---

## Entry 3 — 2026-07-13 — The launch artifact, the metric fix, and the titration protocol

### What & why (the most important lesson so far)
The compliance sweep produced an apparent huge win (rung E7 on the roughest terrain: "0.809 bl/cycle", 3.8x the
baseline). Because a too-good number must be verified, I recomputed it from the raw trajectory — and it was a **metric
artifact**: the robot was **flung airborne** (root z 0.025 -> 0.755 m, 0 alive frames). The cell's `fwd_signed_m` metric
scores launched/dead envs as wins. **Never trust a headline number from a metric without checking the raw trajectory.**

### The metric fix (reusable)
`scripts/analysis/compliance_robust_metric_2026-07-13.py` recomputes total distance from the `_alllink.npz` ROOT
trajectory and flags failures: **launched** (root z-rise > 0.10 m), **teleport** (any single 48 ms step > 50 mm =
lurch/whip), **dead** (alive fraction < 0.8). A config is VALID only if grounded + alive + smooth; invalid configs get
`corrected_bl = min(total, 0)` so a launch can never read as a win. With this, **every** compliance rung on **every**
terrain is invalid — the a004 baseline is the best valid gait everywhere (flat 0.379 -> vrough 0.209). Clean negative
result, rigorously verified.

### The methodology correction (PI pushed on this — rightly)
What I ran was a 10-rung **ladder** (a diagonal path through amp x gain), not a dense grid — fast because the 10 configs
run as parallel envs in one boot. Fair criticism. I built `compliance_grid_2026-07-13.py` (a proper dense 2D grid,
body-only + kept-damping launch-fix, auto-batched to <=12 envs/boot for the 16 GB RAM cap). But the PI then gave a much
better, PRINCIPLED search:

### THE TITRATION PROTOCOL (PI 2026-07-13) -- `compliance_titrate_2026-07-13.py`
Instead of gridding blindly (which just flew apart), match the REALIZED motion, not the commanded:
1. **Stage 1** — at DEFAULT amplitude (x1), lower the body control gains (kp AND kv together) until forward motion /
   realized body wave just vanishes -> the maximally-compliant gain `g*`.
2. **Stage 2** — at `g*`, raise the amplitude COMMAND gradually until the REALIZED body wave (`bar04` = body_amp_ratio,
   normalized to the default 0.4 rad; baseline ~= 1.0) returns to the baseline's ~1.0. Now the command is large but the
   realized motion == baseline => a compliant plant that still moves like the baseline, and it STOPS before the launch
   regime (we cap realized at default, not command).
Body-only (legs held a004 so they propel + don't whip). Metric = robust (grounded+alive+no-teleport) + `bar04`.

**How to run:**
```powershell
$CPY='C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe'
# Stage 1: gain-down at default amp on flat
& $CPY -u scripts/analysis/compliance_titrate_2026-07-13.py --terrain flat --label stage1 `
   --configs 1:1 1:0.3 1:0.1 1:0.03 1:0.01 1:0.003 1:0.001 1:0.0003 1:0.0001
# read the table -> pick g* (gain where fwd motion ~ 0). Stage 2: amp-up at g* (example g*=0.01):
& $CPY -u scripts/analysis/compliance_titrate_2026-07-13.py --terrain flat --label stage2 `
   --configs 1:0.01 2:0.01 3:0.01 5:0.01 10:0.01 20:0.01 30:0.01 50:0.01 100:0.01
# find amp* where bar04_realized ~= 1.0 (baseline). (amp*, g*) is the compliant config -> then verify on rough/vrough.
```

### Traps hit
- **Too-good number = verify from raw before believing.** The whole "compliance wins" story was a metric artifact.
- **16 GB RAM caps ~12 envs/boot** and forbids rendering + booting at once (both OOM-killed) -> serialize.
- **Video audit "fails" on a launch clip is correct** — the robot flies off-screen; that clip documents the failure.

### Files this entry
- NEW `compliance_robust_metric_2026-07-13.py`, `compliance_grid_2026-07-13.py`, `compliance_titrate_2026-07-13.py`,
  `compliance_render_all_2026-07-13.py`, `compliance_launchfix_2026-07-13.py`
- 19 vrough+ videos in `outputs/analysis/compliance_sweep_2026-07-13/videos/`

---

## Entry 4 — 2026-07-13 — Titration result: the gait is DAMPING-driven, not stiffness-driven

### The two titrations (flat, body-only)
Ran the PI titration two ways:
- **kv DOWN (kp AND kv lowered together)** — the literal "control parameters down". Stage 1: forward motion breaks
  below gain ~0.01. Stage 2 (raise amp at g*=1e-3): the realized wave *does* come back (bar04 -> 1.4 at amp 100), but
  the motion **thrashes** — teleport-flagged, forward motion <=0, z-rise low (~0.02, so NOT airborne, it's in-plane
  incoherent lurching). Matching realized *amplitude* != a coherent propulsive wave.
- **kp ONLY (kv KEPT at a004 damping)** — lower ONLY the stiffness. **Walks identically to baseline at EVERY kp down to
  1e-4x** (body_kp runtime 4e-8, ~zero stiffness): forward 0.36-0.38 (= baseline), realized bar04 0.998 (= default),
  smooth (2.5 mm/step), stable. No amplitude increase needed.

### The mechanistic conclusion (important for the project)
`tau = kp*(target-q) + kv*(target_dot-qdot)`. Zeroing kp changes nothing (as long as kv stays) => the **gait is driven
by the velocity term kv, not the position stiffness kp**. The baseline's kp was already near-negligible. So:
- "Compliance via low kp" gives a plant identical to the baseline on flat (kp wasn't doing much).
- The under-damping (from *also* lowering kv) was the entire cause of the earlier thrash/launch.
- **The tension:** true position-compliance would need low kv too, but the gait NEEDS kv to stay coherent. So there's a
  hard floor: you cannot remove the damping without losing the walk.

### `--kp-only` mode (added to compliance_titrate)
`cfg_env` now honors a `--kp-only` flag: lower only body/roll kp, keep kv at a004. Use it to separate
stiffness-compliance from damping.

### What's testing now
kp-sweep (kv kept) on the roughest terrain (vrough=0.533): does the ~zero-stiffness-but-damped plant handle bumps
BETTER than the stiff-kp baseline (the actual point of compliance)? If soft kp doesn't change rough behavior either,
then kp is simply not a useful lever on this morphology and the a004 impedance is already near its compliant limit.

### How to run
```powershell
$CPY='C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe'
& $CPY -u scripts/analysis/compliance_titrate_2026-07-13.py --terrain vrough --label roughkp --kp-only `
   --configs 1:1 1:0.1 1:0.01 1:0.001 1:0.0001
```

---

## Entry 5 — 2026-07-13 — VERDICT: compliance doesn't beat a004 (with the mechanism)

**Confirmation (noise control):** on vrough the baseline at 9 spawns spans fwd [0.155, 0.257] (std 0.032) with NO kp
change — WIDER than the whole kp-sweep [0.178, 0.245]. So the earlier "+17% best kp" was +1.1σ spawn noise. Combined
with realized bar04=0.998 identical across kp 1->1e-4, **kp is inert**. (Resisting this noise-claim is the lesson from
the E7 launch artifact — always check the noise band + the realized-motion channel.)

**VERDICT.** Compliance via the impedance controller (kp / kv / amplitude) does NOT improve on a004 on any terrain.
Full phase space, all verified with the robust metric:
- kv kept, default amp -> identical to baseline (kp inert).
- kv kept, huge amp -> LAUNCH (kv*target_dot feed-forward injects energy).
- kv down (kp+kv), any amp -> THRASH (under-damped, incoherent wave, no propulsion).
**Mechanism:** the gait is velocity(kv)-driven; kp is negligible; kv is BOTH the drive and the effective stiffness, and
it cannot be lowered without losing a coherent propulsive wave. a004 is at a coherence limit -> this is *why* the fling
never yielded to impedance tuning (every past search lived in a space where the gait-critical kv can't be reduced).
Figure: `docs/figures/compliance_VERDICT_2026-07-13.png`. Marker: `outputs/CURRENT_RESULT.md` (P0, UNVERIFIED).

**Where to go next (needs a real STRUCTURAL change, not gains):** true torque/force compliance, morphological softness
(soft joints/materials in the model), or a different actuation model — OR accept a004 and pick a new question.

---

## Entry 6 — 2026-07-13/14 — Controller-dominance: pitch ζ→0 tail + ALL joints (with the generalized harness)

**What & why.** PI asked to (a) resolve the ζ→0 tail of the pitch ring-down ("precisely draw settle_time ~ 1/ζ"), then
(b) "记住这个关系,对其他关节也做同样的事". Both done. The finding: the impedance controller τ=kp(q_ref−q)+kv(q̇_ref−q̇)
sets the *effective* stiffness and damping of an isolated joint almost exactly — `k_eff=kp`, `c_eff=kv`,
`ζ_eff/ζ_cmd≈1.02` — and this holds on **every** joint family, because the settable armature (1e-5) dominates the
inertia so all joints share one plant.

**Code I changed (all in `scripts/analysis/`, none protected):**
- `audit_ringdown_2026-07-13.py` — GENERALIZED the ring-down harness. New flags:
  - `--target-family {pitch,body_yaw,roll,legbase_yaw,legbase_pitch,legbase_roll,knee}` — pick which joint family to
    perturb (was pitch-only).
  - `--target-dofs D D D …` — PER-ENV target dof (one per config) so several joint families are characterized in ONE
    boot. Overrides `--target-family`.
  - `--list-joints` — boots, prints each family's representative dof+joint-name, exits (use it to get the dof indices).
  - `--target-k` — which segment/leg index within a family.
- `compliance_ultralow_zeta_2026-07-13.py` — driver: ultra-dense ζ∈[2e-4,0.05] tail (13 envs, 15 s window).
- `compliance_divergence_plot_2026-07-13.py` — merges the ζ sweeps, fits α, draws the 1/ζ divergence + hidden floor.
- `compliance_zeta_video_2026-07-13.py` — sweeps ζ, morphs the ring-down (the "continuous, not binary" demo video).
- `compliance_multijoint_2026-07-13.py` — driver: same controller-dominance pack on all 7 families (2 boots, per-env).
- `compliance_multijoint_plot_2026-07-13.py` — per-joint k_eff/c_eff/ζ table + comparison bars (overshoot-method ζ,
  validated on known pitch data to 2%).
- `compliance_multijoint_video_2026-07-13.py` — all 7 joints' ring-downs overlaid (they coincide → identical response).

### How to run (characterize ANY joint yourself)
```powershell
# 1) see the joint families' dof indices
IsaacLab\isaaclab.bat -p scripts\analysis\audit_ringdown_2026-07-13.py `
  --mode spring --configs 1e-5:0.1:0:10:0 --out scratch.npz --list-joints
# 2) ring down one family (e.g. knee), 4-config pack, isolated 1-DOF, 1 ms:
python scripts\analysis\compliance_multijoint_2026-07-13.py      # runs all 7 (edit BOOTS/FAM_DOF to subset)
python scripts\analysis\compliance_multijoint_plot_2026-07-13.py # -> docs\figures\fig_multijoint_dominance_2026-07-13.png
```
Config string = `armature:kp:kv:q0deg:qd0degps`. ζ = kv / (2√(kp·armature)). Read ζ_eff/ζ_cmd≈1 and I_eff/armature≈1
off the table — that IS the controller-dominance check.

**Deliverables:** `docs/figures/fig_zeta_divergence_2026-07-13.png`, `fig_multijoint_dominance_2026-07-13.png`;
videos `DEMO_zeta_sweep_2026-07-13.mp4`, `DEMO_multijoint_ringdown_2026-07-13.mp4` (+ per-clip audit JSON). Data +
merged tables in `outputs/analysis/compliance_audit_2026-07-13/data/`. Memory: `project_controller_dominance_pitch`.
