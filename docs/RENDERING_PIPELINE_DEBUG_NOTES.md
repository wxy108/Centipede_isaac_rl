# Rendering Pipeline Debug Notes

> Authoritative reference for **why headless RGB capture broke during the
> Phase 2 robustness audit**, **how the Phase 3 fix works**, and the
> **mandatory checks** that any future rendering script must apply.
>
> Read this before touching `scripts/sanity/render_*.py` or writing a
> new diagnostic video script.

---

## ⇄ START HERE — pick your path (PI role split, 2026-06-22)
- **A) VERIFICATION / our own checking** → **OFFLINE third-party renderer** (CPU, deterministic, no RTX race, geometry-exact, ~6.5×). Full how-to = **`docs/OFFLINE_RENDERER_PIPELINE.md`** (role split + dump-raw-alllink + terrain XY/`to_mujoco_terrain` Z + verify-by-foot-contact). Default for checking.
- **B) DELIVERABLE EXPORT** → **Omniverse/Isaac** → the sections BELOW: "THE ONE WAY + EARLY-ABORT", "COLOR-AGNOSTIC audit", "Phase 3 black-frame". Workflow: **verify offline FIRST, then export.**

---

## 2026-05-11 — Phase 3 black-frame diagnosis and fix

### Symptom

20 condition-specific MP4s rendered by
`scripts/sanity/render_robustness_diagnostic.py` were **all 100 % black
frames**:

- `outputs/visual_audit/phase2_robustness_sweep_videos/videos/*.mp4`
- Every file exactly **6 037 bytes** (H.264 keyframe-only encoding of
  bit-identical black frames).
- Frame audit (`outputs/visual_audit/phase2_robustness_sweep_videos/reports/frame_audit.md`):
  100 / 100 black, 99 / 99 frozen, motion_mean = 0.00 per file.
- A separate single-rollout control rendered by the known-good
  `scripts/sanity/render_policy_visual_audit.py --zero-action --camera oblique`
  in the same session (`_DIAG_zero_action_oblique.mp4`, 1.34 MB,
  mean RGB 217.86) proved the env / RTX pipeline itself was fine.
- Physics metrics in `outputs/visual_audit/phase2_robustness_sweep_videos/video_index.csv`
  remained valid (v_fwd, F_max, vz_rms, z_p2p, n_dones came from
  `base_env.robot.data.root_pos_w` and `contact_sensor.net_forces_w`,
  not the RGB buffer).

### Root cause

`render_robustness_diagnostic.py` ran a multi-condition reset loop
inside one IsaacLab boot:

```python
# render_robustness_diagnostic.py — simplified
cam_path, cam_xform_op = place_camera(stage, "/World/DiagCamera", ...)
annot = attach_rgb_annotator(cam_path, ...)   # ← created ONCE
for block_idx in SEVERITY_BLOCKS:
    for cand_name, wave, imp in CANDIDATES:
        base_env._spawn_rng = PinnedSpawnRng(block_idx, ...)
        inner.reset()                          # ← per-condition reset
        ... warmup ...
        for f in range(n_frames):
            inner.step(actions)
            base_env.sim.render()
            rgb = annot.get_data()             # ← zero buffer from iter 2
```

The first `inner.reset()` inside the loop **detaches the
replicator render product from the live camera prim** at the GPU
side. The Python `annot` handle is still alive but
`annot.get_data()` returns zero-filled buffers for every subsequent
render. Failure is silent — there is no Python-side error.

The renderer's own `failed_frames` counter does **not** catch this
because it only checks `rgb is None or rgb.size == 0`. A zero-filled
non-empty buffer of the right shape passes the check.

### Restored pipeline (Phase 3, 8 / 8 PASS)

`scripts/sanity/render_single_condition_video.py`
(driven by `scripts/sanity/render_debug_phase3_serial.sh`):

- **One condition per fresh IsaacLab subprocess** — the camera / render
  product / annotator are created on every fresh boot from scratch, so
  no reset can detach them.
- Inside each subprocess: **one** `env.reset()`, then warmup
  (physics + render), then one capture loop, then encode + exit.
- Result: 8 / 8 priority videos at 2.2–3.2 MB each, motion_mean
  0.76 – 2.15, 0 black frames, 0 white frames.

The proven sequence inside a single render subprocess:

1. `AppLauncher(args)` boots Isaac Sim headless.
2. Force RTX raytrace via `carb.settings`.
3. `inner = gym.make(task, cfg=env_cfg)`.
4. **Before any reset**: `add_lighting`, `place_camera`,
   `attach_rgb_annotator` to a fresh `rep.create.render_product`.
5. Build the `RlGamesVecEnvWrapper` (needed for the `env.step/reset` API
   even in zero-action mode); register `vecenv` / `env_configurations`.
6. (Optional) monkey-patch `base_env._spawn_rng` to pin spawn.
7. (Optional) monkey-patch `base_env._apply_global_6d_action` to apply
   fixed controller scales every step.
8. (Optional) Load the rl_games player.
9. **Single** `env.reset()`.
10. Physics warmup (`env.step(zeros)` × 20 policy steps).
11. Render warmup (`base_env.sim.render()` × 30 frames).
12. Capture loop: per frame, `env.step()` → update chase xform →
    `base_env.sim.render()` → `annot.get_data()` → append.
13. Encode MP4 (`imageio.v2`, libx264 quality=8) + JSON sidecar.
14. Print `[viz] report -> <out_path>` (the success marker).

The proven camera defaults (from
`outputs/visual_audit/pass12_B4_2026-05-10/single_follow_closeup/closeup_visual_verdict.md`):

- camera: `chase`
- chase-offset: `(-0.25, -0.20, 0.18)`  (eye 32 cm behind-left, 18 cm up)
- chase-alpha: `0.85`
- resolution: 1024 × 576 @ 20 fps
- duration: 5 s capture (100 frames)
- n-warmup: 30 render frames
- steps-warmup-phys: 240 substeps (20 policy steps ≈ 0.96 s)

### Residual non-determinism (the rebound bug)

Even with the single-condition recipe, some renders **still** produce
all-zero buffers occasionally. Failure rate this session:

| condition family | first-attempt failures |
|---|---|
| `wave_scale = 1.0` | 0 / 4  (0 %) |
| `wave_scale = 2.0` | 2 / 3  (67 %) — v02, v04 |
| `wave_scale = 2.5` | 1 / 1  (100 %) — v07 |

Same args succeeded on a retry. The failure is non-deterministic and
correlated with non-baseline wave_number. Hypothesis: a GPU /
replicator initialisation race that interacts with the controller
monkey-patch's first step. Hypothesis has not been confirmed; it has
been **mitigated by retry**, not eliminated.

### Mandatory checks for any future rendering script

Any script that produces MP4s for scientific inspection MUST do all
of the following:

1. **Single condition per subprocess.** Do not loop multiple
   conditions inside one boot with a pre-bound annotator. If you
   genuinely need multi-condition rendering inside one boot, **destroy
   and re-create** the render product *and* annotator after every
   `env.reset()` and confirm by running a frame audit on the very
   first post-reset frame.

2. **Do not trust `failed_frames == 0`.** The
   `rgb is None or rgb.size == 0` check used by today's renderers
   does not catch zero-content non-empty buffers. Add a content
   check: `np.asarray(rgb).max() > 0` or
   `np.asarray(rgb).mean() > 1.0`.

3. **Post-render verification:**
   - File size check: `os.path.getsize(mp4) > 50_000` for a 5 s
     1024×576 H.264 video. All-black files compress to ~6 KB.
   - Motion-mean check: run the frame audit
     (`scripts/sanity/build_render_debug_phase3_reports.py` or
     `scripts/sanity/audit_video_frames.py`) and verify
     `motion_mean > 0.5`.

4. **Automated retry on failure:** wipe the failed MP4 + JSON + log,
   re-run the same single-condition subprocess up to 3 times. See
   `scripts/sanity/retry_black_frame_renders.sh` for the pattern.

5. **Force-kill `python.exe` on `[viz] report ->` marker.** IsaacLab's
   Omniverse teardown hangs for 5–6 minutes after the marker fires.
   Force-killing the launcher's `python.exe` drops cleanup to under
   5 s and frees the GPU for the next render. See
   `scripts/sanity/run_isaaclab_with_watchdog.ps1` and
   `scripts/sanity/render_debug_phase3_serial.sh` for the kill-on-success
   pattern.

### Explicit rule

> **Do not render multiple reset conditions in one
> annotator/render-product loop unless the camera/render product is
> explicitly recreated and verified after every reset.**
>
> **Preferred method: one condition per subprocess for diagnostic
> videos.**

### Tooling — what to use, what not to use

| script | status |
|---|---|
| `scripts/sanity/render_single_condition_video.py` | **CANONICAL** — one render per boot, with controller-pin and spawn-pin monkey-patches |
| `scripts/sanity/render_debug_phase3_serial.sh` | **CANONICAL DRIVER** — Bash, force-kill on `[viz] report ->`, idempotent skip-existing |
| `scripts/sanity/retry_black_frame_renders.sh` | **CANONICAL RETRY** — wipes size<50 KB MP4s and re-invokes the serial driver |
| `scripts/sanity/build_render_debug_phase3_reports.py` | **CANONICAL AUDIT** — reads sidecars, runs frame audit, emits CSV + MD reports |
| `scripts/sanity/render_policy_visual_audit.py` | OK for one-shot policy / zero_action videos (no fixed scales) |
| `scripts/sanity/render_robustness_diagnostic.py` | **BROKEN — do not use until the multi-condition reset bug is fixed.** Preserved on disk for reproducibility of the bug only |

### Deliverables produced under the restored pipeline

`outputs/visual_audit/render_debug_phase3/`:

```
videos/v0[1-8]_*.mp4               (8 / 8 PASS, 2.2–3.2 MB each)
sidecars/v0[1-8]_*.json            (per-video metadata + physics metrics)
frames/v0[1-8]_*_mid.png           (per-video mid-rollout thumbnails)
reports/RENDER_DEBUG_REPORT.md     (long-form diagnosis)
reports/frame_audit.md             (8 / 8 PASS, motion_mean 0.76–2.15)
reports/video_index.md             (metadata table)
video_index.csv                    (machine-readable)
```

Memory note for future Claude sessions:
`~/.claude/projects/.../memory/reference_isaaclab_render_black_frame_race.md`.

---

## 2026-06-21 — half-clip WHITE / BLANK / over-exposed failure mode + COLOR-AGNOSTIC audit (mandatory)

### Symptom (a NEW failure mode, distinct from the all-black one above)
A 2×2-demo clip `c0_g6e5_rough_a16.mp4` had **~53% of its frames pure WHITE/blank** (the robot only appears
in the rest), yet it **PASSED the per-frame audit** (luminance 215.7, motion>0, black-frac 0) and the size
check (1.1 MB > 50 KB). The black-only audit + the size gate are BLIND to white/over-exposed/blank frames.
Quantified by the hardened audit: `blank_frac 0.530, min spatial-std 0.05` (pure-uniform frames).

### Root cause (DIAGNOSED, not guessed — verified by the fix)
`--reattach-after-warmup` (re-creating the render product + annotator AFTER the physics/render warmup, the
fix for the all-black *rebound*) **RESETS the RTX raytrace accumulation**. The first frames captured after the
re-attach are then UNDER-CONVERGED → over-exposed / white, until RTX re-converges. On the FIRST boot of a run
this re-convergence is slowest (c0 = the first/only boot of a standalone render → ~half the clip white). The
re-attach probe even showed `probe max=239` (near-white) on the bad run. (The SAME re-attach also still hits
the intermittent all-black lottery, `probe max=0`.)

### Fix (CONFIRMED)
For controller-pinned single-clip renders, **`--no-reattach-after-warmup` + a longer render warmup
(`--n-warmup 50`)**: rely on the INITIAL warmup to converge RTX and do NOT reset it. c0 went from 53% white
→ **clean on the FIRST attempt** (199 s, full clip shows the robot) with `--no-reattach`. (Trade-off: without
the re-attach you re-expose the all-black rebound for *flat + non-baseline-wave* renders; for those keep the
re-attach but add render-warmup AFTER it, or just rely on retry-until-the-hardened-audit-passes.)

### MANDATORY standing rule — the per-frame audit is now COLOR-AGNOSTIC
Black-frac alone is INSUFFICIENT (it passed a half-white clip 2026-06-21). Every clip for scientific
inspection MUST pass ALL of:
- **black_frac == 0**  (per-frame mean luminance < 8)
- **blank_frac == 0**  (per-frame spatial pixel **std < 6** → a UNIFORM frame, black OR white — color-agnostic)
- **bright_frac == 0** (per-frame mean luminance > 240 → near-white / over-exposed, catches white even if its
  std isn't tiny, e.g. a smooth gradient)
- **motion_mean > 0**  (freeze guard)

Tooling: `scripts/optimization/stage_a_v5_frame_audit_2026-06-21.py` (audits a list/glob of mp4s; has
`--make-blank-test` to self-verify it catches a deliberately-blanked frame — confirmed: a 6-white-frame
injection → blank_frac 0.03, FAIL). The render driver
`scripts/optimization/stage_a_v5_render_batch_2026-06-20.py` now uses this hardened check (`hardened_ok()`) as
its retry criterion, so a black OR white/blank clip triggers a re-boot (the old size>50 KB check let the
half-white c0 through). Use it for EVERY render; do NOT trust luminance/size/black-frac alone.

---

## 2026-06-22 — THE ONE WAY (canonical path) + INLINE EARLY-ABORT (the time fix)

**Why renders kept "taking too much time":** the RTX black/white-frame race is non-deterministic and CANNOT
be eliminated (an Omniverse/replicator init quirk), BUT we made each failure EXPENSIVE — a bad buffer wasn't
detected until AFTER the full capture loop + encode, then Omniverse teardown hung 5–6 min, so the watchdog
killed it at its deadline (≈12 min wasted PER bad render, × retries). Plus 16 render scripts (≥2 broken,
2 competing "canonical") meant re-deriving or mis-picking the recipe each campaign.

### Canonical path — use ONLY this for Stage-A controller clips
| role | script | notes |
|---|---|---|
| **DRIVER** | `scripts/optimization/stage_a_v5_render_batch_2026-06-20.py` | one clip/boot, watchdog-wrapped, hardened `hardened_ok()` post-audit + auto-retry (`--max-retries`), episode auto-extend, arena via `--lam`/`--extra` |
| **RENDER (capture)** | `scripts/sanity/stage_a_v2_render_winner_2026-06-18.py` | the one-condition capture; now has the INLINE EARLY-ABORT below |
| **AUDIT** | `scripts/optimization/stage_a_v5_frame_audit_2026-06-21.py` | color-agnostic (black+white+blank+motion); `--make-blank-test` self-verify |

### INLINE EARLY-ABORT (added 2026-06-22 — the time fix)
The render script now checks the FIRST 13 captured frames; if they are ALL uniformly black/white/blank
(mean<8 / mean>240 / spatial-std<6 — same criteria as the hardened audit) it prints `[viz] EARLY-ABORT`
and `os._exit(2)` **immediately**, instead of capturing+encoding the full clip and hanging to the deadline.
⇒ a lost-race boot now costs ~boot + 0.6 s (then the driver re-boots), NOT ~12 min. The check is
try-wrapped so it can NEVER break a GOOD render, and only fires when ALL early frames are definitively bad
(real scenes are mean 50–220, std≫6). It also catches the front-loaded half-white clip (c0) in 13 frames.
Snapshot: `stage_a_v2_render_winner_2026-06-18.py.before_earlyabort_2026-06-22`.

### QUARANTINED — these refuse to run (exit 3) unless `--i-understand-this-is-quarantined`
- `scripts/sanity/render_robustness_diagnostic.py` — multi-reset annotator detach → all-black (§1 bug).
- `scripts/sanity/p4_v34_decouple_render_2026-06-01.py` — force-installs the WRONG arena, ignores
  `--transition-*` (CLAUDE.md). Snapshots `*.before_guard_2026-06-22`.

### Pre-
### Pre-flight checklist (every Omniverse export render)
1. Driver = `stage_a_v5_render_batch` (or extend it). One clip per boot. Pass the arena explicitly.
2. Co-launch the watchdog; force-kill python on `[viz] report ->`.
3. Inline early-abort makes lost-race boots cheap; driver auto-retries (`--max-retries 4`).
4. Run the hardened color-agnostic audit on every clip; ship the per-clip audit JSON.
5. For >8 s clips the render auto-extends `episode_length_s` (else mid-clip reset teleports the robot).
6. NEVER use a quarantined script; NEVER trust size/black-frac/`failed_frames==0` alone.
