# 5. Canonical Workflow (the runbook)

The end-to-end "one way" to produce a verified clip, plus the pre-flight checklist. Source:
`RENDERING_PIPELINE_DEBUG_NOTES.md` (the proven recipe) + the project CLAUDE.md video standing rule.

## 5.1  Roles of the five scripts (`scripts/`)
| Script | Role |
|---|---|
| `offline_render_from_log_2026-06-22.py` | **PyVista offline renderer** — render + extended audit from a `--dump-raw-alllink` npz. CPU, deterministic. Use for verification. |
| `stage_a_v2_render_winner_2026-06-18.py` | **Omniverse capture** — the canonical RTX capture (warmup → render). Inline early-abort at :801-831. Use for deliverable export. |
| `stage_a_v5_render_batch_2026-06-20.py` | **Driver** — boots the capture per design, `--max-retries 4`, `hardened_ok()` gate, deletes stale MP4s. |
| `stage_a_v5_frame_audit_2026-06-21.py` | **Hardened audit** — color-agnostic black/white/blank/freeze; `--make-blank-test` self-check. |
| `run_isaaclab_with_watchdog.ps1` | **Watchdog** — force-kills the launcher's python on the completion marker (teardown hang). Co-launch with EVERY Omniverse job. |

## 5.2  The proven Omniverse capture recipe (the critical bit)
From the debug notes — the single most important fact: **you must step physics with zero-action BEFORE the
first `sim.render()`**, or the annotator returns a persistent zero buffer (all-black).
```
1. Create env / camera / render product / annotator (ONCE per subprocess).
2. PHYSICS WARMUP: for ~20 steps: env.step(zeros)   # settle physics BEFORE any render
3. RENDER WARMUP: ~30-50 sim.render() calls          # let RTX converge (raise to 50 if white/strobe)
4. Capture loop: per frame  -> step policy -> sim.render() -> annot.get_data()
5. Per frame: assert rgb.max() > 0 (retry up to 4x, else substitute last good frame)
6. Early-abort check at frame 13 (3.1).
7. Encode (imageio.v2 libx264 q8; cv2 fallback).
8. On completion marker the watchdog force-kills python (teardown hang).
```
Camera: **chase camera** following the head (body ≈ 5 cm). Defaults from the close-up notes:
`--chase-offset -0.25 -0.20 0.18`, `--chase-alpha ≈ 0.7`, `--n-warmup 30` (50 if white/strobe).
Arena: use an **arena-correct** script and pass the real `--transition-*` params. **NEVER**
`p4_v34_decouple_render_2026-06-01.py` (forces the wrong arena; quarantined).

## 5.3  The default path: verify offline, export once
```
STEP 0  Read docs/RENDERING_PIPELINE_DEBUG_NOTES.md (always, before any export).
STEP 1  One GPU rollout with --dump-raw-alllink  -> per-link world-pose npz.
STEP 2  PyVista offline render + extended audit   -> verify gait/terrain/pose (deterministic).
        (Iterate camera/lighting here for FREE — no GPU, no race.)
STEP 3  Foot-contact test if the arena changed (lowest foot < ~1 mm above terrain).  (4.3)
STEP 4  Only if a photoreal deliverable is needed: Omniverse capture via the driver
        (--max-retries 4) + watchdog, ONE condition per subprocess.
STEP 5  Hardened audit every Omniverse clip (black AND white AND blank AND freeze, motion>0,
        size>50 KB); ship the per-clip audit JSON.
STEP 6  For >8 s clips, extend episode_length_s (else a mid-clip env reset teleports the robot).
```

## 5.4  Pre-flight checklist (every render)
- [ ] Read `RENDERING_PIPELINE_DEBUG_NOTES.md` first.
- [ ] Arena-correct script; real `--transition-*` params; NOT a quarantined script.
- [ ] One condition per subprocess (or recreate+audit the render product after each reset).
- [ ] Physics warmup (`env.step(zeros)×~20`) BEFORE the first `sim.render()`.
- [ ] Render warmup ≥30 (≥50 if white/strobe seen); decide re-attach vs no-reattach (2.3).
- [ ] Per-frame `rgb.max()>0` retry; inline early-abort armed.
- [ ] Driver `--max-retries 4`; watchdog co-launched; force-kill on the completion marker.
- [ ] Post-render: size>50 KB + hardened color-agnostic audit + motion>0; audit JSON saved.
- [ ] Clip >8 s → `episode_length_s` extended.
- [ ] Remember a video "wave jump on rough" can be a TIME ramp, not terrain conditioning — control for time
      before making any terrain-response claim (qualitative video ≠ quantitative result).

## 5.5  Failure → first thing to try (quick index)
| Symptom | First fix | Ref |
|---|---|---|
| all-black, every file ~6 KB, multi-condition | one condition per subprocess | 2.1 |
| all-black, intermittent, non-baseline wave | retry (`--max-retries 4`); `--reattach-after-warmup` | 2.2 |
| white / blown / strobe, ~50% frames | `--no-reattach-after-warmup` + `--n-warmup 50` | 2.3 |
| robot a tiny smudge / off-frame | chase camera; tune chase-offset/alpha | 2.5 |
| process hangs 5-6 min after "[viz] report ->" | watchdog force-kill on marker | 3.6 |
| robot floating / sunk in offline render | rebuild terrain via to_mujoco_terrain; foot-contact test | 4.3 |
| just need to CHECK it's right | skip Omniverse — render offline (PyVista) | ch.4 |
