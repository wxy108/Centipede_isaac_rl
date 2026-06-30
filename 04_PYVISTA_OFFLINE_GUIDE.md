# 4. Third-party Offline Renderer (PyVista) — usage guide

Why we added it, the exact export→render→audit pipeline, the two terrain-reconstruction bugs and their fixes,
the fidelity check vs Omniverse, and the measured time saved. Source: `OFFLINE_RENDERER_PIPELINE.md` +
`scripts/sanity/offline_render_from_log_2026-06-22.py`.

## 4.1  Why offline
Omniverse's RTX raytracer is the only thing that produces the black/white-frame race (ch. 2). It is also slow
(GPU raytrace per frame + 5–6 min teardown hang). For **internal verification** we don't need photoreal RTX —
we need to know the gait/terrain/pose is right. **PyVista** (VTK, offscreen, CPU) renders the SAME geometry,
**deterministically**, with **no init race**, much faster. So: verify offline, export the deliverable in
Omniverse only once the offline check passes.

## 4.2  The pipeline (3 stages)
**Stage 1 — DUMP (one GPU pass, the only GPU step).** Run the normal Isaac Lab rollout with `--dump-raw-alllink`.
It writes an npz of per-frame WORLD poses for **every link** (not just the root):
- `body_pos_w` (T, N, nb, 3), `body_quat_w` (T, N, nb, 4 wxyz), `dt_pol`, plus terrain params (λ, amp,
  `force_wave_abs`, `spawn_block_id`). One dump serves unlimited re-renders (any camera/lighting) with **no GPU**.

**Stage 2 — RENDER (CPU, PyVista).** `offline_render_from_log_2026-06-22.py`:
1. Load per-link OBJ meshes once (`trimesh`/PyVista), one actor per link.
2. For each frame, set each actor's transform from `(body_pos_w, body_quat_w)` — quaternion is **wxyz** (Isaac),
   convert for VTK. **The poses are absolute world poses → NO forward-kinematics, no joint math; just place links.**
3. Reconstruct the terrain mesh once (4.3) and add it as a static actor.
4. Offscreen `plotter.screenshot()` per frame → encode with `imageio.v2` libx264 quality 8 (cv2 fallback).
5. Chase camera follows the head; sky background L≈213 (non-saturating, so pure white only appears on failure).

**Stage 3 — AUDIT (CPU).** The extended audit (3.4): black/white/blank + partial-white + strobe + outlier +
red-robot-presence + motion, with a self-test that injects each mode. Ships a per-clip audit JSON.

## 4.3  Terrain reconstruction — the two bugs (both fixed)
The dump records the robot's links but not the terrain mesh; we rebuild it from the heightfield params. Two bugs:
- **Bug 1 — wrong height scale (robot floating / sunk):** the heightfield Z must use the SAME amplitude/units as
  the sim's `to_mujoco_terrain`. First reconstruction floated the robot. **Fix:** rebuild via the project's
  `to_mujoco_terrain` path so terrain Z matches the sim exactly.
- **Bug 2 — wrong XY origin/orientation (robot beside the strip):** block origin and axis order must match the
  arena layout. **Fix:** take origin/orientation from the same arena config the rollout used.
- **Verification (foot-contact test):** after both fixes the lowest foot sits **0.43 mm** above terrain vs
  **16.96 mm** before — i.e. visually in contact, geometry-exact. This is the acceptance test for terrain
  reconstruction: re-run it whenever the arena changes.

## 4.4  Fidelity vs Omniverse
- **Geometry/pose:** identical — same world poses, same meshes. The foot-contact test (0.43 mm) confirms it.
- **Appearance:** PyVista is rasterized (flat-ish lighting, simple shadows); Omniverse RTX is photoreal. For
  verification this is irrelevant; for a polished talk clip, export in Omniverse.
- **Determinism:** PyVista is fully deterministic (no RTX race) → reproducible frames, reliable audits.
- We ship a **side-by-side** (Omniverse left / PyVista right) to show the gait/terrain match — that clip is one
  of the deliverable comparison videos.

## 4.5  Time saved (measured A/B, 192 frames)
| Pipeline | Time (192 frames) |
|---|---|
| Omniverse RTX (capture only) | **264 s** |
| PyVista offline (render only) | **40.6 s** |
| **Speedup** | **≈ 6.5×** |
Excludes the 5–6 min Omniverse teardown hang and any race-retries — with those the real-world gap is larger.
The Stage-1 dump is a one-time GPU cost; every subsequent re-render (new camera, lighting, audit threshold) is
**free of GPU and of the race.**

## 4.6  When to use which
| Need | Renderer |
|---|---|
| Check a gait / terrain / pose is correct | **PyVista offline** (default) |
| Iterate camera / lighting / many designs | **PyVista offline** (dump once, re-render free) |
| Final photoreal clip for a talk | **Omniverse**, AFTER offline verification passes |
| Reproduce / debug the RTX race itself | Omniverse (quarantined diagnostic) |
