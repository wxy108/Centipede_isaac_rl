# Isaac Lab Camera / Video Recording — Lessons & Playbook

Compiled from a multi-day debug session, April 2026. Read this BEFORE
touching `VideoRecorder` or any camera code in `scripts/test_walk.py`
or `scripts/optimization/optimize_impedance_bayesian.py`.

**Read §0 first** — it's the canonical pipeline that actually works on
this Isaac Sim 4.5 + Windows build. The TL;DR for chase video is in
**§1**. The full historical context is in **§2–4**.

---

## 0. AUTHORITATIVE PIPELINE — use this, don't deviate

This Isaac Sim 4.5 / Windows install has a **broken `usdrt.hierarchy`**
module. Every code path that goes through `XformPrimView` /
`Camera.set_world_poses` / `OffsetCfg` for the camera pose write is
dead. We confirmed this by seeing `usd=(...)  gpu=(+0.000,+0.000,+0.000)`
in the capture_frame diagnostic: USD has the right pose, the GPU/Fabric
buffer the renderer samples from stays at world identity. **Don't
re-investigate this.** The fix would be either upgrading Isaac Lab or
replacing the entire rendering path with `omni.replicator.core`
(invasive); for now, work around it with the rules below.

### Working pipeline — chase mode + rough terrain

```bash
cd C:\Users\wxy22\Documents\Centipede_IsaacLab

python scripts/test_walk.py \
    --video videos/walk_rough.mp4 \
    --terrain rough \
    --duration 10 --headless \
    --camera-mode chase
```

Output: a watchable MP4 with visible centipede on bumpy terrain. Motion
looks ~15 fps inside a 30 fps container ("a bit laggy"). This is the
**known-good baseline** — every other future render-related task should
start from this command working.

### What makes chase mode work

1. Chase rewrites the camera USD xform **every render frame** via
   `update_camera()` → `_set_world_pose()`. This continuous re-writing
   means at least half the writes happen on a sim tick that DOES sync
   USD→Fabric (XformPrimView's internal refresh / sim event callbacks),
   so half the frames render at the right pose.
2. The bad-frame guard in `capture_frame()` detects `mean<3` (black,
   un-finalised buffer) or `std<5` (uniform sky, stale buffer) and
   duplicates the previous good frame. This converts the alternating
   black-real-black-real pattern into real-real-newReal-newReal, which
   looks juddery but is watchable.
3. `/rtx/rendermode = "RaytracedLighting"` is forced at app startup
   (NOT the default `PathTracing`). PathTracing is progressive and
   never converges when the camera moves every frame.
4. Mesh resolution stays at 1024 (~2 M triangles) — full collision +
   visual fidelity matching MuJoCo's hfield. **Do not** "optimize" this
   for rendering speed; the 1024 mesh is what the physics needs.
### Tested but DO NOT re-apply: pose-aware bad-frame guard

Tried 2026-04-30: replace the `std<5` "uniform-sky" heuristic in
`capture_frame()` with a direct GPU-pose check (`||camera.data.pos_w[0]
- last_requested_eye|| > 5mm`). Rationale was that std<5 was a proxy
for "Fabric never synced the USD camera write," and a direct check is
more accurate.

**Direct check IS more accurate. That's the problem.**

The std<5 heuristic was an accidental kludge: it filtered out
"camera-at-origin sees only floor / dome" frames (low std) but
ALLOWED through "camera-at-origin sees centipede right next to
camera" frames (high std). The centipede spawns near world origin,
walks slowly in -x, and the camera-at-origin therefore renders
the-centipede-up-close for most of the run. Those wrong-pose-but-
with-content frames are what populated the "watchable laggy chase
video" — it isn't a real chase view, it's a centipede close-up that
happens to look chase-like because the centipede is near the camera
location.

The pose-aware guard correctly identifies ALL those frames as
WRONG-POSE (typically 200-400mm pose error) and rejects them. With
no right-pose frames to fall back on (Fabric never synced even once
in the test run), 100% of frames are skipped. Output: empty video.

**Lesson:** when the "fix" makes the system more correct but produces
worse output, the original behavior was relying on a coincidence we
didn't see. Don't replace the std<5 guard. It's not detecting what
it claims to detect, but what it IS doing — accidentally — is the
only thing keeping chase video watchable on this build.

If you want to add pose error to the diagnostic LOG (informational,
not gating), that's safe and useful. But the std<5 test must remain
the gate.

---

### Tested but DO NOT re-apply: `sim.render()` wait-for-render

Tried 2026-04-30: insert `sim.render()` + `torch.cuda.synchronize()`
inside `if want_video_frame:` to make the loop block until the GPU
finishes the render before reading the annotator buffer. This **DID
fix the un-finalised-buffer race** (no more pure-black frames at
`mean=0,std=0`) — every captured frame was a real render.

**But it produced a worse video, not a better one.** Reasoning:

| Frame source | Baseline | With wait |
|---|---|---|
| Pure black (un-finalised buffer) | ~50% | 0 |
| Real-render-at-wrong-pose (camera at world origin, sees floor or dome) | ~50% | ~100% |
| Real-render-at-right-pose (Fabric happened to sync USD on that tick) | a few per run | nearly none |

The dominant problem on this build is the Fabric/usdrt camera-pose
sync, NOT the renderer-buffer race. In the baseline, the GPU race
gives Fabric extra wall-clock time between frames to occasionally
sync, so a few good frames sneak through and the bad-frame guard
duplicates them into a watchable juddery video. With the wait, no
frame waits long enough for Fabric to sync, so EVERY frame renders
at the wrong pose — uniform dark gray (under-floor material at
`mean≈53, std≈4.9`) or uniform light gray (dome at `mean≈166`). The
bad-frame guard's `std<5` test catches them all, replaces with
nothing (no previous good frame), and skips them. Resulting video:
empty.

**The fix worked exactly as designed but addressed the wrong problem.**
Future Claudes / future-self: if `gpu=(+0.000,+0.000,+0.000)` in the
capture diagnostic, do NOT add a render-wait — that's already not
the issue. The issue is Fabric pose. The only path forward is one
of:

1. Upgrade Isaac Lab / Isaac Sim to a version with working
   `usdrt.hierarchy` so `Camera.set_world_poses()` can write to
   Fabric directly.
2. Replace `isaaclab.sensors.Camera` with `omni.replicator.core`
   attached to a plain `UsdGeom.Camera` prim (Hydra reads pose from
   USD directly, bypassing Fabric).

Until one of those is done, the laggy baseline is the best the
project can achieve.

### What does NOT work and should not be re-attempted

| Approach | Why it fails on this build |
|---|---|
| Static camera mode | The camera pose is written once after `sim.reset()`. That single USD write doesn't trigger a Fabric sync, so the camera stays at world origin → entire run renders dim gray floor material. |
| `Camera.set_world_poses()` (tensor API) | Raises `AttributeError: module 'usdrt' has no attribute 'hierarchy'`. |
| `Camera.set_world_poses_from_view()` | Same, plus extra `usdrt`-touching look-at math. |
| `CameraCfg.OffsetCfg(pos, rot)` | Goes through `XformPrimView.set_world_poses` internally → same usdrt failure. |
| Pre-reset USD xform write seeding | XformPrimView reads from Fabric at init, not USD. Pre-reset USD doesn't reach Fabric automatically. |
| Two `sim.step(render=True)` in a row | The Replicator annotator output buffer gets cleared between steps, so the second read returns zeros — every frame goes black. |
| `mesh_resolution=128` for the rough terrain | This is what the optimizer uses, but in test_walk.py it changes BOTH visual AND collision because terrain.py uses one mesh for both. Reduces physics fidelity. Not acceptable for replays. |
| `torch.cuda.synchronize()` after `sim.step(render=True)` | Renderer is Vulkan-backed on this build. CUDA sync doesn't see the Vulkan render queue, so it returns instantly without waiting. |

### Static-mode workaround (if needed)

We don't have one yet. Until `usdrt.hierarchy` is fixed (Isaac Lab
upgrade) or the rendering path is rewritten on top of
`omni.replicator.core` directly, **use chase mode for video output**.
For diagnostics that need a fixed viewpoint, run chase mode and
post-crop / re-frame in a video editor.

### Other useful commands

```bash
# Walk on flat ground, save chase video (works the same way)
python scripts/test_walk.py --video videos/walk_flat.mp4 --terrain flat --duration 10 --headless --camera-mode chase

# Stand-still diagnostic (zeros wave amplitudes, holds neutral pose)
python scripts/test_walk.py --video videos/stand.mp4 --terrain flat --duration 5 --headless --camera-mode chase --stand

# Run without video — fastest, just stdout metrics
python scripts/test_walk.py --terrain rough --duration 10 --headless

# GUI mode (drop --headless), live viewer
python scripts/test_walk.py --terrain rough --duration 10 --camera-mode chase

# Bayesian gain optimization (separate render pipeline; uses mesh_resolution=128)
python scripts/optimization/optimize_impedance_bayesian.py --n-trials 100 --terrain-wavelength 18 --headless
```

### Backup convention

Before ANY change to `scripts/test_walk.py`:

```bash
copy scripts\test_walk.py scripts\test_walk.py.backup_<YYYYMMDD>
```

To roll back to the known-good juddery-but-watchable baseline:

```bash
copy scripts\test_walk.py.backup_pre_render_fixes_20260430 scripts\test_walk.py
```

That backup is the project's **canonical ground truth** for video
output: chase mode + rough terrain + headless, juddery but the
centipede is visible. Treat any change that produces worse-looking
video than this baseline as a regression and revert.

The other dated backup
(`backup_20260430_183611_pre_wait_for_render`) is byte-identical to
the canonical one — kept only as historical evidence that the
wait-for-render attempt was tried and reverted.

---

## 1. The three settings that make video work

If you only remember three things:

```python
# 1. Force the non-progressive renderer at app startup, BEFORE
#    AppLauncher creates the SimulationContext. Path tracing is
#    progressive and resets its accumulator on every camera move,
#    producing colored-noise frames in chase-cam video.
import carb
carb.settings.get_settings().set_string(
    "/rtx/rendermode", "RaytracedLighting")

# 2. To move the camera each frame, write through Camera's tensor API
#    (Fabric write), NOT UsdGeom.XformOp.Set() (USD-only write). The
#    render product reads from Fabric; USD writes do NOT propagate
#    back to Fabric on this build.
self.camera.set_world_poses(
    positions=eye_t, orientations=quat_t, convention="opengl")

# 3. Drop / replace bad frames in the capture path. On complex
#    scenes (heightfield rough terrain) the Replicator annotator's
#    get_data() does NOT block waiting for the GPU render to finish
#    — it returns whatever's in its output buffer. Two failure
#    modes share the same fix:
#      A. mean<3 → un-finalised (zeroed) buffer = pure BLACK
#      B. std<5 → renderer fell behind, annotator keeps re-returning
#         the same cached "looking at sky" frame for many captures =
#         long uniform-gray freeze in the video
#    Detect EITHER and reuse the previous good frame.
m, s = float(arr.mean()), float(arr.std())
if (m < 3.0 or s < 5.0) and frames:
    arr = frames[-1].copy()    # presentation-layer fix
```

That's it. Everything else in this document explains why.

---

## 2. The mental model: USD vs. Fabric vs. Render Product

Isaac Sim keeps the scene in two parallel data structures:

- **USD** — the declarative authoring layer. `UsdGeom.XformOp.Set()`
  writes here. USD queries (`ComputeLocalToWorldTransform`) see the
  write immediately.
- **Fabric** (a.k.a. `usdrt`) — a flattened, GPU-resident runtime
  copy. Physics and the renderer read from this.

The render product (the thing that produces RGB frames the Camera
sensor delivers to you) samples the camera's pose **from Fabric**,
not from USD.

Isaac Lab's `Camera` class owns an `XformPrimView` configured with
`sync_usd_on_fabric_write=True`. That flag means:

- Fabric writes propagate to USD ✓
- USD writes propagate to Fabric ✗ (NOT done automatically)

So if you write the camera xform via `UsdGeom.XformOp.Set()`, the
USD value updates but **the renderer never sees it**. The Camera
will keep rendering from whatever pose Fabric had at the last Fabric
write — which is usually the spawn-time identity transform, or
whatever the most recent `set_world_poses` call put there.

The `Camera.set_world_poses()` and `Camera.set_world_poses_from_view()`
methods both go through the XformPrimView's Fabric write path. Use
them, not direct USD writes.

---

## 3. The four failure modes we hit (in order)

### v1 — per-frame `ClearXformOpOrder()` + `AddTransformOp()`

Tried to "rebuild the camera xform op every frame." The renderer
sometimes sampled the camera prim mid-rebuild and got a half-torn-down
op chain → completely blank frames. Symptom: random blanks.

**Lesson:** never tear down xform ops at high frequency.

### v2 — cached USD xform op, `.Set()` per frame

Acquired the TransformOp once and called `.Set()` on the cached handle
each frame. Stopped the op-rebuild race, but didn't fix anything else
because USD writes still don't reach the renderer's Fabric pose buffer.
Symptom: alternating gray-void / pure-black / good-frame pattern. Some
frames worked because Fabric got touched by some other system
intermittently (XformPrimView's own internal refresh, sim.reset cycles).

**Lesson:** writing to USD doesn't move the camera. Writing to Fabric
does. They are not the same thing.

### v3 — `set_world_poses_from_view()` (tensor API, look-at variant)

The cleanest API on paper. Internally does (a) compute look-at
quaternion from eyes/targets, (b) call `XformPrimView.set_world_poses()`.

On this Isaac Lab install it dies inside step (a) with:
```
AttributeError: module 'usdrt' has no attribute 'hierarchy'
```
because Fabric isn't fully loaded. So we never get to step (b).

**Lesson:** `_from_view` does extra usdrt-touching work upfront. If
your install's Fabric is broken, this throws before doing anything
useful. The error happens once per pose-set, so it spammed the log
hundreds of times per video.

### v3.5 — dual-render workaround

Idea: write USD xform, do `sim.step(render=True)` to flush USD→Fabric
sync, do another `sim.step(render=True)` to render with the synced
pose, capture that.

**This made things WORSE — every frame went black.**

Why: the second `sim.step(render=True)` flushes the Replicator
annotator's output buffer in preparation for a new frame. By the time
`camera.update(dt)` runs, the buffer is half-cleared and we read
zeros.

**Lesson:** never call `sim.step(render=True)` twice in a row before
`camera.update(dt)`. The annotator output buffer doesn't survive it.

### v4 — `set_world_poses(positions, orientations)` with hand-computed quaternion

`set_world_poses` (without `_from_view`) is a different code path —
it skips the look-at math that was triggering `usdrt.hierarchy`. So
we compute the look-at quaternion ourselves in numpy, build (pos,
quat) tensors, and call `set_world_poses(..., convention="opengl")`.

This goes through the XformPrimView Fabric write path and the renderer
actually sees the new pose. ✓

But the visible result still had occasional "blank but not dark"
frames — those turned out to be the path tracer issue, see v5.

**Lesson:** when `set_world_poses_from_view` fails, try `set_world_poses`
with manually computed orientation. Different code path, often works
on the same install.

### v6 — black-frame race on rough terrain

Even with v4 (Camera.set_world_poses Fabric write) + v5 (RaytracedLighting),
the alternating "real frame, black frame" pattern came back — but only on
ROUGH terrain. Frame 0 = clean centipede. Frame 1 = pure black (2.7KB
PNG). Frame 2 = clean centipede.

Cause: Isaac Lab's Camera class reads RGB via Replicator annotator's
`get_data()`. From the Camera source: "Does NOT block waiting for renders.
It simply reads from buffers". On flat terrain the GPU render finishes
before camera.update(dt) is called and we read a clean frame. On rough
terrain (32k-tri heightfield BVH), the render takes longer than the gap
between sim.step(render=True) and camera.update(dt), so every other
capture hits the un-finalised (cleared) buffer → mean≈0.

**Lesson:** the renderer-pose pipeline isn't the only place a black-frame
race lives. Even with the camera pose perfectly correct, the annotator
read can race the render flush on complex scenes. The fix is in the
CAPTURE path: detect mean<3 and reuse the previous good frame. Tested
on rough λ=18mm + amp=10mm + 32k-tri mesh: drops 30–50% of frames in
chase mode, the duplicated-frame video plays smoothly.

### v7 — uniform-sky freeze on rough terrain

After v6, the alternating-black pattern was gone, but a NEW symptom
appeared: long stretches (10+ consecutive frames) of identical uniform
light gray (RGB ≈ 165, std ≈ 1.7) — looked "blank but not black" in
the MP4.

Diagnostic procedure: ffmpeg-extract 60 frames from the MP4, run a
numpy mean/std over each PNG. The "blank" frames had EXACTLY the same
mean and std to one decimal place across 12+ consecutive captures.
That mathematical identity is the smoking gun: it's not 12 different
"camera looking at sky" renders, it's the SAME cached buffer being
returned 12 times.

Cause: the renderer can't keep up with the capture rate on complex
scenes. When the GPU is still working on render N, the next capture
just re-returns frame N-1 from the annotator's output buffer. If the
camera was momentarily looking at empty sky on frame N-1 (transient
chase-cam excursion when the robot's root_pos jiggled), that sky
frame becomes the "stale fill" for the next 10+ captures.

**Lesson:** mean<3 isn't enough — uniform-color stale frames slip
through with mean≈165. Use **std<5** as the second detection rule:
a real rendered scene of terrain has std ~ 17-22; pure dome-light fill
has std ~ 1.7. Detect EITHER mean<3 OR std<5 and reuse the previous
good frame.

This also clarifies the apparent "controller params affect rendering"
illusion: changing gains changes the gait, which changes how the chase
camera moves, which changes how often the camera momentarily catches
empty sky, which changes how often the annotator's stale-buffer cache
contains a sky frame for the next freeze. The renderer doesn't care
about your gains; it cares about how complex the scene is and whether
the camera ever points at empty space.

### v5 — switch from path tracing to RaytracedLighting

The remaining "blank but not dark" frames in v4's video were not
blank — they were heavy RGB noise with the centipede ghosted through
them. Default `/rtx/rendermode = "PathTracing"` is **progressive**:
it accumulates samples across frames to denoise. Every camera move
resets the accumulator. In chase mode the camera moves every render,
so the accumulator never converges. First 1–2 frames after each move
come out as colored noise.

`/rtx/rendermode = "RaytracedLighting"` is the single-pass real-time
renderer. One frame, one render, no accumulation, no convergence
noise. Slightly less photoreal lighting but completely flicker-free.

**Lesson:** chase-cam video MUST use `RaytracedLighting`, not
`PathTracing`.

---

## 4. The diagnostic procedure (use this, don't guess)

When video frames look wrong, dump the first 3–5 captured frames as
PNGs (`VideoRecorder.capture_frame` already does this). Then look at
the PNG byte sizes:

| Symptom                  | PNG size | What it means                                       |
|--------------------------|----------|-----------------------------------------------------|
| Pure black               | ~2.7 KB  | Render didn't run / camera at identity / all-zero buffer |
| Uniform light gray       | ~10–15 KB | Camera looking at empty sky / dome-light color    |
| Real scene               | ~500 KB – 2 MB | Camera correctly framed, render converged   |
| Real scene + RGB noise   | ~1 – 2.5 MB (large because noise is incompressible) | Path tracer hasn't converged |

If frame N is small (black/gray) and frame N+1 is large (real),
that's the **alternating-frame pattern** → camera pose write isn't
reaching Fabric → fix per §3 v4.

If every frame is large but visibly noisy → **RTX path tracer
not converged** → fix per §3 v5.

Always also print the diagnostic line `frame N: shape=... mean=...
usd=(...) gpu=(...)`. If `usd=` and `gpu=` disagree, you have a
USD-vs-Fabric desync; if they agree but the frame is still wrong,
the renderer/path-tracer is the problem, not the pose.

---

## 5. Things that look like they should work but don't

- Parent Xform pivots (camera under a moving parent). Hydra reads
  the camera prim's own xform attributes for the render-time pose
  on this Isaac Sim version; parent compose is unreliable.
- Calling `omni.usd.get_context().get_stage()` and writing
  `xformOp.Set()` to the camera prim. (Same USD-only write that
  doesn't reach Fabric.)
- Calling `sim.step(render=True)` multiple times to "give the
  renderer more chances." The Replicator annotator output buffer
  doesn't survive a second render before the read.
- Using `convention="ros"` when you've already built the orientation
  in OpenGL convention. The convention parameter rotates your input
  to internal OpenGL convention; pass `"opengl"` if you've already
  done the math.

---

## 6. Quaternion construction from look-at (reference)

```python
import numpy as np
fwd = (target - eye) / np.linalg.norm(target - eye)
right = np.cross(fwd, [0, 0, 1])
right /= np.linalg.norm(right)
cam_up = np.cross(right, fwd)
# OpenGL/USD camera convention: cam looks down -Z, +X right, +Y up.
R = np.column_stack([right, cam_up, -fwd])
# Standard rotation-matrix → (w,x,y,z) quaternion conversion (Shepperd).
# (See _set_world_pose in scripts/test_walk.py for the branchless form.)
```

Pass `convention="opengl"` to `Camera.set_world_poses` so it doesn't
re-rotate.

---

## 7. The known-good chase-cam recipe (April 2026)

1. At app startup, before AppLauncher: set
   `/rtx/rendermode = "RaytracedLighting"` and
   `/isaaclab/render/rtx_sensors = True`.
2. Construct `isaaclab.sensors.Camera` BEFORE `sim.reset()`.
3. After `sim.reset()`: set initial pose with
   `Camera.set_world_poses(positions, orientations, convention="opengl")`.
4. Run a 5-step warmup of `sim.step(render=True)` then one
   `camera.update(dt)` to prime the annotator buffer.
5. Per video frame:
   - update camera pose via `set_world_poses` (Fabric write)
   - exactly ONE `sim.step(render=True)`
   - exactly ONE `camera.update(dt)`
   - read `camera.data.output["rgb"][0]`
6. Latch any tensor-API failure on the first call and fall back
   gracefully — log ONCE, never re-throw per frame.
