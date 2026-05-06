#!/usr/bin/env python3
"""
view_arena.py — render an overview of the multi-block arena
=============================================================

Standalone viewer for `scripts/optimization/arena.py`. Builds the arena
in Isaac Sim, places a HIGH OVERHEAD CAMERA pointing at the centre of
the arena, and saves a short MP4 + a PNG snapshot.

This is purely for verifying the terrain looks right before the next
step (RL on multi-block arenas). No centipede is spawned in this
script — that comes next.

Usage
-----
    # Default 4 m × 4 m arena, 4×4 blocks, λ ∈ [10, 100] mm:
    python scripts/view_arena.py --headless --duration 5 \\
        --video videos/arena_overview.mp4

    # Bigger arena with more blocks:
    python scripts/view_arena.py --headless --duration 5 \\
        --arena-size 6.0 --n-blocks 5 --video videos/arena_5x5.mp4

    # Just the matplotlib top-down (no Isaac Sim needed):
    python scripts/optimization/arena.py --out outputs/arena.png

Render path
-----------
Per NOTES_RENDERING §0, true static cameras don't work on this Isaac
Lab build (USD writes don't propagate to Fabric). We use the same
"chase camera locked at a fixed pose" trick as render_top_trials.py:
the chase loop re-writes the camera USD every frame, which sometimes
hits a sim tick where Fabric syncs USD→GPU, so a fraction of frames
render at the correct overview pose. The bad-frame guard duplicates
the good frames and skips black/empty captures, producing a watchable
overview clip.
"""
import argparse
import math
import os
import sys
import time

# ── Resolve project root + force --enable_cameras / --headless ───────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "scripts", "optimization"))

# Inject before parse_args so AppLauncher picks them up via the normal CLI path
if "--enable_cameras" not in sys.argv:
    sys.argv.append("--enable_cameras")

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--arena-size",     type=float, default=4.0,
                    help="Arena side length in metres (default 4.0).")
parser.add_argument("--n-blocks",       type=int,   default=4,
                    help="Block grid is N×N (default 4 → 16 blocks).")
parser.add_argument("--wavelength-min", type=float, default=10.0,
                    help="Min wavelength in mm.")
parser.add_argument("--wavelength-max", type=float, default=100.0,
                    help="Max wavelength in mm.")
parser.add_argument("--amplitude-min",  type=float, default=0.005,
                    help="Min amplitude in m (default 5 mm).")
parser.add_argument("--amplitude-max",  type=float, default=0.015,
                    help="Max amplitude in m (default 15 mm).")
parser.add_argument("--image-size",     type=int,   default=2048,
                    help="Heightmap pixel resolution (default 2048).")
parser.add_argument("--mesh-resolution",type=int,   default=1024,
                    help="Collision/visual mesh side count (default 1024). "
                         "Higher → better terrain detail but heavier PhysX "
                         "cooking + render. 1024 over 4 m = 4 mm cells, "
                         "which Nyquist-resolves the 10 mm wavelength.")
parser.add_argument("--seed",           type=int,   default=42)
parser.add_argument("--video",          default="videos/arena_overview.mp4",
                    help="Output MP4 path. Set empty string to skip video.")
parser.add_argument("--video-fps",      type=int,   default=30)
parser.add_argument("--duration",       type=float, default=5.0,
                    help="Seconds of video to render (just spins the sim "
                         "with a fixed camera; longer = more chances for "
                         "Fabric to sync at least one frame at the right "
                         "pose).")
parser.add_argument("--cam-eye",        nargs=3, type=float,
                    default=(1.0, -2.5, 4.0),
                    help="Camera eye position [x y z] in metres. Default "
                         "(1, -2.5, 4) gives a bird's-eye SE-looking view "
                         "of a 4 m arena centred on origin.")
parser.add_argument("--cam-target",     nargs=3, type=float,
                    default=(0.0, 0.0, 0.0),
                    help="Camera target [x y z]. Default (0, 0, 0) = arena "
                         "centre.")
parser.add_argument("--png-snapshot",   default="videos/arena_overview.png",
                    help="Also save the FIRST captured frame as a PNG for "
                         "easy inspection. Empty string to skip.")

from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

# Render-pipeline settings — see NOTES_RENDERING §1
import carb
_settings = carb.settings.get_settings()
_settings.set_bool("/isaaclab/render/rtx_sensors", True)
_settings.set_string("/rtx/rendermode", "RaytracedLighting")
print("[view_arena] /rtx/rendermode = RaytracedLighting", flush=True)

# Isaac Lab imports (must come AFTER AppLauncher)
import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sensors import Camera, CameraCfg

from arena import (
    generate_arena_heightmap, build_arena_mesh,
    save_topdown_png, save_3d_view_png,
)


CAM_PATH = "/World/OverviewCamera"


# ─────────────────────────────────────────────────────────────────────────────
# Look-at quaternion + pose write helpers (mirrors render_top_trials.py)
# ─────────────────────────────────────────────────────────────────────────────
def _lookat_quat(eye, target):
    """Return (w, x, y, z) quaternion for a camera at `eye` looking at
    `target`, world-up = +Z, OpenGL convention."""
    fwd = np.array(target, float) - np.array(eye, float)
    fwd /= max(np.linalg.norm(fwd), 1e-12)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= max(np.linalg.norm(right), 1e-12)
    cam_up = np.cross(right, fwd)
    R = np.column_stack([right, cam_up, -fwd])
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s,
                (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def _set_camera_pose(cam, eye, target, _state):
    """Write camera pose via the tensor API (Fabric write); on
    AttributeError (usdrt.hierarchy missing) fall back to a USD
    TransformOp write. Latches the failure so we don't keep retrying.
    Mirrors render_top_trials.py:set_camera_pose."""
    qw, qx, qy, qz = _lookat_quat(eye, target)
    if _state.get("tensor_works") is not False:
        try:
            try:
                cam_dev = cam.device
            except AttributeError:
                cam_dev = getattr(cam, "_device", "cuda")
            pos_t  = torch.tensor([list(eye)], dtype=torch.float32, device=cam_dev)
            quat_t = torch.tensor([[qw, qx, qy, qz]], dtype=torch.float32,
                                  device=cam_dev)
            cam.set_world_poses(positions=pos_t, orientations=quat_t,
                                convention="opengl")
            if _state.get("tensor_works") is None:
                _state["tensor_works"] = True
                print("[view_arena] Camera.set_world_poses works (Fabric).",
                      flush=True)
            return
        except Exception as e:
            _state["tensor_works"] = False
            print(f"[view_arena] tensor pose API unavailable "
                  f"({type(e).__name__}: {e}) — falling back to USD xform "
                  f"write. (USD writes intermittently propagate to Fabric "
                  f"on this build — see NOTES_RENDERING §0.)",
                  flush=True)
    # USD fallback
    import omni.usd
    from pxr import UsdGeom, Gf
    stage = omni.usd.get_context().get_stage()
    cam_prim = stage.GetPrimAtPath(CAM_PATH)
    eye_v = Gf.Vec3d(*eye); tgt_v = Gf.Vec3d(*target); up = Gf.Vec3d(0, 0, 1)
    fwd = (tgt_v - eye_v).GetNormalized()
    right = (fwd ^ up).GetNormalized()
    cam_up = (right ^ fwd).GetNormalized()
    m = Gf.Matrix4d()
    m.SetRow(0, Gf.Vec4d(right[0], right[1], right[2], 0))
    m.SetRow(1, Gf.Vec4d(cam_up[0], cam_up[1], cam_up[2], 0))
    m.SetRow(2, Gf.Vec4d(-fwd[0], -fwd[1], -fwd[2], 0))
    m.SetRow(3, Gf.Vec4d(eye_v[0], eye_v[1], eye_v[2], 1))
    if _state.get("xform_op") is None:
        xf = UsdGeom.Xformable(cam_prim)
        existing = xf.GetOrderedXformOps()
        op = next((o for o in existing
                   if o.GetOpType() == UsdGeom.XformOp.TypeTransform), None)
        if op is None:
            xf.ClearXformOpOrder()
            op = xf.AddTransformOp()
        _state["xform_op"] = op
    _state["xform_op"].Set(m)


# ─────────────────────────────────────────────────────────────────────────────
# Scene
# ─────────────────────────────────────────────────────────────────────────────
def build_scene(sim, args):
    from pxr import UsdGeom, UsdLux, UsdPhysics, Gf
    stage = sim.stage

    # Lighting — bright dome + sun for nice perspective shot
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(400.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun.GetIntensityAttr().Set(1200.0)
    sun.GetAngleAttr().Set(0.53)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50, 35, 0))

    # ── Reference markers ──
    # Place a few brightly-coloured marker spheres at world origin and the
    # arena corners so the bad-frame guard has SOMETHING with high pixel
    # variance to lock onto when (rare) Fabric-synced frames render at the
    # correct pose. Without markers, the camera-at-origin captures uniform
    # dome light (mean≈165, std≈1.7), the std<5 guard rejects every
    # candidate frame, and the video stays empty.
    half = args.arena_size / 2.0
    marker_z = 0.10  # 10 cm above arena floor — well above any peak
    marker_r = 0.05  # 5 cm radius
    for name, pos in [
        ("CentreMarker",   ( 0.0,    0.0,   marker_z)),
        ("CornerMarkerNE", ( half-0.3,  half-0.3, marker_z)),
        ("CornerMarkerNW", (-half+0.3,  half-0.3, marker_z)),
        ("CornerMarkerSE", ( half-0.3, -half+0.3, marker_z)),
        ("CornerMarkerSW", (-half+0.3, -half+0.3, marker_z)),
    ]:
        path = f"/World/Markers/{name}"
        sph = UsdGeom.Sphere.Define(stage, path)
        sph.GetRadiusAttr().Set(marker_r)
        # Bright primary color so frames with markers visible have HIGH std
        # (passes the std<5 bad-frame guard; matches the kludge that test_walk
        # relies on per NOTES_RENDERING §0).
        sph.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.2, 0.1)])
        x = UsdGeom.Xformable(sph)
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))

    # Generate the arena heightmap
    print(f"[view_arena] generating {args.n_blocks}×{args.n_blocks} block "
          f"arena ({args.arena_size:.1f} m × {args.arena_size:.1f} m) ...",
          flush=True)
    h, blocks = generate_arena_heightmap(
        arena_size_m=args.arena_size,
        n_blocks=(args.n_blocks, args.n_blocks),
        wavelength_range_mm=(args.wavelength_min, args.wavelength_max),
        amplitude_range_m=(args.amplitude_min, args.amplitude_max),
        image_size=args.image_size,
        seed=args.seed,
    )

    # Bake into USD mesh + PhysX collision
    UsdGeom.Xform.Define(stage, "/World/Arena")
    print(f"[view_arena] baking mesh ({args.mesh_resolution}×"
          f"{args.mesh_resolution} cells = "
          f"~{(args.mesh_resolution-1)**2 * 2 // 1_000_000} M tris) ...",
          flush=True)
    build_arena_mesh(stage=stage,
                     prim_path="/World/Arena/Mesh",
                     heightmap_m=h,
                     arena_size_m=args.arena_size,
                     mesh_resolution=args.mesh_resolution)
    print("[view_arena] mesh built.", flush=True)

    # Save matplotlib previews alongside (instant, doesn't depend on the
    # broken Isaac Sim render pipeline). These are the AUTHORITATIVE
    # arena visualisation — the Isaac Sim chase-trick render below is
    # best-effort.
    if args.png_snapshot:
        topdown_path = args.png_snapshot.replace(".png", "_topdown.png")
        view3d_path  = args.png_snapshot.replace(".png", "_3d.png")
        try:
            save_topdown_png(h, blocks, args.arena_size, topdown_path,
                             n_blocks=(args.n_blocks, args.n_blocks))
        except Exception as e:
            print(f"[view_arena] top-down PNG failed: {e}", flush=True)
        try:
            save_3d_view_png(h, blocks, args.arena_size, view3d_path,
                             n_blocks=(args.n_blocks, args.n_blocks))
        except Exception as e:
            print(f"[view_arena] 3D PNG failed: {e}", flush=True)

    return blocks


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    sim_cfg = SimulationCfg(dt=0.004)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=tuple(args.cam_eye), target=tuple(args.cam_target))

    blocks = build_scene(sim, args)

    # ── Camera (must be created BEFORE sim.reset()) ──
    cam_cfg = CameraCfg(
        prim_path=CAM_PATH,
        update_period=0.0,
        height=720, width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.05, 100.0)),
    )
    cam = Camera(cfg=cam_cfg)

    sim.reset()

    # ── Render warmup (RTX shader compile) ──
    print("[view_arena] render warmup (first sim.step may take 10-30s) ...",
          flush=True)
    t0 = time.time()
    for _ in range(5):
        sim.step(render=True)
    cam.update(sim_cfg.dt)
    print(f"[view_arena] warmup done in {time.time()-t0:.1f}s", flush=True)

    # ── Set the overview camera pose. Using chase-style USD writes per
    # frame is what makes this build's broken Fabric occasionally sync,
    # see NOTES_RENDERING §0 / render_top_trials.py. ──
    cam_state = {}
    _set_camera_pose(cam, tuple(args.cam_eye), tuple(args.cam_target), cam_state)

    if not args.video:
        # No video requested — render a single frame for the snapshot then exit.
        sim.step(render=True)
        cam.update(sim_cfg.dt)
        rgb = cam.data.output.get("rgb", None)
        if rgb is not None and args.png_snapshot:
            arr = rgb[0].detach().cpu().numpy()
            if arr.ndim == 3 and arr.shape[2] == 4:
                arr = arr[:, :, :3]
            arr = arr.astype(np.uint8)
            from PIL import Image
            os.makedirs(os.path.dirname(os.path.abspath(args.png_snapshot)) or ".",
                        exist_ok=True)
            Image.fromarray(arr).save(args.png_snapshot)
            print(f"[view_arena] saved single-frame snapshot → "
                  f"{args.png_snapshot}", flush=True)
        sim_app.close()
        return

    # ── Capture frames for video ──
    n_steps  = int(args.duration / sim_cfg.dt)
    video_dt = 1.0 / args.video_fps
    last_t   = -np.inf
    frames   = []
    n_replaced = 0
    first_good_saved = False

    print(f"[view_arena] capturing {args.duration}s @ {args.video_fps} fps ...",
          flush=True)
    capture_t0 = time.time()
    for step in range(n_steps):
        t = step * sim_cfg.dt
        # Re-write camera pose every render frame so Fabric gets sync chances.
        want_frame = (t - last_t) >= video_dt - 1e-10
        if want_frame:
            _set_camera_pose(cam, tuple(args.cam_eye),
                             tuple(args.cam_target), cam_state)
        sim.step(render=want_frame)
        if want_frame:
            cam.update(sim_cfg.dt)
            rgb = cam.data.output.get("rgb", None)
            if rgb is not None:
                arr = rgb[0].detach().cpu().numpy()
                if arr.ndim == 3 and arr.shape[2] == 4:
                    arr = arr[:, :, :3]
                arr = arr.astype(np.uint8)
                # Bad-frame guard — IMPORTANT for arena-only scenes.
                # Per NOTES_RENDERING §0: when the camera is stuck at world
                # origin (Fabric bug), captures have either mean=0 (black,
                # un-finalised) OR mean≈165 std≈1.7 (uniform dome) OR
                # mean≈53 std≈4.9 (under-floor). A scene with no centipede
                # near origin has NO high-std content at the broken pose,
                # so we MUST filter the uniform-dome captures with std<5
                # — otherwise they fill the video with uniform gray.
                # The bright marker spheres added in build_scene give us
                # at least one direction (target) where a synced frame
                # would have high std and pass through.
                mean_v = float(arr.mean())
                std_v  = float(arr.std())
                is_bad = (mean_v < 3.0) or (std_v < 5.0)
                if is_bad and frames:
                    arr = frames[-1].copy()
                    n_replaced += 1
                elif is_bad:
                    # First-frame bad with no fallback. Skip — wait for
                    # a Fabric sync to land us a real frame to lock on.
                    last_t = t
                    continue
                frames.append(arr)
                # Save the FIRST passing frame as the PNG snapshot
                if (not first_good_saved
                        and not is_bad
                        and args.png_snapshot):
                    try:
                        from PIL import Image
                        os.makedirs(os.path.dirname(
                            os.path.abspath(args.png_snapshot)) or ".",
                            exist_ok=True)
                        Image.fromarray(arr).save(args.png_snapshot)
                        first_good_saved = True
                        print(f"[view_arena] saved first-good snapshot → "
                              f"{args.png_snapshot}", flush=True)
                    except Exception as e:
                        print(f"[view_arena] PNG save failed: {e}",
                              flush=True)
            last_t = t

    # ── Write MP4 ──
    if not frames:
        print("[view_arena] no frames captured — video skipped", flush=True)
    else:
        try:
            import cv2
            h_, w_ = frames[0].shape[:2]
            os.makedirs(os.path.dirname(os.path.abspath(args.video)) or ".",
                        exist_ok=True)
            writer = cv2.VideoWriter(
                args.video, cv2.VideoWriter_fourcc(*'mp4v'),
                args.video_fps, (w_, h_))
            for fr in frames:
                writer.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
            writer.release()
            print(f"[view_arena] wrote {args.video}  "
                  f"({len(frames)} frames, {n_replaced} replaced, "
                  f"capture took {time.time()-capture_t0:.1f}s)",
                  flush=True)
        except ImportError:
            try:
                import imageio
                imageio.mimwrite(args.video, frames, fps=args.video_fps)
                print(f"[view_arena] wrote {args.video} (imageio)", flush=True)
            except ImportError:
                print("[view_arena] no cv2/imageio — video skipped",
                      flush=True)

    sim_app.close()


if __name__ == "__main__":
    main()
