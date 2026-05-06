#!/usr/bin/env python3
"""
view_arena_replicator.py — render the arena via omni.replicator.core directly
==============================================================================

This is the documented "real fix" from NOTES_RENDERING §0 for this Isaac Lab
build's broken `usdrt.hierarchy` Fabric write path.

Why this exists
---------------
Every prior attempt to render the arena from a high overhead camera went
through `isaaclab.sensors.Camera` → `XformPrimView` → `usdrt.hierarchy`,
which raises AttributeError on this build, so camera pose USD writes
don't reach Fabric, the GPU camera buffer stays at world identity, and
the render comes out as uniform grey dome light.

This script avoids that entire chain. Instead:

  1. Create a plain `UsdGeom.Camera` prim manually (no Camera class).
  2. Set its xform via USD ops directly. Hydra reads the camera prim's
     own xform attribute at render time — bypasses Fabric.
  3. Attach an `omni.replicator.core` render product to the camera.
  4. Step the sim, fetch RGB from the annotator, save PNG.

Per NOTES_RENDERING §0, this is one of the two known fixes (the other
being upgrading Isaac Lab to a version with working `usdrt.hierarchy`).

Output
------
A single PNG snapshot of the arena from the requested overhead pose.
No video — for video you'd loop the capture, but for "verify the arena
looks right" a single high-res frame is what we actually need.

Usage
-----
    python scripts/view_arena_replicator.py --headless ^
        --arena-size 6.0 --n-blocks 5 ^
        --out videos/arena_replicator.png

    # Top-down (camera straight up):
    python scripts/view_arena_replicator.py --headless ^
        --cam-eye 0 0 8 --cam-target 0 0 0 ^
        --out videos/arena_topdown_isaac.png
"""
import argparse
import os
import sys
import math
import time

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "scripts", "optimization"))

# Force --enable_cameras and --headless so AppLauncher initialises rendering
if "--enable_cameras" not in sys.argv:
    sys.argv.append("--enable_cameras")

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--arena-size",      type=float, default=6.0)
parser.add_argument("--n-blocks",        type=int,   default=5)
parser.add_argument("--wavelength-min",  type=float, default=10.0)
parser.add_argument("--wavelength-max",  type=float, default=100.0)
parser.add_argument("--amplitude-min",   type=float, default=0.005)
parser.add_argument("--amplitude-max",   type=float, default=0.015)
parser.add_argument("--image-size",      type=int,   default=2048,
                    help="Heightmap pixel resolution.")
parser.add_argument("--mesh-resolution", type=int,   default=1024,
                    help="Collision/visual mesh side count.")
parser.add_argument("--seed",            type=int,   default=42)
parser.add_argument("--cam-eye",         nargs=3, type=float,
                    default=(3.0, -7.0, 2.5),
                    help="Camera eye position [x y z] in metres. Default "
                         "(3, -7, 2.5) is a 'more level' / 平视 view at "
                         "~15° below horizontal — terrain bumps look less "
                         "flat than the prior bird's-eye view. Use "
                         "(0, 0, 8) for straight top-down or "
                         "(3, -8, 8) for the previous bird's-eye angle.")
parser.add_argument("--cam-target",      nargs=3, type=float,
                    default=(0.0, 0.0, 0.3),
                    help="Camera target [x y z]. Default (0, 0, 0.3) is "
                         "slightly elevated above the arena centre so the "
                         "look direction stays nearly horizontal.")
parser.add_argument("--width",           type=int,   default=1920)
parser.add_argument("--height",          type=int,   default=1080)
parser.add_argument("--focal-length",    type=float, default=18.0,
                    help="Camera focal length (mm). Default 18 = wide "
                         "FoV (~57°). Lower → wider; higher → narrower.")
parser.add_argument("--n-warmup",        type=int,   default=20,
                    help="Warmup sim.step calls before capture (gives RTX "
                         "shaders time to compile and the renderer time to "
                         "converge on the static scene).")
parser.add_argument("--out",             default="videos/arena_replicator.png",
                    help="Output PNG path for the still snapshot. The "
                         "PNG is always written; pass --video to also "
                         "capture an MP4 alongside.")
# ── Video capture (optional) ──
parser.add_argument("--video",           default=None,
                    help="If set, also write an MP4 of an orbiting camera "
                         "to this path. The orbit keeps the camera at the "
                         "same height and distance from the arena centre, "
                         "rotating 360° over --duration. Pass empty / unset "
                         "to skip video and only save the PNG.")
parser.add_argument("--duration",        type=float, default=20.0,
                    help="Video duration in seconds (default 20 s — "
                         "gives a slow 18°/s orbit comfortable for "
                         "inspecting the arena, and leaves time for "
                         "any centipedes in the scene to walk a few cm).")
parser.add_argument("--video-fps",       type=int,   default=30)
parser.add_argument("--orbit-degrees",   type=float, default=360.0,
                    help="Orbit sweep in degrees over the duration "
                         "(default 360 = full rotation). Set 0 for a "
                         "static-camera video (same frame repeated).")

from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

# Render-pipeline settings (NOTES_RENDERING §1)
import carb
_settings = carb.settings.get_settings()
_settings.set_bool("/isaaclab/render/rtx_sensors", True)
_settings.set_string("/rtx/rendermode", "RaytracedLighting")
print("[replicator] /rtx/rendermode = RaytracedLighting", flush=True)

import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext

# Replicator + USD imports
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, Gf

from arena import (generate_arena_heightmap, build_arena_mesh,
                   save_topdown_png, save_3d_view_png)


CAM_PATH = "/World/ReplicatorCamera"


# ─────────────────────────────────────────────────────────────────────────────
# Look-at matrix (USD camera convention)
# ─────────────────────────────────────────────────────────────────────────────
def _lookat_matrix(eye, target):
    """Return a Gf.Matrix4d for USD camera at `eye` looking at `target`,
    world-up = +Z. USD camera convention: camera looks down its local -Z,
    +Y is up, +X is right.

    Gf.Matrix4d uses ROW-VECTOR convention (v_world = v_local @ M), so
    the i-th ROW of M is the world-frame coordinates of the i-th
    camera-local basis vector:
        row 0 = world(+X) = right
        row 1 = world(+Y) = cam_up
        row 2 = world(+Z) = −fwd     (camera looks down −Z)
        row 3 = (eye, 1)              (translation)
    """
    eye = np.array(eye, dtype=float)
    tgt = np.array(target, dtype=float)
    fwd = tgt - eye
    fwd /= max(np.linalg.norm(fwd), 1e-12)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(right) < 1e-9:
        # Camera looking straight up/down — pick an arbitrary right
        right = np.array([1.0, 0.0, 0.0])
    right /= max(np.linalg.norm(right), 1e-12)
    cam_up = np.cross(right, fwd)
    neg_fwd = -fwd

    m = Gf.Matrix4d(1.0)   # start as identity
    m.SetRow(0, Gf.Vec4d(float(right[0]),
                         float(right[1]),
                         float(right[2]),
                         0.0))
    m.SetRow(1, Gf.Vec4d(float(cam_up[0]),
                         float(cam_up[1]),
                         float(cam_up[2]),
                         0.0))
    m.SetRow(2, Gf.Vec4d(float(neg_fwd[0]),
                         float(neg_fwd[1]),
                         float(neg_fwd[2]),
                         0.0))
    m.SetRow(3, Gf.Vec4d(float(eye[0]),
                         float(eye[1]),
                         float(eye[2]),
                         1.0))
    return m


# ─────────────────────────────────────────────────────────────────────────────
# Scene
# ─────────────────────────────────────────────────────────────────────────────
def build_scene(sim, args):
    stage = sim.stage

    # Lighting
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(400.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    sun.GetIntensityAttr().Set(1200.0)
    sun.GetAngleAttr().Set(0.53)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-50.0, 35.0, 0.0))

    # Arena
    print(f"[replicator] generating {args.n_blocks}×{args.n_blocks} arena "
          f"({args.arena_size:.1f} m) ...", flush=True)
    h, blocks = generate_arena_heightmap(
        arena_size_m=args.arena_size,
        n_blocks=(args.n_blocks, args.n_blocks),
        wavelength_range_mm=(args.wavelength_min, args.wavelength_max),
        amplitude_range_m=(args.amplitude_min, args.amplitude_max),
        image_size=args.image_size,
        seed=args.seed,
    )

    UsdGeom.Xform.Define(stage, "/World/Arena")
    print(f"[replicator] baking mesh {args.mesh_resolution}×"
          f"{args.mesh_resolution} (~{(args.mesh_resolution-1)**2 * 2 // 1_000_000} M tris) ...",
          flush=True)
    build_arena_mesh(stage=stage,
                     prim_path="/World/Arena/Mesh",
                     heightmap_m=h,
                     arena_size_m=args.arena_size,
                     mesh_resolution=args.mesh_resolution)

    # Save the matplotlib previews alongside (always-good fallback)
    out_dir = os.path.dirname(os.path.abspath(args.out)) or "."
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(args.out))[0]
    try:
        save_topdown_png(h, blocks, args.arena_size,
                         os.path.join(out_dir, f"{base}_topdown.png"),
                         n_blocks=(args.n_blocks, args.n_blocks))
        save_3d_view_png(h, blocks, args.arena_size,
                         os.path.join(out_dir, f"{base}_3d.png"),
                         n_blocks=(args.n_blocks, args.n_blocks))
    except Exception as e:
        print(f"[replicator] matplotlib previews failed: {e}", flush=True)

    return blocks


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    sim_cfg = SimulationCfg(dt=0.004)
    sim = SimulationContext(sim_cfg)
    # GUI viewport hint (cosmetic; doesn't affect the headless render product)
    sim.set_camera_view(eye=tuple(args.cam_eye), target=tuple(args.cam_target))

    blocks = build_scene(sim, args)
    stage = sim.stage

    # ── Plain UsdGeom.Camera prim (NOT isaaclab.sensors.Camera) ──
    # Hydra reads this prim's xformOp directly from USD at render time.
    # No XformPrimView, no Fabric, no usdrt.hierarchy — bypasses the
    # broken pipeline.
    cam_prim = UsdGeom.Camera.Define(stage, CAM_PATH)
    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    op = xf.AddTransformOp()
    op.Set(_lookat_matrix(args.cam_eye, args.cam_target))

    # Camera intrinsics — wider FoV than test_walk's 24mm so a 6 m arena
    # actually fits in the frame.
    cam_prim.GetFocalLengthAttr().Set(float(args.focal_length))
    cam_prim.GetHorizontalApertureAttr().Set(20.955)
    cam_prim.GetVerticalApertureAttr().Set(11.7)   # 16:9 sensor
    cam_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))

    print(f"[replicator] camera at eye={tuple(args.cam_eye)} "
          f"target={tuple(args.cam_target)}  FoV~"
          f"{2*math.degrees(math.atan(0.5*20.955/args.focal_length)):.0f}°",
          flush=True)

    # ── Replicator render product on the USD camera ──
    print(f"[replicator] creating render product "
          f"{args.width}×{args.height} ...", flush=True)
    rp = rep.create.render_product(CAM_PATH, (args.width, args.height))

    # RGB annotator — modern API path
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(rp)

    sim.reset()

    # ── Warmup ──
    print(f"[replicator] warmup ({args.n_warmup} steps; first one may "
          f"take 10-30s while RTX shaders compile) ...", flush=True)
    t0 = time.time()
    for i in range(args.n_warmup):
        sim.step(render=True)
        if i == 0:
            print(f"  first step done in {time.time()-t0:.1f}s",
                  flush=True)
    print(f"[replicator] warmup total {time.time()-t0:.1f}s", flush=True)

    # ── Capture ──
    # -- Final still capture --
    print("[replicator] capturing final frame ...", flush=True)
    sim.step(render=True)
    arr = rgb_annotator.get_data()

    if arr is None:
        print("[replicator] ERROR: annotator returned None.",
              flush=True)
        sim_app.close()
        return

    arr = np.asarray(arr)
    print(f"  raw shape={arr.shape}  dtype={arr.dtype}  "
          f"min={arr.min()}  max={arr.max()}  mean={arr.mean():.1f}  "
          f"std={arr.std():.1f}", flush=True)

    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    if arr.dtype != np.uint8:
        if arr.max() <= 1.5:
            arr = np.clip(arr * 255.0, 0, 255).astype(np.uint8)
        else:
            arr = np.clip(arr, 0, 255).astype(np.uint8)

    try:
        from PIL import Image
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".",
                    exist_ok=True)
        Image.fromarray(arr).save(args.out)
        print(f"[replicator] saved -> {args.out}", flush=True)
    except Exception as e:
        print(f"[replicator] PNG save failed: {e}", flush=True)
        np.save(args.out + ".npy", arr)
        print(f"[replicator] raw RGB -> {args.out}.npy", flush=True)

    # -- Optional video capture (orbiting camera) --
    if args.video:
        n_frames = max(1, int(round(args.duration * args.video_fps)))
        eye0  = np.asarray(args.cam_eye,    dtype=float)
        tgt   = np.asarray(args.cam_target, dtype=float)
        rel    = eye0 - tgt
        radius = float(np.hypot(rel[0], rel[1]))
        z_eye  = float(eye0[2])
        ang0   = math.atan2(rel[1], rel[0])
        d_ang  = math.radians(float(args.orbit_degrees))

        print(f"[replicator] video: {n_frames} frames @ "
              f"{args.video_fps} fps, orbit "
              f"{args.orbit_degrees:+.0f} deg around "
              f"target={tuple(args.cam_target)}  "
              f"radius={radius:.2f} m  height={z_eye:.2f} m",
              flush=True)

        frames = []
        n_bad  = 0
        cap_t0 = time.time()
        for i in range(n_frames):
            t = i / max(n_frames - 1, 1)
            ang = ang0 + d_ang * t
            eye_i = (tgt[0] + radius * math.cos(ang),
                     tgt[1] + radius * math.sin(ang),
                     z_eye)
            op.Set(_lookat_matrix(eye_i, tgt))
            sim.step(render=True)
            arr_i = rgb_annotator.get_data()
            if arr_i is None:
                n_bad += 1
                if frames:
                    frames.append(frames[-1].copy())
                continue
            arr_i = np.asarray(arr_i)
            if arr_i.ndim == 3 and arr_i.shape[2] == 4:
                arr_i = arr_i[:, :, :3]
            if arr_i.dtype != np.uint8:
                if arr_i.max() <= 1.5:
                    arr_i = np.clip(arr_i * 255.0, 0, 255).astype(np.uint8)
                else:
                    arr_i = np.clip(arr_i, 0, 255).astype(np.uint8)
            if float(arr_i.mean()) < 3.0 and frames:
                arr_i = frames[-1].copy()
                n_bad += 1
            frames.append(arr_i)

        if not frames:
            print("[replicator] no video frames captured", flush=True)
        else:
            try:
                import cv2
                h_, w_ = frames[0].shape[:2]
                os.makedirs(os.path.dirname(os.path.abspath(args.video))
                            or ".", exist_ok=True)
                writer = cv2.VideoWriter(
                    args.video, cv2.VideoWriter_fourcc(*'mp4v'),
                    args.video_fps, (w_, h_))
                for fr in frames:
                    writer.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
                writer.release()
                print(f"[replicator] wrote {args.video}  "
                      f"({len(frames)} frames, {n_bad} bad/replaced, "
                      f"capture took {time.time()-cap_t0:.1f}s)",
                      flush=True)
            except ImportError:
                try:
                    import imageio
                    imageio.mimwrite(args.video, frames,
                                     fps=args.video_fps)
                    print(f"[replicator] wrote {args.video} (imageio)",
                          flush=True)
                except ImportError:
                    print("[replicator] no cv2/imageio - video skipped",
                          flush=True)

    sim_app.close()


if __name__ == "__main__":
    main()
