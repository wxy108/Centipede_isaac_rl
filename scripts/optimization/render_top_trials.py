#!/usr/bin/env python3
"""
render_top_trials.py — IN-PROCESS batch video renderer for top trials
======================================================================

Boots Isaac Sim ONCE (HEADLESS, --enable_cameras), builds the scene
ONCE on the OPTIMIZER'S TRAINING TERRAIN (rough λ=18 mm by default),
creates the camera ONCE BEFORE sim.reset() (the documented working
order — same as test_walk.py), then for each of the top N trials:

  1. apply_params(controller, trial_params)   — patch gains in-place
  2. reset_robot_and_controller(...)          — robot back to spawn pose
  3. Run sim for --duration seconds, capturing video frames
     (mirrors test_walk.py main loop: sim.step(render=True) → cam.update
      → bad-frame guard → append to MP4 buffer)
  4. Save the video as rank_NN_trial_TTT_cost_CCCC.mp4

Defaults match the optimizer's TRAINING TERRAIN parameters AND
test_walk.py's VISUAL FIDELITY:
  --terrain rough              ← matches optimizer
  --terrain-wavelength-mm 18   ← matches optimizer
  --terrain-amplitude 0.01     ← matches optimizer
  --terrain-seed 42            ← matches optimizer (same heightmap)
  --mesh-resolution 1024       ← matches test_walk.py for SMOOTH wave-like
                                 visuals (optimizer uses 128 internally
                                 for collision speed; heightmap is the
                                 same, only triangle density differs)
  --duration 8                 ← matches optimizer trial duration
  --camera-mode chase          ← matches test_walk.py default

Override --terrain flat or --camera-mode static for faster diagnostic
rendering.

Output:
  <run_dir>/top_videos/rank_<NN>_trial_<TTT>_cost_<CCCC>.mp4
  <run_dir>/top_yamls/rank_<NN>_trial_<TTT>.yaml          (drop-in YAMLs)

Usage
-----
  # Top 10, default settings (rough λ=18mm + chase cam + 8s clips)
  python scripts/optimization/render_top_trials.py --top 10

  # Top 30 with cost/speed/straightness filters
  python scripts/optimization/render_top_trials.py --top 30 \
      --min-speed 15 --min-strt 0.6

  # Faster diagnostic batch (flat terrain, static cam, 5s clips)
  python scripts/optimization/render_top_trials.py --top 30 \
      --terrain flat --camera-mode static --duration 5

  # Resume a partial batch (skip videos that already exist)
  python scripts/optimization/render_top_trials.py --top 30 --skip-existing

Render pipeline (inherited from test_walk.py — see NOTES_RENDERING.md):
  - --headless and --enable_cameras forced into sys.argv at startup
  - /rtx/rendermode = "RaytracedLighting"  (no path-tracer noise)
  - Camera created BEFORE sim.reset()      (proper render-product alloc)
  - Camera.set_world_poses() with USD TransformOp fallback  (Fabric write)
  - Bad-frame guard (mean<3) reuses previous good frame
  - NaN guard freezes chase cam if root_pos misbehaves
"""

import argparse
import csv
import glob
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

# Windows cmd.exe defaults to the GBK (cp936) codec, which cannot encode the
# ✓/✗ status glyphs this script prints — an unguarded print then raises
# UnicodeEncodeError and aborts the whole batch mid-render. Force UTF-8 on the
# streams so status prints never crash the run regardless of console locale.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# Args + Isaac Sim boot — this MUST happen before any isaaclab imports
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, SCRIPT_DIR)

# S2b 2026-07-16: repointed farms_controller.yaml -> farms_controller_numpy_legacy.yaml.
# This script drives the NUMPY controller (controllers/impedance_controller.py), which still computes torque
# and NEEDS the impedance gains. S2b stripped the gains out of farms_controller.yaml because the LIVE torch
# controller is targets-only now; the numpy ref would have SILENTLY DEFAULTED (body_kp 0.14396 -> 5.0, 35x).
# The gains live on, byte-identical, in the _numpy_legacy copy.
CFG_PATH = os.path.join(PROJECT_ROOT, "configs", "farms_controller_numpy_legacy.yaml")
USD_PATH = os.path.join(PROJECT_ROOT, "assets", "centipede.usd")

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("csv", nargs="?", default=None,
                    help="all_trials.csv (defaults to most recent run).")
parser.add_argument("--top",         type=int,   default=10)
parser.add_argument("--min-speed",   type=float, default=0.0,
                    help="Filter: speed_mps × 1000 >= this (mm/s).")
parser.add_argument("--min-strt",    type=float, default=0.0)
parser.add_argument("--include-buckled", action="store_true")
parser.add_argument("--duration",    type=float, default=8.0,
                    help="Sim seconds per video (default 8 — matches "
                         "optimization trial duration).")
parser.add_argument("--terrain",     choices=["rough", "flat"], default="rough",
                    help="DEFAULT 'rough' — matches the optimizer's "
                         "training terrain so the gait is evaluated under "
                         "the same conditions it was tuned for. Pass "
                         "--terrain flat for faster diagnostic clips.")
parser.add_argument("--terrain-wavelength-mm", type=float, default=18.0,
                    help="Default 18 mm — matches the optimization study.")
parser.add_argument("--terrain-amplitude",     type=float, default=0.01,
                    help="Default 0.01 m (10 mm peak) — matches optimization.")
parser.add_argument("--terrain-seed",          type=int,   default=42,
                    help="Default 42 — same seed as optimization training.")
parser.add_argument("--mesh-resolution",       type=int,   default=1024,
                    help="Default 1024 — matches test_walk.py for SMOOTH "
                         "wave-like terrain visuals (same as your previous "
                         "test_walk renders). Optimizer uses 128 internally "
                         "for collision speed; the heightmap is identical "
                         "between the two — only the triangle density "
                         "differs (1024² verts → smooth bumps, 128² → "
                         "faceted/discrete look). For video purposes the "
                         "1024 visual is what we want.")
parser.add_argument("--camera-mode",  choices=["chase", "static"], default="chase",
                    help="DEFAULT 'chase' — same as test_walk.py default. "
                         "Pass --camera-mode static if chase struggles "
                         "to render on rough terrain.")
parser.add_argument("--video-fps",   type=int,   default=30)
parser.add_argument("--skip-existing", action="store_true",
                    help="Skip trials whose video already exists.")
parser.add_argument("--debug-pngs",  action="store_true",
                    help="Save first 3 frames of each trial as PNG for "
                         "visual inspection.")

# Isaac Lab AppLauncher must add its args BEFORE parse_args
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)

# FORCE --headless so the GUI viewport never opens (it would slow batch
# rendering AND make the script require an X-server / desktop session).
# Pre-inject into argv before parse_args so AppLauncher sees it through
# its normal CLI path (matches the pattern test_walk.py uses for
# --enable_cameras).
if "--headless" not in sys.argv:
    sys.argv.append("--headless")
if "--enable_cameras" not in sys.argv:
    sys.argv.append("--enable_cameras")

args = parser.parse_args()
args.headless = True
args.enable_cameras = True

# Boot Isaac Sim once for the whole batch
app_launcher = AppLauncher(args)
sim_app = app_launcher.app

# Render-pipeline settings (see NOTES_RENDERING.md §1)
import carb
_settings = carb.settings.get_settings()
_settings.set_bool("/isaaclab/render/rtx_sensors", True)
_settings.set_string("/rtx/rendermode", "RaytracedLighting")

# Now safe to import the rest of the Isaac Lab + project stack
import torch
import yaml
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import Camera, CameraCfg

from controllers.impedance_controller import ImpedanceTravelingWaveController
from controllers.kinematics import N_BODY_JOINTS, N_LEGS, N_LEG_DOF
from terrain import generate_single_wavelength_heightmap, build_terrain_mesh


BUCKLED_THR = 1e4
CAM_PATH    = "/World/BatchCamera"
STATIC_EYE  = (-0.35, -0.30, 0.25)
STATIC_TGT  = ( 0.10,  0.00, 0.02)
CHASE_OFF   = (-0.22, -0.18, 0.15)


# ─────────────────────────────────────────────────────────────────────────────
# CSV helpers
# ─────────────────────────────────────────────────────────────────────────────
def find_latest_csv():
    pat = os.path.join(PROJECT_ROOT, "outputs", "optimization",
                       "impedance_bay_*", "all_trials.csv")
    matches = sorted(glob.glob(pat), key=os.path.getmtime, reverse=True)
    return matches[0] if matches else None


def f(row, key, default=0.0):
    v = row.get(key, "")
    try:
        return float(v) if v != "" else default
    except (TypeError, ValueError):
        return default


def row_to_params(row):
    """Extract gain params from a CSV row into the dict apply_params expects."""
    p = {}
    for k in ("body_kp", "body_kv",
              "hip_yaw_kp", "hip_yaw_kv",
              "hip_pitch_kp", "hip_pitch_kv",
              "tibia_kp", "tibia_kv",
              "tarsus_kp", "tarsus_kv",
              "pitch_kp", "pitch_kv",
              "head_yaw_gain"):   # A3 fidelity: the de-spin heading servo the
                                  # candidates rely on — apply_params() handles
                                  # it, but it was never EXTRACTED here, so the
                                  # render ran the default 1.0 (would show a
                                  # spin the optimized controller doesn't have).
        if k in row and row[k] != "":
            try:
                p[k] = float(row[k])
            except ValueError:
                pass
    return p


def export_yaml(row, base_yaml_path, out_path):
    with open(base_yaml_path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    imp = cfg.setdefault("impedance", {})
    imp["body_kp"] = f(row, "body_kp")
    imp["body_kv"] = f(row, "body_kv")
    if "pitch_kp" in row:
        imp["pitch_kp"] = f(row, "pitch_kp")
        imp["pitch_kv"] = f(row, "pitch_kv")
    leg = imp.setdefault("leg", {})
    kp = list(leg.get("kp", [0.5, 0.06, 0.13, 0.13]))
    kv = list(leg.get("kv", [0.001, 0.003, 0.0005, 0.001]))
    kp[0] = f(row, "hip_yaw_kp");   kv[0] = f(row, "hip_yaw_kv")
    kp[1] = f(row, "hip_pitch_kp"); kv[1] = f(row, "hip_pitch_kv")
    if "tibia_kp" in row:
        kp[2] = f(row, "tibia_kp");  kv[2] = f(row, "tibia_kv")
    if "tarsus_kp" in row:
        kp[3] = f(row, "tarsus_kp"); kv[3] = f(row, "tarsus_kv")
    leg["kp"] = kp; leg["kv"] = kv
    with open(out_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(cfg, fh, sort_keys=False)


# ─────────────────────────────────────────────────────────────────────────────
# Scene building (mirrors optimize_impedance_bayesian.build_scene)
# ─────────────────────────────────────────────────────────────────────────────
def build_scene(sim, args):
    from pxr import UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf
    stage = sim.stage

    # Lighting
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(300.0)
    dist = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    dist.GetIntensityAttr().Set(800.0)
    dist.GetAngleAttr().Set(0.53)
    UsdGeom.Xformable(dist.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))

    if args.terrain == "flat":
        UsdGeom.Xform.Define(stage, "/World/GroundPlane")
        slab = UsdGeom.Cube.Define(stage, "/World/GroundPlane/Slab")
        slab.GetSizeAttr().Set(1.0)
        sx = UsdGeom.Xformable(slab); sx.ClearXformOpOrder()
        sx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
        sx.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 0.1))
        UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
        print("[scene] flat slab at z=0", flush=True)
    else:
        wavelength_m = args.terrain_wavelength_mm * 1e-3
        h, rms_m, peak_m = generate_single_wavelength_heightmap(
            wavelength_m=wavelength_m,
            amplitude_m=args.terrain_amplitude,
            seed=args.terrain_seed)
        UsdGeom.Xform.Define(stage, "/World/Terrain")
        build_terrain_mesh(stage=stage, prim_path="/World/Terrain/Mesh",
                           heightmap_m=h, world_size=1.0,
                           mesh_resolution=args.mesh_resolution)
        print(f"[scene] rough terrain  λ={args.terrain_wavelength_mm:.1f}mm  "
              f"amp={args.terrain_amplitude*1000:.1f}mm  "
              f"peak={peak_m*1000:.1f}mm", flush=True)

    # Centipede
    spawn_cfg = sim_utils.UsdFileCfg(usd_path=USD_PATH)
    spawn_cfg.func("/World/Centipede", spawn_cfg, translation=(0.0, 0.0, 0.05))
    wb = stage.GetPrimAtPath("/World/Centipede/worldBody")
    if wb.IsValid():
        wb.SetActive(False)

    robot_cfg = ArticulationCfg(
        prim_path="/World/Centipede/link_body_0",
        spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.05)),
        actuators={
            "active_joints": ImplicitActuatorCfg(
                joint_names_expr=["joint_body_.*", "joint_pitch_body_.*", "joint_leg_.*"],
                effort_limit=10.0, velocity_limit=100.0,
                stiffness=0.0, damping=0.0),
            "foot_passive": ImplicitActuatorCfg(
                joint_names_expr=["joint_foot_.*"],
                effort_limit=1.0, velocity_limit=100.0,
                stiffness=0.0, damping=0.006),
        })
    return Articulation(robot_cfg)


# ─────────────────────────────────────────────────────────────────────────────
# Per-trial: apply gains + reset
# ─────────────────────────────────────────────────────────────────────────────
def apply_params(controller, params):
    if "body_kp" in params: controller.body_kp = float(params["body_kp"])
    if "body_kv" in params: controller.body_kv = float(params["body_kv"])
    if hasattr(controller, "_build_body_kp_vec"):
        controller._build_body_kp_vec()
    if "hip_yaw_kp" in params:
        controller.leg_kp[0] = float(params["hip_yaw_kp"])
        controller.leg_kv[0] = float(params["hip_yaw_kv"])
    if "hip_pitch_kp" in params:
        controller.leg_kp[1] = float(params["hip_pitch_kp"])
        controller.leg_kv[1] = float(params["hip_pitch_kv"])
    if "tibia_kp" in params:
        controller.leg_kp[2] = float(params["tibia_kp"])
        controller.leg_kv[2] = float(params["tibia_kv"])
    if "tarsus_kp" in params:
        controller.leg_kp[3] = float(params["tarsus_kp"])
        controller.leg_kv[3] = float(params["tarsus_kv"])
    controller._leg_kp_flat = np.tile(controller.leg_kp, N_LEGS * 2)
    controller._leg_kv_flat = np.tile(controller.leg_kv, N_LEGS * 2)
    if "pitch_kp" in params and hasattr(controller, "pitch_kp_vec"):
        kp = float(params["pitch_kp"]); kv = float(params["pitch_kv"])
        n_p = len(controller.pitch_kp_vec)
        for k in range(n_p):
            if k > 0:   # skip head pitch
                controller.pitch_kp_vec[k] = kp
                controller.pitch_kv_vec[k] = kv
    # STAGE A: apply the swept head_yaw_gain (proprioceptive hold-initial-heading
    # de-spin servo) so the rendered gait matches the optimized winner. Mirrors
    # apply_params() in optimize_impedance_biological.py.
    if "head_yaw_gain" in params:
        controller.head_yaw_gain = float(params["head_yaw_gain"])


def reset_robot(robot, controller, sim, n_settle=8):
    n_j = robot.num_joints; dev = robot.device
    zero_q = torch.zeros(1, n_j, device=dev)
    zero_v = torch.zeros(1, n_j, device=dev)
    robot.write_joint_state_to_sim(zero_q, zero_v)
    root_state = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=dev))
    controller.head_yaw_ref = None
    controller.body_phases  = None
    controller.leg_phases   = None
    controller._cpg_initialized = False
    dt = sim.get_physics_dt()
    for _ in range(n_settle):
        robot.update(dt)
        controller.step(0.0, dt)
        sim.step(render=False)


# ─────────────────────────────────────────────────────────────────────────────
# Camera pose helpers (mirrors test_walk.py's working pattern)
# ─────────────────────────────────────────────────────────────────────────────
def lookat_quat(eye, target):
    """Look-at quaternion (w,x,y,z) in OpenGL camera convention."""
    fwd = np.array(target, float) - np.array(eye, float)
    fwd /= max(np.linalg.norm(fwd), 1e-12)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= max(np.linalg.norm(right), 1e-12)
    cam_up = np.cross(right, fwd)
    R = np.column_stack([right, cam_up, -fwd])
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return 0.25 * s, (R[2,1] - R[1,2]) / s, (R[0,2] - R[2,0]) / s, (R[1,0] - R[0,1]) / s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        return (R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        return (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s
    else:
        s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        return (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s


def set_camera_pose(cam, eye, target, _state):
    """Try the Camera tensor API; on failure (usdrt.hierarchy missing on
    some Isaac Lab builds) fall back to a USD TransformOp write. Latches
    the failure on _state['tensor_works'] so we don't keep re-throwing."""
    qw, qx, qy, qz = lookat_quat(eye, target)
    if _state.get("tensor_works") is not False:
        try:
            try:
                cam_dev = cam.device
            except AttributeError:
                cam_dev = getattr(cam, "_device", "cuda")
            pos_t  = torch.tensor([list(eye)], dtype=torch.float32, device=cam_dev)
            quat_t = torch.tensor([[qw, qx, qy, qz]], dtype=torch.float32, device=cam_dev)
            cam.set_world_poses(positions=pos_t, orientations=quat_t,
                                convention="opengl")
            if _state.get("tensor_works") is None:
                _state["tensor_works"] = True
                print("[batch] Camera.set_world_poses works (Fabric).", flush=True)
            return
        except Exception as e:
            _state["tensor_works"] = False
            print(f"[batch] tensor pose API unavailable ({type(e).__name__}: "
                  f"{e}) — falling back to USD xform write.", flush=True)
    # USD fallback
    import omni.usd
    from pxr import UsdGeom, Gf
    stage = omni.usd.get_context().get_stage()
    cam_prim = stage.GetPrimAtPath(CAM_PATH)
    eye_v = Gf.Vec3d(*eye); tgt_v = Gf.Vec3d(*target); up = Gf.Vec3d(0,0,1)
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
# Per-trial render
# ─────────────────────────────────────────────────────────────────────────────
def render_one_trial(sim, robot, controller, cam, cam_state, args, video_path):
    """Run one trial, capture frames, save MP4. Returns (n_frames, n_replaced)."""
    dt = sim.get_physics_dt()
    n_steps = int(args.duration / dt)
    video_dt = 1.0 / args.video_fps
    last_frame_t = -np.inf
    frames = []
    n_replaced = 0

    for step in range(n_steps):
        t = step * dt
        robot.update(dt)
        controller.step(t, dt)

        want_frame = (t - last_frame_t) >= video_dt - 1e-10

        if want_frame and args.camera_mode == "chase":
            try:
                root_pos = robot.data.root_pos_w[0].cpu().numpy()
                if np.all(np.isfinite(root_pos)) and \
                   max(abs(root_pos[0]), abs(root_pos[1]), abs(root_pos[2])) < 5.0:
                    tgt = (float(root_pos[0]), float(root_pos[1]), float(root_pos[2]))
                    eye = (tgt[0]+CHASE_OFF[0], tgt[1]+CHASE_OFF[1], tgt[2]+CHASE_OFF[2])
                    set_camera_pose(cam, eye, tgt, cam_state)
            except Exception:
                pass  # keep last good camera pose

        sim.step(render=want_frame)
        if want_frame:
            cam.update(dt)
            rgb = cam.data.output.get("rgb", None)
            if rgb is not None:
                arr = rgb[0].detach().cpu().numpy()
                if arr.ndim == 3 and arr.shape[2] == 4:
                    arr = arr[:, :, :3]
                arr = arr.astype(np.uint8)
                # Bad-frame guard: pure-black mid-write race
                if float(arr.mean()) < 3.0 and frames:
                    arr = frames[-1].copy()
                    n_replaced += 1
                frames.append(arr)
            last_frame_t = t

    if not frames:
        return 0, n_replaced

    # Save MP4
    try:
        import cv2
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(video_path,
                                 cv2.VideoWriter_fourcc(*'mp4v'),
                                 args.video_fps, (w, h))
        for fr in frames:
            writer.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
        writer.release()
    except ImportError:
        try:
            import imageio
            imageio.mimwrite(video_path, frames, fps=args.video_fps)
        except ImportError:
            print(f"            ! cv2/imageio unavailable — skipping MP4 write")
            return len(frames), n_replaced

    if args.debug_pngs:
        try:
            from PIL import Image
            for i in range(min(3, len(frames))):
                Image.fromarray(frames[i]).save(
                    video_path.replace(".mp4", f"_dbg_{i}.png"))
        except Exception:
            pass

    return len(frames), n_replaced


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    csv_path = args.csv or find_latest_csv()
    if not csv_path or not os.path.isfile(csv_path):
        print("ERROR: no all_trials.csv found.", file=sys.stderr)
        sim_app.close()
        return 2

    rows = list(csv.DictReader(open(csv_path, "r", newline="")))
    filtered = []
    for r in rows:
        cost = f(r, "cost", default=BUCKLED_THR)
        if not args.include_buckled and cost >= BUCKLED_THR * 0.99:
            continue
        if f(r, "speed_mps") * 1000.0 < args.min_speed:
            continue
        if f(r, "straightness") < args.min_strt:
            continue
        filtered.append(r)
    filtered.sort(key=lambda r: f(r, "cost", default=BUCKLED_THR))
    top = filtered[:args.top]
    if not top:
        print("No trials match the filters.", file=sys.stderr)
        sim_app.close()
        return 1

    run_dir   = os.path.dirname(csv_path)
    video_dir = os.path.join(run_dir, "top_videos")
    yaml_dir  = os.path.join(run_dir, "top_yamls")
    os.makedirs(video_dir, exist_ok=True)
    os.makedirs(yaml_dir,  exist_ok=True)

    print(f"\n[batch] reading {csv_path}")
    print(f"[batch] {len(rows)} trials in CSV  ·  {len(filtered)} match filters  "
          f"·  rendering top {len(top)}")
    print(f"[batch] terrain={args.terrain}  camera={args.camera_mode}  "
          f"duration={args.duration}s  video_fps={args.video_fps}")
    print(f"[batch] output dir: {video_dir}")

    # ── Build sim, scene, camera ONCE for the whole batch ──
    with open(CFG_PATH, "r", encoding="utf-8") as f_cfg:
        base_cfg = yaml.safe_load(f_cfg)
    dt_phys = float(base_cfg.get("simulation", {}).get("timestep", 0.004))

    sim_cfg = SimulationCfg(dt=dt_phys)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[0.5, 0.5, 0.3], target=[0.0, 0.0, 0.0])

    print("[batch] building scene ...", flush=True)
    robot = build_scene(sim, args)

    # Camera MUST be created BEFORE sim.reset
    cam_cfg = CameraCfg(
        prim_path=CAM_PATH,
        update_period=0.0,
        height=720, width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.05, 100.0)))
    cam = Camera(cfg=cam_cfg)

    sim.reset()
    robot.reset()
    print(f"[batch] articulation: {robot.num_joints} joints", flush=True)

    controller = ImpedanceTravelingWaveController(robot, CFG_PATH)

    # ── Initial render warmup (RTX shader compile) ──
    print("[batch] render warmup ...", flush=True)
    for _ in range(5):
        sim.step(render=True)
    cam.update(dt_phys)

    # ── Initial camera pose (static) ──
    cam_state = {}   # latches tensor_works flag + xform_op cache
    if args.camera_mode == "static":
        set_camera_pose(cam, STATIC_EYE, STATIC_TGT, cam_state)
    else:
        # chase: seed at robot spawn so first frame isn't at origin
        set_camera_pose(cam,
                        (CHASE_OFF[0], CHASE_OFF[1], 0.05 + CHASE_OFF[2]),
                        (0.0, 0.0, 0.05), cam_state)

    # ── Per-trial loop ──
    t0_batch = time.time()
    n_done = n_skipped = n_failed = 0

    for rank, r in enumerate(top, start=1):
        trial_num = int(f(r, "trial"))
        cost      = f(r, "cost")
        speed_mm  = f(r, "speed_mps") * 1000
        strt      = f(r, "straightness")

        video_name = (f"rank_{rank:02d}_trial_{trial_num:04d}"
                      f"_cost_{cost:+07.2f}.mp4")
        video_path = os.path.join(video_dir, video_name)
        yaml_path  = os.path.join(yaml_dir,
                                  f"rank_{rank:02d}_trial_{trial_num:04d}.yaml")

        print(f"\n[batch {rank}/{len(top)}] trial #{trial_num}  cost={cost:+.3f}  "
              f"speed={speed_mm:.1f}mm/s  strt={strt:.3f}", flush=True)

        if args.skip_existing and os.path.isfile(video_path):
            print(f"            [skip] video already exists")
            n_skipped += 1
            continue

        # Save trial's gains as a YAML for later use
        export_yaml(r, CFG_PATH, yaml_path)

        # Apply params + reset robot
        params = row_to_params(r)
        try:
            apply_params(controller, params)
            reset_robot(robot, controller, sim)
        except Exception as e:
            print(f"            ✗ apply/reset failed: {e}")
            n_failed += 1
            continue

        # Render
        t0 = time.time()
        try:
            n_frames, n_replaced = render_one_trial(
                sim, robot, controller, cam, cam_state, args, video_path)
            wall = time.time() - t0
            if n_frames > 0:
                size_kb = os.path.getsize(video_path) / 1024
                pct_repl = 100.0 * n_replaced / max(n_frames, 1)
                print(f"            ✓ {video_name}  ({size_kb:.0f} KB, "
                      f"{wall:.1f}s, {n_frames} frames, "
                      f"{pct_repl:.0f}% replaced)")
                n_done += 1
            else:
                print(f"            ✗ no frames captured ({wall:.1f}s)")
                n_failed += 1
        except Exception as e:
            wall = time.time() - t0
            print(f"            ✗ render exception ({wall:.1f}s): {e}")
            import traceback; traceback.print_exc()
            n_failed += 1

    elapsed = time.time() - t0_batch
    print(f"\n[batch] done — {n_done} rendered, {n_skipped} skipped, "
          f"{n_failed} failed  ·  {elapsed/60:.1f} min total")
    print(f"[batch] videos in: {video_dir}")
    print(f"[batch] yamls   in: {yaml_dir}")
    print(f"\nWhen you've picked a winner:")
    print(f"  copy <yaml_dir>\\rank_<NN>_trial_<TTT>.yaml configs\\farms_controller.yaml")

    # Canonical completion marker so run_isaaclab_with_watchdog.ps1 force-kills
    # python on teardown (Omniverse cleanup otherwise hangs 5-6 min).
    print(f"[viz] report -> {video_dir}", flush=True)
    sim_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
