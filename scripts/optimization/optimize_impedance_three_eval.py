#!/usr/bin/env python3
"""
optimize_impedance_three_eval.py — Three-evaluation Bayesian impedance tuner
=============================================================================
Each Optuna trial tests the SAME gain set on three sequential evaluations
within a single Isaac Sim session, switching the controller's body-wave
frequency between them. The goal is to find gain sets that work across
multiple frequencies (so the same controller doesn't fail when DRL later
sweeps gait timing).

Three evaluations per trial:
  A. Flat ground at 2 Hz       — CoT bio-comparable (target 0.20–0.38 J/m,
                                  Full 1989 inter-arthropod scaling × 2.5g body)
  B. Rough λ=18mm at 1 Hz      — slow-gait robustness on rough terrain
  C. Rough λ=18mm at 2 Hz      — fast-gait robustness on rough terrain

Trial cost = mean(eval_A_cost, eval_B_cost, eval_C_cost).
Any eval that buckles / flips / stuck → trial cost = BUCKLED_COST (1e5).

Constraint cost (identical to optimize_impedance_biological.py):
  cost = CoT  if all hard constraints satisfied
  cost = CoT + 100·Σ(violation_excess)  otherwise
  (w_torque_jerk·jerk is computed but NOT folded into cost — see
  BIO_REFERENCES.md §8.2.)

Hard constraints per eval:
  velocity_quality        ≥ 0.30   [§1.11 Pierce 2023]
  body_z_rms / nom_clear  ≤ 0.10   [§1.11 Pierce 2023]
  body_roll_rms_deg       ≤ 5.0    [§1.12 Pierce 2026]
  peak_F / body_weight    ≤ 8.0    [§1.5  Cocci 2024]

Usage (smoke test, ~10 minutes):
    python scripts\\optimization\\optimize_impedance_three_eval.py ^
        --headless --pretest --n-trials 5 --duration 4

Usage (full overnight, ~10 hours):
    python scripts\\optimization\\optimize_impedance_three_eval.py ^
        --headless --pretest --n-trials 200 --duration 6 --run-tag three_eval_v1
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

# ── Path setup ───────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, SCRIPT_DIR)

# ── Argparse + AppLauncher boot ──────────────────────────────────────────────
parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--n-trials", type=int, default=200)
parser.add_argument("--duration", type=float, default=6.0,
                    help="Sim seconds PER EVALUATION (each trial does 3 evals "
                         "→ trial wall-clock = 3 × per-eval cost).")
parser.add_argument("--seed",     type=int, default=42)
parser.add_argument("--terrain-wavelength-mm", type=float, default=18.0,
                    help="Wavelength of the rough patch in mm (default 18).")
parser.add_argument("--terrain-amplitude", type=float, default=0.01,
                    help="Peak amplitude of rough patch in m (default 0.01).")
parser.add_argument("--terrain-seed", type=int, default=42)
parser.add_argument("--rough-mesh-resolution", type=int, default=1024,
                    help="Collision mesh resolution for rough patch. 1024 over "
                         "1m = 1mm cells, fine enough for 10mm wavelengths.")
parser.add_argument("--freq-flat",    type=float, default=2.0,
                    help="Body-wave frequency for flat-ground eval (Hz).")
parser.add_argument("--freq-rough-low",  type=float, default=1.0)
parser.add_argument("--freq-rough-high", type=float, default=2.0)
parser.add_argument("--tune-distal",    dest="tune_distal", action="store_true",
                    default=True)
parser.add_argument("--no-tune-distal", dest="tune_distal", action="store_false")
parser.add_argument("--tune-pitch",     dest="tune_pitch",  action="store_true",
                    default=True)
parser.add_argument("--no-tune-pitch",  dest="tune_pitch",  action="store_false")
parser.add_argument("--seed-from", default=None,
                    help="Seed first trial from a best_params.yaml.")
parser.add_argument("--resume", default=None,
                    help="Resume an existing study.db.")
parser.add_argument("--run-tag", default="three_eval",
                    help="Inserted into the output dir name.")
parser.add_argument("--pretest", action="store_true",
                    help="Enqueue 4 edge probes BEFORE TPE starts.")
# ── Constraint thresholds (match Isaac Lab + MuJoCo bio cost defaults) ──
parser.add_argument("--leg-proj-length-m",     type=float, default=0.035)
parser.add_argument("--nominal-clearance-m",   type=float, default=0.0258)
parser.add_argument("--min-velocity-quality",  type=float, default=0.3)
parser.add_argument("--max-z-rms-ratio",       type=float, default=0.10)
parser.add_argument("--max-roll-deg",          type=float, default=5.0)
parser.add_argument("--force-limit",           type=float, default=8.0)

# AppLauncher must add its args before parse_args
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

# ── Now safe to import Isaac Lab pieces ──────────────────────────────────────
import torch
import yaml
try:
    import optuna
    from optuna.samplers import TPESampler
except ImportError:
    print("ERROR: optuna required. Install with: pip install optuna")
    sim_app.close(); sys.exit(1)

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

from controllers.impedance_controller import ImpedanceTravelingWaveController
from controllers.kinematics import N_BODY_JOINTS, N_LEGS, N_LEG_DOF
from terrain import generate_single_wavelength_heightmap, build_terrain_mesh

CFG_PATH    = os.path.join(PROJECT_ROOT, "configs", "farms_controller.yaml")
USD_PATH    = os.path.join(PROJECT_ROOT, "assets", "centipede.usd")
OUTPUT_ROOT = os.path.join(PROJECT_ROOT, "outputs", "optimization")

BUCKLED_COST       = 1e5
MAX_ROOT_PITCH_DEG = 45.0
MAX_ROOT_ROLL_DEG  = 45.0

# ── Per-evaluation spawn offsets (world-frame XY for the centipede root) ──
# The centipede USD is authored at world (0, 0, 0.05). We add these offsets
# onto root_state in reset_robot_and_controller(), so an offset of (0, -2, dz)
# means "spawn 2 m -y from world origin". Picked so the flat slab and rough
# patch don't overlap or interfere physically.
FLAT_PATCH_CENTER  = (0.0, -2.0)   # x, y
ROUGH_PATCH_CENTER = (0.0,  0.0)


# ═══════════════════════════════════════════════════════════════════════════════
# Scene: flat slab at y = -2, rough λ=18mm at y = 0
# ═══════════════════════════════════════════════════════════════════════════════

def build_scene(sim, run_dir, args):
    """Spawn lights, both terrain patches, and the centipede.

    Returns:
        Articulation        — the centipede.
        rough_heightmap     — np.ndarray, the rough-patch heightmap (for
                              clearance metric); flat is zero everywhere.
        flat_offset_xyz     — (x, y, z) absolute spawn offset onto the flat patch.
        rough_offset_xyz    — (x, y, z) absolute spawn offset onto the rough patch.
    """
    from pxr import UsdGeom, UsdLux, UsdPhysics, Gf, Sdf
    stage = sim.stage

    # ── Lighting ──
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(300.0)
    dist = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    dist.GetIntensityAttr().Set(800.0)
    dist.GetAngleAttr().Set(0.53)
    UsdGeom.Xformable(dist.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))

    # ── Flat slab at FLAT_PATCH_CENTER ──
    UsdGeom.Xform.Define(stage, "/World/FlatPatch")
    slab = UsdGeom.Cube.Define(stage, "/World/FlatPatch/Slab")
    slab.GetSizeAttr().Set(1.0)
    sx = UsdGeom.Xformable(slab); sx.ClearXformOpOrder()
    # Place centred at FLAT_PATCH_CENTER, surface at z=0, thickness 0.1m below.
    sx.AddTranslateOp().Set(Gf.Vec3d(FLAT_PATCH_CENTER[0],
                                     FLAT_PATCH_CENTER[1],
                                     -0.05))
    sx.AddScaleOp().Set(Gf.Vec3f(2.0, 2.0, 0.1))   # 2m × 2m flat region
    UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
    print(f"[scene] flat patch  centre=({FLAT_PATCH_CENTER[0]:+.2f}, "
          f"{FLAT_PATCH_CENTER[1]:+.2f}) m  size=2x2 m  surface@z=0",
          flush=True)

    # ── Rough λ patch at ROUGH_PATCH_CENTER ──
    wavelength_m = args.terrain_wavelength_mm * 1e-3
    rough_h, rms_m, peak_m = generate_single_wavelength_heightmap(
        wavelength_m  = wavelength_m,
        amplitude_m   = args.terrain_amplitude,
        seed          = args.terrain_seed,
        image_size    = 1024,
        world_size    = 1.0,
    )
    UsdGeom.Xform.Define(stage, "/World/RoughPatch")
    rough_xform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/RoughPatch"))
    rough_xform.ClearXformOpOrder()
    rough_xform.AddTranslateOp().Set(Gf.Vec3d(ROUGH_PATCH_CENTER[0],
                                              ROUGH_PATCH_CENTER[1],
                                              0.0))
    build_terrain_mesh(
        stage           = stage,
        prim_path       = "/World/RoughPatch/Mesh",
        heightmap_m     = rough_h,
        world_size      = 1.0,
        mesh_resolution = args.rough_mesh_resolution,
        mujoco_compat   = True,
    )
    np.save(os.path.join(run_dir, "rough_heightmap.npy"), rough_h)
    print(f"[scene] rough patch centre=({ROUGH_PATCH_CENTER[0]:+.2f}, "
          f"{ROUGH_PATCH_CENTER[1]:+.2f}) m  size=1x1 m  "
          f"λ={args.terrain_wavelength_mm:.1f}mm  amp={args.terrain_amplitude*1000:.1f}mm  "
          f"rms={rms_m*1000:.2f}mm  peak={peak_m*1000:.2f}mm  "
          f"mesh={args.rough_mesh_resolution}",
          flush=True)

    # ── Centipede spawn (default world origin; teleported per-eval later) ──
    spawn_cfg = sim_utils.UsdFileCfg(usd_path=USD_PATH,
                                     activate_contact_sensors=True)
    spawn_cfg.func("/World/Centipede", spawn_cfg, translation=(0.0, 0.0, 0.05))
    wb = stage.GetPrimAtPath("/World/Centipede/worldBody")
    if wb.IsValid():
        wb.SetActive(False)

    # Manual PhysxContactReportAPI workaround (see optimize_impedance_biological.py)
    try:
        import re as _re
        from pxr import PhysxSchema as _PhysxSchema
        _foot_re = _re.compile(r"foot_\d+_[01]$")
        n_applied = 0
        for prim in stage.Traverse():
            if (_foot_re.search(prim.GetName())
                    and prim.HasAPI(UsdPhysics.RigidBodyAPI)
                    and not prim.HasAPI(_PhysxSchema.PhysxContactReportAPI)):
                api = _PhysxSchema.PhysxContactReportAPI.Apply(prim)
                api.CreateThresholdAttr().Set(0.0)
                n_applied += 1
        print(f"[scene] applied PhysxContactReportAPI to "
              f"{n_applied} foot rigid bodies", flush=True)
    except Exception as e:
        print(f"[scene] WARN: contact-report API workaround failed: {e}",
              flush=True)

    # Foot ContactSensor — same configuration as the biological optimizer
    foot_contact_sensor = None
    try:
        from isaaclab.sensors import ContactSensorCfg, ContactSensor
        foot_contact_cfg = ContactSensorCfg(
            prim_path="/World/Centipede/link_body_0/foot_[0-9]+_[01]",
            update_period=0.0,
            history_length=1,
            force_threshold=0.0,
        )
        foot_contact_sensor = ContactSensor(cfg=foot_contact_cfg)
        print("[scene] foot ContactSensor created", flush=True)
    except Exception as e:
        print(f"[scene] WARN: foot ContactSensor creation failed: {e}",
              flush=True)

    # Articulation config
    robot_cfg = ArticulationCfg(
        prim_path="/World/Centipede/link_body_0",
        spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.05)),
        actuators={
            "active_joints": ImplicitActuatorCfg(
                joint_names_expr=[
                    "joint_body_.*", "joint_pitch_body_.*", "joint_leg_.*",
                ],
                effort_limit=10.0, velocity_limit=100.0,
                stiffness=0.0, damping=0.0,
            ),
            "foot_passive": ImplicitActuatorCfg(
                joint_names_expr=["joint_foot_.*"],
                effort_limit=1.0, velocity_limit=100.0,
                stiffness=0.0, damping=0.006,
            ),
        },
    )

    # Compute spawn offsets so the centipede lands above each patch surface.
    # USD-authored z = 0.05; we want it at terrain_z + clearance.
    nominal_clearance = float(args.nominal_clearance_m)
    flat_terrain_z    = 0.0     # flat slab top is at z=0
    rough_terrain_z   = float(rough_h[rough_h.shape[0]//2,
                                       rough_h.shape[1]//2])  # centre pixel
    flat_offset_xyz  = (FLAT_PATCH_CENTER[0],
                        FLAT_PATCH_CENTER[1],
                        flat_terrain_z + nominal_clearance - 0.05)
    rough_offset_xyz = (ROUGH_PATCH_CENTER[0],
                        ROUGH_PATCH_CENTER[1],
                        rough_terrain_z + nominal_clearance - 0.05)

    print(f"[scene] flat  spawn offset = "
          f"({flat_offset_xyz[0]:+.3f}, {flat_offset_xyz[1]:+.3f}, "
          f"{flat_offset_xyz[2]:+.4f}) m", flush=True)
    print(f"[scene] rough spawn offset = "
          f"({rough_offset_xyz[0]:+.3f}, {rough_offset_xyz[1]:+.3f}, "
          f"{rough_offset_xyz[2]:+.4f}) m", flush=True)

    return (Articulation(robot_cfg), rough_h, flat_offset_xyz, rough_offset_xyz,
            foot_contact_sensor)


# ═══════════════════════════════════════════════════════════════════════════════
# Per-evaluation: teleport robot, set freq, run sim, compute cost
# ═══════════════════════════════════════════════════════════════════════════════

def reset_robot_to(robot, controller, sim, contact_sensor,
                   spawn_offset_xyz, n_settle_steps=8):
    """Teleport robot to spawn_offset_xyz and clear all controller state."""
    n_j = robot.num_joints
    dev = robot.device

    zero_q = torch.zeros(1, n_j, device=dev)
    zero_v = torch.zeros(1, n_j, device=dev)
    robot.write_joint_state_to_sim(zero_q, zero_v)

    root_state = robot.data.default_root_state.clone()
    root_state[:, 0] += float(spawn_offset_xyz[0])
    root_state[:, 1] += float(spawn_offset_xyz[1])
    root_state[:, 2] += float(spawn_offset_xyz[2])
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=dev))

    # Clear controller state (CPG, target history, heading reference)
    controller.head_yaw_ref = None
    controller.body_phases  = None
    controller.leg_phases   = None
    controller._cpg_initialized = False
    if hasattr(controller, "reset_target_history"):
        controller.reset_target_history()

    if contact_sensor is not None:
        try: contact_sensor.reset()
        except Exception: pass

    # Settle
    dt = sim.get_physics_dt()
    for _ in range(n_settle_steps):
        robot.update(dt)
        controller.step(0.0, dt)
        sim.step(render=False)
        if contact_sensor is not None:
            try: contact_sensor.update(dt)
            except Exception: pass


def run_one_eval(robot, controller, sim, contact_sensor, duration,
                 buckle_grace_s=1.5):
    """Drive sim for `duration` sec. Returns dict with cost-relevant metrics
    plus buckle status. Records body z, body roll, foot forces, joint torques.
    Computes CoT, velocity_quality, etc. inline (no separate Recorder class
    to keep this file self-contained).
    """
    dt      = sim.get_physics_dt()
    n_steps = int(duration / dt)

    # Per-step state captured at recorder rate (~200 Hz)
    rec_dt        = 1.0 / 200.0
    rec_interval  = max(1, int(round(rec_dt / dt)))

    times    = []
    com_z    = []
    com_xy   = []
    roll     = []
    foot_F   = []     # peak |force| per step across all 38 feet
    joint_q  = []     # actuated joint positions
    joint_qd = []     # actuated joint velocities
    joint_tau = []    # commanded torques

    # Identify "actuated" joints (body_yaw, pitch, leg) for work integration
    actuated_idx = []
    for jname in robot.joint_names:
        if (jname.startswith("joint_body_") or
                jname.startswith("joint_pitch_body_") or
                jname.startswith("joint_leg_")):
            actuated_idx.append(robot.joint_names.index(jname))
    actuated_idx = np.array(actuated_idx, dtype=int)

    buckled = False
    buckle_reason = ""

    for step in range(n_steps):
        t = step * dt
        robot.update(dt)
        controller.step(t, dt)
        sim.step(render=False)
        if contact_sensor is not None:
            try: contact_sensor.update(dt)
            except Exception: pass

        # Sample at rec_dt rate
        if step % rec_interval == 0:
            root_pos  = robot.data.root_pos_w[0].cpu().numpy()
            root_quat = robot.data.root_quat_w[0].cpu().numpy()
            w, x, y, z = root_quat
            sinr = 2.0 * (w*x + y*z)
            cosr = 1.0 - 2.0*(x*x + y*y)
            roll_deg = math.degrees(math.atan2(sinr, cosr))

            jpos = robot.data.joint_pos[0].cpu().numpy()
            jvel = robot.data.joint_vel[0].cpu().numpy()
            jtau = controller.torque_buf.copy()   # last commanded torques

            # Peak per-foot contact force magnitude
            peak_F_step = 0.0
            if contact_sensor is not None:
                try:
                    forces = contact_sensor.data.net_forces_w[0]   # (38, 3)
                    fmag = torch.linalg.norm(forces, dim=-1)
                    peak_F_step = float(fmag.max().item())
                except Exception:
                    pass

            times.append(t)
            com_xy.append((float(root_pos[0]), float(root_pos[1])))
            com_z.append(float(root_pos[2]))
            roll.append(roll_deg)
            foot_F.append(peak_F_step)
            joint_q.append(jpos[actuated_idx].copy())
            joint_qd.append(jvel[actuated_idx].copy())
            joint_tau.append(jtau[actuated_idx].copy())

        # Buckle detection (post-grace)
        if t >= buckle_grace_s:
            check = max(1, int(0.05 / dt))
            if step % check == 0 and step > 0:
                root_quat = robot.data.root_quat_w[0].cpu().numpy()
                w, x, y, z = root_quat
                sinr = 2.0 * (w*x + y*z)
                cosr = 1.0 - 2.0*(x*x + y*y)
                roll_now  = math.degrees(math.atan2(sinr, cosr))
                sinp = 2.0 * (w*y - z*x)
                pitch_now = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
                if abs(pitch_now) > MAX_ROOT_PITCH_DEG:
                    buckled = True
                    buckle_reason = f"root_pitch={pitch_now:.1f}@{t:.2f}s"
                    break
                if abs(roll_now) > MAX_ROOT_ROLL_DEG:
                    buckled = True
                    buckle_reason = f"root_roll={roll_now:.1f}@{t:.2f}s"
                    break

    return dict(
        buckled       = buckled,
        buckle_reason = buckle_reason,
        times         = np.asarray(times),
        com_z         = np.asarray(com_z),
        com_xy        = np.asarray(com_xy),
        roll_deg      = np.asarray(roll),
        peak_F_per_step = np.asarray(foot_F),
        joint_q       = np.asarray(joint_q),
        joint_qd      = np.asarray(joint_qd),
        joint_tau     = np.asarray(joint_tau),
    )


def compute_eval_cost(rec, freq, settle_s, args, body_mass_kg=0.0025,
                      gravity_z=9.81):
    """Constraint-based cost for one evaluation. IDENTICAL formula to
    optimize_impedance_biological.py:
        cost = CoT  if all hard constraints satisfied
        cost = CoT + 100·Σ(violation_excess) if any constraint violated
        cost = BUCKLED_COST if buckled
    Returns (cost, parts_dict).
    """
    if rec["buckled"]:
        return BUCKLED_COST, {"buckled": True, "reason": rec["buckle_reason"]}

    t    = rec["times"]
    if len(t) < 10:
        return BUCKLED_COST, {"buckled": True, "reason": "too few samples"}

    # Mask post-settle
    mask = t >= settle_s
    if mask.sum() < 5:
        return BUCKLED_COST, {"buckled": True, "reason": "too short post-settle"}

    com_xy   = rec["com_xy"][mask]
    com_z    = rec["com_z"][mask]
    roll_deg = rec["roll_deg"][mask]
    peak_F   = rec["peak_F_per_step"][mask]
    j_qd     = rec["joint_qd"][mask]
    j_tau    = rec["joint_tau"][mask]

    # Speed and displacement (post-settle)
    displacement_m = float(np.linalg.norm(com_xy[-1] - com_xy[0]))
    active_time    = max(float(t[mask][-1] - t[mask][0]), 1e-3)
    speed_mps      = displacement_m / active_time

    # Body z RMS / clearance ratio
    body_z_rms_m   = float(np.std(com_z))
    z_rms_ratio    = body_z_rms_m / max(args.nominal_clearance_m, 1e-9)

    # Body roll RMS (deg)
    body_roll_rms_deg = float(np.sqrt(np.mean(roll_deg ** 2)))

    # Peak per-foot force / body weight
    body_weight_N  = body_mass_kg * gravity_z
    peak_FW        = float(peak_F.max()) / max(body_weight_N, 1e-9) if len(peak_F) else 0.0

    # CoT = mechanical work / displacement
    if j_tau.shape == j_qd.shape and j_tau.size > 0:
        joint_power = np.abs(j_tau * j_qd)        # |τ·q̇| per joint per timestep
        dt_record   = max(float(np.median(np.diff(t))), 1e-4)
        total_work_J = float(np.sum(joint_power) * dt_record)
        cot          = total_work_J / max(displacement_m, 1e-3)
    else:
        total_work_J = 0.0
        cot          = 0.0

    # Commanded velocity at this eval's frequency.
    # `A_yaw` is pre-cached on `args` in main() so we don't re-open the YAML
    # 3× per trial (and to avoid Windows GBK-vs-UTF-8 codec issues on
    # comments containing → ≈ · etc.).
    A_yaw   = float(getattr(args, "_A_yaw_cached", 0.6))
    cmd_v   = 2.0 * args.leg_proj_length_m * math.sin(A_yaw) * float(freq)
    velocity_quality = speed_mps / max(cmd_v, 1e-9)

    # Hard constraint violations
    vel_excess  = max(0.0, args.min_velocity_quality - velocity_quality)
    z_excess    = max(0.0, z_rms_ratio - args.max_z_rms_ratio)
    roll_excess = max(0.0, body_roll_rms_deg - args.max_roll_deg)
    F_excess    = max(0.0, peak_FW - args.force_limit)
    total_violation = vel_excess + z_excess + roll_excess + F_excess

    if total_violation > 0:
        cost = cot + 100.0 * total_violation
    else:
        cost = cot

    parts = dict(
        buckled=False,
        cost=cost,
        cost_of_transport=cot,
        total_work_J=total_work_J,
        speed_mps=speed_mps,
        displacement_m=displacement_m,
        body_z_rms_m=body_z_rms_m,
        z_rms_ratio=z_rms_ratio,
        body_roll_rms_deg=body_roll_rms_deg,
        peak_force_over_weight=peak_FW,
        velocity_quality=velocity_quality,
        commanded_velocity_mps=cmd_v,
        vel_excess=vel_excess,
        z_excess=z_excess,
        roll_excess=roll_excess,
        F_excess=F_excess,
        total_violation=total_violation,
    )
    return float(cost), parts


# ═══════════════════════════════════════════════════════════════════════════════
# Apply trial parameters to controller
# ═══════════════════════════════════════════════════════════════════════════════

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
            if k > 0:   # skip head pitch (preserves head_pitch gains)
                controller.pitch_kp_vec[k] = kp
                controller.pitch_kv_vec[k] = kv


# ═══════════════════════════════════════════════════════════════════════════════
# Three-evaluation objective
# ═══════════════════════════════════════════════════════════════════════════════

def three_eval_objective(trial, args, robot, controller, sim, contact_sensor,
                          flat_offset, rough_offset, csv_writer, settle_s):
    """Run one Optuna trial through 3 evaluations. Returns mean cost."""
    # 1. Suggest gains (12-D, log-uniform within sim-stable bounds)
    params = dict(
        body_kp      = trial.suggest_float("body_kp",      1e-4, 0.5, log=True),
        body_kv      = trial.suggest_float("body_kv",      1e-6, 1.0, log=True),
        hip_yaw_kp   = trial.suggest_float("hip_yaw_kp",   1e-4, 0.5, log=True),
        hip_yaw_kv   = trial.suggest_float("hip_yaw_kv",   1e-7, 5e-1, log=True),
        hip_pitch_kp = trial.suggest_float("hip_pitch_kp", 1e-5, 0.5, log=True),
        hip_pitch_kv = trial.suggest_float("hip_pitch_kv", 1e-8, 5e-2, log=True),
    )
    if args.tune_distal:
        params["tibia_kp"]  = trial.suggest_float("tibia_kp",  1e-3, 0.5,  log=True)
        params["tibia_kv"]  = trial.suggest_float("tibia_kv",  1e-7, 5e-2, log=True)
        params["tarsus_kp"] = trial.suggest_float("tarsus_kp", 1e-3, 0.5,  log=True)
        params["tarsus_kv"] = trial.suggest_float("tarsus_kv", 1e-6, 5e-2, log=True)
    if args.tune_pitch:
        params["pitch_kp"] = trial.suggest_float("pitch_kp", 1e-4, 0.5, log=True)
        params["pitch_kv"] = trial.suggest_float("pitch_kv", 1e-6, 5e-1, log=True)

    # 2. Apply gains once (frequency switches happen per-eval below)
    apply_params(controller, params)

    # 3. Three evaluations
    eval_specs = [
        ("flat_2hz",    flat_offset,  args.freq_flat),
        ("rough_1hz",   rough_offset, args.freq_rough_low),
        ("rough_2hz",   rough_offset, args.freq_rough_high),
    ]
    eval_costs = []
    eval_parts = []
    t0_trial = time.time()

    for label, offset, freq in eval_specs:
        # 3a. Update controller frequency (resets CPG, target history, heading)
        controller.set_frequency(freq)
        # 3b. Teleport robot, settle
        reset_robot_to(robot, controller, sim, contact_sensor, offset)
        # 3c. Run for duration
        rec = run_one_eval(robot, controller, sim, contact_sensor, args.duration)
        # 3d. Score
        cost, parts = compute_eval_cost(rec, freq, settle_s, args)
        parts["eval_label"] = label
        parts["frequency"]  = freq
        eval_costs.append(cost)
        eval_parts.append(parts)
        # 3e. Early-out if any eval flips
        if cost >= BUCKLED_COST * 0.99:
            # Fill out remaining evals with BUCKLED so the CSV has consistent columns
            for remaining in eval_specs[len(eval_costs):]:
                eval_costs.append(BUCKLED_COST)
                eval_parts.append({"eval_label": remaining[0],
                                   "frequency": remaining[2],
                                   "buckled": True, "cost": BUCKLED_COST,
                                   "reason": parts.get("reason", "abort_propagated")})
            break

    trial_cost = float(np.mean(eval_costs))
    wall = time.time() - t0_trial

    # Write CSV row
    csv_row = dict(trial=trial.number, cost=trial_cost, wall_s=wall)
    for k, v in params.items():
        csv_row[k] = v
    for parts in eval_parts:
        lbl = parts.get("eval_label", "?")
        for k, v in parts.items():
            if k in ("eval_label",):
                continue
            csv_row[f"{lbl}__{k}"] = v
    csv_writer(csv_row)

    # Console line
    cot_flat = eval_parts[0].get("cost_of_transport", float("nan"))
    cot_r1   = eval_parts[1].get("cost_of_transport", float("nan"))
    cot_r2   = eval_parts[2].get("cost_of_transport", float("nan"))
    speeds   = [eval_parts[i].get("speed_mps", 0.0) * 1000 for i in range(3)]
    n_buck   = sum(1 for p in eval_parts if p.get("buckled", False))
    if n_buck > 0:
        print(f"  [trial {trial.number:4d}] BUCKLE in {n_buck}/3 evals  "
              f"cost_mean={trial_cost:.2f}  ({wall:.1f}s)", flush=True)
    else:
        print(f"  [trial {trial.number:4d}] cost={trial_cost:6.3f}  "
              f"CoT(flat2/r1/r2)=({cot_flat:.2f},{cot_r1:.2f},{cot_r2:.2f})  "
              f"speed(mm/s)=({speeds[0]:.1f},{speeds[1]:.1f},{speeds[2]:.1f})  "
              f"({wall:.1f}s)", flush=True)

    return trial_cost


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    if args.resume:
        storage_path = args.resume
        run_dir = os.path.dirname(os.path.abspath(storage_path))
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(OUTPUT_ROOT, f"impedance_three_eval_{args.run_tag}_{ts}")
        os.makedirs(run_dir, exist_ok=True)
        storage_path = os.path.join(run_dir, "study.db")

    csv_path = os.path.join(run_dir, "all_trials.csv")
    print(f"\n[3eval] run dir: {run_dir}", flush=True)
    print(f"[3eval] eval A: flat ground, freq={args.freq_flat:.1f} Hz "
          f"(CoT bio reference)", flush=True)
    print(f"[3eval] eval B: rough λ={args.terrain_wavelength_mm:.0f}mm, "
          f"freq={args.freq_rough_low:.1f} Hz", flush=True)
    print(f"[3eval] eval C: rough λ={args.terrain_wavelength_mm:.0f}mm, "
          f"freq={args.freq_rough_high:.1f} Hz", flush=True)
    print(f"[3eval] duration per eval: {args.duration:.1f} s   "
          f"→ trial total ≈ {3*args.duration:.0f} s sim", flush=True)

    # Build sim, scene, controller, recorder
    with open(CFG_PATH, "r", encoding="utf-8") as f:
        base_cfg = yaml.safe_load(f)
    # Cache A_yaw (hip-yaw amplitude in rad) for compute_eval_cost so it
    # never has to re-open the YAML mid-trial. Avoids Windows GBK codec
    # crashes on UTF-8 comments (→, ≈, ·, etc.).
    args._A_yaw_cached = float(
        base_cfg.get("leg_wave", {}).get("amplitudes", [0.6])[0])
    print(f"[3eval] cached A_yaw = {args._A_yaw_cached:.3f} rad "
          f"(hip-yaw amplitude from YAML)", flush=True)
    dt = float(base_cfg.get("simulation", {}).get("timestep", 0.004))
    sim_cfg = SimulationCfg(dt=dt)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[1.0, -1.0, 0.5], target=[0.0, -1.0, 0.0])

    robot, rough_h, flat_offset, rough_offset, contact_sensor = build_scene(
        sim, run_dir, args)
    sim.reset()
    robot.reset()

    print(f"[3eval] articulation loaded: {robot.num_joints} joints, "
          f"{robot.num_bodies} bodies", flush=True)

    controller = ImpedanceTravelingWaveController(robot, CFG_PATH)
    settle_s   = controller.settle_time + controller.ramp_time + 0.5

    # CSV writer
    csv_rows = []
    def csv_writer(row):
        csv_rows.append(dict(row))
        if len(csv_rows) == 1:
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(row.keys()))
                w.writeheader()
                w.writerow(row)
        else:
            # Append. Use a SUPERSET of fieldnames so columns stay aligned.
            existing_fields = list(csv_rows[0].keys())
            for k in row:
                if k not in existing_fields:
                    existing_fields.append(k)
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=existing_fields,
                                    extrasaction="ignore")
                w.writeheader()
                for r in csv_rows:
                    w.writerow(r)

    # Optuna study
    storage = f"sqlite:///{storage_path}"
    sampler = TPESampler(seed=args.seed, multivariate=True, constant_liar=True)
    study = optuna.create_study(
        study_name=f"impedance_three_eval",
        storage=storage,
        sampler=sampler,
        direction="minimize",
        load_if_exists=True,
    )

    # Seed first trial from current YAML (or --seed-from)
    if not args.resume and len(study.trials) == 0:
        seed_yaml = args.seed_from or CFG_PATH
        with open(seed_yaml, "r", encoding="utf-8") as f:
            seed_cfg = yaml.safe_load(f)
        imp = seed_cfg.get("impedance", {})
        leg = imp.get("leg", {})
        leg_kp = leg.get("kp", controller.leg_kp.tolist())
        leg_kv = leg.get("kv", controller.leg_kv.tolist())
        seed_params = dict(
            body_kp      = float(imp.get("body_kp", controller.body_kp)),
            body_kv      = float(imp.get("body_kv", controller.body_kv)),
            hip_yaw_kp   = float(leg_kp[0]),
            hip_yaw_kv   = float(leg_kv[0]),
            hip_pitch_kp = float(leg_kp[1]),
            hip_pitch_kv = float(leg_kv[1]),
        )
        if args.tune_distal:
            seed_params["tibia_kp"]  = float(leg_kp[2])
            seed_params["tibia_kv"]  = float(leg_kv[2])
            seed_params["tarsus_kp"] = float(leg_kp[3])
            seed_params["tarsus_kv"] = float(leg_kv[3])
        if args.tune_pitch:
            seed_params["pitch_kp"] = float(imp.get("pitch_kp", controller.pitch_kp))
            seed_params["pitch_kv"] = float(imp.get("pitch_kv", controller.pitch_kv))

        # Clip to bounds
        bounds = dict(
            body_kp=(1e-4,0.5), body_kv=(1e-6,1.0),
            hip_yaw_kp=(1e-4,0.5), hip_yaw_kv=(1e-7,5e-1),
            hip_pitch_kp=(1e-5,0.5), hip_pitch_kv=(1e-8,5e-2),
            tibia_kp=(1e-3,0.5), tibia_kv=(1e-7,5e-2),
            tarsus_kp=(1e-3,0.5), tarsus_kv=(1e-6,5e-2),
            pitch_kp=(1e-4,0.5), pitch_kv=(1e-6,5e-1),
        )
        for k in list(seed_params.keys()):
            if k in bounds:
                lo, hi = bounds[k]
                seed_params[k] = float(np.clip(seed_params[k], lo, hi))
        print(f"[3eval] seeding first trial from {seed_yaml}", flush=True)
        for k, v in seed_params.items():
            print(f"        {k:14s}= {v:.6g}", flush=True)
        study.enqueue_trial(seed_params)

    # Pretest probes (4 edge probes after the seed)
    if args.pretest and not args.resume:
        bounds_kp = (1e-4, 0.5)
        for label, scale_kp, scale_kv in [
            ("SOFTEST",   0.1, 0.1),
            ("STIFFEST", 10.0, 10.0),
            ("OVERDAMPED", 1.0, 10.0),
            ("UNDAMPED",   1.0,  0.1),
        ]:
            p = dict()
            for k, v in seed_params.items():
                if k.endswith("_kp"):
                    p[k] = float(np.clip(v * scale_kp, *bounds_kp))
                elif k.endswith("_kv"):
                    p[k] = max(1e-9, v * scale_kv)
                else:
                    p[k] = v
            study.enqueue_trial(p)
            print(f"[3eval] pretest probe '{label}': body_kp={p.get('body_kp',0):.4f} "
                  f"body_kv={p.get('body_kv',0):.6f}", flush=True)

    # Run optimization
    t0 = time.time()
    try:
        study.optimize(
            lambda trial: three_eval_objective(
                trial, args, robot, controller, sim, contact_sensor,
                flat_offset, rough_offset, csv_writer, settle_s),
            n_trials=args.n_trials,
            show_progress_bar=False,
        )
    except KeyboardInterrupt:
        print("\n[3eval] interrupted — saving partial results", flush=True)

    elapsed_min = (time.time() - t0) / 60.0
    n_done = len(study.trials)

    # Save best params
    best = None
    try:
        best = study.best_trial
    except Exception:
        pass
    if best is not None:
        print(f"\n[3eval] best trial #{best.number}  cost={best.value:.4f}",
              flush=True)
        for k, v in best.params.items():
            print(f"        {k:14s}= {v:.6g}", flush=True)
        # Write best_params.yaml in the same format the bio optimizer uses.
        # encoding="utf-8" required because Windows defaults to GBK locale,
        # which can't decode the → ≈ · characters in YAML comments.
        with open(CFG_PATH, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        imp = cfg.setdefault("impedance", {})
        imp["body_kp"] = best.params["body_kp"]
        imp["body_kv"] = best.params["body_kv"]
        if "pitch_kp" in best.params:
            imp["pitch_kp"] = best.params["pitch_kp"]
            imp["pitch_kv"] = best.params["pitch_kv"]
        leg = imp.setdefault("leg", {})
        kp = list(leg.get("kp", [0.5, 0.06, 0.13, 0.13]))
        kv = list(leg.get("kv", [0.001, 0.003, 0.0005, 0.001]))
        kp[0] = best.params["hip_yaw_kp"];   kv[0] = best.params["hip_yaw_kv"]
        kp[1] = best.params["hip_pitch_kp"]; kv[1] = best.params["hip_pitch_kv"]
        if "tibia_kp" in best.params:
            kp[2] = best.params["tibia_kp"];  kv[2] = best.params["tibia_kv"]
        if "tarsus_kp" in best.params:
            kp[3] = best.params["tarsus_kp"]; kv[3] = best.params["tarsus_kv"]
        leg["kp"] = kp; leg["kv"] = kv
        with open(os.path.join(run_dir, "best_params.yaml"), "w",
                   encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
        with open(os.path.join(run_dir, "best_params.json"), "w",
                   encoding="utf-8") as f:
            json.dump({"trial": best.number, "cost": best.value,
                        "params": best.params}, f, indent=2)

    print(f"[3eval] done — {n_done} trials in {elapsed_min:.1f} min", flush=True)
    sim_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
