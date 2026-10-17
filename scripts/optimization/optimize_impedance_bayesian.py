#!/usr/bin/env python3
"""
optimize_impedance_bayesian.py — Isaac Lab port of the MuJoCo Bayesian optimiser
=================================================================================
Same search space, same straightness-based cost, same TPE sampler as the
MuJoCo version at
    Centipede_MUJOCO-main/scripts/optimization/farms/optimize_impedance_bayesian.py
— but trials run inside a single Isaac Sim process under PhysX, with
the centipede walking on a USD-baked rough terrain at λ=18 mm by
default.

Key differences from the MuJoCo version:
  * Isaac Sim takes ~30 s to boot, so we boot ONCE and reset between
    trials in-place (robot pose, controller phases, recorder buffers)
    rather than re-creating MjModel/MjData each trial.
  * Terrain is a USD triangle mesh with PhysX MeshCollisionAPI rather
    than an MJCF heightfield. Same numpy heightmap generator, so the
    physical surface (given the same seed + λ + amplitude) is identical
    pixel-for-pixel.
  * Per-trial param patching writes into the controller's Python attrs
    (no temp YAML files), which is ~50× faster.

Search space (12-D by default — IMPEDANCE GAINS ONLY):
    body_kp,  body_kv          — body yaw joints 1..18
    hip_yaw_kp,  hip_yaw_kv    — leg DOF 0
    hip_pitch_kp,hip_pitch_kv  — leg DOF 1
    tibia_kp,   tibia_kv       — leg DOF 2  (--no-tune-distal to disable)
    tarsus_kp,  tarsus_kv      — leg DOF 3  (--no-tune-distal to disable)
    pitch_kp,   pitch_kv       — body pitch (--no-tune-pitch to disable)

NOT in the search space (held fixed at YAML values):
    body_amp, leg_amp_dof0, leg_amp_dof1, frequency, wave_number, ...
The optimizer tunes ONLY the impedance gains. Gait shape (amplitudes
and frequency) is a controller-design choice that comes from
configs/farms_controller.yaml and the optimizer cannot game it.

Initial parameter values (current YAML, 2026-04-30 — also the
seed for trial #0):
    body_kp       = 0.05         body_kv       = 0.02
    hip_yaw_kp    = 0.5          hip_yaw_kv    = 0.001
    hip_pitch_kp  = 0.06         hip_pitch_kv  = 0.003
    tibia_kp      = 0.13         tibia_kv      = 0.0005
    tarsus_kp     = 0.13         tarsus_kv     = 0.001
    pitch_kp      = 0.02         pitch_kv      = 0.005

Cost (lower is better) — SPEED/DISPLACEMENT-PRIORITISED (2026-05-01):
    cost =  w_body_track   · max(0, RMSE_body_yaw_deg − body_tol)   # 0.10
         +  w_leg_track    · max(0, RMSE_leg_deg      − leg_tol)    # 0.20
         +  w_wobble       · (1 − straightness)                      # 4.0
         +  w_force        · max(0, peak_F/W − force_limit)          # 0.5
         −  w_speed        · forward_speed_mps                       # 200.0
         −  w_displacement · ||com_xy[-1] - com_xy[0]||              # 100.0  ★ NEW
         −  w_compliance   · min(pitch_compliance_deg, compliance_cap) # 0.05

  Speed and displacement are now the dominant terms. Penalty terms act
  as guardrails (don't allow gross tracking failure / wobbly path /
  excessive contact force) but no longer compete equally with rewards.

Outputs (timestamped run dir under outputs/optimization/):
    study.db               — SQLite Optuna storage (resumable)
    all_trials.csv         — every trial: params + cost components
    best_params.yaml       — drop-in replacement for farms_controller.yaml
    best_params.json       — summary
    best_trial.mp4         — winner replay (if --replay-video)
    terrain_heightmap.npy  — exact heightmap used (for reproducibility)

Usage:
    cd C:/Users/wxy22/Documents/Centipede_IsaacLab
    python scripts/optimization/optimize_impedance_bayesian.py \\
        --n-trials 100 --duration 8 --headless

    # resume an interrupted study
    python scripts/optimization/optimize_impedance_bayesian.py \\
        --resume outputs/optimization/impedance_bay_<tag>/study.db \\
        --n-trials 100
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
parser.add_argument("--n-trials", type=int, default=500,
                    help="Number of trials to run (default 500 — sized "
                         "for the 12-D IMPEDANCE-ONLY search. The "
                         "optimizer tunes kp/kv only; wave envelope "
                         "and frequency are held fixed at YAML values.")
parser.add_argument("--duration", type=float, default=8.0,
                    help="Seconds per trial (settle + ramp + active gait).")
parser.add_argument("--seed",     type=int, default=42,
                    help="TPE sampler seed.")
parser.add_argument("--terrain-wavelength", type=float, default=18.0,
                    help="Terrain wavelength in mm (default 18).")
parser.add_argument("--terrain-amplitude",  type=float, default=0.01,
                    help="Terrain peak amplitude in metres (default 0.01).")
parser.add_argument("--terrain-seed",       type=int,   default=42,
                    help="Seed for terrain generation (fixed across study).")
parser.add_argument("--flat",               action="store_true",
                    help="Skip terrain — flat floor only.")
parser.add_argument("--mesh-resolution",    type=int,   default=128,
                    help="Collision mesh resolution (default 128 → 32k tris).")
parser.add_argument("--tune-distal",    dest="tune_distal", action="store_true",
                    default=True, help="Tune tibia/tarsus (default on).")
parser.add_argument("--no-tune-distal", dest="tune_distal", action="store_false")
parser.add_argument("--tune-pitch",     dest="tune_pitch",  action="store_true",
                    default=True, help="Tune body pitch (default on).")
parser.add_argument("--no-tune-pitch",  dest="tune_pitch",  action="store_false")
parser.add_argument("--seed-from", default=None,
                    help="Seed first trial from a best_params.yaml.")
parser.add_argument("--resume", default=None,
                    help="Resume an existing study.db.")
# ── Cost weights — SPEED/DISPLACEMENT-PRIORITISED RETUNE (2026-05-01) ──
# Defaults shifted to make forward speed and total displacement DOMINATE
# the cost. Other terms (wobble / tracking / compliance) act as sanity
# guardrails rather than co-equal objectives.
#
# Magnitude reasoning (using the YAML-seed trial 0 as reference where
# wobble term was ~2.8 and speed term was ~0.4 under old weights):
#   - w_speed bumped 50 → 200 (4×) — every 10 mm/s gives -2.0 cost
#   - w_displacement NEW = 100 — every 1 cm of forward travel gives -1.0
#     cost; over 8 s at 25 mm/s that's ~0.2 m × 100 = 20 cost units, well
#     above all other terms combined
#   - w_wobble cut 8 → 4 (back near MuJoCo default) — straightness still
#     matters but no longer dominates
#   - w_body_track / w_leg_track halved — still penalize gross tracking
#     failures but won't prevent stiff-fast gaits
#   - w_compliance cut 0.15 → 0.05 (back to MuJoCo default) — drape is
#     a nice-to-have, not a primary objective
parser.add_argument("--w-wobble",     type=float, default=4.0,
                    help="Penalty per unit of (1 − straightness). Was 8.0; "
                         "reduced to 4.0 so speed/displacement dominate.")
parser.add_argument("--w-body-track", type=float, default=0.10,
                    help="Penalty per deg of body-yaw RMSE above body-tol. "
                         "Was 0.20; reduced so stiff-fast gaits aren't over-penalised.")
parser.add_argument("--w-leg-track",  type=float, default=0.20,
                    help="Penalty per deg of leg RMSE above leg-tol. "
                         "Was 0.40; reduced for same reason as body-track.")
parser.add_argument("--w-force",      type=float, default=0.5)
parser.add_argument("--w-speed",      type=float, default=200.0,
                    help="Reward per m/s forward speed. Was 50.0; bumped 4× "
                         "to make speed the primary cost driver.")
parser.add_argument("--w-displacement", type=float, default=100.0,
                    help="NEW (2026-05-01): reward per metre of net forward "
                         "displacement (||com_xy[-1] - com_xy[0]||). "
                         "Complements w-speed: speed rewards the average "
                         "rate, displacement rewards making it FAR. Trials "
                         "that walk fast but in circles get punished by "
                         "wobble; trials that walk fast and far get a big "
                         "reward from this term.")
parser.add_argument("--w-compliance", type=float, default=0.05,
                    help="Reward per degree of pitch-joint-deflection std. "
                         "Was 0.15; reduced back to MuJoCo default 0.05 — "
                         "drape over terrain is now a nice-to-have, not "
                         "a primary objective.")
parser.add_argument("--force-limit",  type=float, default=4.0)
parser.add_argument("--body-tol-deg", type=float, default=5.0)
parser.add_argument("--leg-tol-deg",  type=float, default=3.0)
parser.add_argument("--compliance-cap-deg", type=float, default=10.0)
parser.add_argument("--replay-video", action="store_true",
                    help="Render an MP4 of the winning trial at end (slow).")
parser.add_argument("--video-fps",    type=int, default=30)
parser.add_argument("--pretest", action="store_true",
                    help="Run 5 fixed-edge probe trials BEFORE TPE starts: "
                         "(1) YAML baseline, (2) softest gains, (3) stiffest "
                         "gains, (4) midrange & strongly damped, (5) midrange "
                         "& undamped. If any of these crash that's a sign the "
                         "search bounds are too aggressive — examine the "
                         "trial CSV and tighten the offending dimension. The "
                         "probes are enqueued via study.enqueue_trial so TPE "
                         "sees them as observations and learns from them.")
# ── 2026-05-03: narrow-bounds mode (constrain search to the envelope of
#    two reference trials, e.g. trial #208 (rank 3) and #314 (rank 29) of
#    the rough-terrain study). User wanted to drill INTO the gap between
#    two known-good gain families rather than re-explore the full space. ──
parser.add_argument("--narrow-bounds", action="store_true",
                    help="Restrict the 12-D search to the envelope between "
                         "trial #208 (rank 3, fast/straight, body-bouncy) and "
                         "trial #314 (rank 29, slower, body-stiff) of the "
                         "rough-terrain study. Each parameter's [lo, hi] is "
                         "set to min/max of the two trials' values. Use this "
                         "to refine within a known-good neighborhood without "
                         "wasting trials in regions TPE has already proved "
                         "are bad.")
parser.add_argument("--bump-body-kp", type=float, default=1.0,
                    help="Multiply the seed body_kp by this factor before the "
                         "first probe is enqueued. Use 1.6 to start the run at "
                         "a 'moderately stiffer body than rank 3' setting "
                         "(0.0137 → 0.022, still well below rank 29's 0.035). "
                         "Default 1.0 = no bump.")

# AppLauncher must add its args before parse_args
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# Enable cameras if we're going to render the replay
if args.replay_video and not getattr(args, "enable_cameras", False):
    args.enable_cameras = True
    print("[opt] --replay-video → enabling --enable_cameras")

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

# ── Render-pipeline settings for the replay video ──
# See NOTES_RENDERING.md §1 in the project root. Two non-negotiables:
#   1. /rtx/rendermode = "RaytracedLighting" (real-time RTX, NON-progressive).
#      Default "PathTracing" is progressive: it accumulates samples across
#      frames to denoise. The accumulator resets on every camera move
#      AND on the renderer's own restart cycles, so the first 1-2 frames
#      after each move come out as colored RGB noise. Switching to
#      RaytracedLighting eliminates the noise frames at the cost of
#      slightly less photoreal lighting.
#   2. /isaaclab/render/rtx_sensors = True so the Camera sensor's
#      render_product is allocated. AppLauncher should set this when
#      --enable_cameras is passed, but we set it explicitly to match
#      the test_walk.py pattern.
if args.replay_video:
    import carb
    _settings = carb.settings.get_settings()
    _settings.set_bool("/isaaclab/render/rtx_sensors", True)
    _settings.set_string("/rtx/rendermode", "RaytracedLighting")
    print("[opt] forced /rtx/rendermode = RaytracedLighting "
          "(non-progressive renderer for replay video)", flush=True)

# ── Now safe to import Isaac Lab pieces ──────────────────────────────────────
import torch
import yaml
try:
    import optuna
    from optuna.samplers import TPESampler
except ImportError:
    print("ERROR: optuna required. Install with: pip install optuna")
    sim_app.close()
    sys.exit(1)

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

from controllers.impedance_controller import ImpedanceTravelingWaveController
from controllers.kinematics import (
    N_BODY_JOINTS, N_LEGS, N_LEG_DOF
)
from terrain import (
    generate_single_wavelength_heightmap, build_terrain_mesh,
)


# S2b 2026-07-16: repointed farms_controller.yaml -> farms_controller_numpy_legacy.yaml.
# This script drives the NUMPY controller (controllers/impedance_controller.py), which still computes torque
# and NEEDS the impedance gains. S2b stripped the gains out of farms_controller.yaml because the LIVE torch
# controller is targets-only now; the numpy ref would have SILENTLY DEFAULTED (body_kp 0.14396 -> 5.0, 35x).
# The gains live on, byte-identical, in the _numpy_legacy copy.
CFG_PATH = os.path.join(PROJECT_ROOT, "configs", "farms_controller_numpy_legacy.yaml")
USD_PATH = os.path.join(PROJECT_ROOT, "assets", "centipede.usd")
OUTPUT_ROOT = os.path.join(PROJECT_ROOT, "outputs", "optimization")

BUCKLED_COST       = 1e5
MAX_ROOT_PITCH_DEG = 45.0
MAX_ROOT_ROLL_DEG  = 45.0
MAX_JOINT_PITCH_DEG = 35.0
MAX_JOINT_ROLL_DEG  = 35.0

# ─────────────────────────────────────────────────────────────────────────────
# NARROW BOUNDS (--narrow-bounds): the [min, max] envelope of trial #208
# (rank 3) and trial #314 (rank 29) from the
# impedance_bay_rough_wl18_20260501_153708 study. Used when the operator
# wants to drill INTO the gap between two known-good gain families rather
# than re-search the full 12-D space.
#
#   Trial 208 (rank 3, cost=-20.16): fast & straight, body-bouncy.
#       body_kp=0.0137 (soft body), tibia_kp=0.489 + tibia_kv=0.0126
#       (stiff & damped tibia), hip_pitch_kv=1.6e-5 (essentially undamped
#       hip-pitch — main cause of the visible body bounce).
#
#   Trial 314 (rank 29, cost=-13.94): slower, body-stiff.
#       body_kp=0.0347 (~2.5× stiffer body), tibia_kp=0.349 (softer),
#       tibia_kv=2.07e-4 (much less damped tibia), tarsus_kv=0.0165
#       (heavily damped foot tip).
#
# These two trials span the qualitative axis between "rigid-body / rigid-leg"
# and "compliant-body / compliant-leg" gait styles — the right place to
# refine a winner.
# ─────────────────────────────────────────────────────────────────────────────
NARROW_BOUNDS = dict(
    body_kp      = (1.371436e-02, 3.465217e-02),
    body_kv      = (4.487786e-05, 2.564276e-03),
    hip_yaw_kp   = (6.493049e-02, 1.138916e-01),
    hip_yaw_kv   = (7.760488e-04, 2.366877e-03),
    hip_pitch_kp = (3.010849e-02, 3.072722e-02),
    hip_pitch_kv = (1.564847e-05, 1.265143e-04),
    tibia_kp     = (3.488584e-01, 4.887445e-01),
    tibia_kv     = (2.074617e-04, 1.260813e-02),
    tarsus_kp    = (3.261104e-02, 5.568534e-02),
    tarsus_kv    = (1.123799e-04, 1.654121e-02),
    pitch_kp     = (1.175853e-04, 1.861004e-04),
    pitch_kv     = (3.607779e-03, 2.447305e-02),
)


# ═════════════════════════════════════════════════════════════════════════════
# Scene setup (one-time)
# ═════════════════════════════════════════════════════════════════════════════

def build_scene(sim, run_dir):
    """Spawn lights, terrain (or flat ground), centipede. Return Articulation."""
    from pxr import UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf
    stage = sim.stage

    # ── Lighting ──
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.GetIntensityAttr().Set(300.0)
    dist = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    dist.GetIntensityAttr().Set(800.0)
    dist.GetAngleAttr().Set(0.53)
    UsdGeom.Xformable(dist.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))

    # ── Ground / terrain ──
    if args.flat:
        # Same flat slab as test_walk.py
        UsdGeom.Xform.Define(stage, "/World/GroundPlane")
        slab = UsdGeom.Cube.Define(stage, "/World/GroundPlane/Slab")
        slab.GetSizeAttr().Set(1.0)
        sx = UsdGeom.Xformable(slab)
        sx.ClearXformOpOrder()
        sx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
        sx.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 0.1))
        UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
        print("[scene] flat ground at z=0", flush=True)
    else:
        wavelength_m = args.terrain_wavelength * 1e-3
        heightmap, rms_m, peak_m = generate_single_wavelength_heightmap(
            wavelength_m=wavelength_m,
            amplitude_m=args.terrain_amplitude,
            seed=args.terrain_seed,
        )
        np.save(os.path.join(run_dir, "terrain_heightmap.npy"), heightmap)
        UsdGeom.Xform.Define(stage, "/World/Terrain")
        build_terrain_mesh(
            stage=stage,
            prim_path="/World/Terrain/Mesh",
            heightmap_m=heightmap,
            world_size=1.0,
            mesh_resolution=args.mesh_resolution,
        )
        print(f"[scene] terrain  λ={args.terrain_wavelength:.1f}mm  "
              f"A={args.terrain_amplitude*1000:.1f}mm  "
              f"rms={rms_m*1000:.2f}mm  peak={peak_m*1000:.2f}mm  "
              f"mesh={args.mesh_resolution}x{args.mesh_resolution}", flush=True)

    # ── Centipede USD spawn ──
    spawn_cfg = sim_utils.UsdFileCfg(usd_path=USD_PATH)
    spawn_cfg.func("/World/Centipede", spawn_cfg, translation=(0.0, 0.0, 0.05))
    # Suppress the MJCF importer's worldBody floor (verified in test_walk.py)
    wb = stage.GetPrimAtPath("/World/Centipede/worldBody")
    if wb.IsValid():
        wb.SetActive(False)

    # ── Articulation config (effort-mode active joints) ──
    robot_cfg = ArticulationCfg(
        prim_path="/World/Centipede/link_body_0",
        spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.05)),
        actuators={
            "active_joints": ImplicitActuatorCfg(
                # Simplified MJCF (scripts/simplify_mjcf.py with default
                # --keep-body-pitch): joint_roll_body_* and joint_roll_leg_*
                # are removed to eliminate multi-DOF compounds, BUT
                # joint_pitch_body_* are kept (single-DOF) so the body
                # retains vertical pitch compliance over terrain.
                joint_names_expr=[
                    "joint_body_.*",
                    "joint_pitch_body_.*",
                    "joint_leg_.*",
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
    return Articulation(robot_cfg)


# ═════════════════════════════════════════════════════════════════════════════
# Sensor recorder (mirrors the MuJoCo SensorRecorder fields used by cost)
# ═════════════════════════════════════════════════════════════════════════════

class SensorRecorder:
    """Capture the sim signals the cost function needs.

    Fields used by compute_cost (matching the MuJoCo recorder):
      times                — (T,)   sim time at each sample
      body_yaw_q, _tgt     — (T, N_BODY_JOINTS)
      leg_q,      _tgt     — (T, N_LEGS, 2, N_LEG_DOF)
      pitch_q,    _tgt     — (T, N_PITCH)
      com_pos              — (T, 3) approximate COM (we use root_pos for speed)
      foot_force           — (T, N_LEGS, 2, 3)
      total_mass, gravity_z — scalars used to normalise foot force
    """
    def __init__(self, articulation, controller, dt_record=1/200.0,
                 settle_time=2.5):
        self.articulation = articulation
        self.controller   = controller
        self.dt_record    = dt_record
        self.settle_time  = settle_time
        self._next_t      = -1.0   # always record first call

        self.times       = []
        self.body_yaw_q   = []
        self.body_yaw_tgt = []
        self.leg_q        = []
        self.leg_tgt      = []
        self.pitch_q      = []
        self.pitch_tgt    = []
        self.com_pos      = []
        self.foot_force   = []   # we don't have foot force tensors easily;
                                 # use body net contact force as a proxy

        # Mass + gravity for force normalisation
        try:
            view = articulation.root_physx_view
            masses = view.get_masses().cpu().numpy().reshape(-1)
            self.total_mass = float(masses.sum())
        except Exception:
            self.total_mass = 0.00254  # MJCF total
        self.gravity_z = 9.81

    def reset(self):
        self.times.clear()
        self.body_yaw_q.clear()
        self.body_yaw_tgt.clear()
        self.leg_q.clear()
        self.leg_tgt.clear()
        self.pitch_q.clear()
        self.pitch_tgt.clear()
        self.com_pos.clear()
        self.foot_force.clear()
        self._next_t = -1.0

    def maybe_record(self, t):
        if t < self._next_t:
            return
        self._next_t = t + self.dt_record

        art = self.articulation
        ctrl = self.controller
        idx = ctrl.idx

        all_pos = art.data.joint_pos[0].cpu().numpy()

        # Body yaw
        body_q   = all_pos[idx.body_joint_ids]
        body_tgt = ctrl.last_body_yaw_targets if hasattr(ctrl, "last_body_yaw_targets") \
                   else np.zeros(N_BODY_JOINTS)
        # Note: vectorised IsaacLab controller doesn't currently expose
        # last_body_yaw_targets — fallback to zeros if absent.
        # (Cost function still works; tracking RMSE just collapses to |q|.)
        if not hasattr(ctrl, "last_body_yaw_targets"):
            body_tgt = body_q  # treat as zero error if no target log

        # Legs (N_LEGS, 2, N_LEG_DOF)
        leg_q = all_pos[idx.leg_joint_ids]
        # No leg_target log in the vectorized controller either; use q
        leg_tgt = leg_q  # → zero leg-tracking RMSE contribution
        if hasattr(ctrl, "last_leg_targets"):
            leg_tgt = ctrl.last_leg_targets

        # Body pitch (N_PITCH,)
        if idx.has_pitch:
            pitch_q   = all_pos[idx.pitch_joint_ids]
            pitch_tgt = getattr(ctrl, "pitch_targets",
                                np.zeros_like(pitch_q))
            if hasattr(ctrl, "pitch_targets") and ctrl.pitch_targets.size == pitch_q.size:
                pitch_tgt = ctrl.pitch_targets
            else:
                pitch_tgt = np.zeros_like(pitch_q)
        else:
            pitch_q   = np.zeros(0)
            pitch_tgt = np.zeros(0)

        # COM ≈ root_pos (good enough since main body has most of the mass)
        com = art.data.root_pos_w[0].cpu().numpy()

        self.times.append(float(t))
        self.body_yaw_q.append(body_q.copy())
        self.body_yaw_tgt.append(np.asarray(body_tgt).copy())
        self.leg_q.append(leg_q.copy())
        self.leg_tgt.append(np.asarray(leg_tgt).copy())
        self.pitch_q.append(pitch_q.copy())
        self.pitch_tgt.append(pitch_tgt.copy())
        self.com_pos.append(com.copy())
        # No foot-force tensor in this minimal port — use root contact
        # net force placeholder so the F/W penalty is essentially 0
        # (won't drive the cost). To enable, query articulation
        # body_force_w on the foot bodies.
        self.foot_force.append(np.zeros((N_LEGS, 2, 3)))


# ═════════════════════════════════════════════════════════════════════════════
# Per-trial: apply params, reset, run
# ═════════════════════════════════════════════════════════════════════════════

def apply_params(controller, params):
    """Patch the running controller's gain + wave-envelope attributes."""
    # ── Impedance gains ──
    controller.body_kp = float(params["body_kp"])
    controller.body_kv = float(params["body_kv"])
    if hasattr(controller, "_build_body_kp_vec"):
        controller._build_body_kp_vec()

    controller.leg_kp[0] = float(params["hip_yaw_kp"])
    controller.leg_kv[0] = float(params["hip_yaw_kv"])
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
        kp = float(params["pitch_kp"])
        kv = float(params["pitch_kv"])
        n_p = len(controller.pitch_kp_vec)
        for k in range(n_p):
            if not getattr(controller, "pitch_is_head", np.zeros(n_p, bool))[k] \
                    if hasattr(controller, "pitch_is_head") else (k > 0):
                controller.pitch_kp_vec[k] = kp
                controller.pitch_kv_vec[k] = kv

    # NOTE: body_amp / leg_amp / frequency are NOT in the search space.
    # The optimizer tunes ONLY impedance gains (kp, kv); the gait
    # envelope is held fixed at whatever the YAML specifies. The
    # legacy `if "body_amp" in params: ...` branches that used to live
    # here have been removed because params will never contain those
    # keys. If you want to reintroduce wave-shape search, add the keys
    # back to objective() AND add the assignments back here.


def reset_robot_and_controller(robot, controller, sim, n_settle_steps=8):
    """Place the robot back at spawn and clear all controller state.

    IMPORTANT: After writing root + joint state we run a few
    *controller-driven* settle steps. The active joints have
    stiffness=0/damping=0 (effort-mode), so without the controller
    running they free-fall during any sim.step call and the centipede
    collapses into a heap before the trial even starts. Running the
    controller at t=0 keeps target=0 for everything (we're inside the
    settle window) so impedance gains hold the robot in neutral pose
    while PhysX commits the reset.
    """
    n_j = robot.num_joints
    dev = robot.device

    # 1. Hard-reset sim state
    zero_q = torch.zeros(1, n_j, device=dev)
    zero_v = torch.zeros(1, n_j, device=dev)
    robot.write_joint_state_to_sim(zero_q, zero_v)
    root_state = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=dev))

    # 2. Reset controller phases / latched references
    controller.head_yaw_ref = None
    controller.body_phases  = None
    controller.leg_phases   = None
    controller._cpg_initialized = False
    # 2b. Clear PD-tracking target history so first step of new trial
    #     starts with target_dot = 0 (avoids a huge spurious target_dot
    #     spike from "previous trial's last target → this trial's first
    #     target" finite difference across the reset boundary).
    if hasattr(controller, "reset_target_history"):
        controller.reset_target_history()

    # 3. Controller-driven settle: hold the rest pose while PhysX
    #    commits the new state. Use t=0 so settle gates everything off.
    dt = sim.get_physics_dt()
    for _ in range(n_settle_steps):
        robot.update(dt)
        controller.step(0.0, dt)
        sim.step(render=False)


def run_trial(sim, robot, controller, recorder, duration,
              buckle_grace_s=1.5):
    """Drive one trial. Returns (buckled, buckle_reason).

    `buckle_grace_s` skips the buckle check until then. Necessary
    because settling with stiff impedance gains can transiently spike
    rpy in the first few ticks before the controller damps it out.
    """
    dt = sim.get_physics_dt()
    n_steps = int(duration / dt)

    buckled = False
    buckle_reason = ""
    check_interval = max(1, int(0.05 / dt))   # check every 50ms

    for step in range(n_steps):
        t = step * dt
        robot.update(dt)
        controller.step(t, dt)
        sim.step(render=False)
        recorder.maybe_record(t)

        if t < buckle_grace_s:
            continue
        if step % check_interval == 0 and step > 0:
            root_quat = robot.data.root_quat_w[0].cpu().numpy()
            w, x, y, z = root_quat
            sinr = 2.0 * (w*x + y*z)
            cosr = 1.0 - 2.0*(x*x + y*y)
            roll  = math.degrees(math.atan2(sinr, cosr))
            sinp  = 2.0 * (w*y - z*x)
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
            if abs(pitch) > MAX_ROOT_PITCH_DEG:
                buckled, buckle_reason = True, f"root_pitch={pitch:.1f}@{t:.2f}s"
                break
            if abs(roll) > MAX_ROOT_ROLL_DEG:
                buckled, buckle_reason = True, f"root_roll={roll:.1f}@{t:.2f}s"
                break

    return buckled, buckle_reason


# ═════════════════════════════════════════════════════════════════════════════
# Cost
# ═════════════════════════════════════════════════════════════════════════════

def compute_cost(buckled, buckle_reason, recorder, weights, limits, settle_s):
    if buckled:
        return BUCKLED_COST, {"buckled": True, "reason": buckle_reason}

    if not recorder.times or len(recorder.times) < 10:
        return BUCKLED_COST, {"buckled": True, "reason": "no recorder data"}

    t = np.asarray(recorder.times)
    mask = t >= settle_s
    if mask.sum() < 5:
        return BUCKLED_COST, {"buckled": True, "reason": "trial too short post-settle"}

    body_q   = np.asarray(recorder.body_yaw_q)[mask]
    body_tgt = np.asarray(recorder.body_yaw_tgt)[mask]
    leg_q    = np.asarray(recorder.leg_q)[mask]
    leg_tgt  = np.asarray(recorder.leg_tgt)[mask]
    rmse_body_deg = float(np.degrees(np.sqrt(np.mean((body_q - body_tgt) ** 2))))
    rmse_leg_deg  = float(np.degrees(np.sqrt(np.mean((leg_q  - leg_tgt ) ** 2))))

    com_xy = np.asarray(recorder.com_pos)[mask, :2]
    if len(com_xy) < 3:
        return BUCKLED_COST, {"buckled": True, "reason": "too short for straightness"}
    displacement_m = float(np.linalg.norm(com_xy[-1] - com_xy[0]))
    seg_lens       = np.linalg.norm(np.diff(com_xy, axis=0), axis=1)
    path_length_m  = float(seg_lens.sum())
    straightness   = float(np.clip(displacement_m / max(path_length_m, 1e-9), 0.0, 1.0))

    active_time = max(float(t[mask][-1] - t[mask][0]), 1e-3)
    speed_mps   = displacement_m / active_time

    # Foot force is unavailable in this minimal port → peak_fw = 0.
    foot_mag = np.linalg.norm(np.asarray(recorder.foot_force)[mask], axis=-1)
    body_weight = recorder.total_mass * recorder.gravity_z
    peak_fw = float(foot_mag.max()) / max(body_weight, 1e-9)

    pitch_q   = np.asarray(recorder.pitch_q)
    pitch_tgt = np.asarray(recorder.pitch_tgt)
    if pitch_q.size > 0 and pitch_q.shape == pitch_tgt.shape and pitch_q.ndim == 2:
        pitch_q   = pitch_q[mask]
        pitch_tgt = pitch_tgt[mask]
        if pitch_q.shape[1] >= 2:
            deflect_per_t = np.std(pitch_q - pitch_tgt, axis=1)
            compliance_rad = float(np.mean(deflect_per_t))
        else:
            compliance_rad = 0.0
    else:
        compliance_rad = 0.0
    compliance_deg = float(np.degrees(compliance_rad))
    compliance_reward_d = min(compliance_deg, limits["compliance_cap_deg"])

    body_excess  = max(0.0, rmse_body_deg - limits["track_tol_body_deg"])
    leg_excess   = max(0.0, rmse_leg_deg  - limits["track_tol_leg_deg"])
    wobble       = 1.0 - straightness
    force_excess = max(0.0, peak_fw - limits["force_limit"])

    cost = (
        weights["w_body_track"]   * body_excess
      + weights["w_leg_track"]    * leg_excess
      + weights["w_wobble"]       * wobble
      + weights["w_force"]        * force_excess
      - weights["w_speed"]        * speed_mps
      - weights["w_displacement"] * displacement_m   # NEW 2026-05-01
      - weights["w_compliance"]   * compliance_reward_d
    )

    parts = dict(
        buckled=False,
        rmse_body_yaw_deg=rmse_body_deg,
        rmse_leg_deg=rmse_leg_deg,
        body_excess=body_excess,
        leg_excess=leg_excess,
        straightness=straightness,
        wobble=wobble,
        displacement_m=displacement_m,
        path_length_m=path_length_m,
        speed_mps=speed_mps,
        peak_force_over_weight=peak_fw,
        force_excess=force_excess,
        pitch_compliance_deg=compliance_deg,
        pitch_compliance_reward_deg=compliance_reward_d,
        cost=cost,
    )
    return float(cost), parts


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════

def main():
    weights = dict(
        w_wobble       = args.w_wobble,
        w_body_track   = args.w_body_track,
        w_leg_track    = args.w_leg_track,
        w_force        = args.w_force,
        w_speed        = args.w_speed,
        w_displacement = args.w_displacement,   # NEW 2026-05-01
        w_compliance   = args.w_compliance,
    )
    limits = dict(
        track_tol_body_deg = args.body_tol_deg,
        track_tol_leg_deg  = args.leg_tol_deg,
        force_limit        = args.force_limit,
        compliance_cap_deg = args.compliance_cap_deg,
    )

    if args.resume:
        storage_path = args.resume
        run_dir = os.path.dirname(os.path.abspath(storage_path))
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = "flat" if args.flat else f"rough_wl{int(round(args.terrain_wavelength))}"
        run_dir = os.path.join(OUTPUT_ROOT, f"impedance_bay_{tag}_{ts}")
        os.makedirs(run_dir, exist_ok=True)
        storage_path = os.path.join(run_dir, "study.db")

    csv_path = os.path.join(run_dir, "all_trials.csv")
    print(f"\n[opt] run dir: {run_dir}", flush=True)

    # ── Build sim + scene + controller ──
    with open(CFG_PATH, "r", encoding="utf-8") as f:
        base_cfg = yaml.safe_load(f)
    dt = float(base_cfg.get("simulation", {}).get("timestep", 0.004))
    sim_cfg = SimulationCfg(dt=dt)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[0.5, 0.5, 0.3], target=[0.0, 0.0, 0.0])

    robot = build_scene(sim, run_dir)
    sim.reset()
    robot.reset()

    print(f"[opt] articulation loaded: {robot.num_joints} joints, "
          f"{robot.num_bodies} bodies", flush=True)

    controller = ImpedanceTravelingWaveController(robot, CFG_PATH)
    settle_s   = controller.settle_time + controller.ramp_time + 0.5

    recorder = SensorRecorder(robot, controller, dt_record=1/200.0,
                              settle_time=settle_s)

    # ── Optuna study ──
    storage = f"sqlite:///{storage_path}"
    sampler = TPESampler(seed=args.seed, multivariate=True, constant_liar=True)
    study = optuna.create_study(
        study_name=f"impedance_isaaclab_{'flat' if args.flat else 'rough'}",
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
        # Wave envelope (body_amp, leg_amp_dof0/1) and frequency are
        # NOT seeded — they are NOT in the search space. The optimizer
        # tunes only the impedance gains (kp, kv); the gait is fixed
        # by the YAML.

        # Clip to bounds (must match objective() ranges exactly).
        # All kp upper bounds = 0.5 per user instruction 2026-04-30.
        # If --narrow-bounds is set we use the trial-208 vs trial-314 envelope
        # so the seed is clipped into the narrowed search box.
        if args.narrow_bounds:
            bounds = dict(NARROW_BOUNDS)
            print(f"[opt] --narrow-bounds: using trial #208 ↔ trial #314 envelope",
                  flush=True)
        else:
            bounds = dict(
                body_kp      =(1e-4,0.5), body_kv      =(1e-6,1.0),
                hip_yaw_kp   =(1e-4,0.5), hip_yaw_kv   =(1e-7,5e-1),
                hip_pitch_kp =(1e-5,0.5), hip_pitch_kv =(1e-8,5e-2),
                tibia_kp =(1e-3,0.5), tibia_kv =(1e-7,5e-2),
                tarsus_kp=(1e-3,0.5), tarsus_kv=(1e-6,5e-2),
                pitch_kp =(1e-4,0.5), pitch_kv =(1e-6,5e-1),
            )

        # ── Apply --bump-body-kp BEFORE clipping so the bump is the user's
        #    intended modification on top of the YAML seed (typical use:
        #    seed = trial #208 baseline, bump = 1.6× to start at the
        #    "moderately stiffer body" point). ──
        if args.bump_body_kp != 1.0 and "body_kp" in seed_params:
            old = seed_params["body_kp"]
            seed_params["body_kp"] = old * args.bump_body_kp
            print(f"[opt] --bump-body-kp {args.bump_body_kp}: "
                  f"body_kp {old:.6g} → {seed_params['body_kp']:.6g}",
                  flush=True)

        for k in list(seed_params.keys()):
            if k in bounds:
                lo, hi = bounds[k]
                clipped = float(np.clip(seed_params[k], lo, hi))
                if clipped != seed_params[k]:
                    print(f"[opt] seed.{k} clipped {seed_params[k]:.6g} "
                          f"→ {clipped:.6g}  [bounds {lo:.3g}, {hi:.3g}]",
                          flush=True)
                seed_params[k] = clipped
        print(f"[opt] seeding first trial from {seed_yaml}", flush=True)
        for k, v in seed_params.items():
            print(f"      {k:14s}= {v:.6g}", flush=True)
        study.enqueue_trial(seed_params)

    # ── Pretest probes (--pretest only) ─────────────────────────────────
    # Enqueue 4 additional fixed-edge probes BEFORE TPE starts proposing.
    # These give the optimizer (and us) immediate feedback on whether the
    # widened search bounds are sane: if "softest" buckles instantly, the
    # lower kp bound is too low; if "stiffest" buckles, the upper bound
    # is too high; etc.
    #
    # Each probe is logged to study.db just like a normal trial, so TPE
    # treats them as observations. If you watch the print output you'll
    # see "[trial 1] cost=..." etc. for each probe with its actual
    # buckled / non-buckled outcome.
    if args.pretest and not args.resume:
        # Helper: pull a value out of the seed (YAML baseline) and clip.
        def _clip(name, v):
            if name in bounds:
                lo, hi = bounds[name]
                return float(np.clip(v, lo, hi))
            return float(v)

        def _probe(label, scale_kp, scale_kv):
            """Build a probe trial by scaling the seed kp/kv values.
            Wave amps are held at their YAML values (not in search)."""
            p = dict()
            for k, v in seed_params.items():
                if k.endswith("_kp"):
                    p[k] = _clip(k, v * scale_kp)
                elif k.endswith("_kv"):
                    p[k] = _clip(k, v * scale_kv)
                else:
                    p[k] = v
            return label, p

        probes = [
            _probe("SOFTEST",     0.1,  0.1 ),
            _probe("STIFFEST",   10.0, 10.0 ),
            _probe("OVERDAMPED",  1.0, 10.0 ),
            _probe("UNDAMPED",    1.0,  0.01),
        ]
        print(f"[opt] --pretest: enqueueing {len(probes)} edge probes "
              f"(softest / stiffest / overdamped / undamped). "
              f"Watch their cost & buckle status in the trial log to "
              f"verify the widened search bounds are sane.", flush=True)
        for label, p in probes:
            print(f"      probe '{label}': "
                  f"body_kp={p.get('body_kp', 0):.4f} "
                  f"body_kv={p.get('body_kv', 0):.6f}", flush=True)
            study.enqueue_trial(p)

    # ── Objective ──
    trial_rows = []

    def objective(trial):
        # ════════════════════════════════════════════════════════════════
        # ── Search ranges (12-D IMPEDANCE-ONLY, widened 2026-04-30) ──
        # ════════════════════════════════════════════════════════════════
        # All ranges are log-uniform (log=True). With log sampling, equal
        # exploration weight goes to each decade — no extra normalisation
        # needed. The widened ranges below give the optimizer ~3-5× more
        # room than the v3-era MuJoCo ranges, so it can explore both
        # very soft (terrain-compliant) and very stiff (wave-tracking)
        # regions of parameter space.
        #
        # SEARCH SPACE: 12 dimensions max — JUST the impedance gains
        # (kp, kv) for body / hip / tibia / tarsus / body-pitch joint
        # groups. The wave envelope (body_amp, leg_amp_dof0/1) and
        # frequency are NOT in the search space — they are
        # controller/gait-design choices that come from the YAML and
        # the optimizer should not be allowed to game them.
        #
        # ── INITIAL PARAMETER VALUES (from current YAML, 2026-04-30) ──
        # Trial #0 is seeded from these. They are also the centre point
        # the optimizer's TPE will explore around for the first ~15
        # trials before refining toward higher-density regions.
        #     body_kp       = 0.05         body_kv       = 0.02
        #     hip_yaw_kp    = 0.5          hip_yaw_kv    = 0.001
        #     hip_pitch_kp  = 0.06         hip_pitch_kv  = 0.003
        #     tibia_kp      = 0.13         tibia_kv      = 0.0005
        #     tarsus_kp     = 0.13         tarsus_kv     = 0.001
        #     pitch_kp      = 0.02         pitch_kv      = 0.005
        #
        # If the optimizer never beats the seed cost across hundreds of
        # trials, it means the bounds are too wide and TPE is wasting
        # exploration on hopeless regions — tighten them. If it
        # routinely beats the seed by 2-3×, the bounds are well-sized.
        # ── kp upper bound CAPPED AT 0.5 (user instruction 2026-04-30) ──
        # All proportional gains share a 0.5 ceiling. The earlier study
        # found "winners" with hip_yaw_kp ≈ 3.6 / hip_pitch_kp ≈ 2.26
        # which produced fast but tracking-error-laden gaits — they hit
        # the wave's commanded amplitude before the joint had time to
        # accelerate, causing visible overshoot. Capping kp ≤ 0.5
        # forces the optimizer into the regime where joints actually
        # follow the wave (low body_track / leg_track RMSE).
        # kv ranges are unchanged.
        # ── pick wide vs narrow ranges (set ONCE per call) ──
        # Narrow mode pulls from NARROW_BOUNDS so TPE physically cannot
        # propose a value outside the [trial 208, trial 314] envelope.
        # Wide mode keeps the pre-2026-05-03 ranges.
        def _rng(name, wide_lo, wide_hi):
            if args.narrow_bounds and name in NARROW_BOUNDS:
                return NARROW_BOUNDS[name]
            return (wide_lo, wide_hi)

        _b_lo, _b_hi   = _rng("body_kp",      1e-4, 0.5)
        _bv_lo, _bv_hi = _rng("body_kv",      1e-6, 1.0)
        _hy_lo, _hy_hi = _rng("hip_yaw_kp",   1e-4, 0.5)
        _hyv_lo, _hyv_hi = _rng("hip_yaw_kv", 1e-7, 5e-1)
        _hp_lo, _hp_hi = _rng("hip_pitch_kp", 1e-5, 0.5)
        _hpv_lo, _hpv_hi = _rng("hip_pitch_kv", 1e-8, 5e-2)
        params = dict(
            body_kp      = trial.suggest_float("body_kp",      _b_lo,  _b_hi,  log=True),
            body_kv      = trial.suggest_float("body_kv",      _bv_lo, _bv_hi, log=True),
            hip_yaw_kp   = trial.suggest_float("hip_yaw_kp",   _hy_lo, _hy_hi, log=True),
            hip_yaw_kv   = trial.suggest_float("hip_yaw_kv",   _hyv_lo, _hyv_hi, log=True),
            hip_pitch_kp = trial.suggest_float("hip_pitch_kp", _hp_lo, _hp_hi, log=True),
            hip_pitch_kv = trial.suggest_float("hip_pitch_kv", _hpv_lo, _hpv_hi, log=True),
        )
        if args.tune_distal:
            _tk_lo, _tk_hi = _rng("tibia_kp",  1e-3, 0.5)
            _tv_lo, _tv_hi = _rng("tibia_kv",  1e-7, 5e-2)
            _sk_lo, _sk_hi = _rng("tarsus_kp", 1e-3, 0.5)
            _sv_lo, _sv_hi = _rng("tarsus_kv", 1e-6, 5e-2)
            params["tibia_kp"]  = trial.suggest_float("tibia_kp",  _tk_lo, _tk_hi, log=True)
            params["tibia_kv"]  = trial.suggest_float("tibia_kv",  _tv_lo, _tv_hi, log=True)
            params["tarsus_kp"] = trial.suggest_float("tarsus_kp", _sk_lo, _sk_hi, log=True)
            params["tarsus_kv"] = trial.suggest_float("tarsus_kv", _sv_lo, _sv_hi, log=True)
        if args.tune_pitch:
            _pk_lo, _pk_hi = _rng("pitch_kp", 1e-4, 0.5)
            _pv_lo, _pv_hi = _rng("pitch_kv", 1e-6, 5e-1)
            params["pitch_kp"] = trial.suggest_float("pitch_kp", _pk_lo, _pk_hi, log=True)
            params["pitch_kv"] = trial.suggest_float("pitch_kv", _pv_lo, _pv_hi, log=True)

        # NOTE: body_amp / leg_amp_dof0 / leg_amp_dof1 / frequency
        # are intentionally NOT in the search space. They are gait-
        # design parameters set by configs/farms_controller.yaml. The
        # optimizer's job is to find IMPEDANCE GAINS that make the
        # YAML-specified gait stable and effective on the target
        # terrain — not to redefine the gait itself.

        # Apply params
        apply_params(controller, params)

        # Reset robot + controller + recorder. The reset itself runs
        # controller-driven settle steps so we don't need a separate
        # warmup loop here.
        reset_robot_and_controller(robot, controller, sim)
        recorder.reset()

        # Run
        t0 = time.time()
        buckled, reason = run_trial(sim, robot, controller, recorder,
                                    args.duration)
        cost, parts = compute_cost(buckled, reason, recorder,
                                   weights, limits, settle_s=settle_s)
        wall = time.time() - t0

        if parts.get("buckled", False):
            print(f"  [trial {trial.number:4d}] BUCKLE  {parts.get('reason','?')}  "
                  f"({wall:.1f}s) cost={cost:.1f}", flush=True)
        else:
            print(f"  [trial {trial.number:4d}] cost={cost:7.3f}  "
                  f"speed={parts['speed_mps']*1000:5.1f}mm/s  "
                  f"strt={parts['straightness']:.3f}  "
                  f"cmp={parts['pitch_compliance_deg']:4.1f}°  "
                  f"({wall:.1f}s)", flush=True)

        row = {**params, **parts, "trial": trial.number, "wall_s": wall}
        trial_rows.append(row)
        _dump_csv(csv_path, trial_rows)
        return cost

    # ── Run optimization ──
    t_start = time.time()
    try:
        study.optimize(objective, n_trials=args.n_trials,
                       n_jobs=1, show_progress_bar=False)
    except KeyboardInterrupt:
        print("\n[opt] interrupted — writing partial results", flush=True)

    elapsed = time.time() - t_start
    _dump_csv(csv_path, trial_rows)

    # ── Pick winner ──
    ok = [t for t in study.trials if t.value is not None and t.value < BUCKLED_COST*0.99]
    if not ok:
        print("[opt] no non-buckled trials. Relax weights or extend duration.")
        sim_app.close()
        return 1

    best = min(ok, key=lambda t: t.value)
    print(f"\n[opt] best trial #{best.number}  cost={best.value:.4f}", flush=True)
    for k, v in best.params.items():
        print(f"      {k:14s}= {v:.6g}", flush=True)

    # Write best_params.yaml
    best_cfg = _patch_yaml(base_cfg, best.params)
    best_yaml_path = os.path.join(run_dir, "best_params.yaml")
    with open(best_yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(best_cfg, f, sort_keys=False, allow_unicode=True)
    with open(os.path.join(run_dir, "best_params.json"), "w", encoding="utf-8") as f:
        json.dump({
            "trial_number": best.number,
            "cost": best.value,
            "params": best.params,
            "n_total_trials": len(study.trials),
            "n_non_buckled":  len(ok),
            "elapsed_min":    elapsed / 60.0,
            "tune_distal":    args.tune_distal,
            "tune_pitch":     args.tune_pitch,
            "terrain_flat":   args.flat,
            "terrain_wavelength_mm": args.terrain_wavelength,
            "terrain_amplitude_m":   args.terrain_amplitude,
            "terrain_seed":          args.terrain_seed,
        }, f, indent=2)
    print(f"[opt] wrote {best_yaml_path}", flush=True)

    print(f"\n[opt] done — {len(study.trials)} trials in {elapsed/60:.1f} min")

    # CRITICAL: close the optimizer's Isaac Sim instance BEFORE spawning
    # test_walk.py. Otherwise the GPU / Fabric / RTX context is locked
    # by this process and the subprocess will fail to initialise its
    # own renderer.
    sim_app.close()

    # ── Render the winning trial via test_walk.py (subprocess) ──
    # The previous in-process replay (_replay_with_video) was broken
    # because it created the Camera AFTER the optimizer's sim.reset()
    # — Isaac Lab's documented order is Camera FIRST then sim.reset().
    # Rather than restructure main() to reverse the order (which would
    # break the per-trial loop), we just shell out to test_walk.py
    # which already has the correct ordering and the working render
    # pipeline (Fabric pose write + bad-frame guard + RaytracedLighting).
    if args.replay_video:
        replay_path = os.path.join(run_dir, "best_trial.mp4")
        _render_winner_via_test_walk(best_yaml_path, replay_path,
                                     duration=args.duration,
                                     terrain_flat=args.flat,
                                     terrain_wavelength_mm=args.terrain_wavelength,
                                     terrain_amplitude_m=args.terrain_amplitude)
    return 0


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _dump_csv(path, rows):
    if not rows:
        return
    keys = sorted({k for r in rows for k in r.keys()})
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _patch_yaml(base_cfg, params):
    import copy
    cfg = copy.deepcopy(base_cfg)
    imp = cfg.setdefault("impedance", {})
    imp["body_kp"] = float(params["body_kp"])
    imp["body_kv"] = float(params["body_kv"])
    if "pitch_kp" in params:
        imp["pitch_kp"] = float(params["pitch_kp"])
        imp["pitch_kv"] = float(params["pitch_kv"])
    leg = imp.setdefault("leg", {})
    kp = list(leg.get("kp", [0.5, 0.06, 0.13, 0.13]))
    kv = list(leg.get("kv", [0.001, 0.003, 0.0005, 0.001]))
    kp[0] = float(params["hip_yaw_kp"]);   kv[0] = float(params["hip_yaw_kv"])
    kp[1] = float(params["hip_pitch_kp"]); kv[1] = float(params["hip_pitch_kv"])
    if "tibia_kp" in params:
        kp[2] = float(params["tibia_kp"]);  kv[2] = float(params["tibia_kv"])
    if "tarsus_kp" in params:
        kp[3] = float(params["tarsus_kp"]); kv[3] = float(params["tarsus_kv"])
    leg["kp"] = kp
    leg["kv"] = kv

    # ── Wave envelope: write back to body_wave / leg_wave blocks ──
    bw = cfg.setdefault("body_wave", {})
    if "body_amp"  in params: bw["amplitude"] = float(params["body_amp"])
    if "frequency" in params: bw["frequency"] = float(params["frequency"])
    lw = cfg.setdefault("leg_wave", {})
    leg_amps = list(lw.get("amplitudes", [0.10, 0.05, 0.0, 0.0]))
    if "leg_amp_dof0" in params: leg_amps[0] = float(params["leg_amp_dof0"])
    if "leg_amp_dof1" in params: leg_amps[1] = float(params["leg_amp_dof1"])
    lw["amplitudes"] = leg_amps
    return cfg


def _render_winner_via_test_walk(best_yaml_path, video_path, duration,
                                  terrain_flat, terrain_wavelength_mm,
                                  terrain_amplitude_m):
    """Render the winning trial's video by shelling out to test_walk.py.

    Why subprocess instead of in-process:
      Isaac Lab's documented camera-init order is "create Camera, THEN
      call sim.reset() so the camera's render product gets initialised
      during scene-graph build". The optimizer's main() calls sim.reset()
      ONCE up-front (to start trials), so a Camera created later (at
      replay time) ends up with an uninitialised render product —
      99%+ of frames come back black/uniform regardless of what
      camera-pose write path we use.

      test_walk.py has the correct ordering. Rather than restructure
      the optimizer to reverse the sim.reset / Camera-create order
      (which would break the per-trial loop), we close the optimizer's
      Isaac Sim instance and spawn test_walk.py as a subprocess with
      the winning gains.

    Steps:
      1. Back up the live configs/farms_controller.yaml.
      2. Copy best_params.yaml → configs/farms_controller.yaml.
      3. Run test_walk.py as a subprocess with --terrain matching the
         optimizer's training terrain (so the centipede walks on the
         same heightmap it was tuned for).
      4. Restore the original farms_controller.yaml.

    The user's live YAML is therefore never permanently overwritten.
    """
    import subprocess
    import shutil
    print(f"\n[replay] spawning test_walk.py to render winner → {video_path}",
          flush=True)
    backup_path = CFG_PATH + ".opt_replay_backup_" \
                  + datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        # 1. Backup live YAML
        shutil.copy(CFG_PATH, backup_path)
        # 2. Swap winning gains into the live YAML so test_walk picks them up
        shutil.copy(best_yaml_path, CFG_PATH)

        # 3. Build & run the test_walk command. Match the optimizer's
        #    terrain settings so the centipede sees the SAME world it
        #    was tuned for.
        test_walk = os.path.join(PROJECT_ROOT, "scripts", "test_walk.py")
        cmd = [
            sys.executable, test_walk,
            "--headless",
            "--duration", str(duration),
            "--video", video_path,
            "--camera-mode", "chase",
        ]
        if terrain_flat:
            cmd += ["--terrain", "flat"]
        else:
            cmd += [
                "--terrain", "rough",
                "--terrain-wavelength", str(terrain_wavelength_mm * 1e-3),
                "--terrain-amplitude", str(terrain_amplitude_m),
            ]
        print(f"[replay] command: {' '.join(cmd)}", flush=True)
        result = subprocess.run(cmd, capture_output=False)
        if result.returncode != 0:
            print(f"[replay] test_walk.py exited with code {result.returncode}",
                  flush=True)
        else:
            print(f"[replay] video saved to {video_path}", flush=True)
    except Exception as e:
        print(f"[replay] subprocess replay failed: {e}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        # 4. Always restore the original YAML, even on subprocess error
        if os.path.exists(backup_path):
            shutil.copy(backup_path, CFG_PATH)
            print(f"[replay] restored original YAML from {backup_path}",
                  flush=True)
            # Keep the backup file on disk for safety (user can delete
            # it manually if they want). Cheap insurance against an
            # interrupted shutil.copy.


# ─────────────────────────────────────────────────────────────────────────────
# Legacy in-process replay (kept for reference; NOT called from main)
# ─────────────────────────────────────────────────────────────────────────────
# The function below was the original in-process replay. It was retired
# 2026-05-02 because the camera-init ordering in main() (sim.reset before
# Camera-create) made it produce 99%+ black frames on rough terrain.
# The replacement is _render_winner_via_test_walk above. This code is
# kept commented for reference only — uncomment ONLY if you restructure
# main() to create the Camera before the first sim.reset().

def _replay_with_video_LEGACY_DO_NOT_USE(sim, robot, controller, recorder,
                                          duration, video_path, video_fps=30):
    """LEGACY — see _render_winner_via_test_walk for the working replacement."""
    print(f"[replay] rendering winning trial → {video_path}", flush=True)
    from isaaclab.sensors import Camera, CameraCfg

    cam_cfg = CameraCfg(
        prim_path="/World/ReplayCamera",
        update_period=0.0,
        height=720, width=1280,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0,
            horizontal_aperture=20.955, clipping_range=(0.05, 100.0),
        ),
    )
    cam = Camera(cfg=cam_cfg)
    sim.reset()  # initialize the camera sensor + its Fabric buffers

    # ── Static look-at — 30 cm behind-right, 25 cm up ──
    # Use Camera.set_world_poses (Fabric path) instead of the USD xform
    # write the previous version used. Hand-compute the look-at
    # quaternion (set_world_poses_from_view is broken on this install).
    eye_xyz    = (-0.35, -0.30, 0.25)
    target_xyz = ( 0.10,  0.00, 0.02)
    eye_np = np.asarray(eye_xyz,    dtype=np.float64)
    tgt_np = np.asarray(target_xyz, dtype=np.float64)
    fwd = tgt_np - eye_np
    fwd /= max(np.linalg.norm(fwd), 1e-12)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= max(np.linalg.norm(right), 1e-12)
    cam_up = np.cross(right, fwd)
    # OpenGL/USD camera convention: cam looks down -Z, +X right, +Y up.
    # Rotation matrix columns = world axes of the camera's local frame.
    R = np.column_stack([right, cam_up, -fwd])
    # Standard branchless rotation-matrix → (w,x,y,z) quaternion (Shepperd).
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2,1] - R[1,2]) / s
        qy = (R[0,2] - R[2,0]) / s
        qz = (R[1,0] - R[0,1]) / s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        qw = (R[2,1] - R[1,2]) / s
        qx = 0.25 * s
        qy = (R[0,1] + R[1,0]) / s
        qz = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        qw = (R[0,2] - R[2,0]) / s
        qx = (R[0,1] + R[1,0]) / s
        qy = 0.25 * s
        qz = (R[1,2] + R[2,1]) / s
    else:
        s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        qw = (R[1,0] - R[0,1]) / s
        qx = (R[0,2] + R[2,0]) / s
        qy = (R[1,2] + R[2,1]) / s
        qz = 0.25 * s
    try:
        cam_device = cam.device
    except AttributeError:
        cam_device = getattr(cam, "_device", "cuda")
    pos_t  = torch.tensor([list(eye_xyz)], dtype=torch.float32,
                          device=cam_device)
    quat_t = torch.tensor([[qw, qx, qy, qz]], dtype=torch.float32,
                          device=cam_device)
    # ── Tensor pose API with USD fallback ──
    # On Isaac Lab installs where Fabric (usdrt.hierarchy) isn't
    # fully loaded, cam.set_world_poses raises AttributeError on the
    # very first call (it goes through _set_world_poses_fabric →
    # _initialize_fabric → usdrt.hierarchy.IFabricHierarchy()). We
    # cannot let that propagate or the entire replay crashes after
    # an N-hour optimization. Instead we mirror the test_walk.py
    # pattern: try the tensor API, on failure fall back to writing
    # the camera's USD xform op directly. The render product will
    # eventually pick up the USD pose via sim.step's own USD→Fabric
    # sync — sufficient for a STATIC replay camera (this isn't chase
    # mode; the pose is set ONCE before any rendering).
    try:
        cam.set_world_poses(positions=pos_t, orientations=quat_t,
                             convention="opengl")
        print(f"[replay] camera at eye={eye_xyz} target={target_xyz} "
              f"(Fabric-write path)", flush=True)
    except Exception as e:
        print(f"[replay] tensor pose API unavailable "
              f"({type(e).__name__}: {e}) — falling back to USD xform "
              f"write. Static replay camera + warmup steps should make "
              f"this OK in practice.", flush=True)
        import omni.usd as _omni_usd
        from pxr import UsdGeom as _UsdGeom, Gf as _Gf
        _stage = _omni_usd.get_context().get_stage()
        _cam_prim = _stage.GetPrimAtPath("/World/ReplayCamera")
        _eye_v = _Gf.Vec3d(*eye_xyz)
        _tgt_v = _Gf.Vec3d(*target_xyz)
        _up    = _Gf.Vec3d(0, 0, 1)
        _fwd   = (_tgt_v - _eye_v).GetNormalized()
        _right = (_fwd ^ _up).GetNormalized()
        _camup = (_right ^ _fwd).GetNormalized()
        _m = _Gf.Matrix4d()
        _m.SetRow(0, _Gf.Vec4d(_right[0], _right[1], _right[2], 0))
        _m.SetRow(1, _Gf.Vec4d(_camup[0], _camup[1], _camup[2], 0))
        _m.SetRow(2, _Gf.Vec4d(-_fwd[0],  -_fwd[1],  -_fwd[2],  0))
        _m.SetRow(3, _Gf.Vec4d(_eye_v[0], _eye_v[1], _eye_v[2], 1))
        _xf = _UsdGeom.Xformable(_cam_prim)
        _xf.ClearXformOpOrder()
        _xf.AddTransformOp().Set(_m)
        print(f"[replay] camera at eye={eye_xyz} target={target_xyz} "
              f"(USD-xform-write fallback path)", flush=True)

    reset_robot_and_controller(robot, controller, sim)
    # extra render-warmup so the RTX pipeline has converged frames
    for _ in range(5):
        sim.step(render=True)
    cam.update(sim.get_physics_dt())

    dt_phys = sim.get_physics_dt()
    n_steps = int(duration / dt_phys)
    video_dt = 1.0 / video_fps
    last_t = -1e9
    frames = []
    black_replaced = 0
    for step in range(n_steps):
        t = step * dt_phys
        robot.update(dt_phys); controller.step(t, dt_phys)
        do_render = (t - last_t) >= video_dt
        sim.step(render=do_render)
        if do_render:
            cam.update(dt_phys)
            rgb = cam.data.output.get("rgb", None)
            if rgb is not None:
                arr = rgb[0].detach().cpu().numpy()
                if arr.ndim == 3 and arr.shape[2] == 4:
                    arr = arr[:, :, :3]
                arr = arr.astype(np.uint8)
                # Bad-frame guard — same race fix as test_walk.py.
                # Catches BOTH (a) pure-black mid-write race
                # (mean<3) and (b) uniform-sky stale-buffer holds
                # (std<5) that produce long visual freezes when the
                # renderer falls behind on rough terrain. See
                # test_walk.py capture_frame for full rationale.
                m, s = float(arr.mean()), float(arr.std())
                if (m < 3.0 or s < 5.0) and frames:
                    arr = frames[-1].copy()
                    black_replaced += 1
                frames.append(arr)
            last_t = t
    if black_replaced > 0:
        pct = 100.0 * black_replaced / max(len(frames), 1)
        print(f"[replay] bad-frame guard: replaced "
              f"{black_replaced}/{len(frames)} frames "
              f"({pct:.1f}%) (black or uniform-sky) with "
              f"previous good frame.",
              flush=True)

    # Write MP4
    if not frames:
        print("[replay] no frames captured")
        return
    try:
        import cv2
        h, w = frames[0].shape[:2]
        os.makedirs(os.path.dirname(video_path), exist_ok=True)
        writer = cv2.VideoWriter(video_path,
                                 cv2.VideoWriter_fourcc(*'mp4v'),
                                 video_fps, (w, h))
        for fr in frames:
            writer.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
        writer.release()
        print(f"[replay] wrote {video_path}  ({len(frames)} frames)", flush=True)
    except ImportError:
        try:
            import imageio
            imageio.mimwrite(video_path, frames, fps=video_fps)
            print(f"[replay] wrote {video_path} (imageio)", flush=True)
        except ImportError:
            print("[replay] no cv2 or imageio — skipping video write")


if __name__ == "__main__":
    sys.exit(main())
