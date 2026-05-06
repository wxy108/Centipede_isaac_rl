#!/usr/bin/env python3
"""
optimize_impedance_biological.py — Bayesian optimiser, biology-faithful cost
==============================================================================
Sister script of optimize_impedance_bayesian.py. Same TPE sampler, same
12-D impedance-only search space, same Isaac Sim plumbing — but the cost
function is REWRITTEN to favour biologically-faithful gaits over raw
speed.

What's different from the speed/displacement-prioritised optimiser
(optimize_impedance_bayesian.py, post-2026-05-01):

  * SPEED REWARD IS CAPPED at --speed-cap-mps (default 0.025 m/s ≈ 25
    mm/s, the upper end of biological centipede walking speed). Above
    the cap the reward saturates so the optimizer no longer extracts
    cost by pushing the gait into superhuman territory.

  * NEW PENALTY: vertical body bounce (RMS of CoM z about its mean).
    Real centipedes glide at near-constant height. This term directly
    targets the visible bounce that the wide-bounds optimiser couldn't
    "see" because it wasn't measured.

  * NEW PENALTY: body roll RMS. Real centipedes are planar — they
    don't roll about the long axis. A soft penalty (in addition to
    the buckle threshold at ±45°) discourages even small persistent
    roll oscillation.

  * NEW PENALTY: torque rate / jerk. Biological neural activation
    produces SMOOTH muscle commands. Twitchy, oscillating joint
    torques are unbiological. We penalise mean(|d τ /dt|²).

  * NEW REWARD/PENALTY: cost-of-transport (work per metre travelled).
    Biological systems are energy-efficient. Lower CoT is better.

  * REWEIGHTED: w_speed 200 → 80 and w_displacement 100 → 40 so the
    new biology terms have room to shape the winner. Speed and
    displacement still matter (the gait must actually walk forward),
    but they no longer dominate.

  * w_wobble 4.0 → 6.0 (real centipedes walk straight).

Same outputs (timestamped run dir under outputs/optimization/) as the
speed-prioritised optimiser, just under a different name prefix
(impedance_bio_<terrain>_<ts>/) so studies don't collide.

Usage (cmd):
    cd C:\\Users\\wxy22\\Documents\\Centipede_IsaacLab
    python scripts\\optimization\\optimize_impedance_biological.py ^
        --headless ^
        --terrain-wavelength 18 --terrain-amplitude 0.01 --terrain-seed 42 ^
        --narrow-bounds --bump-body-kp 1.6 ^
        --pretest --n-trials 200 --duration 8

  ── ORIGINAL HEADER FROM optimize_impedance_bayesian.py ──
Same search space, same straightness-based cost (now AUGMENTED), same
TPE sampler as the MuJoCo version. Trials run inside a single Isaac
Sim process under PhysX, with the centipede walking on a USD-baked
rough terrain at λ=18 mm by default.

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
parser.add_argument("--run-tag", default=None,
                    help="Optional tag inserted into the output directory "
                         "name: impedance_bio_<terrain>_<run-tag>_<ts>/. "
                         "Use this to distinguish parallel/sequential "
                         "studies that share the same terrain (e.g. "
                         "--run-tag wide vs --run-tag narrow).")
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
# ── Cost weights — BIOLOGICALLY-FAITHFUL RETUNE (2026-05-03) ──
# Speed and displacement weights cut so the optimiser doesn't extract
# all its reward from raw rate. New bio terms (body z RMS, roll RMS,
# torque jerk, CoT) shape the gait toward smooth, planar, efficient
# motion — like real centipedes.
parser.add_argument("--w-wobble",     type=float, default=6.0,
                    help="Penalty per unit of (1 − straightness). Was 4.0; "
                         "bumped to 6.0 — real centipedes walk straight.")
parser.add_argument("--w-body-track", type=float, default=0.10,
                    help="Penalty per deg of body-yaw RMSE above body-tol.")
parser.add_argument("--w-leg-track",  type=float, default=0.20,
                    help="Penalty per deg of leg RMSE above leg-tol.")
parser.add_argument("--w-force",      type=float, default=0.5)
parser.add_argument("--w-speed",      type=float, default=80.0,
                    help="Reward per m/s forward speed (CAPPED at "
                         "--speed-cap-mps). Was 200.0; cut to 80 so the bio "
                         "terms can shape the winner instead of raw rate "
                         "dominating the cost.")
parser.add_argument("--w-displacement", type=float, default=40.0,
                    help="Reward per metre of net forward displacement. "
                         "Was 100.0; cut to 40 for the same reason as "
                         "w-speed. Still rewards going FAR (vs in circles), "
                         "just no longer dominant.")
parser.add_argument("--w-compliance", type=float, default=0.05,
                    help="Reward per degree of pitch-joint-deflection std.")
# ── NEW BIO PENALTIES ──
parser.add_argument("--w-body-z",     type=float, default=400.0,
                    help="NEW (2026-05-03): penalty per metre of vertical "
                         "body-z RMS (CoM height std). Real centipedes "
                         "glide at near-constant height — this term "
                         "directly attacks visible body bouncing. "
                         "Default 400 means a 5 mm RMS bounce → +2.0 cost; "
                         "a 1 mm RMS bounce → +0.4 cost.")
parser.add_argument("--w-body-roll",  type=float, default=2.0,
                    help="NEW (2026-05-03): penalty per degree of body-roll "
                         "RMS. Real centipedes are planar — they don't roll "
                         "about the long axis. Default 2.0 means a 1° RMS "
                         "roll → +2.0 cost. Buckle threshold (45°) is the "
                         "hard cutoff; this is a soft penalty for any "
                         "persistent oscillation below that.")
parser.add_argument("--w-torque-jerk", type=float, default=1e-3,
                    help="NEW (2026-05-03): penalty on mean (dτ/dt)². "
                         "Biological neural activation is smooth; twitchy "
                         "torque commands are unbiological. Default scales "
                         "the typically-large jerk magnitude (~1e3-1e5) "
                         "down to comparable units with the other terms.")
parser.add_argument("--w-cot",         type=float, default=0.5,
                    help="NEW (2026-05-03): penalty on cost-of-transport "
                         "(total positive joint work / displacement). "
                         "Biological systems minimise energy per metre. "
                         "Default 0.5 means a CoT of 1 J/m → +0.5 cost.")
parser.add_argument("--speed-cap-mps", type=float, default=0.025,
                    help="(legacy) cap on speed reward in the weighted-sum "
                         "cost. Now retained only for the diagnostic "
                         "cost_legacy in the parts dict. The new "
                         "constraint-based cost uses --min-velocity-quality "
                         "instead.")
# ── Constraint-based cost flags (2026-05-04) ──
parser.add_argument("--leg-proj-length-m", type=float, default=0.035,
                    help="Projected leg length on the ground (m), used to "
                         "compute the commanded forward velocity from YAML. "
                         "Default 0.035 = 35 mm (matches our 102.5 mm "
                         "centipede with bent leg).")
parser.add_argument("--nominal-clearance-m", type=float, default=0.0258,
                    help="Nominal body standing height above the ground (m). "
                         "Used to normalise body_z RMS into a dimensionless "
                         "ratio. Default 0.0258 m matches NOMINAL_CLEARANCE.")
parser.add_argument("--min-velocity-quality", type=float, default=0.3,
                    help="Hard constraint floor on v_actual / v_commanded. "
                         "Below this, trial is infeasible (CoT + violation "
                         "penalty). Default 0.3 — anything less than 30%% of "
                         "commanded velocity is 'stuck or dragging'.")
parser.add_argument("--max-z-rms-ratio", type=float, default=0.10,
                    help="(legacy / diagnostic) constraint ceiling on body_z "
                         "RMS / nominal clearance. NOT USED by the new "
                         "constraint-based cost — superseded by "
                         "--max-clearance-rms-ratio (terrain-aware).")
parser.add_argument("--max-clearance-rms-ratio", type=float, default=0.10,
                    help="Hard constraint ceiling on clearance RMS / nominal "
                         "clearance. clearance(t) = body_z(t) - terrain_z(x,y). "
                         "Constant clearance over varied terrain (drape) → "
                         "low metric (good). Rigid body or bouncing → high "
                         "metric (bad). Default 0.10 (10%% of standing height).")
parser.add_argument("--max-roll-deg", type=float, default=5.0,
                    help="Hard constraint ceiling on body_roll RMS (deg). "
                         "Default 5° — biological flatness threshold.")
parser.add_argument("--force-limit",  type=float, default=8.0,
                    help="Hard constraint ceiling on per-foot peak F/body-weight. "
                         "BIO_REFERENCES.md §1.5 (Cocci 2024): real centipedes "
                         "register peaks ~10× body weight per leg. 8× is "
                         "conservative. (Was 4.0 — bumped 2026-05-04 to match "
                         "MuJoCo defaults and the bio reference.)")
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

# ── Multi-block arena + multi-spawn evaluation (Flavor A, 2026-05-04) ──────
# When --arena is set, the optimizer evaluates each Optuna trial on N spawn
# offsets sampled deterministically once at startup (so trial-to-trial
# comparisons are fair), and reports cost = mean over those N runs.
# This is the configuration we hand to DRL afterwards: same arena, same
# spawns, just different policy.
parser.add_argument("--arena", action="store_true",
                    help="Use the multi-block blended arena from "
                         "scripts/optimization/arena.py instead of a single "
                         "fixed-wavelength heightmap. Implies multi-spawn "
                         "evaluation (see --n-spawns).")
parser.add_argument("--arena-size",        type=float, default=4.0,
                    help="Arena side length in metres (default 4.0).")
parser.add_argument("--arena-n-blocks",    type=int,   default=4,
                    help="Block grid side count; arena has N×N blocks "
                         "(default 4 → 16 blocks).")
parser.add_argument("--arena-wavelength-min-mm", type=float, default=10.0)
parser.add_argument("--arena-wavelength-max-mm", type=float, default=100.0)
parser.add_argument("--arena-amplitude-min-m",   type=float, default=0.005)
parser.add_argument("--arena-amplitude-max-m",   type=float, default=0.015)
parser.add_argument("--arena-image-size",  type=int,   default=2048,
                    help="Heightmap pixel resolution (default 2048).")
parser.add_argument("--wavelengths-mm",    type=str,   default="",
                    help="Comma-separated list of terrain wavelengths in mm "
                         "(e.g. '10,18,50'). When non-empty AND --arena is "
                         "set, the optimizer ignores the multi-block arena "
                         "and instead builds N tiled single-wavelength "
                         "terrain patches (one per λ, side-by-side along "
                         "+x). Each Optuna trial evaluates the SAME gains "
                         "on all N patches sequentially; cost = mean. If "
                         "ANY patch buckles, the trial is pruned (Optuna "
                         "discards it). Lets us optimize for terrain "
                         "robustness without the blended-arena complexity.")
parser.add_argument("--n-spawns",          type=int,   default=3,
                    help="When --arena is set WITHOUT --wavelengths-mm, "
                         "number of spawn offsets in the multi-block arena "
                         "to evaluate per trial. With --wavelengths-mm set, "
                         "this is ignored and the spawn count = number of "
                         "wavelengths. Default 3.")
parser.add_argument("--spawn-seed",        type=int,   default=20260504,
                    help="Seed for the deterministic spawn-offset sampler. "
                         "FIXED ACROSS ALL TRIALS so every trial sees the "
                         "exact same N spawn positions (controls for terrain "
                         "variance; lets us compare trials apples-to-apples "
                         "and hand the same setup to DRL).")
parser.add_argument("--spawn-margin-m",    type=float, default=0.6,
                    help="When picking spawn (x, y) offsets, stay this far "
                         "(in metres) from each arena edge. Avoids spawning "
                         "into the rim where blended-block weights drop and "
                         "the centipede has no straight terrain ahead of it. "
                         "Default 0.6 m → centipede has ≥ 0.6 m runway in "
                         "+x direction even from the rear-most legal spawn.")
parser.add_argument("--spawn-clearance-m", type=float, default=0.025,
                    help="Vertical clearance ABOVE the local terrain z at the "
                         "spawn (x, y). Default 25 mm matches the measured "
                         "nominal body-standing height (CLAUDE.md "
                         "NOMINAL_CLEARANCE = 0.0258 m).")

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


CFG_PATH = os.path.join(PROJECT_ROOT, "configs", "farms_controller.yaml")
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

def _compute_fixed_spawn_offsets(heightmap, arena_size_m, n_spawns,
                                 margin_m, clearance_m, seed):
    """Pick `n_spawns` deterministic (x, y, z) spawn offsets inside an
    interior box of the arena. SAME SEED → SAME OFFSETS, EVERY TRIAL.

    Spawn (x, y) is uniform on
        [-S/2 + margin, +S/2 - margin] × [-S/2 + margin, +S/2 - margin]
    and z = local_terrain_z(x, y) + clearance.

    The default centipede USD is authored at world (0, 0, 0.05). We add
    these offsets onto root_state in reset_robot_and_controller(), so an
    offset of (x, y, terrain_z(x,y) + clearance - 0.05) actually lands the
    body the requested clearance above local terrain. To keep the maths
    obvious for downstream callers, we return ABSOLUTE WORLD offsets (the
    delta the caller wants applied to the default spawn pose, not the
    final world position) and let reset_robot_and_controller add them.
    """
    H, W = heightmap.shape
    rng = np.random.default_rng(seed)
    half = arena_size_m * 0.5
    lo, hi = -half + margin_m, +half - margin_m
    if lo >= hi:
        raise ValueError(
            f"spawn_margin={margin_m} m is too large for arena "
            f"size={arena_size_m} m (interior box collapses).")

    def _terrain_z(x_world, y_world):
        # Bilinear lookup — same convention used in compute_cost.
        fx = (x_world + half) / arena_size_m * (W - 1)
        fy = (y_world + half) / arena_size_m * (H - 1)
        fx = float(np.clip(fx, 0.0, W - 1))
        fy = float(np.clip(fy, 0.0, H - 1))
        ix0, iy0 = int(np.floor(fx)), int(np.floor(fy))
        ix1, iy1 = min(ix0 + 1, W - 1), min(iy0 + 1, H - 1)
        tx, ty = fx - ix0, fy - iy0
        return float(
            (1 - tx) * (1 - ty) * heightmap[iy0, ix0]
            +     tx  * (1 - ty) * heightmap[iy0, ix1]
            + (1 - tx) *     ty  * heightmap[iy1, ix0]
            +     tx  *     ty  * heightmap[iy1, ix1])

    offsets = []
    for _ in range(int(n_spawns)):
        x = float(rng.uniform(lo, hi))
        y = float(rng.uniform(lo, hi))
        # USD authored z = +0.05; we want body to start at terrain_z + clearance.
        target_z = _terrain_z(x, y) + clearance_m
        dz = target_z - 0.05
        offsets.append((x, y, dz))
    return offsets


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

    # Ensure the heightmap stash is initialized for both flat and rough
    # terrain branches (cost function reads these for the clearance metric).
    build_scene._last_heightmap     = None
    build_scene._last_arena_size_m  = 1.0
    build_scene._spawn_offsets      = [(0.0, 0.0, 0.0)]   # populated below

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
        # Flat: terrain_z(x, y) = 0 everywhere. Use a 1-cell heightmap.
        build_scene._last_heightmap     = np.zeros((2, 2), dtype=np.float64)
        build_scene._last_arena_size_m  = 1.0
        print("[scene] flat ground at z=0", flush=True)
    elif args.arena:
        # ── Two arena modes ──
        # 1. --wavelengths-mm "10,18,50"  → N tiled SINGLE-wavelength patches
        #    placed side-by-side along +x. Each Optuna trial runs the SAME
        #    gains on all N patches and averages. If any patch buckles, the
        #    whole trial is pruned. This is what we use to reward gain sets
        #    that work across a range of bump scales without overfitting to
        #    one wavelength.
        # 2. (legacy) --wavelengths-mm empty → multi-block BLENDED arena from
        #    arena.generate_arena_heightmap(). Kept for backward compat.
        wl_str = (args.wavelengths_mm or "").strip()
        if wl_str:
            wavelengths_mm = [float(x) for x in wl_str.split(',') if x.strip()]
            patch_size = 1.0  # m per patch — same as the single-wavelength path
            spacing    = patch_size + 0.05  # 5 cm gap between adjacent patches

            patches = []   # list of dicts
            from pxr import UsdGeom as _UsdGeom, Gf as _Gf
            for k, wl_mm in enumerate(wavelengths_mm):
                # Centre tile k at x_off (along +x), y=0
                x_off = (k - (len(wavelengths_mm) - 1) / 2.0) * spacing
                y_off = 0.0

                wavelength_m = wl_mm * 1e-3
                h_m, rms_m, peak_m = generate_single_wavelength_heightmap(
                    wavelength_m=wavelength_m,
                    amplitude_m=args.terrain_amplitude,
                    seed=args.terrain_seed + k,   # unique seed per patch
                )

                # Wrap the mesh in an Xform with a translate op so it lands
                # at (x_off, y_off, 0) in world space.
                xform_path = f"/World/TerrainPatch_{k}"
                _UsdGeom.Xform.Define(stage, xform_path)
                patch_xform = _UsdGeom.Xformable(stage.GetPrimAtPath(xform_path))
                patch_xform.AddTranslateOp().Set(_Gf.Vec3d(x_off, y_off, 0.0))

                build_terrain_mesh(
                    stage=stage,
                    prim_path=xform_path + "/Mesh",
                    heightmap_m=h_m,
                    world_size=patch_size,
                    mesh_resolution=args.mesh_resolution,
                )

                patches.append(dict(
                    heightmap   = h_m,
                    x_off       = x_off,
                    y_off       = y_off,
                    world_size  = patch_size,
                    wavelength_mm = wl_mm,
                    rms_m       = rms_m,
                    peak_m      = peak_m,
                ))
                print(f"[scene] patch[{k}]  λ={wl_mm:5.1f}mm  "
                      f"x_off={x_off:+.3f}m  rms={rms_m*1000:.2f}mm  "
                      f"peak={peak_m*1000:.2f}mm", flush=True)

            np.save(os.path.join(run_dir, "patch_wavelengths_mm.npy"),
                    np.array(wavelengths_mm))
            with open(os.path.join(run_dir, "patches.json"), "w") as f:
                json.dump([dict(wavelength_mm=p["wavelength_mm"],
                                x_off=p["x_off"], y_off=p["y_off"],
                                world_size=p["world_size"],
                                rms_m=p["rms_m"], peak_m=p["peak_m"])
                           for p in patches], f, indent=2)

            build_scene._terrain_patches    = patches
            # The compute_cost code reads _last_heightmap / _last_arena_size_m;
            # we update these per-spawn in the objective loop. Here we just
            # initialise to the first patch so a single-shot smoke test works.
            build_scene._last_heightmap     = patches[0]["heightmap"]
            build_scene._last_arena_size_m  = patch_size
            build_scene._active_x_offset    = patches[0]["x_off"]
            build_scene._active_y_offset    = patches[0]["y_off"]

            # ── Spawn offsets: one per patch, centered on each patch ──
            # Same offsets every Optuna trial (deterministic for cross-trial
            # comparison and DRL handover).
            spawn_clearance = float(args.spawn_clearance_m)
            spawns = []
            for p in patches:
                # Local terrain z at (0, 0) within the patch, via bilinear
                # lookup on the patch heightmap. The centipede USD authors
                # at z=0.05; spawn dz = target_z - 0.05.
                hm = p["heightmap"]
                Hh, Ww = hm.shape
                ix = (Ww - 1) // 2
                iy = (Hh - 1) // 2
                terrain_z_local = float(hm[iy, ix])
                target_z = terrain_z_local + spawn_clearance
                dz = target_z - 0.05
                spawns.append((p["x_off"], p["y_off"], dz))
            build_scene._spawn_offsets = spawns

            print(f"[scene] {len(patches)} tiled wavelengths "
                  f"({wavelengths_mm} mm), spacing={spacing:.2f}m, "
                  f"amp={args.terrain_amplitude*1000:.1f}mm:", flush=True)
            for i_, (sx, sy, sz) in enumerate(spawns):
                print(f"        spawn[{i_}]  ({sx:+.3f}, {sy:+.3f}, "
                      f"{sz:+.4f}) m  on  λ={wavelengths_mm[i_]:.1f}mm",
                      flush=True)

        else:
            # ── Multi-block blended arena (legacy --arena path) ──
            from arena import (
                generate_arena_heightmap as _generate_arena_heightmap,
                build_arena_mesh as _build_arena_mesh,
            )
            heightmap, blocks = _generate_arena_heightmap(
                arena_size_m        = args.arena_size,
                n_blocks            = (args.arena_n_blocks, args.arena_n_blocks),
                wavelength_range_mm = (args.arena_wavelength_min_mm,
                                       args.arena_wavelength_max_mm),
                amplitude_range_m   = (args.arena_amplitude_min_m,
                                       args.arena_amplitude_max_m),
                image_size          = args.arena_image_size,
                seed                = args.terrain_seed,
            )
            np.save(os.path.join(run_dir, "arena_heightmap.npy"), heightmap)
            with open(os.path.join(run_dir, "arena_blocks.json"), "w") as f:
                json.dump([b.to_dict() for b in blocks], f, indent=2)

            build_scene._last_heightmap    = heightmap
            build_scene._last_arena_size_m = float(args.arena_size)
            build_scene._active_x_offset   = 0.0
            build_scene._active_y_offset   = 0.0

            UsdGeom.Xform.Define(stage, "/World/Terrain")
            _build_arena_mesh(
                stage         = stage,
                prim_path     = "/World/Terrain/Mesh",
                heightmap_m   = heightmap,
                arena_size_m  = float(args.arena_size),
                mesh_resolution = args.mesh_resolution,
            )
            print(f"[scene] ARENA  {args.arena_n_blocks}x{args.arena_n_blocks} blocks  "
                  f"size={args.arena_size:.2f}m  "
                  f"image={args.arena_image_size}px  "
                  f"mesh={args.mesh_resolution}", flush=True)

            build_scene._spawn_offsets = _compute_fixed_spawn_offsets(
                heightmap=heightmap,
                arena_size_m=float(args.arena_size),
                n_spawns=int(args.n_spawns),
                margin_m=float(args.spawn_margin_m),
                clearance_m=float(args.spawn_clearance_m),
                seed=int(args.spawn_seed),
            )
            print(f"[scene] {len(build_scene._spawn_offsets)} fixed spawn offsets "
                  f"(seed={args.spawn_seed}, margin={args.spawn_margin_m}m)",
                  flush=True)
    else:
        wavelength_m = args.terrain_wavelength * 1e-3
        heightmap, rms_m, peak_m = generate_single_wavelength_heightmap(
            wavelength_m=wavelength_m,
            amplitude_m=args.terrain_amplitude,
            seed=args.terrain_seed,
        )
        np.save(os.path.join(run_dir, "terrain_heightmap.npy"), heightmap)
        # Stash the heightmap on the build_scene module so the cost
        # function can read it for terrain-aware clearance computation.
        # (Cleaner than threading it through every call site.)
        build_scene._last_heightmap     = heightmap
        build_scene._last_arena_size_m  = 1.0
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
    # activate_contact_sensors=True applies the PhysX contact-reporter API
    # to every collision body in the spawned USD. Required for the foot
    # ContactSensor below — without it, the sensor's _initialize_impl()
    # raises "could not find any bodies with contact reporter API".
    spawn_cfg = sim_utils.UsdFileCfg(usd_path=USD_PATH,
                                     activate_contact_sensors=True)
    spawn_cfg.func("/World/Centipede", spawn_cfg, translation=(0.0, 0.0, 0.05))
    # Suppress the MJCF importer's worldBody floor (verified in test_walk.py)
    wb = stage.GetPrimAtPath("/World/Centipede/worldBody")
    if wb.IsValid():
        wb.SetActive(False)

    # ── Foot contact API setup (belt-and-suspenders) ──
    # UsdFileCfg.activate_contact_sensors=True is supposed to apply
    # PhysxContactReportAPI to every RigidBodyAPI prim under the spawn.
    # Diagnostic runs on this Isaac Lab 4.5 build show that it DID apply
    # the API to the 38 foot rigid bodies — but the ContactSensor's
    # _initialize_impl() still raises "could not find any bodies with
    # contact reporter API". The likely cause is the ContactSensor's
    # prim_path regex `.*foot_[0-9]+_[01]` matching non-rigid-body prims
    # (joint_foot_*, site_foot_*) whose presence confuses the body lookup.
    # We belt-and-suspender the API application here:
    #   1. Limit application strictly to the 38 foot RigidBodyAPI prims
    #   2. Also set the threshold attribute (matches what
    #      activate_contact_sensors=True does internally)
    # The ContactSensor prim_path itself is also tightened below to match
    # only the rigid-body feet.
    try:
        import re as _re
        from pxr import PhysxSchema as _PhysxSchema
        _foot_re = _re.compile(r"foot_\d+_[01]$")
        _foot_rb = [p for p in stage.Traverse()
                    if _foot_re.search(p.GetName())
                    and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        _n_added = _n_already = _n_threshold = 0
        for _prim in _foot_rb:
            if _prim.HasAPI(_PhysxSchema.PhysxContactReportAPI):
                _n_already += 1
                _api = _PhysxSchema.PhysxContactReportAPI(_prim)
            else:
                _api = _PhysxSchema.PhysxContactReportAPI.Apply(_prim)
                _n_added += 1
            try:
                _attr = _api.CreateThresholdAttr()
                _attr.Set(0.0)
                _n_threshold += 1
            except Exception:
                pass
        print(f"[scene] foot contact-report API: {len(_foot_rb)} rigid-body "
              f"feet found, {_n_already} already had API, {_n_added} added by "
              f"workaround, threshold set on {_n_threshold}",
              flush=True)
        # Sample a couple of foot paths so we can verify the ContactSensor
        # regex matches them.
        for _p in _foot_rb[:3]:
            print(f"[scene]   sample foot prim: {_p.GetPath()}", flush=True)
    except Exception as _e:
        import traceback as _tb
        print(f"[scene] WARN: foot contact-report API workaround failed: "
              f"{_e}. ContactSensor init will likely raise.", flush=True)
        _tb.print_exc()

    # ── Foot contact sensor (2026-05-04) ──
    # Without this, foot_force is hardcoded to zero and the peak_F/W
    # constraint is trivially satisfied (a paper-blocker per the audit).
    # Match all 38 foot prims (foot_0_0 .. foot_18_1) wherever they
    # ended up in the USD hierarchy. The MJCF importer puts them at
    # `/World/Centipede/link_body_X/foot_<n>_<side>` based on the
    # `getBypassRenderSkelMeshProcessing` log we saw in test_walk.
    try:
        from isaaclab.sensors import ContactSensorCfg, ContactSensor
        # NOTE: prim_path tightened 2026-05-04. The previous pattern
        # `/World/Centipede/.*foot_[0-9]+_[01]` re.fullmatched not only
        # the 38 foot rigid bodies but also 38 `joint_foot_*` joint prims
        # under /World/Centipede/joints/ — joints don't have RigidBodyAPI,
        # so the sensor's body lookup discarded them and emitted "could
        # not find any bodies with contact reporter API". The new pattern
        # forces the parent dir to be `link_body_0`, which is where the
        # MJCF importer puts every rigid body in this asset.
        foot_contact_cfg = ContactSensorCfg(
            prim_path="/World/Centipede/link_body_0/foot_[0-9]+_[01]",
            update_period=0.0,
            history_length=1,
            force_threshold=0.0,
        )
        foot_contact_sensor = ContactSensor(cfg=foot_contact_cfg)
        build_scene._foot_contact_sensor = foot_contact_sensor
        print("[scene] foot ContactSensor created "
              "(prim_path=/World/Centipede/link_body_0/foot_<n>_<side>)",
              flush=True)
    except Exception as e:
        print(f"[scene] WARN: foot ContactSensor creation failed: {e}. "
              f"peak_F/W will read 0 for every trial.", flush=True)
        build_scene._foot_contact_sensor = None

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

    Fields used by compute_cost (matching the MuJoCo recorder PLUS new
    biology-specific fields):
      times                — (T,)   sim time at each sample
      body_yaw_q, _tgt     — (T, N_BODY_JOINTS)
      leg_q,      _tgt     — (T, N_LEGS, 2, N_LEG_DOF)
      pitch_q,    _tgt     — (T, N_PITCH)
      com_pos              — (T, 3) approximate COM (we use root_pos for speed)
      foot_force           — (T, N_LEGS, 2, 3)
      ── BIOLOGICAL EXTENSIONS (2026-05-03) ──
      root_quat            — (T, 4) root orientation as (w,x,y,z); used
                             to compute body roll RMS (planarity check).
      joint_torque         — (T, n_joints) applied joint torques; used
                             for torque-jerk smoothness penalty AND for
                             cost-of-transport (work / distance).
      joint_vel            — (T, n_joints) joint velocities; used with
                             joint_torque to compute mechanical work.
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
        # ── BIO EXTENSIONS ──
        self.root_quat    = []   # (T, 4) wxyz
        self.joint_torque = []   # (T, n_joints) applied torque
        self.joint_vel    = []   # (T, n_joints) joint velocity

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
        self.root_quat.clear()
        self.joint_torque.clear()
        self.joint_vel.clear()
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
        # ── BIO EXTENSIONS ──
        # Root orientation (w, x, y, z) — used for body roll RMS.
        try:
            root_q = art.data.root_quat_w[0].cpu().numpy()
        except Exception:
            root_q = np.array([1.0, 0.0, 0.0, 0.0])
        # Applied joint torques + joint velocities — used for torque-jerk
        # smoothness penalty and for cost-of-transport.
        try:
            jtau = art.data.applied_torque[0].cpu().numpy()
        except Exception:
            try:
                jtau = art.data.computed_torque[0].cpu().numpy()
            except Exception:
                jtau = np.zeros_like(all_pos)
        try:
            jvel = art.data.joint_vel[0].cpu().numpy()
        except Exception:
            jvel = np.zeros_like(all_pos)

        self.times.append(float(t))
        self.body_yaw_q.append(body_q.copy())
        self.body_yaw_tgt.append(np.asarray(body_tgt).copy())
        self.leg_q.append(leg_q.copy())
        self.leg_tgt.append(np.asarray(leg_tgt).copy())
        self.pitch_q.append(pitch_q.copy())
        self.pitch_tgt.append(pitch_tgt.copy())
        self.com_pos.append(com.copy())
        self.root_quat.append(root_q.copy())
        self.joint_torque.append(jtau.copy())
        self.joint_vel.append(jvel.copy())
        # ── Foot contact force (2026-05-04) ──
        # Read from the ContactSensor created at scene build time. The
        # sensor matches all `foot_<n>_<side>` prims and returns net
        # contact forces in world frame. We store them as (n_feet, 3).
        # If the sensor wasn't created (older scene config), fall back
        # to zeros (preserves the original placeholder behaviour).
        fcs = getattr(build_scene, "_foot_contact_sensor", None)
        if fcs is not None:
            try:
                # net_forces_w: (n_envs=1, n_bodies, 3) torch tensor
                fw = fcs.data.net_forces_w[0].cpu().numpy()
                self.foot_force.append(fw.copy())
            except Exception:
                self.foot_force.append(np.zeros((1, 3)))
        else:
            self.foot_force.append(np.zeros((1, 3)))


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


def reset_robot_and_controller(robot, controller, sim, n_settle_steps=8,
                                  spawn_offset_xyz=None):
    """Place the robot back at spawn and clear all controller state.

    `spawn_offset_xyz`: optional (x, y, z) offset from default spawn.
    Used by multi-terrain mode to teleport the robot to a different
    terrain region between sub-trials. None = default spawn.
    """
    n_j = robot.num_joints
    dev = robot.device

    # 1. Hard-reset sim state
    zero_q = torch.zeros(1, n_j, device=dev)
    zero_v = torch.zeros(1, n_j, device=dev)
    robot.write_joint_state_to_sim(zero_q, zero_v)
    root_state = robot.data.default_root_state.clone()
    if spawn_offset_xyz is not None:
        # Offset from default spawn position.
        root_state[:, 0] += float(spawn_offset_xyz[0])
        root_state[:, 1] += float(spawn_offset_xyz[1])
        root_state[:, 2] += float(spawn_offset_xyz[2])
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=dev))

    # 2. Reset controller phases / latched references
    controller.head_yaw_ref = None
    controller.body_phases  = None
    controller.leg_phases   = None
    controller._cpg_initialized = False
    # 2b. Clear PD-tracking target history (see impedance_controller.py
    #     reset_target_history()). Avoids spurious target_dot spike on
    #     the first step of a new trial.
    if hasattr(controller, "reset_target_history"):
        controller.reset_target_history()

    # 3. Controller-driven settle: hold the rest pose while PhysX
    #    commits the new state. Use t=0 so settle gates everything off.
    dt = sim.get_physics_dt()
    # Reset the foot ContactSensor (if present) so its buffer is cleared
    # and the per-trial peak F/W reading isn't contaminated by stale
    # settle-time spikes from a previous trial.
    _fcs = getattr(build_scene, "_foot_contact_sensor", None)
    if _fcs is not None:
        try:
            _fcs.reset()
        except Exception:
            pass
    for _ in range(n_settle_steps):
        robot.update(dt)
        controller.step(0.0, dt)
        sim.step(render=False)
        if _fcs is not None:
            try:
                _fcs.update(dt)
            except Exception:
                pass


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

    # Standalone-sensor update wiring (the optimizer doesn't use an
    # InteractiveScene wrapper, so the ContactSensor doesn't get its
    # update() called automatically — we have to drive it ourselves
    # after each physics step so net_forces_w stays current).
    _fcs = getattr(build_scene, "_foot_contact_sensor", None)

    for step in range(n_steps):
        t = step * dt
        robot.update(dt)
        controller.step(t, dt)
        sim.step(render=False)
        if _fcs is not None:
            try:
                _fcs.update(dt)
            except Exception:
                pass
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
    """Biology-faithful cost (lower is better).

    cost = penalties − rewards, where:

      penalties:
        + w_body_track   * max(0, body_yaw_RMSE − tol)
        + w_leg_track    * max(0, leg_RMSE      − tol)
        + w_wobble       * (1 − straightness)
        + w_force        * max(0, peak_F/W − force_limit)
        + w_body_z       * RMS(z − mean(z))                    ← NEW
        + w_body_roll    * RMS(roll_deg)                       ← NEW
        + w_torque_jerk  * mean( (Δτ / Δt)² )                  ← NEW
        + w_cot          * total_work / displacement           ← NEW

      rewards:
        − w_speed        * min(speed_mps, speed_cap_mps)       ← CAPPED
        − w_displacement * displacement_m
        − w_compliance   * min(pitch_compliance_deg, compliance_cap)

    Speed reward saturates at speed_cap_mps so the optimiser cannot
    extract reward by pushing the gait beyond biological speed (~25 mm/s).
    Above the cap the speed term contributes a constant — TPE then has
    to find improvements through smoothness (jerk), planarity (roll),
    height stability (z RMS), or efficiency (CoT) instead of raw rate.
    """
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
    com_z  = np.asarray(recorder.com_pos)[mask, 2]
    if len(com_xy) < 3:
        return BUCKLED_COST, {"buckled": True, "reason": "too short for straightness"}
    displacement_m = float(np.linalg.norm(com_xy[-1] - com_xy[0]))
    seg_lens       = np.linalg.norm(np.diff(com_xy, axis=0), axis=1)
    path_length_m  = float(seg_lens.sum())
    straightness   = float(np.clip(displacement_m / max(path_length_m, 1e-9), 0.0, 1.0))

    active_time = max(float(t[mask][-1] - t[mask][0]), 1e-3)
    speed_mps   = displacement_m / active_time

    # ── BIO METRIC 1: vertical body motion — TWO complementary metrics ──
    # body_z_rms (legacy): absolute z deviation from mean. Penalises any
    #   vertical motion, INCLUDING biological draping over terrain. Wrong
    #   sign for rough-terrain gait quality. Kept for diagnostic continuity.
    # clearance_rms (preferred): RMS of (body_z − local_terrain_z) about
    #   its mean. Constant clearance over varied terrain → low metric →
    #   "good drape." Rigid body over varied terrain → high metric → bad.
    #   Bouncing → high metric → bad. Correct sign in all cases.
    body_z_rms = float(np.std(com_z))

    # Look up local terrain height under the body at each recorded
    # timestep (post-settle). build_scene stashes the heightmap +
    # arena size on the build_scene function object.
    com_xyz = np.asarray(recorder.com_pos)[mask]
    heightmap     = getattr(build_scene, "_last_heightmap", None)
    arena_size_m  = getattr(build_scene, "_last_arena_size_m", 1.0)
    clearance_rms = body_z_rms   # fallback if heightmap missing
    if heightmap is not None and heightmap.size > 0:
        H, W = heightmap.shape
        if H >= 2 and W >= 2:
            # World coords → fractional pixel indices via bilinear interp.
            # Multi-patch mode: shift world coords by the active patch's
            # (x_off, y_off) so the lookup is in the patch's local frame.
            ax = float(getattr(build_scene, "_active_x_offset", 0.0))
            ay = float(getattr(build_scene, "_active_y_offset", 0.0))
            fx = (com_xyz[:, 0] - ax + arena_size_m * 0.5) / arena_size_m * (W - 1)
            fy = (com_xyz[:, 1] - ay + arena_size_m * 0.5) / arena_size_m * (H - 1)
            fx = np.clip(fx, 0.0, W - 1.0)
            fy = np.clip(fy, 0.0, H - 1.0)
            ix0 = np.floor(fx).astype(int)
            iy0 = np.floor(fy).astype(int)
            ix1 = np.minimum(ix0 + 1, W - 1)
            iy1 = np.minimum(iy0 + 1, H - 1)
            tx = fx - ix0
            ty = fy - iy0
            terrain_z = (
                (1 - tx) * (1 - ty) * heightmap[iy0, ix0]
                +     tx  * (1 - ty) * heightmap[iy0, ix1]
                + (1 - tx) *     ty  * heightmap[iy1, ix0]
                +     tx  *     ty  * heightmap[iy1, ix1]
            )
            clearance = com_xyz[:, 2] - terrain_z
            clearance_rms = float(np.std(clearance - clearance.mean()))

    # ── BIO METRIC 2: body roll RMS ──
    # Convert root quat (w,x,y,z) → roll (deg). Real centipedes are
    # planar (roll ≈ 0). Soft penalty in degrees.
    rq = np.asarray(recorder.root_quat)
    if rq.ndim == 2 and rq.shape[1] == 4:
        rq = rq[mask]
        w, x, y, z = rq[:, 0], rq[:, 1], rq[:, 2], rq[:, 3]
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll_rad = np.arctan2(sinr, cosr)
        body_roll_rms_deg = float(np.degrees(np.sqrt(np.mean(roll_rad ** 2))))
    else:
        body_roll_rms_deg = 0.0

    # ── BIO METRIC 3: torque jerk + ── BIO METRIC 4: cost of transport ──
    # Both metrics use the recorder's joint_torque (T, n_joints) and
    # joint_vel (T, n_joints) arrays. We must mask BOTH consistently —
    # the previous version masked jt then compared its shape to the
    # UNMASKED jv, which always failed since the masked jt is shorter.
    # That silently zeroed out the CoT term in every trial. (Fixed
    # 2026-05-04 — see BIO_REFERENCES.md §5.6.)
    jt_full = np.asarray(recorder.joint_torque)
    jv_full = np.asarray(recorder.joint_vel)
    have_torque  = (jt_full.ndim == 2 and jt_full.shape[0] >= 2)
    have_vel     = (jv_full.ndim == 2 and jv_full.shape[0] >= 2)
    have_compat  = (have_torque and have_vel
                    and jt_full.shape == jv_full.shape)

    if have_torque:
        jt_post = jt_full[mask]
        dt_record = max(float(np.median(np.diff(t))), 1e-4)
        torque_diff = np.diff(jt_post, axis=0) / dt_record
        torque_jerk = float(np.mean(torque_diff ** 2))
    else:
        torque_jerk = 0.0

    if have_compat:
        jt_post = jt_full[mask]
        jv_post = jv_full[mask]
        dt_record = max(float(np.median(np.diff(t))), 1e-4)
        # Instantaneous mechanical power per joint ≈ |τ · q̇|
        joint_power = np.abs(jt_post * jv_post)
        total_work = float(np.sum(joint_power) * dt_record)
        cot = total_work / max(displacement_m, 1e-3)
    else:
        total_work = 0.0
        cot = 0.0

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

    # ── SPEED-CAP REWARD (deprecated; kept for parts-dict compat) ──
    speed_cap = limits.get("speed_cap_mps", 1.0)
    speed_capped = min(speed_mps, speed_cap)

    # ──────────────────────────────────────────────────────────────────────
    # COST: constraint-based reformulation (2026-05-04)
    # ──────────────────────────────────────────────────────────────────────
    # Per the design discussion, the cost is now structured as:
    #   minimize  CoT (J/m)   ← single biological objective with units
    #   subject to constraints that reflect physical/biological feasibility.
    #
    # Hard-violation cost: BUCKLED_COST + linear violation penalty (so TPE
    # gets a gradient back toward feasibility). Within the feasible region,
    # cost = CoT only.
    #
    # The OLD weighted-sum cost is preserved as `cost_legacy` in the parts
    # dict for diagnostic comparison, but is NOT returned to TPE.
    # ──────────────────────────────────────────────────────────────────────
    nominal_clearance = float(limits.get("nominal_clearance_m", 0.0258))
    commanded_velocity = float(limits.get("commanded_velocity_mps", 0.040))

    velocity_quality       = speed_mps / max(commanded_velocity, 1e-6)
    # Use TERRAIN-AWARE clearance variation (rewards body drape, penalises
    # rigidity AND bouncing). Falls back to body_z RMS only if no heightmap
    # was stashed (e.g. unusual scene config).
    z_rms_ratio            = body_z_rms    / max(nominal_clearance, 1e-6)  # diagnostic only
    clearance_rms_ratio    = clearance_rms / max(nominal_clearance, 1e-6)

    min_velocity_quality      = float(limits.get("min_velocity_quality", 0.3))
    max_clearance_rms_ratio   = float(limits.get("max_clearance_rms_ratio", 0.10))
    max_roll_deg              = float(limits.get("max_roll_deg", 5.0))

    violations = {}
    if velocity_quality < min_velocity_quality:
        violations["velocity_quality"] = (
            min_velocity_quality - velocity_quality)
    if clearance_rms_ratio > max_clearance_rms_ratio:
        violations["clearance_rms_ratio"] = (
            clearance_rms_ratio - max_clearance_rms_ratio)
    if body_roll_rms_deg > max_roll_deg:
        violations["body_roll_deg"] = body_roll_rms_deg - max_roll_deg
    if peak_fw > limits["force_limit"]:
        violations["force"] = peak_fw - limits["force_limit"]

    violation_total = sum(violations.values())
    constraint_penalty = 100.0 * violation_total

    if violation_total > 0:
        # Trial is infeasible. Cost = CoT + linear violation penalty so
        # TPE gets a gradient back toward feasibility.
        cost = cot + constraint_penalty
    else:
        # All constraints satisfied. Return CoT alone.
        cost = cot

    # Legacy weighted-sum cost (DIAGNOSTIC ONLY — not used by optimizer):
    cost_legacy = (
        weights["w_body_track"]   * body_excess
      + weights["w_leg_track"]    * leg_excess
      + weights["w_wobble"]       * wobble
      + weights["w_force"]        * force_excess
      + weights["w_body_z"]       * body_z_rms
      + weights["w_body_roll"]    * body_roll_rms_deg
      + weights["w_torque_jerk"]  * torque_jerk
      + weights["w_cot"]          * cot
      - weights["w_speed"]        * speed_capped
      - weights["w_displacement"] * displacement_m
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
        speed_capped_mps=speed_capped,
        peak_force_over_weight=peak_fw,
        force_excess=force_excess,
        pitch_compliance_deg=compliance_deg,
        pitch_compliance_reward_deg=compliance_reward_d,
        body_z_rms_m=body_z_rms,                        # legacy / diagnostic
        clearance_rms_m=clearance_rms,                  # NEW terrain-aware
        clearance_rms_ratio=clearance_rms_ratio,        # NEW dimensionless
        body_roll_rms_deg=body_roll_rms_deg,
        torque_jerk=torque_jerk,
        total_work_J=total_work,
        cost_of_transport=cot,
        velocity_quality=velocity_quality,
        z_rms_ratio=z_rms_ratio,
        constraint_violations=";".join(
            f"{k}={v:.4f}" for k, v in violations.items()) if violations else "",
        n_violations=len(violations),
        constraint_penalty=constraint_penalty,
        cost_legacy=cost_legacy,
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
        w_displacement = args.w_displacement,
        w_compliance   = args.w_compliance,
        # ── BIO 2026-05-03 ──
        w_body_z       = args.w_body_z,
        w_body_roll    = args.w_body_roll,
        w_torque_jerk  = args.w_torque_jerk,
        w_cot          = args.w_cot,
    )
    # ── Commanded velocity calibration (analytical from YAML) ────────────
    # v_commanded ≈ 2 · L_proj · sin(A_yaw) · f
    #   L_proj  = projected leg length on ground (~35 mm for our centipede;
    #             can be measured by walking the kinematic chain in the
    #             URDF / MJCF). Defaults to 0.035 m.
    #   A_yaw   = leg_wave.amplitudes[0] (hip yaw amplitude in rad).
    #   f       = body_wave.frequency (Hz).
    # The optimizer uses this to compute velocity_quality = v_actual /
    # v_commanded as a dimensionless emergent execution metric. See
    # BIO_REFERENCES.md §0.
    import yaml as _yaml
    with open(CFG_PATH, "r", encoding="utf-8") as _fh:
        _cfg = _yaml.safe_load(_fh)
    _A_yaw = float(_cfg.get("leg_wave", {}).get("amplitudes", [0.6])[0])
    _freq  = float(_cfg.get("body_wave", {}).get("frequency", 1.0))
    _L_proj = float(getattr(args, "leg_proj_length_m", 0.035))
    commanded_velocity_mps = 2.0 * _L_proj * math.sin(_A_yaw) * _freq

    limits = dict(
        track_tol_body_deg = args.body_tol_deg,
        track_tol_leg_deg  = args.leg_tol_deg,
        force_limit        = args.force_limit,
        compliance_cap_deg = args.compliance_cap_deg,
        speed_cap_mps      = args.speed_cap_mps,
        # ── Constraint thresholds for the new constraint-based cost ──
        commanded_velocity_mps  = commanded_velocity_mps,
        nominal_clearance_m     = float(getattr(args, "nominal_clearance_m", 0.0258)),
        min_velocity_quality    = float(getattr(args, "min_velocity_quality", 0.3)),
        max_z_rms_ratio         = float(getattr(args, "max_z_rms_ratio", 0.10)),
        max_clearance_rms_ratio = float(getattr(args, "max_clearance_rms_ratio", 0.10)),
        max_roll_deg            = float(getattr(args, "max_roll_deg", 5.0)),
    )
    print(f"[bio-opt] weights: {weights}", flush=True)
    print(f"[bio-opt] limits : {limits}",  flush=True)
    print(f"[bio-opt] commanded velocity (analytical): "
          f"{commanded_velocity_mps*1000:.1f} mm/s = "
          f"{commanded_velocity_mps/0.1025:.2f} BL/s "
          f"(at A_yaw={_A_yaw:.2f}rad, f={_freq:.2f}Hz, L_proj={_L_proj*1000:.0f}mm)",
          flush=True)

    if args.resume:
        storage_path = args.resume
        run_dir = os.path.dirname(os.path.abspath(storage_path))
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = "flat" if args.flat else f"rough_wl{int(round(args.terrain_wavelength))}"
        # BIOLOGICAL run-dir prefix: impedance_bio_* so it doesn't collide
        # with the speed-prioritised optimiser's impedance_bay_* runs.
        # Optional --run-tag inserts an extra distinguishing label.
        run_tag = f"_{args.run_tag}" if args.run_tag else ""
        run_dir = os.path.join(OUTPUT_ROOT, f"impedance_bio_{tag}{run_tag}_{ts}")
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

        # ── Multi-spawn evaluation (Flavor A) ────────────────────────────
        # If --arena is set, build_scene populated _spawn_offsets with N
        # deterministic (x, y, z) deltas. Loop over them, average the cost.
        # If --arena is NOT set, _spawn_offsets is the singleton [(0, 0, 0)]
        # so this loop reduces to the original single-trial behaviour.
        spawn_offsets = list(getattr(build_scene, "_spawn_offsets",
                                     [(0.0, 0.0, 0.0)]))
        per_spawn_costs = []
        per_spawn_parts = []
        per_spawn_buckled = []

        # If multi-wavelength tiled mode is on, build_scene stashed
        # per-patch heightmaps. Switch the active patch (heightmap +
        # arena_size + active_xy_offset) BEFORE each spawn so compute_cost
        # reads the right terrain for clearance.
        terrain_patches = getattr(build_scene, "_terrain_patches", None)

        t0 = time.time()
        for s_idx, spawn_off in enumerate(spawn_offsets):
            # Switch active terrain patch for this spawn
            if terrain_patches is not None and s_idx < len(terrain_patches):
                patch = terrain_patches[s_idx]
                build_scene._last_heightmap    = patch["heightmap"]
                build_scene._last_arena_size_m = float(patch["world_size"])
                build_scene._active_x_offset   = float(patch["x_off"])
                build_scene._active_y_offset   = float(patch["y_off"])

            # Reset robot + controller + recorder for THIS spawn.
            reset_robot_and_controller(robot, controller, sim,
                                       spawn_offset_xyz=spawn_off)
            recorder.reset()

            buckled_s, reason_s = run_trial(sim, robot, controller, recorder,
                                            args.duration)
            cost_s, parts_s = compute_cost(buckled_s, reason_s, recorder,
                                           weights, limits,
                                           settle_s=settle_s)
            per_spawn_costs.append(float(cost_s))
            per_spawn_parts.append(parts_s)
            per_spawn_buckled.append(bool(parts_s.get("buckled", False)))

            # Multi-wavelength mode: prune the trial as soon as any patch
            # buckles — the user wants "skip if any fails," and continuing
            # would just waste sim time on a trial Optuna will discard.
            if (terrain_patches is not None
                and per_spawn_buckled[-1]
                and getattr(args, "wavelengths_mm", "").strip()):
                wall = time.time() - t0
                wls = (args.wavelengths_mm or "").split(",")
                wl_failed = wls[s_idx] if s_idx < len(wls) else "?"
                print(f"  [trial {trial.number:4d}] PRUNED on patch[{s_idx}] "
                      f"λ={wl_failed.strip()}mm: {reason_s}  ({wall:.1f}s)",
                      flush=True)
                row = {**params, "trial": trial.number, "wall_s": wall,
                       "pruned_at_patch": s_idx,
                       "pruned_reason": reason_s,
                       "wavelength_mm_failed": wl_failed.strip()}
                trial_rows.append(row)
                _dump_csv(csv_path, trial_rows)
                raise optuna.TrialPruned()
        wall = time.time() - t0

        # ── Aggregate ──
        cost = float(np.mean(per_spawn_costs))
        any_buckled = any(per_spawn_buckled)

        # Build a single "parts" record for CSV. Numeric parts: mean across
        # spawns. String/bool parts: take spawn 0 (and add per-spawn columns
        # for full audit).
        parts = {}
        for k, v in per_spawn_parts[0].items():
            try:
                vals = [float(p[k]) for p in per_spawn_parts]
                parts[k] = float(np.mean(vals))
            except (TypeError, ValueError, KeyError):
                parts[k] = v
        # Per-spawn cost & key metrics, suffixed _spawn{i}
        for i, p in enumerate(per_spawn_parts):
            parts[f"cost_spawn_{i}"]            = per_spawn_costs[i]
            parts[f"speed_mps_spawn_{i}"]       = p.get("speed_mps", 0.0)
            parts[f"velocity_quality_spawn_{i}"]= p.get("velocity_quality", 0.0)
            parts[f"clearance_rms_m_spawn_{i}"] = p.get("clearance_rms_m", 0.0)
            parts[f"buckled_spawn_{i}"]         = int(per_spawn_buckled[i])
        parts["cost_mean"]    = cost
        parts["cost_max"]     = float(np.max(per_spawn_costs))
        parts["cost_std"]     = float(np.std(per_spawn_costs))
        parts["n_spawns"]     = len(spawn_offsets)
        parts["n_buckled"]    = int(sum(per_spawn_buckled))
        parts["buckled"]      = any_buckled

        if any_buckled:
            print(f"  [trial {trial.number:4d}] BUCKLE in "
                  f"{sum(per_spawn_buckled)}/{len(spawn_offsets)} spawns  "
                  f"cost_mean={cost:.1f}  "
                  f"costs={[f'{c:.2f}' for c in per_spawn_costs]}  "
                  f"({wall:.1f}s)", flush=True)
        else:
            mean_speed = float(np.mean([p.get('speed_mps', 0.0)
                                        for p in per_spawn_parts]))
            mean_strt  = float(np.mean([p.get('straightness', 0.0)
                                        for p in per_spawn_parts]))
            print(f"  [trial {trial.number:4d}] cost={cost:7.3f}  "
                  f"(spawns={[f'{c:.2f}' for c in per_spawn_costs]})  "
                  f"speed={mean_speed*1000:5.1f}mm/s  "
                  f"strt={mean_strt:.3f}  "
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
            "cost":         best.value,
            "params":       best.params,
            "n_total_trials": len(study.trials),
            "n_non_buckled":  len(ok),
            "elapsed_min":    elapsed / 60.0,
            "tune_distal":    args.tune_distal,
            "tune_pitch":     args.tune_pitch,
            "terrain_flat":   args.flat,
            "terrain_wavelength_mm": args.terrain_wavelength,
            "terrain_amplitude_m":   args.terrain_amplitude,
            "terrain_seed":          args.terrain_seed,
            "arena":          bool(args.arena),
            "n_spawns":       int(args.n_spawns),
            "spawn_seed":     int(args.spawn_seed),
        }, f, indent=2)
    print(f"[opt] wrote {best_yaml_path}", flush=True)

    print(f"\n[opt] done — {len(study.trials)} trials in {elapsed/60:.1f} min")

    # CRITICAL: close the optimizer's Isaac Sim instance BEFORE spawning
    # test_walk.py. Otherwise the GPU / Fabric / RTX context is locked
    # by this process and the subprocess will fail to initialise its
    # own renderer.
    sim_app.close()

    # ── Render the winning trial via test_walk.py (subprocess) ──
    if args.replay_video:
        if args.arena:
            # test_walk.py currently builds single-wavelength rough terrain.
            # Replaying the winning gains there would put the centipede on
            # a DIFFERENT terrain than the multi-block arena it was tuned
            # on, and starting from the default origin spawn instead of any
            # of the deterministic Flavor A spawns. The result would be a
            # misleading video — better to skip and tell the user to use
            # render_top_trials.py once it has arena support.
            print("\n[opt] --replay-video skipped: --arena mode active.\n"
                  "      The optimizer's training arena (multi-block, "
                  f"size={args.arena_size}m) is not yet replayable through "
                  "test_walk.py.\n"
                  "      For arena-faithful replays use:\n"
                  f"        python scripts/optimization/render_top_trials.py "
                  f"--arena-blocks-json {os.path.join(run_dir, 'arena_blocks.json')} "
                  f"--arena-size {args.arena_size} "
                  f"--csv {csv_path} --top 10\n", flush=True)
        else:
            replay_path = os.path.join(run_dir, "best_trial.mp4")
            _render_winner_via_test_walk(best_yaml_path, replay_path,
                                         duration=args.duration,
                                         terrain_flat=args.flat,
                                         terrain_wavelength_mm=args.terrain_wavelength,
                                         terrain_amplitude_m=args.terrain_amplitude)
    return 0


# ─────────────────────────────────────────────────────────────────────────────
# Helpers (CSV + YAML patch + replay)
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

    # Wave envelope: write back to body_wave / leg_wave blocks
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

    Subprocess approach (vs in-process replay) is required because Isaac
    Lab's documented camera-init order is "create Camera, THEN sim.reset()"
    — the optimizer's main() calls sim.reset() once up front, so an
    in-process replay-time Camera ends up with an uninitialised render
    product (99% black frames). test_walk.py has the correct ordering.
    """
    import subprocess
    import shutil
    print(f"\n[replay] spawning test_walk.py to render winner -> {video_path}",
          flush=True)
    backup_path = CFG_PATH + ".opt_replay_backup_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        shutil.copy(CFG_PATH, backup_path)
        shutil.copy(best_yaml_path, CFG_PATH)
        test_walk = os.path.join(PROJECT_ROOT, "scripts", "test_walk.py")
        cmd = [sys.executable, test_walk, "--headless",
               "--duration", str(duration), "--video", video_path,
               "--camera-mode", "chase"]
        if terrain_flat:
            cmd += ["--terrain", "flat"]
        else:
            cmd += ["--terrain", "rough",
                    "--terrain-wavelength", str(terrain_wavelength_mm * 1e-3),
                    "--terrain-amplitude", str(terrain_amplitude_m)]
        print(f"[replay] command: {' '.join(cmd)}", flush=True)
        result = subprocess.run(cmd, capture_output=False)
        if result.returncode != 0:
            print(f"[replay] test_walk.py exited with code {result.returncode}", flush=True)
        else:
            print(f"[replay] video saved to {video_path}", flush=True)
    except Exception as e:
        print(f"[replay] subprocess replay failed: {e}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        if os.path.exists(backup_path):
            shutil.copy(backup_path, CFG_PATH)
            print(f"[replay] restored original YAML from {backup_path}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
