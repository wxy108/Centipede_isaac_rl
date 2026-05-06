#!/usr/bin/env python3
"""
diagnose_motion.py — Deep joint-motion diagnostic for the centipede
====================================================================
Runs a fixed walking test with the current `farms_controller.yaml`,
captures every joint's q + commanded target at 200 Hz, then runs a
series of analyses to identify which joints (if any) are misbehaving
and why.

The output directory contains:
    sensor_data.npz         — raw recording (all per-joint signals)
    report.md               — human-readable diagnostic report with
                              every anomaly the script could detect
    plot_body_yaw.png       — body-yaw q vs target per segment
    plot_body_yaw_error.png — tracking RMSE bar per body-yaw joint
    plot_pitch.png          — body-pitch q vs target per segment
    plot_legs_dof0.png      — hip-yaw q vs target, all 38 legs
    plot_legs_dof1.png      — hip-pitch
    plot_motion_stddev.png  — std dev of motion per joint, all groups
    plot_phase_lag.png      — phase lag of body wave between segments
    plot_com_trajectory.png — top-down x-y path of the body root
    plot_rpy.png            — root roll / pitch / yaw over time

Analyses performed:
    1. Per-joint tracking RMSE (q − target). Highlights joints whose
       achieved motion does NOT match commanded motion — the most
       direct sign of "this joint is broken".
    2. Per-joint motion std-dev. Outliers (joints with much more or
       much less motion than their family mean) are flagged.
    3. Body-wave phase lag between consecutive segments via cross-
       correlation. Tells us whether the traveling wave is actually
       propagating, or stationary, or chaotic.
    4. Leg L/R coordination — phase difference between joint_leg_n_L_0
       and joint_leg_n_R_0 should be ~π for an alternating gait.
    5. COM trajectory analysis — distance covered, straightness, mean
       speed, stops/reverses. Tells us whether the gait actually
       translates the robot.
    6. Body roll/pitch in world frame over time. Tells us whether
       the robot is staying upright or progressively tipping.
    7. Sanity check: detect duplicate joint indices in the model
       index (catches name-resolution bugs from earlier iterations).
    8. Per-joint torque magnitude (if recorder captured it).
    9. Spectral analysis of body-yaw motion: dominant frequency
       should match the controller's freq parameter.
   10. CSV export of every per-joint metric (motion_rmse_per_joint.csv).

Usage:
    python scripts/diagnose_motion.py --duration 10
    python scripts/diagnose_motion.py --duration 10 --rough  # rough terrain
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(SCRIPT_DIR, "optimization"))

# ── Argparse + AppLauncher ────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--duration", type=float, default=10.0,
                    help="Seconds of sim to record (default 10).")
parser.add_argument("--record-hz", type=float, default=200.0,
                    help="Sensor sample rate (default 200 Hz).")
parser.add_argument("--rough", action="store_true",
                    help="Use rough terrain (λ=18mm, A=10mm) instead of flat.")
parser.add_argument("--out-dir", default=None,
                    help="Output directory (default outputs/diagnostic_<ts>).")

from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

import torch
import yaml

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

from controllers.impedance_controller import ImpedanceTravelingWaveController
from controllers.kinematics import (
    N_BODY_JOINTS, N_LEGS, N_LEG_DOF, N_PITCH, N_ROLL,
    body_joint_name, pitch_joint_name, leg_joint_name,
)
from terrain import generate_single_wavelength_heightmap, build_terrain_mesh

CFG_PATH = os.path.join(PROJECT_ROOT, "configs", "farms_controller.yaml")
USD_PATH = os.path.join(PROJECT_ROOT, "assets", "centipede.usd")


# ═════════════════════════════════════════════════════════════════════════════
# Scene + sensor capture
# ═════════════════════════════════════════════════════════════════════════════

def build_scene(sim, rough):
    from pxr import UsdGeom, UsdLux, UsdPhysics, Gf
    stage = sim.stage

    UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(300.0)
    dist = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    dist.GetIntensityAttr().Set(800.0)
    UsdGeom.Xformable(dist.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))

    if rough:
        h, rms_m, peak_m = generate_single_wavelength_heightmap(
            wavelength_m=0.018, amplitude_m=0.01, seed=42,
        )
        UsdGeom.Xform.Define(stage, "/World/Terrain")
        build_terrain_mesh(stage, "/World/Terrain/Mesh",
                           heightmap_m=h, world_size=1.0, mesh_resolution=128)
        print(f"[scene] rough terrain  rms={rms_m*1000:.2f}mm  peak={peak_m*1000:.2f}mm")
    else:
        UsdGeom.Xform.Define(stage, "/World/GroundPlane")
        slab = UsdGeom.Cube.Define(stage, "/World/GroundPlane/Slab")
        slab.GetSizeAttr().Set(1.0)
        sx = UsdGeom.Xformable(slab)
        sx.ClearXformOpOrder()
        sx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
        sx.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 0.1))
        UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
        print("[scene] flat ground at z=0")

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
                # Simplified MJCF (scripts/simplify_mjcf.py with default
                # --keep-body-pitch): body roll + leg roll joints removed
                # to eliminate multi-DOF compounds, but body pitch is
                # KEPT (single-DOF) for vertical compliance over terrain.
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


class FullRecorder:
    """Capture every joint's q, qdot, target + root pose + root vel +
    sim time at the given rate."""

    def __init__(self, robot, controller, dt_record):
        self.robot = robot
        self.controller = controller
        self.idx = controller.idx
        self.dt_record = dt_record
        self._next_t = -1.0
        self._buf_t = []
        self._buf_root_pos = []
        self._buf_root_quat = []
        self._buf_root_lvel = []
        self._buf_root_avel = []
        self._buf_body_q = []
        self._buf_body_tgt = []
        self._buf_pitch_q = []
        self._buf_pitch_tgt = []
        self._buf_leg_q = []
        self._buf_leg_tgt = []

    def maybe_record(self, t):
        if t < self._next_t:
            return
        self._next_t = t + self.dt_record

        rb = self.robot
        ix = self.idx
        all_pos = rb.data.joint_pos[0].cpu().numpy()

        body_q   = all_pos[ix.body_joint_ids]
        body_tgt = getattr(self.controller, "last_body_yaw_targets",
                           np.zeros(N_BODY_JOINTS)).copy()
        leg_q    = all_pos[ix.leg_joint_ids]
        leg_tgt  = getattr(self.controller, "last_leg_targets",
                           np.zeros((N_LEGS, 2, N_LEG_DOF))).copy()
        pitch_q  = all_pos[ix.pitch_joint_ids] if ix.has_pitch else np.zeros(0)
        pitch_tgt = getattr(self.controller, "pitch_targets",
                            np.zeros_like(pitch_q)).copy()

        self._buf_t.append(t)
        self._buf_root_pos.append(rb.data.root_pos_w[0].cpu().numpy().copy())
        self._buf_root_quat.append(rb.data.root_quat_w[0].cpu().numpy().copy())
        self._buf_root_lvel.append(rb.data.root_lin_vel_w[0].cpu().numpy().copy())
        self._buf_root_avel.append(rb.data.root_ang_vel_w[0].cpu().numpy().copy())
        self._buf_body_q.append(body_q.copy())
        self._buf_body_tgt.append(body_tgt)
        self._buf_pitch_q.append(pitch_q.copy())
        self._buf_pitch_tgt.append(pitch_tgt)
        self._buf_leg_q.append(leg_q.copy())
        self._buf_leg_tgt.append(leg_tgt)

    def to_arrays(self):
        return dict(
            t=np.asarray(self._buf_t),
            root_pos=np.asarray(self._buf_root_pos),
            root_quat=np.asarray(self._buf_root_quat),
            root_lvel=np.asarray(self._buf_root_lvel),
            root_avel=np.asarray(self._buf_root_avel),
            body_q=np.asarray(self._buf_body_q),
            body_tgt=np.asarray(self._buf_body_tgt),
            pitch_q=np.asarray(self._buf_pitch_q),
            pitch_tgt=np.asarray(self._buf_pitch_tgt),
            leg_q=np.asarray(self._buf_leg_q),
            leg_tgt=np.asarray(self._buf_leg_tgt),
        )


# ═════════════════════════════════════════════════════════════════════════════
# Simulation runner
# ═════════════════════════════════════════════════════════════════════════════

def run_sim(duration, record_hz, rough):
    with open(CFG_PATH, "r", encoding="utf-8") as f:
        cfg_yaml = yaml.safe_load(f)
    dt = float(cfg_yaml.get("simulation", {}).get("timestep", 0.004))
    # Tested: bumping solver iterations from default (4+1) up to (32+8),
    # and dt from 0.004 down to 0.0005 — neither change unfreezes
    # joint_leg_9_R_0. It's a structural PhysX articulation-solver
    # quirk on that one compound joint. Documented as a known issue.
    sim_cfg = SimulationCfg(dt=dt)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[0.5, 0.5, 0.3], target=[0.0, 0.0, 0.0])

    robot = build_scene(sim, rough)
    sim.reset()
    robot.reset()

    print(f"[sim] articulation: {robot.num_joints} joints, {robot.num_bodies} bodies")
    controller = ImpedanceTravelingWaveController(robot, CFG_PATH)
    recorder = FullRecorder(robot, controller, dt_record=1.0 / record_hz)

    n_steps = int(duration / dt)
    log_interval = int(1.0 / dt)
    print(f"[sim] running {duration}s @ dt={dt}, recording @ {record_hz}Hz", flush=True)

    t_start = time.time()
    for step in range(n_steps):
        t = step * dt
        robot.update(dt)
        controller.step(t, dt)
        sim.step(render=False)
        recorder.maybe_record(t)

        if step % log_interval == 0 and step > 0:
            root_pos = robot.data.root_pos_w[0].cpu().numpy()
            print(f"  t={t:.1f}s  pos=({root_pos[0]:+.4f}, "
                  f"{root_pos[1]:+.4f}, z={root_pos[2]:+.4f})", flush=True)

    elapsed = time.time() - t_start
    print(f"[sim] done — {duration}s sim in {elapsed:.1f}s wall ({duration/elapsed:.2f}x rt)")

    return recorder.to_arrays(), controller, dt


# ═════════════════════════════════════════════════════════════════════════════
# Analyses
# ═════════════════════════════════════════════════════════════════════════════

def quat_to_rpy(quat_wxyz):
    """Convert (N, 4) quaternions to (N, 3) roll/pitch/yaw in degrees."""
    w, x, y, z = quat_wxyz[:, 0], quat_wxyz[:, 1], quat_wxyz[:, 2], quat_wxyz[:, 3]
    sinr = 2.0 * (w*x + y*z)
    cosr = 1.0 - 2.0*(x*x + y*y)
    roll = np.degrees(np.arctan2(sinr, cosr))
    sinp = np.clip(2.0 * (w*y - z*x), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))
    siny = 2.0 * (w*z + x*y)
    cosy = 1.0 - 2.0*(y*y + z*z)
    yaw = np.degrees(np.arctan2(siny, cosy))
    return np.stack([roll, pitch, yaw], axis=1)


def _phase_lag_via_xcorr(a, b, dt_rec):
    """Cross-correlation lag (in seconds) of `a` against `b`. Positive
    lag means `a` lags `b`."""
    a = a - a.mean()
    b = b - b.mean()
    if a.std() < 1e-9 or b.std() < 1e-9:
        return float("nan")
    corr = np.correlate(a, b, mode="full")
    lags = np.arange(-len(a) + 1, len(a))
    return float(lags[np.argmax(corr)] * dt_rec)


def analyse(D, controller, dt, settle_s=2.5):
    """Run every analysis and return a dict of findings + per-joint metrics."""
    R = {}
    t = D["t"]
    mask = t >= settle_s
    n_post = int(mask.sum())
    R["n_post_settle_samples"] = n_post
    R["dt_record"] = float(t[1] - t[0]) if len(t) > 1 else float("nan")

    if n_post < 50:
        R["fatal"] = "too few post-settle samples; rerun with longer --duration"
        return R

    body_q   = D["body_q"][mask]
    body_tgt = D["body_tgt"][mask]
    pitch_q  = D["pitch_q"][mask]
    pitch_tgt = D["pitch_tgt"][mask]
    leg_q    = D["leg_q"][mask]    # (T, 19, 2, 4)
    leg_tgt  = D["leg_tgt"][mask]
    root_pos = D["root_pos"][mask]
    rpy      = quat_to_rpy(D["root_quat"][mask])

    # ── 1. Per-joint tracking RMSE ────────────────────────────────────
    body_err = body_q - body_tgt   # (T, 19)
    body_rmse_deg = np.degrees(np.sqrt(np.mean(body_err**2, axis=0)))
    pitch_err = pitch_q - pitch_tgt if pitch_q.size else np.zeros((0,))
    pitch_rmse_deg = (np.degrees(np.sqrt(np.mean(pitch_err**2, axis=0)))
                      if pitch_q.size else np.array([]))
    leg_err = leg_q - leg_tgt
    leg_rmse_deg = np.degrees(np.sqrt(np.mean(leg_err**2, axis=0)))   # (19,2,4)
    R["body_rmse_deg"] = body_rmse_deg.tolist()
    R["pitch_rmse_deg"] = pitch_rmse_deg.tolist()
    R["leg_rmse_deg"]  = leg_rmse_deg.tolist()

    # ── 2. Per-joint motion std-dev (in degrees) ──────────────────────
    body_std_deg  = np.degrees(np.std(body_q,  axis=0))
    pitch_std_deg = np.degrees(np.std(pitch_q, axis=0)) if pitch_q.size else np.array([])
    leg_std_deg   = np.degrees(np.std(leg_q,   axis=0))    # (19,2,4)
    R["body_std_deg"]  = body_std_deg.tolist()
    R["pitch_std_deg"] = pitch_std_deg.tolist()
    R["leg_std_deg"]   = leg_std_deg.tolist()

    # ── Outlier detection: any joint > 3σ above its family mean ──────
    outliers = []
    def _flag(family_label, name_fn, vals):
        if len(vals) < 4:
            return
        mu, sd = float(np.mean(vals)), float(np.std(vals))
        if sd < 1e-6:
            return
        for i, v in enumerate(vals):
            z = (v - mu) / sd
            if abs(z) > 3.0:
                outliers.append(dict(
                    joint=name_fn(i), family=family_label,
                    value=float(v), family_mean=mu,
                    family_std=sd, zscore=float(z),
                ))
    _flag("body_yaw_std",   lambda i: f"joint_body_{i+1}",      body_std_deg)
    _flag("pitch_std",      lambda i: f"joint_pitch_body_{i}",  pitch_std_deg)
    _flag("body_yaw_rmse",  lambda i: f"joint_body_{i+1}",      body_rmse_deg)
    _flag("pitch_rmse",     lambda i: f"joint_pitch_body_{i}",  pitch_rmse_deg)
    # Legs: per-DOF outliers across all 38 legs of the same DOF
    for dof in range(N_LEG_DOF):
        flat = leg_std_deg[:, :, dof].reshape(-1)
        names = [f"joint_leg_{n}_{s}_{dof}" for n in range(N_LEGS) for s in ("L", "R")]
        _flag(f"leg_std_dof{dof}", lambda i: names[i], flat)
        flat_r = leg_rmse_deg[:, :, dof].reshape(-1)
        _flag(f"leg_rmse_dof{dof}", lambda i: names[i], flat_r)
    R["outliers"] = outliers

    # ── 3. Body-wave phase lag between consecutive segments ────────────
    # Cross-correlate joint_body_{i+1} against joint_body_{i+2}. A
    # propagating wave should give a small, consistent positive lag
    # corresponding to the wave's travel time per segment.
    dt_rec = R["dt_record"]
    inter_seg_lag_s = []
    for i in range(N_BODY_JOINTS - 1):
        lag = _phase_lag_via_xcorr(body_q[:, i], body_q[:, i + 1], dt_rec)
        inter_seg_lag_s.append(lag)
    R["inter_seg_lag_s"] = inter_seg_lag_s

    # ── 4. Leg L/R coordination — phase lag joint_leg_n_L_0 vs _R_0 ───
    # An alternating gait gives ~half a period (= 0.5 / freq).
    expected_LR_lag_s = 0.5 / max(controller.freq, 1e-6)
    leg_LR_lag_s = []
    for n in range(N_LEGS):
        lag = _phase_lag_via_xcorr(leg_q[:, n, 0, 0], leg_q[:, n, 1, 0], dt_rec)
        leg_LR_lag_s.append(lag)
    R["leg_LR_lag_s"] = leg_LR_lag_s
    R["expected_LR_lag_s"] = float(expected_LR_lag_s)

    # ── 5. COM trajectory analysis ────────────────────────────────────
    com_xy = root_pos[:, :2]
    displacement_m = float(np.linalg.norm(com_xy[-1] - com_xy[0]))
    seg_lens = np.linalg.norm(np.diff(com_xy, axis=0), axis=1)
    path_length_m = float(seg_lens.sum())
    straightness = float(np.clip(displacement_m / max(path_length_m, 1e-9), 0, 1))
    speed_mps = displacement_m / max(t[mask][-1] - t[mask][0], 1e-3)
    R["displacement_m"]   = displacement_m
    R["path_length_m"]    = path_length_m
    R["straightness"]     = straightness
    R["forward_speed_mps"] = speed_mps

    # ── 6. Roll / pitch in world frame ────────────────────────────────
    R["max_root_roll_deg"]  = float(np.max(np.abs(rpy[:, 0])))
    R["max_root_pitch_deg"] = float(np.max(np.abs(rpy[:, 1])))
    R["yaw_drift_deg"]      = float(rpy[-1, 2] - rpy[0, 2])

    # ── 7. Duplicate-index sanity check (catches name-resolution bugs) ─
    ix = controller.idx
    all_ids = (list(ix.body_joint_ids) + list(ix.pitch_joint_ids)
               + list(ix.roll_joint_ids) + list(ix.leg_joint_ids.reshape(-1)))
    R["n_unique_joint_ids"] = int(len(set(all_ids)))
    R["n_total_joint_ids"]  = int(len(all_ids))

    # ── 9. Spectral analysis: body-yaw dominant frequency ─────────────
    # For each body-yaw joint i (post-settle), find the dominant
    # non-DC frequency in q. Should match controller.freq for joints
    # that are actually being driven by the wave.
    def _dominant_freq(x, dt):
        if len(x) < 8 or np.std(x) < 1e-9:
            return float("nan")
        x = x - x.mean()
        F = np.fft.rfft(x * np.hanning(len(x)))
        freqs = np.fft.rfftfreq(len(x), d=dt)
        # Skip DC bin
        F[0] = 0
        return float(freqs[np.argmax(np.abs(F))])
    body_freqs = [_dominant_freq(body_q[:, i], dt_rec) for i in range(N_BODY_JOINTS)]
    R["body_dominant_hz"] = body_freqs
    R["controller_freq_hz"] = float(controller.freq)

    return R


# ═════════════════════════════════════════════════════════════════════════════
# Plots
# ═════════════════════════════════════════════════════════════════════════════

def make_plots(D, R, controller, out_dir):
    """Generate diagnostic plots."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[plots] matplotlib not installed — skipping plots")
        return

    t = D["t"]

    # ── Body yaw q vs target ──
    fig, ax = plt.subplots(figsize=(12, 6))
    for i in range(N_BODY_JOINTS):
        ax.plot(t, np.degrees(D["body_q"][:, i]),     alpha=0.7,
                label=f"joint_body_{i+1}" if i % 4 == 0 else None)
        ax.plot(t, np.degrees(D["body_tgt"][:, i]), '--', alpha=0.3,
                color=f"C{i % 10}")
    ax.set_xlabel("t (s)"); ax.set_ylabel("angle (deg)")
    ax.set_title("Body-yaw joints: q (solid) vs target (dashed)")
    ax.legend(loc="upper right", fontsize=8, ncol=2); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "plot_body_yaw.png"), dpi=110)
    plt.close(fig)

    # ── Body-yaw RMSE bar (per joint) ──
    fig, ax = plt.subplots(figsize=(11, 4))
    bars = ax.bar(range(1, N_BODY_JOINTS + 1), R["body_rmse_deg"], color="steelblue")
    # Highlight outliers in red
    family_mean, family_std = np.mean(R["body_rmse_deg"]), np.std(R["body_rmse_deg"])
    for i, b in enumerate(bars):
        if family_std > 1e-6 and abs(R["body_rmse_deg"][i] - family_mean) > 3 * family_std:
            b.set_color("red")
    ax.set_xlabel("joint_body_N"); ax.set_ylabel("RMSE (deg)")
    ax.set_title("Body-yaw tracking RMSE per joint (outliers in red)")
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "plot_body_yaw_error.png"), dpi=110)
    plt.close(fig)

    # ── Pitch ──
    if D["pitch_q"].size:
        fig, ax = plt.subplots(figsize=(12, 6))
        for i in range(D["pitch_q"].shape[1]):
            ax.plot(t, np.degrees(D["pitch_q"][:, i]), alpha=0.6,
                    label=f"joint_pitch_body_{i}" if i in (0, 5, 10, 15, 19) else None)
            ax.plot(t, np.degrees(D["pitch_tgt"][:, i]), '--', alpha=0.25,
                    color=f"C{i % 10}")
        ax.set_xlabel("t (s)"); ax.set_ylabel("angle (deg)")
        ax.set_title("Body-pitch joints: q (solid) vs target (dashed)")
        ax.legend(loc="upper right", fontsize=8); ax.grid(True, alpha=0.3)
        fig.tight_layout(); fig.savefig(os.path.join(out_dir, "plot_pitch.png"), dpi=110)
        plt.close(fig)

    # ── Legs DOF 0 + DOF 1 (separate plots, all 38 legs each) ──
    for dof in (0, 1):
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        for n in range(N_LEGS):
            for si, side, ax in zip((0, 1), ("L", "R"), axes):
                ax.plot(t, np.degrees(D["leg_q"][:, n, si, dof]), alpha=0.5)
        for ax, side in zip(axes, ("L", "R")):
            ax.set_ylabel(f"DOF {dof} {side} (deg)"); ax.grid(True, alpha=0.3)
        axes[1].set_xlabel("t (s)")
        axes[0].set_title(f"Leg DOF {dof} (hip {'yaw' if dof == 0 else 'pitch'}) — "
                          f"all {N_LEGS} legs each side")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"plot_legs_dof{dof}.png"), dpi=110)
        plt.close(fig)

    # ── Motion stddev bar across all groups ──
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    axes[0, 0].bar(range(1, N_BODY_JOINTS + 1), R["body_std_deg"], color="steelblue")
    axes[0, 0].set_title("body_yaw  std-dev (deg)"); axes[0, 0].grid(True, alpha=0.3, axis="y")
    if R["pitch_std_deg"]:
        axes[0, 1].bar(range(N_PITCH), R["pitch_std_deg"], color="darkorange")
        axes[0, 1].set_title("body_pitch  std-dev (deg)"); axes[0, 1].grid(True, alpha=0.3, axis="y")
    leg_arr = np.asarray(R["leg_std_deg"])  # (19,2,4)
    axes[1, 0].bar(range(N_LEGS), leg_arr[:, :, 0].mean(axis=1), color="seagreen")
    axes[1, 0].set_title("leg DOF 0 (hip yaw) — std-dev avg L+R per leg")
    axes[1, 0].set_xlabel("leg n"); axes[1, 0].grid(True, alpha=0.3, axis="y")
    axes[1, 1].bar(range(N_LEGS), leg_arr[:, :, 1].mean(axis=1), color="darkred")
    axes[1, 1].set_title("leg DOF 1 (hip pitch) — std-dev avg L+R per leg")
    axes[1, 1].set_xlabel("leg n"); axes[1, 1].grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_motion_stddev.png"), dpi=110)
    plt.close(fig)

    # ── Phase lag between consecutive body segments ──
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.bar(range(1, N_BODY_JOINTS), R["inter_seg_lag_s"], color="purple")
    expected = 1.0 / max(controller.freq, 1e-6) * (1.0 / max(N_BODY_JOINTS - 1, 1)) \
               * controller.n_wave   # crude estimate
    ax.axhline(0, color="black", lw=0.5)
    ax.set_xlabel("between joint_body_N and N+1")
    ax.set_ylabel("phase lag (s)")
    ax.set_title("Body-wave phase lag between consecutive body-yaw joints")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_phase_lag.png"), dpi=110)
    plt.close(fig)

    # ── COM trajectory top-down ──
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(D["root_pos"][:, 0]*1000, D["root_pos"][:, 1]*1000)
    ax.scatter([D["root_pos"][0, 0]*1000], [D["root_pos"][0, 1]*1000],
               s=80, c="green", marker="o", label="start", zorder=5)
    ax.scatter([D["root_pos"][-1, 0]*1000], [D["root_pos"][-1, 1]*1000],
               s=80, c="red", marker="X", label="end", zorder=5)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.set_title(f"COM trajectory  —  speed={R['forward_speed_mps']*1000:.1f} mm/s, "
                 f"straightness={R['straightness']:.3f}")
    ax.legend(); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_com_trajectory.png"), dpi=110)
    plt.close(fig)

    # ── RPY over time ──
    rpy = quat_to_rpy(D["root_quat"])
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, rpy[:, 0], label="roll");  ax.plot(t, rpy[:, 1], label="pitch")
    ax.plot(t, rpy[:, 2], label="yaw")
    ax.axhline(0, color="black", lw=0.5)
    ax.set_xlabel("t (s)"); ax.set_ylabel("angle (deg)")
    ax.set_title("Root roll / pitch / yaw in world frame")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_rpy.png"), dpi=110)
    plt.close(fig)


# ═════════════════════════════════════════════════════════════════════════════
# Report
# ═════════════════════════════════════════════════════════════════════════════

def write_report(D, R, controller, out_dir, args):
    md = []
    md.append(f"# Centipede motion diagnostic\n")
    md.append(f"Generated: {datetime.now().isoformat(timespec='seconds')}  ")
    md.append(f"Sim duration: {args.duration:.1f} s  ")
    md.append(f"Terrain: {'rough (λ=18mm)' if args.rough else 'flat'}  ")
    md.append(f"Sample rate: {args.record_hz:.0f} Hz  ")
    md.append(f"Post-settle samples: {R.get('n_post_settle_samples', '—')}\n")

    if "fatal" in R:
        md.append(f"## FATAL\n{R['fatal']}\n")
        with open(os.path.join(out_dir, "report.md"), "w", encoding="utf-8") as f:
            f.write("\n".join(md))
        return

    # Top-line locomotion summary
    md.append(f"## Locomotion summary\n")
    md.append(f"| Metric | Value |\n|---|---|\n")
    md.append(f"| Forward speed | **{R['forward_speed_mps']*1000:.2f} mm/s** |")
    md.append(f"| Net displacement | {R['displacement_m']*1000:.2f} mm |")
    md.append(f"| COM path length | {R['path_length_m']*1000:.2f} mm |")
    md.append(f"| Straightness | {R['straightness']:.3f} (1.0 = perfect line) |")
    md.append(f"| Max root roll | {R['max_root_roll_deg']:.1f}° |")
    md.append(f"| Max root pitch | {R['max_root_pitch_deg']:.1f}° |")
    md.append(f"| Total yaw drift | {R['yaw_drift_deg']:+.1f}° |\n")

    # Sanity check
    md.append(f"## Sanity: joint-index uniqueness\n")
    if R["n_unique_joint_ids"] == R["n_total_joint_ids"]:
        md.append(f"- ✓ All {R['n_total_joint_ids']} mapped joint indices are distinct.\n")
    else:
        md.append(f"- ✗ **DUPLICATE JOINT INDICES**: "
                  f"{R['n_total_joint_ids']} mapped names → only "
                  f"{R['n_unique_joint_ids']} unique indices. "
                  f"This is a name-resolution bug in `kinematics.py`.\n")

    # Outliers
    md.append(f"## Outlier joints (|z-score| > 3 within their family)\n")
    if not R["outliers"]:
        md.append(f"- ✓ No outlier joints — all motion / RMSE within 3σ of family means.\n")
    else:
        md.append(f"| Joint | Family | Value | Family μ ± σ | z-score |\n|---|---|---|---|---|\n")
        for o in R["outliers"]:
            md.append(f"| `{o['joint']}` | {o['family']} | {o['value']:.3f} | "
                      f"{o['family_mean']:.3f} ± {o['family_std']:.3f} | "
                      f"**{o['zscore']:+.2f}** |")
        md.append("")

    # Body-yaw RMSE per joint
    md.append(f"## Body-yaw tracking RMSE per joint (deg)\n```")
    for i, v in enumerate(R["body_rmse_deg"]):
        md.append(f"  joint_body_{i+1:2d}: {v:6.2f}°")
    md.append("```\n")

    # Leg RMSE — flag any unusually high
    md.append(f"## Leg tracking RMSE (deg) — by DOF, side, leg index\n")
    leg_rmse = np.asarray(R["leg_rmse_deg"])
    for dof in range(N_LEG_DOF):
        md.append(f"### DOF {dof}\n```")
        for n in range(N_LEGS):
            md.append(f"  leg_{n:2d}_L: {leg_rmse[n,0,dof]:6.2f}°    "
                      f"leg_{n:2d}_R: {leg_rmse[n,1,dof]:6.2f}°")
        md.append("```\n")

    # Phase lag analysis
    md.append(f"## Body-wave propagation\n")
    md.append(f"Cross-correlation lag (in seconds) between each pair of "
              f"consecutive body-yaw joints. A propagating wave should "
              f"show small POSITIVE lags of similar magnitude across "
              f"the chain. Negative lags or wildly varying values "
              f"indicate the wave is NOT cleanly propagating.\n```")
    for i, lag in enumerate(R["inter_seg_lag_s"]):
        marker = " <-- ANOMALY" if (np.isfinite(lag) and abs(lag) > 0.5) else ""
        md.append(f"  joint_body_{i+1:2d} → _{i+2:2d}:  {lag:+.3f}s{marker}")
    md.append("```\n")

    # Leg L/R coordination
    md.append(f"## Leg L/R coordination\n")
    md.append(f"Phase lag between left and right hip-yaw of each leg. "
              f"For an alternating gait, this should be ~half the "
              f"gait period = {R['expected_LR_lag_s']:.3f}s for "
              f"freq={R['controller_freq_hz']:.2f}Hz. Major deviations "
              f"from this mean the L/R legs are not coordinated.\n```")
    expected = R["expected_LR_lag_s"]
    for n, lag in enumerate(R["leg_LR_lag_s"]):
        if not np.isfinite(lag):
            md.append(f"  leg {n:2d}: nan (leg likely stationary)")
        else:
            err = abs(abs(lag) - abs(expected))
            marker = " <-- POOR COORDINATION" if err > expected * 0.5 else ""
            md.append(f"  leg {n:2d}: lag={lag:+.3f}s  (expected ≈ ±{expected:.3f}){marker}")
    md.append("```\n")

    # Frequency analysis
    md.append(f"## Spectral analysis (body-yaw dominant frequencies)\n")
    md.append(f"Each driven body-yaw joint should oscillate at the "
              f"controller frequency = **{R['controller_freq_hz']:.2f} Hz**. "
              f"Joints whose dominant frequency differs significantly "
              f"are not being driven by the wave — possibly stuck, "
              f"saturated against a joint limit, or being torqued by "
              f"the wrong controller channel.\n```")
    target = R["controller_freq_hz"]
    for i, f in enumerate(R["body_dominant_hz"]):
        if not np.isfinite(f):
            md.append(f"  joint_body_{i+1:2d}: nan (no motion)")
        else:
            err = abs(f - target) / max(target, 1e-6)
            marker = " <-- OFF-FREQUENCY" if err > 0.30 else ""
            md.append(f"  joint_body_{i+1:2d}: {f:.3f} Hz  "
                      f"(target {target:.2f}){marker}")
    md.append("```\n")

    # Plots index
    md.append(f"## Plots\n")
    for fn in ["plot_body_yaw.png", "plot_body_yaw_error.png", "plot_pitch.png",
               "plot_legs_dof0.png", "plot_legs_dof1.png",
               "plot_motion_stddev.png", "plot_phase_lag.png",
               "plot_com_trajectory.png", "plot_rpy.png"]:
        if os.path.isfile(os.path.join(out_dir, fn)):
            md.append(f"- ![{fn}]({fn})")

    # Heuristic interpretation
    md.append(f"\n## Interpretation hints\n")
    hints = []
    if R["forward_speed_mps"] < 0.005:
        hints.append("- **Speed < 5 mm/s** — the centipede is barely moving. "
                     "Most likely the wave amplitudes are too small or the "
                     "leg gains are insufficient to convert wave motion into "
                     "ground reaction force. Try increasing `body_wave.amplitude` "
                     "and/or `leg_wave.amplitudes[0]` (hip yaw).")
    if R["straightness"] < 0.5:
        hints.append("- **Straightness < 0.5** — the path is wobbly. The "
                     "heading servo or yaw control is not tracking. Investigate "
                     "`body_kp_head`, `head_yaw_gain`, and the heading-servo "
                     "joint (joint_body_1) target.")
    if R["max_root_roll_deg"] > 30 or R["max_root_pitch_deg"] > 30:
        hints.append("- **Large root tilt** — the body is rolling/pitching past "
                     "30°. Body-wave amplitude is likely too high relative to "
                     "leg support, or one side's legs aren't bearing load. "
                     "Check leg DOF 0 std-dev plot for asymmetric L/R motion.")
    if R["outliers"]:
        worst = max(R["outliers"], key=lambda o: abs(o["zscore"]))
        hints.append(f"- **Worst outlier: `{worst['joint']}` ({worst['family']})** "
                     f"with z-score {worst['zscore']:+.2f}. Either this joint is "
                     f"receiving torques meant for a different joint (name-resolution "
                     f"bug) or its physical setup (mass, axis, limits) differs from "
                     f"its siblings. Check the plot for that family.")
    if R["n_unique_joint_ids"] != R["n_total_joint_ids"]:
        hints.append("- **Joint name-resolution bug confirmed**: multiple named "
                     "joints map to the same articulation index. Inspect "
                     "`controllers/kinematics.py`.")
    # Phase lag chain coherence
    inter = np.asarray(R["inter_seg_lag_s"])
    finite = inter[np.isfinite(inter)]
    if finite.size > 0:
        if np.std(finite) > np.abs(np.mean(finite)) * 1.0:
            hints.append("- **Body-wave phase lags are inconsistent** along the "
                         "chain (high std-dev). Wave is not propagating cleanly — "
                         "could mean some segments are out of phase due to "
                         "stiffness mismatch, or the CPG coupling is unstable.")
        if np.mean(finite) < 0:
            hints.append("- **Mean inter-segment lag is NEGATIVE**, i.e. the wave "
                         "appears to travel from tail toward head — the OPPOSITE "
                         "of the intended direction. Sign error somewhere in the "
                         "wave-target computation.")

    if hints:
        md.extend(hints)
    else:
        md.append("- No specific anomalies detected by the heuristics. "
                  "If behavior still looks wrong, inspect the plots.")

    with open(os.path.join(out_dir, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md))


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════

def main():
    if args.out_dir:
        out_dir = args.out_dir
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(PROJECT_ROOT, "outputs",
                               f"diagnostic_{ts}")
    os.makedirs(out_dir, exist_ok=True)
    print(f"[diag] output dir: {out_dir}")

    D, controller, dt = run_sim(args.duration, args.record_hz, args.rough)

    # Save raw
    np.savez_compressed(os.path.join(out_dir, "sensor_data.npz"), **D)
    print(f"[diag] saved sensor_data.npz ({sum(v.nbytes for v in D.values()) / 1024:.0f} KB)")

    R = analyse(D, controller, dt)

    # Save analysis json
    with open(os.path.join(out_dir, "analysis.json"), "w", encoding="utf-8") as f:
        json.dump(R, f, indent=2, default=lambda o: float(o) if isinstance(o, np.floating) else str(o))

    # Per-joint CSVs (easy filtering)
    import csv
    leg_rmse = np.asarray(R["leg_rmse_deg"])
    leg_std  = np.asarray(R["leg_std_deg"])
    with open(os.path.join(out_dir, "motion_rmse_per_joint.csv"), "w",
              encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["family", "joint", "rmse_deg", "std_deg"])
        for i, (r, s) in enumerate(zip(R["body_rmse_deg"], R["body_std_deg"])):
            w.writerow(["body_yaw", f"joint_body_{i+1}", f"{r:.4f}", f"{s:.4f}"])
        for i, (r, s) in enumerate(zip(R["pitch_rmse_deg"], R["pitch_std_deg"])):
            w.writerow(["body_pitch", f"joint_pitch_body_{i}", f"{r:.4f}", f"{s:.4f}"])
        for n in range(N_LEGS):
            for si, side in enumerate(("L", "R")):
                for dof in range(N_LEG_DOF):
                    w.writerow([f"leg_dof{dof}",
                                f"joint_leg_{n}_{side}_{dof}",
                                f"{leg_rmse[n, si, dof]:.4f}",
                                f"{leg_std[n, si, dof]:.4f}"])

    make_plots(D, R, controller, out_dir)
    write_report(D, R, controller, out_dir, args)
    print(f"[diag] wrote report.md and {len(os.listdir(out_dir))} files in {out_dir}")
    sim_app.close()


if __name__ == "__main__":
    main()
