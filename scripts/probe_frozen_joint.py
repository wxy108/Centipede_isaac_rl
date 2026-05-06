#!/usr/bin/env python3
"""
probe_frozen_joint.py — Why is joint_leg_9_R_0 frozen at runtime?
====================================================================
The motion diagnostic flagged joint_leg_9_R_0 as moving with std=0.003°
while its 37 sibling DOF-0 joints all move with std~3.95°. The USD
inspection shows the underlying compound joint (link_leg_9_R_0) is
byte-identical to working neighbors. So the freezing is a runtime
phenomenon, not a USD-config bug.

Three possibilities:
    (a) The controller's torque doesn't reach this DOF index
        (controller / kinematics issue).
    (b) The torque reaches it but PhysX's articulation solver locks
        the DOF due to a constraint cycle or numerical issue.
    (c) Something specific to the compound-joint solver.

This script runs the same gait test for ~3 s and records, every
single physics step, for joint_leg_9_R_0:
    * commanded torque (what the controller WROTE)
    * actual joint position q
    * actual joint velocity qdot
    * indices used for write and read

Same for joint_leg_8_R_0 (working sibling) for direct comparison.

After the run, prints summary stats. The smoking-gun outcomes:

    Outcome A — controller torque is zero or constant for leg_9_R_0
                while non-zero / oscillating for leg_8_R_0:
        => the controller is NOT writing the right index. Bug in
           kinematics or torque-scatter logic.

    Outcome B — controller torque oscillates similarly for both,
                but only leg_8_R_0's q responds:
        => PhysX locks the DOF. The compound multi-DOF joint solver
           is dropping that DOF. Need to either reimport MJCF or
           split the compound into a revolute chain.

    Outcome C — both have similar torque AND similar q response:
        => the diagnostic flagging was spurious. Re-investigate.
"""

import argparse
import os
import sys

import numpy as np

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, PROJECT_ROOT)

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=3.0,
                    help="Seconds to record (default 3, post-warm).")
parser.add_argument("--warmup", type=float, default=2.5,
                    help="Settle+ramp time before recording (default 2.5).")
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

from controllers.impedance_controller import ImpedanceTravelingWaveController

CFG_PATH = os.path.join(PROJECT_ROOT, "configs", "farms_controller.yaml")
USD_PATH = os.path.join(PROJECT_ROOT, "assets", "centipede.usd")


def main():
    import yaml
    from pxr import UsdGeom, UsdLux, UsdPhysics, Gf

    with open(CFG_PATH, "r", encoding="utf-8") as f:
        cfg_yaml = yaml.safe_load(f)
    dt = float(cfg_yaml.get("simulation", {}).get("timestep", 0.004))

    sim_cfg = SimulationCfg(dt=dt)
    sim = SimulationContext(sim_cfg)
    stage = sim.stage
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(300.0)

    # Flat ground
    UsdGeom.Xform.Define(stage, "/World/GroundPlane")
    slab = UsdGeom.Cube.Define(stage, "/World/GroundPlane/Slab")
    slab.GetSizeAttr().Set(1.0)
    sx = UsdGeom.Xformable(slab); sx.ClearXformOpOrder()
    sx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    sx.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(slab.GetPrim())

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
    robot = Articulation(robot_cfg)

    sim.reset(); robot.reset()

    controller = ImpedanceTravelingWaveController(robot, CFG_PATH)

    # Resolve target indices
    name_to_id = {n: i for i, n in enumerate(robot.joint_names)}
    target_names = ["joint_leg_9_R_0", "joint_leg_8_R_0",
                    "joint_leg_10_R_0", "joint_leg_9_L_0"]
    target_ids = {}
    for n in target_names:
        if n in name_to_id:
            target_ids[n] = name_to_id[n]
            print(f"[probe] {n:24s} → articulation index {name_to_id[n]}")
        else:
            print(f"[probe] {n}: NOT in articulation joint_names")

    # Sanity: the controller's leg index for leg 9 R DOF 0 should match.
    leg_9_R_0_via_idx = int(controller.idx.leg_joint_ids[9, 1, 0])
    leg_8_R_0_via_idx = int(controller.idx.leg_joint_ids[8, 1, 0])
    leg_10_R_0_via_idx = int(controller.idx.leg_joint_ids[10, 1, 0])
    leg_9_L_0_via_idx = int(controller.idx.leg_joint_ids[9, 0, 0])
    print(f"[probe] kinematics says:")
    print(f"        leg_joint_ids[ 9, R, 0] = {leg_9_R_0_via_idx}")
    print(f"        leg_joint_ids[ 8, R, 0] = {leg_8_R_0_via_idx}")
    print(f"        leg_joint_ids[10, R, 0] = {leg_10_R_0_via_idx}")
    print(f"        leg_joint_ids[ 9, L, 0] = {leg_9_L_0_via_idx}")

    # Verify they match
    if "joint_leg_9_R_0" in target_ids:
        if leg_9_R_0_via_idx != target_ids["joint_leg_9_R_0"]:
            print(f"  *** MISMATCH: kinematics index ({leg_9_R_0_via_idx}) "
                  f"does NOT equal articulation index "
                  f"({target_ids['joint_leg_9_R_0']}) ***")
        else:
            print(f"  ✓ kinematics index matches articulation for leg_9_R_0")

    # Per-step recording
    n_warm = int(args.warmup / dt)
    n_rec  = int(args.duration / dt)
    rec_t       = np.zeros(n_rec)
    rec_torque  = np.zeros((n_rec, 4))   # leg_9_R_0, leg_8_R_0, leg_10_R_0, leg_9_L_0
    rec_q       = np.zeros((n_rec, 4))
    rec_qdot    = np.zeros((n_rec, 4))

    print(f"\n[probe] warming up for {args.warmup}s …")
    for step in range(n_warm):
        t = step * dt
        robot.update(dt); controller.step(t, dt); sim.step(render=False)
    print(f"[probe] recording for {args.duration}s …")
    for step in range(n_rec):
        t = (n_warm + step) * dt
        robot.update(dt); controller.step(t, dt)

        # Snapshot torque BEFORE sim.step (controller has just written it)
        if "joint_leg_9_R_0" in target_ids:
            rec_torque[step, 0] = float(controller.torque_buf[target_ids["joint_leg_9_R_0"]])
        if "joint_leg_8_R_0" in target_ids:
            rec_torque[step, 1] = float(controller.torque_buf[target_ids["joint_leg_8_R_0"]])
        if "joint_leg_10_R_0" in target_ids:
            rec_torque[step, 2] = float(controller.torque_buf[target_ids["joint_leg_10_R_0"]])
        if "joint_leg_9_L_0" in target_ids:
            rec_torque[step, 3] = float(controller.torque_buf[target_ids["joint_leg_9_L_0"]])

        sim.step(render=False)

        # Read q / qdot AFTER sim.step
        all_pos = robot.data.joint_pos[0].cpu().numpy()
        all_vel = robot.data.joint_vel[0].cpu().numpy()
        rec_t[step] = t
        for k, n in enumerate(("joint_leg_9_R_0", "joint_leg_8_R_0",
                               "joint_leg_10_R_0", "joint_leg_9_L_0")):
            if n in target_ids:
                rec_q[step, k]    = float(all_pos[target_ids[n]])
                rec_qdot[step, k] = float(all_vel[target_ids[n]])

    # Summary
    print(f"\n{'='*72}")
    print(f"PER-JOINT STATS over {args.duration}s ({n_rec} steps)")
    print(f"{'='*72}")
    print(f"{'joint':24s}  {'mean_τ':>10s}  {'std_τ':>10s}  "
          f"{'std_q (deg)':>14s}  {'std_qdot':>10s}  {'pk-pk q (deg)':>14s}")
    for k, n in enumerate(("joint_leg_9_R_0", "joint_leg_8_R_0",
                           "joint_leg_10_R_0", "joint_leg_9_L_0")):
        if n in target_ids:
            tau = rec_torque[:, k]
            q   = rec_q[:, k]
            qd  = rec_qdot[:, k]
            print(f"{n:24s}  {tau.mean():10.3e}  {tau.std():10.3e}  "
                  f"{np.degrees(q.std()):14.3f}  {qd.std():10.3e}  "
                  f"{np.degrees(q.max()-q.min()):14.3f}")

    # Save raw for offline plotting
    out_path = os.path.join(PROJECT_ROOT, "outputs", "probe_frozen_joint.npz")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez_compressed(out_path,
                        t=rec_t, torque=rec_torque, q=rec_q, qdot=rec_qdot,
                        names=np.array(target_names))
    print(f"\n[probe] raw data → {out_path}")

    # Smoking-gun verdict
    if "joint_leg_9_R_0" in target_ids and "joint_leg_8_R_0" in target_ids:
        tau_9 = rec_torque[:, 0]
        tau_8 = rec_torque[:, 1]
        q_9   = rec_q[:, 0]
        q_8   = rec_q[:, 1]
        ratio_tau = tau_9.std() / max(tau_8.std(), 1e-12)
        ratio_q   = q_9.std()   / max(q_8.std(),   1e-12)
        print(f"\n{'='*72}")
        print(f"VERDICT")
        print(f"{'='*72}")
        print(f"  τ std ratio (leg_9_R_0 / leg_8_R_0) = {ratio_tau:.3f}")
        print(f"  q std ratio (leg_9_R_0 / leg_8_R_0) = {ratio_q:.3f}")
        if ratio_tau < 0.1:
            print(f"  → Outcome A: the controller is NOT writing torque to "
                  f"leg_9_R_0 (only {ratio_tau:.1%} of leg_8_R_0's torque). "
                  f"Bug is in kinematics or controller torque scatter.")
        elif ratio_tau > 0.3 and ratio_q < 0.1:
            print(f"  → Outcome B: the controller writes a similar torque to "
                  f"leg_9_R_0 ({ratio_tau:.1%} of leg_8_R_0) but the joint "
                  f"barely moves ({ratio_q:.1%} of leg_8_R_0's motion). "
                  f"PhysX is locking this DOF. Likely a compound-joint "
                  f"articulation-solver issue. Fix needs MJCF re-import.")
        else:
            print(f"  → Outcome C: both torque and motion are similar. The "
                  f"freezing might be transient; try a longer warmup or "
                  f"check whether body 10 is in an unusual pose.")

    sim_app.close()


if __name__ == "__main__":
    main()
