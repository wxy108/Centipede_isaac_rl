#!/usr/bin/env python3
"""
run_standalone.py — Standalone Isaac Lab script to run the centipede
=====================================================================
Usage:
    python scripts/run_standalone.py --mode stand --duration 5
    python scripts/run_standalone.py --mode walk --duration 10
    python scripts/run_standalone.py --mode walk --duration 10 --headless
"""

import argparse
import os
import sys

# ── Parse args BEFORE SimulationApp (Isaac Lab pattern) ──
parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["stand", "walk"], default="walk")
parser.add_argument("--duration", type=float, default=10.0)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--config", default=None)
parser.add_argument("--usd", default="assets/centipede.usd")
args, unknown = parser.parse_known_args()

# ── Isaac Sim must be launched BEFORE any other Isaac imports ──
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": args.headless})

# ── Now safe to import Isaac Lab ──
import torch
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext

# ── Project imports ──
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, BASE_DIR)


def main():
    # ── Simulation context ──
    sim_cfg = SimulationCfg(dt=0.0005)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[0.5, 0.5, 0.3], target=[0.0, 0.0, 0.0])

    # ── Ground plane (simple collision plane, no Nucleus textures needed) ──
    import omni.isaac.core.utils.stage as stage_utils
    from pxr import UsdGeom, UsdPhysics, Gf

    stage = sim.stage
    ground_path = "/World/GroundPlane"
    UsdGeom.Xform.Define(stage, ground_path)
    plane_path = ground_path + "/CollisionPlane"
    plane_geom = UsdGeom.Plane.Define(stage, plane_path)
    plane_geom.GetAxisAttr().Set("Z")
    plane_geom.GetExtentAttr().Set([(-50, -50, 0), (50, 50, 0)])
    # Add collision
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(plane_path))
    # Add ground plane physics scene if not yet present
    scene_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(scene_path).IsValid():
        UsdPhysics.Scene.Define(stage, scene_path)
        physics_scene = UsdPhysics.Scene.Get(stage, scene_path)
        physics_scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.GetGravityMagnitudeAttr().Set(9.81)

    # ── Load centipede from USD ──
    usd_path = os.path.join(BASE_DIR, args.usd)
    if not os.path.isfile(usd_path):
        print(f"ERROR: USD file not found: {usd_path}")
        print(f"Run convert_mjcf.py first to generate it.")
        sim_app.close()
        sys.exit(1)

    # Spawn the USD into the stage
    prim_path = "/World/Centipede"
    cfg_robot = sim_utils.UsdFileCfg(usd_path=usd_path)
    cfg_robot.func(prim_path, cfg_robot, translation=(0.0, 0.0, 0.05))

    # ── Reset simulation (initializes physics) ──
    sim.reset()

    print(f"\n{'='*60}")
    print(f"Centipede USD loaded at {prim_path}")
    print(f"Mode: {args.mode}, Duration: {args.duration}s, dt={sim_cfg.dt}")
    print(f"{'='*60}\n")

    # ── Simulation loop ──
    dt = sim_cfg.dt
    n_steps = int(args.duration / dt)
    log_interval = int(1.0 / dt)  # log every 1 second

    for step in range(n_steps):
        t = step * dt

        # Step physics
        sim.step()

        # Periodic logging
        if step % log_interval == 0 and step > 0:
            print(f"  t={t:.1f}s  (simulation running)")

    # ── Final report ──
    print(f"\n{'='*60}")
    print(f"Simulation complete: {args.duration}s at dt={dt}")
    print(f"{'='*60}")

    sim_app.close()


if __name__ == "__main__":
    main()
