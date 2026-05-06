#!/usr/bin/env python3
"""
inspect_mass.py — Dump per-body mass + inertia of the centipede USD
====================================================================
Loads the converted centipede USD the same way test_walk.py does and
prints the mass + diagonal inertia of every link, plus aggregate totals,
so we can compare against the MuJoCo MJCF values.

Expected MJCF baseline (computed from centipede.xml directly):
    Total mass        : 0.002536 kg  (2.536 g)
    Body count        : 231
    Distribution:
      <1e-8 kg          : 96   (placeholder pivot bodies)
      1e-8 .. 1e-5 kg   : 114  (foot / leg-tip)
      1e-5 .. 1e-3 kg   : 21   (the 21 main body segments)

Usage:
    python scripts/inspect_mass.py
"""

import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--detail", action="store_true",
                    help="Print every body, not just the summary.")
parser.add_argument("--top", type=int, default=25,
                    help="With --detail, how many heaviest bodies to print.")

# AppLauncher boot (same pattern as test_walk.py — needed so isaacsim/
# isaaclab modules can be imported below).
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))


def main():
    sim_cfg = SimulationCfg(dt=0.004)
    sim = SimulationContext(sim_cfg)

    # Spawn the centipede USD (same path & translation as test_walk.py)
    usd_path = os.path.join(BASE_DIR, "assets", "centipede.usd")
    spawn_cfg = sim_utils.UsdFileCfg(usd_path=usd_path)
    spawn_cfg.func("/World/Centipede", spawn_cfg, translation=(0.0, 0.0, 0.02))

    stage = sim.stage

    # Suppress the MJCF importer's worldBody floor (same fix as test_walk.py)
    wb = stage.GetPrimAtPath("/World/Centipede/worldBody")
    if wb.IsValid():
        wb.SetActive(False)

    # Build a minimal Articulation (no actuator config needed for mass query).
    robot_cfg = ArticulationCfg(
        prim_path="/World/Centipede/link_body_0",
        spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.02)),
        actuators={
            "all": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit=10.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )
    robot = Articulation(robot_cfg)

    sim.reset()
    robot.reset()

    body_names = list(robot.body_names)
    n_bodies = robot.num_bodies

    # Two API paths to mass/inertia. Try each, fall back gracefully.
    masses = None
    inertias = None
    try:
        # Newer Isaac Lab — masses are on root_physx_view
        view = robot.root_physx_view
        masses = view.get_masses().cpu().numpy()
        try:
            inertias = view.get_inertias().cpu().numpy()
        except Exception:
            inertias = None
    except Exception:
        pass
    if masses is None:
        try:
            masses = robot.data.default_mass.cpu().numpy()
        except Exception:
            pass

    if masses is None:
        print("[ERROR] Could not query body masses through any known API.")
        sim_app.close()
        sys.exit(1)

    masses = np.asarray(masses).reshape(-1)[:n_bodies]
    total_mass = float(masses.sum())

    # Bucket by magnitude, mirroring the MJCF analysis
    n_tiny   = int(np.sum(masses < 1e-8))
    n_small  = int(np.sum((masses >= 1e-8) & (masses < 1e-5)))
    n_med    = int(np.sum((masses >= 1e-5) & (masses < 1e-3)))
    n_big    = int(np.sum(masses >= 1e-3))

    print(f"\n{'='*68}")
    print(f"Isaac Lab USD mass inspection")
    print(f"{'='*68}")
    print(f"USD                  : {usd_path}")
    print(f"Articulation bodies  : {n_bodies}")
    print(f"Total mass           : {total_mass:.6f} kg  ({total_mass*1000:.3f} g)")
    print(f"  bodies < 1e-8 kg     : {n_tiny}  (expected 96)")
    print(f"  bodies 1e-8..1e-5    : {n_small} (expected 114)")
    print(f"  bodies 1e-5..1e-3    : {n_med}  (expected 21)")
    print(f"  bodies >= 1e-3       : {n_big}  (expected 0)")

    expected_total = 0.002536
    err_pct = abs(total_mass - expected_total) / expected_total * 100
    verdict = "OK" if err_pct < 5 else "MISMATCH"
    print(f"\nMJCF expected total  : {expected_total:.6f} kg  (2.536 g)")
    print(f"Difference           : {(total_mass-expected_total)*1000:+.4f} g  "
          f"({err_pct:+.2f}%)  → {verdict}")

    # Inertia summary
    if inertias is not None:
        inertias = np.asarray(inertias).reshape(n_bodies, -1)
        # Take diagonal entries (Ixx, Iyy, Izz) — first 3 if 9, or 3-tuple
        if inertias.shape[1] == 9:
            diag = inertias[:, [0, 4, 8]]
        elif inertias.shape[1] >= 3:
            diag = inertias[:, :3]
        else:
            diag = None
        if diag is not None:
            mean_diag = float(np.mean(diag[diag > 0])) if (diag > 0).any() else 0.0
            min_diag  = float(np.min(diag[diag > 0])) if (diag > 0).any() else 0.0
            max_diag  = float(np.max(diag))
            print(f"\nDiag inertia summary (Ixx,Iyy,Izz across all bodies)")
            print(f"  min positive       : {min_diag:.3e}")
            print(f"  mean positive      : {mean_diag:.3e}")
            print(f"  max                : {max_diag:.3e}")

    if args.detail:
        idx = np.argsort(masses)[::-1]  # heaviest first
        print(f"\nTop {args.top} heaviest bodies:")
        for k in idx[:args.top]:
            mass = masses[k]
            inrt = ""
            if inertias is not None and inertias.shape[1] >= 3:
                d = inertias[k, [0, 4, 8]] if inertias.shape[1] == 9 else inertias[k, :3]
                inrt = f"  I=({d[0]:.2e}, {d[1]:.2e}, {d[2]:.2e})"
            print(f"  [{k:3d}] {body_names[k]:35s}  m={mass:.4e} kg{inrt}")

    print(f"\n{'='*68}")
    sim_app.close()


if __name__ == "__main__":
    main()
