#!/usr/bin/env python3
"""
inspect_joint_props.py — Compare USD joint properties to find a sick one
==========================================================================
The motion diagnostic flagged that everything attached to or feeding
into `link_body_10` is misbehaving:
    joint_body_9            (parent body-yaw of link_body_10)
    joint_pitch_body_10     (pitch joint of link_body_10)
    joint_leg_9_L_0 / _R_0  (leg 9 hip-yaw — both legs of link_body_10)
    joint_leg_9_R_2         (tibia, R side)

…and `joint_leg_9_R_0` specifically is FROZEN (std = 0.003° while other
hip-yaws have ~3.95° std).

The MJCF source for these joints looks identical to their neighbors
(same axes, ranges, damping, inertias, masses). So the bug must be in
the USD that the MJCF importer produced. This script dumps every
PhysX-relevant attribute of the suspect joints AND a known-good
neighbor (joint_leg_8_R_0, joint_leg_10_R_0) so we can diff them.

Usage:
    python scripts/inspect_joint_props.py --headless
"""

import argparse
import os
import sys

parser = argparse.ArgumentParser()
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
USD_PATH   = os.path.join(BASE_DIR, "assets", "centipede.usd")


def main():
    sim_cfg = SimulationCfg(dt=0.004)
    sim = SimulationContext(sim_cfg)

    # Spawn the centipede USD
    spawn_cfg = sim_utils.UsdFileCfg(usd_path=USD_PATH)
    spawn_cfg.func("/World/Centipede", spawn_cfg, translation=(0.0, 0.0, 0.05))
    stage = sim.stage

    # Suppress the floor (matches test_walk.py)
    wb = stage.GetPrimAtPath("/World/Centipede/worldBody")
    if wb.IsValid():
        wb.SetActive(False)

    # Wrap as Articulation so we can also query its tensor index
    robot_cfg = ArticulationCfg(
        prim_path="/World/Centipede/link_body_0",
        spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.05)),
        actuators={
            "all": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit=10.0, velocity_limit=100.0,
                stiffness=0.0, damping=0.0,
            ),
        },
    )
    robot = Articulation(robot_cfg)
    sim.reset()
    robot.reset()

    # Joints we want to inspect.
    #
    # The MJCF importer collapsed compound MJCF joints (two <joint>
    # elements sharing one body pair) into a single multi-DOF
    # PhysicsJoint prim named after the BODY rather than the joint.
    # So the actual USD prims to probe are the *body-named* compounds:
    #   link_leg_9_R_0     ← compound for hip_yaw + leg_roll (R side)
    #   link_pitch_body_10 ← compound for body_pitch + body_roll
    # …not the original MJCF joint names.
    suspects = [
        "joint_body_9",            # standard revolute, sanity-check
        "link_pitch_body_10",      # compound (pitch + roll) — body 10
        "link_leg_9_L_0",          # compound (hip_yaw + roll) — leg 9 L
        "link_leg_9_R_0",          # compound (hip_yaw + roll) — leg 9 R, FROZEN
        "joint_leg_9_R_2",         # standard revolute
    ]
    controls = [
        "joint_body_8",            # standard revolute neighbor
        "joint_body_10",           # standard revolute neighbor
        "link_pitch_body_9",       # compound neighbor
        "link_pitch_body_11",      # compound neighbor
        "link_leg_8_R_0",          # compound neighbor (good)
        "link_leg_10_R_0",         # compound neighbor (good)
        "joint_leg_9_R_1",         # standard revolute on the SAME leg
    ]

    name_to_id = {n: i for i, n in enumerate(robot.joint_names)}
    print(f"\nArticulation has {robot.num_joints} joints, "
          f"{robot.num_bodies} bodies\n")

    # Probe via USD prim attributes (more revealing than tensor)
    from pxr import UsdPhysics, Sdf, Gf, PhysxSchema

    def probe_joint(name):
        """Walk the stage to find a UsdPhysics.Joint prim with this name
        and dump its PhysX-relevant attributes.

        Fall back to a direct path lookup at /World/Centipede/joints/<name>
        and to a stage-wide name match (any prim with this name) — the
        joint may be of a USD type other than UsdPhysics.Joint
        (e.g. PhysicsSphericalJoint, PhysxArticulationJoint).
        """
        target_prim = None
        # 1. The standard search.
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint) and prim.GetName() == name:
                target_prim = prim
                break
        # 2. Direct path lookup if the importer put it under .../joints/.
        if target_prim is None:
            p = stage.GetPrimAtPath(f"/World/Centipede/joints/{name}")
            if p.IsValid():
                target_prim = p
        # 3. Last resort: any prim with this leaf name, regardless of type.
        if target_prim is None:
            for prim in stage.Traverse():
                if prim.GetName() == name:
                    target_prim = prim
                    break
        if target_prim is None:
            print(f"  [{name}] ✗ NOT FOUND IN STAGE (any path/type)")
            return None

        path = target_prim.GetPath().pathString
        out = {"name": name, "path": path}

        joint_api = UsdPhysics.Joint(target_prim)
        out["joint_type"] = target_prim.GetTypeName()

        # Bodies that the joint connects
        try:
            body0_rel = joint_api.GetBody0Rel().GetTargets()
            body1_rel = joint_api.GetBody1Rel().GetTargets()
            out["body0"] = str(body0_rel[0]) if body0_rel else "(none)"
            out["body1"] = str(body1_rel[0]) if body1_rel else "(none)"
        except Exception as e:
            out["bodies"] = f"err: {e}"

        # Joint enabled / locked / type-specific properties
        out["jointEnabled"]   = joint_api.GetJointEnabledAttr().Get() \
                               if joint_api.GetJointEnabledAttr() else "n/a"
        out["collisionEnabled"] = joint_api.GetCollisionEnabledAttr().Get() \
                                 if joint_api.GetCollisionEnabledAttr() else "n/a"
        out["excludeFromArticulation"] = joint_api.GetExcludeFromArticulationAttr().Get() \
                                         if joint_api.GetExcludeFromArticulationAttr() else "n/a"

        # Local frame poses
        try:
            out["localPos0"]  = tuple(joint_api.GetLocalPos0Attr().Get() or (0,0,0))
            out["localPos1"]  = tuple(joint_api.GetLocalPos1Attr().Get() or (0,0,0))
            out["localRot0"]  = tuple(joint_api.GetLocalRot0Attr().Get() or (1,0,0,0))
            out["localRot1"]  = tuple(joint_api.GetLocalRot1Attr().Get() or (1,0,0,0))
        except Exception:
            pass

        # If revolute, get axis + limits
        if target_prim.GetTypeName() == "PhysicsRevoluteJoint":
            rev = UsdPhysics.RevoluteJoint(target_prim)
            try:
                out["axis"]  = str(rev.GetAxisAttr().Get())
                out["lower"] = rev.GetLowerLimitAttr().Get()
                out["upper"] = rev.GetUpperLimitAttr().Get()
            except Exception as e:
                out["limits_err"] = str(e)

        # PhysxJointAPI extras (joint friction, etc.)
        if PhysxSchema.PhysxJointAPI(target_prim):
            px = PhysxSchema.PhysxJointAPI(target_prim)
            try:
                out["px_jointFriction"] = px.GetJointFrictionAttr().Get() \
                                          if px.GetJointFrictionAttr() else "n/a"
            except Exception:
                pass

        # Drive (if any)
        for drive_type in ("angular", "linear"):
            drive_api = UsdPhysics.DriveAPI(target_prim, drive_type)
            if drive_api:
                try:
                    if drive_api.GetTypeAttr().Get() is not None:
                        out[f"drive_{drive_type}_type"]      = drive_api.GetTypeAttr().Get()
                        out[f"drive_{drive_type}_stiffness"] = drive_api.GetStiffnessAttr().Get()
                        out[f"drive_{drive_type}_damping"]   = drive_api.GetDampingAttr().Get()
                        out[f"drive_{drive_type}_max_force"] = drive_api.GetMaxForceAttr().Get()
                except Exception:
                    pass

        # Articulation tensor index
        if name in name_to_id:
            out["articulation_idx"] = name_to_id[name]
        else:
            out["articulation_idx"] = "(not in articulation joint_names)"

        return out

    def print_props(out):
        if out is None:
            return
        print(f"\n──── {out['name']} ────")
        for k in sorted(out.keys()):
            if k == "name":
                continue
            print(f"  {k:30s}: {out[k]}")

    # ── First: enumerate every prim under /World/Centipede/joints/ ──
    # Tells us what names the MJCF importer actually produced. The
    # diagnostic flagged that joint_pitch_body_X and joint_leg_X_Y_0
    # come back as NOT FOUND — this listing will show us where they
    # actually went (if they exist) and what type they got converted
    # to.
    print(f"{'='*70}")
    print(f"ALL JOINT PRIMS UNDER /World/Centipede/joints/")
    print(f"{'='*70}")
    joints_root = stage.GetPrimAtPath("/World/Centipede/joints")
    if joints_root.IsValid():
        all_joint_prims = list(joints_root.GetChildren())
        print(f"Found {len(all_joint_prims)} children under /joints/")
        # Group by joint type so we can spot patterns
        from collections import defaultdict
        by_type = defaultdict(list)
        for p in all_joint_prims:
            by_type[p.GetTypeName()].append(p.GetName())
        for tname, names in sorted(by_type.items()):
            print(f"\n  Type: {tname!r}  ({len(names)} prims)")
            for n in sorted(names)[:8]:
                print(f"    {n}")
            if len(names) > 8:
                print(f"    ... +{len(names)-8} more")
        # Specifically: what's the type of joint_pitch_body_10?
        for name in ("joint_pitch_body_10", "joint_leg_9_R_0",
                     "joint_leg_9_L_0", "joint_pitch_body_9",
                     "joint_pitch_body_11"):
            p = stage.GetPrimAtPath(f"/World/Centipede/joints/{name}")
            if p.IsValid():
                print(f"\n  {name}: type={p.GetTypeName()!r} "
                      f"(IsA UsdPhysics.Joint = {p.IsA(UsdPhysics.Joint)})")
            else:
                # Try anywhere in the stage
                hits = [pp.GetPath().pathString for pp in stage.Traverse()
                        if pp.GetName() == name]
                if hits:
                    print(f"\n  {name}: at path {hits[0]} "
                          f"(type={stage.GetPrimAtPath(hits[0]).GetTypeName()!r})")
                else:
                    print(f"\n  {name}: NOT IN STAGE AT ALL")
    else:
        print("/World/Centipede/joints does not exist.")
    print()

    print(f"{'='*70}")
    print(f"SUSPECT JOINTS")
    print(f"{'='*70}")
    suspect_data = {}
    for n in suspects:
        d = probe_joint(n)
        suspect_data[n] = d
        print_props(d)

    print(f"\n{'='*70}")
    print(f"CONTROL (KNOWN-GOOD) JOINTS — for comparison")
    print(f"{'='*70}")
    control_data = {}
    for n in controls:
        d = probe_joint(n)
        control_data[n] = d
        print_props(d)

    # ── Diff-print: side-by-side compare link_leg_9_R_0 to neighbor ──
    print(f"\n{'='*70}")
    print(f"SIDE-BY-SIDE: link_leg_9_R_0  vs  link_leg_8_R_0")
    print(f"{'='*70}")
    a = suspect_data.get("link_leg_9_R_0") or {}
    b = control_data.get("link_leg_8_R_0") or {}
    if not a:
        print("  link_leg_9_R_0 was not found — skipping diff.")
    if not b:
        print("  link_leg_8_R_0 was not found — skipping diff.")
    keys = sorted(set(a.keys()) | set(b.keys()))
    print(f"  {'attribute':30s}  {'leg_9_R_0':35s}  {'leg_8_R_0':35s}  diff?")
    for k in keys:
        if k in ("name", "path", "articulation_idx"):
            continue
        v9 = a.get(k, "(absent)")
        v8 = b.get(k, "(absent)")
        diff = " ← DIFF" if str(v9) != str(v8) else ""
        print(f"  {k:30s}  {str(v9):35.35s}  {str(v8):35.35s}{diff}")

    # ── Diff-print: side-by-side compare leg_9_R_0 to leg_10_R_0 ──
    print(f"\n{'='*70}")
    print(f"SIDE-BY-SIDE: link_leg_9_R_0  vs  link_leg_10_R_0")
    print(f"{'='*70}")
    a = suspect_data.get("link_leg_9_R_0") or {}
    c = control_data.get("link_leg_10_R_0") or {}
    if not a or not c:
        print("  one of the joints was not found — skipping diff.")
    keys = sorted(set(a.keys()) | set(c.keys()))
    print(f"  {'attribute':30s}  {'leg_9_R_0':35s}  {'leg_10_R_0':35s}  diff?")
    for k in keys:
        if k in ("name", "path", "articulation_idx"):
            continue
        v9 = a.get(k, "(absent)")
        v10 = c.get(k, "(absent)")
        diff = " ← DIFF" if str(v9) != str(v10) else ""
        print(f"  {k:30s}  {str(v9):35.35s}  {str(v10):35.35s}{diff}")

    sim_app.close()


if __name__ == "__main__":
    main()
