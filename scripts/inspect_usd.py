#!/usr/bin/env python3
"""
inspect_usd.py — List all joints in a converted centipede USD
=============================================================
This tells us what joint names the MJCF importer created, so we can
update kinematics.py to match. It also classifies each physics joint by
its concrete USD schema type so we can assert **single-DOF everywhere**
(no multi-DOF PhysicsJoint / SphericalJoint — the compound the PhysX
solver locks; the documented "leg-10 thrash").

Usage:
    # default = deployed assets/centipede.usd (byte-unchanged behavior)
    python scripts/inspect_usd.py

    # Stage 0.1 model-rebuild probe (GO_STAGE_0_1_MODEL_REBUILD 2026-06-17):
    python scripts/inspect_usd.py \
        --usd outputs/analysis/model_rebuild_2026-06-17/data/centipede_split_probe.usd \
        --expect-joints 287 --require-single-dof
"""

import argparse
import os
import sys

# ── Parse args BEFORE launching Isaac Sim (so --usd selects the stage) ──
_parser = argparse.ArgumentParser(description="Inspect joints in a centipede USD")
_parser.add_argument("--usd", default=None,
                     help="USD path to inspect (default: <repo>/assets/centipede.usd; "
                          "default path => deployed behavior byte-unchanged)")
_parser.add_argument("--expect-joints", type=int, default=None,
                     help="If set, assert total UsdPhysics.Joint count == this value")
_parser.add_argument("--require-single-dof", action="store_true",
                     help="If set, FAIL when any joint is a multi-DOF type "
                          "(Spherical / generic D6 PhysicsJoint)")
_args = _parser.parse_args()

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
usd_path = os.path.abspath(_args.usd) if _args.usd else os.path.join(BASE_DIR, "assets", "centipede.usd")

stage = Usd.Stage.Open(usd_path)

print(f"\n{'='*60}")
print(f"Inspecting: {usd_path}")
print(f"{'='*60}\n")

# Concrete USD physics joint schema typenames that are single-DOF or 0-DOF.
# Anything else (PhysicsSphericalJoint, generic 'PhysicsJoint' D6, ...) is
# multi-DOF-capable = the compound the PhysX solver can lock.
SINGLE_OR_ZERO_DOF_TYPES = {
    "PhysicsRevoluteJoint",   # 1 rotational DOF
    "PhysicsPrismaticJoint",  # 1 translational DOF
    "PhysicsFixedJoint",      # 0 DOF (welded)
}

# Find all joints + bodies, recording concrete schema type per joint.
joints = []          # list of (path, typename)
bodies = []
type_counts = {}
multi_dof = []       # joints whose type is NOT in SINGLE_OR_ZERO_DOF_TYPES
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Joint):
        p = prim.GetPath().pathString
        tn = prim.GetTypeName()
        joints.append((p, tn))
        type_counts[tn] = type_counts.get(tn, 0) + 1
        if tn not in SINGLE_OR_ZERO_DOF_TYPES:
            multi_dof.append((p, tn))
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        bodies.append(prim.GetPath().pathString)

joint_paths = [p for (p, _t) in joints]

print(f"Total joints found: {len(joints)}")
print(f"Total rigid bodies: {len(bodies)}")

print(f"\n--- Joint schema-type breakdown ---")
for tn in sorted(type_counts):
    flag = "" if tn in SINGLE_OR_ZERO_DOF_TYPES else "   <-- MULTI-DOF / not single-DOF"
    print(f"  {tn:28s}: {type_counts[tn]}{flag}")

# Categorize joints by name (body roll vs leg roll kept distinct).
body_yaw = [j for j in joint_paths if "joint_body_" in j and "pitch" not in j and "roll" not in j]
pitch = [j for j in joint_paths if "joint_pitch" in j]
roll_body = [j for j in joint_paths if "joint_roll_body" in j]
roll_leg = [j for j in joint_paths if "joint_roll_leg" in j]
leg = [j for j in joint_paths if "joint_leg" in j]
foot = [j for j in joint_paths if "joint_foot" in j]
categorized = set(body_yaw + pitch + roll_body + roll_leg + leg + foot)
other = [j for j in joint_paths if j not in categorized]

print(f"\n--- Joint name categories ---")
print(f"  body_yaw  (joint_body_*)      : {len(body_yaw)}")
print(f"  body_pitch(joint_pitch_*)     : {len(pitch)}")
print(f"  body_roll (joint_roll_body_*) : {len(roll_body)}")
print(f"  leg       (joint_leg_*)       : {len(leg)}")
print(f"  leg_roll  (joint_roll_leg_*)  : {len(roll_leg)}")
print(f"  foot      (joint_foot_*)      : {len(foot)}")
print(f"  other                         : {len(other)}")
if other:
    for j in sorted(other):
        print(f"      {j}")

print(f"\n--- Joint name format examples ---")
for (j, tn) in sorted(joints)[:6]:
    name = j.split("/")[-1]
    print(f"  [{tn}] {name}")

# ── Assertions ──
print(f"\n{'='*60}")
print("ASSERTIONS")
print(f"{'='*60}")
ok = True

if _args.expect_joints is not None:
    passed = (len(joints) == _args.expect_joints)
    ok = ok and passed
    print(f"  joint count == {_args.expect_joints:<5d}: "
          f"{'PASS' if passed else 'FAIL'} (got {len(joints)})")

# Single-DOF check (reportable always; only FAILs the run under --require-single-dof)
single_dof_ok = (len(multi_dof) == 0)
print(f"  all joints single-DOF      : "
      f"{'PASS' if single_dof_ok else 'FAIL'} "
      f"({len(multi_dof)} multi-DOF joint(s))")
if multi_dof:
    for (p, tn) in multi_dof[:20]:
        print(f"      MULTI-DOF [{tn}] {p}")
    if len(multi_dof) > 20:
        print(f"      ... ({len(multi_dof)} total)")
if _args.require_single_dof:
    ok = ok and single_dof_ok

# Body-roll prim presence (informational; the probe expects 20 in the split model)
print(f"  body-roll joints present   : "
      f"{'YES' if len(roll_body) > 0 else 'NO'} ({len(roll_body)})")

print(f"\n=== inspect_usd verdict: {'PASS' if ok else 'FAIL'} ===")

print("PROBE_DONE finished", flush=True)  # watchdog kill marker (pre-teardown)
sim_app.close()
sys.exit(0 if ok else 1)
