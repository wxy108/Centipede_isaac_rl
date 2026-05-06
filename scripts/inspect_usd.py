#!/usr/bin/env python3
"""
inspect_usd.py — List all joints in the converted centipede USD
================================================================
This tells us what joint names the MJCF importer created, so we can
update kinematics.py to match.

Usage:
    python scripts/inspect_usd.py
"""

import os
import sys

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
usd_path = os.path.join(BASE_DIR, "assets", "centipede.usd")

stage = Usd.Stage.Open(usd_path)

print(f"\n{'='*60}")
print(f"Inspecting: {usd_path}")
print(f"{'='*60}\n")

# Find all joints
joints = []
bodies = []
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Joint):
        joints.append(prim.GetPath().pathString)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        bodies.append(prim.GetPath().pathString)

print(f"Total joints found: {len(joints)}")
print(f"Total rigid bodies: {len(bodies)}")

# Categorize joints by type
body_yaw = [j for j in joints if "joint_body_" in j and "pitch" not in j and "roll" not in j]
pitch = [j for j in joints if "joint_pitch" in j]
roll = [j for j in joints if "joint_roll" in j]
leg = [j for j in joints if "joint_leg" in j]
foot = [j for j in joints if "joint_foot" in j]
other = [j for j in joints if j not in body_yaw + pitch + roll + leg + foot]

print(f"\n--- Body yaw joints ({len(body_yaw)}) ---")
for j in sorted(body_yaw):
    print(f"  {j}")

print(f"\n--- Pitch joints ({len(pitch)}) ---")
for j in sorted(pitch):
    print(f"  {j}")

print(f"\n--- Roll joints ({len(roll)}) ---")
for j in sorted(roll):
    print(f"  {j}")

print(f"\n--- Leg joints ({len(leg)}) ---")
# Just show first few and count
for j in sorted(leg)[:8]:
    print(f"  {j}")
if len(leg) > 8:
    print(f"  ... ({len(leg)} total)")

print(f"\n--- Foot joints ({len(foot)}) ---")
for j in sorted(foot)[:4]:
    print(f"  {j}")
if len(foot) > 4:
    print(f"  ... ({len(foot)} total)")

if other:
    print(f"\n--- Other joints ({len(other)}) ---")
    for j in sorted(other):
        print(f"  {j}")

# Extract just the joint names (last path component)
print(f"\n--- Joint name format examples ---")
for j in sorted(joints)[:10]:
    name = j.split("/")[-1]
    print(f"  path: {j}  →  name: {name}")

sim_app.close()
