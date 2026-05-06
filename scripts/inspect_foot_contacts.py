#!/usr/bin/env python3
"""
inspect_foot_contacts.py — diagnose the ContactSensor "no bodies with
contact reporter API" error in optimize_impedance_biological.py.

Opens assets/centipede.usd and reports, for every prim whose name
matches the foot regex `foot_\\d+_[01]`:
  * Path
  * Has UsdPhysics.RigidBodyAPI ?
  * Has UsdPhysics.CollisionAPI ?
  * Has PhysxSchema.PhysxContactReportAPI ?

Then summarises which of the three hypotheses is true:
  H1: Foot prims exist but have no RigidBodyAPI → activate_contact_sensors
      silently skipped them.
  H2: Foot prims have no PhysxContactReportAPI → activate_contact_sensors
      did not actually run / is broken on this build.
  H3: Foot prims don't match the regex pattern at all.

Usage:
    python scripts/inspect_foot_contacts.py
"""

import os
import re
import sys

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics
try:
    from pxr import PhysxSchema
    HAS_PHYSX_SCHEMA = True
except ImportError:
    HAS_PHYSX_SCHEMA = False
    print("[warn] PhysxSchema not importable — contact-report API check disabled")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
USD_PATH   = os.path.join(BASE_DIR, "assets", "centipede.usd")

# Same pattern the ContactSensor uses in optimize_impedance_biological.py
SENSOR_REGEX = re.compile(r"foot_\d+_[01]$")

stage = Usd.Stage.Open(USD_PATH)
print(f"\n{'='*72}")
print(f"Inspecting: {USD_PATH}")
print(f"Sensor prim_path pattern: /World/Centipede/.*foot_[0-9]+_[01]")
print(f"{'='*72}\n")

# Find every prim whose leaf name matches the foot pattern
candidates = []
all_foot_named = []  # broader: anything containing "foot" in the name
for prim in stage.Traverse():
    name = prim.GetName()
    if "foot" in name.lower():
        all_foot_named.append(prim)
    if SENSOR_REGEX.search(name):
        candidates.append(prim)

print(f"Prims whose name contains 'foot' (any form): {len(all_foot_named)}")
print(f"Prims matching foot_<int>_[01] regex:        {len(candidates)}")
print()

if not candidates and all_foot_named:
    print("HYPOTHESIS H3 LIKELY TRUE — feet exist but the regex misses them.")
    print("First few foot-like names actually found in the USD:")
    for p in all_foot_named[:10]:
        print(f"  {p.GetPath()}  (name='{p.GetName()}')")
    print()
elif not candidates:
    print("No foot prims of any kind found — model is missing feet entirely.")
    print()

# Per-prim detail
n_rb        = 0
n_collision = 0
n_contact   = 0

print(f"{'path':<70}  RB  COLL  CONTACT")
print(f"{'-'*70}  --  ----  -------")
for prim in candidates[:50]:  # cap at 50 lines for readability
    has_rb       = prim.HasAPI(UsdPhysics.RigidBodyAPI)
    has_coll     = prim.HasAPI(UsdPhysics.CollisionAPI)
    has_contact  = (HAS_PHYSX_SCHEMA
                    and prim.HasAPI(PhysxSchema.PhysxContactReportAPI))
    n_rb        += int(has_rb)
    n_collision += int(has_coll)
    n_contact   += int(has_contact)
    print(f"{str(prim.GetPath()):<70}  {'Y' if has_rb else '.':<2}  "
          f"{'Y' if has_coll else '.':<4}  {'Y' if has_contact else '.':<7}")

if len(candidates) > 50:
    print(f"... ({len(candidates) - 50} more prims not shown — see counts below)")

# Same count over ALL candidates (not just shown 50)
n_rb        = sum(int(p.HasAPI(UsdPhysics.RigidBodyAPI)) for p in candidates)
n_collision = sum(int(p.HasAPI(UsdPhysics.CollisionAPI)) for p in candidates)
if HAS_PHYSX_SCHEMA:
    n_contact = sum(
        int(p.HasAPI(PhysxSchema.PhysxContactReportAPI)) for p in candidates)
else:
    n_contact = -1

print()
print("=" * 72)
print("SUMMARY")
print("=" * 72)
print(f"  Foot prims matching regex:      {len(candidates)}")
print(f"  ... with RigidBodyAPI:          {n_rb}")
print(f"  ... with CollisionAPI:          {n_collision}")
if HAS_PHYSX_SCHEMA:
    print(f"  ... with PhysxContactReportAPI: {n_contact}")
else:
    print(f"  ... with PhysxContactReportAPI: (PhysxSchema not importable)")
print()

# Diagnose
if not candidates:
    print("DIAGNOSIS: H3 — sensor regex matches zero prims.")
    print("  Fix: change the ContactSensor prim_path pattern to whatever "
          "naming the importer actually used.")
elif n_rb == 0:
    print("DIAGNOSIS: H1 — foot prims exist but none have RigidBodyAPI.")
    print("  → activate_contact_sensors=True silently skips them because it "
          "only adds PhysxContactReportAPI to RigidBodyAPI prims.")
    print("  Fix: apply RigidBodyAPI to feet at spawn time, then "
          "PhysxContactReportAPI; OR move the sensor to the parent links "
          "(link_body_*) which DO have RigidBodyAPI.")
elif HAS_PHYSX_SCHEMA and n_contact == 0:
    print("DIAGNOSIS: H2 — foot prims are rigid bodies but lack "
          "PhysxContactReportAPI.")
    print("  → activate_contact_sensors=True is not being honored on this "
          "build, OR the USD was authored without it AND the spawn code "
          "didn't apply it.")
    print("  Fix: apply PhysxSchema.PhysxContactReportAPI manually after "
          "spawn, before sim.reset() — see suggested patch in chat.")
elif HAS_PHYSX_SCHEMA and n_contact == n_rb:
    print("DIAGNOSIS: All matching foot prims already have rigid body + "
          "contact report API.")
    print("  Note: this inspection is on the static USD before any spawn "
          "code runs. activate_contact_sensors=True applies the API at "
          "spawn time, which we DON'T see here. So a clean USD with no "
          "contact API is normal — the test happens at runtime.")
    print("  Re-run the optimizer with extra prim-traversal logging right "
          "before the ContactSensor is created to see the real state.")
else:
    print(f"DIAGNOSIS: mixed state — {n_rb}/{len(candidates)} feet have "
          f"RigidBodyAPI, {n_contact}/{len(candidates)} have "
          f"PhysxContactReportAPI. Investigate which feet are missing what.")

# As a side check, also count what's on the broader foot-named set
if all_foot_named and (not candidates or n_rb < len(candidates)):
    print()
    print("Side check — broader scan ('foot' anywhere in name):")
    n_rb2  = sum(int(p.HasAPI(UsdPhysics.RigidBodyAPI)) for p in all_foot_named)
    print(f"  total prims:         {len(all_foot_named)}")
    print(f"  with RigidBodyAPI:   {n_rb2}")
    if HAS_PHYSX_SCHEMA:
        n_contact2 = sum(
            int(p.HasAPI(PhysxSchema.PhysxContactReportAPI))
            for p in all_foot_named)
        print(f"  with ContactReport:  {n_contact2}")

sim_app.close()
