#!/usr/bin/env python3
"""
diag_usdrt.py — diagnose what usdrt looks like inside Isaac Sim
================================================================
Reports:
  - Whether usdrt is importable at all (it should be, inside the app)
  - Where it's loaded from on disk
  - What top-level attributes it exposes
  - Whether usdrt.hierarchy is reachable (the specific submodule the
    Camera Fabric write needs)
  - Which loaded kit extensions mention 'usdrt' or 'fabric'
  - Where Isaac Sim's exts and extscache directories are on disk

Run with:
    python scripts/diag_usdrt.py
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import os
import sys
import traceback

print("\n" + "=" * 72)
print(" USDRT DIAGNOSTIC")
print("=" * 72)

# ─────────────────────────────────────────────────────────────────────────
# 1. import usdrt — top-level
# ─────────────────────────────────────────────────────────────────────────
print("\n[1] import usdrt")
try:
    import usdrt
    print(f"    OK")
    print(f"    __file__: {getattr(usdrt, '__file__', 'NONE')}")
    print(f"    __path__: {list(getattr(usdrt, '__path__', []))}")
    print(f"    public attributes:")
    pub_attrs = sorted(x for x in dir(usdrt) if not x.startswith("_"))
    for a in pub_attrs:
        print(f"        {a}")
    print(f"    has_hierarchy: {hasattr(usdrt, 'hierarchy')}")
except Exception as e:
    print(f"    FAILED: {type(e).__name__}: {e}")
    traceback.print_exc()

# ─────────────────────────────────────────────────────────────────────────
# 2. import usdrt.hierarchy explicitly (might trigger a lazy load that
#    doesn't show up in dir(usdrt))
# ─────────────────────────────────────────────────────────────────────────
print("\n[2] import usdrt.hierarchy")
try:
    import usdrt.hierarchy as _h
    print(f"    OK")
    print(f"    __file__: {getattr(_h, '__file__', 'NONE')}")
    h_attrs = sorted(x for x in dir(_h) if not x.startswith("_"))
    print(f"    {len(h_attrs)} public attribute(s):")
    for a in h_attrs[:30]:
        print(f"        {a}")
    if len(h_attrs) > 30:
        print(f"        ... and {len(h_attrs)-30} more")
except Exception as e:
    print(f"    FAILED: {type(e).__name__}: {e}")

# ─────────────────────────────────────────────────────────────────────────
# 3. List on-disk locations where usdrt could live
# ─────────────────────────────────────────────────────────────────────────
print("\n[3] scanning candidate usdrt install locations")
candidates = []
# conda env site-packages
import site
for sp in site.getsitepackages():
    candidates.append(sp)
candidates.append(site.getusersitepackages())
# Isaac Sim default extension cache dirs
home = os.path.expanduser("~")
candidates.append(os.path.join(home, "AppData", "Local", "ov", "pkg"))
candidates.append(os.path.join(home, "AppData", "Local", "ov", "data",
                                "Kit", "Isaac-Sim"))

found_any = False
for root in candidates:
    if not os.path.isdir(root):
        continue
    # walk only one or two levels deep — usdrt won't be deeply nested
    for dirpath, dirnames, filenames in os.walk(root):
        # limit depth
        depth = dirpath[len(root):].count(os.sep)
        if depth > 4:
            del dirnames[:]
            continue
        # match a usdrt directory or file
        for name in dirnames + filenames:
            ln = name.lower()
            if ln == "usdrt" or ln.startswith("usdrt.") or \
               ln.startswith("usdrt-") or "usdrt_hierarchy" in ln \
               or "usdrt.hierarchy" in ln:
                full = os.path.join(dirpath, name)
                print(f"    {full}")
                found_any = True

if not found_any:
    print("    (no usdrt-related path found in standard locations)")

# ─────────────────────────────────────────────────────────────────────────
# 4. Loaded kit extensions that mention usdrt or fabric
# ─────────────────────────────────────────────────────────────────────────
print("\n[4] kit extensions mentioning usdrt or fabric")
try:
    import omni.kit.app
    mgr = omni.kit.app.get_app().get_extension_manager()
    exts = mgr.get_extensions()
    print(f"    total extensions loaded: {len(exts)}")
    keep = []
    for e in exts:
        eid = (e.get("id") or e.get("name") or "").lower()
        if "usdrt" in eid or "fabric" in eid or "hierarchy" in eid:
            keep.append(e)
    print(f"    matching usdrt/fabric/hierarchy: {len(keep)}")
    for e in keep:
        eid = e.get("id") or e.get("name", "?")
        path = e.get("path", "")
        enabled = e.get("enabled", "?")
        print(f"      {eid}")
        print(f"          enabled={enabled}  path={path}")
except Exception as e:
    print(f"    FAILED: {type(e).__name__}: {e}")
    traceback.print_exc()

# ─────────────────────────────────────────────────────────────────────────
# 5. sys.path entries that look kit-related
# ─────────────────────────────────────────────────────────────────────────
print("\n[5] sys.path entries that look kit/extscache-related")
for p in sys.path:
    pl = p.lower()
    if ("kit" in pl or "extscache" in pl or "isaacsim" in pl
            or "ov" in pl):
        print(f"    {p}")

print("\n" + "=" * 72)
print(" DIAGNOSTIC COMPLETE — paste the output above")
print("=" * 72 + "\n")

app.close()
