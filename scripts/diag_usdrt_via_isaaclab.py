#!/usr/bin/env python3
"""
diag_usdrt_via_isaaclab.py — second usdrt check, this time using
the SAME app-boot path test_walk.py uses (isaaclab.app.AppLauncher
+ isaaclab.python.headless.rendering.kit).

If usdrt.hierarchy is missing here but present in the simpler
diag_usdrt.py, the Isaac Lab kit file is missing the extension
that provides hierarchy.

Run with:
    python scripts/diag_usdrt_via_isaaclab.py
"""
import argparse
parser = argparse.ArgumentParser()
# Force --enable_cameras like test_walk.py does when --video is passed,
# so we exercise the exact same kit configuration.
import sys
if "--enable_cameras" not in sys.argv:
    sys.argv.append("--enable_cameras")
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
launcher = AppLauncher(args)
app = launcher.app

print("\n" + "=" * 72)
print(" USDRT DIAGNOSTIC (via isaaclab.app.AppLauncher)")
print("=" * 72)

print("\n[1] import usdrt")
try:
    import usdrt
    pub = sorted(x for x in dir(usdrt) if not x.startswith("_"))
    print(f"    OK; has_hierarchy_attr: {hasattr(usdrt, 'hierarchy')}")
    print(f"    public: {pub}")
except Exception as e:
    print(f"    FAILED: {type(e).__name__}: {e}")

print("\n[2] import usdrt.hierarchy explicitly")
try:
    import usdrt.hierarchy
    print(f"    OK; module: {usdrt.hierarchy}")
    print(f"    file: {getattr(usdrt.hierarchy, '__file__', 'NONE')}")
    print(f"    has IFabricHierarchy: "
          f"{hasattr(usdrt.hierarchy, 'IFabricHierarchy')}")
except Exception as e:
    print(f"    FAILED: {type(e).__name__}: {e}")

print("\n[3] after the explicit import, recheck usdrt.hierarchy attr")
import usdrt
print(f"    has_hierarchy_attr: {hasattr(usdrt, 'hierarchy')}")

print("\n[4] try the call Camera.set_world_poses uses internally")
try:
    h = usdrt.hierarchy.IFabricHierarchy()
    print(f"    IFabricHierarchy() instantiated: {h}")
except Exception as e:
    print(f"    FAILED: {type(e).__name__}: {e}")

print("\n[5] check kit extensions for usdrt_delegate / hierarchy")
try:
    import omni.kit.app
    mgr = omni.kit.app.get_app().get_extension_manager()
    exts = mgr.get_extensions()
    for e in exts:
        eid = (e.get("id") or e.get("name") or "").lower()
        if "usdrt" in eid or "fabric" in eid or "hierarchy" in eid:
            print(f"    {e.get('id', '?'):60s}  enabled={e.get('enabled')}")
except Exception as e:
    print(f"    FAILED: {e}")

print("\n" + "=" * 72)
print(" DIAGNOSTIC COMPLETE")
print("=" * 72 + "\n")
app.close()
