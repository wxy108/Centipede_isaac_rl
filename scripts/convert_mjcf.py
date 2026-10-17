#!/usr/bin/env python3
"""
convert_mjcf.py — Convert the MuJoCo centipede XML to Isaac Sim USD
====================================================================
Uses Isaac Sim's built-in MJCF importer to produce a USD articulation
that can be loaded by Isaac Lab.

Usage:
    python scripts/convert_mjcf.py \
        --input  ../Centipede_MUJOCO-main/models/farms/centipede.xml \
        --output assets/centipede.usd

Requirements:
    Isaac Sim must be installed (provides isaacsim.asset.importer.mjcf).

Notes:
    - The MJCF importer handles joints, actuators, meshes, and collision.
    - Heightfield terrain is NOT imported (we recreate it in Isaac Lab).
    - After conversion, inspect the USD in Isaac Sim GUI to verify joint
      count, collision geometry, and axis orientations.
"""

import argparse
import os
import sys


def convert(input_xml: str, output_usd: str):
    """Run the MJCF → USD conversion inside an Isaac Sim context."""

    # Isaac Sim must be initialized before importing its extensions
    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": True})

    # Now we can import the MJCF converter
    from isaaclab.sim.converters import MjcfConverter, MjcfConverterCfg

    cfg = MjcfConverterCfg(
        asset_path=os.path.abspath(input_xml),
        usd_dir=os.path.dirname(os.path.abspath(output_usd)),
        usd_file_name=os.path.basename(output_usd),
        # Fix the base link so the freejoint is preserved as a
        # floating-base articulation (not a fixed-base robot).
        fix_base=False,
        # Import collision meshes from the MJCF geoms
        import_sites=True,
    )

    converter = MjcfConverter(cfg)
    usd_path = converter.usd_path

    print(f"\n{'='*60}")
    print(f"MJCF → USD conversion complete!")
    print(f"  Input:  {input_xml}")
    print(f"  Output: {usd_path}")
    print(f"{'='*60}")
    print(f"\nNext steps:")
    print(f"  1. Open {usd_path} in Isaac Sim GUI")
    print(f"  2. Verify joint count (expect 288+)")
    print(f"  3. Check collision geometry")
    print(f"  4. Run: python scripts/run_standalone.py --mode stand")

    print("PROBE_DONE finished", flush=True)  # watchdog kill marker (pre-teardown)
    sim_app.close()
    return usd_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert MJCF centipede to USD")
    parser.add_argument("--input", required=True,
                        help="Path to centipede.xml (MuJoCo MJCF)")
    parser.add_argument("--output", default="assets/centipede.usd",
                        help="Output USD path (default: assets/centipede.usd)")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"ERROR: Input file not found: {args.input}")
        sys.exit(1)

    convert(args.input, args.output)
