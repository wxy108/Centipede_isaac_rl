"""Rebuild the fixed-wave landscape with one auditable process per cell."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--terrains", nargs="+", default=["flat", "l120a4", "l90a8", "l60a12", "l45a16", "l30a16"])
parser.add_argument("--terrain-seeds", nargs="+", type=int, default=[244949, 264575])
parser.add_argument("--waves", nargs="+", type=float, default=[1.0 + 0.25 * i for i in range(13)])
parser.add_argument("--out", type=Path, default=ROOT / "outputs" / "open_loop_sweep")
parser.add_argument("--video", action="store_true")
args = parser.parse_args()


def main():
    for terrain in args.terrains:
        for terrain_seed in args.terrain_seeds:
            for wave in args.waves:
                command = [
                    sys.executable, str(ROOT / "scripts" / "evaluate.py"),
                    "--terrain", terrain,
                    "--terrain-seed", str(terrain_seed),
                    "--fixed-wave", str(wave),
                    "--out", str(args.out),
                    "--headless",
                ]
                if args.video:
                    command.append("--video")
                print(f"[SWEEP] {terrain} seed={terrain_seed} wave={wave:g}", flush=True)
                subprocess.run(command, cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
