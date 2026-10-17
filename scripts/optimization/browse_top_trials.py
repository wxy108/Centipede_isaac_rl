#!/usr/bin/env python3
"""
browse_top_trials.py — list the top N trials from a study and optionally
                       export any one of them as a drop-in YAML
=========================================================================

After running optimize_impedance_bayesian.py, the run directory contains
`all_trials.csv` with EVERY trial's params and cost components. The
"winner" reported by the optimizer is just `argmin(cost)` — but if you
want to inspect runner-ups (e.g. the cost-#2 trial walks slower but is
much straighter, or has more compliance, or uses safer kp values),
this script ranks them and lets you pick one.

Usage
-----
  # List top 10 of the latest run (auto-picks most recent run dir)
  python scripts/optimization/browse_top_trials.py

  # Top 25 of a specific run
  python scripts/optimization/browse_top_trials.py \
      outputs/optimization/impedance_bay_rough_wl18_<ts>/all_trials.csv \
      --top 25

  # Same, but ALSO export trial #N as a drop-in YAML alongside best_params.yaml
  python scripts/optimization/browse_top_trials.py --top 25 --export 47

The exported YAML lives in the same run directory as
`trial_<N>_params.yaml` and you can use it directly:

  copy outputs/optimization/<run_dir>/trial_47_params.yaml ^
       configs/farms_controller_numpy_legacy.yaml
  python scripts/test_walk.py --headless --duration 10 ^
       --video videos/trial_47.mp4 --camera-mode chase ^
       --terrain rough --terrain-wavelength 0.018 --terrain-amplitude 0.01

Filtering options
-----------------
  --min-speed 20      only show trials with speed_mps >= 20mm/s
  --min-strt 0.7      only show trials with straightness >= 0.7
  --no-buckled        exclude buckled trials (default)
  --include-buckled   include buckled trials (cost > 1e4)
"""

import argparse
import csv
import glob
import os
import sys
import yaml


SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
# 2026-07-16: repointed farms_controller.yaml -> farms_controller_numpy_legacy.yaml (this script was
# MISSED by S2b's repoint of the other 7 numpy-family consumers). This tool patches CMA-ES gains into
# a base yaml and exports a drop-in for the NUMPY controller (controllers/impedance_controller.py, via
# scripts/test_walk.py), which still computes torque and NEEDS those gains. S2b made
# farms_controller.yaml GAIN-FREE -- it configures the TARGETS-ONLY torch controller, whose gains come
# from env.set_joint_impedance() -> PhysX -- so using it as the base made the leg-gain
# `.get(..., [defaults])` in export_trial_as_yaml fall back SILENTLY to hard-coded values 35-168x off
# the tuned ones. The gains live on, byte-identical, in the _numpy_legacy copy, which also KEEPS its
# `impedance:` block name -- correctly: there it still carries real impedance. (configs/
# farms_controller.yaml's block was renamed to `reference_targets:` on 2026-07-16 because it no longer
# does.) See docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md.
CFG_PATH     = os.path.join(PROJECT_ROOT, "configs", "farms_controller_numpy_legacy.yaml")
BUCKLED_THR  = 1e4    # trials with cost above this are "buckled"


def find_latest_csv():
    pat = os.path.join(PROJECT_ROOT, "outputs", "optimization",
                       "impedance_bay_*", "all_trials.csv")
    matches = sorted(glob.glob(pat), key=os.path.getmtime, reverse=True)
    return matches[0] if matches else None


def parse_csv(path):
    with open(path, "r", newline="") as f:
        return list(csv.DictReader(f))


def f(row, key, default=0.0):
    """Float-parse a CSV cell with a default."""
    v = row.get(key, "")
    try:
        return float(v) if v != "" else default
    except (TypeError, ValueError):
        return default


def export_trial_as_yaml(row, base_yaml_path, out_path):
    """Build a drop-in YAML by patching the trial's params into the
    configs/farms_controller_numpy_legacy.yaml structure (the NUMPY controller's config)."""
    with open(base_yaml_path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    # setdefault would MASK a wrong base: pointed at the (gain-free) configs/farms_controller.yaml it
    # would silently CREATE an empty block, and the leg-gain `.get(..., [defaults])` below would then
    # inject hard-coded values 35-168x off the tuned ones. Require the real gain block instead.
    # NOTE the exported yaml targets the NUMPY controller -- `impedance:` is still the right name there.
    if "impedance" not in cfg:
        raise ValueError(
            "base yaml %r has no `impedance:` gain block.\n"
            "If it has a `reference_targets:` block instead, it is configs/farms_controller.yaml -- "
            "the TARGETS-ONLY torch controller's config, gain-free since 2026-07-16 (impedance is now "
            "env.set_joint_impedance() -> PhysX, and the solver is PhysX's implicit PD). This exporter "
            "produces a drop-in for the NUMPY controller and needs the real gains: use "
            "configs/farms_controller_numpy_legacy.yaml (= CFG_PATH).\n"
            "See docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md." % base_yaml_path
        )
    imp = cfg["impedance"]
    imp["body_kp"] = f(row, "body_kp")
    imp["body_kv"] = f(row, "body_kv")
    if "pitch_kp" in row:
        imp["pitch_kp"] = f(row, "pitch_kp")
        imp["pitch_kv"] = f(row, "pitch_kv")
    leg = imp.setdefault("leg", {})
    kp = list(leg.get("kp", [0.5, 0.06, 0.13, 0.13]))
    kv = list(leg.get("kv", [0.001, 0.003, 0.0005, 0.001]))
    kp[0] = f(row, "hip_yaw_kp");   kv[0] = f(row, "hip_yaw_kv")
    kp[1] = f(row, "hip_pitch_kp"); kv[1] = f(row, "hip_pitch_kv")
    if "tibia_kp" in row:
        kp[2] = f(row, "tibia_kp");  kv[2] = f(row, "tibia_kv")
    if "tarsus_kp" in row:
        kp[3] = f(row, "tarsus_kp"); kv[3] = f(row, "tarsus_kv")
    leg["kp"] = kp
    leg["kv"] = kv
    with open(out_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(cfg, fh, sort_keys=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default=None,
                    help="all_trials.csv from a run. If omitted, uses "
                         "the most recent run under outputs/optimization/.")
    ap.add_argument("--top", type=int, default=15,
                    help="Number of top trials to display (default 15).")
    ap.add_argument("--export", type=int, default=None,
                    help="Export trial #N's gains as a drop-in YAML "
                         "(<run_dir>/trial_<N>_params.yaml).")
    ap.add_argument("--min-speed",   type=float, default=0.0,
                    help="Only show trials with speed_mps >= this (mm/s).")
    ap.add_argument("--min-strt",    type=float, default=0.0,
                    help="Only show trials with straightness >= this.")
    ap.add_argument("--include-buckled", action="store_true",
                    help="Include buckled trials in the ranking.")
    ap.add_argument("--show-kv", action="store_true",
                    help="Print all damping (kv) values instead of all "
                         "stiffness (kp) values. Useful for comparing the "
                         "ELASTICITY of two gain sets — low kv = elastic / "
                         "ringing, high kv = damped / dead-feeling.")
    ap.add_argument("--trials", type=str, default=None,
                    help="Comma-separated list of trial numbers to print "
                         "(comparison view, ignores --top). e.g. "
                         "--trials 208,314 to see ranks 3 vs 29.")
    args = ap.parse_args()

    csv_path = args.csv or find_latest_csv()
    if not csv_path or not os.path.isfile(csv_path):
        print("ERROR: no all_trials.csv found.", file=sys.stderr)
        return 2

    rows = parse_csv(csv_path)
    if not rows:
        print("CSV is empty.", file=sys.stderr)
        return 2

    # Filter
    filtered = []
    for r in rows:
        cost = f(r, "cost", default=BUCKLED_THR)
        if not args.include_buckled and cost >= BUCKLED_THR * 0.99:
            continue
        if f(r, "speed_mps") * 1000.0 < args.min_speed:
            continue
        if f(r, "straightness") < args.min_strt:
            continue
        filtered.append(r)

    if not filtered:
        print("No trials match the filters.", file=sys.stderr)
        return 1

    # Sort by cost (lowest = best)
    filtered.sort(key=lambda r: f(r, "cost", default=BUCKLED_THR))

    # ── --trials override: just print the requested trial rows in cost order
    if args.trials:
        wanted = {int(t.strip()) for t in args.trials.split(",")}
        picked = [r for r in rows if int(f(r, "trial", -1)) in wanted]
        picked.sort(key=lambda r: f(r, "cost", default=BUCKLED_THR))
        filtered = picked
        if not picked:
            print(f"No trials in CSV match --trials {args.trials}",
                  file=sys.stderr)
            return 1

    # ── Display top N ──
    print(f"\nReading {csv_path}")
    print(f"Total trials in CSV: {len(rows)}  ·  matching filters: {len(filtered)}\n")
    print(f"Filters: min-speed={args.min_speed} mm/s · min-strt={args.min_strt} · "
          f"buckled={'incl' if args.include_buckled else 'excl'}\n")

    if args.show_kv:
        print(f"  rank #trial   cost   speed   strt    cmp°  body_kv  "
              f"hyaw_kv  hpitch_kv tibia_kv tarsus_kv pitch_kv")
        print(f"  ---- ------ -------- ------ ------ ------ -------- "
              f"-------- --------- -------- --------- --------")
        for rank, r in enumerate(filtered[:args.top]):
            print(f"  {rank+1:>4} {int(f(r,'trial')):>6}  "
                  f"{f(r,'cost'):>7.3f}  "
                  f"{f(r,'speed_mps')*1000:>5.1f}  "
                  f"{f(r,'straightness'):>5.3f}  "
                  f"{f(r,'pitch_compliance_deg'):>5.2f}  "
                  f"{f(r,'body_kv'):>7.1e}  "
                  f"{f(r,'hip_yaw_kv'):>7.1e}  "
                  f"{f(r,'hip_pitch_kv'):>8.1e}  "
                  f"{f(r,'tibia_kv'):>7.1e}  "
                  f"{f(r,'tarsus_kv'):>8.1e}  "
                  f"{f(r,'pitch_kv'):>7.1e}")
    else:
        print(f"  rank #trial   cost   speed   strt    cmp°  body_kp body_kv  "
              f"hyaw_kp hpitch_kp tibia_kp tarsus_kp pitch_kp")
        print(f"  ---- ------ -------- ------ ------ ------ -------- -------- "
              f"-------- --------- -------- --------- --------")
        for rank, r in enumerate(filtered[:args.top]):
            print(f"  {rank+1:>4} {int(f(r,'trial')):>6}  "
                  f"{f(r,'cost'):>7.3f}  "
                  f"{f(r,'speed_mps')*1000:>5.1f}  "
                  f"{f(r,'straightness'):>5.3f}  "
                  f"{f(r,'pitch_compliance_deg'):>5.2f}  "
                  f"{f(r,'body_kp'):>7.4f}  "
                  f"{f(r,'body_kv'):>7.1e}  "
                  f"{f(r,'hip_yaw_kp'):>7.4f}  "
                  f"{f(r,'hip_pitch_kp'):>8.4f}  "
                  f"{f(r,'tibia_kp'):>7.4f}  "
                  f"{f(r,'tarsus_kp'):>8.4f}  "
                  f"{f(r,'pitch_kp'):>7.4f}")

    # ── Export ──
    if args.export is not None:
        target = next((r for r in rows if int(f(r, "trial", -1)) == args.export),
                      None)
        if target is None:
            print(f"\nERROR: trial #{args.export} not found in CSV.",
                  file=sys.stderr)
            return 1
        run_dir = os.path.dirname(csv_path)
        out_path = os.path.join(run_dir, f"trial_{args.export}_params.yaml")
        export_trial_as_yaml(target, CFG_PATH, out_path)
        print(f"\n✓ Wrote {out_path}")
        print(f"\nTo USE this gain set:")
        print(f"  copy {out_path} configs\\farms_controller_numpy_legacy.yaml")
        print(f"  python scripts/test_walk.py --headless --duration 10 \\")
        print(f"      --video videos/trial_{args.export}.mp4 \\")
        print(f"      --camera-mode chase --terrain rough \\")
        print(f"      --terrain-wavelength 0.018 --terrain-amplitude 0.01")
    else:
        print(f"\nTo export any one of these as a drop-in YAML:")
        print(f"  python scripts/optimization/browse_top_trials.py "
              f"--export <trial_number>")

    return 0


if __name__ == "__main__":
    sys.exit(main())
