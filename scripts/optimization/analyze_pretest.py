#!/usr/bin/env python3
"""
analyze_pretest.py — break down the pretest probes' costs and suggest
                     weight adjustments
====================================================================

After running optimize_impedance_bayesian.py with --pretest, point this
script at the run's all_trials.csv and it will:

  1. Print each trial's cost decomposed into its 6 components
     (wobble, body_track, leg_track, force, speed, compliance).
  2. Compute the median magnitude of each component across the
     non-buckled trials.
  3. Suggest weight adjustments so that the dominant terms are within
     ~10× of each other (so the optimizer cares about all of them).

This is the "manual interpretation" path described in
NOTES_RENDERING / the conversation log: the optimizer itself does NOT
auto-adjust cost weights between trials. This helper just makes the
manual rebalance fast.

Usage
-----
  python scripts/optimization/analyze_pretest.py \
      outputs/optimization/impedance_bay_rough_wl18_<timestamp>/all_trials.csv

Or, omit the path and the script will auto-pick the most recent run:
  python scripts/optimization/analyze_pretest.py
"""

import argparse
import csv
import glob
import os
import sys
from datetime import datetime


# ── These match the defaults in optimize_impedance_bayesian.py
#    (post 2026-05-01 speed/displacement-prioritised retune) ──
DEFAULT_WEIGHTS = dict(
    w_body_track   = 0.10,
    w_leg_track    = 0.20,
    w_wobble       = 4.0,
    w_force        = 0.5,
    w_speed        = 200.0,
    w_displacement = 100.0,   # NEW 2026-05-01
    w_compliance   = 0.05,
)
DEFAULT_LIMITS = dict(
    body_tol_deg       = 5.0,
    leg_tol_deg        = 3.0,
    force_limit        = 4.0,
    compliance_cap_deg = 10.0,
)


def find_latest_run():
    """If no path given, pick the most recent run dir."""
    pattern = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "..", "outputs", "optimization",
                           "impedance_bay_*", "all_trials.csv")
    matches = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
    return matches[0] if matches else None


def median(xs):
    xs = sorted(xs)
    if not xs:
        return 0.0
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def parse_csv(path):
    with open(path, "r", newline="") as f:
        return list(csv.DictReader(f))


def cost_components(row, weights, limits):
    """Recompute the 7 cost components from the CSV row, using the
    weights and limits passed in. Returns dict + total."""
    def get(name, default=0.0):
        v = row.get(name, "")
        try:
            return float(v) if v != "" else default
        except (TypeError, ValueError):
            return default
    rmse_b   = get("rmse_body_yaw_deg")
    rmse_l   = get("rmse_leg_deg")
    wobble   = get("wobble")
    fw       = get("peak_force_over_weight")
    speed    = get("speed_mps")
    disp     = get("displacement_m")
    cmp_d    = get("pitch_compliance_deg")
    body_excess  = max(0.0, rmse_b - limits["body_tol_deg"])
    leg_excess   = max(0.0, rmse_l - limits["leg_tol_deg"])
    force_excess = max(0.0, fw     - limits["force_limit"])
    cmp_reward   = min(cmp_d, limits["compliance_cap_deg"])
    parts = dict(
        body_track   = + weights["w_body_track"]   * body_excess,
        leg_track    = + weights["w_leg_track"]    * leg_excess,
        wobble       = + weights["w_wobble"]       * wobble,
        force        = + weights["w_force"]        * force_excess,
        speed        = - weights["w_speed"]        * speed,        # reward (negative cost)
        displacement = - weights["w_displacement"] * disp,         # reward
        compliance   = - weights["w_compliance"]   * cmp_reward,   # reward
    )
    parts["total"] = sum(parts.values())
    return parts


def fmt_part(v):
    sign = "+" if v >= 0 else "−"
    return f"{sign}{abs(v):6.3f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default=None,
                    help="all_trials.csv from a pretest run. If omitted, "
                         "uses the most recent run under outputs/optimization/.")
    ap.add_argument("--n", type=int, default=10,
                    help="Show the first N trials (default 10).")
    args = ap.parse_args()

    csv_path = args.csv or find_latest_run()
    if not csv_path or not os.path.isfile(csv_path):
        print("ERROR: no all_trials.csv found.", file=sys.stderr)
        return 2
    print(f"reading {csv_path}", flush=True)
    rows = parse_csv(csv_path)
    if not rows:
        print("CSV is empty.", file=sys.stderr)
        return 2

    print(f"\n── per-trial cost decomposition (first {args.n}) ──")
    print(f"{'#':>3}  {'cost':>9}  "
          f"{'body_t':>8}  {'leg_t':>8}  {'wobble':>8}  "
          f"{'force':>8}  {'speed':>8}  {'displ':>8}  {'compl':>8}  "
          f"buck note")
    nonbuck = []
    for r in rows[:args.n]:
        n = r.get("trial", "?")
        buckled = (r.get("buckled", "False") == "True"
                   or float(r.get("cost", "0") or "0") > 1e4)
        parts = cost_components(r, DEFAULT_WEIGHTS, DEFAULT_LIMITS)
        if not buckled:
            nonbuck.append(parts)
        try:
            cost = float(r.get("cost", "0") or "0")
            cs = f"{cost:9.3f}"
        except ValueError:
            cs = "     nan "
        print(f"{n:>3}  {cs}  "
              f"{fmt_part(parts['body_track']):>8}  "
              f"{fmt_part(parts['leg_track']):>8}  "
              f"{fmt_part(parts['wobble']):>8}  "
              f"{fmt_part(parts['force']):>8}  "
              f"{fmt_part(parts['speed']):>8}  "
              f"{fmt_part(parts['displacement']):>8}  "
              f"{fmt_part(parts['compliance']):>8}  "
              f"{'YES' if buckled else '   '}  {r.get('reason','')}")

    if not nonbuck:
        print("\nAll shown trials buckled. The current bounds are too "
              "aggressive — tighten the lower bounds on kp (more stiffness) "
              "or the lower bounds on kv (more damping) and re-run.")
        return 1

    # ── Median absolute magnitude of each component (rewards in abs) ──
    mags = {
        k: median([abs(p[k]) for p in nonbuck])
        for k in ("body_track", "leg_track", "wobble",
                  "force", "speed", "displacement", "compliance")
    }
    print("\n── median |cost component| across non-buckled trials ──")
    biggest = max(mags.values())
    for k, v in sorted(mags.items(), key=lambda kv: -kv[1]):
        bar = "█" * int(round(40 * v / max(biggest, 1e-9)))
        print(f"  {k:<11}: {v:7.3f}   {bar}")

    # ── Suggest rebalances ──
    # Goal: every term within 10× of the largest. Anything <=1/10 of the
    # max we recommend bumping its weight; anything way above we recommend
    # lowering. 'force' is currently inert in the Isaac port — skip it.
    print("\n── suggested weight adjustments ──")
    biggest_k = max(mags, key=mags.get)
    if mags[biggest_k] < 1e-3:
        print("  All cost components are tiny — the optimizer is bunched "
              "right at zero cost. Either widen the search bounds further "
              "or extend --duration so the gait has more time to develop.")
        return 0
    print(f"  Dominant term: {biggest_k}  ({mags[biggest_k]:.3f})")
    suggestions = []
    weight_map = {
        "body_track"   : "w_body_track",
        "leg_track"    : "w_leg_track",
        "wobble"       : "w_wobble",
        "force"        : "w_force",
        "speed"        : "w_speed",
        "displacement" : "w_displacement",
        "compliance"   : "w_compliance",
    }
    for k, m in mags.items():
        if k == "force":
            continue
        if k == biggest_k:
            continue
        ratio = m / max(mags[biggest_k], 1e-9)
        if ratio < 0.1:
            scale = (0.1 / max(ratio, 1e-9)) ** 0.5  # damp the suggestion
            new_w = DEFAULT_WEIGHTS[weight_map[k]] * scale
            suggestions.append(
                (weight_map[k], DEFAULT_WEIGHTS[weight_map[k]], new_w,
                 f"too small ({ratio:.2%} of dominant)"))
    if not suggestions:
        print("  All non-force terms are within 10× of the dominant — "
              "weights look well-balanced. You can run the full study.")
    else:
        print("  Some terms are <1/10 of the dominant. Bump their weights "
              "so the optimizer pays attention to them:")
        for w_name, old, new, why in suggestions:
            print(f"    --{w_name.replace('_','-')}  {old:.3g}  →  {new:.3g}   ({why})")
        print("\n  Suggested CLI override for the full study:")
        flags = " ".join(f"--{w_name.replace('_','-')} {new:.3g}"
                         for w_name, old, new, _ in suggestions)
        print(f"    python scripts/optimization/optimize_impedance_bayesian.py \\")
        print(f"        --n-trials 200 --duration 8 --headless \\")
        print(f"        --replay-video --resume {csv_path.replace('all_trials.csv','study.db')} \\")
        print(f"        {flags}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
