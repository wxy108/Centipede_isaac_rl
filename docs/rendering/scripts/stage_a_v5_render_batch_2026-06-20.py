"""stage_a_v5_render_batch_2026-06-20.py — Part 3 render batch (v5 top-N rough clips).

Renders ONE rough (λ60/A16, 1 Hz) clip per controller via the FROZEN render script
(scripts/sanity/stage_a_v2_render_winner_2026-06-18.py) with the body-PCA HUD burned in, through
the watchdog. The companion grid-composite script then tiles the clips into the 3x3 grid.

1ms note: --steps-warmup-phys is in PHYSICS substeps; at decimation 48 a value of 960 gives
960//48 = 20 policy warmup steps (== the proven 4ms settling), and the render's episode_length_s
auto-extends to fit warmup+capture+margin (the 2026-06-21 >8s-truncation lesson). Plain CPU driver;
ONE watchdog-supervised GPU render at a time. NO PROTECTED EDITS.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, os.pardir, os.pardir))
RENDER = os.path.join("scripts", "sanity", "stage_a_v2_render_winner_2026-06-18.py")
WATCHDOG = os.path.join("scripts", "sanity", "run_isaaclab_with_watchdog.ps1")


def write_gains_one(path, gains_dict):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"gains": [gains_dict]}, f, indent=2)


def hardened_ok(mp4):
    """Color-agnostic clip OK-check (GO_FIX_RENDER_AUDIT_GAPS): retry triggers on BLACK *or* WHITE/BLANK,
    not just file size — the half-white c0 clip was >50KB and slipped the size/black checks. PASS iff no
    black (mean<8), no blank (spatial std<6, uniform black/white), no over-exposed (mean>240) frame, motion>0."""
    if not (os.path.isfile(mp4) and os.path.getsize(mp4) > 50_000):
        return False
    try:
        import cv2
        import numpy as np
    except Exception:
        return True   # cv2 missing: fall back to the size check already passed above
    cap = cv2.VideoCapture(mp4)
    means, stds, prev, mot = [], [], None, []
    while True:
        ok, fr = cap.read()
        if not ok or fr is None:
            break
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY).astype("float32")
        means.append(float(g.mean())); stds.append(float(g.std()))
        if prev is not None:
            mot.append(float(np.abs(g - prev).mean()))
        prev = g
    cap.release()
    if not means:
        return False
    means, stds = np.array(means), np.array(stds)
    motion = float(np.mean(mot)) if mot else 0.0
    return bool((means < 8).sum() == 0 and (stds < 6).sum() == 0
                and (means > 240).sum() == 0 and motion > 0)


def render_one(cid, gains_json, out_dir, usd, amp_mm, dur, fps, warmup_phys, exp_min,
               lam=60, extra=""):
    mp4 = os.path.join(out_dir, "videos", f"{cid}_rough_a{int(amp_mm)}.mp4")
    js = os.path.join(out_dir, "videos", f"{cid}_rough_a{int(amp_mm)}.json")
    log = os.path.join(out_dir, "logs", f"render_{cid}.log")
    for p in (mp4, js, log):
        os.makedirs(os.path.dirname(p), exist_ok=True)
    if os.path.exists(mp4):
        os.remove(mp4)
    usd_arg = f"--usd {usd} " if usd else ""
    extra_arg = f"{extra} " if extra else ""
    cmd = (f".\\IsaacLab\\isaaclab.bat -p {RENDER} "
           f"--freesearch-gains-json {gains_json} --homogeneous-lambda-mm {lam} "
           f"--homogeneous-amp-mm {amp_mm} --homogeneous-seed 4242 --freq-hz 1.0 "
           f"--pin-block 0 --n-y-strata 5 --pin-y-strat 2 --no-jitter {extra_arg}"
           f"--duration-sec {dur} --video-fps {fps} --steps-warmup-phys {warmup_phys} "
           f"--camera chase --width 640 --height 384 "
           f"--out {mp4} --out-json {js} --policy-name {cid} {usd_arg}--reattach-after-warmup")
    wd = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", WATCHDOG,
          "-Cmd", cmd, "-LogFile", log, "-ExpectedReport", mp4, "-ExpectedDurationMin", str(exp_min)]
    print(f"[v5render] {cid}: rough a{amp_mm} {dur}s", flush=True)
    t0 = time.time()
    try:
        subprocess.run(wd, cwd=PROJECT_ROOT, timeout=exp_min * 90 + 300, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        print(f"[v5render] {cid} TIMEOUT", flush=True)
    wall = time.time() - t0
    ok = hardened_ok(mp4)   # color-agnostic: rejects black OR white/blank OR frozen clips (not just size)
    print(f"[v5render] {cid} {'OK' if ok else 'FAILED(black/white/blank)'} ({wall:.0f}s) size="
          f"{os.path.getsize(mp4) if os.path.isfile(mp4) else 0}B", flush=True)
    return dict(id=cid, mp4=mp4, json=js, ok=ok, wall_sec=wall)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True,
                    help='JSON: [{"id":"v5_r0","gains":{...cell-gains...}}, ...] (top-N controllers)')
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--usd", default="assets/centipede_rebuild_v1_renamed.usd")
    ap.add_argument("--amp-mm", type=float, default=16.0)
    ap.add_argument("--dur", type=float, default=16.0)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--warmup-phys", type=int, default=960)
    ap.add_argument("--exp-min", type=int, default=18)
    ap.add_argument("--max-retries", type=int, default=4,
                    help="Black-frame race (RENDERING_PIPELINE_DEBUG_NOTES.md): re-boot until a "
                         "valid (non-black, >50KB) clip is produced. The race is per-boot.")
    ap.add_argument("--lam", type=int, default=60, help="homogeneous wavelength mm")
    ap.add_argument("--extra", default="", help="extra render-script args appended verbatim (e.g. arena flags)")
    args = ap.parse_args()
    with open(args.manifest, "r", encoding="utf-8") as f:
        controllers = json.load(f)
    results = []
    gdir = os.path.join(args.out_dir, "gains")
    for c in controllers:
        gp = os.path.join(gdir, f"{c['id']}.json")
        write_gains_one(gp, c["gains"])
        r, attempt = None, 0
        for attempt in range(1, args.max_retries + 1):
            r = render_one(c["id"], gp, args.out_dir, args.usd, args.amp_mm,
                           args.dur, args.fps, args.warmup_phys, args.exp_min,
                           lam=args.lam, extra=args.extra)
            if r["ok"]:
                break
            print(f"[v5render] {c['id']} attempt {attempt}/{args.max_retries} FAILED "
                  f"(black/no-mp4 — render race); re-booting", flush=True)
        r["attempts"] = attempt
        results.append(r)
    idx = os.path.join(args.out_dir, "videos", "render_index.json")
    with open(idx, "w", encoding="utf-8") as f:
        json.dump(dict(n=len(results), n_ok=sum(r["ok"] for r in results), clips=results), f, indent=2)
    print(f"[v5render] {sum(r['ok'] for r in results)}/{len(results)} clips OK -> {idx}", flush=True)
    print("[v5render] report -> done", flush=True)


if __name__ == "__main__":
    sys.exit(main())
