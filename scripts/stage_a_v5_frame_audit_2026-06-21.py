"""stage_a_v5_frame_audit_2026-06-21.py — HARDENED, color-agnostic per-frame video audit.

GO_FIX_RENDER_AUDIT_GAPS: the prior audit checked only BLACK (+motion+mean-lum), so a half-WHITE/blank
clip (c0_g6e5, luminance 215.7) PASSED. This audit is color-agnostic:
  * black_frac   = frac frames with mean luminance < BLACK_MEAN (8)          — all-black
  * blank_frac   = frac frames with spatial pixel std < BLANK_STD (6)        — UNIFORM (black OR white)
  * bright_frac  = frac frames with mean luminance > WHITE_MEAN (240)        — near-white / over-exposed
                   (catches a white/gradient frame even if its std isn't tiny)
  * motion_mean/min = frame-to-frame |Δ| (freeze / no-motion guard)
A clip PASSES iff  black_frac==0 AND blank_frac==0 AND bright_frac==0 AND motion_mean>0.
Audits a list/dir of mp4s; writes a JSON + prints a table. Also has --make-blank-test to emit a clip with
a deliberately-blanked (white) frame for self-verification. numpy + cv2; no IsaacLab, no GPU.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import numpy as np

try:
    import cv2
except Exception as e:  # pragma: no cover
    print(f"[audit] cv2 unavailable ({e})", flush=True)
    sys.exit(2)

BLACK_MEAN = 8.0     # mean lum below → black frame
WHITE_MEAN = 240.0   # mean lum above → near-white / over-exposed frame
BLANK_STD = 6.0      # spatial pixel std below → uniform/blank (color-agnostic)


def audit_clip(path):
    cap = cv2.VideoCapture(path)
    means, stds = [], []
    prev = None
    motions = []
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        means.append(float(g.mean()))
        stds.append(float(g.std()))
        if prev is not None:
            motions.append(float(np.abs(g - prev).mean()))
        prev = g
    cap.release()
    n = len(means)
    if n == 0:
        return dict(path=path, n_frames=0, PASS=False, reason="no frames")
    means, stds = np.array(means), np.array(stds)
    motions = np.array(motions) if motions else np.array([0.0])
    black = int((means < BLACK_MEAN).sum())
    blank = int((stds < BLANK_STD).sum())
    bright = int((means > WHITE_MEAN).sum())
    res = dict(
        path=path, n_frames=n,
        black_frac=black / n, blank_frac=blank / n, bright_frac=bright / n,
        mean_lum=float(means.mean()), min_lum=float(means.min()), max_lum=float(means.max()),
        min_std=float(stds.min()), max_std=float(stds.max()),
        motion_mean=float(motions.mean()), motion_min=float(motions.min()),
        n_black=black, n_blank=blank, n_bright=bright,
    )
    res["PASS"] = bool(res["black_frac"] == 0 and res["blank_frac"] == 0
                       and res["bright_frac"] == 0 and res["motion_mean"] > 0)
    res["reason"] = "" if res["PASS"] else ",".join(
        x for x, c in [("black", black), ("blank", blank), ("white/overexp", bright),
                       ("no-motion", 1 if res["motion_mean"] <= 0 else 0)] if c)
    return res


def make_blank_test(src, out):
    """Self-verify: copy src but BLANK (white) a middle frame — the hardened audit MUST flag it."""
    cap = cv2.VideoCapture(src)
    fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    nf = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        if nf // 3 <= i <= nf // 3 + 5:        # blank ~6 frames to pure white
            frame = np.full_like(frame, 255)
        vw.write(frame)
        i += 1
    vw.release(); cap.release()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", nargs="*", default=[], help="mp4 paths and/or globs")
    ap.add_argument("--out-json", default=None)
    ap.add_argument("--make-blank-test", nargs=2, metavar=("SRC", "OUT"), default=None,
                    help="emit a deliberately-blanked test clip from SRC for self-verification")
    args = ap.parse_args()

    if args.make_blank_test:
        out = make_blank_test(*args.make_blank_test)
        print(f"[audit] wrote blank-test clip -> {out}")
        args.clips = list(args.clips) + [out]

    paths = []
    for c in args.clips:
        paths += sorted(glob.glob(c)) if any(ch in c for ch in "*?[") else [c]
    paths = [p for p in paths if os.path.isfile(p)]

    rows = [audit_clip(p) for p in paths]
    print(f"{'clip':52s} {'frames':>6s} {'PASS':>5s} {'black':>6s} {'blank':>6s} {'white':>6s} "
          f"{'meanL':>6s} {'minStd':>7s} {'motion':>7s}  reason")
    for r in rows:
        print(f"{os.path.basename(r['path']):52s} {r.get('n_frames',0):6d} {str(r['PASS']):>5s} "
              f"{r.get('black_frac',0):6.3f} {r.get('blank_frac',0):6.3f} {r.get('bright_frac',0):6.3f} "
              f"{r.get('mean_lum',0):6.1f} {r.get('min_std',0):7.2f} {r.get('motion_mean',0):7.2f}  {r.get('reason','')}")
    n_fail = sum(not r["PASS"] for r in rows)
    if args.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_json)), exist_ok=True)
        with open(args.out_json, "w", encoding="utf-8") as f:
            json.dump(dict(thresholds=dict(BLACK_MEAN=BLACK_MEAN, WHITE_MEAN=WHITE_MEAN, BLANK_STD=BLANK_STD),
                           n_clips=len(rows), n_fail=n_fail, clips=rows), f, indent=2)
        print(f"[audit] wrote {args.out_json}")
    print(f"[audit] {len(rows)-n_fail}/{len(rows)} PASS  ({n_fail} fail)")
    print("[audit] report -> done")


if __name__ == "__main__":
    main()
