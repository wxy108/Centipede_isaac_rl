# 3. Pre-checks & Watchdog (anti-hang / anti-waste)

Exact thresholds, exit codes, and file:line. These are what keep a non-deterministic renderer from burning
hours.

## 3.1  Inline EARLY-ABORT (the time fix)
`stage_a_v2_render_winner_2026-06-18.py:801-831`. After capturing the **first 13 frames** (`EARLY_N=12`,
fires at `f==EARLY_N`), abort immediately if they are definitively bad:
- `_uniform_bad = ((L<8) | (L>240) | (std<6)).all()` — all black / white / blank.
- `_partial_bad = (imb>0.30).all()` — all partial/gradient-white (per-quadrant pure-white >250 imbalance).
- `_strobe_bad = (djump>25).mean() > 0.6` — most frame-to-frame luminance jumps >25.
On any → print `[viz] EARLY-ABORT` and `os._exit(2)`. A lost-race boot now costs **~boot + 0.6 s, not ~12 min.**
The whole block is try-wrapped so it can NEVER break a good render (real scenes are mean 50–220, std≫6).

## 3.2  Color-agnostic HARDENED AUDIT
`stage_a_v5_frame_audit_2026-06-21.py`. Thresholds (`:30-32`):
```
BLACK_MEAN = 8.0     # mean luminance below → black
WHITE_MEAN = 240.0   # mean luminance above → near-white / over-exposed
BLANK_STD  = 6.0     # spatial pixel std below → uniform/blank (color-agnostic, catches black AND white)
```
**PASS iff** `black_frac==0 AND blank_frac==0 AND bright_frac==0 AND motion_mean>0`. Luminance on grayscale.
`--make-blank-test` whites-out a middle frame to self-verify the audit catches it. The same `hardened_ok()`
(size>50 KB + the four checks) is the driver's retry gate (`stage_a_v5_render_batch_2026-06-20.py:33-61`).

## 3.3  Robot-aware "blown-white" check
A pure-luminance test both misses washed-out (~235) failures and false-flags bright-but-normal white-arena
renders. Robot-aware test: `blown = (mean>200 AND red_blob<0.0015)`, where the red robot blob =
`(R>G+25)&(R>B+25)&(R>50)`. Offline encodes a robot-presence gate `robot_present = red_blob_frac>0.001`.

## 3.4  Extended offline audit (additional modes)
`offline_render_from_log_2026-06-22.py:316-353`: black/white/blank + `partial_white_frac` (per-quadrant
>250 imbalance >0.30) + `strobe_frac` (|ΔL|>25) + `outlier_frac` (|L−median|>max(40, 6·MAD)) + red-blob +
motion. Background is a non-saturating sky (L≈213<250) so pure white only appears on a failure. Self-test
injects each mode and asserts the right check fires.

## 3.5  Driver auto-retry
`stage_a_v5_render_batch_2026-06-20.py`: `--max-retries` default **4**; re-boot until `hardened_ok(mp4)`
passes (the race is per-boot). Deletes any stale MP4 before each attempt. File-size gate `>50_000` bytes
(all-black compresses to ~6 KB). Inside the capture script: per-frame content retry (`rgb.max()>0`, up to 4×,
else substitute last good frame); hard fail the whole MP4 if `n_failed > max(10, 0.10·n_frames)` → `return 4`.

## 3.6  Watchdog force-kill on completion marker
`run_isaaclab_with_watchdog.ps1`: on a success marker (incl. `[viz] report ->`) wait ≤10 s for the launcher's
`python.exe` descendants to exit; if they don't (Omniverse teardown hang), **force-kill ONLY the launcher's
python.exe descendants by ParentProcessId** — never broad-kill all python. Teardown hang ≈ **5–6 min**;
force-kill drops cleanup to <5 s. (RESOLVED: ~5-6 min is the measured RENDER-job hang; the "10-22 min" in the parent CLAUDE.md is a compaction error from "saved 10-22 min per launch", STATUS_REPORT_2026-05-20:136 - not a teardown duration. With the watchdog, realized cost is always ~10-15 s.)

## 3.7  QUARANTINED scripts (refuse to run, exit 3)
- `render_robustness_diagnostic.py` — the multi-reset annotator-detach bug (2.1). Kept on disk only to
  reproduce the bug.
- `p4_v34_decouple_render_2026-06-01.py` — force-installs the WRONG arena, ignores `--transition-*`.
Both require `--i-understand-this-is-quarantined` to run.

## 3.8  Mandatory checks before delivering ANY clip
1. One condition per subprocess (or recreate+audit render product after every reset).
2. Don't trust `failed_frames==0` — add `np.asarray(rgb).max()>0`.
3. Post-render: `getsize(mp4)>50_000` AND hardened audit with `motion>0`.
4. Auto-retry on failure (driver `--max-retries 4`).
5. Force-kill python on `[viz] report ->`.
6. Audit must be color-agnostic (black AND white AND blank AND freeze); ship the per-clip audit JSON.
7. For >8 s clips, auto-extend `episode_length_s` (else a mid-clip env reset teleports the robot).
