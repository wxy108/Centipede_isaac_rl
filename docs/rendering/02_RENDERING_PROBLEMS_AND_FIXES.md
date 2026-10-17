# 2. Rendering Problems & Fixes (chronological)

Every failure mode we hit in Omniverse/Isaac, its root cause, and the fix. Source: the project's
`RENDERING_PIPELINE_DEBUG_NOTES.md` + the failure museum (`2026-06-22_rendering_failures_vs_offline_fix/`).

## 2.1  Persistent all-BLACK / zero buffer — annotator detach on multi-condition reset (2026-05-11)
- **Symptom:** 20 condition MP4s all 100% black, every file exactly **6037 bytes** (H.264 keyframe-only of
  identical black); audit 100/100 black, motion 0.00. A single-rollout control rendered fine → the env/RTX
  pipeline itself was OK. Physics metrics stayed valid (they read root pose / forces, not the RGB buffer).
- **Root cause:** the annotator was created ONCE before a per-condition `reset()` loop. The first in-loop
  `inner.reset()` **detaches the replicator render product from the live camera prim on the GPU side**. The
  Python `annot` handle stays alive, so `annot.get_data()` returns **zero-filled buffers** thereafter —
  **silently** (no exception). The native `failed_frames` check only tests `rgb is None or rgb.size==0`, so a
  zero-filled buffer of the right shape PASSES it.
- **Fix:** **one condition per fresh Isaac Lab subprocess** — the camera / render-product / annotator are
  created on every boot, so no reset can detach them. (Or: explicitly destroy+recreate the render product AND
  annotator after every reset, and audit the first post-reset frame.)
- **Rule:** never render multiple reset conditions in one annotator/render-product loop unless the render
  product is recreated and verified after each reset. Preferred = one condition per subprocess.

## 2.2  The "rebound" bug — residual non-determinism, worse at non-baseline wave
- **Symptom:** even with one-condition-per-boot, some renders still come out all-zero. One session:
  wave_scale 1.0 → 0/4 fail; 2.0 → 2/3 fail (67%); 2.5 → 1/1 fail (100%). Same args succeed on retry.
- **Cause (hypothesis, not fully confirmed):** a GPU/replicator init race that interacts with the controller
  monkey-patch's first step; correlated with non-baseline wave number.
- **Mitigation:** retry (not elimination). A sharper deterministic variant — non-baseline wave on FLAT →
  reliably all-black — is handled by `--reattach-after-warmup` (recreate render product + annotator AFTER the
  warmup, on the live post-reset camera prim). But see 2.3: re-attach has its own white side-effect.

## 2.3  Half-clip WHITE / blank / over-exposed + STROBE (2026-06-21)
- **Symptom:** a 2×2 demo clip had **~53% pure-white/blank frames yet PASSED** the (old, black-only) audit and
  the size gate. Hardened audit caught it: blank_frac 0.530, min spatial-std 0.05.
- **Root cause:** `--reattach-after-warmup` **resets the RTX raytrace accumulation**; the first frames after
  re-attach are **under-converged → over-exposed/white** until RTX re-converges (slowest on the first boot).
- **Fix (for controller-pinned single clips):** `--no-reattach-after-warmup` **+ longer render warmup
  `--n-warmup 50`** — rely on the INITIAL warmup to converge RTX, don't reset it. The bad clip went 53% white
  → clean on the first attempt. Trade-off: without re-attach you re-expose the all-black rebound (2.2) for
  flat+non-baseline-wave; for those keep re-attach + warmup-after, or retry-until-hardened-audit-passes.
- **STROBE flavour:** over-exposure is intermittent → the robot flashes in/out (one clip: 48% blown, ~12
  transitions). Over-exposure is a washed **~230–237** white, not pure 255 — so test **robot-aware**:
  `blown = (mean>200 AND red-blob<0.0015)` (a blown frame loses the red robot; a bright-but-normal white-arena
  render keeps it). See 3.3.

## 2.4  The RTX black/white-frame race is non-deterministic and CANNOT be eliminated (2026-06-22)
- **Diagnosis:** the race is an Omniverse/replicator init quirk. It can't be removed; the cost was the
  problem — a bad buffer wasn't detected until AFTER the full capture + encode, then Omniverse teardown hung
  5–6 min, so the watchdog killed it at its deadline ≈ **12 min wasted per bad render × retries**.
- **Root-cause taxonomy:** (a) multi-reset annotator detach → zero buffer (2.1); (b) RTX init race, worse at
  non-baseline wave (2.2); (c) over-exposure rebound from `--reattach` → white/gradient (2.3).
- **Fix family:** make each failure CHEAP (inline early-abort, 3.1) + catch all colors (hardened audit, 3.2)
  + eliminate the race for verification by rendering offline (PyVista, ch. 4).

## 2.5  Earlier close-up failure table (2026-05-10)
| symptom | cause | fix |
|---|---|---|
| all-black mp4 (~10 KB) | chase camera too close to rough heightmap; RTX didn't converge | `--chase-offset -0.25 -0.20 0.18` min, `--n-warmup 30` |
| robot invisible at oblique view | camera too far for a ~5 cm body | use the chase camera |
| all-black at num_envs=16 | rl_games `vecenv.register` interferes with the replicator camera at multi-env | don't render n=16 with this renderer; use the dedicated demo script |
| robot walks off-frame | chase EMA too laggy | lower `--chase-alpha` (≈0.7) or raise chase-offset z |
| watchdog misses `[viz] report ->` | success-marker list incomplete | already added |
| false `unexpected_exit` | PowerShell `-like` bracket-class bug | fixed: watchdog uses `.Contains()` |

Why a close-up pipeline exists: default arena ≈ 2.2 m, body ≈ 5 cm → a far oblique camera makes the robot a
~30-pixel smudge on 1280×720. A chase camera that follows the head fixes it.
