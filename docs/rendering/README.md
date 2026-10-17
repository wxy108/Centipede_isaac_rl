# Isaac Lab Rendering Pipeline — Notes & Offline (PyVista) Renderer Guide

A standalone, exhaustive reference for rendering a small articulated robot (a centipede) in
**NVIDIA Isaac Lab / Isaac Sim**, the failure modes we hit, the pre-checks that stop them wasting
GPU time, and the **third-party offline renderer (PyVista)** we switched to for fast, deterministic,
race-free video. Extracted from the Centipede Isaac Lab RL project (notes dated 2026-05-10 → 2026-06-22).

## TL;DR
- Omniverse's RTX raytracer has a **non-deterministic black / white-frame race at init that cannot be
  eliminated** — only bounded. Left unguarded, one bad render costs ~12 min (full capture + encode +
  a 5–6 min Omniverse teardown hang). See `02_RENDERING_PROBLEMS_AND_FIXES.md`.
- We bound the cost with **pre-checks**: an inline early-abort (kills a bad boot in ~13 frames / ~0.6 s),
  a color-agnostic hardened audit (black AND white AND blank AND freeze), driver auto-retry, and a
  watchdog that force-kills the hung process on the completion marker. See `03_PRECHECKS_AND_WATCHDOG.md`.
- We eliminated the race for *our own checking* by rendering **offline on CPU with PyVista** from a
  per-frame world-pose dump: **40.6 s vs 264 s ≈ 6.5× faster, deterministic, geometry-exact**
  (A/B = 192 frames). See `04_PYVISTA_OFFLINE_GUIDE.md`.

## The one rule (renderer router)
- **Internal verification** (is the gait / terrain / pose right?) → **offline PyVista** renderer. Default for checking work.
- **Deliverable export** (a clip for a talk) → **Omniverse/Isaac**. Always **verify offline FIRST, then export.**

## Contents
1. `01_SYSTEM_AND_VERSIONS.md` — exact hardware, OS, driver, CUDA, Isaac Lab / Isaac Sim, Python, and the
   render dependency versions (+ items needing confirmation).
2. `02_RENDERING_PROBLEMS_AND_FIXES.md` — every failure mode (black buffer, white/strobe, RTX race, rebound),
   root cause, and fix — chronological.
3. `03_PRECHECKS_AND_WATCHDOG.md` — the anti-hang / anti-waste machinery, with exact thresholds and exit codes.
4. `04_PYVISTA_OFFLINE_GUIDE.md` — the third-party offline renderer: export → render → audit, terrain
   reconstruction (the two bugs), fidelity vs Omniverse, and the exact time saved.
5. `05_CANONICAL_WORKFLOW.md` — the end-to-end runbook: roles, the 14-step capture recipe, camera/warmup
   defaults, and the pre-flight checklist.
- `notes_verbatim/` — the three original project docs, copied verbatim (nothing omitted).
- `scripts/` — reference copies of the five key scripts (offline renderer, Omniverse capture, driver, audit,
  watchdog). They run inside the parent project (they import project modules); included here as reference.

## Provenance
All facts are cited to the source file:line in the parent project. Verbatim source notes are in `notes_verbatim/`.
