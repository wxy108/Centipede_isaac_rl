# Canonical Structure-Reveal Demo

From the project root, the complete presentation is one command:

```powershell
python scripts\render_structure_reveal_demo.py
```

The command is idempotent. If the canonical 2329-frame result already passes
its hash and audit checks, it prints the video path without spending another
render. Use `--force` for a deliberate full rebuild. Use `--smoke` for a short
640x360 chain test, or `--dry-run` to inspect the resolved command.

The frozen source is a camera-free, fully surviving L60/A12, seed 244949,
wave-3.0 exact-pose trace. The pipeline renders the current collision model,
wipes to a kinematic skeleton and terrain-proximity support structure, then
assembles the audited high-fidelity skin from head to tail. The terrain uses
the exact collision heightfield with smooth normals and visual-only surface
detail. The final output is 1280x720, 120 fps, unlabeled and borderless.

The 120 fps frames are presentation interpolation, not 120 Hz physics or
contact measurements. Scientific values remain bound to `source/metrics.json`
and the exact-pose fixture hashes in `SOURCE.json`.

The implementation authority is the clean edition. A legacy checkout may use
the same launcher and fixture, but it discovers the sibling clean edition
instead of carrying a second copy of the Blender skin and renderer internals.
