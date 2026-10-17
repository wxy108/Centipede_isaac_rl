# Organization and release verification — 2026-09-04

This pass repairs navigation and provenance. It does not change runtime code, model, gains, policies, reward or observations.

## What Clean contains

- Seven `centipede/` Python modules, one layered USD model, one controller config and 13-knot gait map.
- Three epoch-24 policies, selected P13 evidence, portable visual assets and a frozen presentation fixture.
- Nine scripts for checking, evaluation, training and rendering.
- An optional PI course; student data live only under `course/workspace/` and are excluded from source distribution.
- Retained demonstrations under `outputs/`; these are evidence/presentation, not a second runtime.

Before organization, 311 readable files occupied 302.73 MiB; outputs accounted for 263.84 MiB.
These counts exclude .git and inaccessible pytest caches. The source/runtime/selected-evidence footprint excluding outputs,
student state, Python caches and editor settings is about 39 MiB. Use the external inventory for exact counts.
Retained demo paths stay fixed because manifests, the course and presentations reference them.

## Provenance repair

The initial integrity check failed on controller.py; exhaustive lock comparison found exactly three drifted entries:
`centipede/controller.py`, `tests/test_core.py`, `docs/01_RUNTIME_LINE_BY_LINE.md`.
All three original locked versions were recovered from clean_project_staging and matched their recorded hashes.
The complete diff contains the PI-approved leg phase midpoint change, its regression/golden update, and matching line guide.
It introduces no rigid-tip asset or body-yaw envelope. The existing 26 CPU/course tests passed before this repair.

Authorization source: research task 019f70a4-bc8f-7303-a7a2-ac51cbe68083,
turn 01a05e3b-e9b5-7d92-8e06-3834f2bbd724: the PI explicitly requested synchronization of the phase correction
to original RL, Clean and the course. The September 4 request authorizes organization and integrity repair.
The release lock is amended only for these three reviewed entries and the organization documents/ignore rules changed here.
Old lock bytes, matching baseline files, full diff and operation hashes are preserved in the original project's organization run.
Historical `legacy_*` hashes are retained; the current target-sequence fingerprint is recorded separately.

## Scientific boundary

Current code includes the segment-center leg-phase correction. August evidence and the three retained policies predate it.
Evaluating an old policy with the current controller is a cross-contract test; prior rollout scores are not recertified.
The current phase demo is `outputs/report/segment_center_phase_demo_20260901/`.
P13 open loop remains conditional/calibration-dependent, and its historical closed-loop matrix remains incomplete.
Later rigid-main/passive-tip, body-yaw envelopes and rebuilt skin remain experimental in the original project.

## Portability boundary

The runtime source uses project-relative assets/configs and does not import original experiment code.
However, the current `isaaclab_50` environment installs Isaac Lab editably from the original project's `IsaacLab/`.
`environment.yml` documents dependencies but does not alone install the full Isaac stack.
Do not delete/move that checkout until a separately installed, pinned framework is tested.
Blender has a local default executable path that can be overridden by its renderer argument.
No new GPU/Isaac simulation was run during this audit; source/CPU checks do not establish fresh physics behavior.

## Maintenance

Run `python scripts/check_project.py` and `python -m pytest -q -p no:cacheprovider` for source/CPU verification.
Do not refresh all hashes to silence a failure: first classify every diff against its approved change.
When sharing source, omit outputs, course/workspace, __pycache__, pytest caches and editor state.
Keep assets, checkpoints, evidence and demo/structure_reveal. Package selected videos separately when needed.
Reversible organization records: sibling original `outputs/analysis/project_organization_20260904/`.
