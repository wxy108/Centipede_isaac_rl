# Clean-Room Migration Report

## Scope

This repository is a clean-room runtime release derived from the active
Centipede Isaac Lab contract on 2026-08-17. The original repository at
`C:\Users\wxy22\Documents\Centipede_IsaacLab_RL` was read as provenance and
left unchanged. Nothing in this release imports project code from that tree.

## Retained

- One robot wrapper USD and its four composition layers.
- One CPG-to-implicit-PD controller and its active configuration.
- One 34-D observation / 1-D wave-action environment.
- One deterministic tri-terrain generator and collision conversion.
- One frozen 13-knot wave-conditioned gait map.
- One training entrypoint, one evaluation entrypoint, one open-loop sweep
  entrypoint, and one deterministic exact-world-pose offline renderer.
- Matching visual-only MJCF/OBJ geometry required by that renderer; it is not
  an alternate physics asset.
- One optional CC0 Blender presentation skin with its original packed textures,
  plus a deterministic 105-mesh to 97-link exact-pose binding. It is isolated
  from the USD/collision runtime and can re-render existing traces without GPU
  simulation.
- Three epoch-24 policy checkpoints, one per policy seed.
- Final open-loop falsification evidence and the observation-classifier audit.
- A fail-loudly integrity checker, focused CPU tests, and provenance records.
- A separate PI teaching layer covering every runtime/config source range,
  commands, outputs, evidence interpretation, demos, and troubleshooting.
- Exactly one clean-generated canonical handoff demo (flat, fixed wave 1.5,
  9.6 s) with raw metrics/trace, four high-fidelity views, a synchronized 2x2
  view, and machine-readable audits. It lives under ignored `outputs/` and is
  evidence, not runtime source.

## Deliberately Excluded

- Historical model variants, retired observation and reward definitions, and
  superseded controller branches.
- P7, Stage-E, oracle, Bayesian-optimization, recovery, guardian, supervisor,
  watcher, receipt, and attempt-specific launchers.
- Incomplete closed-loop evaluation matrices and any claim derived from them.
- Intermediate simulations, RTX render caches, logs, temporary videos, duplicate
  plots, backups, checkpoints other than the three retained epoch-24 policies,
  and agent-authored narrative history.
- Alternate high-detail meshes, unpacked texture copies and old rendering rigs;
  only the one hash-locked packed Blender source and its skin map were admitted.
- Isaac Lab itself. It is an external pinned dependency, not vendored project
  code.

The source lock intentionally excludes `outputs/`. At handoff, every generated
run except the canonical three-condition demo panel is removed so that generated
evidence cannot be mistaken for an alternate current campaign.

## Reproduction Boundary

The release reproduces the active model, controller target sequence, terrain
construction, legal observation, action dynamics, pure-forward reward, policy
architecture, and retained checkpoint loading. The open-loop report and raw
tables are retained as final scientific evidence. The incomplete P13
closed-loop matrix is intentionally not represented as a completed result.

Run `python scripts/check_project.py`, then `pytest -q`, before simulation. The
integrity checker verifies every released file against `SOURCE_LOCK.json`,
validates the checkpoint interfaces, and rebuilds the frozen tri-terrain
hashes. Live one-environment physics and camera-free video smokes are recorded
in `VERIFICATION.md`.
