# Centipede Isaac Lab Clean

The adopted research runtime: 34 legal observations -> PPO -> one wave action -> wave-conditioned CPG -> PhysX implicit PD.
The PI code course and retained demonstration evidence are separate directories in the same project.

## Start

```powershell
conda activate isaaclab_50
cd C:\Users\wxy22\Documents\Centipede_IsaacLab_RL_Clean
python scripts\check_project.py
python -m pytest -q -p no:cacheprovider
```

Read [PI_HANDOFF.md](PI_HANDOFF.md), [the reading index](docs/README.md), and
[organization/integrity notes](docs/07_ORGANIZATION_AND_INTEGRITY.md).

## Runtime contract

- Isaac Sim 5.0.0, pinned Isaac Lab v2.2.1; physics 1 ms, policy 12 ms, episode 9.6 s, default batch 32.
- Scalar action [-1,1] -> wave [1,4], zero/reset 2.5, first-order time constant 0.25 s and rate limit 3.5 wave/s.
- A 13-knot wave map controls body/leg yaw amplitude and yaw impedance; leg pitch scale is fixed.
- Reward is world -x displacement of the unweighted 61-link spine centroid / 0.1025 m; terminal/reset transitions are zero.
- No terrain ID, absolute position or oracle roughness in policy observations/reward.
- The approved segment-center leg phase correction is adopted. September rigid-tip/envelope/skin experiments are not adopted.
- Three retained epoch-24 policies and August evidence predate the phase correction; distinguish their original contract from current evaluation.

## Tools

```powershell
python scripts\evaluate.py --terrain flat --policy-seed 161803 --headless
python scripts\evaluate.py --terrain l60a12 --fixed-wave 3 --video --headless
python scripts\render_structure_reveal_demo.py
```

Training and sweeps use `scripts/train.py` and `scripts/open_loop_sweep.py` for an explicitly chosen experiment.
The course is under `course/`; preserve its private `workspace/` drafts/progress/wrong notebook.
Generated data go in `outputs/`. The retained source fixture is under `demo/structure_reveal/`.

## Independence and evidence

Runtime assets/configs are project-relative. The current shared environment still resolves Isaac Lab inside
the original project's `IsaacLab/` checkout; install the pinned framework elsewhere before removing that dependency.
P13 open-loop evidence is conditional/calibration-dependent. No final closed-loop trend claim is certified here.
High-fidelity meshes and interpolated frames are presentation-only and do not change collision or simulation.
