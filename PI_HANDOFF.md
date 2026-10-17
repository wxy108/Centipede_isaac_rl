# PI Handoff

Current organization note, 2026-09-04: [scope and verification](docs/07_ORGANIZATION_AND_INTEGRITY.md).
The segment-center leg phase correction is already adopted. Later rigid-tip/body-yaw/skin refinements remain held.
The August demos and retained policies below are historical evidence under the previous phase contract.
For the updated controller's demo, use `outputs/report/segment_center_phase_demo_20260901/`.
The source-lock mismatch caused by the already-approved phase change was reviewed against matching old sources,
not resolved by regenerating every hash. The local editable Isaac Lab dependency still points into the original tree.

## What Is Current

The runnable contract is `34 observations -> PPO -> 1 wave action`, at 12 ms,
with a pure spine-centroid forward-displacement reward. The wave-conditioned
gait map binds wave number to body/leg yaw amplitudes and yaw impedance. The
training terrain contains three longitudinal strips: flat, L90/A8, and L60/A12.

Three scratch policies are retained at epoch 24:

- `checkpoints/policy_seed_161803.pth`
- `checkpoints/policy_seed_141421.pth`
- `checkpoints/policy_seed_173205.pth`

## Canonical Handoff Demos

Open these three files first; no Isaac or Blender launch is required:

```text
outputs\demo\flat\terrain_seed_244949\open_loop_w1.5\video\high_fidelity\high_fidelity_45deg.mp4
outputs\demo\l90a8\terrain_seed_244949\open_loop_w2\video\high_fidelity\high_fidelity_45deg.mp4
outputs\demo\l60a12\terrain_seed_244949\open_loop_w3\video\high_fidelity\high_fidelity_45deg.mp4
```

This is one canonical three-condition panel: flat/wave1.5, mild L90/A8/wave2.0,
and rough L60/A12/wave3.0, all at terrain seed 244949, one environment, 9.6 s,
and 193 frames. All three rollouts survived and measured `0.3323137462`,
`0.3317343891`, and `0.2625610232 body-length/cycle`, respectively. The
machine-readable index is `outputs/demo/TERRAIN_DEMO_PANEL_20260818.json`.
Every video passes frame, motion, color, binding, and terrain-coordinate audits.
The high-fidelity head shell is presentation-only: its source visual down-pitch
is corrected to a measured `+8.094 degrees`; collision, recorded link poses,
controller, reward, and metrics are unchanged.

The canonical model-structure presentation is:

```text
outputs\report\rough_structure_reveal_20260820\final_120fps\rough_structure_reveal_45deg.mp4
```

It is 1280x720, 120 fps, 2329 frames and 19.4 s. It presents the collision
model, kinematic skeleton/near-terrain support structure and high-fidelity skin
on one continuous rough-terrain trajectory. Reproduce or locate it with one
command: `python scripts\render_structure_reveal_demo.py`. The frozen fixture
and concise contract are under `demo/structure_reveal/`. The 120 fps samples
are presentation interpolation, not 120 Hz physics/contact evidence.

## Evidence Boundary

- Open loop: complete, 72/72 GPU cells. The preregistered verdict is
  `CONDITIONAL_LOCAL_OR_CALIBRATION_DEPENDENT`, not universal support for a
  rougher-terrain-to-higher-wave law. See `evidence/P13_OPEN_LOOP_FINAL_REPORT.md`.
- Observation: the tested linear whole-boot classifier found usable roughness
  information. At 1.44 s, the no-wave/no-phase 31-D subset reached 0.9444
  balanced accuracy under seed-and-wave double holdout; the phase-only control
  was 0.3333. This proves linearly accessible information in the tested panel,
  not complete terrain reconstruction or guaranteed PPO use.
- Closed loop: the three policies are complete, but the P13 policy-seed x
  terrain-seed evaluation matrix was incomplete at migration. No final
  closed-loop trend claim is carried into this repository.

## 34-D Observation

1. `realized_wave`.
2. `cpg_sin`, `cpg_cos`.
3. Body-frame root linear velocity: 3 channels.
4. Body-frame root angular velocity times 0.25: 3 channels.
5. Projected gravity: 3 channels.
6. One-second contact history: first three spatial DCT modes of 19-segment
   contact standard deviation, first three of contact spike rate, plus angular
   velocity crest/spike and total-load standard deviation/spike: 10 channels.
7. Current 21-link spine geometry after root-frame rotation, chord detrending,
   and chord-length normalization: four lateral DCT modes, four vertical DCT
   modes, lateral/vertical curvature RMS, edge-length CV, and tortuosity: 12 channels.

## First Five Minutes

1. Open the canonical 2x2 demo above and its
   `high_fidelity_multiview_2x2.audit.json`.
2. Read `docs/README.md`, then `docs/00_PI_MASTER_RUNBOOK.md`.
3. Run `python scripts/check_project.py`, then `pytest -q`.
4. Only if a fresh runtime check is needed, run the 20-step flat smoke in the
   master runbook.
5. Before scientific comparison, run one retained policy on flat and L60/A12.
6. Treat 32 environments as parallel deterministic conditions, never as 32
   independent experimental replicates.

The migration-time smoke and the complete 9.6 s flat demo have both passed;
there is no pending guardian gate in this clean edition. Still keep one Isaac
GPU owner at a time and inspect unknown GPU processes before terminating them.

The old repository was not modified or deleted. It remains provenance only and
is not a runtime dependency except that the existing conda environment has an
editable Isaac Lab installation pointing into its `IsaacLab/` checkout.
