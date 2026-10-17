# Migration Verification

- Old repository left unchanged.
- Controller: clean target generator compared against the active legacy
  controller for 1,800 physics substeps, four independently changing wave
  commands, and all 211 target joints. Maximum position-target error: `0.0`.
  Maximum velocity-target error: `0.0`.
- Legacy target-sequence SHA-256: `aace39b3c55eeeff423ee99f3d20e73437efec16ae9ad34fc01f89fc56ae81fb`.
- Tri-terrain source SHA-256: `024fc9f8eb1cb4a62df0be6515c0e6f3e1931e3edd1109ed3d51f88de2c466d4`.
- Tri-terrain collision SHA-256: `dd8ab150ab1bb33ac6ff2fcfb0a816beea309047885192ec9cbd161a65f9142d`.
- Each retained checkpoint safely loads with `weights_only=True`, records epoch
  24, has a 34-wide normalized input, and one action output.

## Release Checks

- `python scripts/check_project.py`: PASS, including a full 1536 x 1536 frozen
  terrain rebuild and release-wide file hashes.
- `pytest -q`: 5 passed.
- Locked source inventory includes the source lock, 5 USD files, 3 checkpoints,
  and the visual-only MJCF+21 OBJ offline geometry set. Generated outputs are
  outside the source lock; the handoff directory retains one canonical
  three-condition panel under `outputs/demo/`.

## Live Isaac And Video Boundary

- Camera-free live smoke, `flat`, fixed wave `1.5`, one env, 20 steps: PASS.
  It wrote `metrics.json` and `traces.npz`; one survivor, no done, finite speed.
- Automatic `--video` smoke on the same condition: PASS. It wrote a 251-link
  exact-world-pose trace, rendered 5 sampled frames through the third-party
  offline renderer, and returned exit code 0.
- Video audit: zero black/white/blank/partial-white/strobe/outlier frames,
  robot present in every frame, `mean_motion=0.8641805053`, overall PASS.
- The recurring Warp `cuDeviceGetUuid` / CUDA error 36 warning appeared in the
  successful camera-free smoke. It is therefore not treated as a standalone
  failure criterion; artifact and audit completion are authoritative.
- The deprecated Gymnasium `RecordVideo` path was separately reproduced as an
  RTX/Hydra startup access violation and removed from the clean evaluator.
- Full command from the PI runbook, `flat`, fixed wave `1.5`, one env, 800
  steps: PASS after extending the evaluation horizon by one control step.
  Result: `survivors=1`, `first_done_step=-1`, speed
  `0.3323137462 body-length/s`, 193 video frames, and `audit.pass=true`.
  The pose trace ends at exactly `9.6 s`, remains alive, and its main-body
  centroid has moved `-0.326483 m` in world X; the final frame is not a reset.

## High-Fidelity Presentation Skin

- Blender source hash: `1904b0fffab66f7f902924c6975c73770635923236c47a10153f50e6593e858a`.
- Binding smoke: PASS. Exactly 105 source visual meshes bind to 97 current
  recorded rigid bodies; all source/target names resolve, and all 105 `col_*`
  source proxies remain hidden.
- Topology split: 21 body shells, 76 articulated leg meshes, 6 head-attached
  meshes (antennae, forcipules, mandibles), and 2 tail-attached ultimate legs.
- Material audit: PASS. The original body/leg/tail images are packed, sRGB,
  their UV/material nodes are not rewritten, and Blender uses AgX with neutral
  lights and no hue edits.
- Direct old-MuJoCo color reference has median OpenCV hue 19 for saturated
  visible skin pixels. The current three-frame Blender smoke measured 18
  (tolerance 6), so `source_hue_preserved=true`.
- Leg registration removes the old rig's transverse frame offset, extends each
  segment 0.50 mm past each current endpoint, and uses a 1.05 mm presentation
  diameter. Proximal and distal meshes overlap 1.00 mm at a shared joint. This
  is a vertex-only presentation transform; physics and collision remain
  unchanged.
- A full-video postcheck caught that the first implementation left old body
  mesh centers as much as 9.920484 mm from current link origins: frame 0 looked
  plausible, while large bends separated shells. The released binding rebases
  all 21 shells; maximum registered center residual is `0.0 m`, and maximum
  leg-span endpoint error is `8.68e-19 m`. Frames 0, 96 and 192 were then
  visually inspected for continuity before the final 193-frame render.
- Final visual tuning uses body scale margin `[1.08, 1.04, 1.04]`. The first
  postcephalic shell retains recorded X/Y and orientation while its display Z
  follows the mean of `link_body_0` and `link_body_2`; the three-frame audit
  measured zero correction in the straight pose and a maximum correction of
  `0.0018042657 m` over the full trajectory. Both textured antennae pivot at their
  head-side endpoint, point along the target head link's local -X forward axis,
  splay outward by 18 degrees, and measure 22 degrees local elevation. The
  report requires both `antennae_forward=true` and `antennae_raised=true`. All
  changes are downstream presentation geometry only.
- The source head shell's front-cap axis measured `-5.837979 degrees` in the
  target head-link frame. The released binding rotates the head shell,
  forcipules, mandibles and both antenna roots by `+16.5 degrees` about the
  posterior-center hinge `[0.00417015, 0, 0] m`, producing a measured shell
  elevation of `+8.094221 degrees`. The antennae are aimed after this group
  transform and retain their final 22-degree elevation and 18-degree outward
  azimuth. `head_visual_group_complete`, `head_visual_pitch_raised`, and
  `head_visual_hinge_preserved` are all required. Recorded head-link and
  collision poses are unchanged.
- The old final shell followed the 11.824 mm physical terminal extent plus the
  generic body margin, producing a 12.770 mm orange block. The released visual
  shell uses local scale factor `[0.67, 1.00, 1.00]` and a 2.107 mm headward
  shift, yielding about `8.56 x 7.16 x 5.27 mm` while retaining cross-section
  continuity with the preceding segment. The ultimate legs now share that
  terminal transform instead of the former independent `[0.53, 0.30, 0.30]`
  squeeze, then move another 1.10 mm headward. Left/right root overlaps are
  `0.002085 m` and `0.002903 m`. Their presentation frame now follows the
  per-frame `link_body_19 -> link_body_20` terminal tangent; the six-wave
  dorsal panel measured minimum alignment dot `0.999999892`. The
  `tail_shell_tapered`, `ultimate_legs_overlap_tail_shell`, and
  `ultimate_legs_follow_terminal_tangent` checks must all pass. Physics is
  unchanged.
- Corrected formal 45-degree render: 193/193 frames at 20 fps, MP4 SHA-256
  `dfb65f72f61098c662301a767844b1fd068f01e222b2745b6fd234045009ecab`.
  Its released chase-camera offset is `[-0.19, -0.15, 0.12] m`, about 25%
  farther than the initial presentation view. The terrain crop has 0.45 m
  margin so the wider view never exposes an unrendered boundary.
  It has zero black/white/blank/strobe frames, nonzero motion, encoded median
  hue 17 versus MuJoCo reference 19, and overall `pass=true`. The retained path
  is `outputs/demo/flat/terrain_seed_244949/open_loop_w1.5/video/high_fidelity/`;
  generated output is intentionally outside the source lock.
- The matching mild L90/A8/wave2.0 and rough L60/A12/wave3.0 rollouts also
  survived 800/800 steps. Their speeds are `0.3317343891` and `0.2625610232`
  body-length/cycle. Their 193-frame high-fidelity MP4 SHA-256 values are
  `0066f99b74f12afad75d6a60803ec231fd853830a736b5cd355bf61923c9464e`
  and `de068650489f27a89ad90f4587b270b1536608e339eb137182dafc37f51efa7d`.
  Both binding/video audits pass. All three 45-degree audits record
  `z[row,col] -> world(y[row],x[col])`, no transpose or flip, and exact copied
  collision Z with maximum saved error `0 m`. The panel index is
  `outputs/demo/TERRAIN_DEMO_PANEL_20260818.json`.
- Three additional 193-frame views independently pass the same binding and
  video audits: front SHA-256
  `297ee09a36eaa8a44ec0ce497582423e85c381110fab2ee29a7aa4b5dfe400f6`,
  side `1ad2a32a3a429c24171def436fdaf600abe14b838d97937304843d43cc0bdd73`,
  and top `f3d9e05f6be527cbd49e045a039df618ebe788c01fedf22108a585ea5d4c85eb`.
  All four report zero black/white/blank/strobe fractions and both antenna
  direction checks true.
- The unlabeled, borderless synchronized layout is `[[45deg, front], [side,
  top]]`. Its 193-frame MP4 SHA-256 is
  `7a2167e710e986cc0c92ac4b7862778f7b37adfbea1cdae06bc3821aed7fb35c`;
  median hue is 19 versus reference 19 and the composite audit passes. Frames
  0, 96 and 192 were visually inspected after encoding.
- The six-wave dorsal morphology panel (`1.5` through `4.0`) uses exact live
  Isaac poses at `2.256 s`, one common camera, `light_scale=0.45`, and
  `exposure=-0.35`. All six binding/video audits pass with packed source
  textures untouched, terminal-tangent alignment true, and no clipped model.
  The labeled and clean panels plus their hashes are recorded under
  `outputs/report/wave_number_hifi_top_panel_20260819/`.

## Structure-Reveal Presentation

- The source is a new camera-free `L60/A12`, terrain-seed `244949`, fixed-wave
  `3.0` rollout: `1667/1667` control steps, `20.004 s`, one survivor,
  `first_done_step=-1`, and speed `0.2815006971 body-length/cycle`. Its 401-frame
  exact-pose trace is finite, alive throughout, never reset, and has SHA-256
  `c1bd51f2b77f6532e31728527dc93a03e693b2a19bb26b47aa43e7bfe0aa13f6`.
- The formal video discards only the spawn-settling prefix and consumes source
  times `[0.6, 20.004] s`: 389 consecutive frames, 20 fps, 19.4 s, 1280 x 720,
  no loop and no interpolation. The chase offset is
  `[-0.24, -0.19, 0.145] m`.
- The initial layer is the current MJCF `group="0"` collision model: 211
  collision-bearing bodies. The middle layer has 210 kinematic edges, 59 nodes,
  38 foot support markers and two presentation-only scan rings. The final layer
  is the audited 105-mesh high-fidelity skin, assembled head-to-tail.
- Support markers are terrain-proximity evidence derived from exact foot world
  positions, exact collision-heightfield Z, 0.45 mm foot radius and a 1.35 mm
  fade clearance. They are explicitly not raw PhysX contacts, impulses or
  touch-sensor measurements.
- Full video audit: 389/389 frames, black/white/blank/strobe fractions all zero,
  mean motion `2.1465791711`, collision/skeleton/support/skin phase detectors all
  true, and final median skin hue 17 versus MuJoCo reference 19. Scene, binding,
  terrain-axis and video audits all pass. MP4 SHA-256:
  `59461c458834a26e1bf0912f37c9b99798aaf6f3269fe50a209b84820b81fdfe`.
- The canonical directory is
  `outputs/report/rough_structure_reveal_20260820/final/`; its `MANIFEST.json`
  binds the source trace, terrain, camera, keyframes and all audit paths.
- The smooth presentation revision is retained under
  `outputs/report/rough_structure_reveal_20260820/final_120fps/`. It uses the
  same original trace SHA-256
  `c1bd51f2b77f6532e31728527dc93a03e693b2a19bb26b47aa43e7bfe0aa13f6`.
  Positions and shortest-arc quaternion SLERP produce a 777-frame, 40 fps
  presentation trace with SHA-256
  `846f1e7de4b9497b1f8ad1bd2871b6c4a49677821193cbe5e54e72491365a5cd`;
  bidirectional AOBMC/VSBMC motion compensation then produces exactly
  2329 frames at 120 fps and 19.4 s. These are presentation samples, not
  additional physics/contact observations.
- The 120 fps MP4 SHA-256 is
  `f753e930086a119d626e461a9995173adf51c278f248441064ae88b3c24bea51`.
  Its full decode audit reports 2329/2329 frames and zero
  black/white/blank/strobe fractions; all four phase detectors, the Blender
  scene audit and the 105-mesh binding audit pass. Final encoded skin hue is
  17 versus MuJoCo reference 19.
- This revision uses a 640-point maximum deterministic terrain subset, smooth
  normals, procedural neutral-grey color variation and a 0.22 mm visual-only
  micro-bump. The saved collision-heightfield Z values and axis mapping remain
  unchanged. High-resolution collision, skeleton, skin and an actual
  interpolated frame were visually inspected; the former regular square-grid
  appearance is absent.
- The canonical source fixture is now source-locked under
  `demo/structure_reveal/source/`, with its concise contract in `SOURCE.json`.
  `python scripts/render_structure_reveal_demo.py` is the only public demo
  entry point; it resolves the current Blender renderer and validates an
  existing canonical result before returning it. `--force` rebuilds the full
  presentation and `--smoke` runs the short chain test. The legacy project has
  a byte-identical launcher and fixture but delegates renderer internals to the
  sibling clean edition, so there remains one implementation and one copy of
  the high-fidelity asset stack.
- Clean-root and legacy-root launcher smokes independently completed at
  118/118 frames, 120 fps and 0.975 s. Both passed every video, scene and skin
  binding check and produced MP4 SHA-256
  `b3e060ff0058805bcd4e2fc393652308146b93337fb8dde05abe1c05b9645f82`.
