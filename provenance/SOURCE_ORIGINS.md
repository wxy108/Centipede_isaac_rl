# Source Origins

The clean project was assembled from the read-only repository
`C:\Users\wxy22\Documents\Centipede_IsaacLab_RL`. Only the following runtime
lineage was admitted.

## Runtime

- Robot wrapper and four composition layers:
  `assets/centipede_rebuild_v1_renamed.usd` and
  `assets/configuration/centipede_rebuild_v1_renamed_{base,physics,robot,sensor}.usd`.
- Offline verification geometry: the matching renamed MJCF plus its 21 body
  OBJ meshes from `models/farms/`. They are visual-only; physics continues to
  use the frozen USD. The renderer maps by exact body name and consumes logged
  world poses, so these files do not define a second dynamics model.
- Optional high-fidelity presentation skin:
  `C:\Users\wxy22\Documents\bio_experiment\model\centipede_reference\source_Centipede_v1_CC0.blend`,
  source SHA-256 `1904b0fffab66f7f902924c6975c73770635923236c47a10153f50e6593e858a`.
  The CC0 source is `Scolopendra subspinipes mutilans`; its 105 visual meshes,
  packed `QS1167-W01-body`/`ashi`/`tail` images, UVs and material nodes are
  retained unchanged. Registration poses came from the prior MuJoCo rendering
  rig `Centipede_MUJOCO-main/archive/blender_pipeline/centipede_archive/centipede.xml`,
  SHA-256 `e71d65d8d33042da62cc5e91c56c059ebec48e3df0f99358edb82a640272d8de`.
  Its `col_*` meshes are hidden references only; neither they nor the Blender
  asset are loaded by Isaac or used for collision.
- Controller values:
  `outputs/analysis/p8_wave_amplitude_map_2026-07-20/cfg/farms_controller_head8_ramp025.yaml`,
  source SHA-256 `f8b667283d95d4cfd6ca3fae8a62dabde10a8b629a1a8c2045a3cfe7107f9fb8`.
- Gait map values:
  `outputs/analysis/p10_lowwave_map_extension_2026-07-22/data/flat_locked_gait_map_v5_1p0_to_4p0_13knot.json`,
  source SHA-256 `8764f4fad304cf687454eb7d16350c5a20d8da8cbeea522ee4415d10b881f45a`.
- PPO values:
  `outputs/analysis/p8_phasec_frequency_rl_2026-07-19/meta/rlgames_cfg_dec12.json`.
- Active equations were consolidated from the P10/P13 consumer chain into
  `centipede/`; no legacy launcher or monkeypatch was copied.
- Video routing and audit rules were distilled from `NOTES_RENDERING.md`,
  `docs/OFFLINE_RENDERER_PIPELINE.md`,
  `docs/RENDERING_PIPELINE_DEBUG_NOTES.md`, and
  `docs/VISUAL_RENDERING_CLOSEUP_PIPELINE.md`. Historical render campaigns and
  failure samples were not copied.

The cleaned controller and gait files intentionally have new hashes because
comments, stale trial labels, selection notes, and unused fields were removed.
Their active numeric values are unchanged.

## Binary Hashes

| File | SHA-256 |
|---|---|
| `assets/centipede_rebuild_v1_renamed.usd` | `6dfe73a944edd3179dd792931ee13c134b68fda9818849f7780e56e235335db9` |
| `assets/configuration/centipede_rebuild_v1_renamed_base.usd` | `db128256fd4fa83f644ad4c06df7dd05aca46939b1f2e29def4eea274d222709` |
| `assets/configuration/centipede_rebuild_v1_renamed_physics.usd` | `4589990e10fb324d53956a902d0e6ff146197d39dd238670caeccdd7c4b81ef4` |
| `assets/configuration/centipede_rebuild_v1_renamed_robot.usd` | `e4a4f72dc927b5178970399c15d94c4b7e3ae74b3ba798fc898f11fd343f0b41` |
| `assets/configuration/centipede_rebuild_v1_renamed_sensor.usd` | `a16a7b67c985fc7d6621da1221de7591587f262d82a8a331d188a407bef911c6` |
| `assets/high_fidelity/source_Centipede_v1_CC0.blend` | `1904b0fffab66f7f902924c6975c73770635923236c47a10153f50e6593e858a` |
| `checkpoints/policy_seed_161803.pth` | `427e58e1fae4353ac244104f3d6165c9671f147e046e7ad2d293309c96974297` |
| `checkpoints/policy_seed_141421.pth` | `1cfd2508c60a5c215206f183a1070b8073ef9a218fdb5fbe8b3f298f3970183d` |
| `checkpoints/policy_seed_173205.pth` | `b0358fb6336540d8c2b0dc168b9a2aee67cc368cb34e3f5cdd2185521037e8de` |

The release-wide destination hashes are machine-checked by
`scripts/check_project.py` against `provenance/SOURCE_LOCK.json`.
