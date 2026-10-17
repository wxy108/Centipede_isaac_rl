# Centipede Isaac Lab RL

Centipede locomotion and reinforcement learning in Isaac Lab. This repository contains the GPU controller, simulation environment, USD/model assets, experiment scripts and tests.

## Start here

- [Setup guide](SETUP_GUIDE.md): the repository's existing installation and experiment instructions.
- [Biological references](BIO_REFERENCES.md): model and locomotion references.
- [Rendering notes](NOTES_RENDERING.md): the original in-repository notes.
- [Rendering reference collection](docs/rendering/README.md): the complete notes and script snapshots formerly stored in [`Omniverse_rendering_problem`](https://github.com/wxy108/Omniverse_rendering_problem).

The rendering collection records a particular project's environment and observations from May–June 2026. Its performance measurements and failure descriptions are historical evidence, not guarantees for every system. Files under `docs/rendering/scripts/` are reference snapshots that may depend on a later parent-project workspace; they are not installed runtime entry points for this checkout. See [provenance](docs/rendering/PROVENANCE.md) before reuse.

## Contents

| Path | Purpose |
| --- | --- |
| `controllers/` | Scalar and Torch controller implementations |
| `envs/` | Isaac Lab environment and configuration |
| `assets/`, `models/` | USD and source robot assets |
| `configs/` | Controller and simulation settings |
| `scripts/` | Conversion, simulation and experiment tools |
| `tests/` | Existing tests |
| `docs/rendering/` | Consolidated rendering documentation and historical reference scripts |

## Repository consolidation

On 2026-10-17 all 18 source files from `Omniverse_rendering_problem` were imported byte-for-byte into `docs/rendering/`, including its full original Git history. The original source head `9875196b6b6b6eae2ac6c8937aa9bbfeb244460a` is preserved on [`legacy/rendering-notes/main`](https://github.com/wxy108/Centipede_isaac_rl/tree/legacy/rendering-notes/main) and in the merged history. The source repository remains available as an archive.

This organization change leaves existing runtime scripts, physics, controllers, parameters and tests unchanged. It does not adopt the archived reference scripts as the current runtime or claim new training/rendering validation.

## Related projects

- [Centipede_MuJoCo](https://github.com/wxy108/Centipede_MuJoCo): MuJoCo biological locomotion research and backbone wave notes.
- [Centipede_Robot_Sim](https://github.com/wxy108/Centipede_Robot_Sim): CAD-derived robot simulation and benchmarks.
- [Terrain_model](https://github.com/wxy108/Terrain_model): standalone terrain-to-STL utility.
