# Centipede locomotion research — current original-project source

Local working-tree snapshot synchronized on 2026-10-17. This is the current **open-loop locomotion/contact-reliability research** workspace; the repository name is historical and does not imply a validated RL solution.

## Start here

- [Current state](PROJECT_STATE.md), then [mainline](mainline/README.md).
- [Latest completed study and limits](mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/FINAL_REPORT.md).
- [Latest delivery receipt](mainline/terrain_roughness/libo_cutoff_height_expansion_2026-10-01/FINAL_DELIVERY.json).
- [Synchronization status and portability boundary](SYNC_STATUS.md).
- [Frozen Clean edition](https://github.com/wxy108/Centipede_isaac_rl/tree/snapshot/clean-2026-10-17), which has a separate runtime and evidence contract.
- [Rendering reference](docs/rendering/README.md), preserved as historical reference snapshots.

## Evidence status

The Libo 2/4/8 mm cutoff study contains **4,055 planned conditions: 3,553 complete and 502 failed**, with 3,418 retained statistical observations under the recorded exclusion rule. Failures comprise 492 state divergences, eight boundary stops and two startup failures. Further height extension is paused for insufficient complete cases; no physical-stall threshold was established.

Different gaits retain their existing amplitudes, so their comparison does not isolate wave number as a single causal variable. Contact loss is not automatically an independent erasure probability. The information-theoretic interpretation remains a mechanism to test, not an established equality.

Random contact erasure is separately paused: 130 complete, 65 numerical divergences and 75 unscored stochastic conditions. Failed or interrupted runs are never reported as zero displacement. Newton/soft-leg work and further Clean adoption remain held.

No simulation, training, optimization or queue was started by this GitHub synchronization. Scientific source files were copied byte-for-byte; original local working directories remain untouched.
