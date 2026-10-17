# Wave-Terrain Optimization Pipeline

Date: 2026-07-20

## Authority

Read in this order:

1. outputs/CURRENT_RESULT.md
2. this file
3. outputs/analysis/p8_wave_terrain_final_confirm_2026-07-20/reports/final_report.md
4. data/final_operating_point.json and data/trend_trajectory.csv in that run
5. the signed driver, analysis script and per-boot audits

The old FARMS Picture1 is a qualitative prior, not current numerical data.

## Pipeline

### 1. Validate infrastructure before interpreting dynamics

- Reproduce suspicious failures at warmup=0.
- Pair active gait with a zero-amplitude passive control.
- A first-frame or passive-control flip is infrastructure-invalid.
- A later asymmetric-contact flip after a valid upright start is a valid
  survival/non-flip outcome.
- Use one GPU owner and N=32 only.

The closed bug was a coordinate mismatch: reset sampled the raw
mean-centered heightmap while collision used the quantized, upward-shifted
mesh. Terrain-aware spawn uses the collision-coordinate maximum under a
75 mm footprint plus the existing 25 mm clearance. The feature remains
default-off and must be explicitly enabled by --terrain-aware-spawn.

### 2. Calibrate model posture independently

- Controller route: CENTIPEDE_CONTROLLER_YAML only.
- Selected YAML: outputs/analysis/p8_head_angle_calibration_2026-07-20/cfg/farms_controller_head_mild_70.yaml
- Offset: -0.7.
- Flat measured world head pitch: approximately 15 degrees.
- Do not tune posture and impedance in the same comparison.

### 3. Map the wave-terrain landscape

- Candidates: LC_Y50_POUT10 and LC_YKV50_POUT25.
- Waves: 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5.
- Terrains: flat, A6/lambda75, A12/lambda75, A12/lambda60,
  A12/lambda45.
- Seeds: 5150 and 6270.
- Replicates: 2 per candidate-wave-seed-terrain cell.
- Packing: 2 candidates x 8 waves x 2 replicates = 32 envs per boot.
- Total: 10 boots, 320 rollouts.

Do not reduce the sweep to two endpoints. Low/high regions are analysis
partitions only: low <=2.0, high >=3.0; n=2.5 remains measured.

### 4. Keep metrics separate

Primary metric:

- speed_bl_cycle: net XY displacement of the unweighted spine centroid.

Diagnostics:

- fwd_speed_bl_cycle
- net yaw
- roll RMS
- survival and non-flip
- NaN-free rate
- head world pitch
- PD effort

Do not use root bl_per_cycle, path length, a weighted reward, or a scalar
winner score.

### 5. Interpret the biological trend

Required phenotype:

- flat favors the low-wave region;
- low/high regions approach crossover as roughness rises;
- rough terrain favors high-wave speed and/or survival;
- the feasible optimum migrates upward in wave-number region.

Low-wave loss of survival on rough terrain is admissible evidence when a
high-wave configuration remains viable. Do not use minimum survival over
all waves as a candidate veto.

### 6. Select the operating point hierarchically

Order:

1. pass the PI phenotype;
2. maximize worst-terrain high-wave-region survival;
3. minimize flat head-pose error from 15 degrees;
4. maximize severe-terrain high-vs-low speed contrast.

This hierarchy selected LC_Y50_POUT10. The full impedance table is in
data/final_operating_point.json. Do not relabel it as a universal optimum:
it is the operating point for this model, gait amplitude/frequency, terrain
family and discrete wave range.

### 7. Reporting and demos

- Preserve long-form raw rows, pooled profiles, trend trajectory, source
  hashes and per-boot audits.
- Figures must show all eight wave numbers and all five terrain severities.
- Demo videos are clean pixels only: no HUD, text, labels, borders,
  dividers, padding or letterboxing.
- Keep a separate VIDEO_INDEX that maps filenames to candidate, terrain,
  wave, seed, replicate and env.

## Live Entrypoints

- scripts/analysis/p8_wave_terrain_final_confirm_2026-07-20.py
- scripts/analysis/p8_wave_terrain_final_analyze_2026-07-20.py
- scripts/analysis/p8_headup_incremental_audit_2026-07-20.py
- scripts/analysis/p8_spawn_onset_replay_2026-07-20.py
- scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py
- scripts/analysis/locomotion_metrics_2026-07-17.py

P7 BO, Stage-E, oracle scripts, quarantined code and the old -1.0 head-up
screen are historical only for this pipeline.
