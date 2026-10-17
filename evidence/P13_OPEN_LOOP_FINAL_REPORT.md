# P13 Rougher-to-Higher Wave Falsification Audit

Status: **CONDITIONAL_LOCAL_OR_CALIBRATION_DEPENDENT**

This report applies the preregistered decision rule. It does not select a post-hoc terrain ladder and does not collapse survival, speed, premeasurement validity, belly clearance, leg crossing, yaw, and roll into one reward.

## Completion

- GPU jobs: 72/72.
- Complete panel: True.
- Cells with retained inadequately initialized waves: 2; those wave-level locomotion scores are excluded from selection, not treated as missing boots.
- Premeasurement-warmup-invalid rows: 31 (1.345%).

## Preregistered Tests

- Geometry path nondecreasing: False.
- Geometry path endpoint shift: 2.0.
- Geometry path bootstrap P(endpoint rough > flat): 0.806.
- Bootstrap method: paired hierarchical terrain-seed then matched spatial-stratum resampling; the former independent endpoint calculation is retained only as sensitivity analysis.
- Positive fixed-lambda rows: 3/5.
- Positive fixed-amplitude columns: 2/4.
- Nonnegative geometry-dominance pairs: 0.6692307692307692.
- Practical (+0.25 wave) geometry-dominance pairs: 0.4076923076923077.

## Decision

- path_nondecreasing: False
- path_endpoint_ge_0p25: True
- lambda_positive_at_least_4_of_5: False
- amplitude_positive_at_least_3_of_4: False
- dominance_nonnegative_at_least_0p75: False
- bootstrap_endpoint_positive_at_least_0p95: False

## Interpretation Discipline

- `SUPPORTED` means the full preregistered conjunction passed under the calibrated controller. It is not a universal theorem over every terrain generator.
- `CONDITIONAL` means a local trend exists but is not globally robust, is too small, or depends on the controller contract.
- Any warmup termination invalidates the post-auto-reset locomotion score. The current artifact cannot causally distinguish spawn penetration from early open-loop dynamics, so such rows are reported as unresolved premeasurement failures with first-done-step timing.
- Low-wave failure with retained high-wave survival is retained as a valid phenotype.
- The `default_timing_control` string is a provenance label only. That YAML changes both ramp time and head-pitch offset, so the comparison is a default-controller-bundle sensitivity check and cannot identify a timing-only causal effect. See `P13_CONTROLLER_CONTROL_CONFOUND_CORRECTION.md`.
- P12 is compared only as action-level evidence; its controller YAML was not recorded and therefore cannot establish operating-point identity.

## Artifacts

- `data/p13_open_loop_long_form_quiescent.csv`: per-env raw audit table from one unchanged boot-index snapshot.
- `data/p13_quiescent_collection_summary.json`: snapshot hash, input receipts, row/key coverage, and CSV hash.
- `data/p13_cell_results.json`: per-cell curves, robust choices, and bootstrap draws.
- `data/p13_trend_tests.json`: preregistered tests.
- `figures/p13_open_loop_speed_curves.png`.
- `figures/p13_open_loop_survival_curves.png`.
- `figures/p13_open_loop_premeasurement_validity_curves.png`.
- `figures/p13_robust_wave_preference_heatmap.png`.
- `figures/p13_near_optimal_interval_span.png`.
- `figures/p13_preregistered_geometry_path.png`.
- `figures/p13_controller_contract_sensitivity.png`.
