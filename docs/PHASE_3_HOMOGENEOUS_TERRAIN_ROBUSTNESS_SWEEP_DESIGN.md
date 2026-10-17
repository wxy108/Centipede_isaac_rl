# Phase 3 — Homogeneous-terrain robustness sweep — DESIGN

> Status: **DESIGN ONLY. No sweep has been run.**
> Date: 2026-05-11.
> Authors: Claude Code (builder); awaiting ChatGPT review per
> `docs/CHATGPT_SUPERVISION_PROTOCOL.md`.

---

## 1. Motivation

The Phase 2 robustness sweep used the **composite transition arena**:
6 sequential blocks (rough_a, rough_b, medium, intermediate, long_wave,
flat) blended with a smoothstep partition-of-unity. Within one episode
the centipede traverses several blocks; cross-condition comparison at
fixed "severity" lumps together:

1. **block λ / amp** (the nominal severity knob)
2. **spawn x-position** (which block the env happens to start on)
3. **terrain phase** at the spawn (the random sinusoid phase under the
   robot's feet at step 0)
4. **gait phase** (the CPG `_sim_time` state at step 0)
5. **neighbour-block leakage** (every pixel is a weighted sum of up to
   4 adjacent blocks' contributions — the agent never walks on "pure"
   rough_b unless it's exactly at block-center)

Phase 2 audit findings (`outputs/visual_audit/phase2_robustness_sweep_videos/reports/visual_sweep_diagnostic.md`)
show: at fixed pinned spawn, the **same conditions** produce v_fwd
estimates that disagree with both the sweep aggregate and the held-out
validation by >10 mm/s. This is the spawn-bias × terrain-phase ×
gait-phase interaction signal — not measurement noise.

**The composite arena is therefore unfit as the terrain x-axis for the
robustness science question.** We need arenas that vary one terrain
parameter at a time and pin the rest.

---

## 2. Terrain generator audit (what exists on disk)

### 2.1 Single-wavelength generator (already exists)

`scripts/optimization/terrain.py::generate_single_wavelength_heightmap(wavelength_m, amplitude_m, seed, image_size=1024, world_size=1.0, n_components=8)`

Returns `(h_m, rms_m, peak_m)`. The heightmap is the **sum of n_components
random sinusoids** in a narrow frequency band around `1/λ` — same
spectral content everywhere in the arena, single seed, single phase.
This is the **true homogeneous-λ generator**.

Verified in code (terrain.py:65-90). Comment says "byte-faithful port"
of the MuJoCo sweep generator, so seed-matched arenas are
bit-identical between MuJoCo and IsaacLab.

### 2.2 Multi-block arena generator (used today)

`scripts/optimization/arena.py::generate_arena_heightmap(arena_size_m,
n_blocks, wavelength_range_mm, amplitude_range_m, image_size, seed, ...)`

Generates Nx×Ny blocks, each at its own (λ, amp, seed), blended with a
**partition-of-unity smoothstep tent weight** (lines 22-26 of
`arena.py`):
```
H_arena = Σ_b W_b(x,y) · H_b(x,y)   with Σ W_b = 1
```
Result: C¹ continuous, no slope kink at boundaries, but the local
spectrum varies smoothly across blocks. **Each pixel is a weighted
average of up to 4 neighbouring blocks** — the agent never sees a
single-λ patch in the interior unless `n_blocks = (1, 1)`.

### 2.3 Transition wrapper (the arena we have been using)

`scripts/optimization/arena.py::generate_flat_to_rough_transition_heightmap`
(arena.py:311). Wraps `generate_arena_heightmap` with a 6×1 layout and
the default `(λ, amp)` per block listed in the env cfg comments
(`envs/centipede_env_cfg.py:276-283`). Currently the only terrain_kind
the env builds.

### 2.4 Spawn knobs (already exist; no file edit needed to use)

The env cfg exposes (env_cfg.py:284-330):

| field | default | role |
|---|---|---|
| `transition_arena_size_m` | 2.2 | arena side length |
| `transition_rough_wavelength_mm` | 18.0 | "rough" block λ |
| `transition_rough_amplitude_m` | 0.010 | "rough" block amp |
| `transition_seed` | 42 | master terrain seed |
| `transition_image_size` | 1024 | heightmap resolution |
| `transition_block_centers_x` | (-0.917, -0.55, -0.183, +0.183, +0.55, +0.917) | per-block x centre |
| `transition_block_labels` | ("rough_a", "rough_b", "medium", "intermediate", "long_wave", "flat") | per-block label |
| `transition_spawn_jitter_x/y` | 0.025 / 0.10 | jitter inside spawn cell |
| `transition_spawn_clearance_m` | 0.025 | foot-to-ground at spawn |
| `transition_spawn_margin_m` | 0.10 | safe-zone margin from arena edge |
| `transition_spawn_seed` | 1234 | independent spawn-RNG seed |
| `transition_spawn_n_y_strata` | 4 | y-stratification cells |
| `transition_spawn_y_strata_centres` | (-0.6, -0.2, +0.2, +0.6) | y-strat centres |

Also: `envs/centipede_env.py:1490-1547` exposes `self._spawn_rng`
(numpy default_rng). Monkey-patching this attribute from a sanity
script lets us pin (block_idx, y_strat_idx) deterministically (see
`scripts/sanity/render_robustness_diagnostic.py::PinnedSpawnRng`).

### 2.5 Heightmap stats accessible at runtime

`self._transition_heightmap_m` is the numpy array stored on the env
instance (`centipede_env.py:407`). A sanity script can compute
`height_rms`, `slope_rms`, `slope_p95`, and PSD without touching env
code.

---

## 3. What "homogeneous terrain" means in this design

A terrain is **homogeneous** if every (x, y) pixel of the arena has
the **same expected spectral content**. Two practical realisations are
available without protected-file edits:

### 3.1 Strict homogeneous (single-seed, single-phase)

Generate one full-arena heightmap with
`generate_single_wavelength_heightmap(λ, amp, seed, …)`. Single seed
→ single phase pattern across the entire arena. No block boundaries.

**Use case**: cleanest physics. Phase-variance between repeats comes
only from re-seeding.

**Risk**: spatial-phase resonance with the CPG. A single phase pattern
might happen to align with the wave_num of one candidate and not
another, biasing results.

### 3.2 Phase-averaged homogeneous (multi-block, uniform λ+amp, varied seed)

Use the existing `generate_arena_heightmap` (n_blocks=(K, K))
machinery but set **every block's (λ, amp) to the same value**, with
per-block seed = seed_base + block_idx. The partition-of-unity blend
then produces a heightmap with uniform spectral character but
spatially varying phase (each block is a different random realisation
of the same spectrum).

**Use case**: averages out phase-resonance bias inside one rollout. A
single seed_base controls the whole field.

**Risk**: per-pixel blend of 4 neighbouring blocks slightly broadens
the spectral content at block boundaries — not exactly pure λ
everywhere.

**Recommendation: use 3.2 (phase-averaged) as the default for Phase 3.**
Reason: matches the existing arena machinery, avoids spatial-phase
resonance with a CPG sweep, and the residual band-broadening at block
boundaries is much smaller than the existing transition arena's
λ-mixing.

### 3.3 What is NOT homogeneous

- Anything labelled `transition` — the current arena. Different λ in
  different blocks.
- Anything with `amplitude_range_m=(a_lo, a_hi)` and `a_lo ≠ a_hi`.
- Single-phase sinusoid `h(x, y) = A·sin(2π x/λ)` — only one
  wave direction, can be walked around the wave; not a fair model of
  natural rough ground.

---

## 4. How to build homogeneous arenas without protected-file edits

### 4.1 Read-only approach A — cfg override only

Override the existing transition mode's per-block tuples to be uniform:

```python
# Inside a sanity script, BEFORE gym.make:
env_cfg.transition_rough_wavelength_mm = LAMBDA_MM
env_cfg.transition_rough_amplitude_m   = AMP_M
env_cfg.transition_seed = SEED
# These tuples are the per-block lists consumed by
# generate_flat_to_rough_transition_heightmap. Make them uniform.
# Note: this is a CFG-FIELD override, not a file edit. The cfg dataclass
# itself is constructed from defaults at env build time; mutating the
# instance is read-only w.r.t. files on disk.
```

**Caveat**: this requires the env to accept `transition_wavelengths_mm`
and `transition_amplitudes_m` as cfg fields with 6-tuple defaults. Per
my read of `envs/centipede_env_cfg.py`, only `transition_rough_*`
(scalar) and the per-block label/centre tuples exist. The 6-tuple λ/amp
list is hard-coded in `arena.generate_flat_to_rough_transition_heightmap`'s
signature (arena.py:318-319).

**Verdict**: cfg-only override is INSUFFICIENT — the per-block λ/amp
tuple is set at the function-call level inside the env, not from cfg.

### 4.2 Read-only approach B — monkey-patch the heightmap

After env construction but BEFORE the first reset:

```python
from scripts.optimization.terrain import generate_single_wavelength_heightmap
h_m, _rms, _peak = generate_single_wavelength_heightmap(
    wavelength_m = LAMBDA_M, amplitude_m = AMP_M, seed = SEED,
    image_size = env_cfg.transition_image_size,
    world_size = env_cfg.transition_arena_size_m,
)
base_env._transition_heightmap_m = h_m  # replace cached map
# Rebake the USD mesh OR rely on the existing mesh and the heightmap
# replacement only for spawn-z lookup. (USD-mesh rebuild requires
# stage edits; safer to rebuild the env with the new map via a
# protected-file edit.)
```

**Caveat**: replacing `_transition_heightmap_m` AFTER mesh build does
not change the USD collision mesh — the physics surface remains the
multi-block transition map. Spawn-z lookup uses the replaced array but
physics contacts use the old USD mesh. **Mismatch — unsafe.**

**Verdict**: monkey-patch is INSUFFICIENT for physics-correct
homogeneous rollouts.

### 4.3 Protected-file approach C — packet-gated cfg + env change

The clean approach is one small additive change behind a flag, with
ChatGPT review:

1. Add new cfg field `terrain_kind: str` already accepts a value like
   `"homogeneous"` (search the env construction switch on
   `terrain_kind`).
2. Add cfg fields `homogeneous_wavelength_mm`, `homogeneous_amplitude_m`,
   `homogeneous_seed`, `homogeneous_n_blocks` (for the phase-averaged
   3.2 variant).
3. In the env `_build_terrain` branch (find the existing
   `if self.cfg.terrain_kind == "transition"` block), add a parallel
   `elif self.cfg.terrain_kind == "homogeneous"` branch that calls
   `arena.generate_arena_heightmap` with uniform (λ, amp) tuples and
   bakes the USD mesh through the same path.
4. Add a `--homogeneous-terrain-kind` CLI hook to any sanity / sweep
   driver that needs it (optional — the cfg field is enough for
   driver scripts that build env_cfg directly).
5. Snapshot the affected files as `.before_homogeneous_<date>` per
   the supervision protocol.

**Recommended size of change**: ≤ 80 lines. Default `terrain_kind`
stays `"transition"` so Pass 8/9/10/11/12/13 behaviour is byte-identical.

**Verdict**: this is the only correct path. It is a small,
flag-gated, reversible change — exactly what the supervision protocol
governs. **This design doc proposes the change but does NOT
implement it.** Implementation requires a fresh ChatGPT packet.

---

## 5. Proposed sweep grid

### 5.1 Knobs

| knob | values | rationale |
|---|---|---|
| **terrain λ** (mm) | {30, 45, 60, 90} | spans Phase-2 useful range; coarser than current 18 mm "rough" because amp/λ = severity is what matters and high-amplitude high-frequency may not be physical |
| **terrain amp** (mm) | {2, 5, 8, 12} | gives severity S=amp/λ ∈ {2/90 ≈ 0.022, 5/60 ≈ 0.083, 5/45 ≈ 0.11, 8/45 ≈ 0.18, 12/30 = 0.40, 8/30 ≈ 0.27, 12/45 ≈ 0.27, 12/60 = 0.20, ...}. **Pick the (λ, amp) pairs that give a clean severity ladder**, not the full grid. |
| **wave_number scale** | {1.0, 1.5, 2.0, 2.5} | primary scientific knob; covers the Case-A range AND the suspected-harmful range |
| **body_imp scale** | {1.0, 1.5, 2.0} | secondary scientific knob; matches Phase 2 |
| **freq scale** | **1.0 pinned** | per project policy `feedback_freq_pinned_near_1.md` |
| **body_amp / leg_yaw / leg_pitch** | **1.0 pinned** | hold fixed during robustness sweep; their exploration belongs to a different study |

### 5.2 Recommended (λ, amp) ladder (5 severity levels)

| level | λ (mm) | amp (mm) | S = amp/λ | rationale |
|---|---:|---:|---:|---|
| L0 (flat) | — | 0 | 0.000 | baseline, h ≡ 0 (use `terrain_kind="flat"`) |
| L1 | 60 | 4 | 0.067 | gentle; matches Phase 2 long_wave |
| L2 | 60 | 8 | 0.133 | mid; one knob (amp) changed from L1 |
| L3 | 45 | 10 | 0.222 | shorter λ same regime |
| L4 | 30 | 12 | 0.400 | rough; near current rough_b severity |

Hold λ × amp pairs **at fixed (λ, amp)** per arena — homogeneous —
so the severity axis varies one parameter at a time within the
heightmap statistics, not across stitched blocks.

### 5.3 Spawn × phase design

For each (terrain × wave_num × body_imp) cell:

- **Within-arena spawn lattice** — 3×3 = 9 spawn positions:
  - x ∈ {-0.6, 0.0, +0.6} m (interior, ≥ 0.3 m from arena edge)
  - y ∈ {-0.6, 0.0, +0.6} m
- **Gait-phase offsets** — 3 values:
  - `_sim_time` offset at reset ∈ {0, T/3, 2T/3} where T = 1 / freq.
    This rotates the CPG phase by 0°, 120°, 240° at rollout start so
    wave_num × terrain-phase resonance is averaged.
- **Terrain seeds** — 3 values per arena (e.g. {42, 43, 44}). Different
  realisations of the same spectrum.
- **n_envs per cell** = 9 × 3 = 27 paired samples (single seed) or
  27 × 3 = 81 samples (3 seeds).
- **Rollout length** — 5 s post-warmup. Long enough for v_fwd to
  average over several gait cycles; short enough that drift across
  terrain doesn't change the local statistics under the agent.

### 5.4 Total runtime budget

- 4 wave × 3 imp × 5 severity × 81 paired samples = **4860 rollouts**.
  At 27 envs/cell parallel and ~6 s per rollout that's
  ~5 × 60 = **300 GPU-rollout-seconds per (wave × imp) cell**, × 12
  cells = ~60 GPU-minutes if perfectly parallel.
- Realistic wallclock: **90–120 min** including boot, warmups, and the
  10-15 s per-cell teardown overhead.
- If runtime is a concern, **drop to 3 seeds × 1 phase = 27 samples
  per cell** for a 30-40 min sweep.

---

## 6. Metrics to compute per cell

### 6.1 Per-rollout

- `v_fwd_mps` — net forward velocity over the capture window
- `F_max_N` — peak summed foot contact force
- `vz_rms_mps` — RMS vertical velocity (hopping signature)
- `z_p2p_m` — body-z peak-to-peak (bounce amplitude)
- `n_dones` — failure count (large dx_raw → reset)
- `effort_per_m` — Σ|τ·ω|·dt / net forward displacement
- `tilt_rms_deg` — RMS body roll/pitch (heading-loss signature)

### 6.2 Per cell (= mean over 27-81 paired samples)

- mean(v_fwd), std(v_fwd), p5(v_fwd), p95(v_fwd)
- **retention** = v_fwd(severity=S) / v_fwd(severity=0), per (wave, imp)
- **decline slope** = ∂ v_fwd / ∂ S (regressed over the 5 severities)
- **normalized decline slope** = slope / v_fwd(S=0)
- **worst-case** = p5(v_fwd) at the highest-S terrain
- **coefficient of variation** = std / mean within the cell

### 6.3 Paired comparisons (the key contribution over Phase 2)

For matched (spawn, phase, seed), compute the **paired difference**:
```
Δv(wave=W vs wave=1.0 | spawn, phase, seed, imp, S)
  = v_fwd(W, spawn, phase, seed, imp, S) − v_fwd(1.0, spawn, phase, seed, imp, S)
```
Take the mean and 95 % CI of Δv over the 27-81 matched samples. This
removes the spawn × phase × seed nuisance variation directly and is
the appropriate statistical test for "does wave_num improve over
baseline at this severity?".

### 6.4 Heightmap statistics per arena (for x-axis honesty)

Logged at boot + saved to a small CSV per arena:
- `height_rms` (m)
- `height_p95_abs` (m)
- `slope_rms` (rad)
- `slope_p95` (rad)
- `psd_peak_lambda_m` (sanity check: peak should be at the configured λ)
- `psd_bandwidth_at_-3dB`

These let the published x-axis be a derived terrain statistic
(e.g. `slope_rms`) rather than a label.

---

## 7. Visualisations to produce after the sweep

- **v_fwd vs severity** curves — one curve per wave_num value, faceted
  by body_imp. Error bars = paired-comparison SE.
- **retention vs severity** curves — normalised by per-condition flat
  speed (NOT by baseline flat — that confound is what spoiled Phase 2).
- **paired-difference forest plot** — Δv(wave=W vs 1.0) at each
  severity × imp, with 95 % CI.
- **heatmap** wave × imp, value = retention at the highest severity.
- **trade-off plots** — v_fwd vs F_max, v_fwd vs vz_rms, v_fwd vs
  effort/m.
- **heightmap snapshots** — top-down render of each arena (already
  available via `arena.save_topdown_png`).
- **terrain profile slice** — h(x, y=0) for x ∈ [-1.1, +1.1] for each
  arena.
- **terrain PSD** — radial-averaged spectrum, one panel per arena.
- **representative video** — 1 per cell × 5 severities × 12 cells =
  60 short clips, using the `render_single_condition_video.py` recipe
  proven in render_debug_phase3.

---

## 8. Implementation order (what would need to happen, in sequence)

1. **(THIS SESSION)** Confirm rendering pipeline works on the current
   composite arena. **Deliverable:** `render_debug_phase3/`.
2. **(THIS SESSION)** This design doc + ChatGPT review packet.
3. **(GATED ON ChatGPT verdict)** Add `terrain_kind="homogeneous"`
   support to `envs/centipede_env.py` + cfg (per §4.3). Snapshot
   `.before_homogeneous_<date>`. Audit gate G1-G5:
   - G1 finite, G2 height/slope stats within designed range, G3
     baseline (wave=imp=freq=1.0) is non-failing on every L0-L4 arena,
     G4 mesh USD matches the heightmap to FP precision, G5
     `terrain_kind="transition"` byte-identical regression.
4. **(GATED)** Write the actual sweep driver
   `scripts/sanity/sweep_robustness_phase3_homogeneous.py` modelled on
   `sweep_robustness_phase2.py` with:
   - parameterised (λ, amp, seed) per arena;
   - lattice spawn (no jitter; `_spawn_rng` monkey-patched to
     deterministic cycler over the 3×3 lattice);
   - gait-phase reseeding (override the CPG `_sim_time` initial value at
     reset);
   - paired-sample bookkeeping (one CSV row per sample, columns
     include spawn_idx, phase_idx, seed_idx, and the wave/imp scales).
5. **(GATED)** Run the small "dry-run" sweep on 1 cell × L0–L4 (200
   samples) to verify the pipeline.
6. **(GATED)** Run the full Phase 3 sweep.
7. Analyse + write the result report.
8. **(GATED on Phase 3 verdict)** Decide whether the Pass 13 reward
   formula or any RL pilot is warranted.

---

## 9. Strict non-actions for the design phase

While this design is in review:

- **No PPO training.**
- **No Pass 13 enablement.**
- **No reward weight changes.**
- **No action-bound changes.**
- **No checkpoint promotion / deletion / overwrite.**
- **No protected-file edits.** The new `terrain_kind="homogeneous"`
  branch is described in §4.3 but not coded.
- **No IsaacLab core or asset edits.**
- **No claim about wave_num robustness based on this design** — the
  design is upstream of any conclusion.

---

## 10. Criteria — what would count as evidence for / against the
robustness hypothesis

### 10.1 SUPPORT (positive result)

All of:
1. Paired Δv(wave=2.0 vs 1.0) at the two highest-severity arenas is
   **positive and statistically significant** (95 % CI excludes 0),
   averaged over 27+ paired samples.
2. The **retention curve** for wave=2.0 is **above** wave=1.0 at the
   two highest severities **at the same body_imp**.
3. wave=2.0 advantage is **robust to the (λ, amp) choice** — i.e.
   holds for at least 3 of the 4 high-severity arenas, not only one.
4. No wave > 2.0 setting outperforms wave=2.0 in retention (i.e. the
   "wave=2.0 is the optimum" picture replicates).
5. Effort/m at wave=2.0 is within 1.5× of wave=1.0 (the advantage is
   not bought via massive effort).

### 10.2 REFUTE (negative result)

Any of:
1. Paired Δv straddles 0 (95 % CI includes 0) at every severity.
2. Effect direction flips between L3 and L4 (sign change at the
   roughness end of the ladder).
3. The wave=2.0 advantage exists only on one (λ, amp) arena and
   vanishes on the others — interpreted as spatial-phase resonance,
   not robustness.
4. The advantage exists only at one body_imp and reverses at another.

### 10.3 INCONCLUSIVE

Effect direction is consistent but |Δv| is below practical relevance
(e.g. < 1 mm/s, ~3 % of baseline speed). Document and stop. Do **not**
proceed to a reward design on a 3 %-effect target.

---

## 11. Comparison to Phase 2 (what changes)

| element | Phase 2 | Phase 3 (proposed) |
|---|---|---|
| arena | composite 6-block transition | homogeneous per arena, 5 severities |
| terrain x-axis | block label | derived terrain statistic (slope_rms or amp/λ) |
| spawn | random over 24 lattice cells | deterministic 3×3 lattice per arena |
| gait phase | uncontrolled (whatever `_sim_time` happens to be) | 3-offset rotation per cell |
| terrain phase | uncontrolled (within block + stitching) | 3 seeds per arena |
| effective n per cell | 5-30 | 27 (1 seed) or 81 (3 seeds) |
| statistical test | unpaired aggregate | **paired Δv** |
| comparison fairness | confounded by spawn, phase, neighbours | matched on (spawn, phase, seed) |

---

## 12. Open questions for ChatGPT

A. **Is the homogeneous-arena cfg/env change (§4.3) approved as a
   default-OFF additive change?** Estimated diff size ≤ 80 lines,
   gated by `terrain_kind="homogeneous"`. Default `"transition"`
   remains byte-identical.

B. **Is the proposed (λ, amp) ladder (L0–L4 in §5.2) the right
   severity x-axis?** Or should we sample λ and amp on a finer grid
   (e.g. log-spaced) and let the analyst pick the comparison cuts
   post hoc?

C. **3-seed vs 1-seed Phase 3?** 3-seed gives 3× more statistical
   power per cell at 3× wallclock cost. Recommend 1-seed for the
   dry-run, 3-seed for the publication-grade sweep.

D. **Should we additionally render comparison videos on a
   sine-plus-Perlin terrain (per user note in §8 of the reboot
   prompt)?** This requires a separate generator (Perlin) which is
   NOT currently in `scripts/optimization/`. Recommend deferring to
   Phase 4.

E. **Should the spawn-z be re-derived from the homogeneous heightmap
   at reset, or pinned to (0, 0, clearance_m)?** Pinning eliminates a
   small (~mm) confound; re-deriving keeps physics-realistic foot
   contacts.

F. **Are paired-Δv with bootstrap-CI estimators acceptable as the
   primary test, or should we also pre-register a non-parametric
   sign-test on per-sample Δv > 0?**

G. **What is the minimum effect size we should be powered to
   detect?** I propose ≥ 3 mm/s absolute or ≥ 10 % relative,
   whichever is smaller in the relevant regime. With 81-paired
   samples and observed paired-SE ≈ 1.5 mm/s, we power-detect
   3 mm/s with high probability.

---

## 13. Deliverables this design unlocks (once approved)

- `envs/centipede_env_cfg.py` — adds `terrain_kind="homogeneous"`
  fields. Snapshot `.before_homogeneous_<date>`.
- `envs/centipede_env.py` — adds the homogeneous build branch.
  Snapshot.
- `scripts/sanity/sweep_robustness_phase3_homogeneous.py` — sweep
  driver (cell-wise, paired-sample bookkeeping).
- `scripts/sanity/analyze_robustness_phase3.py` — analysis (paired Δv,
  retention curves, heightmap stats vs severity).
- `docs/PHASE_3_ROBUSTNESS_SWEEP_REPORT.md` — results after run.
- `outputs/visual_audit/phase3_homogeneous/` — representative videos
  via the proven `render_single_condition_video.py` recipe.

Nothing in this list is created in the current session beyond this
design document.
