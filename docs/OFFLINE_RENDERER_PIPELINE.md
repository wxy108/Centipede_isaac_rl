# Offline renderer + "which renderer" router (2026-06-22, PI-directed)

> Companion to `RENDERING_PIPELINE_DEBUG_NOTES.md` (kept separate because that file hits a mount size cap).
> This is the ENTRY doc for visualization: decide your purpose, follow the branch.

## ⇄ START HERE — pick your path (role split)
- **A) INTERNAL VERIFICATION / our own checking** (is the gait / terrain / pose right?) → **use the OFFLINE third-party renderer** (this file, below). CPU, deterministic, NO RTX race, geometry-exact, ~6.5× faster. Default for checking work.
- **B) DELIVERABLE VIDEO EXPORT** (a clip for the PI / group meeting) → **use Omniverse/Isaac** → read `RENDERING_PIPELINE_DEBUG_NOTES.md`: "THE ONE WAY + INLINE EARLY-ABORT", "half-clip WHITE/BLANK + COLOR-AGNOSTIC audit", "Phase 3 black-frame fix". Workflow: **VERIFY with offline FIRST (branch A), THEN export with Omniverse.**

## Offline renderer — `scripts/sanity/offline_render_from_log_2026-06-22.py` (verification path)
Physics runs headless; the cell/render dumps per-frame ALL-link world pose via the additive default-OFF `--dump-raw-alllink` (`stage_a_vec_sweep_cell_2026-06-17.py`, `stage_a_v2_render_winner_2026-06-18.py`). Offline places each link's MJCF visual geom at its logged WORLD pose (NO FK → no forward-convention risk). Deterministic ⇒ the RTX black/white race CANNOT occur. Legs/feet are MJCF sphere/capsule primitives (that's what the model's legs ARE; body uses real `link_N.obj`).

### TERRAIN — two bugs lived here; get BOTH right, verify the RIGHT way
1. **XY orientation (a 90° transpose bug):** heightmap array → world is **`heightmap[row,col] → world (x ← col, y ← row)`, NO axis flip** (env ground truth, `arena.heightmap_value_at` + `terrain.build_terrain_mesh`). In offline use `np.meshgrid(xs, ys, indexing="xy")` (row→Y, col→X) and centre the terrain at the **ENV ARENA CENTRE (world origin), NOT the robot COM**. `indexing="ij"` rotates single-wavelength ridges 90° — orientation is SCIENTIFIC content (body-wave vs ridge alignment), not cosmetic.
2. **Absolute Z (robot must stand ON the terrain, not float ~17 mm above):** the robot walks on `terrain.build_terrain_mesh` built `mujoco_compat=True` → `terrain.to_mujoco_terrain(h)` (uint8 quantise + z-shift to world span [0, z_max]; λ60/amp16 mean +16.76 mm). The offline MUST apply `to_mujoco_terrain` too — the raw mean-centred heightmap is ~17 mm too low ⇒ legs float.
3. **VERIFY BY FOOT-CONTACT, not heightmap-value:** comparing offline vs `heightmap_value_at` is CIRCULAR on Z (both use the raw map; it proves XY only). The correct test = stance-foot min clearance from the all-link log ≈ foot radius (~0.45 mm), NOT ~17 mm. Proofs: `outputs/visual_audit/offline_renderer_2026-06-22/meta/foot_contact_proof.py` (Z/contact) + `meta/prove_terrain_alignment.py` (XY). Auditor-verified by disjoint recompute (16.96 mm → 0.45 mm after `to_mujoco_terrain`).

### Frame-audit (offline + the Omniverse early-abort use the SAME criteria) — ALL failure modes
Not just all-black: also **gradient/partial-white** (per-quadrant pure-white saturation imbalance), **strobe/flicker** (temporal frame-to-frame luminance jump), **per-frame outlier** (vs clip median), **robot-presence** (red blob each frame). Self-test: `meta/audit_selftest.py` (clean PASS + every injected failure caught).

### Timing (A/B, c1_g3e8, 192 frames)
Offline 40.6 s (CPU, from the pose log, deterministic, no RTX race) vs Omniverse 264 s (4.4 min; GPU boot+sim+render, race-prone) ≈ **6.5×**, race-free. Re-renders from an existing log are CPU-only (no GPU sim, no Isaac boot) — the big win for iterating camera/HUD; the first pose log comes from the headless sim we run anyway (`--dump-raw-alllink` ≈ free).

### Status
Geometry verified on ALL axes (XY pattern + orientation + absolute Z + foot-contact); offline viz TRUSTWORTHY for verification. Marker `RESULT-2026-06-22T19:40Z-zfx` P1 VERIFIED (auditor disjoint recompute).
