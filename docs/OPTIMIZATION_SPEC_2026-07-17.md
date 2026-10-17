# Optimization / sweep spec — REVISED per PI 2026-07-17 (落盘)

Scope: tune the **14 PD gains** (7 joint families × Kp/Kv; `head_pitch` LOCKED, excluded). `wave_number`,
amplitudes, freq FIXED. Head lock (task 2): `head_pitch_offset = -1.0` (nose-up, MIN +24.8°, ALWAYS up).

The reward function is **not written** — it is too weight-dependent, and with 6 objectives any single scalar
(or even a Pareto front) hides the tradeoff. Instead: **evaluate all objectives per config, dump the full
table, filter/sort/threshold post-hoc.** See "GRID vs OPTIMIZATION" at the end.

---

## THE REFERENCE VARIABLES (the numbers every config is scored on)

### O1. FLAT straightness — `net_yaw_flat` [deg]  → MINIMISE |·|   (unchanged, OK per PI)
Net heading change of the **body-PCA orientation**, start → last-alive (cell:1201). NOT path-straightness:
the body wave is NOT penalised ("身体可以有波但是不能往一边拐"); only the NET turn is. Flat, main wave.

### O2. FLAT total speed — `fwd_flat` [BL/cycle]  → MAXIMISE   (unchanged, OK per PI)
On flat, heading is held straight, so the −x (`fwd`) speed ≈ the real travel. Canonical `locomotion_metrics`.

### O3. ROUGH total speed — `speed_rough` [BL/cycle, % of control]  → MAXIMISE   ⭐ REVISED (PI)
**WAS: `fwd` (−x projection). NOW: `speed` = |NET XY displacement of the mass-weighted spine centroid| / T.**
PI: *"rough速度指的是rough上几何中心的总速度而不是fwd speed，rough上根本走不了直线"*. On rough the heading
wanders, so the −x projection UNDER-counts real travel; the COM's XY-plane net displacement is the honest
"how far did it actually get". NET (not path) — path is gameable by wriggling in place (PI's earlier ruling).
= the FROZEN `speed_bl_cycle`, applied on rough, normalised to THAT boot's own control `speed` (the per-boot
normalisation cancels the terrain seed: MEASURED to hold the ratio to 0.4–2.7pp while absolute speed moves 1–6%).
`terrain = l75a12` (the PI's reference mild rough).

### O4. BELLY contact — `belly_contact` [fraction of frames]  → MINIMISE   ⭐ REVISED (PI)
**WAS: geometric — belly-link bottom minus foot-link bottom (a global-plane comparison). NOW: ACTUAL CONTACT
FORCE on the belly (spine) links, from `contact_sensor_all.net_forces_w` (`--dump-contact-friction`; the
all-body sensor covers link_body_1..18; `PhysxContactReportAPI` is on every body).**
PI: *"你的地面的高度是surface表面的平均高度吧，belly探到1.893不能说明问题吧，可能只是在坑里，不能说明belly
和ground有contact或者penetration"*. **The PI is right and this RETRACTS my geometric belly claim's strength:**
- On ROUGH the geometric test is INVALID: a belly link low in absolute z may just be OVER A VALLEY, not touching.
- Even on FLAT, "belly bottom 1.893 mm < foot-plane 2.46 mm" is geometric PROXIMITY, **not proof of contact
  force**. Contact/penetration must be read from the CONTACT SENSOR, not from a z-comparison to a mean surface.
- ⇒ `belly_contact` = fraction of frames with a nonzero contact force on ANY belly (spine) link (or the mean
  belly contact force, N). ZERO = the belly never actually touches. The geometric margin is kept only as a
  weak secondary report, NOT as the belly criterion.
- **DOWNGRADE**: my "the belly scrapes 88% of frames" was GEOMETRIC (belly is the lowest surface). It is
  SUGGESTIVE on flat, INVALID on rough, and unproven as CONTACT until the force confirms it. To be re-measured.

### O5. LOW-WAVE straightness — `net_yaw_lowwave` [deg]  → MINIMISE |·|   ⭐ NEW (PI)
PI: *"再加一个低波，wave number=1.5在平地上的表现，低波也要能走直线（net yaw，对body link pca之后的朝向不能
偏航拐弯）"*. Same body-PCA net-heading metric as O1, but at **`wave_abs = 1.5`** on flat.
- **wave mapping CONFIRMED**: `wave_abs` IS the wave number in absolute units (`n_wave_scalar = 3.0`; gains
  `wave_abs` → scale = wave_abs/3.0; effective = 3.0×scale = wave_abs). So wave number 1.5 = `wave_abs: 1.5`.
- ⚠️ MEASURED context: at `wave_abs = 1.0` the DEFAULT gains give net_yaw **+36.83°** (drifts hard the OTHER
  way). So low wave with default gains does NOT walk straight. Whether the GAINS can fix it is UNKNOWN — the
  drift is largely wave-SET (task-1 finding), so this is a genuine open question the sweep answers, not a given.
- The point: find gains that walk straight ROBUSTLY across wave numbers, not just at the one we fix.

### O6. ROUGH yaw compliance — `compliance_yaw` [mm]  → MAXIMISE   ⭐ NEW (PI — this finally DEFINES GAP A)
PI: *"rough上yaw的compliance...用整个脊柱各个body link的distribution作为参考，越compliant，bodylink形成的
身体形状和无disturbance下的body wave差别越大，越大越compliant"*. **This is a PI-chosen definition for the
standing OPEN PROBLEM (`project_yaw_compliance_open_2026-07-15`, GAP A) — record it as such.**
Concrete, in-plane (yaw), translation- and heading-removed:
1. Per frame, spine-link XY positions → body frame: subtract the mass-weighted centroid, then rotate so the
   horizontal PCA axis is along x. This removes translation AND heading, leaving only the SHAPE.
2. Shape = the signed lateral offset `lat_i(t)` of each spine link i (⊥ the principal axis, in the horizontal
   plane). Over an episode each link has a DISTRIBUTION of `lat_i` (the body wave sweeps it).
3. Reference = the FLAT episode (same gains, same wave, NO terrain) = the undisturbed body wave's distribution.
4. `compliance_yaw` = aggregate distance between the ROUGH and FLAT distributions of {lat_i}, summed over links
   (e.g. Σ_i |RMS_rough(lat_i) − RMS_flat(lat_i)|, in mm — the EXTRA lateral reshaping the terrain caused).
   LARGER = more terrain-induced in-plane reshaping = MORE compliant.
⚠️ THREE CAVEATS THAT MUST TRAVEL WITH THIS NUMBER (or it lies):
  (a) **DEVIATION CONFLATES COMPLIANCE WITH COLLAPSE.** A buckling / falling-apart robot deviates from the clean
      wave the MOST. So `compliance_yaw` is meaningful ONLY among configs that are STILL WALKING — gate on
      `speed ≥ threshold` AND not buckled AND upright. The "most compliant" config unfiltered will be the one
      that is disintegrating. This is the standing RANK≠MAGNITUDE / need-a-gate rule, again.
  (b) **IN-PLANE ONLY** (horizontal / yaw), NOT vertical (Z / pitch conformance) — the PI asked for YAW.
  (c) Report the ABSOLUTE reshaping in **mm**, not a rank, and next to `net_yaw` — a config that reshapes a lot
      AND turns a lot is drifting, not conforming.

---

## SUMMARY TABLE — what each config is scored on
| # | variable | terrain | wave | direction | source |
|---|---|---|---|---|---|
| O1 | `net_yaw_flat` (body-PCA) | flat | main | min \|·\| | net heading Δ |
| O2 | `fwd_flat` | flat | main | max | −x COM speed |
| O3 | `speed_rough` (COM XY net, % ctrl) | l75a12 | main | max | ⭐ was fwd |
| O4 | `belly_contact` (contact FORCE) | both | main | min | ⭐ was geometry |
| O5 | `net_yaw_lowwave` (body-PCA) | flat | **1.5** | min \|·\| | ⭐ NEW |
| O6 | `compliance_yaw` (shape deviation, gated) | rough−flat | main | max | ⭐ NEW = GAP A defn |

Conditions per config: {flat main-wave, flat wave-1.5, rough main-wave} = **3 evaluations**.

---

## ⭐ GRID vs OPTIMIZATION — the arithmetic (PI: "是不是又太多了？grid还是optimization")

**A 14-D grid is categorically impossible.** Even 2 levels/param = 2^14 = **16,384 configs** = 1,024 boots
(16 env/boot) × ~6 min = **~4.3 days** — and 2 levels resolves nothing. 3 levels = 4.8M configs = years.
So a full 14-D grid is OUT, and the reason is NOT the joints — it is dimension count.

**But the effective dimensionality is NOT 14 — the sweeps already MEASURED which params are inert:**
| param | measured behaviour | grid levels needed |
|---|---|---|
| body_yaw **Kp** | INERT over 5 decades (Kp/1e5 → fwd 100.3%) | 1 (pin) |
| legbase_yaw **Kv** | ~0 base; down = no-op; upper bound 3e-4 | 2 |
| legbase_pitch **Kv** | ~0 base; UP is a lever (3e-4 = +16.7%) | 2–3 |
| legbase_roll Kp+Kv | LOCKED, no CPG signal | 1 (pin) each |
| knee Kp+Kv | LOCKED, no CPG signal | 1–2 each |
| **legbase_pitch Kp** | THE speed lever; cliff at 0.03%; belly | **4–5** |
| **body_roll Kv** | biggest COM lever (+53%) | **4** |
| **body_yaw Kv** | straightness lever | **3–4** |
| **legbase_yaw Kp** | the floor | **3** |
| **body_pitch Kp+Kv** | NEVER SWEPT — the likely pitch-conformance lever | **3** each |
| **body_roll Kp** | NEVER SWEPT | 2–3 |
⇒ **~6 params carry almost all the variance.** A grid on THOSE is feasible AND is the "非常详细的sweep" done right.

**Cost with the 3 conditions (× 3 boots per 16 configs):**
| configs | ≈ params × levels | boots (×3 cond) | wall-clock |
|---|---|---|---|
| 256 | 4×4 or 6×~2.5 | 48 | ~5 h ✅ |
| 512 | 6×~2.8 or 5×3.5 | 96 | ~10 h ✅ overnight |
| 1024 | 5×4 or 6×3.2 | 192 | ~19 h ⚠️ |
STAGING cuts this hard: run the two FLAT conditions first (cheap), keep only configs that walk straight, then
run ROUGH+compliance on the survivors only. Rough is the expensive part; don't spend it on configs that fail O1/O5.

**The weight worry is ALREADY solved either way:**
- We do NOT scalarise. A GRID gives the full table → you filter/sort/threshold by any objective post-hoc, no
  weights, no premature combination. With **6 objectives** this is the ONLY clean option — a 6-objective Pareto
  front is nearly the whole space (almost everything is non-dominated), and a scalar reward needs 6 weights.
- Multi-objective BO is also weight-free (it returns a Pareto front), but 6 objectives × 14-D → sparse,
  hard-to-read front, and it samples near optima rather than mapping the landscape.

### RECOMMENDATION (mine — your call; you said "我们想想")
**A staged GRID on the ~6 measured-to-matter params, NOT the full 14, NOT a scalar reward, NOT a 6-obj BO.**
1. Pin the measured-inert params (body_yaw Kp, legbase_roll, knee) at best-known + 1 sanity level.
2. Grid the ~6 that move objectives, ~3–4 levels each (log-spaced, spanning the measured cliffs/floors) → ~512.
3. Stage: FLAT (O1,O2,O5,belly) first → filter to straight-walkers → ROUGH (O3,O6,belly) on survivors.
4. Dump the full table; you (PI) filter/sort — no weights ever committed.
This is feasible overnight, weight-free, fully interpretable, and covers interactions among the params that
actually matter. **What I need from you:** confirm the ~6-param subset + levels, or say "all 14 → BO instead".

⚠️ Head note carried forward: `head_pitch` Kp does NOT lock the head's WORLD pitch (Kp ×30000 barely moved it);
the offset −1.0 sets the pose, the body's motion sets the ±13° swing. "关节锁死" yes; "world pitch 始终不变" no.
