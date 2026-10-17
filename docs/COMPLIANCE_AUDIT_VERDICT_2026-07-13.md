# Controller-Dominance Compliance Audit — Verdict & Deliverables (2026-07-13)

**Goal:** determine whether the effective pitch **stiffness** and **damping** are set primarily by *controller*
parameters vs hidden USD / actuator / contact / filtering / clipping / timestep-solver effects — separating inertia,
true dissipation, saturation, filtering, contact-modal damping, and numerical dissipation.

Method: read the actual runtime buffers + USD (not grep); trace the live controller path; then a clean **airborne
1-DOF ring-down** of a single body-pitch joint (all other joints locked), sweeping dt × armature × controller kp/kv.
All artifacts in `outputs/analysis/compliance_audit_2026-07-13/`.

---

## Deliverable 1 — Every hidden stiffness / damping / inertia / friction source (measured, with source)

Per body-pitch joint, from the loaded PhysX buffers + the USD directly (`data/phaseA_inventory.json`):

| quantity | value | classification | source (USD vs cfg) |
|---|---|---|---|
| joint drive stiffness | **0** | — | USD drive + actuator cfg (ImplicitActuator stiffness=0) |
| joint drive damping | **0** | — | USD drive + actuator cfg (damping=0) |
| joint friction (`physxJoint:jointFriction`) | **0** | — | USD |
| **joint armature** | **1e-4** | **inertia** (NOT damping) | **USD** (`physxJoint:armature`; cfg armature=null) |
| **rigid-body angularDamping** | **0.05** (every link) | **true dissipation** (∝ω) | **USD** (`physxRigidBody:angularDamping`) |
| rigid-body linearDamping | 0 | — | USD |
| effort limit | 10 Nm | saturation cap | cfg |
| velocity limit | ~∞ (`maxJointVelocity 1e6`) | — | USD (cfg's 100 not enforced) |
| joint position limit | ±90° (hard; limit stiffness/damping=0) | boundary | USD |
| sleepThreshold / stabilizationThreshold | 5e-5 / 1e-5 | artifact risk (disabled in tests) | USD |
| controller pitch torque (under run flags) | `6e-4·(q0−q)`, **0 damping** | stiffness only | our env passive spring |

**Controller path (Phase B, traced live):** under `--suppress-active-pitch --passive-spring --bio-pitch-k 6e-4
--passive-c-scale 0`, the body-pitch joint receives **only a `6e-4·(q0−q)` position spring** — the active PD (incl.
its `kv` velocity term) is computed then zeroed; passive damping `c = c_scale·k = 0`. **No** EMA / low-pass / slew /
rate-limit / deadband / friction / sign term anywhere on the pitch torque; the effort clamp is default-OFF.

---

## Deliverable 3+4 — Ring-down fits; dt × armature × kp (isolated 1-DOF, airborne, other joints locked)

Clean spring ring-downs of one body-pitch joint (kp = controller spring; peaks fit over 50–350 cycles):

| armature | kp | period | I_eff | ζ (damping ratio) | c_eff | E-loss/cycle | ωn·dt @1ms |
|---|---|---|---|---|---|---|---|
| 1e-4 | 0.5 | 88 ms | **9.8e-5 (≈armature)** | **1e-5** | 1.3e-7 | 0.011% | 0.07 |
| 1e-5 | 0.5 | 28 ms | 9.9e-6 (≈armature) | 4e-5 | 1.6e-7 | 0.046% | 0.22 |
| 1e-4 | 0.1 | 198 ms | 9.9e-5 | 8e-5 | 5.1e-7 | 0.10% | 0.03 |
| 1e-4 | 2.0 | 44 ms | 9.8e-5 | 3e-6 | 6.8e-8 | 0.003% | 0.14 |
| **0** | 0.5 | — | — | **NaN (unstable)** | — | — | — |

**Findings, separated per your rules:**
- **Inertia:** `I_eff ≈ armature` to 2–3 %. The armature (1e-4, USD) **dominates** the pitch inertia (physical link
  inertia is negligible) and sets the frequency (`ωn=√(kp/armature)`). It is *pure inertia* — it changes period, not
  energy decay. **armature = 0 is numerically unstable (NaN)** → armature ≥ 1e-5 required.
- **True dissipation:** **negligible** — ζ ≈ 1e-5…1e-4, c_eff ≈ 1e-7, energy loss ≈ 0.01–0.1 %/cycle. The
  `angularDamping 0.05` acts on the tiny *physical* link inertia, which the 1e-4 armature dwarfs, so its effect on the
  *joint* is diluted to ~nothing. (In the *full* free robot, dissipation is larger because energy spreads to ~200
  damped links + contact — a whole-body, not single-joint, effect.)
- **Timestep/solver numerical dissipation:** at fine dt (0.5–1 ms) ζ is dt-independent (= physical). At **4 ms** the
  fast/low-armature oscillation gains extra numerical damping + a period shift (ωn·dt ~1 at 4 ms for the 28 ms mode).
  → **run at 1 ms.**
- **Saturation / clipping:** torque max ≪ effort limit 10 Nm (sat fraction 0); no velocity-limit hits; the ±90° joint
  limit only confounds the *no-spring* free-decay test (drop it or bound with a spring).

---

## Deliverable — Phase F controller-dominance (the decisive test) → `fig_controller_dominance.png`

Isolated 1-DOF, airborne, dt 1 ms & 0.5 ms, armature 1e-5. Sweep controller kv (damping) and kp (stiffness):

| controller kv | c_eff (fitted) | c_eff − kv | zeta |
|---|---|---|---|
| 0 | **1.6e-7** (hidden baseline) | — | 0 |
| 5e-4 | 4.96e-4 | −4e-6 (0.8%) | 0.11 |
| 1e-3 | 9.85e-4 | −1.5e-5 (1.5%) | 0.23 |

| controller kp | k_eff (fitted) | I_eff |
|---|---|---|
| 0.5 | **0.500** | 9.93e-6 |
| 1.0 | **1.000** | 1.01e-5 |
| 2.0 | **2.000** | 9.93e-6 |

**All 7 success criteria PASS:** (1) c_eff linear in kv (slope 1); (2) k_eff = kp exactly; (3) hidden baseline c_eff
(kv=0) = 1.6e-7 = **0.03 %** of operating kv << 10 %; (4) hidden stiffness = 0 << 10 %; (5) fits change < 2 % between
1 ms and 0.5 ms; (6) torque saturation 0; (7) no contact (airborne). I_eff constant across kp confirms the plant
inertia is the armature, independent of the controller.

## Deliverable 5 — Grounded vs airborne (contact-mediated) → `fig_grounded_vs_airborne.png`

Same joint, locked body **resting on the terrain** (gravity ON) vs **airborne**:

| config | zeta airborne | zeta grounded | period | c_eff grounded |
|---|---|---|---|---|
| arm1e-4 kp0.5 | 1e-5 | **2.1e-3** | 88 ms (unchanged) | 2.9e-5 |
| arm1e-5 kp0.5 | 4e-5 | **3.0e-3** | 28 ms (unchanged) | 1.3e-5 |
| arm1e-4 kp0.1 | ~0 | **3.2e-3** | 198 ms (unchanged) | 2.0e-5 |

**Contact adds a small hidden DAMPING** (c_eff ~ 1–3e-5, zeta ~ 0.002–0.003) and **NO stiffness** (period unchanged) —
about 2–6 % of the operating kv (5e-4), under 10 % **provided kv >= ~3e-4**. Grounded controller-dominance holds as
long as the commanded damping isn't below the contact floor.

## Deliverable 2 — Isolated free-velocity decay → `fig_ringdown_examples.png` (right)

No-spring, initial joint velocity: with no restoring force the joint rotates into the ±90° position limit, so a full
free-decay is limit-confounded. Over the **pre-limit** window the velocity decays only weakly — consistent with the
near-zero hidden dissipation the spring ring-downs measure cleanly. The bounded spring ring-down is the reliable
dissipation probe; the free-decay is reported with this caveat (angular momentum ~conserved airborne).

## Deliverable 6 — Recommended Isaac Lab configuration (minimize hidden effects, stay stable)

1. **physics dt = 1 ms** (4 ms injects numerical damping into fast/low-inertia modes + shifts period; 0.5 ms no better, ~2× cost).
2. **armature = 1e-5** (lowest stable; **0 → NaN**). Low inertia floor, still stable. (Default 1e-4 fine too — higher inertia.) Armature is inertia, lossless.
3. **Joint drive stiffness = damping = friction = 0** (already so) → controller kp/kv own the stiffness/damping.
4. **rigid-body `angularDamping`**: joint-level effect already negligible (armature dominates); optionally set `physxRigidBody:angularDamping = 0` in the USD to remove the small grounded/whole-body residual.
5. Disable sleep/stabilization during dynamics probes (harness does this).
6. Effort limit 10 Nm non-binding (keep).

## Deliverable 7 — VERDICT

**Controller-dominant stiffness and damping is ACHIEVED** in the isolated joint (airborne, dt 1 ms, armature 1e-5):
`k_eff = k_controller` exactly (hidden stiffness = 0) and `c_eff = c_controller` to ~1 % with a hidden damping baseline
of 1.6e-7 = **0.03 %** of the operating value — all seven success criteria pass.

Three precise qualifications (per your interpretation rules):
- **Inertia is armature-dominated** (`I_eff ≈ armature`, a hidden USD value) — this is *inertia, not damping*; lossless;
  fixes the resonant frequency; kept because armature = 0 is numerically unstable. Owning the inertia/frequency too
  would need lowering the armature (bounded ~1e-5 by stability). Stiffness and damping are already controller-owned.
- **Grounded contact adds ~2–6 % hidden damping** (no stiffness); keep kv >= ~3e-4 to stay < 10 % grounded.
- **Run at 1 ms**; 4 ms adds numerical damping to fast modes.

Artifacts: `docs/figures/fig_controller_dominance_2026-07-13.png`, `fig_dt_armature_kp_2026-07-13.png`,
`fig_ringdown_examples_2026-07-13.png`, `fig_grounded_vs_airborne_2026-07-13.png`; data + fits + inventory in
`outputs/analysis/compliance_audit_2026-07-13/`.
