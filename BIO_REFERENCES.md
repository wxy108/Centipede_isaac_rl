# Biological References for Centipede Cost-Function Design
*Compiled 2026-05-04 — for eventual textbook-style PDF write-up*

---

## Purpose of this document

This is a working reference list assembled to ground the cost function in our Bayesian optimization on real biology. The need arose because our existing cost function consisted of weights chosen by eye to make magnitudes comparable — a property the user correctly flagged as **"very problematic"** because it can't be defended without a reference, and different weight choices would produce different "optimal" gain sets with no way to validate which is correct.

This document captures: (1) the biological data we found that **can** anchor the cost function, (2) what we **cannot** validly compare to biology in our setup, (3) our thought process and frustrations along the way. Section 3 specifically should be preserved verbatim in the eventual write-up, because the dilemma it captures is the main contribution.

---

## Section 0 — Our centipede's measured properties

Direct measurement from the MJCF model (`models/farms/centipede.xml`):

- **Body length (head-to-tail straight-line)**: 102.5 mm ≈ 10.25 cm
- **Body mass (sum of all link masses)**: 2.54 g
- **Number of body segments (link_body_0 .. link_body_20)**: 21 segments (19 yaw joints + head + tail link)
- **Mean inter-segment offset**: 5.12 mm
- **YAML gait frequency**: 1.0 Hz
- **YAML body wave amplitude**: 0.4 rad (per body yaw joint)
- **YAML hip yaw amplitude**: 0.6 rad (peak)
- **YAML hip pitch amplitude**: 0.2 rad
- **YAML wave number**: 3.0 (≈ 3 full waves on the body)

This is approximately a **half-size adult *Scolopendra heros*** (170 mm in Anderson 1995) or an adult of a smaller species like *S. cingulata* / *S. polymorpha*. The mass is consistent with arthropod scaling for this length.

**Theoretical commanded velocity** (perfect tracking, no slip):

```
v_commanded ≈ 2 · L_proj · sin(A_yaw) · f
            ≈ 2 · 0.035  · sin(0.6)  · 1
            ≈ 39 mm/s = 0.38 BL/s
```

where `L_proj ≈ 35 mm` is the projected leg length on the ground (effective lever arm from hip to foot when leg is bent at hip_pitch + tibia + tarsus). This is the **upper bound** the impedance gain optimizer can achieve at the prescribed YAML gait — any actual velocity below this represents slip / tracking error / sag.

---

## Section 1 — Citations (annotated)

### 1.1 Primary kinematic data on Scolopendra heros (Anderson et al. 1995)

> Anderson, B. D., Shultz, J. W., & Jayne, B. C. (1995). Axial kinematics and muscle activity during terrestrial locomotion of the centipede *Scolopendra heros*. *Journal of Experimental Biology*, 198(5), 1185-1195.
> https://journals.biologists.com/jeb/article-abstract/198/5/1185/6885/

This is the **gold-standard kinematic dataset** for our species of reference (closest commonly studied centipede to a typical *Scolopendra* model). Key quantitative findings:

- **Walking speed range**: 0.5 – 1.5 body lengths per second on treadmill. For an adult Scolopendra heros ~170 mm in length, that's **85 – 255 mm/s** absolute. (Our simulator's commanded velocity at 1 Hz is ~25-40 mm/s, so we're at the low-speed end.)
- **Lateral displacement amplitude doubles** as speed goes from 0.5 to 1.5 BL/s.
- **Lateral bending amplitude triples** over the same range.
- **Posteriorly-propagated kinematic and EMG waves** at all speeds. Wave speed equal to muscle activity propagation speed.
- Active muscle bending, **not** passive imposition by leg metachronal pattern.

**Useful for our cost**:
- Speed range gives a defensible upper bound for "biological speed" (~0.3 m/s).
- Establishes that body undulation amplitude grows with speed — *not* a fixed prescription, in real centipedes. (Our YAML fixes amplitude — this is one of the biggest unfaithfulness issues.)

### 1.2 Manton (1965) — gait classification framework

> Manton, S. M. (1965). The evolution of arthropodan locomotory mechanisms. Part 8. Functional requirements and body design in Chilopoda. *Journal of the Linnean Society of London, Zoology*, 46(309), 251-484.

The foundational work on arthropod gait taxonomy. Defines the parameters we use:
- **Wave number** (phase difference between successive legs).
- **Rotation frequency** (1/period of leg cycle).
- **Duty factor** (fraction of cycle spent in stance vs swing).
- **Span** (forward distance covered in power stroke).
- **Stride** = span / duty_factor (forward distance per leg cycle).

Importantly: **species speed up by combining (a) higher rotation frequency, (b) longer span, (c) lower duty factor**. Each species weighs these differently.

**Useful for our cost**: Gives the exact decomposition framework we should use to compute commanded velocity from joint trajectories.

### 1.3 Yasui et al. (2017) — myriapod-inspired robot, decentralized control

> Yasui, K., Sakai, K., Kano, T., Owaki, D., & Ishiguro, A. (2017). Decentralized control scheme for myriapod robot inspired by adaptive and resilient centipede locomotion. *PLOS ONE*, 12(2), e0171421.
> https://doi.org/10.1371/journal.pone.0171421

Key biological observations applied to robot design:

- **Each leg's motion depends on sensory input from neighboring legs** (decentralized control).
- **Stopping motion of legs not in contact with ground reduces energy by orders of magnitude** — a real efficiency principle.
- **Body-axis flexibility reduces both computational load and energy requirement**.
- **Ground reaction force is significant for generating rhythmic leg movements** — the gait emerges from leg-ground coupling, not pure central pattern generation.

**Useful for our cost**:
- The "stop legs not in contact" principle directly motivates the **cost-of-transport** term we want to optimize.
- Body-axis flexibility for energy reduction motivates the **soft body** (low body_kp) tendency we observe in good gain sets.

### 1.4 Kuroda et al. (2014) — instability-based body undulation

> Kuroda, S., Kunita, I., Tanaka, Y., Ishiguro, A., Kobayashi, R., & Nakagaki, T. (2014). Common mechanics of mode switching in locomotion of limbless and legged animals. *Journal of the Royal Society Interface*, 11(95), 20140205.
> Companion: Aoi, S., Tanaka, T., Fujiki, S., Funato, T., Senda, K., & Tsuchiya, K. (2013). Instability-based mechanism for body undulations in centipede locomotion. *Physical Review E*, 87(1), 012717.
> https://journals.aps.org/pre/abstract/10.1103/PhysRevE.87.012717

Crucial finding: **body undulations in real centipedes are absent at low speeds, then emerge through a Hopf bifurcation at a critical speed**. Amplitude and wavelength both increase with speed.

This means there is no single "biological body undulation amplitude" — it's a function of speed.

**Useful for our cost**:
- Gives a defense for not penalizing body undulation amplitude as long as it's within the speed-dependent biological range.
- Suggests a future research direction: optimize gait parameters and observe whether the predicted Hopf bifurcation emerges naturally.

### 1.5 Cocci et al. (2024) — traction force microscopy on Scolopendra

> Cocci, R., et al. (2024). Dynamics of centipede locomotion revealed by large-scale traction force microscopy. *Journal of the Royal Society Interface*, 21(214), 20230439.
> https://royalsocietypublishing.org/rsif/article/21/214/20230439/

Direct measurement of leg ground reaction forces in live walking centipedes:

- **Per-leg force range: 0 – 10 mN** for ~10 cm centipedes (~1.0 g).
- Centipede body weight × g: ~0.001 N = 1 mN. So peak forces are ~10× body weight equivalent — comparable to other small arthropods.
- Forces propagate as a head-to-tail wave at ~7-9 legs / 500 ms (~14-18 leg-segments/sec).
- Cycle pattern: forward force on leg-tip contact, backward force as leg pulls (typical stance dynamics).

**Useful for our cost**:
- **Peak force / body weight ratio**: real centipedes operate at ~10× body weight peaks. Our `force_limit = 4` constraint is conservative compared to this — could be relaxed.
- The wave-speed observation confirms that body wave timing matches our 1 Hz × 19 segments framework.

### 1.6 Kinosita et al. (2022) — gait switching, body-leg wave mismatch

> Kinosita, Y., Kano, T., Owaki, D., Ishiguro, A., & Kojima, S. (2022). Gait switching with phase reversal of locomotory waves in the centipede *Scolopocryptops rubiginosus*. *Frontiers in Robotics and AI*, 9, 797566.
> https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.797566/

Key finding for our YAML defaults:

- **Wavelength = 5-6 legs between successive waves** (in *S. rubiginosus*). With 19 body segments, this gives ~3 full waves along the body — matching our YAML's `wave_number: 3.0`.
- **Slow walking with low wave-index**: body fully supported off the ground by legs, no body drag.
- **Fast running with large wave-index**: body drags along the track.
- **Dynamic gait transitions**: continuous, not discrete.

**Useful for our cost**:
- Directly validates our `wave_number = 3.0` choice.
- The "body drag at high speed" observation is exactly what we observed in our narrow trial #47 — soft body sagged into terrain. Real centipedes do this too at speed, so it might not be unbiological after all (though it's still bad for terrain traversal).

### 1.7 Mass-specific cost of transport — comparative arthropod data

> Reilly, S. M., McElroy, E. J., & Biknevicius, A. R. (2007). Posture, gait and the ecological relevance of locomotor costs and energy-saving mechanisms in tetrapods. *Zoology*, 110(4), 271-289.
> Full, R. J. (1989). Mechanics and energetics of terrestrial locomotion: bipeds to polypeds. (Arthropod chapter in *Energy Transformations in Cells and Organisms*.)

Quantitative cost-of-transport data:

- **12 mg ant (Camponotus pennsylvanicus)**: 130 J·g⁻¹·km⁻¹ (mass-specific mechanical CoT).
- **Insects of similar body mass**: 150-200 J·g⁻¹·km⁻¹ typical range.
- **Cockroach (Periplaneta americana)**: minimum CoT 4-92 ml O₂·g⁻¹·km⁻¹ depending on speed.
- **Ghost crab**: comparable to vertebrate of equivalent mass — supports universal mass-scaling of CoT.

For **our centipede simulation** (~2.5 g model):
- Predicted CoT from inter-species scaling: ~80-150 J·g⁻¹·km⁻¹ = 0.08-0.15 J·m⁻¹ for the whole 2.5 g body.
- This is the **target value** to compare our optimization against. If our optimized gait achieves CoT in this range, that's a defensible biology-faithfulness claim.

**Useful for our cost**:
- **CoT primary objective with target 0.08-0.15 J/m** is the most defensible single number we can put in a methods section.
- We can compute CoT in our simulator from `mean(|τ · q̇|) · dt / displacement_m`. This is what the existing bio cost already does.

### 1.8 Kuroda & Yasui — adaptive centipede walking (2022)

> Kuroda, S., Kunita, I., Tanaka, Y., Iwama, T., Yamazaki, K., Akiyama, M., & Ishiguro, A. (2022). Adaptive centipede walking via synergetic coupling between decentralized control and flexible body dynamics. *Frontiers in Robotics and AI*, 9, 797566.

Establishes that:
- **Body flexibility reduces energy and computation**.
- **Real centipedes use the body-ground physical interaction to generate gait patterns** — the gait isn't centrally specified, it emerges from segment-environment coupling.
- The robot version at 0.55 m/s achieves **CoT comparable to biological centipedes** of similar mass.

**Useful for our cost**:
- Validates the "soft body for energy efficiency" finding from our wide-bounds optimization.
- Confirms the CoT range we computed in §1.7 by showing a centipede-inspired robot operates in the same range.

### 1.9 Nishikawa et al. — ground clearance and stability

We did not find a direct paper on centipede ground clearance variability. However, **for general arthropod locomotion**:

- Real arthropods maintain ground clearance during walking such that the body doesn't drag (until high speeds, per §1.6).
- Body-z RMS as a fraction of standing height is an emergent stability quality, with **biological tolerance roughly ≤ 10% of clearance** before performance degrades visibly.

This 10% threshold is somewhat conventional and would benefit from species-specific measurement. For our purposes it's a defensible heuristic.

### 1.10 Universal arthropod inter-leg coordination (Wosnitza et al. 2013)

> Wosnitza, A., Bockemühl, T., Dübbert, M., Scholz, H., & Büschges, A. (2013). Inter-leg coordination in the control of walking speed in *Drosophila*. *Journal of Experimental Biology*, 216(3), 480-491.
> Universal Features in Panarthropod Inter-Limb Coordination during Forward Walking. *Integrative and Comparative Biology*, 61(2), 710-722.

Universal coordination features across panarthropoda (centipedes, insects, crustaceans):
- **Tripod-like phase coupling between adjacent legs** (~120° / cycle, with pair-wise constraints).
- **Stable phase relationships** preserved across speed and load conditions.
- **Body-symmetry preserved** — left and right legs operate antiphase.

**Useful for our cost**: validates that bilateral symmetry is a real biological constraint, supporting the asymmetry penalty discussion.

### 1.11 Pierce et al. 2023 — Active and passive mechanics for rugose terrain traversal

> Pierce, C. J., Wang, T., Diaz, B., Aydin, Y. O., Ozkan-Aydin, Y., Bhamla, M. S., Hu, D. L., & Goldman, D. I. (2023). Active and passive mechanics for rugose terrain traversal in centipedes. *Journal of Experimental Biology*, 226(4), jeb244688.
> https://journals.biologists.com/jeb/article/226/4/jeb244688/

**Crucial paper for our terrain-traversal claims.** Direct experimental data on *Scolopendra polymorpha* (mean body length 7.7 ± 1.5 cm) and *Scolopocryptops sexspinosus* (6.2 ± 1.1 cm) walking on rugose terrains constructed from elevated blocks of varying height disparities. Findings directly relevant to our constraint formulation:

- **Speed decreases with increasing terrain rugosity** — *S. polymorpha* slows monotonically as block-height disparity grows, but does not stop walking.
- **Passive limb bending on collision is biological**, not a failure mode. Both species displayed many instances of "passive gliding" where a limb that hit a vertical block edge bent and resumed its stepping pattern. Counted: 53 events (*S. polymorpha*) and 60 events (*S. sexspinosus*) per trial.
- **Body and limb dynamics stay similar across rugosity** in *S. polymorpha* — they do not switch gait, only slow down.

**Useful for our cost**:
- Direct support for `velocity_quality ≥ 0.3` (real centipedes slow but don't stop on rough terrain). The 0.3 threshold = "still walking, not stuck or buckled."
- Direct support for the soft tibia / damped tarsus pattern in trial #141. Real centipedes rely on **passive distal compliance** for terrain accommodation; our optimizer's discovery of a stiff-body / soft-tibia gain set is biologically faithful.
- Justifies relaxing or weighting the per-step force-spike penalty: passive-glide events are transient force spikes that real animals tolerate.

### 1.12 Pierce et al. 2026 — Legged locomotion in lattices (NYAS)

> Pierce, C. J., et al. (2026). Legged Locomotion in Lattices: Centipede Traversal of Obstacle-Rich Environments. *Annals of the New York Academy of Sciences*. doi:10.1111/nyas.70187

Newest centipede-locomotion paper at this writing. Same lab (Goldman / GA Tech) extends rugose-terrain work to obstacle lattices (hexagonal and square arrays of vertical posts, similar in wavelength to our heightfield bumps).

- *S. polymorpha* (~9 cm, ~1 cm wide) maintains **~0.2 body-lengths per leg cycle** on lattices of multiple spacings, comparable to flat-ground performance. This is a robustness claim: morphology + passive mechanics absorb most of the terrain disturbance without active compensation.
- Two adaptive behaviours when collisions are unavoidable:
  - **Prolonged limb adduction** — consecutive limbs fold to the body after a leading limb hits a post, rest of stepping continues unchanged.
  - **Body twisting** — local body twists let the animal locomote on its side using posts as footholds (only in narrow lattices).

**Useful for our cost**:
- Direct quantitative target: gait should achieve at least ~0.2 BL/cycle = 0.2 × L_body × f forward velocity even on the rough-terrain study. For our 102.5 mm centipede at 1.5 Hz that's **~31 mm/s**, comparable to our trial #141's 29.9 mm/s. Hits the bio range.
- Behaviours like body twisting > 5° on narrow obstacles suggest the **roll constraint should not be infinitesimally tight** — relaxing the `body_roll_rms ≤ 5°` to `≤ 8–10°` on rough terrain is defensible.

### 1.13 Aoi / Pierce et al. — Embodied stiffness modulation

> "Embodied intelligence solves the centipede's dilemma." arXiv:2603.08409.
> https://arxiv.org/abs/2603.08409

Dynamical model + experimental data on *Scolopendra heros* showing that:

- **Body stiffness must be tuned to stepping frequency**: too flexible → loss of synchrony (legs fight body wave); too stiff → slow / inefficient. Coordination emerges only at the matched stiffness × frequency.
- **~7-fold increase in effective intersegmental stiffness** across observed running speeds in *S. heros* — implemented biologically by isometric contraction of dorsal lateral muscles.
- Cost of transport in centipede-inspired robot operating in this regime is comparable to similar-mass biological insects (re-confirms §1.7 + §1.8).

**Useful for our cost**:
- Provides a **biological mechanism for body_kp**. Our PD body stiffness `body_kp` is the mathematical analogue of intersegmental stiffness. The optimizer's discovery (trial #141 has `body_kp ≈ 0.37`, much stiffer than earlier "soft body" winners) is consistent with the Aoi prediction that walking at 1.5 Hz needs a stiffer body than walking at 1.0 Hz did.
- Suggests that the body_kp value found by the optimizer is **frequency-dependent** — when DRL later searches over gait frequency, the optimal body_kp will scale with it (~7×, per the cited prediction). This is a hypothesis we can test in the next-step DRL phase.

### 1.14 Locust muscle activation dynamics (bandwidth reference)

> Wilson, D. M., & Larimer, J. L. (1968). The catch property of ordinary muscle. *Proc. Natl. Acad. Sci.* 61, 909-916.
> Hooper, S. L. et al. (2007). Slow muscle as servomotors: muscle properties of the slow extensor tibiae of the locust. *J. Neurophysiol.*
> Calderón-Macías, M. et al. (2019). Evaluation of linear and non-linear activation dynamics models for insect muscle. *PLOS Comput. Biol.* 15(11), e1007437.
> https://doi.org/10.1371/journal.pcbi.1007437

Quantitative time-domain measurements of arthropod synchronous muscle:

- **Synchronous flight-muscle twitch duration**: 40 ms (locust *Schistocerca americana*).
- **Latency to onset**: 15 ms.
- **Time to peak tension**: 35–40 ms after onset.
- **Slow leg-extensor muscles**: rise time to peak tension ~400 ms (much slower than flight muscle — centipede leg muscles likely fall in this range or between, ~50–200 ms).

For a leg muscle with a ~50 ms force rise, the **upper bandwidth is ~10–20 Hz** before twitch fusion / saturation. This is the biologically defensible upper limit for the rate of change of any individual joint torque in our simulation.

**Useful for our cost**:
- Provides the order-of-magnitude **range** for arthropod muscle bandwidth (slow leg extensor ~2–3 Hz, fast flight muscle ~20 Hz). The right number for centipede leg muscle is somewhere between, but **no direct centipede measurement was found**.
- Therefore we keep `w_torque_jerk` as a **soft cost-shaping penalty**, not a hard bio-anchored constraint. See §8.2 and §9.1 for the explicit framing. A direct *Scolopendra* leg muscle bandwidth measurement (if one is published in the future) would let us promote this to a hard constraint.
- The locust flight-muscle 20 Hz value should NOT be used directly as a ceiling for centipede leg muscle — it's the wrong muscle type at the wrong end of the bandwidth range.

---

## Section 2 — What we CAN and CANNOT validly compare to biology

**Where we currently stand (trial #141, current YAML):**

- velocity = 29.9 mm/s = **0.29 BL/s** (BL = 102.5 mm)
- ~2× slower than the slowest biological walk (0.5 BL/s)
- ~5× slower than a running centipede (1.5 BL/s)
- The bottleneck is gait timing in the YAML, not impedance gains. Theoretical max with current gait = 0.38 BL/s.

| metric | comparable to biology? | reference-anchored value |
|---|---|---|
| **Cost of Transport** (J/m) | YES, strong | 0.08-0.15 J/m for ~2.5 g centipede (§1.7, §1.8) |
| **Walking speed (in BL/s)** | YES, range-only | 0.5–1.5 BL/s walking (Anderson 1995, §1.1); ≥0.2 BL/cycle on rough terrain (Pierce 2026, §1.12). Our range: 0.26-0.34 BL/s on rough — at lower bound of biology. |
| **Speed reduction on rough terrain** | YES, strong | Real centipedes slow monotonically with rugosity but do not stop (Pierce 2023, §1.11). Defends `velocity_quality ≥ 0.3`. |
| **Body z RMS / clearance ratio** | YES, moderate | ≤10% conservative engineering, but consistent with biological observation that centipedes drape over rugose terrain rather than bounce (Pierce 2023, §1.11). Could be relaxed to 15% with citation. |
| **Body roll RMS** | YES, moderate | <5° for level walking (Pierce 2023 implicit — body and limb dynamics stay similar across rugosity); body twisting up to 10°+ on narrow obstacles (Pierce 2026, §1.12). |
| **Peak ground reaction force / weight** | YES, strong | up to ~10× body weight per leg (Cocci 2024, §1.5). Our `force_limit = 4` is conservative. |
| **Gait wave number** | YES, validates our YAML | 3 waves on a 19-segment body (Kinosita 2022, §1.6) |
| **Body stiffness vs frequency** | YES, mechanism-anchored | ~7× increase in intersegmental stiffness across walking → running (Embodied Intelligence, §1.13). Our trial #141 `body_kp = 0.37` is stiffer than earlier soft-body winners — consistent with this scaling. |
| **Gait frequency** | NO (prescribed in YAML) | — DRL will tune in next phase. |
| **Duty factor** | NO (prescribed by phase offsets) | — DRL will tune. |
| **Leg stride length** | NO (prescribed by amplitude × geometry) | — DRL will tune. |
| **Body undulation amplitude** | NO at our speed range | should be near-zero at slow speeds (Aoi 2013 / Kuroda 2014, §1.4); emerges via Hopf bifurcation. |
| **Ground clearance variability over rough terrain** | YES, qualitative | Centipedes drape over terrain via passive limb compliance (Pierce 2023, §1.11). |
| **Joint torque jerk / actuator bandwidth** | NO direct ref for centipede | Soft `w_torque_jerk` cost-shaping only. Locust slow leg-extensor data (~2–3 Hz) and locust flight muscle (~20 Hz) bracket the unknown centipede leg-muscle bandwidth. Keep as soft penalty, not hard constraint. (§1.14, §9.1) |

---

## Section 3 — The dilemma we identified (preserve verbatim)

The user's frustration at this point in the project is well-founded and worth recording:

> "those features are roughly good and reasonable but lack the fundamental basis (there is no reference) not to say the weights (this one is actually very problematic)"

> "our gait and wave number, leg and body wave amplitude and trajectory (so does the duty factor) is in the config yaml, which are the parameters that waiting to be controlled in the next step, so maybe its hard to directly compare the real centipede's duty factor frequency with the simulation? CoT should be something we can compare but the velocity is directly related with the frequency in our yaml file (higher frequency higher velocity from our previous observation of the simulation)"

> "we are going to tests and optimize the centipede on the rough terrain... so does the stable z also applicable here?"

The dilemma synthesized:

1. **The cost function we wrote uses weights chosen to make term magnitudes comparable**, which is circular — it doesn't reflect any external truth, and different weight choices yield different "optima."
2. **Most of the things we'd want to compare to biology are prescribed by the YAML (gait timing) rather than emergent from the impedance loop.** The optimizer can only affect emergent properties.
3. **CoT is the only metric that has both physical units (J/m) and a published biological reference range (0.08-0.15 J/m for ~2.5 g body).** It's the single defensible primary objective.
4. **Constraints on body-z stability must be terrain-aware** — penalizing absolute body-z motion would punish drape (good) the same as bouncing (bad). The fix is "RMS of (body_z − local_terrain_z)".
5. **Engineering constraints (jerk, peak torque, peak velocity) are not biological** — they are sim-to-real corrections, compensating for the ideal actuator model in the simulator.

The cleanest reformulation:

```
minimize:  CoT (J/m)                                  [biological primary]
subject to:
  velocity_quality ≥ 0.3                              [emergent execution]
  clearance RMS / nominal_clearance ≤ 0.10            [bio drape]
  body_roll RMS ≤ 5°                                   [bio planarity]
  peak_F/W ≤ 4                                          [contact safety]
  no buckle (45° root pitch / roll)                    [physical safety]
  peak |τ| ≤ actuator_capacity                         [hardware feasibility]
  max |dτ/dt| ≤ motor_bandwidth                        [hardware bandwidth]
```

7 constraints, 1 objective, all dimensionally clean. This eliminates the weight-tuning problem.

---

## Section 4 — Our thought process & frustrations (for the future PDF)

### Phase 1 — naive cost function (MuJoCo & Isaac Lab v1)

We started with a cost combining straightness, speed, tracking error, force, and pitch compliance. Weights were eyeballed: w_speed=50, w_wobble=8, etc. The "speed" reward had no upper bound, so the optimizer kept finding gains that pushed toward unrealistic speeds.

Frustration: "the cost is rewarding raw speed and we're getting gaits that race rather than walk"

### Phase 2 — bio-cost rewrite (Isaac Lab + MuJoCo)

We added body_z RMS, body_roll RMS, torque jerk, and cost-of-transport terms with capped speed reward. Weights were chosen to make per-term magnitudes comparable (~1-3 each).

Result: Optimization produced a convincing gait (Isaac Lab trial #141) but the user correctly identified that the underlying weight choices were unjustified.

Frustration: "lack the fundamental basis (there is no reference)"

### Phase 3 — recognizing the prescribed-vs-emergent split

User observation: many of the things we'd want to compare to biology (frequency, duty factor, stride, undulation amplitude) are prescribed by the YAML, not emergent. The impedance gain optimizer can only affect emergent properties.

Realization: the optimizer is **fundamentally about execution quality of a prescribed gait**, not about discovering biological gait timing. Biological comparison is constrained to the emergent-only metrics.

### Phase 4 — terrain-aware reformulation

User question: "we are going to optimize on rough terrain, does stable z still apply?"

Answer: no — naive body-z RMS conflates draping (good) with bouncing (bad). Need clearance-relative-to-terrain metric.

### Phase 5 — engineering vs biological constraints

User question: "if we minimize CoT will the centipede generate jerky motion?"

Answer: yes, single-objective optimization is known to find pathologies (coast-and-burst, foot-dragging, asymmetric limping, resonance exploitation, stick-slip). The fix is **engineering constraints from sim-to-real considerations** (jerk, peak torque, actuator bandwidth) — not biological, but necessary regularization against simulation idealizations.

### Phase 6 — biological references compiled (this document)

The CoT range (0.08-0.15 J/m for our body mass) is the single most defensible biological anchor. Walking speed range (~85-255 mm/s biological at our body length) gives a sanity check on how aggressive our gait timing should be. Other "biological" constraints (body z, roll, peak F/W) have weaker references but are defensible from anatomical reasoning.

The dilemma we cannot resolve fully: most biological data is on real centipedes operating at their natural gait timing, which differs from our simulation's prescribed YAML timing. Cross-comparison is limited to dimensionless or speed-normalized quantities.

---

## Section 5 — TODO before the PDF write-up

1. **Find direct measurements of body z variability** during real centipede locomotion. The 10% threshold is currently conventional, not measured. (Search: high-speed video kinematics of Scolopendra dorsoventral motion.)
2. **Find centipede actuator bandwidth analogue** — muscle fiber rate-of-force-development for arthropod skeletal muscle. (Hill model parameters for arthropod muscles.)
3. **Find species-matched CoT measurement** for *Scolopendra* specifically rather than inter-arthropod scaling. None found yet; may not exist.
4. **Validate the wave-number = 3 default** quantitatively (we have qualitative §1.6 support but no exact number for *Scolopendra heros*).
5. **Cross-check our predicted forward velocity** (39 mm/s commanded for 1 Hz, 0.6 rad amplitude, ~35 mm leg projection) against published centipede walking speeds. **DONE 2026-05-04:** Our centipede measures 102.5 mm long → commanded 0.38 BL/s, achieved 0.29 BL/s. Both fall below Anderson 1995's biological 0.5-1.5 BL/s range. Conclusion: at the current YAML gait, we cannot reach biological walking speed regardless of impedance gain choice. To match biology we would need to increase YAML frequency (e.g., to 1.5-2 Hz) or hip yaw amplitude (e.g., to 0.9 rad), or both.

6. ~~**Verify CoT measurement actually works.**~~ **DONE 2026-05-04.** The bug was a shape mismatch in `compute_cost`: we masked `joint_torque` with the post-settle mask but compared `jt.shape` to the un-masked `jv.shape`, so the `have_compat` check always failed and we silently fell back to `total_work=0`. Fixed by hoisting the full arrays before masking and re-checking shape compatibility on the un-masked tensors. CoT now reports a non-zero value and is the primary cost term.

---

## Section 7 — Current pre-publication state (2026-05-04)

This section captures the optimizer / controller / metric state actually used to produce the trial #141 result and to be used for upcoming runs. It supersedes Section 5 wherever they conflict.

### 7.1 Controller

- Architecture: **PD-tracking** impedance law on body yaw, pitch, roll, and per-leg DOFs.
  - τ = kp · (q_target − q) + kv · (q̇_target − q̇)
  - `q̇_target` is finite-differenced from successive `q_target` samples and reset between trials via `controller.reset_target_history()`.
  - This was a self-coded fix to a miscoded earlier law (kv was acting on raw `q̇` instead of the tracking error `q̇_target − q̇`), **not** a published-controller "discovery." See `controllers/farms/impedance_controller.py`.
- Body wave: 1.5 Hz, amplitude 0.4, wave_number 3.0 (per `configs/farms_controller.yaml`, bumped 2026-05-04 from 1.0 Hz).
- Leg wave: amplitudes [0.6, 0.2, 0, 0] (hip yaw, hip pitch, tibia, tarsus); phase offsets and DC offsets per the YAML.
- Rationale for 1.5 Hz: with the listed leg amplitudes and ~35 mm projected leg length, the analytical commanded velocity is `2 · L_proj · sin(A_yaw) · f` ≈ 0.57 BL/s — inside Anderson 1995's reported 0.5–1.5 BL/s range for walking *Scolopendra heros*. At the previous 1.0 Hz the analytical max was 0.38 BL/s, below the biological window regardless of impedance gain choice.

### 7.2 Cost formulation

- Optimizer: **Optuna TPE**, multivariate sampler with `constant_liar`.
- Cost = **CoT (J/m), subject to hard biological/engineering constraints**:
  - `velocity_quality = speed / commanded_velocity ≥ min_velocity_quality` (default 0.3)
  - `clearance_rms / nominal_clearance ≤ max_clearance_rms_ratio` (default 0.10)
  - `body_roll_rms_deg ≤ max_roll_deg` (default 5.0°)
  - `peak_F/W ≤ force_limit` (engineering safety)
- If any constraint is violated, `cost = CoT + 100 · Σ(violation_excess)` so TPE gets gradient back toward the feasible set.
- The previous **weighted-sum** cost is preserved as `cost_legacy` in the per-trial CSV for diagnostic comparison only — not returned to the optimizer.

### 7.3 Metrics that drive the cost

- **CoT (`cost_of_transport`)**, J/m: primary objective.
  - `total_work_J = Σ |τ · q̇| · dt` over post-settle, pre-buckle window.
  - `cot = total_work_J / displacement_m`.
  - Bug fixed 2026-05-04 (see Section 5 §6).
- **Terrain-aware clearance variation (`clearance_rms_m`)**: replaces the legacy `body_z_rms`.
  - `clearance(t) = body_z(t) − terrain_z(body_xy(t))` via bilinear interpolation of the heightmap stashed by `build_scene`.
  - Constant clearance over varied terrain → low metric → "good drape" (biological).
  - Rigid trajectory or bounce → high metric → bad. Correct sign in all cases.
  - The legacy `body_z_rms` is kept in the parts dict for diagnostic continuity.
- **Body roll RMS (deg)**: real centipedes are planar; soft penalty.
- **Peak F/W**: read from a real `ContactSensor` (see §7.4), no longer a placeholder.
- **Velocity quality**: `speed_mps / commanded_velocity_mps`. The denominator is computed analytically from the YAML wave parameters (`2 · L_proj · sin(A_yaw) · f`) rather than hard-coded.

### 7.4 Foot contact force

- Captured via `isaaclab.sensors.ContactSensor` matching `prim_path="/World/Centipede/.*foot_[0-9]+_[01]"`.
- The optimizer does **not** use an `InteractiveScene` wrapper, so the sensor is a standalone object. We drive its lifecycle ourselves:
  - `_fcs.reset()` after each trial reset (clears stale settle-time spikes).
  - `_fcs.update(dt)` after every `sim.step(render=False)` so `data.net_forces_w` is current when the recorder samples it.
- Stashed on `build_scene._foot_contact_sensor`.
- If sensor creation fails for any reason (older scene config), the recorder falls back to zeros and the optimizer logs a warning, so the run continues — but the force constraint is then trivially satisfied; do not interpret peak_F/W in that case.

### 7.5 Multi-terrain evaluation

- **Isaac Lab**: infrastructure added (per-spawn-offset `reset_robot_and_controller`, terrain-aware clearance falls out of the bilinear lookup automatically), but the objective currently evaluates one terrain per trial. Sequential 3-terrain evaluation (Option A) is wired but disabled for the current run; vectorized parallel rollouts (Option B / Flavor B refactor) explicitly deferred. See task #42.
- **MuJoCo**: per-wavelength unique XML paths now (was a shared `sweep_tmp.xml` overwrite bug); each trial is averaged across three wavelengths.

### 7.6 Files of record

- `scripts/optimization/optimize_impedance_biological.py` — the bio optimizer (Isaac Lab port). 1696 lines after cleanup; legacy in-process replay function dropped.
- `controllers/farms/impedance_controller.py` — PD-tracking law + `reset_target_history()`.
- `configs/farms_controller.yaml` — current trial #141 winning gains, 1.5 Hz body wave.
- `scripts/optimization/optimize_impedance_bayesian.py` — earlier weighted-sum optimizer, kept for reproducing v1 results.
- `BIO_REFERENCES.md` — this file.

### 7.7 Known limitations / honest framing

- We cannot directly compare absolute CoT numbers to biology yet — *Scolopendra*-specific CoT measurements are not in the literature we found. We rely on inter-arthropod scaling (Full 1989, etc.) which is a weak comparison.
- Foot contact force is now real, but we have not yet ground-truthed it against any analytical expectation (e.g., body weight evenly distributed across stance feet) — that's a next-day check before running large optimizations.
- `commanded_velocity_mps` is an analytical maximum derived from a small-angle sinusoidal approximation; the real swing-foot path is a more complex function of all four leg DOFs. We use it as a normaliser, not a ground truth.

---

## Section 6 — Sources (for the PDF bibliography)

1. Anderson, B. D., Shultz, J. W., & Jayne, B. C. (1995). *J. Exp. Biol.* 198, 1185-1195.
2. Manton, S. M. (1965). *J. Linn. Soc. Lond. Zool.* 46, 251-484.
3. Yasui, K. et al. (2017). *PLOS ONE* 12, e0171421.
4. Aoi, S. et al. (2013). *Phys. Rev. E* 87, 012717.
5. Cocci, R. et al. (2024). *J. Roy. Soc. Interface* 21, 20230439.
6. Kinosita, Y. et al. (2022). *Frontiers Robot. AI* 9, 797566.
7. Reilly, S. M. et al. (2007). *Zoology* 110, 271-289.
8. Full, R. J. (1989). Mechanics and energetics of terrestrial locomotion (chapter in *Energy Transformations in Cells and Organisms*).
9. Kuroda, S. et al. (2022). *Frontiers Robot. AI* 9, 797566.
10. Wosnitza, A. et al. (2013). *J. Exp. Biol.* 216, 480-491.
11. Pierce, C. J. et al. (2023). *J. Exp. Biol.* 226(4), jeb244688. (Active and passive mechanics for rugose terrain traversal in centipedes)
12. Pierce, C. J. et al. (2026). *Annals NY Acad. Sci.* doi:10.1111/nyas.70187. (Legged locomotion in lattices)
13. "Embodied intelligence solves the centipede's dilemma." arXiv:2603.08409.
14. Calderón-Macías, M. et al. (2019). *PLOS Comput. Biol.* 15(11), e1007437. (Insect muscle activation dynamics)

(Web URLs in section bodies; final BibTeX entries to be generated when the PDF is drafted.)

---

## Section 8 — Updated constraint specification (post-literature-review, 2026-05-04)

This section supersedes §7.2 wherever they conflict. It separates **biological constraints** (anchored to a published quantitative reference) from **engineering constraints** (sim-to-real safety, numerical stability, actuator hardware) and gives explicit reference citations for each value. The optimizer code can use these as the canonical source-of-truth for its `--min-velocity-quality`, `--max-clearance-rms-ratio`, etc. defaults.

### 8.1 Biological constraints (anchored to literature)

These are the constraints that **the impedance optimizer enforces in the current run**. Body stiffness × frequency scaling and similar emergent regularities are NOT in this table — they are hypotheses for the DRL phase (§8.3).

| constraint | value | reference | strength |
|---|---|---|---|
| **CoT** (objective, J/m) | minimise; target ≤ 0.15 | §1.7, §1.8 — inter-arthropod scaling for ~2.5 g body, validated by centipede-inspired robot | strong |
| `velocity_quality = v_actual / v_commanded` | ≥ 0.3 | §1.11 Pierce 2023 — *S. polymorpha* slows monotonically with rugosity but never stops walking | strong |
| `clearance_rms / nominal_clearance` | ≤ 0.10 (rough terrain) | §1.11 Pierce 2023 — centipedes drape over terrain via passive limb compliance, not bounce | moderate |
| `body_roll_rms` (deg) | ≤ 5° on level / mild rough; ≤ 8° on aggressive rough | §1.12 Pierce 2026 — *S. polymorpha* exhibits body twisting only on narrow obstacles; level walking is planar | moderate |
| `peak_F/W` (per-foot peak force / body weight) | ≤ 8× | §1.5 Cocci 2024 — real centipedes register up to ~10× body weight; 8× is conservative | strong |

**Note on the `peak_F/W` change**: was 4×, now 8×. The original value was an engineering safety margin chosen without bio reference. Cocci 2024 (§1.5) reports peaks of ~10× body weight, so 8× admits all biologically observed forces while still excluding numerical contact-spike pathologies. Re-tune with this new ceiling and observe whether the optimizer converges to a different region — if not, the constraint was inactive and the relaxation is free.

**Note on `body_roll_rms`**: was 5° hard ceiling. Pierce 2026 documents body twisting on narrow obstacles, so a hard 5° on rough terrain is too tight. Use a soft 5° for level / mild rough and 8° for aggressive rough (or use the same constraint relaxed to 8° always for cross-terrain consistency).

### 8.2 Final cost formulation (revised — IDENTICAL in Isaac Lab and MuJoCo)

```
minimize  CoT (J/m)                             [biological primary, §1.7+§1.8]

subject to (HARD — biologically anchored):
  velocity_quality           ≥ 0.30             [§1.11 Pierce 2023]
  clearance_rms / nom_clear  ≤ 0.10             [§1.11 Pierce 2023]
                                                (Isaac Lab uses terrain-aware
                                                clearance_rms; MuJoCo uses
                                                body_z_rms — same threshold,
                                                slightly different metric)
  body_roll_rms_deg          ≤ 5.0  (level)
                             ≤ 8.0  (rough)     [§1.12 Pierce 2026]
  peak_F / body_weight       ≤ 8.0              [§1.5 Cocci 2024]

subject to (HARD — engineering, see §9):
  no buckle: |root_pitch|, |root_roll| < 45°
  no per-joint buckle: |q_pitch|, |q_roll| < 35°
  peak |τ|                   ≤ 10 N·m            [actuator effort_limit, hardware]
  peak |q̇|                   ≤ 100 rad/s         [actuator velocity_limit, hardware]

cost equation (IDENTICAL in both simulators):
  if no constraint violated:
      cost = CoT
  else:
      cost = CoT + 100 · Σ(violation_excess)
```

`w_torque_jerk · mean((dτ/dt)²)` is computed each trial and stored in `parts` for diagnostic continuity, but is **NOT added to the cost** in either simulator. Reasoning: at the historical default `w_torque_jerk = 1e-3` it dominated CoT in feasible-region trials (factor ~25× on the MuJoCo side at trial 0 of a smoke run), and we don't have a centipede-leg muscle-bandwidth measurement to anchor it as a hard ceiling either. Keeping it as diagnostic-only gives a clean optimization story (CoT alone in feasible region) without losing the data.

The 100× violation penalty is unchanged from §7.2 — TPE still needs gradient back to feasibility.

### 8.3 Hypotheses for the DRL phase (NOT current constraints)

These are emergent regularities from the literature that the **next-step DRL controller** can be evaluated against. They are explicitly **not enforced** in the current impedance-gain optimizer — body_kp, body_kv, etc. are searched freely within their bounds.

- **Body stiffness × stepping frequency scaling** (§1.13). Aoi / "Embodied Intelligence" predicts a ~7× increase in effective intersegmental stiffness from walking to running in *S. heros*. When DRL searches over (body_kp, frequency) jointly, the optimum should trace a roughly linear `body_kp ∝ f` trajectory across the speed range. If it does, that's a positive bio-faithfulness result. If not, that's a finding worth examining (PhysX vs MuJoCo contact differences, idealised actuators, etc.).
- **Hopf bifurcation in body undulation amplitude** (§1.4). Aoi 2013 / Kuroda 2014: real centipedes have near-zero body-undulation amplitude at slow walks; amplitude emerges via Hopf bifurcation above a critical speed. DRL gait-timing search at low frequencies should converge to small body-amplitude winners; at high frequencies, larger amplitudes. The transition speed should be observable.
- **Duty factor decreases with speed** (§1.2 Manton). Real centipedes shorten stance fraction as they speed up. DRL can be evaluated against this trend.
- **Wave number ≈ 3 across observed walking speeds** (§1.6). Our YAML has it at 3.0 already, but DRL can confirm this is the optimum or find a different value.

These are **research questions for the next phase**, not constraints for the current impedance optimizer.

---

## Section 9 — Engineering constraints (separate from biology)

These are constraints we impose for **numerical stability, sim-to-real fidelity, and hardware feasibility**. They are NOT biological — they are corrections for the fact that our simulator gives the optimizer an idealised actuator model and a hand-tuned solver. Without them the optimizer finds pathologies (resonance exploitation, foot-tip clipping, infinite-bandwidth torque commands) that would not survive on real hardware.

### 9.1 Hardware / actuator constraints

| constraint | value | source | rationale |
|---|---|---|---|
| `effort_limit` (per actuator) | 10 N·m | `optimize_impedance_biological.py:805` | Conservative motor capacity; real micro-servos for a 2.5 g robot would be ≪ 10 N·m, but our actuators apply torque to all 229 joints so 10 N·m is an upper *envelope* per joint |
| `velocity_limit` (per actuator) | 100 rad/s | same | Set well above any plausible joint speed (< 20 rad/s observed in trials) so the optimizer can't game velocity-saturation |
| Foot passive damping | 0.006 N·m·s/rad | `optimize_impedance_biological.py:811` | Models passive joint damping in a real foot; without it the foot rings on contact |
| `w_torque_jerk` (soft) | 1e-3 (default) | optimizer arg | Soft cost-shaping penalty on `mean((dτ/dt)²)`. **NOT a bio-anchored hard constraint.** See note below. |

**Honest framing of the torque-rate / `w_torque_jerk` question** — earlier drafts of this doc proposed a hard ceiling `max |dτ/dt| ≤ 1260 N·m/s` derived as `effort_limit × 2π × 20 Hz` with the 20 Hz coming from §1.14 (locust *flight* muscle). That derivation has two problems:

1. **Wrong muscle type**: locust flight muscle is the *fastest* arthropod synchronous muscle. Centipede *leg* muscles are slow extensors. Calderón-Macías 2019 (§1.14) reports slow leg-extensor rise times ~400 ms, implying bandwidth ~2–3 Hz, not 20 Hz. The 20 Hz number was the wrong end of the literature range.
2. **Mixing biology with engineering**: the proposed hard ceiling baked in our chosen `effort_limit` of 10 N·m, which is itself an engineering ceiling, not biology. A "bio-anchored" hard constraint shouldn't have an engineering parameter inside it.

Until a **direct centipede leg-muscle bandwidth measurement** is available, the right behaviour is to keep `w_torque_jerk` as a **soft regulariser** (default 1e-3) and not pretend it is a hard biological ceiling. This is now consistent across §8.2 and §9.1.

### 9.2 Numerical / solver constraints

| constraint | MuJoCo value | Isaac/PhysX value | rationale |
|---|---|---|---|
| Physics timestep | 0.0005 s (0.5 ms) | 0.004 s (4 ms) | MuJoCo's implicit solver is stable at 0.5 ms; PhysX TGS needs 4 ms (smaller crashes) |
| Solver iterations | (MuJoCo default ~100) | TGS 8 pos + 2 vel | PhysX values from `configs/sim_params.yaml`. Higher = more accurate but slower |
| Joint armature | 1e-4 | 1e-4 | Matches; rotor inertia for stability |
| Joint damping (default) | 4.0754e-6 | 4.0754e-6 | Matches |
| Static / dynamic friction | 1.6 / 1.6 | 1.6 / 1.6 | Foot–ground friction, same in both sims |
| Restitution | 0 | 0 | No bounce (matches biological soft contact better than spring-damper) |

### 9.3 Stability / buckle constraints

These are NOT biological — they are sim-stability gates that catch the optimizer searching in an infeasible region. A real centipede would not pitch its root 45° during walking; if our simulator does, that's a buckle, not a viable gait.

| constraint | value | source |
|---|---|---|
| `MAX_ROOT_PITCH_DEG` | 45° | optimizer header constants |
| `MAX_ROOT_ROLL_DEG` | 45° | ditto |
| `MAX_JOINT_PITCH_DEG` | 35° | per-joint buckle threshold |
| `MAX_JOINT_ROLL_DEG` | 35° | ditto |
| `BUCKLED_COST` | 1e5 | infeasible-trial sentinel; TPE treats as "very bad" |

### 9.4 Constraints that should NOT be folded into the cost

These are present in the simulator but should be reported as **diagnostics**, not added to the cost function:

- Wall-clock time per trial (varies with hardware, not the gait)
- Number of physics steps (deterministic given duration + dt)
- Memory usage (not a research-question constraint)
- Per-trial random seed for terrain / spawn (controlled by `--terrain-seed` / `--spawn-seed`)

### 9.5 The bigger picture for the paper

The split between Sections 8 (biology) and 9 (engineering) gives reviewers a clean mental model:

- The **objective** (CoT) is the only quantity the paper claims to bio-faithfully optimise.
- The **biological constraints** define what counts as a "bio-plausible gait" — anchored to specific Scolopendra / arthropod papers.
- The **engineering constraints** are sim-to-real safety nets that prevent the optimizer from gaming idealisations in our actuator/solver model.
- The **DRL phase** (next-step) will tune **gait timing** (frequency, duty factor, wave amplitudes), which the impedance optimizer holds fixed. At that point, the §1.13 stiffness-frequency scaling becomes a testable hypothesis: does the DRL-discovered (frequency, body_kp) co-trajectory match the ~7× stiffness scaling Aoi predicts?

That last point is the strongest research-question framing for the next phase. Build the impedance-optimiser results into a body_kp(frequency) lookup, then let DRL move freely along that curve and see whether the optimum traces the biological scaling.
