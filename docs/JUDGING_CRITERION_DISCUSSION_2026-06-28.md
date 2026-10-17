# The Judging-Criterion Problem — discussion brief for multi-agent review
*Centipede IsaacLab RL · 2026-06-28 · for cross-agent discussion (and human PI)*

## TL;DR (the question on the table)

We have spent ~6 search rounds (v6→v16) tuning a simulated centipede's **joint impedance** (per-group PD
stiffness/damping kp, kv) and ranking the results by a metric we call **"fling"** (peak body-yaw angular
rate, deg/s — a proxy for "spins out / unstable"). The PI's verdict on the whole effort:

> **"我们好像缺少判据啊"** — *we seem to be lacking a judging criterion.*

The discomfort is real and specific: **"fling" has no biological or external reference**, so "which design
is best" has no defensible answer — different proxies give different winners with nothing to validate against.
This brief summarizes the system, how the criterion drifted, the current (unsatisfying) criteria, the
difficulties, and the open question: **how do we decide which criterion to actually use?**

---

## 1. The system

- **Model:** 21-segment centipede, body length 102.5 mm, mass 2.54 g (≈ half-size adult *Scolopendra heros*),
  211 actuated joints / 251 bodies. Body-yaw undulation + per-leg (yaw/pitch/roll) joints.
- **Controller:** PD impedance law `τ = kp·(q_ref − q) + kv·(q̇_ref − q̇)` driven by a CPG body wave.
  The **gait timing is PRESCRIBED in a YAML and locked**: frequency (1.0–1.5 Hz), body-wave amplitude (0.4 rad),
  `wave_number = 3.0` (≈3 full waves on the body), leg amplitudes/phases. The search only varies the
  **impedance gains** (kp/kv per group) — i.e. the *execution quality of a fixed gait*, not the gait itself.
- **Task regime of interest:** rough terrain at LOW wave-number (the "fling-prone" regime), homogeneous
  single-wavelength heightmaps (λ30–60 mm, amp 8–16 mm).

## 2. How the criterion drifted (the history)

1. **Eyeballed weighted-sum cost (early).** straightness + speed + tracking + force, weights chosen "by eye."
   The PI flagged this as *"very problematic — lacks the fundamental basis (there is no reference), and
   different weight choices yield different optima with no way to validate which is correct."*
2. **Bio-anchored reformulation (`BIO_REFERENCES.md`, 2026-05-04).** Resolved the above by adopting a single
   dimensioned, literature-anchored **objective = mechanical Cost of Transport (CoT, J/m)**, target band
   **0.08–0.15 J/m** for a 2.5 g body, subject to bio-anchored constraints (velocity_quality ≥ 0.3; drape
   clearance; body_roll ≤ 5–8°; peak foot-force ≤ 8× body weight). Each value cited to a specific
   *Scolopendra*/arthropod paper (Anderson 1995, Pierce 2023/2026, Cocci 2024, Aoi 2013, Manton 1965, …).
3. **The recent campaign (v6→v16) quietly drifted OFF that anchor.** It ranks by **"fling"** (peak body-yaw
   rate) + **"wave visibility"** (body-wave amplitude is non-collapsed) + hard gates (walks, doesn't tip,
   forward not reversed). **CoT is not used in any recent round.** This is the source of the "no criterion"
   feeling: fling and wave-visibility are internal proxies with no external reference.

## 3. The current criteria actually in use — and why each is unsatisfying

| current criterion | what it is | why it's shaky |
|---|---|---|
| **fling** (primary rank) | peak \|dψ/dt\| of body yaw (body-PCA), low = "quiet/stable" | **No biological reference / no threshold.** Worse: Pierce 2023 shows real centipedes undergo 53–60 *passive limb-bending / transient perturbation* events per trial on rough terrain and just keep walking — so penalizing yaw-rate spikes may be penalizing the *biologically correct* passive compliance. |
| **wave visibility** | body-wave amplitude not collapsed (bar ≥ ~0.7 of commanded) | **Contradicts biology.** Aoi 2013 / Kuroda 2014: real body undulation is near-zero at slow walk and *emerges via a Hopf bifurcation* with speed. Requiring a visible wave at low speed may be unbiological. |
| **walk / no-tip / forward gates** | hard feasibility gates | These are sound as *safety gates*, but they don't *rank* — they only admit/reject. |

**v16 verdict (the result that triggered the PI's question):** searching impedance for
"walk + quiet(low-fling) + visible-wave" simultaneously → **ZERO winners**. Wave-visibility and low-wave
quietness are *antagonistic through body stiffness*. Conclusion: the lever is **wave kinematics (wave number)**,
**not** joint impedance. (i.e. the thing that would actually change the answer is locked in the YAML.)

## 4. The difficulties (why this is genuinely hard)

1. **No ground truth.** No real centipede labeled any of our gaits "good." Every criterion is therefore a
   *bet* of one of three kinds (see §5).
2. **Prescribed-vs-emergent split.** The biology that most defines "a centipede" — gait frequency, duty factor,
   stride, undulation amplitude, the Hopf onset, the stiffness∝frequency scaling — lives in the **locked YAML
   gait-timing space**. The impedance search can only touch *emergent execution quality*. **We have been
   searching the subspace biology barely constrains, while holding fixed the subspace it strongly constrains.**
3. **Multi-objective reality.** Real animals are an evolutionary Pareto compromise (energy, speed, stability,
   robustness, injury, sensing). Collapsing to one scalar discards the tradeoff that *is* the science.
4. **Proxy with no anchor.** "fling" can't be defended in a methods section; its threshold is arbitrary.
5. **Degeneracy risks (observed before).** Earlier rounds found reward/metric degeneracies (e.g. bit-identical
   metrics across physically-different configs; rank-high-but-magnitude-trivial conditioning). A criterion that
   doesn't *discriminate* the candidates, or whose differences don't correlate with anything checkable, is useless.

## 5. The key reframing: criteria come in THREE different TYPES

"Which criterion do we use" is really "which *kind of claim* do we want to make":

- **TYPE A — Optimality** ("what is the centipede *for*?"): pick what evolution optimizes — CoT, speed,
  stability margin, robustness. Risk: the true fitness function is unknown and multi-objective; picking one is a bet.
- **TYPE B — Resemblance** ("does it *move like* a centipede?"): don't ask what it optimizes; measure whether the
  resulting behavior falls inside the *measured* real-centipede manifold (wave number, amplitude-vs-speed curve,
  duty factor, body-roll range, force-wave timing, passive-bend event rate). Criterion = *distance to the
  biological measurement cloud*. Arguably more defensible — needs the animal measured, not its objective known.
- **TYPE C — Mechanism** ("does it *work the way* a centipede works?"): match the generative principle, not the
  output — gait should *emerge* from leg-ground coupling + body compliance (Yasui/Kuroda, decentralized); body
  undulation should arise via Hopf (Aoi); stiffness should self-tune to frequency (~7×, Aoi/"Embodied
  Intelligence"). Tested by whether the right phenomena emerge under perturbation. Deepest, hardest.

**Diagnosis:** the current campaign mixes *half-baked* versions: `fling` = an unreferenced Type-A; `wave
visibility` = a Type-B that contradicts the actual biology. Neither stands.

## 6. The menu of candidate criteria (divergent — not just CoT)

1. **Cost of Transport** (Type A) — energy/distance; units + reference (0.08–0.15 J/m). But frequency-dominated → weak discriminator for kp/kv while frequency is locked.
2. **Kinematic resemblance** (Type B) — Mahalanobis distance to measured real-centipede kinematics (amplitude-vs-speed, wave number, duty factor, roll).
3. **Terrain-generalist robustness** — ONE parameter set across the whole terrain×perturbation distribution; pick the one that degrades most gracefully (AUC of speed-vs-rugosity). Most "one body, walks everywhere" — and we've barely used it (we tune per terrain).
4. **Passive stability / preflex** — at FIXED open-loop drive, largest perturbation absorbed before tipping + recovery speed. Reframes "fling" correctly: not *minimize* spikes but *recover from* them.
5. **Force-wave signature match** (Cocci 2024) — reproduce the head-to-tail traction-force wave, per-leg 0–10 mN, forward-on-contact/backward-on-pull. Very concrete "is it really walking like one."
6. **Activation economy** (Yasui 2017) — "stop legs not in stance" saves orders of magnitude energy; reward minimal non-supporting-leg work.
7. **Emergence tests** (Type C, PASS/FAIL) — does undulation appear via Hopf as speed rises? does body_kp∝f emerge if frequency varies?
8. **Expert visual judgment** (Type B, informal) — "watch it and it's obviously a centipede." Implicitly what the PI has been using via the videos. Not rigorous, but not nothing.

## 7. The actual hard question: HOW do we decide which to use?

Adjudication strategies on the table:

- **(a) By the sentence we want to write.** "biologically efficient" → CoT; "moves like a real centipede" →
  resemblance; "uses the centipede control principle" → emergence. Decide the claim, the criterion follows.
- **(b) By discriminating power.** A good criterion must *separate* our candidates AND those differences must
  correlate with something independently checkable. Test each candidate criterion empirically on the designs we have.
- **(c) By triangulation (multi-criterion agreement).** Don't pre-commit to one scalar. Compute several cheap
  criteria on the same candidates; commit where they *agree*; treat *disagreement* as the finding (it reveals
  the real Pareto tradeoff). Strongest against "which single metric do you prefer" reviewer pushback.
- **(d) By what is actually free to vary.** Don't use a criterion dominated by a locked parameter. With
  frequency locked, prefer criteria sensitive to what we *do* search (compliance → preflex, drape, force distribution).
- **(e) By anchoring to one external dataset.** The Pierce/Goldman rough-terrain *Scolopendra* data (speed-vs-
  rugosity curves, passive-bend rates, ~0.2 BL/cycle on lattices) is the most regime-matched external ground
  truth; criterion = "reproduce that curve." External, falsifiable, on-topic.

## 8. Open questions for the discussion

1. Which **TYPE** (A optimize / B resemble / C mechanism) should the criterion be — i.e. what claim do we want?
2. Should we commit to a **single scalar** (to pick parameters) or **triangulate multiple criteria** (to find the tradeoff)?
3. The **locked-frequency** knot: can a defensible "real-centipede parameters" criterion even exist while gait
   timing is fixed? Or must frequency/gait-timing enter the search first (→ the DRL phase)?
4. Is **"fling" salvageable** — reframed from an *objective* into a *preflex/recovery* criterion or a safety gate —
   or should it be dropped?
5. Is **terrain-generalist robustness** (one body, graceful degradation across all terrains) the most
   "centipede" criterion we're under-using?
6. Empirical shortcut: should we first just **compute several criteria on the existing candidates and see if
   they conflict**, letting the data tell us which criterion has discriminating power before we commit?

## 9. Panel synthesis — 3-agent discussion (2026-06-28)

Three agents (comparative-biomechanics lens, optimization/control lens, skeptical methods-reviewer lens)
read this brief independently. They **converged** to a near-identical position:

**Consensus (all 3 agreed):**
1. **Drop CoT as the PRIMARY criterion here.** Best-referenced *number* but the *weakest discriminator*: it is
   frequency-dominated and frequency is locked, so it barely moves across (kp,kv). Skeptic adds it is *more*
   dangerous than fling because "defensible units (J/m) ≠ defensible claim" — no *Scolopendra*-specific CoT exists
   (it's inter-arthropod mass-scaling), and idealized PD actuators ≠ metabolic cost. Keep CoT as a *constraint/
   diagnostic* (∈ bio band), not the objective.
2. **"fling" — drop as an objective, salvage as preflex/recovery.** Minimizing peak body-yaw is wrong-signed:
   Pierce 2023 counts 53–60 passive-limb-bending events/trial as NORMAL ("bend, then resume stepping"). Reframe to
   a two-sided **recovery** metric (perturbation impulse absorbed before tip + return-to-limit-cycle settling time);
   keep peak-yaw only as a tip-over **safety gate**, never a rank.
3. **Delete the "visible-wave" requirement.** Aoi 2013 Hopf bifurcation: body undulation is ~0 at slow walk. At
   our 0.26–0.34 BL/s the d0195 collapsed-wave gait is **biologically correct**, not a defect. Judge d0195 on its
   tip/speed, not its wave amplitude.
4. **The criterion should be TERRAIN-GENERALIST ROBUSTNESS** = ONE impedance set across the whole terrain
   distribution, scored as **AUC of forward-speed-vs-rugosity**, validated against the external **Pierce/Goldman**
   curve (monotonic slow-but-never-stop, ~0.2 BL/cycle on lattices) + kinematic *invariance* as terrain roughens.
   This is the criterion that is (a) free to vary under locked frequency, (b) externally anchored in the regime we
   actually run, (c) falsifiable. (Type B resemblance + adjudication-(e) external anchor.) We've under-used it
   because we tune per-terrain.
5. **The locked frequency is the deeper problem.** v16's "zero winners / lever = wave-number" is the *structural
   inevitability* of searching the execution-quality subspace while the defining gait variables are frozen — not a
   tuning failure. Optimality (A) and emergence (C) claims must wait for **frequency/gait-timing to enter the search
   (the DRL phase)**. Only a *scoped robustness* claim is honest now.

**Method (how to decide, before committing):**
- **Identifiability screen** (optimization): on the existing v6–v16 archive, measure each candidate criterion's
  sensitivity to (kp,kv) vs to a small deliberate frequency perturbation → reveals which criteria the impedance
  search can actually move. Pick a criterion that lives in the channel impedance controls.
- **Discriminating-power test:** between-design variance must exceed within-design (seed/spawn) variance — F-ratio
  / ICC / η², NOT rank correlation alone (enforces the project's own "rank ≠ magnitude" rule); + 2nd disjoint
  recompute + held-out seed/terrain ranking stability; flag bit-identical-across-configs as a pipeline bug.
- **Triangulation is rigor ONLY if PRE-REGISTERED** (skeptic): as loosely proposed ("commit where they agree,
  disagreement = finding") it has *no failure mode* = guaranteed victory = no test. Must pre-declare each
  criterion's pass threshold + direction, verify the criteria respond to *different physical channels* (else
  frequency-dominated metrics agree spuriously), and define what a *contradiction* forces us to retract.

**Cross-lens blind-spot warnings:**
- Biologist → ML: do NOT treat yaw-spikes / transient bends / low-speed wave-collapse as noise to minimize — that
  selects a brittle stiff body that plots cleanly and tips on the one terrain you didn't test. *The transients are
  the mechanism (passive compliance does the terrain negotiation).*
- Optimization → biologist: "biologically well-referenced ≠ usable selection criterion here" — a criterion also
  needs *identifiability within the searched params* and *discriminating power above noise*.
- Skeptic → all: the criterion problem is *downstream of a search-space problem*; any criterion now only
  discriminates execution noise within a hand-authored gait. State that minimal claim honestly.

**The minimal honest claim the project can stand behind (skeptic, endorsed by all):**
> *"At a fixed prescribed gait, impedance gains G traverse rough terrain with the most graceful speed-vs-rugosity
> degradation, reproducing the monotonic-slow-but-never-stop and ~0.2 BL/cycle behavior of Pierce/Goldman
> Scolopendra — and the full biological-parameter question cannot be settled until gait frequency/timing enters
> the search."*

---

## Appendix — key biological anchors (from `BIO_REFERENCES.md`)

- CoT target 0.08–0.15 J/m (~2.5 g body; inter-arthropod scaling, Full 1989 / Reilly 2007; centipede-robot confirms).
- Walking speed 0.5–1.5 BL/s (Anderson 1995); ours ~0.26–0.34 BL/s on rough = low end.
- Rough terrain: slows monotonically but never stops; passive limb bending is normal (Pierce 2023). ~0.2 BL/cycle on lattices (Pierce 2026).
- Body undulation amplitude is speed-dependent, ~0 at slow walk, emerges via Hopf bifurcation (Aoi 2013 / Kuroda 2014).
- Body intersegmental stiffness ↑ ~7× from walk→run, tuned to stepping frequency (Aoi / "Embodied Intelligence" arXiv:2603.08409). → body_kp should track frequency, not be chosen in isolation.
- wave_number ≈ 3 validated (Kinosita 2022: 5–6 legs/wave on 19 segments).
- Peak foot force ~10× body weight (Cocci 2024); head-to-tail force wave.
- **The prescribed-vs-emergent split:** frequency, duty factor, stride, undulation amplitude are YAML-prescribed,
  NOT emergent from the impedance loop — so the impedance optimizer is fundamentally about *execution quality of a
  prescribed gait*, and biological comparison is limited to emergent/dimensionless quantities.
