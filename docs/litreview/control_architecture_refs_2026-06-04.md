# Centralized ↔ Decentralized Control — Core References
**Centipede IsaacLab RL · compiled 2026-06-04 (trimmed to essentials)**

Complements `BIO_REFERENCES.md` (biomechanics). This is the **control-architecture**
axis only, mapped onto our actual controller. Six papers, each a distinct role.

---

## §0 — Where OUR controller sits (read from code)

From `controllers/impedance_controller.py` + `_torch.py`:

| Subsystem | Code | Class |
|---|---|---|
| Body yaw wave | Kuramoto **chain**, nearest-neighbour coupling (`_integrate_cpg_phases`, l.317-348) | **Decentralized** intersegmental oscillator chain |
| Head | yaw **servo** (`head_yaw_gain`, l.377-389), currently OFF | **Centralized** head ref (latent) |
| Legs | phase-**slaved** to body segment via `kuramoto_leg_step` | **Hierarchical body→leg** (one shared wave number) |
| CPG feedback | none — feedforward phase only | **Open-loop CPG** (NOT force-feedback-decentralized) |

We are a **feedforward decentralized chain + latent central head + body→leg slaving**,
*without* the local force feedback that defines the Ishiguro/Owaki school. `wave_number_scale`
sets the single shared spatial wave number — the knob the bio-prior is about.

---

## The six

**1 — Framework / the lens.**
Holmes, P., Full, R.J., Koditschek, D.E., Guckenheimer, J. (2006). "The Dynamics of Legged
Locomotion: Models, Analyses, and Challenges." *SIAM Review* 48(2):207-304.
DOI: 10.1137/S0036144504445133.
→ **Templates vs anchors.** Our analytical k*(λ,A) = template; the 287-joint sim = anchor.
  The whole "missing basis" argument is a templates-vs-anchors gap.

**2 — Our architectural precedent.**
Ijspeert, A.J., Crespi, A., Ryczko, D., Cabelguen, J.-M. (2007). "From Swimming to Walking
with a Salamander Robot Driven by a Spinal Cord Model." *Science* 315:1416-1420.
DOI: 10.1126/science.1138353.
→ A body **oscillator chain drives limb oscillators**; the body wave entrains the legs.
  Structurally almost identical to our `kuramoto_leg_step`. Best single precedent we have.

**3 — The axis itself.**
Aoi, S., Manoonpong, P., Ambe, Y., Matsuno, F., Wörgötter, F. (2017). "Adaptive Control
Strategies for Interlimb Coordination in Legged Robots: A Review." *Front. Neurorobotics*
11:39. DOI: 10.3389/fnbot.2017.00039.
→ The review that surveys **centralized vs decentralized** interlimb coordination. Backbone
  citation for positioning ourselves on the spectrum.

**4 — Decentralized pole (the contrast).**
Owaki, D., Kano, T., Nagasawa, K., Tero, A., Ishiguro, A. (2013). "Simple robot suggests
physical interlimb communication is essential for quadruped walking." *J. R. Soc. Interface*
10(78):20120669. DOI: 10.1098/rsif.2012.0669.
→ Coordination from **local ground-reaction-force feedback**, decoupled oscillators. The
  canonical "physics, not central wiring" result — the opposite design choice to ours.

**5 — Redundancy pole (rough-terrain tension).**
Chong, B., He, J., Soto, D., Wang, T., Irvine, D., Blekherman, G., Goldman, D.I. (2023).
"Multilegged matter transport: A framework for locomotion on noisy landscapes." *Science*
380:509-515. DOI: 10.1126/science.ade4985.
→ On noisy terrain, **spatial redundancy substitutes for sensing/control**; open-loop works.
  Frames our core tension: interpretable open-loop wave control vs sensed/closed-loop RL.

**6 — Our dependent variable, in centipedes.**
Ambe, Y., Aoi, S., et al. (2021). "Generation of Direct-, Retrograde-, and Source-Wave Gaits
in Multi-Legged Locomotion in a Decentralized Manner via Embodied Sensorimotor Interaction."
*Front. Neural Circuits* 15:706064. DOI: 10.3389/fncir.2021.706064.
→ The body **wave number/direction** itself, generated decentrally. Closest myriapod paper to
  our control variable.

---

Positioning in one line: our open question — *does rougher terrain raise the useful wave
number?* — sits between Goldman's "redundancy replaces sensing" (open-loop suffices) and
Ishiguro/Owaki's "feedback drives adaptation" (closed-loop needed), and asks whether the
adaptation can be an interpretable function of a single wave-number parameter.
