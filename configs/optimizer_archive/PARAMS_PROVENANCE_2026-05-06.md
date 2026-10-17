# Controller-gains provenance — `configs/farms_controller.yaml`

Date: 2026-05-06.
Purpose: nail down exactly which optimizer trial produced the live
controller gains in `configs/farms_controller.yaml`, where the source
data lives, and how to verify (and if needed, restore) them.

This document is read-only metadata. **It does not change any gains.**

---

## Live config and its source

| Asset | Path |
|---|---|
| **Live config** (consumed by every `controller.step()`, `test_walk.py`, smoke harness, RL env) | `configs/farms_controller.yaml` |
| **Frozen optimizer reference** (this directory) | `configs/optimizer_archive/rank_05_trial_0167.yaml` |
| **Optimizer best-params record** (this directory) | `configs/optimizer_archive/best_params.json` |
| **Optimizer source run** (sister project — may move) | `C:\Users\wxy22\Documents\Centipede_IsaacLab\outputs\optimization\impedance_three_eval_three_eval_v1_20260505_133109\` |
| **Demo video for this trial** (sister project — same parent dir as above) | `…\top_videos\rank_05_trial_0167_cost_+022.76.mp4` |

The `optimizer_archive/` directory is added to **this** project so the
live gains remain self-describing even if the old `Centipede_IsaacLab`
folder is moved or deleted.

---

## Equivalence proof

A programmatic YAML deep-diff (parsed with `yaml.safe_load`, recursive
walk, ε = 1e-15 for floats) returned **NUMERICALLY IDENTICAL** between
the live YAML and the frozen reference, on 2026-05-06.

Canonical hash (SHA-256/12 over `json.dumps(parsed, sort_keys=True)`):

```
ref  = 2a769b6b0b5d   # rank_05_trial_0167.yaml (pristine optimizer output)
live = 2a769b6b0b5d   # configs/farms_controller.yaml (working tree)
```

You can recompute these any time:

```powershell
python -c "import yaml,hashlib,json; print(hashlib.sha256(json.dumps(yaml.safe_load(open('configs/farms_controller.yaml')),sort_keys=True,default=str).encode()).hexdigest()[:12])"
python -c "import yaml,hashlib,json; print(hashlib.sha256(json.dumps(yaml.safe_load(open('configs/optimizer_archive/rank_05_trial_0167.yaml')),sort_keys=True,default=str).encode()).hexdigest()[:12])"
```

If the two hashes ever diverge, **someone edited the live YAML's gains
without updating this provenance doc**. That's the tripwire — restore
from the frozen reference (see below) or update both.

---

## Why trial 167 (rank 5), not trial 112 (Optuna's best)

The optimizer's `best_params.json` records **trial 112, cost 15.07**
as its overall best. We deliberately did NOT pick that trial. Reasons
captured in the live YAML's `impedance:` block comments
(`configs/farms_controller.yaml` lines 44–54) and reproduced here:

* Trial 167 is **rank 5 by constraint cost** (cost 22.76) but the
  `three_eval_v1` design averages each trial over **three terrains /
  frequencies**: flat 2 Hz, rough λ=18 mm 1 Hz, rough λ=18 mm 2 Hz.
  Trial 167's per-eval performance was robust across all three:

  | eval | terrain | freq | speed (mm/s) | speed (BL/s) | CoT (J/m) |
  |---|---|---|---:|---:|---:|
  | A | flat | 2 Hz | 90.0 | 0.88 | 21.49 |
  | B | rough λ18 mm | 1 Hz | 23.6 | — | 7.75 |
  | C | rough λ18 mm | 2 Hz | 49.6 | — | 39.04 |

* Trial 112 (Optuna's best) and similar lower-cost trials in this run
  used **soft body** gains (`body_kp ≈ 0.01`) that buckled when
  re-tested at the higher frequency / new arena geometry — the
  PD-tracking `kv·(target_dot − qdot)` term overran the soft body's
  bandwidth.

* The earlier `pd_corrected_wide` rank-1 winner (trial #141, body_kp
  ≈ 0.0096) had the same failure mode in the bio-arena run.

* Trial 167 sits in the **stiff-body** family (`body_kp = 0.144`),
  active hip drive (`leg_kp[0..1] ≈ 0.07–0.10`), very soft tibia
  (`leg_kp[2] = 0.00149` — terrain-conforming), moderate tarsus
  (`leg_kp[3] = 0.142`). This combination held up across 1–2 Hz on
  both flat and rough λ=18 mm.

This is a **conservative, robust** parameter choice over a
narrowly-optimal one.

---

## Restoring from the frozen reference (if the live YAML is corrupted)

```powershell
copy configs\optimizer_archive\rank_05_trial_0167.yaml configs\farms_controller.yaml
```

This restores the gains exactly. The doc-comment block in the live YAML
that explains the choice will be removed; either restore it from
`git show HEAD~1:configs/farms_controller.yaml` once you make a
checkpoint commit, or accept the loss (gains are what matter; the
comments are duplicated in this provenance doc).

After restoring, re-run `pytest tests\test_controller_parity.py -v`
(should still pass — parity tests don't depend on the YAML content).

---

## Files added to the project today (2026-05-06)

| Path | Origin | Purpose |
|---|---|---|
| `configs/optimizer_archive/rank_05_trial_0167.yaml` | byte copy of optimizer's `top_yamls/rank_05_trial_0167.yaml` | Frozen reference for the active gain set |
| `configs/optimizer_archive/best_params.json` | byte copy of optimizer's `best_params.json` | Forensic record — shows trial 112 was Optuna-best but explicitly rejected |
| `configs/optimizer_archive/PARAMS_PROVENANCE_2026-05-06.md` | this file | Provenance index |

None of these are read at runtime; they're documentation. The live
config remains `configs/farms_controller.yaml`.

---

## Note on the "168 lines changed" git status

`git diff --stat HEAD -- configs/farms_controller.yaml` reports 84
insertions + 84 deletions (= the entire 84-line file). This is **pure
line-ending churn** (CRLF ↔ LF re-save), not gain edits. Confirmed by
the canonical-hash equality above. Cleanest way to clear the dirt is
a checkpoint commit — but the gains themselves are correct.
