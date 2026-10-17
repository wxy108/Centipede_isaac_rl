# OUTPUT ORGANIZATION POLICY — standing rule for ALL future outputs
_Owner: senior simulation engineering practice. Established 2026-06-17. Applies to the worker and the auditor. Non-optional._

Every artifact a run produces (data, logs, figures, videos, checkpoints, metrics) MUST land in a designated, dated,
self-describing run directory — never loose at a folder root, never in an ad-hoc location. This keeps the repo navigable
and prevents the history-pollution that motivated the 2026-06-17 cleanup.

## 1. Canonical top-level taxonomy (group same-class results here; create a new sibling only for a genuinely new class)
```
outputs/
  directives/          # directives + ⛳ gate receipts (live) ; outputs/directives/_archive/ = superseded/lineage
  optimization/        # controller / parameter optimization studies
  analysis/            # post-hoc analysis runs (metrics, ablations, sweeps)
  rl/                  # RL training runs (checkpoints, summaries)
  visual_audit/        # rendered videos + per-frame audit JSON
  figures/             # cross-run / publication figures promoted from a run (also copy to docs/figures/ for the paper)
  _archive_pre_reset/  # frozen lineage — do NOT treat as live, do NOT edit
```
A new result class (e.g. `controllers_frozen/` for accepted Stage-B param sets) gets its OWN top-level sibling under
`outputs/`, documented here, not dumped into an existing one.

## 2. Run directory — the unit of organization
Each experiment = ONE directory: `outputs/<class>/<topic>_<YYYY-MM-DD>[_vN]/` with the standard skeleton:
```
<topic>_<YYYY-MM-DD>_vN/
  MANIFEST.md     # REQUIRED — see §4
  data/           # CSVs, npz, study.db, raw metrics
  logs/           # launch + run logs (the as-run log governs, never cfg defaults)
  figures/        # plots from THIS run
  videos/         # clips from THIS run, each with its per-frame audit JSON sidecar
  meta/           # configs as-run, controller hashes, snapshots
```
No loose files at `outputs/` root or at a class root. Temp scratch → `<run>/tmp/` and delete it before the gate.

## 3. Naming (strict)
- lowercase_snake_case; date `YYYY-MM-DD`; version suffix `_vN` (never overwrite a prior version).
- Consistent class prefix (e.g. `stage_a_`, `sweep_`, `ablation_`). No spaces, no path-as-filename, no `0-` style prefixes.
- Figures/videos: `<metric>_<condition>.<ext>` (e.g. `wave_vs_severity_lambda60.png`), x-axis conventions per project rules
  (roughness/severity ascending left→right).

## 4. MANIFEST.md (required in every run dir) — track or it didn't happen
One short file per run: (a) what it is + which directive/⛳gate it serves; (b) headline result (1–2 lines);
(c) date + git/controller hash; (d) exact reproduce command (script:line + full CLI); (e) per-number CSV citations;
(f) status: ACTIVE / SUPERSEDED-by-<dir> / ARCHIVED. On completion, append a one-line entry to `outputs/RUN_INDEX.md`.

## 5. Supersession & archival (never delete data; mirror the cleanup discipline)
- Superseding a run = new dated `_vN` dir + add `SUPERSEDED → <new dir>` to the old MANIFEST; move clearly-dead runs to
  `outputs/_archive_pre_reset/` (or a dated `_archive/`), logged in a FILE_MAP. Reversible moves only; deletion limited to
  empty (0-byte) files / empty dirs, and never package markers (`__init__.py`).
- Protected dirs (`envs/ controllers/ assets/ tasks/ scripts/rl/ IsaacLab/`) are never reorganized.

## 6. Figures & videos — the two the PI calls out
- Figures → the run's `figures/`; promote paper-ready ones to `outputs/figures/` AND `docs/figures/`. Vector (.svg) +
  raster (.png) + a transparent variant when overlay is likely.
- Videos → the run's `videos/`, EACH clip with its per-frame audit JSON (luminance, black-frac=0, motion>0, on-block);
  no clip is "done" without a passing audit (see `docs/RENDERING_PIPELINE_DEBUG_NOTES.md`).
```
```

## 7. Audit trail & 留痕 — record-keeping for autonomous runs (added 2026-06-17, PI-directed)
For any self-paced / overnight autonomous run, the run dir + MANIFEST are NOT enough — maintain a single append-only ledger and audit-grade data so the auditor can independently verify and the PI can resume from any step.
- **`outputs/AUTORUN_LEDGER.md`** — ONE append-only ledger (never rewrite/trim). One STEP entry per atomic action (build/convert/smoke/opt-gen/eval), START + END, with: goal, inputs+sha256, exact command (script:line + CLI), run_dir, result (headline numbers + CSV path:line + V2 disjoint recompute), outputs+sha256, snapshots taken, decision/branch, **resume_hint** (what to revert + re-run if this step is later found wrong), status. Supersede by a new entry naming the old STEP id — never edit a past entry.
- **Every change is logged + snapshotted — including changes a TOOL made on your behalf.** After any command that can touch a tracked file (esp. converters writing sidecars, e.g. an `MjcfConverter` config.yaml), run `git status` and log+snapshot any diff in the SAME step. An unlogged change breaks "resume from any step" and is a breach, not a slip. (Lesson: 2026-06-17 a converter sidecar silently flipped `assets/config.yaml`; caught only by auditor git-diff.)
- **Audit-grade data (auditor-independently-verifiable):** every reported number must be CPU-recomputable from raw arrays without GPU/IsaacLab — see `AUDIT_DATA_FORMAT_SPEC_2026-06-17.md` (raw `.npz/.parquet/.csv` + `schema.json` + `meta.json` (as-run CLI/seed/LUT/conventions/window) + `metrics.json` (value+raw_source+formula+V2) + standalone `recompute.py`). Numbers not CPU-recomputable = UNVERIFIED.
- **Protected edits:** `*.before_<tag>_<date>` snapshot + default-OFF flag + flag-OFF bit-identity (controller hash END==START) — and the snapshot + a ledger STEP, no exceptions.
- **Archive / supersede notes (留痕 for prior work):** never delete data. Move dead/superseded runs to a dated `_archive/` (or `_archive_pre_reset/`), add `SUPERSEDED → <new dir>` to the old MANIFEST, and record the move in a `FILE_MAP_<date>.md`. The archive must always say WHAT was archived, WHEN, WHY, and WHERE it went, so lineage is reconstructable.
