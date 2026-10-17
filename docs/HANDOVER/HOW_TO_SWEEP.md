# HOW TO RUN A COMPLIANCE SWEEP + VISUALIZE IT (yourself)
_Practical runbook. Python = `C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe` (call it `$CPY` below)._

## The mental model (30 sec)
- You describe a list of **configs**, each config = `amp:gain`. `amp` scales the body-wave amplitude command
  (×0.4 rad base); `gain` scales the body stiffness kp (and kv too, UNLESS you pass `--kp-only`, which keeps kv).
- Each config runs as **one env**; all configs in a call run **in parallel in ONE IsaacLab boot** (fast). Keep it to
  **≤12 configs per call** (16 GB RAM cap) — for more, split into several calls.
- Everything else is held at a004 (legs, pitch/bio, wave=3.0, coupling 40/80) and physics runs at **1 ms**.
- You pick a **terrain**: `flat` (amp0) · `mild` (λ90a8) · `rough` (λ60a16=0.267) · `vrough` (λ30a16=0.533) ·
  `xrough` (λ30a20=0.667). Roughness rises left→right.

## STEP 1 — run the sweep
Main tool: `scripts/analysis/compliance_titrate_2026-07-13.py`.
```powershell
$CPY='C:\Users\wxy22\anaconda3\envs\isaaclab_50\python.exe'
cd C:\Users\wxy22\Documents\Centipede_IsaacLab_RL
# example: on vrough, sweep stiffness from 1x down to 1e-4x, KEEPING damping (kv):
& $CPY -u scripts/analysis/compliance_titrate_2026-07-13.py --terrain vrough --label mysweep --kp-only `
    --configs 1:1 1:0.3 1:0.1 1:0.03 1:0.01 1:0.003 1:0.001 1:0.0003 1:0.0001
```
- `--label` names the run (used in filenames + as the ckpt key `mysweep_vrough_s4242`).
- Drop `--kp-only` to lower kp **and** kv together (the under-damped regime).
- To sweep AMPLITUDE instead: fix the gain, vary amp, e.g. `--configs 1:0.01 3:0.01 10:0.01 30:0.01 100:0.01`.
- It prints a table at the end AND saves everything. Watch progress live:
  `Get-Content outputs\analysis\compliance_titrate_2026-07-13\AUTORUN_LEDGER.md -Wait`
- ONE GPU job at a time. The script kills its own boot between runs; if you Ctrl-C, run
  `Get-Process python | Stop-Process -Force` before the next launch.

## STEP 2 — read the numbers (any run, any time)
```powershell
& $CPY scripts/analysis/compliance_view_2026-07-13.py --label mysweep_vrough
```
Columns: **`corr_bl`** = forward speed (bl/cycle), ≤0 if the config failed. **`bar04_real`** = realized body wave
(1.0 = baseline magnitude). **`valid`** = grounded + alive + no-teleport. **`why`** = launched / teleport / dead if
invalid. (Raw data also in `outputs\analysis\compliance_titrate_2026-07-13\data\titrate_ckpt.json`.)

## STEP 3 — VISUALIZE as video (the same helper renders it)
```powershell
# print the table AND render env 0 and env 6 of that run to mp4:
& $CPY scripts/analysis/compliance_view_2026-07-13.py --label mysweep_vrough --render 0 6
```
- `--render <indices>` renders those envs (0-based, in the order of your `--configs`) to
  `outputs\analysis\compliance_titrate_2026-07-13\videos\mysweep_vrough_s4242_env<N>.mp4` + an audit json.
- CPU-only PyVista OFFLINE renderer (not Omniverse), ~100 s/clip. **Run it with no IsaacLab boot alive** (they fight
  for RAM). It uses `--terrain` (feet rest ~0.45 mm on the surface, correct); it prints each clip's full path.

## STEP 4 — VISUALIZE as a curve (optional)
`scripts/analysis/compliance_titrate_figure_2026-07-13.py` plots the two-stage titration (realized + forward vs gain,
then vs amp). Re-point it at your labels if you want a custom curve. Figures land in the run's `figures/` and
`docs/figures/`.

## Recap of the 3 commands
```powershell
# 1 run     : compliance_titrate_2026-07-13.py --terrain <t> --label <name> [--kp-only] --configs amp:gain ...
# 2 read    : compliance_view_2026-07-13.py --label <name>_<terrain>
# 3 render  : compliance_view_2026-07-13.py --label <name>_<terrain> --render <env indices>
```
Each config = 1 env, ≤12/boot, 1 ms physics, everything but (amp,gain) held at a004.
