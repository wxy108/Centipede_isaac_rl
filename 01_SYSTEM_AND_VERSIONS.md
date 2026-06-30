# 1. System & Versions

Values verified against the parent project on 2026-06-30 (sources cited file:line). Where a value is only a
requirements **pin** (not a captured installed version), it is labelled as such — see §1.3.

## 1.1  Verified table
| Component | Value | Source / kind |
|---|---|---|
| OS | Windows 11 | REPORT:400 |
| Conda env | `isaaclab_50` | CLOSEUP:102 |
| Python | **3.11** (rendering / IsaacLab pipeline) | runtime-confirmed: 505 `*.cpython-311.pyc`; pinned `IsaacLab/environment.yml:10` (`python=3.11`). A few standalone optimization scripts ran 3.10 (`scripts/optimization/__pycache__/*.cpython-310.pyc`). Exact patch not captured. |
| GPU | NVIDIA GeForce RTX 5070 Laptop | REPORT:401 |
| NVIDIA driver | 577.05 | REPORT:401 |
| CUDA | 12.9 (driver-reported) | REPORT:401 |
| Isaac Lab | **2.2.1** | IsaacLab/VERSION |
| **Isaac Sim** | **5.0.0** (RESOLVED — see §1.2) | default app config `IsaacLab/apps/isaaclab.python.headless.kit:18` (`app.version="5.0.0"`); run assets resolve to `Assets/Isaac/5.0/` (`IsaacLab/logs/rl_games/cartpole/2026-05-06_18-24-20/params/env.yaml:128,195`); selector `app_launcher.py:703-706,934-956` |
| PyVista (offline renderer) | **0.48.4** | **installed-runtime** (`pv.__version__`) transcribed to `outputs/visual_audit/offline_renderer_2026-06-22/MANIFEST.md:68` from `meta/inspect_assets.py:47` |
| FFmpeg (bundled via imageio-ffmpeg) | **7.1** | MANIFEST.md:71 — NOTE: this is FFmpeg's version (`imageio_ffmpeg.get_ffmpeg_version()`, inspect_assets.py:27), **not** the `imageio-ffmpeg` pip package version (that is not recorded anywhere). |
| torch | **==2.7.0** (cu128) | **pin** — install cmd `IsaacLab/isaaclab.sh:374,380` / `.bat:346,352`; range `>=2.7` in `IsaacLab/source/isaaclab/setup.py:23`. Installed version not captured. |
| gymnasium | **==1.2.0** | **pin** — `IsaacLab/source/isaaclab/setup.py:30`, egg-info requires.txt:7 (a STALE `0.29.0` lingers in `config/extension.toml:20`). |
| Pillow (PIL) | **==11.2.1** | **pin** — `IsaacLab/source/isaaclab/isaaclab.egg-info/requires.txt:13` |
| numpy | **<2** (i.e. 1.x) | **range** + runtime corroboration `outputs/visual_audit/offline_renderer_2026-06-22/logs/omniverse_fresh.log:93` ("does not support NumPy 2.0"); pin `isaaclab.egg-info/requires.txt:1` |
| rl_games | git ref `@python3.11` (no semver) | `IsaacLab/source/isaaclab_rl/isaaclab_rl.egg-info/requires.txt` |
| imageio (encoder) | `imageio.v2`, libx264, quality 8 | render_winner:931; offline:548 |
| trimesh / opencv-python (cv2) / imageio / vtk / matplotlib | used by the pipeline, **no version recorded anywhere in the repo** | trimesh used offline:39,41 (declared unversioned in egg-info); cv2 render_winner:940; matplotlib unpinned `IsaacLab/docs/requirements.txt:15` |

## 1.2  Isaac Sim is 5.0.0 — the 4.5-vs-5.0 question, resolved
- Isaac Lab keys its asset root to the running Isaac Sim major version. Two independent real runs resolved to
  `Assets/Isaac/**5.0**/...` (`logs/rl_games/cartpole/2026-05-06_18-24-20/params/env.yaml:128,195`; same in the
  18-18-19 run). Zero hits for `Assets/Isaac/4.5/` anywhere.
- The launcher uses the **root** `apps/` config (`app.version="5.0.0"`, headless.kit:18) by default and switches to
  `apps/isaacsim_4_5/` (`app.version="4.5.0"`) **only if** `is_isaac_sim_version_4_5()` is True
  (`app_launcher.py:703-706`); that check reads the installed `isaacsim` package and is True only for `4.5*`
  (`app_launcher.py:934-956`). The 5.0 asset root proves that branch was False → installed Isaac Sim is **not** 4.5.x.
- The conda env name `isaaclab_50` is therefore the **correct** signal. The remembered "4.5 boot banner" is a red
  herring: the `isaacsim_4_5/` folder is Isaac Lab's stock back-compat config (dormant here), and the Kit content
  channel is `kit/prod/106` in BOTH configs — so a "106" banner does not imply 4.5.
- Official pairing (secondary to the repo evidence): Isaac Lab 2.2.x ships for **Isaac Sim 5.0** with 4.5
  back-compat; Isaac Lab 2.3 moved to 5.1. This repo is 2.2.1. (Isaac Lab releases / 2.2 GA notes.)
- **Residual:** the exact patch (5.0.0 vs a 5.0.x hotfix) is not in the repo; the default app config declares
  `5.0.0`. The only file that would pin the installed patch is `<isaaclab_50 env>/.../isaacsim/VERSION` — it lives
  in the conda env, not the repo (§1.3).

## 1.3  What the repo does NOT contain (and how to pin it exactly)
There is **no captured environment dump** anywhere in the repo — no `pip freeze`, `pip list`, `conda list`, or
`conda env export`, and no vendored site-packages / `*.dist-info`. So the only **installed-confirmed** versions are
**PyVista 0.48.4** and **Python 3.11**; everything else above is a requirements **pin/range**, and
trimesh / cv2 / imageio / vtk have no recorded version at all. To pin them exactly, run this once on the Windows PC
and (optionally) drop the output into a `meta/` folder here:
```powershell
conda activate isaaclab_50
pip freeze   > C:\Users\wxy22\Documents\isaaclab-rendering-notes\meta\pip_freeze.txt
conda list   > C:\Users\wxy22\Documents\isaaclab-rendering-notes\meta\conda_list.txt
pip show isaacsim trimesh opencv-python imageio imageio-ffmpeg vtk torch numpy | Out-File `
            C:\Users\wxy22\Documents\isaaclab-rendering-notes\meta\key_pkgs.txt
```
That `pip_freeze.txt` is the authoritative answer for every "version not captured" row above (incl. the Isaac Sim patch).

## 1.4  Two machines
- **Dev / render box** — the RTX 5070 Laptop / Win11 above; all rendering measurements come from here.
- **Lab compute PC** — a separate Ubuntu machine (user `xqw5351`, no git) used for headless training/sweeps;
  deliverable rendering was done on the Windows box.
