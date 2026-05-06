# Centipede Isaac Lab — Setup Guide

## Prerequisites

- **GPU**: NVIDIA RTX 3070+ (16 GB VRAM recommended)
- **OS**: Ubuntu 22.04 (Windows possible but Linux recommended)
- **Driver**: NVIDIA driver 535+ (production branch)
- **Python**: 3.10 (Isaac Sim 4.x) or 3.11 (Isaac Sim 5.x)

## Step 1 — Install Isaac Sim via pip

```bash
# Create a fresh conda env (recommended)
conda create -n isaaclab python=3.10 -y
conda activate isaaclab

# Install Isaac Sim (pip method — simplest)
pip install isaacsim-rl isaacsim-replicator isaacsim-extscache-physics isaacsim-extscache-kit-sdk
```

If the pip install fails (version conflicts, missing wheels), fall back to the
Omniverse Launcher method:
  https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html

## Step 2 — Install Isaac Lab from source

```bash
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Install Isaac Lab (editable mode)
./isaaclab.sh --install  # Linux
# or: isaaclab.bat --install  # Windows
```

Verify:
```bash
python -c "import isaaclab; print(isaaclab.__version__)"
```

## Step 3 — Install this project

```bash
cd /path/to/Centipede_IsaacLab
pip install -e .
```

## Step 4 — Convert the MuJoCo model to USD

Isaac Sim has a built-in MJCF importer that converts `.xml` → `.usd`.

```bash
python scripts/convert_mjcf.py \
    --input  ../Centipede_MUJOCO-main/models/farms/centipede.xml \
    --output assets/centipede.usd
```

This script uses Isaac Sim's `MjcfConverter` API. It will:
1. Parse the MJCF XML (joints, actuators, meshes, hfield)
2. Build a USD articulation with matching joint names
3. Save to `assets/centipede.usd`

**After conversion, open `assets/centipede.usd` in Isaac Sim GUI** to verify:
- All 288+ joints present
- Collision meshes look right
- Freejoint on root body
- No missing links or flipped axes

## Step 5 — Run flat-ground standing test

```bash
python scripts/run_standalone.py --mode stand --duration 5
```

This loads the USD, spawns the centipede on a flat plane, and lets it settle
under gravity with no controller. Verify it stands at roughly the same height
as in MuJoCo (~0.026 m clearance).

## Step 6 — Run flat-ground walking test

```bash
python scripts/run_standalone.py --mode walk --duration 10
```

This activates the impedance traveling-wave controller and should produce
forward locomotion on flat ground.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ModuleNotFoundError: isaacsim` | Activate the conda env; re-run pip install |
| USD loads but robot explodes | Contact/friction params need tuning in `configs/sim_params.yaml` |
| Robot sinks through floor | Check collision filters; terrain plane may need `collisionEnabled=True` |
| Joints missing after MJCF import | Some joint types don't convert cleanly; check `convert_mjcf.py` output log |
| Very different behavior vs MuJoCo | Expected — PhysX vs MuJoCo contact solvers differ; re-tune gains |
