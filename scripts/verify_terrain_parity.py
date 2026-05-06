#!/usr/bin/env python3
"""
verify_terrain_parity.py — assert Isaac and MuJoCo heightmaps are identical
==========================================================================
Runs Centipede_MUJOCO-main's ``generate_single_wavelength_terrain`` and
Centipede_IsaacLab's ``generate_single_wavelength_heightmap`` with
matching arguments and the same RNG seed, and verifies pixel-equality.

After the byte-faithful port of _spectral_band, the two functions should
produce arrays that are equal up to floating-point round-off (typically
exact, since both use float64 and the same numpy ops in the same order).

Usage
-----
    cd Centipede_IsaacLab
    python scripts/verify_terrain_parity.py
"""
import os
import sys
import numpy as np

ISAAC_LAB_ROOT  = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
MUJOCO_ROOT     = os.path.normpath(os.path.join(
    ISAAC_LAB_ROOT, "..", "Centipede_MUJOCO-main"))

# Import Isaac generator
sys.path.insert(0, os.path.join(ISAAC_LAB_ROOT, "scripts", "optimization"))
from terrain import generate_single_wavelength_heightmap as gen_isaac

# Import MuJoCo generator (it lives inside scripts/sweep/wavelength_sweep.py)
sys.path.insert(0, os.path.join(MUJOCO_ROOT, "scripts", "sweep"))
sys.path.insert(0, os.path.join(MUJOCO_ROOT, "controllers", "farms"))
sys.path.insert(0, os.path.join(MUJOCO_ROOT, "terrain", "generator"))

# wavelength_sweep imports from impedance_controller; if that import
# fails (no mujoco / yaml here in plain python), import the function
# in isolation by exec'ing just the relevant chunk.
try:
    from wavelength_sweep import generate_single_wavelength_terrain as gen_mujoco
except Exception as e:
    print(f"[verify] direct import failed: {e}")
    print(f"[verify] falling back to isolated re-implementation read from "
          f"wavelength_sweep.py ...")
    # Pull the function via exec — it depends only on numpy + scipy +
    # generate._spectral_band, none of which need mujoco.
    src_path = os.path.join(MUJOCO_ROOT, "scripts", "sweep",
                            "wavelength_sweep.py")
    with open(src_path, "r", encoding="utf-8") as f:
        src = f.read()
    # Find the function block.
    import textwrap
    start = src.find("def generate_single_wavelength_terrain")
    end   = src.find("\ndef ", start + 1)
    fn_src = src[start:end]
    # Inject deps into a module namespace.
    from generate import _spectral_band  # MuJoCo's _spectral_band
    from scipy.ndimage import gaussian_filter
    ns = {"np": np, "_spectral_band": _spectral_band,
          "gaussian_filter": gaussian_filter}
    exec(fn_src, ns)
    gen_mujoco = ns["generate_single_wavelength_terrain"]


def compare_one(wavelength_m, amplitude_m, seed):
    h_isaac, rms_i, peak_i = gen_isaac(
        wavelength_m=wavelength_m, amplitude_m=amplitude_m, seed=seed,
        image_size=1024, world_size=1.0, n_components=8,
    )
    h_muj, rms_m, peak_m = gen_mujoco(
        wavelength_m=wavelength_m, amplitude_m=amplitude_m, seed=seed,
        image_size=1024, world_half=0.5, n_components=8,
    )
    diff = h_isaac - h_muj
    max_abs = float(np.max(np.abs(diff)))
    rmse    = float(np.sqrt(np.mean(diff * diff)))
    print(f"  λ={wavelength_m*1000:6.2f}mm  amp={amplitude_m*1000:5.2f}mm  "
          f"seed={seed:>3d}  | "
          f"Isaac rms={rms_i*1000:.4f}mm peak={peak_i*1000:.4f}mm  | "
          f"MuJoCo rms={rms_m*1000:.4f}mm peak={peak_m*1000:.4f}mm  | "
          f"max|Δ|={max_abs:.3e}m  rmse={rmse:.3e}m")
    return max_abs


def compare_physical(wavelength_m, amplitude_m, seed):
    """Compare what the SOLVERS actually see — i.e. after MuJoCo's PNG
    quantisation and z-shift, and Isaac's mujoco-compat to_mujoco_terrain.
    These should produce the same world-frame z values at every pixel.
    """
    from terrain import to_mujoco_terrain, mujoco_z_max
    from PIL import Image
    import io

    # Isaac side: float heightmap → MuJoCo-compat physical heights
    h_isaac, _, peak_i = gen_isaac(
        wavelength_m=wavelength_m, amplitude_m=amplitude_m, seed=seed,
    )
    h_isaac_phys, z_max_i = to_mujoco_terrain(h_isaac)

    # MuJoCo side: float heightmap → uint8 PNG → re-decode → z_max scaling.
    # We emulate the path Centipede_MUJOCO-main takes: heightmap_to_png
    # (float→uint8 with full 0..255 range) then MuJoCo hfield reads
    # pixel/255 * z_max as the world z.
    h_muj, _, peak_m = gen_mujoco(
        wavelength_m=wavelength_m, amplitude_m=amplitude_m, seed=seed,
        image_size=1024, world_half=0.5,
    )
    h_min = h_muj.min(); h_max = h_muj.max(); span = h_max - h_min
    pixel = np.round((h_muj - h_min) / span * 255.0).astype(np.uint8)
    # Round-trip through PNG file just to be sure no extra quantisation
    # or palette mangling sneaks in.
    buf = io.BytesIO()
    Image.fromarray(pixel, mode="L").save(buf, format="PNG")
    buf.seek(0)
    pixel_rt = np.array(Image.open(buf).convert("L"), dtype=np.uint8)
    assert np.array_equal(pixel, pixel_rt), "PNG roundtrip altered pixels!"
    z_max_m = mujoco_z_max(peak_m)
    h_muj_phys = pixel_rt.astype(np.float64) / 255.0 * z_max_m

    diff = h_isaac_phys - h_muj_phys
    max_abs = float(np.max(np.abs(diff)))
    rmse    = float(np.sqrt(np.mean(diff * diff)))
    print(f"  λ={wavelength_m*1000:6.2f}mm amp={amplitude_m*1000:5.2f}mm "
          f"seed={seed:>3d}  z_max(Isaac/MuJoCo)={z_max_i*1000:.2f}/"
          f"{z_max_m*1000:.2f}mm  range=[{h_isaac_phys.min()*1000:+.2f},"
          f"{h_isaac_phys.max()*1000:+.2f}]mm  max|Δ|={max_abs:.3e}m "
          f"rmse={rmse:.3e}m")
    return max_abs


def main():
    print("[verify] STAGE 1: float heightmap parity (numpy arrays)")
    print()
    cases = [
        (0.018, 0.005, 42),
        (0.018, 0.010, 42),
        (0.030, 0.005,  7),
        (0.090, 0.010, 13),
        (0.130, 0.005, 99),
    ]
    worst1 = 0.0
    for wl, amp, sd in cases:
        worst1 = max(worst1, compare_one(wl, amp, sd))
    print()
    if worst1 < 1e-9:
        print(f"[verify] ✓ float heightmaps pixel-identical "
              f"(max|Δ| = {worst1:.3e}m)")
    elif worst1 < 1e-6:
        print(f"[verify] ≈ float heightmaps match within FP round-off "
              f"(max|Δ| = {worst1:.3e}m)")
    else:
        print(f"[verify] ✗ stage 1 MISMATCH ({worst1:.3e}m); aborting "
              f"before checking the physical model.")
        sys.exit(1)

    print()
    print("[verify] STAGE 2: physical-model parity")
    print("[verify] (Isaac to_mujoco_terrain == MuJoCo PNG → hfield z mapping)")
    print()
    worst2 = 0.0
    for wl, amp, sd in cases:
        worst2 = max(worst2, compare_physical(wl, amp, sd))
    print()
    if worst2 < 1e-9:
        print(f"[verify] ✓ physical heights pixel-identical "
              f"(max|Δ| = {worst2:.3e}m). Isaac mesh and MuJoCo hfield "
              f"are the same terrain.")
    elif worst2 < 1e-6:
        print(f"[verify] ≈ physical heights match within FP round-off "
              f"(max|Δ| = {worst2:.3e}m).")
    else:
        print(f"[verify] ✗ stage 2 MISMATCH ({worst2:.3e}m). The PNG/"
              f"hfield model in Isaac has drifted from MuJoCo.")
        sys.exit(1)


if __name__ == "__main__":
    main()
