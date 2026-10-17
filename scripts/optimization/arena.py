"""
arena.py — multi-block blended terrain arena for Isaac Lab
============================================================

Builds a LARGE arena where each block has its own (wavelength, amplitude)
single-wavelength roughness, and adjacent blocks blend smoothly so there
are NO sharp seams at block boundaries.

Design
------
The arena is divided into an Nx × Ny grid of "blocks." Each block has a
randomly-sampled wavelength λ ∈ [10, 100] mm and amplitude a ∈ a_range.
The full-arena heightmap is computed via a partition-of-unity blend:

  for each block b at center (cx_b, cy_b) with footprint = 1 block-side:
      generate a *full-arena* heightmap H_b at block b's parameters
      compute a tent weight W_b(x,y) = max(0, 1-|x-cx_b|/Δx) ·
                                       max(0, 1-|y-cy_b|/Δy)
      H_arena += W_b · H_b
  H_arena /= sum_b W_b

Tent weights overlap by exactly one block-width so each pixel is a
weighted average of its 4 nearest block centers (bilinear blend).
The weights sum to 1 (partition of unity) → no seams at boundaries,
visible "wavelength character" persists in each block's interior, and
transitions are smooth.

For RL: each block is a distinct "patch" of terrain difficulty, the
robot can spawn on one and traverse to a neighbor with a continuously
varying terrain underfoot. Because adjacent blocks differ in wavelength
but the heightmap itself is continuous, the centipede won't see a step
discontinuity when crossing a boundary.

Output
------
Use `generate_arena_heightmap(...)` to get a full heightmap array;
use `build_arena_mesh(...)` to bake it into a USD triangle mesh with
PhysX collision; both are wrappers over the existing `terrain.py`
functions, so the same MuJoCo-byte-compat behavior applies.

Typical usage
-------------
    from arena import generate_arena_heightmap, build_arena_mesh

    h, blocks = generate_arena_heightmap(
        arena_size_m=4.0,
        n_blocks=(4, 4),
        wavelength_range_mm=(10.0, 100.0),
        amplitude_range_m=(0.005, 0.015),
        image_size=2048,
        seed=42,
    )

    # Render with matplotlib for a quick check:
    from arena import save_topdown_png
    save_topdown_png(h, blocks, arena_size_m=4.0, out_path="arena.png")

    # Or build the USD mesh in an Isaac Sim scene:
    build_arena_mesh(stage, "/World/Arena/Mesh", h,
                     arena_size_m=4.0, mesh_resolution=1024)
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple

import numpy as np

# Import from sibling terrain.py
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from terrain import (
    generate_single_wavelength_heightmap,
    build_terrain_mesh,
)


# ─────────────────────────────────────────────────────────────────────────────
# Block descriptor
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class BlockSpec:
    """Per-block parameters. (bx, by) are integer grid coords starting at 0."""
    bx: int
    by: int
    wavelength_mm: float          # mm
    amplitude_m:   float          # m
    seed:          int

    def to_dict(self):
        return dict(bx=self.bx, by=self.by,
                    wavelength_mm=self.wavelength_mm,
                    amplitude_m=self.amplitude_m,
                    seed=self.seed)


def default_blocks(n_blocks: Tuple[int, int],
                   wavelength_range_mm: Tuple[float, float] = (10.0, 100.0),
                   amplitude_range_m:   Tuple[float, float] = (0.005, 0.015),
                   seed: int = 42) -> List[BlockSpec]:
    """Sample one (wavelength, amplitude, seed) per block uniformly within
    the requested ranges. Deterministic given `seed`."""
    Nx, Ny = n_blocks
    rng = np.random.default_rng(seed)
    blocks = []
    for bx in range(Nx):
        for by in range(Ny):
            wl_mm = float(rng.uniform(*wavelength_range_mm))
            amp_m = float(rng.uniform(*amplitude_range_m))
            bseed = int(rng.integers(1, 1 << 30))
            blocks.append(BlockSpec(bx=bx, by=by,
                                    wavelength_mm=wl_mm,
                                    amplitude_m=amp_m,
                                    seed=bseed))
    return blocks


# ─────────────────────────────────────────────────────────────────────────────
# Heightmap generation
# ─────────────────────────────────────────────────────────────────────────────

def generate_arena_heightmap(
    arena_size_m:        float = 4.0,
    n_blocks:            Tuple[int, int] = (4, 4),
    wavelength_range_mm: Tuple[float, float] = (10.0, 100.0),
    amplitude_range_m:   Tuple[float, float] = (0.005, 0.015),
    image_size:          int   = 2048,
    seed:                int   = 42,
    blocks:              Optional[List[BlockSpec]] = None,
    edge_extrapolation_block_widths: float = 0.0,
    n_components:        int   = 8,
    weight_kind:         str   = "tent",
) -> Tuple[np.ndarray, List[BlockSpec]]:
    """Generate the full multi-block arena heightmap with smooth transitions.

    Args
    ----
    arena_size_m
        Full side length of the arena in metres. Arena is centered on world
        (0, 0). 4 m gives ~40 centipede body lengths of room — plenty for
        an RL pilot.
    n_blocks
        (Nx, Ny) grid count. Default 4×4 = 16 distinct terrain patches.
    wavelength_range_mm
        Min/max wavelength sampled per block. Default (10, 100) mm matches
        the user spec — fine bumps to long undulations. Each block draws
        ONE wavelength uniformly in this range.
    amplitude_range_m
        Min/max amplitude (peak height) sampled per block. Default
        (5 mm, 15 mm) keeps roughness within centipede leg-clearance range.
    image_size
        Heightmap resolution (pixels per side). 2048 over 4 m = 2 mm/pixel,
        which resolves the smallest 10 mm wavelength with 5 cells.
    seed
        Master RNG seed; controls the per-block (wavelength, amplitude,
        per-block-seed) sampling AND the per-block heightmap synthesis.
    blocks
        Optional explicit list of BlockSpec. If None, uses
        default_blocks(n_blocks, wavelength_range_mm, amplitude_range_m, seed).
    edge_extrapolation_block_widths
        Extrapolate weights past arena edges by this many block widths.
        0.0 (default) → tent weights vanish exactly at arena edges, so
        boundary blocks have full weight along their outer edge.
        Increase to 0.25-0.5 to soften the arena rim slightly.

    Returns
    -------
    heightmap_m : (image_size, image_size) numpy.ndarray, float64
        Full arena heightmap in metres. Mean-centered around 0.
    blocks : list of BlockSpec
        The block configuration that was used (for logging / RL spawn
        reference).
    """
    Nx, Ny = n_blocks
    if blocks is None:
        blocks = default_blocks(n_blocks, wavelength_range_mm,
                                amplitude_range_m, seed)
    elif len(blocks) != Nx * Ny:
        raise ValueError(f"blocks length ({len(blocks)}) doesn't match "
                         f"n_blocks ({Nx}×{Ny}={Nx*Ny})")

    H_arena = np.zeros((image_size, image_size), dtype=np.float64)
    W_total = np.zeros((image_size, image_size), dtype=np.float64)

    # Block size in pixels
    bs_x = image_size / float(Nx)
    bs_y = image_size / float(Ny)

    # Pre-compute a coordinate grid (pixel coords)
    YY, XX = np.meshgrid(
        np.arange(image_size, dtype=np.float64),
        np.arange(image_size, dtype=np.float64),
        indexing="ij",
    )

    print(f"[arena] generating {Nx}×{Ny} blocks at {image_size}×{image_size} "
          f"px over {arena_size_m:.2f} m  "
          f"(cell size = {arena_size_m/image_size*1000:.2f} mm)",
          flush=True)

    for k, spec in enumerate(blocks):
        # Per-block heightmap at FULL ARENA resolution (so each block
        # contributes spatially-aligned heights to the same grid).
        wl_m  = spec.wavelength_mm * 1e-3
        amp_m = spec.amplitude_m
        H_b, _, _ = generate_single_wavelength_heightmap(
            wavelength_m=wl_m,
            amplitude_m=amp_m,
            seed=spec.seed,
            image_size=image_size,
            world_size=arena_size_m,
            n_components=n_components,
        )

        # Per-block partition-of-unity weight centered at this block's center.
        cx_pix = (spec.bx + 0.5) * bs_x
        cy_pix = (spec.by + 0.5) * bs_y
        # Distance in BLOCK UNITS (1.0 = one block side)
        dx = (XX - cx_pix) / bs_x
        dy = (YY - cy_pix) / bs_y
        # Tent ramp: 1 at center, 0 at adjacent block centers.
        tx = np.maximum(0.0, 1.0 - np.abs(dx))
        ty = np.maximum(0.0, 1.0 - np.abs(dy))
        if weight_kind == "tent":
            # Original C0 (piecewise-linear) blend. Slope (dH/dx) has a
            # KINK of magnitude 2 |h_b(center)| / block_width at every
            # block center because each block's tent ramp flips
            # derivative sign there. Acceptable when block centers
            # happen to have h ≈ 0, but produces visible slope jumps
            # otherwise.
            W_b = tx * ty
        elif weight_kind == "smoothstep":
            # Cubic smoothstep s(t) = t² (3 - 2t) applied to each axis,
            # then tensor-product. Properties:
            #   s(0) = 0, s(1) = 1
            #   s'(0) = s'(1) = 0     -> ZERO derivative at endpoints
            #   s(1-t) + s(t) = 1     -> partition-of-unity preserved on
            #                            a regular grid (so Σ_blocks W_b
            #                            stays = 1.0 inside the arena
            #                            interior, same as tent).
            # Net effect: H = Σ W_b · h_b is C1 EVERYWHERE, including
            # block centers — no slope kink, no curvature blow-up.
            sx = tx * tx * (3.0 - 2.0 * tx)
            sy = ty * ty * (3.0 - 2.0 * ty)
            W_b = sx * sy
        else:
            raise ValueError(
                f"weight_kind={weight_kind!r}; expected 'tent' or 'smoothstep'."
            )

        H_arena += W_b * H_b
        W_total += W_b

        print(f"  block ({spec.bx:>2d}, {spec.by:>2d})  "
              f"λ={spec.wavelength_mm:6.2f} mm  "
              f"amp={spec.amplitude_m*1000:5.2f} mm  "
              f"seed={spec.seed:>10d}", flush=True)

    # ── NO RENORMALISATION (deliberate) ──
    # Earlier code did `H_arena /= W_total` (true partition of unity).
    # That produced a "spiky rim" artifact: at the arena corner only one
    # block contributes with W = 0.25, so the divide amplified that
    # block's heightmap by 4× while interior 4-block corners (W = 1.0)
    # got the AVERAGE of 4 random-phase sinusoids (≈0.5σ amplitude).
    # Net visual: full-amplitude bumps along the arena edge, smoothed
    # half-amplitude bumps in the interior — opposite of what we want.
    #
    # Without renormalisation:
    #   arena corner:        W ≈ 0.25  → result ≈ 0.25 × σ (fades to flat)
    #   arena edge midpoint: W ≈ 0.5   → result ≈ 0.5  × σ
    #   block centre:        W ≈ 1.0   → full block content (peak σ)
    #   2-block interior bd: W ≈ 1.0   → mean of 2 (≈0.7 σ)
    #   4-block interior bd: W ≈ 1.0   → mean of 4 (≈0.5 σ)
    # The arena rim now fades naturally to zero (like a physical arena
    # with a sloped edge), and block centres show full per-block content.
    H_arena -= H_arena.mean()

    # ── NO FINAL GAUSSIAN ──
    # An earlier version smoothed the blended heightmap with σ ≈
    # block_size/32 to kill any residual blending gradient. That sigma
    # was 4-16 pixels, which at our cell size (~2-8 mm/px) is a 20-30 mm
    # kernel — and it COMPLETELY destroys the 10 mm wavelength bumps
    # AND substantially attenuates the 50-100 mm bumps. Result: every
    # block looked uniform-gray, no visible per-block character.
    #
    # The bilinear partition-of-unity tent weights are inherently smooth
    # (C0 continuous everywhere; weights and their first derivatives are
    # continuous AT block centers and edges — only the second derivative
    # has a small kink at boundaries). No additional filtering is needed
    # to avoid sharp seams. The block-level wavelength character is
    # preserved.
    #
    # If smoothing is ever wanted again it must be ≤ 0.5 px (sub-cell)
    # to avoid touching the 10 mm wavelength.

    print(f"[arena] heightmap stats: min={H_arena.min()*1000:+.2f} mm  "
          f"max={H_arena.max()*1000:+.2f} mm  "
          f"rms={H_arena.std()*1000:.2f} mm", flush=True)

    return H_arena, list(blocks)


# ─────────────────────────────────────────────────────────────────────────────
# Stage B-v0 Pass 3 (2026-05-07): flat → rough transition arena
# ─────────────────────────────────────────────────────────────────────────────

def generate_flat_to_rough_transition_heightmap(
    arena_size_m:           float = 1.8,
    rough_wavelength_mm:    float = 18.0,
    rough_amplitude_m:      float = 0.010,
    seed:                   int   = 42,
    image_size:             int   = 1024,
    n_components:           int   = 8,
    transition_wavelengths_mm: Tuple[float, ...] = (18.0, 18.0, 30.0, 50.0, 60.0),
    transition_amplitudes_m:   Tuple[float, ...] = (0.010, 0.010, 0.010, 0.008, 0.006),
) -> Tuple[np.ndarray, List[BlockSpec]]:
    """Build a multi-block flat → rough heightmap with a WAVELENGTH GRADIENT.

    The transition zone uses several blocks of progressively shorter
    wavelength (and gradually-tapering amplitude). The rightmost block
    is flat (amp = 0 → zero heightmap). The existing
    `generate_arena_heightmap` machinery applies a partition-of-unity
    tent-weight blend between blocks; with multiple wavelengths in
    adjacent blocks, the spectral character of the heightmap smoothly
    shifts as you move across x. Height itself is C0 continuous
    everywhere (sum of weighted C∞ contributions). NO sudden height
    change at any boundary.

    Default block layout (n_blocks = 6 × 1, arena_size_m = 1.8 m,
    each block 0.30 m wide ≈ 14 centipede body lengths):
        block (0, 0)  centered at world x = -0.75 m  → λ=18 mm,  amp=10 mm  (rough_a)
        block (1, 0)  centered at world x = -0.45 m  → λ=18 mm,  amp=10 mm  (rough_b, different seed)
        block (2, 0)  centered at world x = -0.15 m  → λ=30 mm,  amp=10 mm  (medium)
        block (3, 0)  centered at world x = +0.15 m  → λ=50 mm,  amp= 8 mm  (intermediate)
        block (4, 0)  centered at world x = +0.45 m  → λ=60 mm,  amp= 6 mm  (long-wave)
        block (5, 0)  centered at world x = +0.75 m  → flat      (amp= 0)

    The 2 rough blocks have the SAME wavelength (18 mm) but DIFFERENT
    random seeds, so they have independent random phases. The
    smoothstep partition-of-unity blend between them produces a wider
    rough zone (~0.75 m of effectively-rough terrain when clipped to
    the arena bounds) without any visible seam.

    Forward locomotion runs the centipede from the flat side
    (block 3, x ≈ +0.15) toward decreasing x, sequentially encountering
    the long-wavelength gentle block, the medium block, and finally
    the full λ=18 mm rough block. There is NO sharp wavelength jump:
    the partition-of-unity blend mixes neighbouring blocks' spectra.

    Args
    ----
    arena_size_m
        Total side length of the (square) heightmap in metres. Default
        1.8 m (each block 0.30 m wide ≈ 14 centipede body lengths).
        Sized so that even a robot spawned at the leftmost block centre
        has > 100 mm of forward walking room PLUS a body half-length
        (≈ 12 mm) of safety buffer before the arena boundary, even at
        the verified gait speed (~17 mm/s × 6 s).
    rough_wavelength_mm, rough_amplitude_m
        Reference rough end (= block 0). Used as the FIRST entry of
        `transition_wavelengths_mm` / `transition_amplitudes_m` if those
        are left at defaults; ignored if the caller passes explicit
        sequences. Kept for back-compat and as a "rough end" marker.
    seed
        Master RNG seed. Each block's seed is derived as `seed + bx`
        so adjacent blocks have INDEPENDENT random phases (otherwise
        they would correlate visibly at the boundary).
    image_size
        Heightmap resolution per side (default 1024; cell size at 0.4 m
        is 0.39 mm/pix, well below the 18 mm wavelength).
    n_components
        Random sinusoids per block (default 8, matches arena.py default).
    transition_wavelengths_mm
        Per-rough-block wavelengths in mm, ROUGH → GENTLE order.
        Default (18, 18, 30, 50, 60) — TWO rough blocks at λ=18 with
        different per-block seeds give independent random phases that
        the partition-of-unity blend stitches together into a wider
        rough zone. Length defines the number of rough blocks; one
        extra flat block (amp=0) is appended on the right.
    transition_amplitudes_m
        Per-rough-block amplitudes in metres, in the same order.
        Default (0.010, 0.010, 0.010, 0.008, 0.006) — full amp for the
        two rough blocks AND the medium block, tapered through the
        intermediate λ=50 mm and long-wavelength gentle blocks.

    Returns
    -------
    heightmap_m : (image_size, image_size) numpy.ndarray, float64.
    blocks      : list of BlockSpec used (rough...gentle..flat order).
    """
    if len(transition_wavelengths_mm) != len(transition_amplitudes_m):
        raise ValueError(
            "transition_wavelengths_mm and transition_amplitudes_m must "
            f"have the same length; got "
            f"{len(transition_wavelengths_mm)} vs {len(transition_amplitudes_m)}"
        )
    n_rough = len(transition_wavelengths_mm)
    n_total = n_rough + 1   # +1 flat block at the right
    blocks: List[BlockSpec] = []
    for i, (wl_mm, amp_m) in enumerate(zip(transition_wavelengths_mm,
                                            transition_amplitudes_m)):
        blocks.append(BlockSpec(
            bx=i, by=0,
            wavelength_mm=float(wl_mm),
            amplitude_m=float(amp_m),
            seed=int(seed) + i,
        ))
    # Final flat block (amp=0). Wavelength is irrelevant when amp=0
    # (the heightmap collapses to zero in _spectral_band's std-norm
    # division — the call short-circuits to zeros for amp=0).
    # We still record the longest wavelength as a placeholder.
    blocks.append(BlockSpec(
        bx=n_total - 1, by=0,
        wavelength_mm=float(max(transition_wavelengths_mm)),
        amplitude_m=0.0,
        seed=int(seed) + n_total - 1,
    ))

    # generate_arena_heightmap signature requires wavelength_range_mm
    # and amplitude_range_m; with explicit blocks they are unused for
    # block sampling (default_blocks is bypassed) but the validators
    # still expect tuples. Provide tight ranges that contain the
    # selected values.
    min_wl_mm = float(min(transition_wavelengths_mm))
    max_wl_mm = float(max(transition_wavelengths_mm))
    return generate_arena_heightmap(
        arena_size_m=arena_size_m,
        n_blocks=(n_total, 1),
        wavelength_range_mm=(min_wl_mm, max_wl_mm),
        amplitude_range_m=(0.0, float(max(transition_amplitudes_m))),
        image_size=image_size,
        seed=int(seed),
        blocks=blocks,
        n_components=n_components,
        weight_kind="smoothstep",   # C1-continuous slope, NO kink at block centers
    )


# ─────────────────────────────────────────────────────────────────────────────
# USD mesh wrapper
# ─────────────────────────────────────────────────────────────────────────────

def build_arena_mesh(stage, prim_path: str, heightmap_m: np.ndarray,
                     arena_size_m: float = 4.0, mesh_resolution: int = 1024,
                     mujoco_compat: bool = True):
    """Bake the multi-block heightmap into a USD triangle mesh + PhysX
    collision. Thin wrapper over `terrain.build_terrain_mesh` that just
    forwards `arena_size_m` as the world_size."""
    return build_terrain_mesh(
        stage=stage,
        prim_path=prim_path,
        heightmap_m=heightmap_m,
        world_size=arena_size_m,
        mesh_resolution=mesh_resolution,
        mujoco_compat=mujoco_compat,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Spawn-point query (for upcoming RL step)
# ─────────────────────────────────────────────────────────────────────────────

def block_center_xy(arena_size_m: float, n_blocks: Tuple[int, int],
                    bx: int, by: int) -> Tuple[float, float]:
    """World-frame (x, y) at the centre of the requested block."""
    Nx, Ny = n_blocks
    bw = arena_size_m / float(Nx)
    bh = arena_size_m / float(Ny)
    x = -arena_size_m / 2.0 + (bx + 0.5) * bw
    y = -arena_size_m / 2.0 + (by + 0.5) * bh
    return x, y


def heightmap_value_at(heightmap_m: np.ndarray, arena_size_m: float,
                       x_world: float, y_world: float) -> float:
    """Bilinear lookup of the arena heightmap at world position (x, y)."""
    H, W = heightmap_m.shape
    # Pixel coordinates (note the y/row axis convention)
    fx = (x_world + arena_size_m / 2.0) / arena_size_m * (W - 1)
    fy = (y_world + arena_size_m / 2.0) / arena_size_m * (H - 1)
    fx = float(np.clip(fx, 0.0, W - 1))
    fy = float(np.clip(fy, 0.0, H - 1))
    ix0, iy0 = int(np.floor(fx)), int(np.floor(fy))
    ix1, iy1 = min(ix0 + 1, W - 1), min(iy0 + 1, H - 1)
    tx, ty = fx - ix0, fy - iy0
    z = ((1 - tx) * (1 - ty) * heightmap_m[iy0, ix0]
         +     tx  * (1 - ty) * heightmap_m[iy0, ix1]
         + (1 - tx) *     ty  * heightmap_m[iy1, ix0]
         +     tx  *     ty  * heightmap_m[iy1, ix1])
    return float(z)


def heightmap_patch_max_at(heightmap_m: np.ndarray, arena_size_m: float,
                           x_world: float, y_world: float,
                           radius_m: float) -> float:
    """Maximum sampled height in a square footprint around world ``(x, y)``."""
    if radius_m <= 0.0:
        return heightmap_value_at(heightmap_m, arena_size_m, x_world, y_world)
    H, W = heightmap_m.shape
    fx = (x_world + arena_size_m / 2.0) / arena_size_m * (W - 1)
    fy = (y_world + arena_size_m / 2.0) / arena_size_m * (H - 1)
    rx = radius_m / arena_size_m * (W - 1)
    ry = radius_m / arena_size_m * (H - 1)
    ix0 = max(0, int(np.floor(fx - rx)))
    ix1 = min(W - 1, int(np.ceil(fx + rx)))
    iy0 = max(0, int(np.floor(fy - ry)))
    iy1 = min(H - 1, int(np.ceil(fy + ry)))
    if ix0 > ix1 or iy0 > iy1:
        return heightmap_value_at(heightmap_m, arena_size_m, x_world, y_world)
    return float(np.max(heightmap_m[iy0:iy1 + 1, ix0:ix1 + 1]))


# ─────────────────────────────────────────────────────────────────────────────
# Top-down PNG visualization (no Isaac Sim needed)
# ─────────────────────────────────────────────────────────────────────────────

def save_topdown_png(heightmap_m: np.ndarray,
                     blocks: List[BlockSpec],
                     arena_size_m: float,
                     out_path: str,
                     n_blocks: Optional[Tuple[int, int]] = None,
                     dpi: int = 150) -> None:
    """Save a top-down terrain-coloured image of the arena with block
    boundaries overlaid as dashed lines and per-block (λ, a) labels.
    Useful for verifying the arena layout without booting Isaac Sim."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    H, W = heightmap_m.shape
    half = arena_size_m / 2.0

    fig, ax = plt.subplots(figsize=(8, 8), dpi=dpi)
    im = ax.imshow(
        heightmap_m * 1000.0,        # mm
        extent=(-half, half, -half, half),
        origin="lower",
        cmap="terrain",
        aspect="equal",
    )
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("height [mm]")

    # Block grid lines + labels
    if n_blocks is None:
        Nx = max(b.bx for b in blocks) + 1
        Ny = max(b.by for b in blocks) + 1
    else:
        Nx, Ny = n_blocks
    bw = arena_size_m / Nx
    bh = arena_size_m / Ny
    for i in range(1, Nx):
        ax.axvline(-half + i * bw, color="black", lw=0.5,
                   linestyle=":", alpha=0.6)
    for j in range(1, Ny):
        ax.axhline(-half + j * bh, color="black", lw=0.5,
                   linestyle=":", alpha=0.6)
    for spec in blocks:
        cx, cy = block_center_xy(arena_size_m, (Nx, Ny), spec.bx, spec.by)
        ax.text(cx, cy,
                f"λ={spec.wavelength_mm:.0f}mm\n"
                f"a={spec.amplitude_m*1000:.1f}mm",
                ha="center", va="center",
                fontsize=7, color="white",
                bbox=dict(facecolor="black", alpha=0.55,
                          edgecolor="none", pad=2))

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Multi-block arena  {arena_size_m:.1f} m × {arena_size_m:.1f} m  "
                 f"({Nx}×{Ny} blocks)")
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    print(f"[arena] wrote top-down preview → {out_path}", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# Matplotlib 3D isometric view (works ALWAYS — no Isaac Sim required)
# ─────────────────────────────────────────────────────────────────────────────

def save_3d_view_png(heightmap_m: np.ndarray,
                     blocks: List[BlockSpec],
                     arena_size_m: float,
                     out_path: str,
                     n_blocks: Optional[Tuple[int, int]] = None,
                     elev: float = 30.0,
                     azim: float = -55.0,
                     z_exaggeration: float = 4.0,
                     plot_resolution: int = 256,
                     dpi: int = 150) -> None:
    """Save an isometric 3D surface plot of the arena. This is the
    canonical 'preview the arena' rendering — it bypasses the Isaac Lab
    renderer entirely and is therefore immune to the Fabric/usdrt bug
    documented in NOTES_RENDERING §0. Use this to verify the terrain
    layout before running anything in Isaac Sim.

    Parameters
    ----------
    heightmap_m
        Full-arena heightmap in metres (output of generate_arena_heightmap).
    blocks
        List of BlockSpec with the per-block (λ, amp) parameters; used
        to label each block at its centre.
    arena_size_m
        Side length of the arena in metres (defines the x/y axes).
    out_path
        Where to write the PNG.
    elev, azim
        Matplotlib view angles in degrees. Default (30, -55) gives a
        clean bird's-eye SE perspective.
    z_exaggeration
        Vertical scale factor for visual readability. Default 4× makes
        the bumps pronounced enough to see on a 4 m-wide arena where
        the actual peak is ~15 mm. The colorbar shows true heights.
    plot_resolution
        Surface mesh resolution for the plot (downsamples from full
        heightmap for speed). 256 plots cleanly in a few seconds.
    dpi
        Output DPI.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers '3d')

    H, W = heightmap_m.shape
    half = arena_size_m / 2.0

    if H > plot_resolution:
        from scipy.ndimage import zoom
        ratio = plot_resolution / float(H)
        h_plot = zoom(heightmap_m, ratio, order=1)
    else:
        h_plot = heightmap_m
    nx, ny = h_plot.shape

    x = np.linspace(-half, half, nx)
    y = np.linspace(-half, half, ny)
    X, Y = np.meshgrid(x, y)
    Z_mm = h_plot * 1000.0

    fig = plt.figure(figsize=(12, 9), dpi=dpi)
    ax  = fig.add_subplot(111, projection="3d")

    surf = ax.plot_surface(X, Y, Z_mm,
                           cmap="terrain",
                           edgecolor="none",
                           antialiased=True,
                           rcount=plot_resolution,
                           ccount=plot_resolution,
                           linewidth=0)
    cbar = fig.colorbar(surf, ax=ax, fraction=0.025, pad=0.06)
    cbar.set_label("true height [mm]")

    if n_blocks is None:
        Nx = max(b.bx for b in blocks) + 1
        Ny = max(b.by for b in blocks) + 1
    else:
        Nx, Ny = n_blocks
    bw = arena_size_m / Nx
    bh = arena_size_m / Ny
    z_floor = float(Z_mm.min())
    for i in range(1, Nx):
        xi = -half + i * bw
        ax.plot([xi, xi], [-half, half], [z_floor, z_floor],
                color="black", lw=0.5, alpha=0.5)
    for j in range(1, Ny):
        yj = -half + j * bh
        ax.plot([-half, half], [yj, yj], [z_floor, z_floor],
                color="black", lw=0.5, alpha=0.5)

    label_z = float(Z_mm.max()) + 4.0
    for spec in blocks:
        cx, cy = block_center_xy(arena_size_m, (Nx, Ny), spec.bx, spec.by)
        ax.text(cx, cy, label_z,
                f"\u03bb={spec.wavelength_mm:.0f}\na={spec.amplitude_m*1000:.1f}",
                ha="center", va="bottom", fontsize=6, color="black",
                bbox=dict(facecolor="white", alpha=0.7,
                          edgecolor="grey", lw=0.3, pad=1.5))

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("height [mm]")
    ax.set_title(f"Multi-block arena  {arena_size_m:.1f} m \u00d7 {arena_size_m:.1f} m  "
                 f"({Nx}\u00d7{Ny} blocks)")
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect([1.0, 1.0, 1.0 / max(z_exaggeration, 1e-3)])

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    print(f"[arena] wrote 3D isometric view -> {out_path}", flush=True)


# ----------------------------------------------------------------------
# CLI: generate + save previews (top-down + 3D)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Generate a multi-block arena heightmap and save "
                    "matplotlib previews (top-down + 3D isometric). "
                    "No Isaac Sim required.")
    parser.add_argument("--arena-size",     type=float, default=4.0)
    parser.add_argument("--n-blocks",       type=int,   default=4,
                        help="Block grid is N x N (default 4 -> 16 blocks).")
    parser.add_argument("--wavelength-min", type=float, default=10.0)
    parser.add_argument("--wavelength-max", type=float, default=100.0)
    parser.add_argument("--amplitude-min",  type=float, default=0.005)
    parser.add_argument("--amplitude-max",  type=float, default=0.015)
    parser.add_argument("--image-size",     type=int,   default=2048)
    parser.add_argument("--seed",           type=int,   default=42)
    parser.add_argument("--n-components",   type=int,   default=8,
                        help="Number of random sinusoids summed per "
                             "block (default 8, matches the optimizer "
                             "training terrain). Lower = cleaner directional "
                             "waves, less spiky 3D look. Set 1 for a single "
                             "directional sinusoid per block, or 2-4 for a "
                             "compromise. CHANGES the actual physical "
                             "terrain — use the optimizer default (8) when "
                             "you want the centipede to encounter the same "
                             "kind of terrain it was trained on.")
    parser.add_argument("--out",            default="outputs/arena_preview.png",
                        help="Top-down PNG path.")
    parser.add_argument("--out-3d",         default=None,
                        help="3D isometric PNG path. If unset, "
                             "<out>_3d.png. Empty string skips.")
    parser.add_argument("--z-exaggeration", type=float, default=4.0,
                        help="Vertical scale multiplier for the 3D view "
                             "(default 4 makes mm bumps visible against "
                             "metre-wide arena). Set 1.0 for true scale "
                             "(bumps will look almost flat).")
    parser.add_argument("--plot-resolution",type=int,   default=256,
                        help="Surface mesh resolution for the 3D plot "
                             "(default 256, downsamples from --image-size). "
                             "Higher = more interior detail visible but "
                             "slower to render. 512 is a good balance.")
    parser.add_argument("--elev",           type=float, default=30.0,
                        help="3D view elevation angle in degrees.")
    parser.add_argument("--azim",           type=float, default=-55.0,
                        help="3D view azimuth angle in degrees.")
    args = parser.parse_args()

    h, blocks = generate_arena_heightmap(
        arena_size_m=args.arena_size,
        n_blocks=(args.n_blocks, args.n_blocks),
        wavelength_range_mm=(args.wavelength_min, args.wavelength_max),
        amplitude_range_m=(args.amplitude_min, args.amplitude_max),
        image_size=args.image_size,
        seed=args.seed,
        n_components=args.n_components,
    )
    save_topdown_png(h, blocks, args.arena_size, args.out,
                     n_blocks=(args.n_blocks, args.n_blocks))
    out_3d = args.out_3d
    if out_3d is None:
        base, ext = os.path.splitext(args.out)
        out_3d = f"{base}_3d{ext or '.png'}"
    if out_3d:
        save_3d_view_png(h, blocks, args.arena_size, out_3d,
                         n_blocks=(args.n_blocks, args.n_blocks),
                         elev=args.elev, azim=args.azim,
                         z_exaggeration=args.z_exaggeration,
                         plot_resolution=args.plot_resolution)
