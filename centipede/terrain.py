"""The two terrain constructors used by the clean project."""

from __future__ import annotations

import math

import numpy as np
from scipy.ndimage import gaussian_filter


ARENA_SIZE_M = 3.0
IMAGE_SIZE = 1536
TRI_LABELS = ("l1000a0", "l90a8", "l60a12")
TRI_SPECS = ((1000.0, 0.0), (90.0, 8.0), (60.0, 12.0))
TRI_Y_M = (-0.95, 0.0, 0.95)
TRI_X_M = 0.65
TRI_HALF_WIDTH_M = 0.43


def _spectral_band(size, world_size, freq_min, freq_max, components, amplitude, rng):
    x = np.linspace(-world_size / 2.0, world_size / 2.0, size)
    xx, yy = np.meshgrid(x, x)
    height = np.zeros((size, size), dtype=np.float64)
    for _ in range(int(components)):
        frequency = rng.uniform(freq_min, freq_max)
        angle = rng.uniform(0.0, np.pi)
        phase = rng.uniform(0.0, 2.0 * np.pi)
        kx = 2.0 * np.pi * frequency * np.cos(angle)
        ky = 2.0 * np.pi * frequency * np.sin(angle)
        height += np.sin(kx * xx + ky * yy + phase)
    std = height.std()
    return height / std * amplitude if std > 1e-12 else height


def single_wavelength(lambda_mm, amplitude_mm, seed, size=IMAGE_SIZE, world_size=ARENA_SIZE_M):
    """MuJoCo-compatible source heightmap before PNG quantization and z shift."""
    amplitude_m = float(amplitude_mm) * 1e-3
    if amplitude_m == 0.0:
        return np.zeros((int(size), int(size)), dtype=np.float64)
    wavelength_m = float(lambda_mm) * 1e-3
    frequency = 1.0 / wavelength_m
    height = _spectral_band(
        int(size), float(world_size), 0.9 * frequency, 1.1 * frequency,
        8, amplitude_m, np.random.default_rng(int(seed)),
    )
    pixels_per_cycle = wavelength_m / float(world_size) * int(size)
    height = gaussian_filter(height, sigma=max(0.25 * pixels_per_cycle, 0.5))
    height -= height.mean()
    peak = max(abs(height.min()), abs(height.max()))
    return height * (amplitude_m / peak) if peak > 1e-12 else height


def tri_terrain(seed=223607):
    height = np.zeros((IMAGE_SIZE, IMAGE_SIZE), dtype=np.float64)
    cell = ARENA_SIZE_M / (IMAGE_SIZE - 1)
    half_pixels = int(round(TRI_HALF_WIDTH_M / cell))
    width = 2 * half_pixels
    center = IMAGE_SIZE // 2
    for center_y, (lambda_mm, amplitude_mm) in zip(TRI_Y_M, TRI_SPECS):
        row_center = int(round((center_y + ARENA_SIZE_M / 2) / ARENA_SIZE_M * (IMAGE_SIZE - 1)))
        row0, row1 = row_center - half_pixels, row_center + half_pixels
        if amplitude_mm == 0.0:
            strip = np.zeros((width, IMAGE_SIZE), dtype=np.float64)
        else:
            full = single_wavelength(lambda_mm, amplitude_mm, seed)
            strip = full[center - half_pixels : center - half_pixels + width].copy()
            strip -= strip.mean()
        height[row0:row1] = strip
    return height


def homogeneous_terrain(lambda_mm, amplitude_mm, seed):
    return single_wavelength(lambda_mm, amplitude_mm, seed)


def to_mujoco_collision(heightmap):
    """Apply the PNG uint8 round-trip and MuJoCo hfield vertical convention."""
    source = np.asarray(heightmap, dtype=np.float64)
    low, high = float(source.min()), float(source.max())
    peak = max(abs(low), abs(high))
    z_max = max(2.0 * peak, 0.005)
    if high - low < 1e-15:
        return np.full_like(source, 0.5 * z_max)
    pixels = np.round((source - low) / (high - low) * 255.0).astype(np.uint8)
    return pixels.astype(np.float64) / 255.0 * z_max


def height_at(heightmap, x_m, y_m, arena_size_m=ARENA_SIZE_M):
    rows, cols = heightmap.shape
    fx = np.clip((x_m + arena_size_m / 2) / arena_size_m * (cols - 1), 0, cols - 1)
    fy = np.clip((y_m + arena_size_m / 2) / arena_size_m * (rows - 1), 0, rows - 1)
    x0, y0 = int(np.floor(fx)), int(np.floor(fy))
    x1, y1 = min(x0 + 1, cols - 1), min(y0 + 1, rows - 1)
    tx, ty = fx - x0, fy - y0
    return float(
        (1 - tx) * (1 - ty) * heightmap[y0, x0]
        + tx * (1 - ty) * heightmap[y0, x1]
        + (1 - tx) * ty * heightmap[y1, x0]
        + tx * ty * heightmap[y1, x1]
    )


def footprint_max(heightmap, x_m, y_m, radius_m=0.075, arena_size_m=ARENA_SIZE_M):
    rows, cols = heightmap.shape
    fx = (x_m + arena_size_m / 2) / arena_size_m * (cols - 1)
    fy = (y_m + arena_size_m / 2) / arena_size_m * (rows - 1)
    rx = radius_m / arena_size_m * (cols - 1)
    ry = radius_m / arena_size_m * (rows - 1)
    x0, x1 = max(0, math.floor(fx - rx)), min(cols - 1, math.ceil(fx + rx))
    y0, y1 = max(0, math.floor(fy - ry)), min(rows - 1, math.ceil(fy + ry))
    return float(np.max(heightmap[y0 : y1 + 1, x0 : x1 + 1]))


def build_mesh(stage, prim_path, source_heightmap, world_size=ARENA_SIZE_M):
    """Bake the collision-coordinate heightmap into a static USD triangle mesh."""
    from pxr import UsdGeom, UsdPhysics, Vt

    height = to_mujoco_collision(source_heightmap)
    n = height.shape[0]
    cell = float(world_size) / (n - 1)
    half = float(world_size) / 2.0
    ii, jj = np.meshgrid(np.arange(n), np.arange(n), indexing="xy")
    points = np.stack(
        [-half + ii * cell, -half + jj * cell, height], axis=-1
    ).astype(np.float32).reshape(-1, 3)

    j, i = np.meshgrid(np.arange(n - 1), np.arange(n - 1), indexing="ij")
    v00 = (j * n + i).ravel()
    v01 = (j * n + i + 1).ravel()
    v10 = ((j + 1) * n + i).ravel()
    v11 = ((j + 1) * n + i + 1).ravel()
    indices = np.empty((n - 1) * (n - 1) * 6, dtype=np.int32)
    indices[0::6], indices[1::6], indices[2::6] = v00, v01, v11
    indices[3::6], indices[4::6], indices[5::6] = v00, v11, v10
    counts = np.full((n - 1) * (n - 1) * 2, 3, dtype=np.int32)

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    return height

