"""
terrain.py — single-wavelength rough terrain generator for Isaac Lab
======================================================================
Mirrors the numpy/scipy pipeline from
    Centipede_MUJOCO-main/scripts/sweep/wavelength_sweep.py
    Centipede_MUJOCO-main/terrain/generator/generate.py
so that the SAME physical terrain (given the same seed + wavelength +
amplitude) can be reproduced in PhysX. The MuJoCo version writes a PNG
and lets MuJoCo's `<hfield>` consume it; here we bake the heightmap
into a USD triangle mesh and apply a PhysX CollisionAPI.

Design choices:
  * Heightmap is generated at high resolution (1024×1024 over a 1m×1m
    arena) for fidelity, then DOWNSAMPLED to a coarser collision mesh
    (default 128×128) — collision queries don't need millimeter-grade
    detail and 1M-vertex meshes blow PhysX cooking budgets.
  * Mesh top surface is at z=0 by default; the heightmap is recentered
    around its mean so amplitude = peak deflection from z=0.
  * No visual material is bound — caller can bind one if they want
    something prettier than the default white.
"""

import numpy as np
from scipy.ndimage import gaussian_filter


# ── Spectral-band synthesis ───────────────────────────────────────────────────
# This is a BYTE-FAITHFUL PORT of MuJoCo's
#   Centipede_MUJOCO-main/terrain/generator/generate.py::_spectral_band
# The earlier "adapted" version had four small but significant differences
# (grid construction, angle range, sin vs cos, std vs n-component
# normalisation) that meant Isaac and MuJoCo with the same seed produced
# DIFFERENT heightmaps. After this fix, identical seeds → identical pixels
# → identical terrain.

def _spectral_band(image_size, world_size, freq_min, freq_max,
                   n_components, amplitude, orientation_spread, rng):
    """Sum of n random sinusoids in [freq_min, freq_max] cycles/m.

    Exact port of MuJoCo's generate.py::_spectral_band:
      * grid via np.linspace(-w/2, w/2, image_size)  [endpoint inclusive]
      * angle ~ U(0, π · orientation_spread)
      * h += np.sin(kx·X + ky·Y + phase)
      * normalise by std, then multiply by amplitude
    Same RNG calls in the same order (freq, angle, phase per component),
    so a shared np.random.default_rng(seed) reproduces the MuJoCo terrain
    pixel-for-pixel.
    """
    x = np.linspace(-world_size / 2.0, world_size / 2.0, image_size)
    X, Y = np.meshgrid(x, x)
    h = np.zeros((image_size, image_size), dtype=np.float64)
    for _ in range(int(n_components)):
        freq  = rng.uniform(freq_min, freq_max)
        angle = rng.uniform(0.0, np.pi * orientation_spread)
        phase = rng.uniform(0.0, 2.0 * np.pi)
        kx = 2.0 * np.pi * freq * np.cos(angle)
        ky = 2.0 * np.pi * freq * np.sin(angle)
        h += np.sin(kx * X + ky * Y + phase)
    std = h.std()
    if std > 1e-12:
        h = h / std * amplitude
    return h


def generate_single_wavelength_heightmap(wavelength_m, amplitude_m, seed,
                                         image_size=1024, world_size=1.0,
                                         n_components=8):
    """Generate a (image_size, image_size) heightmap at the given wavelength.

    Byte-faithful port of
    ``Centipede_MUJOCO-main/scripts/sweep/wavelength_sweep.py
        ::generate_single_wavelength_terrain``.
    The MuJoCo signature took ``world_half`` (default 0.5) and doubled it
    internally; we take ``world_size`` directly (default 1.0) to match
    the existing Isaac callers — the math is identical when the same
    full-extent value is used (MuJoCo default 0.5×2 = 1.0 = our default).

    Same seed + same wavelength + same amplitude + same world_size
    produces an IDENTICAL heightmap to the MuJoCo simulation.

    Returns
    -------
    h_m : np.ndarray (image_size, image_size), float
        Heightmap in METRES, peak amplitude = amplitude_m.
    rms_m : float
        Root-mean-square height.
    peak_m : float
        Peak (max abs) height.
    """
    rng = np.random.default_rng(seed)
    freq = 1.0 / wavelength_m
    freq_min = freq * 0.9
    freq_max = freq * 1.1

    h = _spectral_band(
        image_size, world_size,
        freq_min=freq_min,
        freq_max=freq_max,
        n_components=n_components,
        amplitude=amplitude_m,
        orientation_spread=1.0,
        rng=rng,
    )

    # Adaptive Gaussian smooth: σ = 0.25× pixels-per-cycle. Removes
    # sub-wavelength pixelation without attenuating the signal.
    px_per_cycle = wavelength_m / world_size * image_size
    sigma = max(0.25 * px_per_cycle, 0.5)
    h = gaussian_filter(h, sigma=sigma)
    h -= h.mean()

    # Rescale so peak amplitude matches the requested amplitude exactly.
    cur_peak = max(abs(h.min()), abs(h.max()))
    if cur_peak > 1e-12:
        h = h * (amplitude_m / cur_peak)

    rms_m  = float(np.std(h))
    peak_m = float(max(abs(h.min()), abs(h.max())))
    return h, rms_m, peak_m


# ── MuJoCo physical-model match ──────────────────────────────────────────────
# Constants matching Centipede_MUJOCO-main/terrain/generator/patch_xml.py:
#   MUJOCO_Z_MAX_FLOOR     — floor on the hfield z_max (5 mm). Even if the
#                            heightmap peak is smaller, MuJoCo never goes below
#                            this for numerical stability in the hfield.
#   MUJOCO_Z_BOTTOM_SKIRT  — hfield "skirt depth" (0.001 m / 1 mm) under the
#                            grid. Visual only; we mirror it on the under-floor.
#   MUJOCO_SPAWN_CLEARANCE — vertical clearance the centipede spawns above the
#                            terrain peak directly under it (0.015 m / 15 mm).
MUJOCO_Z_MAX_FLOOR     = 0.005
MUJOCO_Z_BOTTOM_SKIRT  = 0.001
MUJOCO_SPAWN_CLEARANCE = 0.015


def mujoco_z_max(peak_m):
    """Compute MuJoCo's hfield z_max from peak amplitude (matches
    wavelength_sweep.py: ``z_max = max(2 * peak_m, 0.005)``)."""
    return max(2.0 * float(peak_m), MUJOCO_Z_MAX_FLOOR)


def to_mujoco_terrain(heightmap_m):
    """Apply MuJoCo's PNG-uint8 quantization + hfield z-shift to a
    mean-centered float heightmap.

    MuJoCo's physical pipeline:
      1. heightmap_m (mean ≈ 0, range [-peak, +peak])
      2. heightmap_to_png:
           pixel = round( (h - h_min) / (h_max - h_min) * 255 ).astype(uint8)
      3. <hfield> loads the PNG, normalises to [0, 1], multiplies by z_max:
           h_world = pixel/255 * z_max         where z_max = max(2*peak, 0.005)

    Net effect: heights span [0, z_max] in WORLD z (NOT [-peak, +peak]) AND
    are quantised into 256 levels with step ≈ z_max/255.

    Returns
    -------
    h_world : np.ndarray same shape as heightmap_m, float64
        Heights in metres after the PNG-roundtrip and z_max scaling. The
        terrain mean is now ≈ +peak (shifted up from the input).
    z_max : float
        The hfield z_max that MuJoCo would have used.
    """
    h = np.asarray(heightmap_m, dtype=np.float64)
    h_min = float(h.min())
    h_max = float(h.max())
    span  = h_max - h_min
    peak_m = float(max(abs(h_min), abs(h_max)))
    z_max  = mujoco_z_max(peak_m)
    if span < 1e-15:
        return np.full_like(h, 0.5 * z_max), z_max
    pixel = np.round((h - h_min) / span * 255.0).astype(np.uint8)
    h_world = pixel.astype(np.float64) / 255.0 * z_max
    return h_world, z_max


def mujoco_spawn_z(heightmap_m, sample_radius_m=None, world_size=1.0,
                   clearance=MUJOCO_SPAWN_CLEARANCE):
    """Mirror MuJoCo's spawn_z_from_png: read terrain height at world (0,0)
    (centre of the hfield), take the max within a ±sample_radius patch, and
    add a fixed clearance.

    This is the same recipe Centipede_MUJOCO-main uses to position the
    free-joint body at sim-start so the centipede lands above the highest
    point directly under it. Use it in test_walk.py / optimizer in place of
    the hard-coded translation=(0, 0, 0.02)."""
    h_world, z_max = to_mujoco_terrain(heightmap_m)
    H, W = h_world.shape
    cy, cx = H // 2, W // 2
    # Default sample patch ≈ centipede half-length (~7.5 cm). Convert to
    # pixels via heightmap pixel pitch.
    if sample_radius_m is None:
        sample_radius_m = 0.075
    pix_per_m = H / float(world_size)
    r_pix = max(1, int(round(sample_radius_m * pix_per_m)))
    patch = h_world[max(0, cy - r_pix): cy + r_pix + 1,
                     max(0, cx - r_pix): cx + r_pix + 1]
    return float(patch.max()) + float(clearance)


# ── USD mesh baking ──────────────────────────────────────────────────────────

# GO_CONTACT_COMPLIANCE_v2 (2026-07-10): default-OFF hook. Set COMPLIANT_MATERIAL to a dict of RigidBodyMaterialCfg
# fields (static_friction, dynamic_friction, restitution, friction_combine_mode, restitution_combine_mode,
# compliant_contact_stiffness, compliant_contact_damping) to bind a compliant physics material to the terrain collider.
# Default None => the branch is skipped => the rigid terrain path is byte-identical.
COMPLIANT_MATERIAL = None


def build_terrain_mesh(stage, prim_path, heightmap_m, world_size=1.0,
                       mesh_resolution=1024, mujoco_compat=True):
    """Bake the given heightmap into a USD triangle mesh prim with PhysX
    collision, centred on the world origin in xy.

    Args
    ----
    stage           : USD stage to spawn into.
    prim_path       : USD path of the new mesh (e.g. "/World/Terrain/Mesh").
    heightmap_m     : (H, W) numpy float, heights in metres. Pass the OUTPUT
                      of generate_single_wavelength_heightmap (mean-centered,
                      range [-peak, +peak]); the mujoco_compat flag below
                      will apply the MuJoCo z-shift + quantisation.
    world_size      : Width/length of the terrain in metres (default 1m).
    mesh_resolution : Side length of the collision grid. Default 1024 to
                      match MuJoCo's hfield row/col count exactly. If the
                      input heightmap is smaller (e.g. 1024×1024 already)
                      no upsampling happens; if larger, indices are taken
                      via linspace block-stride.
    mujoco_compat   : If True (default), apply MuJoCo's PNG-uint8 quantize
                      + z-shift so the resulting mesh is PHYSICALLY identical
                      to what MuJoCo's hfield gives. If False, use the raw
                      float heightmap (mesh spans [-peak, +peak] in world z,
                      not [0, 2*peak]).

    Returns
    -------
    UsdGeom.Mesh prim.
    """
    from pxr import UsdGeom, UsdPhysics, Gf, Vt

    # ── Apply MuJoCo PNG-roundtrip + z-shift if requested ──
    if mujoco_compat:
        heightmap_m, z_max_for_log = to_mujoco_terrain(heightmap_m)
    else:
        z_max_for_log = None

    # ── Downsample / select to mesh_resolution × mesh_resolution ──
    H, W = heightmap_m.shape
    if mesh_resolution < H or mesh_resolution < W:
        ys = np.linspace(0, H - 1, mesh_resolution).astype(int)
        xs = np.linspace(0, W - 1, mesh_resolution).astype(int)
        h_grid = heightmap_m[np.ix_(ys, xs)]
    else:
        h_grid = heightmap_m

    n = h_grid.shape[0]   # square assumed
    cell = world_size / float(n - 1)
    half = world_size / 2.0

    # ── Build vertex array (n*n vertices) ──
    # Vectorised — the original double-Python-loop took several seconds at
    # n=1024 (1M verts). Pure numpy reshapes the same in ~50 ms.
    ii, jj = np.meshgrid(np.arange(n), np.arange(n), indexing="xy")
    xs_w = -half + ii * cell
    ys_w = -half + jj * cell
    verts = np.stack([xs_w, ys_w, h_grid], axis=-1).astype(np.float32).reshape(-1, 3)

    # ── Build face arrays: (n-1)*(n-1) quads → 2(n-1)² tris ──
    # Vectorised: at n=1024 the original double-Python-loop took >30 s,
    # while this is ~100 ms. Same triangulation pattern as before:
    # per quad (j,i) take v00, v01, v10, v11 with v?? = j*n + i? and emit
    # two triangles (v00 v01 v11) and (v00 v11 v10).
    n_quads = (n - 1) * (n - 1)
    face_vertex_counts = np.full(n_quads * 2, 3, dtype=np.int32)
    j = np.arange(n - 1)
    i = np.arange(n - 1)
    J, I = np.meshgrid(j, i, indexing="ij")
    v00 = (J      * n + I     ).ravel()
    v01 = (J      * n + I + 1 ).ravel()
    v10 = ((J + 1) * n + I    ).ravel()
    v11 = ((J + 1) * n + I + 1).ravel()
    face_vertex_indices = np.empty(n_quads * 6, dtype=np.int32)
    face_vertex_indices[0::6] = v00
    face_vertex_indices[1::6] = v01
    face_vertex_indices[2::6] = v11
    face_vertex_indices[3::6] = v00
    face_vertex_indices[4::6] = v11
    face_vertex_indices[5::6] = v10

    # ── Define the USD Mesh prim ──
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(verts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(face_vertex_counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(face_vertex_indices))
    # Subdivision off so PhysX uses the polys directly
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)

    # ── Per-face FLAT-SHADED normals ──
    # Without explicit normals USD/Hydra auto-generates smooth (Gouraud)
    # vertex normals, which interpolates across triangles and visually
    # SOFTENS the heightfield bumps — making the rendered terrain look
    # flatter than MuJoCo's flat-shaded hfield viewer at the same z_max.
    # By setting per-face normals (interpolation="uniform") we force the
    # renderer to flat-shade each triangle, matching MuJoCo's look so
    # the visible roughness reflects the actual physical bump amplitude
    # (peak-to-peak = z_max).
    #
    # Compute each triangle's normal from its three vertices via the
    # cross-product of two edges. Vectorised — at n=1024 this is ~50 ms.
    tri_v0 = verts[face_vertex_indices[0::3]]
    tri_v1 = verts[face_vertex_indices[1::3]]
    tri_v2 = verts[face_vertex_indices[2::3]]
    edge1 = tri_v1 - tri_v0
    edge2 = tri_v2 - tri_v0
    tri_normals = np.cross(edge1, edge2)
    norm_mag = np.linalg.norm(tri_normals, axis=1, keepdims=True)
    norm_mag = np.maximum(norm_mag, 1e-12)  # guard against degenerate tris
    tri_normals = (tri_normals / norm_mag).astype(np.float32)
    # Make sure normals point UP (+Z hemisphere) — flip any with -Z.
    flip_mask = tri_normals[:, 2] < 0.0
    tri_normals[flip_mask] *= -1.0

    normals_attr = mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(tri_normals))
    mesh.SetNormalsInterpolation("uniform")  # one normal per face = flat shading

    # ── PhysX collision ──
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    # PhysX cooks the mesh as a triangle-mesh static collider (default).

    # GO_CONTACT_COMPLIANCE_v2: OPTIONAL compliant material bind (default OFF -> skipped -> rigid path byte-identical).
    if COMPLIANT_MATERIAL is not None:
        import isaaclab.sim as _sim_su
        from isaaclab.sim.utils import bind_physics_material as _bpm
        _mat_path = prim_path + "/physicsMaterial"
        _mcfg = _sim_su.RigidBodyMaterialCfg(**COMPLIANT_MATERIAL)
        _mcfg.func(_mat_path, _mcfg)
        _bpm(prim_path, _mat_path)
        print(f"[terrain] COMPLIANT_MATERIAL bound to {prim_path}: "
              f"k_c={COMPLIANT_MATERIAL.get('compliant_contact_stiffness')} "
              f"c_c={COMPLIANT_MATERIAL.get('compliant_contact_damping')}", flush=True)

    return mesh
