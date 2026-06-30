"""offline_render_from_log_2026-06-22.py — THIRD-PARTY OFFLINE frame renderer (CPU, deterministic).

GO_OFFLINE_RENDERER_2026-06-22. Sidesteps the non-deterministic Omniverse RTX black/white-frame
race at the ROOT: physics is run headless and logged (cell --dump-raw-alllink), and this script
renders frame-by-frame OFFLINE from the centipede meshes + the reconstructed terrain.

VIZ ONLY. Reads the logged per-link WORLD pose and places each link's MJCF visual geom there —
NO forward kinematics, NO joint-angle replay, NO forward-direction convention (Rule 3): every link
is placed by its logged body_pos_w/body_quat_w. The numbers still come from logged state.

Inputs
------
  --alllink-npz   <out>_alllink.npz from the cell (body_pos_w/body_quat_w/body_names/meta).
  --mjcf          renamed MJCF (default models/farms/centipede_rebuild_v1_renamed.xml) — source of
                  the body->visual-geom map (mesh+scale / sphere / capsule), so a renamed model maps
                  by NAME, never by index.
Modes
-----
  --frame N + --static-montage   ⛳ §3 mapping gate: one multi-view PNG of frame N (assembly check).
  (default)                      §4 animation: PNG sequence -> MP4 + per-frame color-agnostic audit.

Mapping (verified against the MJCF, GO_OFFLINE_RENDERER §3):
  body_names[i] == 'link_body_N'        -> mesh link_N.obj (×0.05), geom pos=0 => body frame.
  body_names[i] =~ 'link_leg_M_[LR]_K'  -> sphere (K in {0,1}) / capsule (K in {2,3}) per MJCF.
  body_names[i] =~ 'foot_M_S'           -> sphere.
  body_names[i] =~ 'link_pitch_body_*'  -> massless joint body, no geom -> skipped.
"""
from __future__ import annotations
import argparse, os, sys, json, math, re
import xml.etree.ElementTree as ET
import numpy as np

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, os.pardir, os.pardir))
for p in (PROJECT_ROOT, os.path.join(PROJECT_ROOT, "scripts", "optimization")):
    if p not in sys.path:
        sys.path.insert(0, p)

import trimesh                      # noqa: E402
import pyvista as pv                # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

pv.OFF_SCREEN = True

# ---- color categories (tunable) — body dark red / legs light tan / feet slate / eyes black ----
CAT_COLORS = {
    "body": (0.62, 0.10, 0.10),
    "leg":  (0.82, 0.74, 0.66),
    "foot": (0.28, 0.30, 0.38),
    "eye":  (0.02, 0.02, 0.02),
}


def quat_wxyz_to_R(q):
    """(...,4) wxyz -> (...,3,3) rotation. Robust to non-unit (normalises)."""
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    q = q / np.maximum(n, 1e-12)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3), dtype=np.float64)
    R[..., 0, 0] = 1 - 2 * (y * y + z * z); R[..., 0, 1] = 2 * (x * y - w * z); R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z); R[..., 1, 1] = 1 - 2 * (x * x + z * z); R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y); R[..., 2, 1] = 2 * (y * z + w * x); R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def _floats(s):
    return [float(v) for v in s.replace(",", " ").split()]


def category_of(body_name, geom_name):
    if geom_name and "eye" in geom_name:
        return "eye"
    if body_name.startswith("link_body"):
        return "body"
    if body_name.startswith("link_leg"):
        return "leg"
    if body_name.startswith("foot"):
        return "foot"
    return "body"


def parse_mjcf(mjcf_path, mesh_dir, mesh_scale_default=1.0, prim_subdiv=1, cyl_sections=10):
    """Return body_name -> list of local trimesh geoms (visual group=='2' only), each a trimesh.Trimesh
    expressed in the BODY frame with .metadata['cat'] set. Meshes loaded+scaled+cached."""
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    # assets: mesh name -> (abs file, scale (3,))
    assets = {}
    for m in root.iter("mesh"):
        nm = m.get("name")
        f = m.get("file")
        sc = _floats(m.get("scale", "1 1 1"))
        if len(sc) == 1:
            sc = sc * 3
        assets[nm] = (os.path.join(mesh_dir, f), np.array(sc, dtype=np.float64))
    mesh_cache = {}

    def load_mesh(mesh_name):
        if mesh_name in mesh_cache:
            return mesh_cache[mesh_name]
        f, sc = assets[mesh_name]
        g = trimesh.load(f, process=False)
        if isinstance(g, trimesh.Scene):
            g = g.dump(concatenate=True)
        v = np.asarray(g.vertices, dtype=np.float64) * sc[None, :]
        m = trimesh.Trimesh(vertices=v, faces=np.asarray(g.faces), process=False)
        mesh_cache[mesh_name] = m
        return m

    def geom_to_mesh(g):
        gtype = g.get("type", "sphere")
        pos = np.array(_floats(g.get("pos", "0 0 0")), dtype=np.float64)
        quat = g.get("quat")
        Rl = quat_wxyz_to_R(_floats(quat)) if quat else np.eye(3)
        if gtype == "mesh":
            m = load_mesh(g.get("mesh")).copy()
        elif gtype == "sphere":
            r = float(_floats(g.get("size", "0.001"))[0])
            m = trimesh.creation.icosphere(subdivisions=prim_subdiv, radius=r)
        elif gtype == "capsule":
            ft = g.get("fromto")
            r = float(_floats(g.get("size", "0.001"))[0])
            if ft is not None:
                p = _floats(ft)
                p0 = np.array(p[0:3]); p1 = np.array(p[3:6])
                seg = float(np.linalg.norm(p1 - p0))
                parts = [trimesh.creation.icosphere(subdivisions=prim_subdiv, radius=r)]
                parts[0].apply_translation(p0)
                s1 = trimesh.creation.icosphere(subdivisions=prim_subdiv, radius=r); s1.apply_translation(p1)
                parts.append(s1)
                if seg > 1e-9:
                    cyl = trimesh.creation.cylinder(radius=r, segment=np.array([p0, p1]), sections=cyl_sections)
                    parts.append(cyl)
                m = trimesh.util.concatenate(parts)
                pos = np.zeros(3); Rl = np.eye(3)  # fromto is already in body frame
            else:
                size = _floats(g.get("size", "0.001 0.001"))
                r = size[0]; half = size[1] if len(size) > 1 else r
                m = trimesh.creation.capsule(height=2 * half, radius=r, count=[cyl_sections, cyl_sections])
        elif gtype in ("box",):
            size = _floats(g.get("size", "0.001 0.001 0.001"))
            m = trimesh.creation.box(extents=np.array(size) * 2.0)
        else:
            r = float(_floats(g.get("size", "0.001"))[0])
            m = trimesh.creation.icosphere(subdivisions=prim_subdiv, radius=r)
        # apply geom-local transform (mesh/sphere/box; capsule already body-frame)
        T = np.eye(4); T[:3, :3] = Rl; T[:3, 3] = pos
        m.apply_transform(T)
        return m

    body_geoms = {}

    def walk(body_el):
        bname = body_el.get("name")
        geoms = []
        for child in body_el:
            if child.tag == "geom" and child.get("group") == "2":
                m = geom_to_mesh(child)
                m.metadata["cat"] = category_of(bname, child.get("name"))
                geoms.append(m)
        if bname is not None:
            body_geoms[bname] = geoms
        for child in body_el:
            if child.tag == "body":
                walk(child)

    wb = root.find("worldbody")
    for b in wb.findall("body"):
        walk(b)
    return body_geoms, list(mesh_cache.keys()), assets


def assemble_frame(body_names, pos_f, quat_f, body_geoms):
    """Transform every link's local geoms by its logged world pose -> per-category (verts, faces).
    pos_f (nb,3), quat_f (nb,4 wxyz). Returns {cat: (V (n,3), F (m,3))}, plus a coverage report."""
    R_f = quat_wxyz_to_R(quat_f)  # (nb,3,3)
    buckets = {c: ([], [], 0) for c in CAT_COLORS}  # cat -> (vlist, flist, voffset)
    placed, skipped, unknown = 0, 0, []
    for i, bn in enumerate(body_names):
        geoms = body_geoms.get(bn)
        if geoms is None:
            unknown.append(bn); continue
        if not geoms:
            skipped += 1; continue   # massless joint body (no visual geom)
        Ri = R_f[i]; pi = pos_f[i]
        for m in geoms:
            cat = m.metadata.get("cat", "body")
            vw = (Ri @ np.asarray(m.vertices).T).T + pi[None, :]
            vlist, flist, off = buckets[cat]
            vlist.append(vw)
            flist.append(np.asarray(m.faces) + off)
            buckets[cat] = (vlist, flist, off + len(vw))
        placed += 1
    out = {}
    for cat, (vlist, flist, off) in buckets.items():
        if vlist:
            out[cat] = (np.concatenate(vlist, 0), np.concatenate(flist, 0))
    return out, dict(placed=placed, skipped=skipped, unknown=unknown)


def faces_to_vtk(F):
    F = np.asarray(F)
    return np.hstack([np.full((len(F), 1), 3, dtype=np.int64), F.astype(np.int64)]).ravel()


def add_cat_meshes(pl, cat_meshes):
    for cat, (V, F) in cat_meshes.items():
        poly = pv.PolyData(V, faces_to_vtk(F))
        pl.add_mesh(poly, color=CAT_COLORS[cat], smooth_shading=True,
                    specular=0.2, diffuse=0.9, ambient=0.25)


def robot_bounds(cat_meshes):
    allv = np.concatenate([V for (V, F) in cat_meshes.values()], 0)
    return allv.min(0), allv.max(0)


# ---------- terrain ----------
def build_terrain(meta, com_xy, max_grid=384, return_arrays=False):
    """Reconstruct the env terrain surface in WORLD coords. MUST match the env convention
    (arena.heightmap_value_at:483-490 + terrain.build_terrain_mesh:256-259):
    heightmap[row, col] sits at world (x <- COL, y <- ROW), no axis flip. So meshgrid
    indexing='xy' => Xg[i,j]=xs[j] (col->X), Yg[i,j]=ys[i] (row->Y). The earlier indexing='ij'
    placed row->X => ridges TRANSPOSED 90deg (GO_OFFLINE_RENDERER_TERRAIN_FIX).
    WORLD-Z (GO_OFFLINE_RENDERER_ZFIX): the robot WALKS ON build_terrain_mesh's mesh, which the env
    builds with mujoco_compat=True => terrain.to_mujoco_terrain(h) (PNG-uint8 quantize + z-shift =>
    world span [0, z_max], mean ~ +peak). The RAW mean-centred h (== arena.heightmap_value_at) is
    ~peak too LOW => robot floated ~17mm. So apply to_mujoco_terrain (the env's OWN fn) here too."""
    from terrain import generate_single_wavelength_heightmap, to_mujoco_terrain
    lam_mm = float(meta["lambda_mm"]); amp_mm = float(meta["amp_mm"])
    seed = int(meta["terrain_seed"]); arena = float(meta["arena_size_m"])
    img = int(meta["image_size"])
    img = min(img, max_grid)  # render-resolution grid (physics used full res; viz needs less)
    if amp_mm <= 0.0:
        h_raw = np.zeros((img, img), dtype=np.float64)
    else:
        h_raw, _r, _p = generate_single_wavelength_heightmap(
            wavelength_m=lam_mm * 1e-3, amplitude_m=amp_mm * 1e-3, seed=seed,
            image_size=img, world_size=arena)
    h, _zmax = to_mujoco_terrain(h_raw)   # WORLD-Z of the WALKED mesh (matches build_terrain_mesh)
    xs = np.linspace(-arena / 2, arena / 2, img)        # world X along COLUMN axis
    ys = np.linspace(-arena / 2, arena / 2, img)        # world Y along ROW axis
    Xg, Yg = np.meshgrid(xs + com_xy[0], ys + com_xy[1], indexing="xy")  # Xg[i,j]=xs[j], Yg[i,j]=ys[i]
    grid = pv.StructuredGrid(Xg, Yg, h)
    if return_arrays:
        return grid, dict(xs=xs, ys=ys, h=h, h_raw=h_raw, arena=arena, com_xy=np.asarray(com_xy, float))
    return grid


# ---------- HUD (computed from the SAME logged poses) ----------
def spine_indices(body_names):
    idx, n = [], 0
    while f"link_body_{n}" in list(body_names):
        idx.append(list(body_names).index(f"link_body_{n}")); n += 1
    return idx


def compute_hud_series(npz, e):
    bn = [str(x) for x in npz["body_names"]]
    sid = spine_indices(bn)
    pos = npz["body_pos_w"][:, e, :, :]      # (T,nb,3)
    quat = npz["body_quat_w"][:, e, :, :]    # (T,nb,4)
    alive = npz["alive"][:, e]
    T = pos.shape[0]
    sxy = pos[:, sid, :2]                      # (T,S,2)
    # body-PCA heading (head-wag-robust), unwrapped net yaw
    x = sxy[..., 0]; y = sxy[..., 1]
    xm = x - x.mean(1, keepdims=True); ym = y - y.mean(1, keepdims=True)
    a = (xm * xm).mean(1); c = (ym * ym).mean(1); b = (xm * ym).mean(1)
    th = 0.5 * np.arctan2(2 * b, a - c)
    vx = np.cos(th); vy = np.sin(th)
    htx = sxy[:, 0, 0] - sxy[:, -1, 0]; hty = sxy[:, 0, 1] - sxy[:, -1, 1]
    flip = (vx * htx + vy * hty) < 0
    vx = np.where(flip, -vx, vx); vy = np.where(flip, -vy, vy)
    heading = np.degrees(np.arctan2(vy, vx))
    hu = np.degrees(np.unwrap(np.radians(heading)))
    net_yaw = hu - hu[0]
    # body-median roll from spine quats
    sq = quat[:, sid, :]
    w, qx, qy, qz = sq[..., 0], sq[..., 1], sq[..., 2], sq[..., 3]
    roll = np.degrees(np.median(np.arctan2(2 * (w * qx + qy * qz), 1 - 2 * (qx**2 + qy**2)), axis=1))
    # forward = world -x of spine COM
    xcom = sxy[..., 0].mean(1)
    dt = float(npz["dt_pol"])
    t = np.arange(T) * dt
    fwd = -(xcom - xcom[0])
    vfwd = np.zeros(T)
    vfwd[1:] = (fwd[1:]) / np.maximum(t[1:], 1e-6)
    wave = float(npz["force_wave_abs"]) if np.isfinite(npz["force_wave_abs"]) else float("nan")
    return dict(net_yaw=net_yaw, roll=roll, vfwd=vfwd, fwd=fwd, alive=alive, wave=wave, t=t,
                xcom=xcom, ycom=sxy[..., 1].mean(1))


def draw_hud(png_path, lines):
    im = Image.open(png_path).convert("RGB")
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("arial.ttf", 16)
    except Exception:
        font = ImageFont.load_default()
    y = 6
    for ln in lines:
        d.text((9, y + 1), ln, fill=(0, 0, 0), font=font)
        d.text((8, y), ln, fill=(240, 240, 80), font=font)
        y += 19
    im.save(png_path)


# ---------- per-frame audit: ALL failure modes (GO_OFFLINE_RENDERER_ZFIX §4) ----------
# Catches, beyond all-black/white/blank: GRADIENT/PARTIAL-white (per-quadrant pure-white SATURATION
# imbalance — a half-white frame saturates one region while another stays normal), STROBE (temporal
# luminance jump), per-frame OUTLIER (deviation from clip median), and ROBOT-ABSENCE (red-channel blob
# each frame — catches a terrain-only frame). Saturation = L>250 (near pure white); the renderer uses a
# NON-saturating light-sky bg so pure white only appears on a render failure. Color-agnostic on black.
def extended_audit_arrays(frames_rgb):
    L = []; sat_q = []; redblob = []; blank = []
    prev = None; motion = []
    for f in frames_rgb:
        rgb = np.asarray(f)[..., :3].astype(np.float32)
        R, G, Bc = rgb[..., 0], rgb[..., 1], rgb[..., 2]
        lum = 0.299 * R + 0.587 * G + 0.114 * Bc
        L.append(float(lum.mean())); blank.append(1.0 if lum.std() < 6 else 0.0)
        H, W = lum.shape
        sat = (lum > 250)
        quads = [sat[:H // 2, :W // 2], sat[:H // 2, W // 2:], sat[H // 2:, :W // 2], sat[H // 2:, W // 2:]]
        qf = [float(q.mean()) for q in quads]
        sat_q.append(max(qf) - min(qf))                       # pure-white saturation IMBALANCE
        redblob.append(float(((R > G + 25) & (R > Bc + 25) & (R > 50)).mean()))  # red robot presence
        if prev is not None:
            motion.append(float(np.abs(lum - prev).mean()))
        prev = lum
    L = np.array(L); med = float(np.median(L)); mad = float(np.median(np.abs(L - med))) + 1e-6
    dj = np.abs(np.diff(L)) if len(L) > 1 else np.array([0.0])
    res = dict(
        n=len(L),
        black_frac=float((L < 8).mean()),
        white_frac=float((L > 245).mean()),
        blank_frac=float(np.mean(blank)),
        partial_white_frac=float(np.mean(np.array(sat_q) > 0.30)),   # localized blow-out
        strobe_frac=float((dj > 25).mean()), max_frame_djump=float(dj.max()),
        outlier_frac=float((np.abs(L - med) > max(40.0, 6 * mad)).mean()),
        min_red_blob_frac=float(np.min(redblob)),
        motion=float(np.mean(motion)) if motion else 0.0,
        mean_lum=float(L.mean()), min_mean=float(L.min()), max_mean=float(L.max()))
    checks = dict(
        no_black=res["black_frac"] == 0.0, no_white=res["white_frac"] == 0.0,
        no_blank=res["blank_frac"] == 0.0, no_partial_white=res["partial_white_frac"] == 0.0,
        no_strobe=res["strobe_frac"] == 0.0, no_outlier=res["outlier_frac"] == 0.0,
        robot_present=res["min_red_blob_frac"] > 0.001, has_motion=res["motion"] > 0.0)
    res["checks"] = checks
    res["PASS"] = bool(all(checks.values()))
    return res


def audit_frames(frame_paths):
    return extended_audit_arrays([np.asarray(Image.open(p).convert("RGB")) for p in frame_paths])


def set_camera(pl, mode, lo, hi, offset_dir, dist_k, abs_offset=None):
    com = (lo + hi) / 2.0
    rad = float(np.linalg.norm(hi - lo)) / 2.0 + 1e-9
    if abs_offset is not None and mode == "chase":
        # absolute world-space chase offset (match an Omniverse chase clip exactly)
        pos = com + np.asarray(abs_offset, dtype=np.float64)
        pl.camera_position = [tuple(pos), tuple(com), (0, 0, 1)]
        pl.camera.SetViewUp(0, 0, 1)
        return
    if mode == "side":
        d = np.array([0.0, -1.0, 0.15])
    elif mode == "front":
        d = np.array([-1.0, 0.0, 0.15])
    elif mode == "top":
        d = np.array([0.0, 0.0, 1.0])
    else:  # chase / persp
        d = np.array(offset_dir, dtype=np.float64)
    d = d / np.linalg.norm(d)
    pos = com + d * (dist_k * max(rad, 1e-6))
    up = (0, 0, 1) if mode != "top" else (1, 0, 0)
    pl.camera_position = [tuple(pos), tuple(com), up]
    pl.camera.SetViewUp(*up)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alllink-npz", required=True)
    ap.add_argument("--metrics-json", default=None, help="optional cell metrics json (provenance/cross-check)")
    ap.add_argument("--mjcf", default=os.path.join(PROJECT_ROOT, "models", "farms", "centipede_rebuild_v1_renamed.xml"))
    ap.add_argument("--mesh-dir", default=os.path.join(PROJECT_ROOT, "models", "farms", "meshes"))
    ap.add_argument("--env-index", type=int, default=0)
    ap.add_argument("--out-mp4", default=None)
    ap.add_argument("--frames-dir", default=None)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=384)
    ap.add_argument("--camera", default="chase", choices=["chase", "side", "front", "top"])
    ap.add_argument("--cam-offset", default="-1,-0.8,0.6", help="chase dir (x,y,z), world; forward=-x")
    ap.add_argument("--cam-dist-k", type=float, default=2.2)
    ap.add_argument("--cam-abs-offset", default=None,
                    help="absolute world chase offset 'x,y,z' (target+offset, no scaling) to match an "
                         "Omniverse chase clip; overrides --cam-offset/--cam-dist-k for chase mode")
    ap.add_argument("--chase-target", default="head", choices=["head", "com"],
                    help="abs-offset chase look-at: 'head'=link_body_0 (matches Omniverse root chase) or 'com'")
    ap.add_argument("--chase-alpha", type=float, default=0.85,
                    help="abs-offset chase target EMA smoothing (matches Omniverse --chase-alpha)")
    ap.add_argument("--terrain", action="store_true", help="reconstruct + show the rough heightmap")
    ap.add_argument("--terrain-offset", default="0,0", help="x,y shift of terrain centre (alignment tweak)")
    ap.add_argument("--no-hud", action="store_true")
    ap.add_argument("--frame", type=int, default=None, help="render a single frame index")
    ap.add_argument("--static-montage", action="store_true", help="single frame, 2x2 multi-view (mapping gate)")
    ap.add_argument("--out-png", default=None, help="for --frame/--static-montage")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--report-json", default=None)
    args = ap.parse_args()

    npz = np.load(args.alllink_npz, allow_pickle=True)
    body_names = [str(x) for x in npz["body_names"]]
    e = int(args.env_index)
    Tn = npz["body_pos_w"].shape[0]
    nb = npz["body_pos_w"].shape[2]
    print(f"[render] npz {os.path.basename(args.alllink_npz)}: T={Tn} envs={npz['body_pos_w'].shape[1]} nb={nb} env={e}", flush=True)

    body_geoms, meshes_loaded, assets = parse_mjcf(args.mjcf, args.mesh_dir)
    # coverage report (mapping audit) — use frame 0
    pos0 = npz["body_pos_w"][0, e]; quat0 = npz["body_quat_w"][0, e]
    cat0, cover = assemble_frame(body_names, pos0, quat0, body_geoms)
    n_with_geom = sum(1 for bn in body_names if body_geoms.get(bn))
    print(f"[render] MJCF bodies={len(body_geoms)} meshes_loaded={len(meshes_loaded)}", flush=True)
    print(f"[render] mapping: nb_dump={nb} bodies_with_visual_geom={n_with_geom} "
          f"placed={cover['placed']} massless_skipped={cover['skipped']} "
          f"UNKNOWN(no MJCF match)={len(cover['unknown'])}", flush=True)
    if cover["unknown"]:
        print("[render] UNKNOWN body_names (first 10): " + ", ".join(cover["unknown"][:10]), flush=True)
    lo0, hi0 = robot_bounds(cat0)
    print(f"[render] frame0 robot bounds lo={np.round(lo0,4).tolist()} hi={np.round(hi0,4).tolist()} "
          f"span={np.round(hi0-lo0,4).tolist()}", flush=True)

    offdir = _floats(args.cam_offset)
    abs_off = _floats(args.cam_abs_offset) if args.cam_abs_offset else None
    toff = _floats(args.terrain_offset)
    meta = {k: npz[k] for k in ("lambda_mm", "amp_mm", "terrain_seed", "arena_size_m", "image_size")}

    def render_one(fi, pl, with_terrain, terrain_grid=None):
        pos_f = npz["body_pos_w"][fi, e]; quat_f = npz["body_quat_w"][fi, e]
        cat_m, _cov = assemble_frame(body_names, pos_f, quat_f, body_geoms)
        add_cat_meshes(pl, cat_m)
        lo, hi = robot_bounds(cat_m)
        com = (lo + hi) / 2.0
        if with_terrain:
            # terrain is WORLD-FIXED (built once) so the robot visibly traverses it; if not
            # supplied (static single frame) build it under the current com.
            grid = terrain_grid if terrain_grid is not None else \
                build_terrain(meta, np.array([com[0] + toff[0], com[1] + toff[1]]))
            pl.add_mesh(grid, color=(0.55, 0.52, 0.48), opacity=1.0, smooth_shading=True,
                        specular=0.05, ambient=0.3)
        return lo, hi

    def fixed_terrain_grid():
        """world-fixed terrain at the ENV ARENA CENTRE = world origin (the env builds
        TransitionTerrain arena-centred at (0,0); spawn-z uses arena.heightmap_value_at
        arena-centred). Centring on the robot COM would shift the robot onto the wrong ridges.
        --terrain-offset nudges if a given env's arena origin differs."""
        return build_terrain(meta, np.array([toff[0], toff[1]]))

    report = dict(alllink=os.path.basename(args.alllink_npz), env=e, T=Tn, nb=nb,
                  mapping=dict(bodies_with_geom=n_with_geom, placed=cover["placed"],
                               massless_skipped=cover["skipped"], unknown=cover["unknown"],
                               meshes_loaded=sorted(meshes_loaded)),
                  frame0_bounds=dict(lo=lo0.tolist(), hi=hi0.tolist()))

    # ---------- STATIC MONTAGE (mapping gate) ----------
    if args.static_montage or args.frame is not None:
        fi = int(args.frame if args.frame is not None else 0)
        out_png = args.out_png or os.path.splitext(args.alllink_npz)[0] + f"_frame{fi}.png"
        if args.static_montage:
            views = ["chase", "side", "top", "front"]
            tiles = []
            for v in views:
                pl = pv.Plotter(off_screen=True, window_size=(args.width, args.height))
                pl.set_background((0.80, 0.855, 0.92))  # light sky-gray (non-saturating: L~213<250 => pure white = failure only)
                lo, hi = render_one(fi, pl, args.terrain)
                set_camera(pl, v, lo, hi, offdir, args.cam_dist_k)
                pl.add_text(v, font_size=10, color="black")
                img = pl.screenshot(return_img=True); pl.close()
                tiles.append(np.asarray(img))
            top = np.concatenate([tiles[0], tiles[1]], axis=1)
            bot = np.concatenate([tiles[2], tiles[3]], axis=1)
            Image.fromarray(np.concatenate([top, bot], axis=0)).save(out_png)
        else:
            pl = pv.Plotter(off_screen=True, window_size=(args.width, args.height))
            pl.set_background((0.80, 0.855, 0.92))  # light sky-gray (non-saturating: L~213<250 => pure white = failure only)
            lo, hi = render_one(fi, pl, args.terrain)
            set_camera(pl, args.camera, lo, hi, offdir, args.cam_dist_k, abs_off)
            pl.screenshot(out_png); pl.close()
        report["static_png"] = out_png
        print(f"[render] wrote static -> {out_png}", flush=True)
        if args.report_json:
            with open(args.report_json, "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2)
        print("[render] report -> done", flush=True)
        return 0

    # ---------- ANIMATION -> MP4 ----------
    hud = compute_hud_series(npz, e) if not args.no_hud else None
    n_alive = int(npz["alive"][:, e].sum())
    n_render = min(Tn, args.max_frames) if args.max_frames else Tn
    frames_dir = args.frames_dir or (os.path.splitext(args.alllink_npz)[0] + f"_env{e}_frames")
    os.makedirs(frames_dir, exist_ok=True)
    tgrid = fixed_terrain_grid() if args.terrain else None
    head_idx = body_names.index("link_body_0") if "link_body_0" in body_names else None
    sm_tgt = None  # EMA-smoothed chase target (matches Omniverse)
    frame_paths = []
    for fi in range(n_render):
        pl = pv.Plotter(off_screen=True, window_size=(args.width, args.height))
        pl.set_background((0.80, 0.855, 0.92))  # light sky-gray (non-saturating: L~213<250 => pure white = failure only)
        lo, hi = render_one(fi, pl, args.terrain, tgrid)
        if abs_off is not None and args.camera == "chase":
            com = (lo + hi) / 2.0
            raw = (npz["body_pos_w"][fi, e, head_idx].astype(np.float64)
                   if (args.chase_target == "head" and head_idx is not None) else com)
            sm_tgt = raw if sm_tgt is None else args.chase_alpha * sm_tgt + (1 - args.chase_alpha) * raw
            pos = sm_tgt + np.asarray(abs_off, dtype=np.float64)
            pl.camera_position = [tuple(pos), tuple(sm_tgt), (0, 0, 1)]
            pl.camera.SetViewUp(0, 0, 1)
        else:
            set_camera(pl, args.camera, lo, hi, offdir, args.cam_dist_k, abs_off)
        fp = os.path.join(frames_dir, f"f{fi:04d}.png")
        pl.screenshot(fp); pl.close()
        if hud is not None:
            alive_f = bool(hud["alive"][fi])
            wv = hud["wave"]
            lines = [
                f"frame {fi:3d}/{n_render-1}  t={hud['t'][fi]:.2f}s  {'ALIVE' if alive_f else 'done'}",
                f"net_yaw {hud['net_yaw'][fi]:+6.1f} deg   roll {hud['roll'][fi]:+5.1f} deg",
                f"v_fwd {hud['vfwd'][fi]*1000:+6.1f} mm/s   fwd {hud['fwd'][fi]*1000:+6.1f} mm",
                f"wave_abs {wv:.2f}" if wv == wv else "wave_abs op(1.0x)",
            ]
            draw_hud(fp, lines)
        frame_paths.append(fp)
        if fi % 25 == 0:
            print(f"[render] frame {fi}/{n_render}", flush=True)

    aud = audit_frames(frame_paths)
    report["frames"] = n_render; report["audit"] = aud
    print(f"[render] AUDIT: {json.dumps(aud)}", flush=True)

    out_mp4 = args.out_mp4 or (os.path.splitext(args.alllink_npz)[0] + f"_env{e}.mp4")
    import imageio.v2 as imageio
    with imageio.get_writer(out_mp4, fps=args.fps, codec="libx264", quality=8,
                            macro_block_size=None) as w:
        for fp in frame_paths:
            w.append_data(imageio.imread(fp))
    report["mp4"] = out_mp4
    print(f"[render] wrote MP4 -> {out_mp4}  ({os.path.getsize(out_mp4)} bytes)", flush=True)

    if hud is not None:
        report["hud_final"] = dict(net_yaw_deg=float(hud["net_yaw"][min(n_alive, n_render)-1]),
                                   roll_deg=float(hud["roll"][min(n_alive, n_render)-1]),
                                   vfwd_mm_s=float(hud["vfwd"][min(n_alive, n_render)-1]*1000),
                                   wave_abs=float(hud["wave"]) if hud["wave"] == hud["wave"] else None,
                                   n_alive=n_alive)
    if args.report_json:
        with open(args.report_json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
    print("[render] report -> done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
