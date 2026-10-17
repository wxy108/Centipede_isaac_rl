"""Render an exact-pose verification video without starting Isaac Sim or RTX."""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import pyvista as pv
import trimesh


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
DEFAULT_MJCF = ROOT / "assets" / "offline_geometry" / "centipede_rebuild_v1_renamed.xml"
DEFAULT_MESH_DIR = ROOT / "assets" / "offline_geometry" / "meshes"
COLORS = {
    "body": (0.62, 0.10, 0.10),
    "leg": (0.82, 0.74, 0.66),
    "foot": (0.28, 0.30, 0.38),
    "eye": (0.02, 0.02, 0.02),
}

pv.OFF_SCREEN = True


def _floats(text):
    return [float(value) for value in text.replace(",", " ").split()]


def _quat_wxyz_to_matrix(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float64)
    quaternion /= np.maximum(np.linalg.norm(quaternion, axis=-1, keepdims=True), 1e-12)
    w, x, y, z = (quaternion[..., index] for index in range(4))
    matrix = np.empty(quaternion.shape[:-1] + (3, 3), dtype=np.float64)
    matrix[..., 0, 0] = 1 - 2 * (y * y + z * z)
    matrix[..., 0, 1] = 2 * (x * y - w * z)
    matrix[..., 0, 2] = 2 * (x * z + w * y)
    matrix[..., 1, 0] = 2 * (x * y + w * z)
    matrix[..., 1, 1] = 1 - 2 * (x * x + z * z)
    matrix[..., 1, 2] = 2 * (y * z - w * x)
    matrix[..., 2, 0] = 2 * (x * z - w * y)
    matrix[..., 2, 1] = 2 * (y * z + w * x)
    matrix[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return matrix


def _category(body_name, geom_name):
    if geom_name and "eye" in geom_name.lower():
        return "eye"
    if body_name.startswith("link_body"):
        return "body"
    if body_name.startswith("link_leg"):
        return "leg"
    if body_name.startswith("foot"):
        return "foot"
    return "body"


def _parse_visual_geometry(mjcf_path, mesh_dir, group="2"):
    root = ET.parse(mjcf_path).getroot()
    assets = {}
    for element in root.iter("mesh"):
        scale = _floats(element.get("scale", "1 1 1"))
        if len(scale) == 1:
            scale *= 3
        assets[element.get("name")] = (mesh_dir / element.get("file"), np.asarray(scale))
    mesh_cache = {}

    def load_mesh(name):
        if name not in mesh_cache:
            path, scale = assets[name]
            mesh = trimesh.load(path, process=False)
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.dump(concatenate=True)
            mesh = trimesh.Trimesh(
                vertices=np.asarray(mesh.vertices, dtype=np.float64) * scale[None, :],
                faces=np.asarray(mesh.faces),
                process=False,
            )
            mesh_cache[name] = mesh
        return mesh_cache[name].copy()

    def geom_mesh(element):
        geom_type = element.get("type", "sphere")
        position = np.asarray(_floats(element.get("pos", "0 0 0")), dtype=np.float64)
        quaternion = element.get("quat")
        rotation = _quat_wxyz_to_matrix(_floats(quaternion)) if quaternion else np.eye(3)
        if geom_type == "mesh":
            mesh = load_mesh(element.get("mesh"))
        elif geom_type == "sphere":
            radius = _floats(element.get("size", "0.001"))[0]
            mesh = trimesh.creation.icosphere(subdivisions=1, radius=radius)
        elif geom_type == "capsule" and element.get("fromto"):
            values = _floats(element.get("fromto"))
            start, end = np.asarray(values[:3]), np.asarray(values[3:])
            radius = _floats(element.get("size", "0.001"))[0]
            parts = [trimesh.creation.icosphere(subdivisions=1, radius=radius)]
            parts[0].apply_translation(start)
            end_cap = trimesh.creation.icosphere(subdivisions=1, radius=radius)
            end_cap.apply_translation(end)
            parts.append(end_cap)
            if np.linalg.norm(end - start) > 1e-9:
                parts.append(trimesh.creation.cylinder(radius=radius, segment=[start, end], sections=10))
            mesh = trimesh.util.concatenate(parts)
            position, rotation = np.zeros(3), np.eye(3)
        elif geom_type == "box":
            mesh = trimesh.creation.box(extents=2 * np.asarray(_floats(element.get("size"))))
        else:
            size = _floats(element.get("size", "0.001 0.001"))
            mesh = trimesh.creation.capsule(height=2 * size[-1], radius=size[0], count=[10, 10])
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = position
        mesh.apply_transform(transform)
        return mesh

    bodies = {}

    def walk(body):
        name = body.get("name")
        geoms = []
        for child in body:
            if child.tag == "geom" and child.get("group") == group:
                mesh = geom_mesh(child)
                mesh.metadata["category"] = _category(name, child.get("name"))
                geoms.append(mesh)
        if name:
            bodies[name] = geoms
        for child in body:
            if child.tag == "body":
                walk(child)

    for body in root.find("worldbody").findall("body"):
        walk(body)
    return bodies


def _assemble(body_names, positions, quaternions, body_geometries):
    rotations = _quat_wxyz_to_matrix(quaternions)
    buckets = {category: [[], [], 0] for category in COLORS}
    unknown = []
    for index, body_name in enumerate(body_names):
        geoms = body_geometries.get(body_name)
        if geoms is None:
            if "__split" not in body_name:
                unknown.append(body_name)
            continue
        for geom in geoms:
            category = geom.metadata.get("category", "body")
            vertices = (rotations[index] @ np.asarray(geom.vertices).T).T + positions[index]
            vertex_parts, face_parts, offset = buckets[category]
            vertex_parts.append(vertices)
            face_parts.append(np.asarray(geom.faces, dtype=np.int64) + offset)
            buckets[category][2] += len(vertices)
    assembled = {}
    for category, (vertices, faces, _) in buckets.items():
        if vertices:
            assembled[category] = (np.concatenate(vertices), np.concatenate(faces))
    return assembled, sorted(set(unknown))


def _vtk_faces(faces):
    faces = np.asarray(faces, dtype=np.int64)
    return np.column_stack((np.full(len(faces), 3, dtype=np.int64), faces)).ravel()


def _terrain_grid(trace, main_positions):
    from centipede.terrain import homogeneous_terrain, to_mujoco_collision

    arena = float(trace["arena_size_m"])
    image_size = int(trace["image_size"])
    source = homogeneous_terrain(
        float(trace["lambda_mm"]), float(trace["amp_mm"]), int(trace["terrain_seed"])
    )
    height = to_mujoco_collision(source)
    margin = 0.30
    x_low, y_low = main_positions[..., :2].min(axis=(0, 1)) - margin
    x_high, y_high = main_positions[..., :2].max(axis=(0, 1)) + margin
    x_low, y_low = np.maximum([x_low, y_low], -arena / 2)
    x_high, y_high = np.minimum([x_high, y_high], arena / 2)

    def pixel(value):
        return int(np.clip(round((value + arena / 2) / arena * (image_size - 1)), 0, image_size - 1))

    x0, x1, y0, y1 = pixel(x_low), pixel(x_high), pixel(y_low), pixel(y_high)
    x1, y1 = max(x1, x0 + 1), max(y1, y0 + 1)
    xs = np.linspace(-arena / 2, arena / 2, image_size)[x0 : x1 + 1]
    ys = np.linspace(-arena / 2, arena / 2, image_size)[y0 : y1 + 1]
    x_grid, y_grid = np.meshgrid(xs, ys, indexing="xy")
    return pv.StructuredGrid(x_grid, y_grid, height[y0 : y1 + 1, x0 : x1 + 1])


class FrameAudit:
    def __init__(self):
        self.means = []
        self.blank = []
        self.partial_white = []
        self.red_fraction = []
        self.motion = []
        self.previous_luminance = None

    def add(self, frame):
        rgb = np.asarray(frame)[..., :3].astype(np.float32)
        red, green, blue = rgb[..., 0], rgb[..., 1], rgb[..., 2]
        luminance = 0.299 * red + 0.587 * green + 0.114 * blue
        self.means.append(float(luminance.mean()))
        self.blank.append(float(luminance.std() < 6.0))
        height, width = luminance.shape
        saturated = luminance > 250
        quadrants = (
            saturated[: height // 2, : width // 2],
            saturated[: height // 2, width // 2 :],
            saturated[height // 2 :, : width // 2],
            saturated[height // 2 :, width // 2 :],
        )
        fractions = [float(quadrant.mean()) for quadrant in quadrants]
        self.partial_white.append(float(max(fractions) - min(fractions) > 0.30))
        self.red_fraction.append(float(((red > green + 25) & (red > blue + 25) & (red > 50)).mean()))
        if self.previous_luminance is not None:
            self.motion.append(float(np.abs(luminance - self.previous_luminance).mean()))
        self.previous_luminance = luminance

    def result(self):
        means = np.asarray(self.means)
        jumps = np.abs(np.diff(means)) if len(means) > 1 else np.asarray([0.0])
        median = float(np.median(means))
        mad = float(np.median(np.abs(means - median))) + 1e-6
        result = {
            "frames": int(len(means)),
            "black_fraction": float(np.mean(means < 8)),
            "white_fraction": float(np.mean(means > 245)),
            "blank_fraction": float(np.mean(self.blank)),
            "partial_white_fraction": float(np.mean(self.partial_white)),
            "strobe_fraction": float(np.mean(jumps > 25)),
            "outlier_fraction": float(np.mean(np.abs(means - median) > max(40.0, 6 * mad))),
            "minimum_robot_red_fraction": float(np.min(self.red_fraction)),
            "mean_motion": float(np.mean(self.motion)) if self.motion else 0.0,
            "mean_luminance": float(np.mean(means)),
        }
        checks = {
            "no_black": result["black_fraction"] == 0.0,
            "no_white": result["white_fraction"] == 0.0,
            "no_blank": result["blank_fraction"] == 0.0,
            "no_partial_white": result["partial_white_fraction"] == 0.0,
            "no_strobe": result["strobe_fraction"] == 0.0,
            "no_outlier": result["outlier_fraction"] == 0.0,
            "robot_present_every_frame": result["minimum_robot_red_fraction"] > 0.001,
            "motion_present": result["mean_motion"] > 0.0,
        }
        result["checks"] = checks
        result["pass"] = bool(all(checks.values()))
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--alllink-npz", type=Path, required=True)
    parser.add_argument("--out-mp4", type=Path, required=True)
    parser.add_argument("--report-json", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--mesh-dir", type=Path, default=DEFAULT_MESH_DIR)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--terrain", action="store_true")
    parser.add_argument("--camera-offset", default="-0.25,-0.20,0.18")
    parser.add_argument("--chase-alpha", type=float, default=0.85)
    args = parser.parse_args()

    trace = np.load(args.alllink_npz, allow_pickle=True)
    body_names = [str(name) for name in trace["body_names"]]
    positions = trace["body_pos_w"][:, 0]
    quaternions = trace["body_quat_w"][:, 0]
    main_indices = [body_names.index(f"link_body_{index}") for index in range(21)]
    geometries = _parse_visual_geometry(args.mjcf, args.mesh_dir)
    first, unknown = _assemble(body_names, positions[0], quaternions[0], geometries)
    if unknown:
        raise RuntimeError(f"Unmapped non-split bodies: {unknown}")

    plotter = pv.Plotter(off_screen=True, window_size=(args.width, args.height))
    plotter.set_background((0.80, 0.855, 0.92))
    plotter.enable_lightkit()
    if args.terrain:
        terrain = _terrain_grid(trace, positions[:, main_indices])
        plotter.add_mesh(
            terrain,
            color=(0.55, 0.52, 0.48),
            opacity=1.0,
            smooth_shading=True,
            specular=0.05,
            ambient=0.3,
        )
    meshes = {}
    for category, (vertices, faces) in first.items():
        mesh = pv.PolyData(vertices, _vtk_faces(faces))
        plotter.add_mesh(
            mesh,
            color=COLORS[category],
            smooth_shading=True,
            specular=0.2,
            diffuse=0.9,
            ambient=0.25,
        )
        meshes[category] = mesh

    args.out_mp4.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    offset = np.asarray(_floats(args.camera_offset), dtype=np.float64)
    audit = FrameAudit()
    smoothed_target = None
    with imageio.get_writer(
        args.out_mp4, fps=args.fps, codec="libx264", quality=8, macro_block_size=None
    ) as writer:
        for frame_index in range(len(positions)):
            assembled, unknown = _assemble(
                body_names, positions[frame_index], quaternions[frame_index], geometries
            )
            if unknown:
                raise RuntimeError(f"Geometry mapping changed at frame {frame_index}: {unknown}")
            for category, (vertices, _) in assembled.items():
                meshes[category].points = vertices
                meshes[category].Modified()
            target = positions[frame_index, main_indices].mean(axis=0)
            smoothed_target = (
                target
                if smoothed_target is None
                else args.chase_alpha * smoothed_target + (1.0 - args.chase_alpha) * target
            )
            plotter.camera_position = [
                tuple(smoothed_target + offset),
                tuple(smoothed_target),
                (0.0, 0.0, 1.0),
            ]
            plotter.camera.view_angle = 35.0
            plotter.reset_camera_clipping_range()
            plotter.render()
            frame = np.asarray(plotter.screenshot(return_img=True))[..., :3]
            audit.add(frame)
            writer.append_data(frame)
            if frame_index % 25 == 0 or frame_index == len(positions) - 1:
                print(f"[offline video] frame {frame_index + 1}/{len(positions)}", flush=True)
    plotter.close()

    audit_result = audit.result()
    report = {
        "renderer": "third_party_offline_exact_world_pose",
        "source_trace": str(args.alllink_npz.resolve()),
        "output_mp4": str(args.out_mp4.resolve()),
        "body_count": len(body_names),
        "visual_geometry_source": str(args.mjcf.resolve()),
        "terrain_xy": "height[row,col] -> world(y,x), no flip",
        "terrain_z": "centipede.terrain.to_mujoco_collision",
        "camera": "clean 45-degree chase, no labels or black bars",
        "audit": audit_result,
    }
    args.report_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    return 0 if audit_result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
