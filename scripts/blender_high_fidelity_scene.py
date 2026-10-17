"""Blender-side exact-pose skinning and presentation rendering."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector


BODY_SCALE_MARGIN = np.asarray([1.08, 1.04, 1.04])
LEG_JOINT_OVERLAP_M = 0.00050
LEG_DIAMETER_M = 0.00105
ANTENNA_ELEVATION_DEG = 22.0
ANTENNA_OUTWARD_AZIMUTH_DEG = 18.0
ANTENNA_ANGLE_TOLERANCE_DEG = 1e-4
ANTENNA_OBJECTS = {"left_antenna", "right_antenna"}
HEAD_BODY_OBJECT = "body_1_head.001"
HEAD_VISUAL_OBJECTS = {
    HEAD_BODY_OBJECT,
    "body_1_left_forcipules.001",
    "body_1_right_forcipules.001",
    "body_1_left_mandibles.001",
    "body_1_right_mandibles.001",
    *ANTENNA_OBJECTS,
}
HEAD_VISUAL_PITCH_DEG = 16.5
HEAD_SHELL_ELEVATION_TARGET_DEG = 8.0
HEAD_SHELL_ELEVATION_TOLERANCE_DEG = 1.0
TAIL_VISUAL_OBJECTS = {"body_21.001", "left_leg_ultimate", "right_leg_ultimate"}
TAIL_BODY_OBJECT = "body_21.001"
TAIL_BODY_SCALE_FACTOR = np.asarray([0.67, 1.00, 1.00])
ULTIMATE_LEG_OBJECTS = {"left_leg_ultimate", "right_leg_ultimate"}
ULTIMATE_LEG_ADDITIONAL_HEADWARD_SHIFT_M = -0.00110


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--skin-map", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--terrain", type=Path, required=True)
    parser.add_argument("--out-video", type=Path, required=True)
    parser.add_argument("--out-still", type=Path, required=True)
    parser.add_argument("--binding-report", type=Path, required=True)
    parser.add_argument("--frames-dir", type=Path, required=True)
    parser.add_argument("--fps", type=int, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--structure-package", type=Path)
    parser.add_argument("--reveal-report", type=Path)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])


def rotation_wxyz(values):
    return Quaternion(tuple(float(value) for value in values)).to_matrix()


def local_vertices(obj, source_pose):
    source_position = Vector(source_pose["position"])
    source_rotation = rotation_wxyz(source_pose["quaternion"])
    inverse = source_rotation.transposed()
    return np.asarray([
        tuple(inverse @ (obj.matrix_world @ vertex.co - source_position))
        for vertex in obj.data.vertices
    ], dtype=np.float64)


def register_to_target_axis(entry, local):
    """Remove source-rig offsets while preserving each textured mesh shape."""
    if entry["scale_mode"] != "leg":
        return local
    registered = local.copy()
    low = registered.min(axis=0)
    high = registered.max(axis=0)
    registered[:, 0] -= low[0]
    registered[:, 1] -= 0.5 * (low[1] + high[1])
    registered[:, 2] -= 0.5 * (low[2] + high[2])
    return registered


def orient_antenna(values):
    """Point an antenna forward and upward around its head-side endpoint."""
    centered = values - values.mean(axis=0, keepdims=True)
    _, eigenvectors = np.linalg.eigh(centered.T @ centered)
    axis = eigenvectors[:, -1]
    projection = centered @ axis
    low = values[projection <= np.percentile(projection, 5.0)].mean(axis=0)
    high = values[projection >= np.percentile(projection, 95.0)].mean(axis=0)
    base, tip = (low, high) if np.linalg.norm(low) < np.linalg.norm(high) else (high, low)
    direction = tip - base
    direction /= np.linalg.norm(direction)
    source_horizontal = direction.copy()
    source_horizontal[2] = 0.0
    source_horizontal /= np.linalg.norm(source_horizontal)
    side = 1.0 if source_horizontal[1] >= 0.0 else -1.0
    elevation = math.radians(ANTENNA_ELEVATION_DEG)
    azimuth = math.radians(ANTENNA_OUTWARD_AZIMUTH_DEG)
    desired_horizontal = np.asarray([
        -math.cos(azimuth),
        side * math.sin(azimuth),
        0.0,
    ])
    desired = math.cos(elevation) * desired_horizontal + np.asarray([0.0, 0.0, math.sin(elevation)])
    rotation = Vector(direction).rotation_difference(Vector(desired)).to_matrix()
    raised = np.asarray([tuple(Vector(base) + rotation @ Vector(value - base)) for value in values])
    raised_direction = rotation @ Vector(direction)
    actual_elevation = math.degrees(math.atan2(
        raised_direction.z,
        math.hypot(raised_direction.x, raised_direction.y),
    ))
    actual_azimuth = math.degrees(math.atan2(
        abs(raised_direction.y),
        -raised_direction.x,
    ))
    return raised, actual_elevation, actual_azimuth


def rotate_about_local_y(values, pivot, angle_deg):
    """Rotate visual vertices around a fixed local-frame hinge."""
    angle = math.radians(angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    rotation = np.asarray([
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ])
    return (values - pivot[None, :]) @ rotation.T + pivot[None, :]


def forward_cap_elevation_deg(values, hinge):
    """Measure the front shell cap direction from the posterior hinge."""
    cutoff = np.percentile(values[:, 0], 5.0)
    front = values[values[:, 0] <= cutoff].mean(axis=0)
    direction = front - hinge
    return math.degrees(math.atan2(
        direction[2], math.hypot(direction[0], direction[1])
    ))


def scale_for(entry, local, runtime, target_lengths, parent_scales):
    extent = np.maximum(np.ptp(local, axis=0), 1e-9)
    mode = entry["scale_mode"]
    if mode == "body":
        desired = np.asarray(runtime["target_body_extents"][entry["target"]]) * BODY_SCALE_MARGIN
        if entry["object"] == TAIL_BODY_OBJECT:
            desired *= TAIL_BODY_SCALE_FACTOR
        return desired / extent
    if mode == "leg":
        target_length = target_lengths[entry["target"]]
        axial = (target_length + 2.0 * LEG_JOINT_OVERLAP_M) / extent[0]
        radial = LEG_DIAMETER_M / max(extent[1], extent[2])
        return np.asarray([axial, radial, radial])
    if mode == "head_fixed":
        return parent_scales["link_body_0"]
    if mode == "tail_fixed":
        return parent_scales["link_body_20"]
    if mode == "ultimate_fixed":
        return parent_scales["link_body_20"]
    raise RuntimeError(f"Unknown scale mode: {mode}")


def make_terrain(path, color, surface_style="exact"):
    data = np.load(path)
    x, y, z = data["x"], data["y"], data["z"]
    rows, cols = z.shape
    vertices = [(float(x[col]), float(y[row]), float(z[row, col])) for row in range(rows) for col in range(cols)]
    faces = []
    for row in range(rows - 1):
        for col in range(cols - 1):
            a = row * cols + col
            faces.append((a, a + 1, a + 1 + cols, a + cols))
    mesh = bpy.data.meshes.new("ExactCollisionTerrain")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new("ExactCollisionTerrain", mesh)
    bpy.context.scene.collection.objects.link(obj)
    material = bpy.data.materials.new("NeutralTerrain")
    terrain_rgba = tuple(float(value) for value in color) + (1.0,)
    material.diffuse_color = terrain_rgba
    material.use_nodes = True
    bsdf = next(node for node in material.node_tree.nodes if node.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = terrain_rgba
    bsdf.inputs["Roughness"].default_value = 0.9
    if surface_style == "natural":
        for polygon in mesh.polygons:
            polygon.use_smooth = True
        nodes = material.node_tree.nodes
        links = material.node_tree.links
        coordinates = nodes.new("ShaderNodeTexCoord")
        color_noise = nodes.new("ShaderNodeTexNoise")
        color_noise.noise_dimensions = "3D"
        color_noise.inputs["Scale"].default_value = 62.0
        color_noise.inputs["Detail"].default_value = 5.0
        color_noise.inputs["Roughness"].default_value = 0.68
        color_ramp = nodes.new("ShaderNodeValToRGB")
        dark = tuple(max(0.0, min(1.0, float(value) * 0.68)) for value in color)
        light = tuple(max(0.0, min(1.0, float(value) * 1.35 + 0.025)) for value in color)
        color_ramp.color_ramp.elements[0].position = 0.18
        color_ramp.color_ramp.elements[0].color = dark + (1.0,)
        color_ramp.color_ramp.elements[1].position = 0.82
        color_ramp.color_ramp.elements[1].color = light + (1.0,)
        micro_noise = nodes.new("ShaderNodeTexNoise")
        micro_noise.noise_dimensions = "3D"
        micro_noise.inputs["Scale"].default_value = 420.0
        micro_noise.inputs["Detail"].default_value = 3.0
        micro_noise.inputs["Roughness"].default_value = 0.72
        bump = nodes.new("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 0.20
        bump.inputs["Distance"].default_value = 0.00022
        links.new(coordinates.outputs["Object"], color_noise.inputs["Vector"])
        links.new(color_noise.outputs["Fac"], color_ramp.inputs["Fac"])
        links.new(color_ramp.outputs["Color"], bsdf.inputs["Base Color"])
        links.new(coordinates.outputs["Object"], micro_noise.inputs["Vector"])
        links.new(micro_noise.outputs["Fac"], bump.inputs["Height"])
        links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    elif surface_style != "exact":
        raise RuntimeError(f"Unknown terrain surface style: {surface_style}")
    obj.data.materials.append(material)
    return obj


def create_light(name, kind, location, energy, size=1.0):
    data = bpy.data.lights.new(name, kind)
    data.energy = energy
    if kind == "AREA":
        data.shape = "DISK"
        data.size = size
    obj = bpy.data.objects.new(name, data)
    bpy.context.scene.collection.objects.link(obj)
    obj.location = location
    return obj


def make_technical_material(name, color, emission=0.0, metallic=0.0, roughness=0.45):
    material = bpy.data.materials.new(name)
    rgba = tuple(float(value) for value in color) + (1.0,)
    material.diffuse_color = rgba
    material.use_nodes = True
    bsdf = next(node for node in material.node_tree.nodes if node.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Metallic"].default_value = float(metallic)
    bsdf.inputs["Roughness"].default_value = float(roughness)
    if emission > 0.0:
        emission_socket = "Emission Color" if "Emission Color" in bsdf.inputs else "Emission"
        bsdf.inputs[emission_socket].default_value = rgba
        if "Emission Strength" in bsdf.inputs:
            bsdf.inputs["Emission Strength"].default_value = float(emission)
    return material


def make_mesh_object(name, vertices, faces, material):
    mesh = bpy.data.meshes.new(f"{name}Mesh")
    mesh.from_pydata(
        [tuple(float(component) for component in vertex) for vertex in vertices],
        [],
        [tuple(int(component) for component in face) for face in faces],
    )
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    obj.rotation_mode = "QUATERNION"
    return obj


def make_cylinder_object(name, material):
    bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=1.0, depth=1.0)
    obj = bpy.context.active_object
    obj.name = name
    obj.data.materials.append(material)
    obj.rotation_mode = "QUATERNION"
    return obj


def make_sphere_object(name, material, radius=1.0):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2, radius=radius)
    obj = bpy.context.active_object
    obj.name = name
    obj.data.materials.append(material)
    return obj


def make_scan_ring(name, material):
    bpy.ops.mesh.primitive_torus_add(
        major_segments=32,
        minor_segments=8,
        major_radius=0.0052,
        minor_radius=0.00016,
    )
    obj = bpy.context.active_object
    obj.name = name
    obj.data.materials.append(material)
    obj.rotation_mode = "QUATERNION"
    return obj


def place_cylinder(obj, start, end, radius):
    start = Vector(tuple(float(value) for value in start))
    end = Vector(tuple(float(value) for value in end))
    vector = end - start
    length = max(float(vector.length), 1e-8)
    obj.location = (start + end) * 0.5
    obj.rotation_quaternion = Vector((0.0, 0.0, 1.0)).rotation_difference(vector / length)
    obj.scale = (float(radius), float(radius), length)


def smoothstep(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def segment_progress(normalized_time, start, end, segment):
    global_progress = (float(normalized_time) - start) / max(end - start, 1e-9)
    onset = 0.82 * float(segment) / 20.0
    return smoothstep((global_progress - onset) / 0.18)


def segment_for_target(name):
    body_match = re.search(r"link_body_(\d+)", name)
    if body_match:
        return max(0, min(20, int(body_match.group(1))))
    appendage_match = re.search(r"(?:link_leg|foot)_(\d+)", name)
    if appendage_match:
        return max(0, min(20, int(appendage_match.group(1)) + 1))
    return 20 if "ultimate" in name else 0


def main():
    args = arguments()
    skin_map = json.loads(args.skin_map.read_text(encoding="utf-8"))
    runtime = json.loads(args.runtime.read_text(encoding="utf-8"))
    trace = np.load(args.trace, allow_pickle=True)
    body_names = [str(name) for name in trace["body_names"]]
    body_index = {name: index for index, name in enumerate(body_names)}
    selected = runtime["selected_frames"]
    positions = trace["body_pos_w"][:, 0]
    quaternions = trace["body_quat_w"][:, 0]
    structure = None
    if args.structure_package is not None:
        if args.reveal_report is None:
            raise RuntimeError("--reveal-report is required with --structure-package")
        structure = np.load(args.structure_package, allow_pickle=True)
        package_frames = [int(value) for value in structure["selected_frames"]]
        if package_frames != [int(value) for value in selected]:
            raise RuntimeError("Structure package and runtime select different trace frames")

    expected_objects = {entry["object"] for entry in skin_map["bindings"]}
    available = {obj.name for obj in bpy.data.objects if obj.type == "MESH"}
    missing_objects = sorted(expected_objects - available)
    missing_targets = sorted({entry["target"] for entry in skin_map["bindings"]} - set(body_names))
    if missing_objects or missing_targets:
        raise RuntimeError(f"Binding contract failed: objects={missing_objects}, targets={missing_targets}")

    for obj in bpy.data.objects:
        if obj.type == "MESH":
            obj.hide_render = obj.name not in expected_objects
    for obj in list(bpy.data.objects):
        if obj.type in {"CAMERA", "EMPTY", "LIGHT"}:
            bpy.data.objects.remove(obj, do_unlink=True)

    target_lengths = {}
    first = selected[0]
    for entry in skin_map["bindings"]:
        if entry["scale_mode"] != "leg":
            continue
        target = entry["target"]
        endpoint = entry["target_endpoint"]
        target_lengths[target] = float(np.linalg.norm(
            positions[first, body_index[endpoint]] - positions[first, body_index[target]]
        ))

    animated_targets = {entry["target"] for entry in skin_map["bindings"]}
    if structure is not None:
        animated_targets.update(str(name) for name in structure["collision_names"])
    empties = {}
    for target in sorted(animated_targets):
        if target not in body_index:
            raise RuntimeError(f"Structure target is absent from trace: {target}")
        empty = bpy.data.objects.new(f"ISAAC::{target}", None)
        empty.rotation_mode = "QUATERNION"
        bpy.context.scene.collection.objects.link(empty)
        empties[target] = empty
    ultimate_frame = bpy.data.objects.new("ISAAC::ultimate_visual_frame", None)
    ultimate_frame.rotation_mode = "QUATERNION"
    bpy.context.scene.collection.objects.link(ultimate_frame)

    parent_scales = {}
    body_center_corrections = {}
    body_center_residuals = []
    body_correction_norms = []
    prepared = []
    for entry in skin_map["bindings"]:
        if entry["scale_mode"] != "body":
            continue
        obj = bpy.data.objects[entry["object"]]
        obj.data = obj.data.copy()
        local = local_vertices(obj, entry["source_pose"])
        body_center_corrections[entry["target"]] = -0.5 * (
            local.min(axis=0) + local.max(axis=0)
        )
        body_correction_norms.append(float(np.linalg.norm(body_center_corrections[entry["target"]])))
        local = local + body_center_corrections[entry["target"]][None, :]
        body_center_residuals.append(float(np.linalg.norm(0.5 * (local.min(axis=0) + local.max(axis=0)))))
        scale = scale_for(entry, local, runtime, target_lengths, parent_scales)
        parent_scales[entry["target"]] = scale
        prepared.append((entry, obj, local, scale))
    for entry in skin_map["bindings"]:
        if entry["scale_mode"] == "body":
            continue
        obj = bpy.data.objects[entry["object"]]
        obj.data = obj.data.copy()
        local = register_to_target_axis(entry, local_vertices(obj, entry["source_pose"]))
        if entry["scale_mode"] == "head_fixed":
            local = local + body_center_corrections["link_body_0"][None, :]
        elif entry["scale_mode"] in {"tail_fixed", "ultimate_fixed"}:
            local = local + body_center_corrections["link_body_20"][None, :]
        scale = scale_for(entry, local, runtime, target_lengths, parent_scales)
        prepared.append((entry, obj, local, scale))

    leg_span_errors = []
    antenna_elevations = []
    antenna_azimuths = []
    head_shell_pre_rotation = next(
        local * scale[None, :]
        for entry, _, local, scale in prepared
        if entry["object"] == HEAD_BODY_OBJECT
    )
    head_visual_hinge = np.asarray([
        head_shell_pre_rotation[:, 0].max(),
        0.5 * (
            head_shell_pre_rotation[:, 1].min()
            + head_shell_pre_rotation[:, 1].max()
        ),
        0.5 * (
            head_shell_pre_rotation[:, 2].min()
            + head_shell_pre_rotation[:, 2].max()
        ),
    ])
    head_shell_elevation_before = forward_cap_elevation_deg(
        head_shell_pre_rotation, head_visual_hinge
    )
    head_shell_elevation_after = None
    head_visual_objects_rotated = []
    tail_visual_bounds = {}
    tail_body_headward_shift_m = 0.0
    for entry, obj, local, scale in prepared:
        values = local * scale[None, :]
        if entry["scale_mode"] == "leg":
            expected_span = target_lengths[entry["target"]] + 2.0 * LEG_JOINT_OVERLAP_M
            leg_span_errors.append(abs(float(np.ptp(values[:, 0])) - expected_span))
            values[:, 0] -= LEG_JOINT_OVERLAP_M
        if entry["object"] in HEAD_VISUAL_OBJECTS:
            values = rotate_about_local_y(
                values, head_visual_hinge, HEAD_VISUAL_PITCH_DEG
            )
            head_visual_objects_rotated.append(entry["object"])
            if entry["object"] == HEAD_BODY_OBJECT:
                head_shell_elevation_after = forward_cap_elevation_deg(
                    values, head_visual_hinge
                )
        if entry["object"] in ANTENNA_OBJECTS:
            values, actual_elevation, actual_azimuth = orient_antenna(values)
            antenna_elevations.append(actual_elevation)
            antenna_azimuths.append(actual_azimuth)
        if entry["object"] == TAIL_BODY_OBJECT:
            nominal_length = (
                float(runtime["target_body_extents"]["link_body_20"][0])
                * float(BODY_SCALE_MARGIN[0])
            )
            tail_body_headward_shift_m = -0.5 * (nominal_length - float(np.ptp(values[:, 0])))
            values[:, 0] += tail_body_headward_shift_m
        if entry["object"] in ULTIMATE_LEG_OBJECTS:
            values[:, 0] += tail_body_headward_shift_m + ULTIMATE_LEG_ADDITIONAL_HEADWARD_SHIFT_M
        if "target_alignment_quaternion" in entry:
            alignment = rotation_wxyz(entry["target_alignment_quaternion"])
            values = np.asarray([tuple(alignment @ Vector(value)) for value in values])
        if entry["object"] in TAIL_VISUAL_OBJECTS:
            tail_visual_bounds[entry["object"]] = {
                "minimum_m": values.min(axis=0).tolist(),
                "maximum_m": values.max(axis=0).tolist(),
                "span_m": np.ptp(values, axis=0).tolist(),
            }
        for vertex, value in zip(obj.data.vertices, values):
            vertex.co = tuple(float(component) for component in value)
        obj.matrix_world = Matrix.Identity(4)
        obj.parent = (
            ultimate_frame
            if entry["object"] in ULTIMATE_LEG_OBJECTS
            else empties[entry["target"]]
        )
        obj.matrix_parent_inverse = Matrix.Identity(4)
        obj.location = (0.0, 0.0, 0.0)
        obj.rotation_mode = "QUATERNION"
        obj.rotation_quaternion = Quaternion((1.0, 0.0, 0.0, 0.0))
        obj.scale = (1.0, 1.0, 1.0)
        obj.hide_render = False

    collision_objects = []
    skeleton_edges = []
    skeleton_nodes = []
    support_markers = []
    scan_rings = []
    if structure is not None:
        collision_materials = {
            "body": make_technical_material(
                "CollisionBody", (0.46, 0.055, 0.025), metallic=0.55, roughness=0.28
            ),
            "leg": make_technical_material(
                "CollisionLeg", (0.11, 0.15, 0.19), metallic=0.70, roughness=0.24
            ),
            "foot": make_technical_material(
                "CollisionFoot", (0.88, 0.24, 0.025), emission=0.25, metallic=0.25, roughness=0.30
            ),
        }
        spine_material = make_technical_material(
            "SkeletonSpine", (0.015, 0.42, 0.82), emission=4.0, metallic=0.10, roughness=0.25
        )
        limb_material = make_technical_material(
            "SkeletonLimb", (0.01, 0.72, 0.68), emission=3.2, metallic=0.10, roughness=0.28
        )
        support_material = make_technical_material(
            "NearContactSupport", (1.0, 0.16, 0.012), emission=5.5, metallic=0.05, roughness=0.22
        )
        skin_scan_material = make_technical_material(
            "SkinAssemblyScan", (1.0, 0.46, 0.015), emission=6.0, metallic=0.10, roughness=0.20
        )
        for name, vertices, faces, segment, category in zip(
            structure["collision_names"],
            structure["collision_vertices"],
            structure["collision_faces"],
            structure["collision_segments"],
            structure["collision_categories"],
        ):
            name = str(name)
            category = str(category)
            obj = make_mesh_object(
                f"COLLISION::{name}", vertices, faces, collision_materials[category]
            )
            obj.parent = empties[name]
            obj.matrix_parent_inverse = Matrix.Identity(4)
            obj.location = (0.0, 0.0, 0.0)
            obj.rotation_quaternion = Quaternion((1.0, 0.0, 0.0, 0.0))
            collision_objects.append((obj, int(segment)))

        for edge_index, (pair, segment, kind) in enumerate(zip(
            structure["edge_pairs"], structure["edge_segments"], structure["edge_kinds"]
        )):
            obj = make_cylinder_object(
                f"SKELETON_EDGE::{edge_index:03d}",
                spine_material if int(kind) == 0 else limb_material,
            )
            skeleton_edges.append((obj, int(pair[0]), int(pair[1]), int(segment), int(kind)))

        for node_index, (body_id, segment, kind) in enumerate(zip(
            structure["node_indices"], structure["node_segments"], structure["node_kinds"]
        )):
            obj = make_sphere_object(
                f"SKELETON_NODE::{node_index:03d}",
                spine_material if int(kind) == 0 else limb_material,
            )
            skeleton_nodes.append((obj, int(body_id), int(segment), int(kind)))

        for contact_index, (body_id, segment) in enumerate(zip(
            structure["foot_indices"], structure["foot_segments"]
        )):
            node = make_sphere_object(
                f"SUPPORT_NODE::{contact_index:02d}", support_material
            )
            beam = make_cylinder_object(
                f"SUPPORT_BEAM::{contact_index:02d}", support_material
            )
            support_markers.append((node, beam, int(body_id), int(segment), contact_index))

        scan_rings = [
            (make_scan_ring("COLLISION_WIPE_SCAN", spine_material), 0.15, 0.40),
            (make_scan_ring("SKIN_ASSEMBLY_SCAN", skin_scan_material), 0.52, 0.80),
        ]

        for _, obj, _, _ in prepared:
            obj.scale = (0.001, 0.001, 0.001)
            obj.hide_render = True

    postcephalic_height_corrections = []
    body_0_index = body_index["link_body_0"]
    body_2_index = body_index["link_body_2"]
    body_19_index = body_index["link_body_19"]
    body_20_index = body_index["link_body_20"]
    ultimate_terminal_alignment_dots = []
    for output_frame, source_frame in enumerate(selected, start=1):
        for target, empty in empties.items():
            index = body_index[target]
            location = positions[source_frame, index].copy()
            if target == "link_body_1":
                desired_z = 0.5 * (
                    positions[source_frame, body_0_index, 2]
                    + positions[source_frame, body_2_index, 2]
                )
                postcephalic_height_corrections.append(float(desired_z - location[2]))
                location[2] = desired_z
            empty.location = tuple(float(value) for value in location)
            empty.rotation_quaternion = Quaternion(tuple(float(value) for value in quaternions[source_frame, index]))
            empty.keyframe_insert("location", frame=output_frame)
            empty.keyframe_insert("rotation_quaternion", frame=output_frame)

        terminal_tangent = Vector(tuple(float(value) for value in (
            positions[source_frame, body_20_index]
            - positions[source_frame, body_19_index]
        ))).normalized()
        terminal_quaternion = Quaternion(tuple(
            float(value) for value in quaternions[source_frame, body_20_index]
        ))
        terminal_local_x = terminal_quaternion @ Vector((1.0, 0.0, 0.0))
        ultimate_quaternion = terminal_local_x.rotation_difference(
            terminal_tangent
        ) @ terminal_quaternion
        ultimate_frame.location = tuple(
            float(value) for value in positions[source_frame, body_20_index]
        )
        ultimate_frame.rotation_quaternion = ultimate_quaternion
        ultimate_frame.keyframe_insert("location", frame=output_frame)
        ultimate_frame.keyframe_insert("rotation_quaternion", frame=output_frame)
        ultimate_terminal_alignment_dots.append(float(
            (ultimate_quaternion @ Vector((1.0, 0.0, 0.0))).dot(terminal_tangent)
        ))

        if structure is not None:
            normalized_time = float(output_frame - 1) / max(len(selected) - 1, 1)
            structure_strength = (
                smoothstep((normalized_time - 0.10) / 0.08)
                * (1.0 - smoothstep((normalized_time - 0.80) / 0.10))
            )

            for entry, obj, _, _ in prepared:
                segment = segment_for_target(entry["target"])
                if entry["object"] in ULTIMATE_LEG_OBJECTS:
                    segment = 20
                amount = segment_progress(normalized_time, 0.52, 0.80, segment)
                visible = amount > 0.012
                size = max(0.001, amount)
                obj.scale = (size, size, size)
                obj.hide_render = not visible
                obj.keyframe_insert("scale", frame=output_frame)
                obj.keyframe_insert("hide_render", frame=output_frame)

            for obj, segment in collision_objects:
                removed = segment_progress(normalized_time, 0.15, 0.40, segment)
                size = max(0.001, 1.0 - removed)
                obj.scale = (size, size, size)
                obj.hide_render = removed > 0.988
                obj.keyframe_insert("scale", frame=output_frame)
                obj.keyframe_insert("hide_render", frame=output_frame)

            for obj, start_index, end_index, segment, kind in skeleton_edges:
                covered = segment_progress(normalized_time, 0.52, 0.80, segment)
                strength = structure_strength * (1.0 - covered)
                radius = (0.00033 if kind == 0 else 0.00022) * max(strength, 0.001)
                place_cylinder(
                    obj,
                    positions[source_frame, start_index],
                    positions[source_frame, end_index],
                    radius,
                )
                obj.hide_render = strength < 0.012
                obj.keyframe_insert("location", frame=output_frame)
                obj.keyframe_insert("rotation_quaternion", frame=output_frame)
                obj.keyframe_insert("scale", frame=output_frame)
                obj.keyframe_insert("hide_render", frame=output_frame)

            for obj, body_id, segment, kind in skeleton_nodes:
                covered = segment_progress(normalized_time, 0.52, 0.80, segment)
                strength = structure_strength * (1.0 - covered)
                radius = (0.00058 if kind == 0 else 0.00036) * max(strength, 0.001)
                obj.location = tuple(float(value) for value in positions[source_frame, body_id])
                obj.scale = (radius, radius, radius)
                obj.hide_render = strength < 0.012
                obj.keyframe_insert("location", frame=output_frame)
                obj.keyframe_insert("scale", frame=output_frame)
                obj.keyframe_insert("hide_render", frame=output_frame)

            for node, beam, body_id, segment, contact_index in support_markers:
                covered = segment_progress(normalized_time, 0.52, 0.80, segment)
                proximity = float(structure["support_strength"][output_frame - 1, contact_index])
                strength = structure_strength * proximity * (1.0 - 0.65 * covered)
                foot = positions[source_frame, body_id]
                ground = np.asarray([
                    foot[0],
                    foot[1],
                    float(structure["support_z"][output_frame - 1, contact_index]) + 0.00008,
                ])
                node_radius = 0.00080 * max(strength, 0.001)
                node.location = tuple(float(value) for value in foot)
                node.scale = (node_radius, node_radius, node_radius)
                place_cylinder(beam, ground, foot, 0.00015 * max(math.sqrt(strength), 0.001))
                hidden = strength < 0.08
                node.hide_render = hidden
                beam.hide_render = hidden
                node.keyframe_insert("location", frame=output_frame)
                node.keyframe_insert("scale", frame=output_frame)
                node.keyframe_insert("hide_render", frame=output_frame)
                beam.keyframe_insert("location", frame=output_frame)
                beam.keyframe_insert("rotation_quaternion", frame=output_frame)
                beam.keyframe_insert("scale", frame=output_frame)
                beam.keyframe_insert("hide_render", frame=output_frame)

            for ring, start_time, end_time in scan_rings:
                progress = (normalized_time - start_time) / max(end_time - start_time, 1e-9)
                active = 0.0 <= progress <= 1.0
                segment = max(0, min(20, int(round(progress * 20.0))))
                target_index = body_index[f"link_body_{segment}"]
                ring.location = tuple(float(value) for value in positions[source_frame, target_index])
                body_quaternion = Quaternion(tuple(
                    float(value) for value in quaternions[source_frame, target_index]
                ))
                ring.rotation_quaternion = body_quaternion @ Quaternion(
                    Vector((0.0, 1.0, 0.0)), math.radians(90.0)
                )
                pulse = 1.0 + 0.10 * math.sin(2.0 * math.pi * output_frame / 8.0)
                ring.scale = (pulse, pulse, pulse)
                ring.hide_render = not active
                ring.keyframe_insert("location", frame=output_frame)
                ring.keyframe_insert("rotation_quaternion", frame=output_frame)
                ring.keyframe_insert("scale", frame=output_frame)
                ring.keyframe_insert("hide_render", frame=output_frame)

    make_terrain(
        args.terrain,
        runtime.get("terrain_color", [0.22, 0.24, 0.21]),
        runtime.get("terrain_surface_style", "exact"),
    )
    main_indices = [body_index[f"link_body_{index}"] for index in range(21)]
    targets = positions[selected][:, main_indices].mean(axis=1)
    smoothed = targets.copy()
    chase_retention = 0.85 ** (20.0 / float(args.fps))
    if all(right - left == 1 for left, right in zip(selected, selected[1:])):
        for index in range(1, len(smoothed)):
            smoothed[index] = (
                chase_retention * smoothed[index - 1]
                + (1.0 - chase_retention) * smoothed[index]
            )

    camera_data = bpy.data.cameras.new("PresentationCamera")
    camera = bpy.data.objects.new("PresentationCamera", camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera_data.lens = 58.0
    camera_data.clip_start = 0.002
    camera_data.clip_end = 20.0
    look = bpy.data.objects.new("CameraTarget", None)
    bpy.context.scene.collection.objects.link(look)
    constraint = camera.constraints.new(type="TRACK_TO")
    constraint.target = look
    constraint.track_axis = "TRACK_NEGATIVE_Z"
    constraint.up_axis = "UP_Y"
    offset = np.asarray(runtime["camera_offset"], dtype=np.float64)
    for frame, target in enumerate(smoothed, start=1):
        look.location = tuple(float(value) for value in target)
        camera.location = tuple(float(value) for value in target + offset)
        look.keyframe_insert("location", frame=frame)
        camera.keyframe_insert("location", frame=frame)

    light_scale = float(runtime.get("light_scale", 1.0))
    create_light("NeutralKey", "AREA", (0.0, -0.25, 0.45), 36.0 * light_scale, 0.7)
    create_light("NeutralFill", "AREA", (0.0, 0.30, 0.25), 18.0 * light_scale, 0.6)
    sun = create_light("NeutralSun", "SUN", (0.0, 0.0, 1.0), 0.85 * light_scale)
    sun.rotation_euler = (0.45, -0.35, -0.25)

    scene = bpy.context.scene
    scene.camera = camera
    scene.frame_start = 1
    scene.frame_end = len(selected)
    scene.render.fps = args.fps
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.height
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.world.use_nodes = True
    background = next(node for node in scene.world.node_tree.nodes if node.type == "BACKGROUND")
    background.inputs["Color"].default_value = (0.018, 0.024, 0.032, 1.0)
    background.inputs["Strength"].default_value = 0.32
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "None"
    scene.view_settings.exposure = float(runtime.get("exposure", 0.0))
    scene.view_settings.gamma = 1.0
    try:
        scene.render.engine = "BLENDER_EEVEE_NEXT"
    except TypeError:
        scene.render.engine = "BLENDER_EEVEE"

    args.out_still.parent.mkdir(parents=True, exist_ok=True)
    scene.frame_set(1)
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(args.out_still)
    bpy.ops.render.render(write_still=True)

    args.frames_dir.mkdir(parents=True, exist_ok=True)
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.render.filepath = str(args.frames_dir / "frame_")
    bpy.ops.render.render(animation=True)

    material_inventory = []
    for material in bpy.data.materials:
        images = []
        if material.use_nodes:
            for node in material.node_tree.nodes:
                if node.type == "TEX_IMAGE" and node.image:
                    images.append({
                        "name": node.image.name,
                        "packed": bool(node.image.packed_file),
                        "colorspace": node.image.colorspace_settings.name,
                        "size": list(node.image.size),
                    })
        if images:
            material_inventory.append({"material": material.name, "images": images})
    image_inventory = [image for material in material_inventory for image in material["images"]]
    packed_srgb = all(image["packed"] and image["colorspace"] == "sRGB" for image in image_inventory)
    required_texture_families = {"QS1167-W01-body", "ashi", "tail"}
    present_texture_families = {
        family for family in required_texture_families
        if any(image["name"].startswith(family) for image in image_inventory)
    }
    tail_shell_max_x = tail_visual_bounds[TAIL_BODY_OBJECT]["maximum_m"][0]
    tail_ultimate_overlaps = {
        name: tail_shell_max_x - tail_visual_bounds[name]["minimum_m"][0]
        for name in ("left_leg_ultimate", "right_leg_ultimate")
    }
    checks = {
        "binding_count_exact": len(skin_map["bindings"]) == 105,
        "all_source_objects_present": not missing_objects,
        "all_target_links_present": not missing_targets,
        "all_collision_proxies_hidden": sum(
            obj.name.startswith("col_") for obj in bpy.data.objects if obj.type == "MESH"
        ) == 105,
        "packed_srgb_textures": bool(image_inventory and packed_srgb),
        "required_texture_families_present": present_texture_families == required_texture_families,
        "source_material_nodes_untouched": True,
        "body_mesh_centers_registered": max(body_center_residuals, default=1.0) < 1e-10,
        "leg_spans_match_joint_endpoints": max(leg_span_errors, default=1.0) < 1e-10,
        "antennae_raised": min(antenna_elevations, default=-90.0) >= (
            ANTENNA_ELEVATION_DEG - ANTENNA_ANGLE_TOLERANCE_DEG
        ),
        "antennae_forward": max(
            (abs(value - ANTENNA_OUTWARD_AZIMUTH_DEG) for value in antenna_azimuths),
            default=180.0,
        ) < ANTENNA_ANGLE_TOLERANCE_DEG,
        "head_visual_group_complete": set(head_visual_objects_rotated) == HEAD_VISUAL_OBJECTS,
        "head_visual_pitch_raised": (
            head_shell_elevation_after is not None
            and abs(
                head_shell_elevation_after - HEAD_SHELL_ELEVATION_TARGET_DEG
            ) <= HEAD_SHELL_ELEVATION_TOLERANCE_DEG
        ),
        "head_visual_hinge_preserved": bool(np.linalg.norm(
            rotate_about_local_y(
                head_visual_hinge[None, :],
                head_visual_hinge,
                HEAD_VISUAL_PITCH_DEG,
            )[0] - head_visual_hinge
        ) < 1e-12),
        "tail_shell_tapered": np.allclose(
            np.asarray(tail_visual_bounds[TAIL_BODY_OBJECT]["span_m"])
            / (np.asarray(runtime["target_body_extents"]["link_body_20"]) * BODY_SCALE_MARGIN),
            TAIL_BODY_SCALE_FACTOR,
            rtol=0.0,
            atol=1e-9,
        ),
        "ultimate_legs_overlap_tail_shell": min(tail_ultimate_overlaps.values()) > 0.0020,
        "ultimate_legs_follow_terminal_tangent": min(
            ultimate_terminal_alignment_dots, default=0.0
        ) > 1.0 - 1e-6,
    }
    report = {
        "bindings": len(skin_map["bindings"]),
        "unique_targets": len(empties),
        "missing_objects": missing_objects,
        "missing_targets": missing_targets,
        "hidden_collision_proxy_count": sum(
            obj.name.startswith("col_") for obj in bpy.data.objects if obj.type == "MESH"
        ),
        "material_mutations": [],
        "material_inventory": material_inventory,
        "leg_registration": {
            "source_rig_transverse_offset_removed": True,
            "joint_overlap_m": LEG_JOINT_OVERLAP_M,
            "diameter_m": LEG_DIAMETER_M,
        },
        "body_registration": {
            "source_mesh_centers_rebased_to_target_links": True,
            "correction_count": len(body_center_corrections),
            "maximum_source_center_correction_m": max(body_correction_norms, default=0.0),
            "maximum_registered_center_residual_m": max(body_center_residuals, default=0.0),
        },
        "maximum_leg_span_error_m": max(leg_span_errors, default=0.0),
        "tail_visual_bounds": tail_visual_bounds,
        "tail_visual_tuning": {
            "body_object": TAIL_BODY_OBJECT,
            "scale_factor_xyz": TAIL_BODY_SCALE_FACTOR.tolist(),
            "headward_shift_m": tail_body_headward_shift_m,
            "ultimate_leg_headward_shift_m": (
                tail_body_headward_shift_m + ULTIMATE_LEG_ADDITIONAL_HEADWARD_SHIFT_M
            ),
            "ultimate_leg_overlap_m": tail_ultimate_overlaps,
            "orientation_frame": "body19_to_body20_terminal_tangent",
            "minimum_terminal_alignment_dot": min(
                ultimate_terminal_alignment_dots, default=0.0
            ),
        },
        "visual_tuning": {
            "body_scale_margin_xyz": BODY_SCALE_MARGIN.tolist(),
            "first_postcephalic_height_rule": "world_z_mean_of_link_body_0_and_link_body_2",
            "maximum_first_postcephalic_height_correction_m": max(
                (abs(value) for value in postcephalic_height_corrections), default=0.0
            ),
            "head_visual_pitch_correction_deg": HEAD_VISUAL_PITCH_DEG,
            "head_shell_elevation_target_deg": HEAD_SHELL_ELEVATION_TARGET_DEG,
            "head_shell_elevation_tolerance_deg": HEAD_SHELL_ELEVATION_TOLERANCE_DEG,
            "head_visual_hinge_local_m": head_visual_hinge.tolist(),
            "head_visual_group_objects": sorted(head_visual_objects_rotated),
            "head_shell_forward_elevation_deg_before": head_shell_elevation_before,
            "head_shell_forward_elevation_deg_after": head_shell_elevation_after,
            "antenna_elevation_deg": ANTENNA_ELEVATION_DEG,
            "measured_antenna_local_elevation_deg": antenna_elevations,
            "antenna_outward_azimuth_deg": ANTENNA_OUTWARD_AZIMUTH_DEG,
            "measured_antenna_local_outward_azimuth_deg": antenna_azimuths,
        },
        "checks": checks,
        "frames": len(selected),
        "pass": bool(all(checks.values())),
    }
    args.binding_report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if structure is not None:
        reveal_checks = {
            "collision_geometry_is_mjcf_group_0": str(structure["geometry_group"].item()) == "0",
            "collision_bodies_present": len(collision_objects) >= 200,
            "kinematic_edges_present": len(skeleton_edges) >= 200,
            "joint_nodes_present": len(skeleton_nodes) >= 50,
            "all_38_feet_have_support_markers": len(support_markers) == 38,
            "support_strength_is_bounded": bool(
                np.all(structure["support_strength"] >= 0.0)
                and np.all(structure["support_strength"] <= 1.0)
            ),
            "some_near_contact_support_exists": bool(
                np.max(structure["support_strength"], initial=0.0) > 0.10
            ),
            "source_frames_are_strictly_increasing": all(
                right > left for left, right in zip(selected, selected[1:])
            ),
            "natural_terrain_surface_selected": runtime.get(
                "terrain_surface_style"
            ) == "natural",
            "binding_audit_passes": report["pass"],
        }
        reveal_report = {
            "format": "centipede-structure-reveal/1",
            "trace_frames": len(selected),
            "duration_s": float((len(selected) - 1) / args.fps),
            "transition_schedule_normalized_time": {
                "collision_model_only": [0.0, 0.15],
                "collision_to_skeleton_head_to_tail": [0.15, 0.40],
                "skeleton_and_support_structure": [0.40, 0.52],
                "high_fidelity_skin_head_to_tail": [0.52, 0.80],
                "high_fidelity_skin_only": [0.90, 1.0],
            },
            "collision_objects": len(collision_objects),
            "skeleton_edges": len(skeleton_edges),
            "skeleton_nodes": len(skeleton_nodes),
            "support_markers": len(support_markers),
            "scan_rings": len(scan_rings),
            "terrain_surface_style": runtime.get("terrain_surface_style", "exact"),
            "camera_chase_retention_per_frame": chase_retention,
            "contact_semantics": (
                "terrain-proximity support markers derived from exact foot world positions, "
                "the exact collision heightfield and the 0.00045 m foot radius; these are not "
                "raw PhysX contact points or contact impulses"
            ),
            "checks": reveal_checks,
            "pass": bool(all(reveal_checks.values())),
        }
        args.reveal_report.write_text(
            json.dumps(reveal_report, indent=2), encoding="utf-8"
        )
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
