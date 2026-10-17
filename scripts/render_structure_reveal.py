"""Render a collision-to-skeleton-to-skin presentation from an exact pose trace."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import cv2
import imageio_ffmpeg
import numpy as np
import trimesh

import render_high_fidelity as high_fidelity
import render_offline


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BLENDER = Path(r"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe")
DEFAULT_BLEND = ROOT / "assets" / "high_fidelity" / "source_Centipede_v1_CC0.blend"
DEFAULT_SKIN_MAP = ROOT / "assets" / "high_fidelity" / "skin_map.json"
DEFAULT_SCENE_SCRIPT = ROOT / "scripts" / "blender_high_fidelity_scene.py"
DEFAULT_MJCF = ROOT / "assets" / "offline_geometry" / "centipede_rebuild_v1_renamed.xml"
DEFAULT_MESH_DIR = ROOT / "assets" / "offline_geometry" / "meshes"
FOOT_RADIUS_M = 0.00045
SUPPORT_FADE_CLEARANCE_M = 0.00135


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def object_array(values):
    result = np.empty(len(values), dtype=object)
    result[:] = values
    return result


def segment_for_name(name):
    parts = name.split("_")
    if name.startswith("link_body_"):
        return max(0, min(20, int(parts[2])))
    if name.startswith("link_leg_"):
        return max(0, min(20, int(parts[2]) + 1))
    if name.startswith("foot_"):
        return max(0, min(20, int(parts[1]) + 1))
    return 0


def category_for_name(name):
    if name.startswith("link_body_"):
        return "body"
    if name.startswith("foot_"):
        return "foot"
    return "leg"


def sampled_height(height, xy, arena_size_m):
    size = height.shape[0]
    pixel = (xy + arena_size_m / 2.0) / arena_size_m * (size - 1)
    x = np.clip(pixel[..., 0], 0.0, size - 1.0)
    y = np.clip(pixel[..., 1], 0.0, size - 1.0)
    x0 = np.floor(x).astype(np.int64)
    y0 = np.floor(y).astype(np.int64)
    x1 = np.minimum(x0 + 1, size - 1)
    y1 = np.minimum(y0 + 1, size - 1)
    tx = x - x0
    ty = y - y0
    return (
        (1.0 - tx) * (1.0 - ty) * height[y0, x0]
        + tx * (1.0 - ty) * height[y0, x1]
        + (1.0 - tx) * ty * height[y1, x0]
        + tx * ty * height[y1, x1]
    )


def quaternion_slerp_wxyz(left, right, alpha):
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    alpha = np.asarray(alpha, dtype=np.float64)
    dot = np.sum(left * right, axis=-1, keepdims=True)
    right = np.where(dot < 0.0, -right, right)
    dot = np.clip(np.abs(dot), 0.0, 1.0)
    theta = np.arccos(dot)
    sine = np.sin(theta)
    near = sine < 1e-7
    left_weight = np.where(
        near,
        1.0 - alpha,
        np.sin((1.0 - alpha) * theta) / np.maximum(sine, 1e-12),
    )
    right_weight = np.where(
        near,
        alpha,
        np.sin(alpha * theta) / np.maximum(sine, 1e-12),
    )
    result = left_weight * left + right_weight * right
    return result / np.maximum(np.linalg.norm(result, axis=-1, keepdims=True), 1e-12)


def resample_presentation_trace(trace, start_time_s, fps, output, source_sha256):
    source_time = np.asarray(trace["time_s"], dtype=np.float64)
    require(np.all(np.diff(source_time) > 0.0), "Source trace time must be strictly increasing")
    require(start_time_s <= source_time[-1], "Presentation start is after the source trace")
    frame_count = int(np.floor((source_time[-1] - start_time_s) * fps + 1e-9)) + 1
    target_time = start_time_s + np.arange(frame_count, dtype=np.float64) / float(fps)
    right = np.searchsorted(source_time, target_time, side="right")
    right = np.clip(right, 1, len(source_time) - 1)
    left = right - 1
    interval = source_time[right] - source_time[left]
    alpha = ((target_time - source_time[left]) / interval)[:, None, None]

    source_positions = np.asarray(trace["body_pos_w"][:, 0], dtype=np.float64)
    positions = (
        (1.0 - alpha) * source_positions[left]
        + alpha * source_positions[right]
    )
    source_quaternions = np.asarray(trace["body_quat_w"][:, 0], dtype=np.float64)
    quaternions = quaternion_slerp_wxyz(
        source_quaternions[left], source_quaternions[right], alpha
    )
    source_wave = np.asarray(trace["realized_wave"][:, 0], dtype=np.float64)
    wave_alpha = alpha[:, 0, 0]
    realized_wave = (
        (1.0 - wave_alpha) * source_wave[left]
        + wave_alpha * source_wave[right]
    )
    source_alive = np.asarray(trace["alive"][:, 0], dtype=bool)
    alive = source_alive[left] & source_alive[right]

    np.savez_compressed(
        output,
        body_names=np.asarray(trace["body_names"]),
        body_pos_w=positions[:, None].astype(np.float32),
        body_quat_w=quaternions[:, None].astype(np.float32),
        alive=alive[:, None],
        realized_wave=realized_wave[:, None].astype(np.float32),
        time_s=target_time,
        lambda_mm=np.asarray(trace["lambda_mm"]),
        amp_mm=np.asarray(trace["amp_mm"]),
        terrain_seed=np.asarray(trace["terrain_seed"]),
        arena_size_m=np.asarray(trace["arena_size_m"]),
        image_size=np.asarray(trace["image_size"]),
        dt_pol=np.asarray(trace["dt_pol"]),
        presentation_interpolation=np.asarray(
            "linear rigid-body position plus shortest-arc quaternion SLERP"
        ),
        presentation_fps=np.asarray(float(fps), dtype=np.float64),
        original_trace_sha256=np.asarray(source_sha256),
    )
    saved = np.load(output, allow_pickle=True)
    require(len(saved["time_s"]) == frame_count, "Presentation trace frame count changed")
    require(bool(np.all(saved["alive"])), "Presentation trace includes a non-alive frame")
    require(bool(np.isfinite(saved["body_pos_w"]).all()), "Presentation positions are non-finite")
    require(bool(np.isfinite(saved["body_quat_w"]).all()), "Presentation rotations are non-finite")
    return {
        "enabled": True,
        "method": "linear rigid-body position plus shortest-arc quaternion SLERP",
        "source_frame_count": int(len(source_time)),
        "source_sampling_hz_median": float(1.0 / np.median(np.diff(source_time))),
        "target_fps": int(fps),
        "target_frame_count": int(frame_count),
        "source_time_interval_s": [float(start_time_s), float(source_time[-1])],
        "render_time_interval_s": [float(target_time[0]), float(target_time[-1])],
        "maximum_source_gap_s": float(np.max(np.diff(source_time))),
        "all_output_quaternion_norms_unit": bool(np.allclose(
            np.linalg.norm(saved["body_quat_w"], axis=-1), 1.0, rtol=0.0, atol=2e-6
        )),
    }


def build_structure_package(trace, selected, output):
    body_names = [str(name) for name in trace["body_names"]]
    body_index = {name: index for index, name in enumerate(body_names)}
    geometries = render_offline._parse_visual_geometry(
        DEFAULT_MJCF, DEFAULT_MESH_DIR, group="0"
    )
    collision_names = []
    collision_vertices = []
    collision_faces = []
    collision_segments = []
    collision_categories = []
    for name in body_names:
        parts = geometries.get(name, [])
        if not parts:
            continue
        combined = trimesh.util.concatenate(parts)
        collision_names.append(name)
        collision_vertices.append(np.asarray(combined.vertices, dtype=np.float64))
        collision_faces.append(np.asarray(combined.faces, dtype=np.int64))
        collision_segments.append(segment_for_name(name))
        collision_categories.append(category_for_name(name))

    edge_pairs = []
    edge_segments = []
    edge_kinds = []
    for index in range(20):
        edge_pairs.append((body_index[f"link_body_{index}"], body_index[f"link_body_{index + 1}"]))
        edge_segments.append(index + 1)
        edge_kinds.append(0)
    for index in range(19):
        for side, foot_side in (("L", 0), ("R", 1)):
            chain = [
                f"link_body_{index + 1}",
                f"link_leg_{index}_{side}_0",
                f"link_leg_{index}_{side}_1",
                f"link_leg_{index}_{side}_2",
                f"link_leg_{index}_{side}_3",
                f"foot_{index}_{foot_side}",
            ]
            for start, end in zip(chain, chain[1:]):
                edge_pairs.append((body_index[start], body_index[end]))
                edge_segments.append(index + 1)
                edge_kinds.append(1)

    node_indices = [body_index[f"link_body_{index}"] for index in range(21)]
    node_segments = list(range(21))
    node_kinds = [0] * 21
    foot_names = [f"foot_{index}_{side}" for index in range(19) for side in range(2)]
    foot_indices = [body_index[name] for name in foot_names]
    foot_segments = [index + 1 for index in range(19) for _ in range(2)]
    node_indices.extend(foot_indices)
    node_segments.extend(foot_segments)
    node_kinds.extend([1] * len(foot_indices))

    from centipede.terrain import homogeneous_terrain, to_mujoco_collision

    height = to_mujoco_collision(homogeneous_terrain(
        float(trace["lambda_mm"]), float(trace["amp_mm"]), int(trace["terrain_seed"])
    ))
    positions = trace["body_pos_w"][selected, 0]
    foot_positions = positions[:, foot_indices]
    support_z = sampled_height(
        height, foot_positions[..., :2], float(trace["arena_size_m"])
    )
    clearance = foot_positions[..., 2] - support_z - FOOT_RADIUS_M
    support_strength = np.clip(
        (SUPPORT_FADE_CLEARANCE_M - clearance) / SUPPORT_FADE_CLEARANCE_M,
        0.0,
        1.0,
    )

    np.savez_compressed(
        output,
        geometry_group=np.asarray("0"),
        selected_frames=np.asarray(selected, dtype=np.int64),
        collision_names=np.asarray(collision_names),
        collision_vertices=object_array(collision_vertices),
        collision_faces=object_array(collision_faces),
        collision_segments=np.asarray(collision_segments, dtype=np.int64),
        collision_categories=np.asarray(collision_categories),
        edge_pairs=np.asarray(edge_pairs, dtype=np.int64),
        edge_segments=np.asarray(edge_segments, dtype=np.int64),
        edge_kinds=np.asarray(edge_kinds, dtype=np.int64),
        node_indices=np.asarray(node_indices, dtype=np.int64),
        node_segments=np.asarray(node_segments, dtype=np.int64),
        node_kinds=np.asarray(node_kinds, dtype=np.int64),
        foot_indices=np.asarray(foot_indices, dtype=np.int64),
        foot_segments=np.asarray(foot_segments, dtype=np.int64),
        support_z=np.asarray(support_z, dtype=np.float64),
        support_strength=np.asarray(support_strength, dtype=np.float64),
    )
    return {
        "collision_bodies": len(collision_names),
        "skeleton_edges": len(edge_pairs),
        "skeleton_nodes": len(node_indices),
        "support_feet": len(foot_indices),
        "minimum_clearance_m": float(np.min(clearance)),
        "maximum_clearance_m": float(np.max(clearance)),
        "support_strength_range": [
            float(np.min(support_strength)), float(np.max(support_strength))
        ],
        "contact_semantics": (
            "terrain-proximity support markers, not raw PhysX contact points or impulses"
        ),
    }


def keyframe_evidence(frames_dir, output_dir, expected_frames):
    stages = {
        "01_collision_model": 0.08,
        "02_collision_wipe": 0.28,
        "03_skeleton_support": 0.46,
        "04_skin_reveal": 0.70,
        "05_high_fidelity": 0.98,
    }
    saved = {}
    for name, fraction in stages.items():
        index = 1 + int(round(fraction * (expected_frames - 1)))
        source = frames_dir / f"frame_{index:04d}.png"
        require(source.is_file(), f"Missing rendered key frame: {source}")
        destination = output_dir / f"{name}.png"
        shutil.copy2(source, destination)
        saved[name] = {
            "frame": index,
            "normalized_time": fraction,
            "path": str(destination),
            "sha256": high_fidelity.sha256(destination),
        }
    return saved


def interpolate_video(source, output, source_fps, target_fps, expected_frames):
    require(target_fps > source_fps, "Target interpolation fps must exceed source fps")
    filter_graph = (
        f"tpad=stop_mode=clone:stop_duration={2.0 / source_fps:.9f},"
        f"minterpolate=fps={target_fps}:mi_mode=mci:mc_mode=aobmc:"
        f"me_mode=bidir:vsbmc=1,trim=end_frame={expected_frames}"
    )
    command = [
        imageio_ffmpeg.get_ffmpeg_exe(),
        "-y",
        "-i",
        str(source),
        "-vf",
        filter_graph,
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "slow",
        "-crf",
        "14",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]
    completed = subprocess.run(command, cwd=ROOT, check=False)
    require(completed.returncode == 0, f"Video interpolation failed with exit code {completed.returncode}")
    require(output.is_file() and output.stat().st_size > 0, "Interpolated video was not written")
    return {
        "enabled": True,
        "method": "FFmpeg bidirectional motion-compensated interpolation (AOBMC + VSBMC)",
        "source_fps": int(source_fps),
        "target_fps": int(target_fps),
        "source_frame_count": int(round((expected_frames - 1) * source_fps / target_fps)) + 1,
        "target_frame_count": int(expected_frames),
        "scientific_semantics": (
            "presentation-only temporal interpolation; not additional physics, contact, or pose samples"
        ),
    }


def audit_video(video, expected_frames, fps, reveal_report):
    capture = cv2.VideoCapture(str(video))
    require(capture.isOpened(), f"Could not open rendered video: {video}")
    frames = []
    means = []
    blank = []
    motion = []
    previous = None
    while True:
        ok, bgr = capture.read()
        if not ok:
            break
        frames.append(bgr)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
        means.append(float(gray.mean()))
        blank.append(float(gray.std() < 5.0))
        if previous is not None:
            motion.append(float(np.abs(gray - previous).mean()))
        previous = gray
    capture.release()
    require(frames, "Rendered video contains no readable frames")

    def at(fraction):
        return frames[int(round(fraction * (len(frames) - 1)))]

    early = at(0.08)
    middle = at(0.46)
    late = at(0.98)
    eb, eg, er = (channel.astype(np.int16) for channel in cv2.split(early))
    mb, mg, mr = (channel.astype(np.int16) for channel in cv2.split(middle))
    ember = (er > eg + 35) & (er > eb + 35) & (er > 55)
    cyan = (mb > mr + 25) & (mg > mr + 18) & (mb > 55)
    amber = (mr > mg + 50) & (mr > mb + 65) & (mr > 100)
    hsv = cv2.cvtColor(late, cv2.COLOR_BGR2HSV)
    colored = (hsv[..., 1] > 45) & (hsv[..., 2] > 35)
    hue_values = hsv[..., 0][colored]
    late_hue = float(np.median(hue_values)) if len(hue_values) else -1.0
    hue_error = min(abs(late_hue - 19.0), 180.0 - abs(late_hue - 19.0))
    means = np.asarray(means, dtype=np.float64)
    jumps = np.abs(np.diff(means)) if len(means) > 1 else np.asarray([0.0])
    result = {
        "frames": len(frames),
        "expected_frames": int(expected_frames),
        "duration_s": float((len(frames) - 1) / fps),
        "black_fraction": float(np.mean(means < 8.0)),
        "white_fraction": float(np.mean(means > 245.0)),
        "blank_fraction": float(np.mean(blank)),
        "strobe_fraction": float(np.mean(jumps > 30.0)),
        "mean_motion": float(np.mean(motion)) if motion else 0.0,
        "early_collision_ember_fraction": float(np.mean(ember)),
        "middle_skeleton_cyan_fraction": float(np.mean(cyan)),
        "middle_support_amber_fraction": float(np.mean(amber)),
        "late_colored_fraction": float(np.mean(colored)),
        "late_skin_median_hue_opencv": late_hue,
        "late_skin_hue_error_opencv": float(hue_error),
        "blender_reveal_report_pass": bool(reveal_report["pass"]),
    }
    result["checks"] = {
        "frame_count_exact": result["frames"] == result["expected_frames"],
        "no_black": result["black_fraction"] == 0.0,
        "no_white": result["white_fraction"] == 0.0,
        "no_blank": result["blank_fraction"] == 0.0,
        "no_strobe": result["strobe_fraction"] == 0.0,
        "motion_present": result["mean_motion"] > 0.0,
        "collision_model_visible_early": result["early_collision_ember_fraction"] > 0.00015,
        "skeleton_visible_midway": result["middle_skeleton_cyan_fraction"] > 0.00008,
        "support_structure_visible_midway": result["middle_support_amber_fraction"] > 0.000005,
        "high_fidelity_skin_visible_late": result["late_colored_fraction"] > 0.0003,
        "source_skin_hue_preserved_late": result["late_skin_hue_error_opencv"] <= 6.0,
        "blender_reveal_audit_passes": result["blender_reveal_report_pass"],
    }
    result["pass"] = bool(all(result["checks"].values()))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--alllink-npz", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--render-fps", type=int, default=0)
    parser.add_argument("--interpolate-video", action="store_true")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--start-time-s", type=float, default=0.0)
    parser.add_argument("--interpolate-pose-trace", action="store_true")
    parser.add_argument("--camera-offset", default="-0.24,-0.19,0.145")
    parser.add_argument("--terrain-color", default="0.16,0.18,0.17")
    parser.add_argument("--light-scale", type=float, default=0.82)
    parser.add_argument("--exposure", type=float, default=-0.15)
    args = parser.parse_args()
    args.alllink_npz = args.alllink_npz.resolve()
    args.out_dir = args.out_dir.resolve()
    args.blender = args.blender.resolve()

    for path, label in (
        (args.alllink_npz, "exact pose trace"),
        (args.blender, "Blender executable"),
        (DEFAULT_BLEND, "CC0 high-fidelity skin"),
        (DEFAULT_SKIN_MAP, "skin map"),
        (DEFAULT_SCENE_SCRIPT, "Blender scene script"),
        (DEFAULT_MJCF, "collision MJCF"),
    ):
        require(path.is_file(), f"Missing {label}: {path}")
    require(args.fps > 0, "--fps must be positive")
    render_fps = int(args.render_fps or args.fps)
    require(render_fps > 0, "--render-fps must be positive")
    if args.interpolate_video:
        require(args.fps > render_fps, "--interpolate-video requires --fps > --render-fps")
    else:
        require(args.fps == render_fps, "Different render/output fps requires --interpolate-video")
    require(args.width > 0 and args.height > 0, "Render dimensions must be positive")
    camera_offset = [float(value) for value in args.camera_offset.split(",")]
    terrain_color = [float(value) for value in args.terrain_color.split(",")]
    require(len(camera_offset) == 3, "--camera-offset requires x,y,z")
    require(len(terrain_color) == 3, "--terrain-color requires r,g,b")

    require(args.start_time_s >= 0.0, "--start-time-s must be non-negative")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    source_trace_path = args.alllink_npz
    source_trace_sha256 = high_fidelity.sha256(source_trace_path)
    source_trace = np.load(source_trace_path, allow_pickle=True)
    render_trace_path = source_trace_path
    interpolation_audit = {
        "enabled": False,
        "method": None,
        "source_frame_count": int(len(source_trace["time_s"])),
        "target_fps": int(render_fps),
    }
    if args.interpolate_pose_trace:
        render_trace_path = args.out_dir / f"presentation_trace_{render_fps}fps.npz"
        interpolation_audit = resample_presentation_trace(
            source_trace,
            args.start_time_s,
            render_fps,
            render_trace_path,
            source_trace_sha256,
        )
        trace = np.load(render_trace_path, allow_pickle=True)
        selected = list(range(len(trace["time_s"])))
    else:
        trace = source_trace
        selected = [
            index for index, time_s in enumerate(trace["time_s"])
            if float(time_s) + 1e-12 >= args.start_time_s
        ]
    if args.max_frames > 0:
        selected = selected[: args.max_frames]
    require(len(selected) >= 40, "A structure-reveal video needs at least 40 source frames")
    require(bool(np.all(trace["alive"][selected, 0])), "Source trace is not alive for the full reveal")

    video = args.out_dir / "rough_structure_reveal_45deg.mp4"
    start_still = args.out_dir / "01_collision_model_start.png"
    binding_report_path = args.out_dir / "high_fidelity_binding.json"
    reveal_report_path = args.out_dir / "structure_reveal.scene_audit.json"
    audit_path = args.out_dir / "rough_structure_reveal_45deg.audit.json"
    manifest_path = args.out_dir / "MANIFEST.json"

    with tempfile.TemporaryDirectory(prefix="centipede_structure_reveal_") as temporary:
        temporary = Path(temporary)
        frames_dir = temporary / "frames"
        runtime_path = temporary / "runtime.json"
        terrain_path = temporary / "terrain.npz"
        structure_path = temporary / "structure.npz"
        terrain_audit = high_fidelity.prepare_terrain(trace, terrain_path, max_grid=640)
        structure_audit = build_structure_package(trace, selected, structure_path)
        runtime = {
            "target_body_extents": high_fidelity.target_body_extents(),
            "selected_frames": selected,
            "camera_offset": camera_offset,
            "camera_view": "45deg_structure_reveal",
            "terrain_color": terrain_color,
            "terrain_surface_style": "natural",
            "light_scale": float(args.light_scale),
            "exposure": float(args.exposure),
        }
        runtime_path.write_text(json.dumps(runtime, indent=2), encoding="utf-8")
        base_video = temporary / "rough_structure_reveal_rendered.mp4"
        command = [
            str(args.blender),
            "--background",
            str(DEFAULT_BLEND),
            "--python",
            str(DEFAULT_SCENE_SCRIPT),
            "--",
            "--trace",
            str(render_trace_path),
            "--skin-map",
            str(DEFAULT_SKIN_MAP),
            "--runtime",
            str(runtime_path),
            "--terrain",
            str(terrain_path),
            "--out-video",
            str(base_video),
            "--out-still",
            str(start_still),
            "--binding-report",
            str(binding_report_path),
            "--frames-dir",
            str(frames_dir),
            "--fps",
            str(render_fps),
            "--width",
            str(args.width),
            "--height",
            str(args.height),
            "--structure-package",
            str(structure_path),
            "--reveal-report",
            str(reveal_report_path),
        ]
        completed = subprocess.run(command, cwd=ROOT, check=False)
        require(completed.returncode == 0, f"Blender failed with exit code {completed.returncode}")
        reveal_report = json.loads(reveal_report_path.read_text(encoding="utf-8"))
        binding_report = json.loads(binding_report_path.read_text(encoding="utf-8"))
        require(reveal_report["pass"], "Blender structure-reveal audit failed")
        require(binding_report["pass"], "High-fidelity binding audit failed")
        high_fidelity.encode_frames(frames_dir, base_video, render_fps, len(selected))
        keyframes = keyframe_evidence(frames_dir, args.out_dir, len(selected))
        if args.interpolate_video:
            expected_output_frames = int(round(
                (len(selected) - 1) * float(args.fps) / float(render_fps)
            )) + 1
            video_interpolation_audit = interpolate_video(
                base_video, video, render_fps, args.fps, expected_output_frames
            )
        else:
            expected_output_frames = len(selected)
            shutil.copy2(base_video, video)
            video_interpolation_audit = {
                "enabled": False,
                "method": None,
                "source_fps": int(render_fps),
                "target_fps": int(args.fps),
                "source_frame_count": int(len(selected)),
                "target_frame_count": int(expected_output_frames),
            }
        video_audit = audit_video(video, expected_output_frames, args.fps, reveal_report)
        require(video_audit["pass"], f"Video audit failed: {video_audit['checks']}")

    video_audit["video_sha256"] = high_fidelity.sha256(video)
    video_audit["trace_sha256"] = high_fidelity.sha256(render_trace_path)
    video_audit["source_trace_sha256"] = source_trace_sha256
    audit_path.write_text(json.dumps(video_audit, indent=2), encoding="utf-8")
    manifest = {
        "format": "centipede-structure-reveal-manifest/2",
        "video": str(video),
        "video_sha256": video_audit["video_sha256"],
        "source_trace": str(source_trace_path),
        "source_trace_sha256": source_trace_sha256,
        "presentation_trace": str(render_trace_path),
        "presentation_trace_sha256": video_audit["trace_sha256"],
        "pose_trace_interpolation": interpolation_audit,
        "video_interpolation": video_interpolation_audit,
        "terrain": {
            "lambda_mm": float(trace["lambda_mm"]),
            "amplitude_mm": float(trace["amp_mm"]),
            "seed": int(trace["terrain_seed"]),
            "audit": terrain_audit,
            "visual_surface": {
                "style": "natural",
                "smooth_vertex_normals": True,
                "procedural_color_scale_per_m": 62.0,
                "procedural_micro_bump_scale_per_m": 420.0,
                "procedural_micro_bump_distance_m": 0.00022,
                "physics_geometry_unchanged": True,
            },
        },
        "fixed_wave": float(np.median(trace["realized_wave"][:, 0])),
        "camera": {"view": "45deg chase", "offset_m": camera_offset},
        "rendered_source_frames": len(selected),
        "frames": int(expected_output_frames),
        "render_fps": int(render_fps),
        "fps": int(args.fps),
        "duration_s": float((expected_output_frames - 1) / args.fps),
        "source_time_interval_s": [
            float(trace["time_s"][selected[0]]),
            float(trace["time_s"][selected[-1]]),
        ],
        "structure": structure_audit,
        "keyframes": keyframes,
        "binding_report": str(binding_report_path),
        "scene_audit": str(reveal_report_path),
        "video_audit": str(audit_path),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"video": str(video), "audit": video_audit, "manifest": str(manifest_path)}, indent=2))


if __name__ == "__main__":
    main()
