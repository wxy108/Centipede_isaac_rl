"""Launch the optional Blender presentation renderer from an exact Isaac pose trace."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BLENDER = Path(r"C:\Program Files\Blender Foundation\Blender 5.0\blender.exe")
DEFAULT_BLEND = ROOT / "assets" / "high_fidelity" / "source_Centipede_v1_CC0.blend"
DEFAULT_SKIN_MAP = ROOT / "assets" / "high_fidelity" / "skin_map.json"
DEFAULT_SCENE_SCRIPT = ROOT / "scripts" / "blender_high_fidelity_scene.py"
DEFAULT_MJCF = ROOT / "assets" / "offline_geometry" / "centipede_rebuild_v1_renamed.xml"
DEFAULT_MESH_DIR = ROOT / "assets" / "offline_geometry" / "meshes"
MUJOCO_REFERENCE_HUE_OPENCV = 19.0
HUE_TOLERANCE_OPENCV = 6.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def target_body_extents() -> dict[str, list[float]]:
    """Return exact low-model visual extents in each Isaac body-local frame."""
    import xml.etree.ElementTree as ET

    root = ET.parse(DEFAULT_MJCF).getroot()
    scales = {}
    for element in root.find("asset").findall("mesh"):
        values = [float(value) for value in element.get("scale", "1 1 1").split()]
        if len(values) == 1:
            values *= 3
        scales[element.get("name")] = np.asarray(values, dtype=np.float64)
    result = {}
    for index in range(21):
        mesh = trimesh.load(DEFAULT_MESH_DIR / f"link_{index}.obj", process=False)
        result[f"link_body_{index}"] = (mesh.extents * scales[f"link_{index}"]).tolist()
    return result


def prepare_terrain(trace, path: Path, max_grid: int = 320) -> dict:
    """Build and audit the collision-coordinate terrain around the trajectory."""
    sys.path.insert(0, str(ROOT))
    from centipede.terrain import homogeneous_terrain, to_mujoco_collision

    body_names = [str(name) for name in trace["body_names"]]
    main = [body_names.index(f"link_body_{index}") for index in range(21)]
    positions = trace["body_pos_w"][:, 0, main, :2]
    arena = float(trace["arena_size_m"])
    size = int(trace["image_size"])
    margin = 0.45
    low = np.maximum(positions.min(axis=(0, 1)) - margin, -arena / 2)
    high = np.minimum(positions.max(axis=(0, 1)) + margin, arena / 2)
    source = homogeneous_terrain(
        float(trace["lambda_mm"]), float(trace["amp_mm"]), int(trace["terrain_seed"])
    )
    height = to_mujoco_collision(source)

    def pixel(value):
        return int(np.clip(round((value + arena / 2) / arena * (size - 1)), 0, size - 1))

    x0, y0 = pixel(low[0]), pixel(low[1])
    x1, y1 = pixel(high[0]), pixel(high[1])
    x1, y1 = max(x1, x0 + 1), max(y1, y0 + 1)
    x_ids = np.linspace(x0, x1, min(max_grid, x1 - x0 + 1)).round().astype(int)
    y_ids = np.linspace(y0, y1, min(max_grid, y1 - y0 + 1)).round().astype(int)
    coordinates = np.linspace(-arena / 2, arena / 2, size)
    render_height = height[np.ix_(y_ids, x_ids)]
    np.savez_compressed(
        path,
        x=coordinates[x_ids],
        y=coordinates[y_ids],
        z=render_height,
    )
    saved = np.load(path)
    copied_height = saved["z"]
    return {
        "sha256": sha256(path),
        "axis_contract": "z[row,col] -> world(y[row],x[col]); no transpose or axis flip",
        "z_contract": "centipede.terrain.to_mujoco_collision(source_heightmap)",
        "source_shape": list(height.shape),
        "crop_pixel_bounds_xyxy_inclusive": [x0, y0, x1, y1],
        "render_shape": list(render_height.shape),
        "x_source_pixel_indices": x_ids.tolist(),
        "y_source_pixel_indices": y_ids.tolist(),
        "decimated_for_rendering": bool(
            len(x_ids) < x1 - x0 + 1 or len(y_ids) < y1 - y0 + 1
        ),
        "maximum_saved_z_copy_error_m": float(
            np.max(np.abs(copied_height - render_height))
        ),
        "checks": {
            "source_is_trace_sized": list(height.shape) == [size, size],
            "x_is_strictly_increasing": bool(np.all(np.diff(saved["x"]) > 0.0)),
            "y_is_strictly_increasing": bool(np.all(np.diff(saved["y"]) > 0.0)),
            "z_shape_matches_yx": list(copied_height.shape)
            == [len(saved["y"]), len(saved["x"])],
            "saved_z_is_exact_source_subset": bool(
                np.array_equal(copied_height, render_height)
            ),
        },
    }


def audit_video(path: Path, expected_frames: int) -> dict:
    capture = cv2.VideoCapture(str(path))
    require(capture.isOpened(), f"Could not open rendered video: {path}")
    means, blanks, color_fraction, motion = [], [], [], []
    hue_histogram = np.zeros(180, dtype=np.int64)
    previous = None
    while True:
        ok, bgr = capture.read()
        if not ok:
            break
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        luminance = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        colored = (hsv[..., 1] > 45) & (hsv[..., 2] > 35)
        means.append(float(luminance.mean()))
        blanks.append(float(luminance.std() < 5.0))
        color_fraction.append(float(colored.mean()))
        hue_histogram += np.bincount(hsv[..., 0][colored], minlength=180)
        if previous is not None:
            motion.append(float(np.abs(luminance - previous).mean()))
        previous = luminance
    capture.release()
    means = np.asarray(means, dtype=np.float64)
    jumps = np.abs(np.diff(means)) if len(means) > 1 else np.asarray([0.0])
    hue_median = float(np.searchsorted(np.cumsum(hue_histogram), hue_histogram.sum() / 2.0))
    hue_error = min(
        abs(hue_median - MUJOCO_REFERENCE_HUE_OPENCV),
        180.0 - abs(hue_median - MUJOCO_REFERENCE_HUE_OPENCV),
    )
    result = {
        "frames": int(len(means)),
        "expected_frames": int(expected_frames),
        "black_fraction": float(np.mean(means < 8.0)) if len(means) else 1.0,
        "white_fraction": float(np.mean(means > 245.0)) if len(means) else 1.0,
        "blank_fraction": float(np.mean(blanks)) if blanks else 1.0,
        "strobe_fraction": float(np.mean(jumps > 30.0)),
        "minimum_colored_fraction": float(np.min(color_fraction)) if color_fraction else 0.0,
        "colored_pixel_median_hue_opencv": hue_median,
        "mujoco_reference_median_hue_opencv": MUJOCO_REFERENCE_HUE_OPENCV,
        "hue_error_opencv": float(hue_error),
        "mean_motion": float(np.mean(motion)) if motion else 0.0,
    }
    result["checks"] = {
        "frame_count_exact": result["frames"] == result["expected_frames"],
        "no_black": result["black_fraction"] == 0.0,
        "no_white": result["white_fraction"] == 0.0,
        "no_blank": result["blank_fraction"] == 0.0,
        "no_strobe": result["strobe_fraction"] == 0.0,
        "colored_skin_present": result["minimum_colored_fraction"] > 0.0003,
        "source_hue_preserved": result["hue_error_opencv"] <= HUE_TOLERANCE_OPENCV,
        "motion_present": result["mean_motion"] > 0.0,
    }
    result["pass"] = bool(all(result["checks"].values()))
    return result


def encode_frames(frames_dir: Path, output: Path, fps: int, expected_frames: int) -> None:
    frames = sorted(frames_dir.glob("frame_*.png"))
    require(len(frames) == expected_frames, f"Rendered {len(frames)} PNGs, expected {expected_frames}")
    with imageio.get_writer(
        output, fps=fps, codec="libx264", quality=9, macro_block_size=None,
        ffmpeg_params=["-pix_fmt", "yuv420p"],
    ) as writer:
        for path in frames:
            writer.append_data(imageio.imread(path)[..., :3])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--alllink-npz", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--frame-step", type=int, default=1)
    parser.add_argument("--camera-offset", default="-0.19,-0.15,0.12")
    parser.add_argument("--terrain-color", default="0.22,0.24,0.21")
    parser.add_argument("--light-scale", type=float, default=1.0)
    parser.add_argument("--exposure", type=float, default=0.0)
    parser.add_argument("--view-name", default="45deg")
    args = parser.parse_args()

    for path, label in (
        (args.alllink_npz, "pose trace"),
        (args.blender, "Blender executable"),
        (DEFAULT_BLEND, "CC0 Blender skin"),
        (DEFAULT_SKIN_MAP, "skin map"),
        (DEFAULT_SCENE_SCRIPT, "Blender scene script"),
    ):
        require(path.is_file(), f"Missing {label}: {path}")
    require(args.frame_step > 0, "--frame-step must be positive")
    view_name = args.view_name.strip().lower().replace(" ", "_")
    require(
        view_name and all(character.isalnum() or character in "-_" for character in view_name),
        "--view-name must contain only letters, digits, dashes or underscores",
    )
    camera_offset = [float(value) for value in args.camera_offset.split(",")]
    require(len(camera_offset) == 3, "--camera-offset must contain exactly x,y,z")
    terrain_color = [float(value) for value in args.terrain_color.split(",")]
    require(len(terrain_color) == 3, "--terrain-color must contain exactly r,g,b")
    require(all(0.0 <= value <= 1.0 for value in terrain_color), "--terrain-color values must be in [0,1]")
    require(args.light_scale > 0.0, "--light-scale must be positive")
    require(-5.0 <= args.exposure <= 5.0, "--exposure must be in [-5,5]")
    trace = np.load(args.alllink_npz, allow_pickle=True)
    total = len(trace["body_pos_w"])
    selected = list(range(0, total, args.frame_step))
    if args.max_frames > 0:
        selected = selected[: args.max_frames]
    require(len(selected) >= 2, "At least two frames are required for an animation audit")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    if view_name == "45deg":
        output_video = args.out_dir / "high_fidelity_45deg.mp4"
        output_still = args.out_dir / "high_fidelity_alignment.png"
        binding_report = args.out_dir / "high_fidelity_binding.json"
        final_report = args.out_dir / "high_fidelity_45deg.audit.json"
    else:
        stem = f"high_fidelity_{view_name}"
        output_video = args.out_dir / f"{stem}.mp4"
        output_still = args.out_dir / f"{stem}_alignment.png"
        binding_report = args.out_dir / f"{stem}_binding.json"
        final_report = args.out_dir / f"{stem}.audit.json"

    runtime = {
        "target_body_extents": target_body_extents(),
        "selected_frames": selected,
        "camera_offset": camera_offset,
        "camera_view": view_name,
        "terrain_color": terrain_color,
        "light_scale": float(args.light_scale),
        "exposure": float(args.exposure),
    }
    with tempfile.TemporaryDirectory(prefix="centipede_hifi_") as temporary:
        temporary = Path(temporary)
        terrain_path = temporary / "terrain.npz"
        runtime_path = temporary / "runtime.json"
        frames_dir = temporary / "frames"
        frames_dir.mkdir()
        terrain_alignment = prepare_terrain(trace, terrain_path)
        runtime_path.write_text(json.dumps(runtime, indent=2), encoding="utf-8")
        command = [
            str(args.blender), "--background", str(DEFAULT_BLEND),
            "--python", str(DEFAULT_SCENE_SCRIPT), "--",
            "--trace", str(args.alllink_npz.resolve()),
            "--skin-map", str(DEFAULT_SKIN_MAP.resolve()),
            "--runtime", str(runtime_path),
            "--terrain", str(terrain_path),
            "--out-video", str(output_video.resolve()),
            "--out-still", str(output_still.resolve()),
            "--binding-report", str(binding_report.resolve()),
            "--frames-dir", str(frames_dir),
            "--fps", str(args.fps), "--width", str(args.width), "--height", str(args.height),
        ]
        subprocess.run(command, check=True, cwd=ROOT)
        encode_frames(frames_dir, output_video, args.fps, len(selected))

    binding = json.loads(binding_report.read_text(encoding="utf-8"))
    require(binding.get("pass") is True, "Blender binding report failed")
    video_audit = audit_video(output_video, len(selected))
    report = {
        "renderer": "blender_cc0_high_fidelity_exact_pose_skin",
        "camera_view": view_name,
        "camera_offset_m": runtime["camera_offset"],
        "terrain_color_linear_rgb": runtime["terrain_color"],
        "light_scale": runtime["light_scale"],
        "exposure": runtime["exposure"],
        "physics_unchanged": True,
        "collision_unchanged": True,
        "source_trace": str(args.alllink_npz.resolve()),
        "source_trace_sha256": sha256(args.alllink_npz),
        "source_blend": str(DEFAULT_BLEND.resolve()),
        "source_blend_sha256": sha256(DEFAULT_BLEND),
        "skin_map_sha256": sha256(DEFAULT_SKIN_MAP),
        "terrain_render_input_sha256": terrain_alignment["sha256"],
        "terrain_alignment": terrain_alignment,
        "color_contract": "packed source textures, original UVs, sRGB, AgX, neutral lights, no hue edits",
        "output_video": str(output_video.resolve()),
        "output_still": str(output_still.resolve()),
        "binding": binding,
        "video_audit": video_audit,
        "pass": bool(
            binding["pass"]
            and video_audit["pass"]
            and all(terrain_alignment["checks"].values())
        ),
    }
    final_report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
