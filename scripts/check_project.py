"""Fail-loudly integrity check for the clean repository."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--quick", action="store_true", help="Skip the 1536x1536 terrain rebuild")
args = parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    lock_path = ROOT / "provenance" / "SOURCE_LOCK.json"
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    for relative, expected in lock["release_sha256"].items():
        path = ROOT / relative
        require(path.is_file(), f"Missing locked file: {relative}")
        require(sha256(path) == expected, f"Hash drift: {relative}")

    forbidden = {"_archive", "_quarantine", "closed_loop", "attempts", "recovery", "guardian", "supervisor", "logs"}
    present = {part.lower() for path in ROOT.rglob("*") for part in path.relative_to(ROOT).parts}
    require(not (forbidden & present), f"Historical runtime directories present: {sorted(forbidden & present)}")
    usd = sorted(path.relative_to(ROOT).as_posix() for path in (ROOT / "assets").rglob("*.usd"))
    require(len(usd) == 5, f"Expected one model with four layers, got {usd}")
    offline_geometry = ROOT / "assets" / "offline_geometry"
    offline_mjcf = offline_geometry / "centipede_rebuild_v1_renamed.xml"
    require(offline_mjcf.is_file(), "Offline visual-geometry MJCF is missing")
    mesh_references = sorted(
        element.get("file") for element in ET.parse(offline_mjcf).getroot().iter("mesh")
    )
    offline_meshes = sorted(path.name for path in (offline_geometry / "meshes").glob("*.obj"))
    require(len(offline_meshes) == 21, f"Expected 21 offline body meshes, got {offline_meshes}")
    require(offline_meshes == mesh_references, "Offline MJCF/OBJ reference drift")

    high_fidelity = ROOT / "assets" / "high_fidelity"
    high_blend = high_fidelity / "source_Centipede_v1_CC0.blend"
    skin_map_path = high_fidelity / "skin_map.json"
    require(high_blend.is_file() and skin_map_path.is_file(), "High-fidelity visual skin is incomplete")
    blend_files = sorted(path.relative_to(ROOT).as_posix() for path in ROOT.rglob("*.blend"))
    require(
        blend_files == ["assets/high_fidelity/source_Centipede_v1_CC0.blend"],
        f"Unexpected Blender assets: {blend_files}",
    )
    skin_map = json.loads(skin_map_path.read_text(encoding="utf-8"))
    bindings = skin_map.get("bindings", [])
    mode_counts = {
        mode: sum(entry.get("scale_mode") == mode for entry in bindings)
        for mode in ("body", "head_fixed", "leg", "ultimate_fixed")
    }
    require(len(bindings) == 105, f"Expected 105 high-fidelity visual bindings, got {len(bindings)}")
    require(mode_counts == {
        "body": 21, "head_fixed": 6, "leg": 76, "ultimate_fixed": 2,
    }, f"High-fidelity topology drift: {mode_counts}")
    require(len({entry["object"] for entry in bindings}) == 105, "Duplicate high-fidelity source object")
    require(len({entry["target"] for entry in bindings}) == 97, "High-fidelity target-link count drift")
    require(
        skin_map.get("physics_policy") == "visual-only; never loaded by Isaac or collision",
        "High-fidelity physics isolation contract drift",
    )
    require(
        sha256(high_blend) == skin_map.get("source_blend_sha256"),
        "High-fidelity Blender source/map hash mismatch",
    )
    color = skin_map.get("source_color_contract", {})
    require(color.get("packed_images") == ["QS1167-W01-body", "ashi", "tail"], "Texture-family drift")
    require(color.get("colorspace") == "sRGB", "High-fidelity texture colorspace drift")
    require(color.get("uv_policy") == "preserve", "High-fidelity UV policy drift")
    require(color.get("material_policy") == "preserve_source_nodes", "High-fidelity material policy drift")
    require(color.get("mujoco_reference_median_hue_opencv") == 19.0, "MuJoCo color reference drift")
    require(color.get("hue_tolerance_opencv") == 6.0, "High-fidelity color tolerance drift")
    checkpoints = sorted((ROOT / "checkpoints").glob("*.pth"))
    require([p.name for p in checkpoints] == [
        "policy_seed_141421.pth", "policy_seed_161803.pth", "policy_seed_173205.pth"
    ], "Checkpoint whitelist drift")

    sys.path.insert(0, str(ROOT))
    import torch
    from centipede.gait import GaitMap, WaveActionLaw

    gait = GaitMap(ROOT / "config" / "gait_map.json", "cpu")
    require(len(gait.waves) == 13, "Gait-map knot count drift")
    require(WaveActionLaw.target(-1) == 1.0 and WaveActionLaw.target(1) == 4.0, "Action bounds drift")
    for checkpoint in checkpoints:
        doc = torch.load(checkpoint, map_location="cpu", weights_only=True)
        state = doc["model"]
        require(doc.get("epoch") == 24, f"Unexpected epoch: {checkpoint.name}")
        require(tuple(state["a2c_network.actor_mlp.0.weight"].shape) == (256, 34), f"Obs mismatch: {checkpoint.name}")
        require(tuple(state["a2c_network.mu.weight"].shape) == (1, 128), f"Action mismatch: {checkpoint.name}")

    if not args.quick:
        import numpy as np
        from centipede.terrain import to_mujoco_collision, tri_terrain

        def array_hash(array):
            value = np.ascontiguousarray(array)
            digest = hashlib.sha256()
            digest.update(str(value.dtype).encode("ascii"))
            digest.update(json.dumps(list(value.shape)).encode("ascii"))
            digest.update(value.view(np.uint8).tobytes())
            return digest.hexdigest()

        source = tri_terrain(223607)
        collision = to_mujoco_collision(source)
        require(array_hash(source) == lock["runtime_contract"]["tri_source_sha256"], "Tri terrain source drift")
        require(array_hash(collision) == lock["runtime_contract"]["tri_collision_sha256"], "Tri terrain collision drift")
    print("CLEAN_PROJECT_CHECK: PASS")


if __name__ == "__main__":
    main()
