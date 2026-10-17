"""One-command entry point for the canonical structure-reveal demo."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "demo" / "structure_reveal" / "source" / "all_link_world_poses.npz"
FIXTURE_SHA256 = "c1bd51f2b77f6532e31728527dc93a03e693b2a19bb26b47aa43e7bfe0aa13f6"
CANONICAL_REPORT = (
    ROOT
    / "outputs"
    / "report"
    / "rough_structure_reveal_20260820"
    / "final_120fps"
)
DEFAULT_OUTPUT = (
    CANONICAL_REPORT
    if CANONICAL_REPORT.is_dir()
    else ROOT / "outputs" / "demo" / "structure_reveal_120fps"
)
VIDEO_NAME = "rough_structure_reveal_45deg.mp4"
AUDIT_NAME = "rough_structure_reveal_45deg.audit.json"
MANIFEST_NAME = "MANIFEST.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def renderer_root() -> Path:
    candidates = [ROOT, ROOT.parent / "Centipede_IsaacLab_RL_Clean"]
    for candidate in candidates:
        if (
            (candidate / "scripts" / "render_structure_reveal.py").is_file()
            and (candidate / "assets" / "high_fidelity" / "source_Centipede_v1_CC0.blend").is_file()
        ):
            return candidate
    raise RuntimeError(
        "No complete structure-reveal renderer found. Keep Centipede_IsaacLab_RL_Clean "
        "beside the legacy project or run this launcher from the clean edition."
    )


def runtime_python() -> Path:
    configured = os.environ.get("CENTIPEDE_RENDER_PYTHON")
    candidates = [
        Path(configured).expanduser() if configured else None,
        Path.home() / "anaconda3" / "envs" / "isaaclab_50" / "python.exe",
        Path(sys.executable),
    ]
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate.resolve()
    raise RuntimeError("No Python runtime found for the renderer")


def verified_existing(output: Path, source_sha256: str) -> bool:
    video = output / VIDEO_NAME
    audit_path = output / AUDIT_NAME
    manifest_path = output / MANIFEST_NAME
    if not (video.is_file() and audit_path.is_file() and manifest_path.is_file()):
        return False
    try:
        audit = json.loads(audit_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return bool(
        audit.get("pass")
        and audit.get("frames") == 2329
        and manifest.get("fps") == 120
        and manifest.get("source_trace_sha256") == source_sha256
        and manifest.get("video_sha256") == sha256(video)
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render the canonical 120 fps collision-to-skeleton-to-skin demo."
    )
    parser.add_argument("--trace", type=Path, default=FIXTURE)
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--smoke", action="store_true", help="Render a 40-frame 640x360 chain test")
    parser.add_argument("--force", action="store_true", help="Re-render even when a verified result exists")
    parser.add_argument("--dry-run", action="store_true", help="Print the resolved command without rendering")
    args = parser.parse_args()

    trace = args.trace.resolve()
    if not trace.is_file():
        raise RuntimeError(f"Missing demo trace: {trace}")
    trace_sha256 = sha256(trace)
    if args.trace == FIXTURE and trace_sha256 != FIXTURE_SHA256:
        raise RuntimeError("Canonical demo trace hash drift")

    output = (
        args.out_dir.resolve()
        if args.out_dir is not None
        else (
            ROOT / "outputs" / "demo" / "structure_reveal_smoke"
            if args.smoke
            else DEFAULT_OUTPUT
        )
    )
    if not args.dry_run and not args.smoke and not args.force and verified_existing(output, trace_sha256):
        print(f"DEMO_READY: {output / VIDEO_NAME}")
        return 0
    if not args.dry_run and output.exists() and any(output.iterdir()) and not args.force:
        raise RuntimeError(f"Output is non-empty but not a verified demo; inspect it or pass --force: {output}")

    implementation = renderer_root()
    command = [
        str(runtime_python()),
        str(implementation / "scripts" / "render_structure_reveal.py"),
        "--alllink-npz",
        str(trace),
        "--out-dir",
        str(output),
        "--start-time-s",
        "0.6",
        "--fps",
        "120",
        "--render-fps",
        "40",
        "--interpolate-pose-trace",
        "--interpolate-video",
        "--width",
        "640" if args.smoke else "1280",
        "--height",
        "360" if args.smoke else "720",
    ]
    if args.smoke:
        command.extend(["--max-frames", "40"])
    if args.dry_run:
        print(json.dumps({"cwd": str(implementation), "command": command}, indent=2))
        return 0

    completed = subprocess.run(command, cwd=implementation, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"Structure-reveal renderer failed with exit code {completed.returncode}")
    audit = json.loads((output / AUDIT_NAME).read_text(encoding="utf-8"))
    if not audit.get("pass"):
        raise RuntimeError(f"Rendered video failed audit: {audit.get('checks')}")
    print(f"DEMO_READY: {output / VIDEO_NAME}")
    print(f"frames={audit['frames']} duration_s={audit['duration_s']} pass={audit['pass']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
