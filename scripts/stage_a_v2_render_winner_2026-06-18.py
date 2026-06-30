"""render_single_condition_video.py — render ONE diagnostic video per
IsaacLab boot. Designed for render_debug_phase3.

Why this exists:
    `render_robustness_diagnostic.py` rendered 20 conditions inside a
    single boot, calling `inner.reset()` between conditions. After the
    first reset-inside-the-loop, `annot.get_data()` returned zero
    buffers — all 20 condition MP4s were black. The known-good
    `render_policy_visual_audit.py` calls `env.reset()` exactly once
    before the capture loop and renders fine. So we follow that recipe
    literally: ONE condition per process, ONE reset, then capture.

    To control wave_num / body_imp / spawn_block without modifying env
    code, we monkey-patch the methods that the failed renderer also
    monkey-patches (`_apply_global_6d_action`, `_spawn_rng`). These are
    READ-ONLY w.r.t. files on disk — only the live env instance is
    perturbed.

Usage (one video per invocation):
    python -m scripts.sanity.render_single_condition_video \
        --wave-scale 2.0 --imp-scale 1.0 \
        --pin-block 5 --pin-y-strat 1 --no-jitter \
        --camera chase --duration-sec 5 \
        --out outputs/visual_audit/render_debug_phase3/videos/v01.mp4 \
        --out-json outputs/visual_audit/render_debug_phase3/sidecars/v01.json \
        --policy-name flat_baseline

This script is RENDER-ONLY. It does not edit env / reward / controller
source / asset / IsaacLab core. The supervision protocol allows new
sanity scripts under `scripts/sanity/` that read state but do not
modify the protected files; this script does not modify any.

Created 2026-05-11 for the render_debug_phase3 deliverable.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys


# ─────────────────────────────────────────────────────────────────────────
# 0. CLI
# ─────────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                 description=__doc__)

src = parser.add_mutually_exclusive_group()
src.add_argument("--checkpoint", type=str, default=None,
                 help="rl_games checkpoint .pth path. If omitted, runs the "
                      "zero_action CPG baseline (action=0 → scale=1.0).")
src.add_argument("--zero-action", action="store_true",
                 help="Explicit zero_action baseline. Same as omitting --checkpoint.")

parser.add_argument("--policy-name", type=str, required=True,
                    help="Short label written to the JSON sidecar.")
parser.add_argument("--task", type=str,
                    default="Isaac-Centipede-Stage-B-Transition-v0")
parser.add_argument("--seed", type=int, default=42)

# Pass-N flags (kept for parity with render_policy_visual_audit.py)
parser.add_argument("--pass11-cot",                action="store_true", default=False)
parser.add_argument("--pass10-effort",             action="store_true", default=False)
parser.add_argument("--pass12-speed-floor",        action="store_true", default=False)
parser.add_argument("--pass12-structural-guard",   action="store_true", default=False)
parser.add_argument("--pass12-ang-split",          action="store_true", default=False)
parser.add_argument("--pass12-cot-bounded",        action="store_true", default=False)
parser.add_argument("--pass12-functional-guard",   action="store_true", default=False)
# Pass 13 is left at its env-cfg default (OFF); no CLI hook here so this
# script cannot enable it.

# ── Controller-pin (applied every step via apply_global_6d_modulation) ──
parser.add_argument("--body-amp-scale",  type=float, default=1.0)
parser.add_argument("--leg-yaw-scale",   type=float, default=1.0)
parser.add_argument("--leg-pitch-scale", type=float, default=1.0)
parser.add_argument("--wave-scale",      type=float, default=1.0,
                    help="wave_number_scale (== a3 in global_6d). 1.0 = "
                         "controller default baseline.")
parser.add_argument("--imp-scale",       type=float, default=1.0,
                    help="body_imp_scale (== a4). 1.0 = baseline.")
parser.add_argument("--freq-scale",      type=float, default=1.0,
                    help="freq_scale (== a5). Pinned at 1.0 per project "
                         "policy (do not exploit gait frequency).")

# ── Spawn-pin (deterministic block × y-stratum) ──
parser.add_argument("--pin-block", type=int, default=None,
                    help="If set, force every reset to spawn on this block "
                         "(0..5). Block labels: rough_a, rough_b, medium, "
                         "intermediate, long_wave, flat.")
parser.add_argument("--pin-y-strat", type=int, default=1,
                    help="y-stratum index when --pin-block is set. 0..3 → "
                         "y center ∈ (-0.6, -0.2, +0.2, +0.6).")
parser.add_argument("--n-y-strata", type=int, default=4,
                    choices=[4, 5],
                    help="Number of y-strata to configure in the env cfg. "
                         "4 = env default (-0.6, -0.2, 0.2, 0.6). "
                         "5 = Phase 3B sweep convention (-0.7, -0.35, 0.0, "
                         "0.35, 0.7); required when --pin-y-strat = 4.")
parser.add_argument("--no-jitter", action="store_true",
                    help="Zero out transition_spawn_jitter_x/y BEFORE env "
                         "construction. Combined with --pin-block, every "
                         "reset lands at the same (x, y, z).")
parser.add_argument("--spawn-seed", type=int, default=None,
                    help="Override CentipedeEnvCfg.transition_spawn_seed.")

# ── Homogeneous-terrain override (Phase 3, 2026-05-11) ──
# When --homogeneous-lambda-mm is set, monkey-patch the arena generator to
# produce a SINGLE-WAVELENGTH heightmap instead of the composite 6-block one.
# Matches the pattern in scripts/sanity/sweep_phase3_absolute_wave.py.
parser.add_argument("--homogeneous-lambda-mm", type=float, default=None,
                    help="If set, replace transition arena with a homogeneous "
                         "single-wavelength heightmap. Pass --pin-block to a "
                         "valid block (used only for the spawn-x lookup; the "
                         "underlying terrain is identical everywhere).")
parser.add_argument("--homogeneous-amp-mm", type=float, default=6.0,
                    help="Amplitude (mm) for the homogeneous arena. Ignored "
                         "unless --homogeneous-lambda-mm is set.")
parser.add_argument("--homogeneous-seed", type=int, default=4242,
                    help="Heightmap seed for the homogeneous arena.")
parser.add_argument("--freesearch-gains-json", type=str, default=None,
                    help="Stage-A v2 free-search gains json ({'gains':[{body_kp,body_kv,"
                         "pitch_kp,pitch_kv,roll_kp,roll_kv,leg_kp[4],leg_kv[4],effort[7],"
                         "body_amp,leg_yaw_amp,leg_pitch_amp}]}). When set, the controller "
                         "is pinned to these per-group gains via set_freesearch_runtime "
                         "(mirrors the sweep cell's fixed_apply) + head-servo OFF.")
parser.add_argument("--usd", type=str, default=None,
                    help="USD override (rebuilt+renamed model) via env_cfg.usd_path_override.")
parser.add_argument("--freq-hz", type=float, default=1.0,
                    help="CPG gait frequency for the free-search render (Stage-A s3 = 1.0 Hz).")
# --- Bench demo (GO_FINAL_5WAY_2026-06-21): large no-edge arena + central spawn. DEFAULT OFF. ---
parser.add_argument("--arena-size-m", type=float, default=None,
                    help="Bench demo: large no-edge arena extent (e.g. 9.0). None = cfg default (2.2 m).")
parser.add_argument("--central-spawn", action="store_true",
                    help="Bench demo: single central block (x=0) so the robot walks -x with the full arena ahead "
                         "(no run-off). Use with --pin-block 0.")
parser.add_argument("--image-size", type=int, default=None,
                    help="Bench demo: heightmap/mesh px (fine terrain on the larger arena; e.g. 1536).")
parser.add_argument("--force-wave-abs", type=float, default=None,
                    help="Demo: force the open-loop wave applied_abs (e.g. 2.0, 1.5) in the free-search path; "
                         "wave_number_scale = force_wave_abs / 3.0. None = wave_number_scale 1.0.")
# Black-frame rebound fix (docs/RENDERING_PIPELINE_DEBUG_NOTES.md §1): non-baseline wave
# on FLAT → deterministic all-black; re-create the render product + annotator AFTER warmup
# on the live post-reset camera prim. Default ON for this top-10 render (the flat panel is
# exactly the failure case). Mirrors p4_v34_video_render_bigblock's --reattach-after-warmup.
parser.add_argument("--reattach-after-warmup", dest="reattach", action="store_true", default=True,
                    help="Re-create render product + annotator after warmup (flat-panel rebound fix).")
parser.add_argument("--no-reattach-after-warmup", dest="reattach", action="store_false",
                    help="Disable the post-warmup re-attach.")
parser.add_argument("--absolute-wave-number", type=float, default=None,
                    help="If set, override --wave-scale = abs_wave / 3.0. "
                         "Used by the Phase 3 video renderer to keep "
                         "filenames and JSON metadata in the 'absolute wave "
                         "number' convention.")

# ── Camera ──
parser.add_argument("--camera", default="chase",
                    choices=["oblique", "top", "side", "medium", "rough_b",
                             "flat", "long_wave", "intermediate", "chase"],
                    help="'chase' (default) follows the agent — best for "
                         "single-agent gait inspection.")
parser.add_argument("--cam-eye",    nargs=3, type=float, default=None)
parser.add_argument("--cam-target", nargs=3, type=float, default=None)
parser.add_argument("--width",  type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--video-fps",    type=int,   default=20)
parser.add_argument("--duration-sec", type=float, default=5.0)
parser.add_argument("--n-warmup",     type=int,   default=30,
                    help="Render warmup frames (RTX raytrace converges in "
                         "~20-30 frames; the known-good audit used 30).")
parser.add_argument("--steps-warmup-phys", type=int, default=240,
                    help="Pre-capture physics warmup substeps (4 ms each → "
                         "240 = 0.96 s wallclock). Lets the centipede "
                         "settle on the heightmap before frame 0.")
parser.add_argument("--dome-intensity",    type=float, default=1500.0)
parser.add_argument("--distant-intensity", type=float, default=3000.0)
parser.add_argument("--chase-offset", nargs=3, type=float,
                    default=(-0.25, -0.20, 0.18),
                    help="Chase camera eye offset (m) from smoothed root. "
                         "Default matches the Pass-12 close-up follow that "
                         "passed frame audit at 0 black / 0 frozen.")
parser.add_argument("--chase-alpha", type=float, default=0.85)

parser.add_argument("--num-envs", type=int, default=1)
parser.add_argument("--out",      type=str, required=True,
                    help="Output MP4 path.")
parser.add_argument("--out-json", type=str, default=None,
                    help="Optional JSON sidecar with metadata + metrics.")
parser.add_argument("--dump-raw-alllink", action="store_true",
                    help="GO_OFFLINE_RENDERER_2026-06-22 §4: ALSO save <out>_alllink.npz = full per-link "
                         "body_pos_w/body_quat_w/body_names of THIS render's physics, every captured frame, so "
                         "the offline renderer reproduces the IDENTICAL motion (same-run Omniverse-vs-offline "
                         "comparison). Additive, default OFF.")

if "--enable_cameras" not in sys.argv:
    sys.argv.append("--enable_cameras")
if "--headless" not in sys.argv:
    sys.argv.append("--headless")

from isaaclab.app import AppLauncher  # noqa: E402
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
sim_app = app_launcher.app

# ─────────────────────────────────────────────────────────────────────────
# 1. RTX raytrace
# ─────────────────────────────────────────────────────────────────────────
import carb  # noqa: E402
_carb = carb.settings.get_settings()
_carb.set_string("/rtx/rendermode", "RaytracedLighting")
_carb.set_bool("/isaaclab/render/rtx_sensors", True)
print("[viz] forced /rtx/rendermode = RaytracedLighting", flush=True)

# ─────────────────────────────────────────────────────────────────────────
# 2. kit-touching imports
# ─────────────────────────────────────────────────────────────────────────
import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
import gymnasium as gym  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402
import omni.replicator.core as rep  # noqa: E402

SCRIPT_DIR   = os.path.abspath(os.path.dirname(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, os.pardir, os.pardir))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


# ─────────────────────────────────────────────────────────────────────────
# 3. Helpers (verbatim from render_policy_visual_audit.py)
# ─────────────────────────────────────────────────────────────────────────
def add_lighting(stage, dome_intensity: float, distant_intensity: float):
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.GetIntensityAttr().Set(dome_intensity)
    dome.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    dist_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    dist_light.GetIntensityAttr().Set(distant_intensity)
    dist_light.GetAngleAttr().Set(0.53)
    dist_light.GetColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.92))
    xf = UsdGeom.Xformable(dist_light.GetPrim())
    xf.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))


def lookat_matrix(eye, target, up=(0.0, 0.0, 1.0)):
    e = np.asarray(eye, dtype=np.float64)
    t = np.asarray(target, dtype=np.float64)
    f = (t - e); f /= (np.linalg.norm(f) or 1.0)
    u = np.asarray(up, dtype=np.float64); u /= (np.linalg.norm(u) or 1.0)
    r = np.cross(f, u); rn = np.linalg.norm(r) or 1.0; r = r / rn
    u2 = np.cross(r, f)
    M = np.eye(4, dtype=np.float64)
    M[:3, 0] = r; M[:3, 1] = u2; M[:3, 2] = -f; M[:3, 3] = e
    return M


def place_camera(stage, cam_path: str, eye, target):
    cam = UsdGeom.Camera.Define(stage, cam_path)
    M = lookat_matrix(eye, target)
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    op = xf.AddTransformOp()
    op.Set(Gf.Matrix4d(*M.T.flatten().tolist()))
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
    cam.GetFocalLengthAttr().Set(24.0)
    cam.GetHorizontalApertureAttr().Set(20.955)
    return cam_path, op


def update_camera_xform(xform_op, eye, target):
    M = lookat_matrix(eye, target)
    xform_op.Set(Gf.Matrix4d(*M.T.flatten().tolist()))


def attach_rgb_annotator(cam_path: str, width: int, height: int):
    rp = rep.create.render_product(cam_path, resolution=(width, height))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    return annot


def annotator_to_uint8_rgb(rgb):
    if rgb is None:
        return None
    arr = np.asarray(rgb)
    if arr.dtype != np.uint8:
        arr = (np.clip(arr, 0.0, 1.0) * 255.0).astype(np.uint8)
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    return np.ascontiguousarray(arr)


# Camera presets — arena-frame fixed lookat. The block-aligned presets are
# useful when the agent is pinned to that block and would otherwise sit
# off-camera in the wide oblique view.
CAMERA_PRESETS = {
    "oblique":      dict(eye=(0.0,   -1.9, 1.0),  target=(0.0,    0.0, 0.02)),
    "top":          dict(eye=(0.0,    0.0, 2.5),  target=(0.0,    0.0, 0.00)),
    "side":         dict(eye=(0.0,   -1.5, 0.10), target=(0.0,    0.0, 0.05)),
    "flat":         dict(eye=(0.917, -0.6, 0.35), target=(0.917,  0.0, 0.02)),
    "long_wave":    dict(eye=(0.550, -0.6, 0.35), target=(0.550,  0.0, 0.02)),
    "intermediate": dict(eye=(0.183, -0.6, 0.35), target=(0.183,  0.0, 0.02)),
    "medium":       dict(eye=(-0.183,-0.6, 0.35), target=(-0.183, 0.0, 0.02)),
    "rough_b":      dict(eye=(-0.55, -0.6, 0.35), target=(-0.55,  0.0, 0.02)),
}

# Block index → label / λ_mm / amp_mm (matches env_cfg.transition_block_labels)
BLOCK_LABELS  = ("rough_a", "rough_b", "medium", "intermediate", "long_wave", "flat")
BLOCK_LAMBDA  = (18.0, 18.0, 30.0, 50.0, 60.0, 60.0)
BLOCK_AMP     = (10.0, 10.0, 10.0,  8.0,  6.0,  0.0)
BLOCK_SEV     = (10.0/18.0, 10.0/18.0, 10.0/30.0, 8.0/50.0, 6.0/60.0, 0.0)


class PinnedSpawnRng:
    """Drop-in replacement for env._spawn_rng. Returns a fixed
    (block_idx, y_strat_idx) pair on every .integers() call, and zero
    on every .uniform() call. The env's _reset_idx loops over envs and
    calls .integers(0, n_blocks) then .integers(0, n_y_strata) per env,
    interleaved with two .uniform() calls for jitter — see
    envs/centipede_env.py:1522-1527."""

    def __init__(self, block_idx: int, y_strat_idx: int = 1):
        self.block_idx = int(block_idx)
        self.y_strat_idx = int(y_strat_idx)
        self._integers_count = 0

    def integers(self, low, high):
        v = self.block_idx if (self._integers_count % 2 == 0) else self.y_strat_idx
        self._integers_count += 1
        return int(v)

    def uniform(self, low, high):
        return 0.0


# ─────────────────────────────────────────────────────────────────────────
# 4. Main — one-condition recipe (mirrors render_policy_visual_audit.py)
# ─────────────────────────────────────────────────────────────────────────
def main() -> int:
    if args.checkpoint is not None and not os.path.isfile(args.checkpoint):
        print(f"[viz] ERROR: checkpoint missing: {args.checkpoint}", flush=True)
        return 2

    out_path = os.path.abspath(args.out)
    out_dir  = os.path.dirname(out_path)
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.abspath(args.out_json) if args.out_json else None
    if json_path:
        os.makedirs(os.path.dirname(json_path), exist_ok=True)
    print(f"[viz] policy='{args.policy_name}'  camera='{args.camera}'  "
          f"out='{out_path}'", flush=True)
    print(f"[viz] scales  body_amp={args.body_amp_scale}  "
          f"leg_yaw={args.leg_yaw_scale}  leg_pitch={args.leg_pitch_scale}  "
          f"wave={args.wave_scale}  imp={args.imp_scale}  freq={args.freq_scale}",
          flush=True)

    # If --absolute-wave-number is set, override the wave_scale arg accordingly.
    if args.absolute_wave_number is not None:
        args.wave_scale = float(args.absolute_wave_number) / 3.0
        print(f"[viz] absolute_wave_number={args.absolute_wave_number} → "
              f"wave_scale={args.wave_scale:.4f} (baseline n_wave_scalar=3.0)",
              flush=True)

    # Install homogeneous-terrain monkey-patch BEFORE env construction.
    if args.homogeneous_lambda_mm is not None:
        ARENA_DIR = os.path.join(PROJECT_ROOT, "scripts", "optimization")
        if ARENA_DIR not in sys.path:
            sys.path.insert(0, ARENA_DIR)
        import arena as arena_module  # type: ignore
        from terrain import generate_single_wavelength_heightmap  # type: ignore
        captured_lambda_mm = float(args.homogeneous_lambda_mm)
        captured_amp_mm    = float(args.homogeneous_amp_mm)
        captured_seed      = int(args.homogeneous_seed)
        # signature MUST match the sweep cell's (keyword defaults) — the arena
        # module calls this without all positionals; required positionals raise
        # "missing 2 required positional arguments" (observed 2026-06-19).
        def homogeneous(arena_size_m, rough_wavelength_mm=None, rough_amplitude_m=None,
                        seed=None, image_size=1024, **kwargs):
            _ = (rough_wavelength_mm, rough_amplitude_m, seed)
            if captured_amp_mm <= 0.0:
                h = np.zeros((int(image_size), int(image_size)), dtype=np.float64)
            else:
                h, _rms, _peak = generate_single_wavelength_heightmap(
                    wavelength_m = captured_lambda_mm * 1e-3,
                    amplitude_m  = captured_amp_mm * 1e-3,
                    seed         = captured_seed,
                    image_size   = int(image_size),
                    world_size   = float(arena_size_m),
                )
            return h, []
        arena_module.generate_flat_to_rough_transition_heightmap = homogeneous
        print(f"[viz] HOMOGENEOUS arena: λ={captured_lambda_mm} mm  "
              f"amp={captured_amp_mm} mm  seed={captured_seed}", flush=True)

    from envs.centipede_env_cfg import CentipedeEnvCfg
    from tasks import register_centipede_tasks
    register_centipede_tasks()

    env_cfg = CentipedeEnvCfg()
    env_cfg.scene.num_envs = int(args.num_envs)
    env_cfg.terrain_kind = "transition"
    env_cfg.seed = int(args.seed)
    # Stage-A v2: select the rebuilt+renamed model (program-consistency with the search).
    if args.usd is not None:
        env_cfg.usd_path_override = args.usd
        _usd_eff = env_cfg.resolve_asset_override()
        print(f"[viz] USD override in effect = {_usd_eff}", flush=True)
    if args.spawn_seed is not None:
        env_cfg.transition_spawn_seed = int(args.spawn_seed)
    if args.no_jitter:
        # Zeroing jitter BEFORE construction — this is a cfg field, not
        # protected env code. Combined with PinnedSpawnRng below, every
        # reset lands at exactly the block centre × y-stratum centre.
        env_cfg.transition_spawn_jitter_x = 0.0
        env_cfg.transition_spawn_jitter_y = 0.0
        print("[viz] spawn jitter zeroed via cfg override", flush=True)

    # Phase 3B compatibility: when 5 y-strata are requested (i.e. the
    # sweep used n_y_strata=5 with centres (-0.7, -0.35, 0, 0.35, 0.7)),
    # expand the env cfg to match so --pin-y-strat ∈ {0..4} is valid.
    if int(args.n_y_strata) == 5:
        env_cfg.transition_spawn_n_y_strata = 5
        env_cfg.transition_spawn_y_strata_centres = (-0.7, -0.35, 0.0, 0.35, 0.7)
        print("[viz] Phase 3B: n_y_strata=5 centres=(-0.7, -0.35, 0.0, 0.35, 0.7)",
              flush=True)

    # ---- Bench demo (GO_FINAL_5WAY): large no-edge arena + central spawn (default OFF) ----
    if getattr(args, "arena_size_m", None) is not None:
        env_cfg.transition_arena_size_m = float(args.arena_size_m)
    if getattr(args, "image_size", None) is not None:
        env_cfg.transition_image_size = int(args.image_size)
    if getattr(args, "central_spawn", False):
        env_cfg.transition_block_centers_x = (0.0,)   # single central x → walk -x with full arena ahead
        print(f"[viz] BENCH central spawn: arena {env_cfg.transition_arena_size_m}m x=0 "
              f"(pin-block 0)", flush=True)

    env_cfg.pass11_cot_enabled              = bool(args.pass11_cot)
    env_cfg.pass10_effort_enabled           = bool(args.pass10_effort)
    env_cfg.pass12_speed_floor_enabled      = bool(args.pass12_speed_floor)
    env_cfg.pass12_structural_guard_enabled = bool(args.pass12_structural_guard)
    env_cfg.pass12_ang_split_enabled        = bool(args.pass12_ang_split)
    env_cfg.pass12_cot_bounded_enabled      = bool(args.pass12_cot_bounded)
    env_cfg.pass12_functional_guard_enabled = bool(args.pass12_functional_guard)

    # Extend the episode so a LONG render (e.g. 16 s) does NOT truncate + re-spawn mid-clip
    # (the default 8 s episode resets ~7 s into a 16 s capture, teleporting the robot and
    # corrupting the HUD/fwd metrics). Cover warmup + capture + margin. (2026-06-20.)
    _dt_pol_pre = int(env_cfg.decimation) * float(env_cfg.sim.dt)
    _n_cap = int(round(float(args.duration_sec) * float(args.video_fps)))
    _n_warm_pol = int(args.steps_warmup_phys // max(int(env_cfg.decimation), 1))
    env_cfg.episode_length_s = (_n_warm_pol + _n_cap + 30) * _dt_pol_pre
    print(f"[viz] episode_length_s -> {env_cfg.episode_length_s:.1f}s "
          f"(warmup {_n_warm_pol} + capture {_n_cap} + 30 margin; prevents mid-clip reset)", flush=True)

    inner = gym.make(args.task, cfg=env_cfg, render_mode=None)
    base_env = inner.unwrapped
    decimation = int(env_cfg.decimation)
    sim_dt = float(env_cfg.sim.dt)
    E = base_env.num_envs
    print(f"[viz] env constructed  num_envs={E}  device={base_env.device}  "
          f"decimation={decimation}  sim_dt={sim_dt}", flush=True)

    # Lighting + camera + annotator — BEFORE first reset, matching the
    # known-good order from render_policy_visual_audit.py.
    stage = base_env.sim.stage
    add_lighting(stage, args.dome_intensity, args.distant_intensity)
    # For the initial camera placement we use a sensible fixed preset (or
    # the user-provided override). Chase mode will move the camera per
    # frame; the initial pose just has to be non-degenerate.
    if args.camera == "chase":
        # Pick a sensible initial lookat near the pinned block (if any),
        # else over the arena origin.
        if args.pin_block is not None:
            cx = (-0.917, -0.550, -0.183, +0.183, +0.550, +0.917)[args.pin_block]
            init_eye    = (cx + args.chase_offset[0], args.chase_offset[1], 0.18)
            init_target = (cx, 0.0, 0.02)
        else:
            init_eye    = (0.0,  -0.6, 0.5)
            init_target = (0.0,  0.0,  0.02)
    else:
        preset = CAMERA_PRESETS[args.camera]
        init_eye = tuple(args.cam_eye)    if args.cam_eye    else preset["eye"]
        init_target = tuple(args.cam_target) if args.cam_target else preset["target"]
    print(f"[viz] camera '{args.camera}'  init_eye={tuple(round(e, 3) for e in init_eye)}  "
          f"init_target={tuple(round(t, 3) for t in init_target)}", flush=True)
    cam_path, cam_xform_op = place_camera(stage, "/World/DiagCamera",
                                          init_eye, init_target)
    annot = attach_rgb_annotator(cam_path, args.width, args.height)

    # Build rl_games wrapper (only used when --checkpoint is given, but
    # the same wrapper imports also handle zero_action because we build
    # `env` for both paths and reset it).
    from rl_games.common import env_configurations, vecenv
    from isaaclab_rl.rl_games import (
        RlGamesGpuEnv, RlGamesVecEnvWrapper,
    )

    spec = gym.spec(args.task)
    yaml_path = spec.kwargs["rl_games_cfg_entry_point"]
    with open(yaml_path, "r", encoding="utf-8") as f:
        agent_cfg = yaml.safe_load(f)
    rl_device = agent_cfg["params"]["config"]["device"]
    clip_obs = agent_cfg["params"]["env"].get("clip_observations", math.inf)
    clip_actions = agent_cfg["params"]["env"].get("clip_actions", math.inf)

    env = RlGamesVecEnvWrapper(inner, rl_device, clip_obs, clip_actions)
    vecenv.register(
        "IsaacRlgWrapper",
        lambda config_name, num_actors, **kwargs: RlGamesGpuEnv(
            config_name, num_actors, **kwargs),
    )
    env_configurations.register(
        "rlgpu",
        {"vecenv_type": "IsaacRlgWrapper",
         "env_creator": lambda **kwargs: env},
    )

    # ── Monkey-patch _spawn_rng if --pin-block was requested ──
    pinned_spawn = args.pin_block is not None
    if pinned_spawn:
        base_env._spawn_rng = PinnedSpawnRng(
            block_idx=int(args.pin_block),
            y_strat_idx=int(args.pin_y_strat))
        print(f"[viz] spawn pinned to block={args.pin_block} "
              f"({BLOCK_LABELS[args.pin_block]})  y_strat={args.pin_y_strat}",
              flush=True)

    # ── Stage-A v2 FREE-SEARCH gains path (mirrors stage_a_vec_sweep_cell fixed_apply) ──
    # When --freesearch-gains-json is given, pin the controller to the winner's
    # per-group kp/kv + per-class effort + amplitudes via set_freesearch_runtime,
    # head-servo OFF, gait freq = --freq-hz. This reproduces EXACTLY what the
    # search scored (no global-6d scale path).
    if args.freesearch_gains_json is not None:
        with open(args.freesearch_gains_json, "r", encoding="utf-8") as _f:
            _g = json.load(_f)["gains"][0]   # single-env render
        ctrl = base_env.controller
        ctrl.head_yaw_gain = 0.0
        ctrl.set_frequency(float(args.freq_hz))
        dev = base_env.device
        D = ctrl.dtype
        n = base_env.num_envs
        def _v(x):
            return torch.full((n,), float(x), dtype=torch.float32, device=dev)
        def _v4(lst):
            return torch.tensor([[float(x) for x in lst]] * n, dtype=torch.float32, device=dev)
        fs_body_kp=_v(_g["body_kp"]); fs_body_kv=_v(_g["body_kv"])
        fs_pitch_kp=_v(_g["pitch_kp"]); fs_pitch_kv=_v(_g["pitch_kv"])
        fs_roll_kp=_v(_g["roll_kp"]); fs_roll_kv=_v(_g["roll_kv"])
        fs_leg_kp=_v4(_g["leg_kp"]); fs_leg_kv=_v4(_g["leg_kv"])
        fs_body_amp=_v(_g["body_amp"]); fs_leg_yaw_amp=_v(_g["leg_yaw_amp"])
        fs_leg_pitch_amp=_v(_g["leg_pitch_amp"])
        _eff=_g["effort"]
        fs_eff_by=_v(_eff[0]); fs_eff_bp=_v(_eff[1]); fs_eff_br=_v(_eff[2])
        fs_eff_leg=_v4(_eff[3:7])
        fs_ones=_v(1.0)
        # forced open-loop wave (GO_ORIGIN_VS_G3E8_LOWWAVE demo): wave_number_scale = wave/3.0
        fs_wave = _v(float(args.force_wave_abs) / 3.0) if args.force_wave_abs is not None else fs_ones
        if args.force_wave_abs is not None:
            print(f"[viz] FORCED wave applied_abs={args.force_wave_abs} -> "
                  f"wave_number_scale={float(args.force_wave_abs)/3.0:.4f}", flush=True)
        def freesearch_apply(_actions):
            ctrl.apply_global_6d_modulation(
                body_amp_scale=fs_body_amp, leg_yaw_amp_scale=fs_leg_yaw_amp,
                leg_pitch_amp_scale=fs_leg_pitch_amp, wave_number_scale=fs_wave,
                body_imp_scale=fs_ones, freq_scale=fs_ones)
            ctrl.set_freesearch_runtime(
                body_kp=fs_body_kp.to(D), body_kv=fs_body_kv.to(D),
                pitch_kp=fs_pitch_kp.to(D), pitch_kv=fs_pitch_kv.to(D),
                roll_kp=fs_roll_kp.to(D), roll_kv=fs_roll_kv.to(D),
                leg_kp=fs_leg_kp.to(D), leg_kv=fs_leg_kv.to(D),
                effort_body_yaw=fs_eff_by.to(D), effort_body_pitch=fs_eff_bp.to(D),
                effort_body_roll=fs_eff_br.to(D), effort_leg=fs_eff_leg.to(D),
                enable_effort=True)
            if hasattr(base_env, "_u_target") and hasattr(base_env, "_u_applied"):
                u = torch.stack([fs_ones]*6, dim=1).to(base_env._u_applied.dtype)
                base_env._u_target.copy_(u); base_env._u_applied.copy_(u)
        base_env._apply_global_6d_action = freesearch_apply
        print(f"[viz] controller pinned to FREE-SEARCH gains from {args.freesearch_gains_json} "
              f"(head_yaw=0, freq={args.freq_hz}Hz)", flush=True)
        pin_controller = False   # free-search path owns the apply hook; skip global-6d path
    else:
        pin_controller = None

    # ── Monkey-patch the controller modulation if any scale != 1.0 ──
    # Same pattern as render_robustness_diagnostic.py & sweep runner.
    fixed_scales = (args.body_amp_scale, args.leg_yaw_scale, args.leg_pitch_scale,
                    args.wave_scale, args.imp_scale, args.freq_scale)
    if pin_controller is None:
        pin_controller = any(abs(s - 1.0) > 1e-9 for s in fixed_scales) or args.zero_action

    if pin_controller:
        def fixed_apply_action(_actions):
            n = base_env.num_envs
            dev = base_env.device
            def vec(x):
                return torch.full((n,), float(x), dtype=torch.float32, device=dev)
            base_env.controller.apply_global_6d_modulation(
                body_amp_scale      = vec(fixed_scales[0]),
                leg_yaw_amp_scale   = vec(fixed_scales[1]),
                leg_pitch_amp_scale = vec(fixed_scales[2]),
                wave_number_scale   = vec(fixed_scales[3]),
                body_imp_scale      = vec(fixed_scales[4]),
                freq_scale          = vec(fixed_scales[5]),
            )
            if hasattr(base_env, "_u_target") and hasattr(base_env, "_u_applied"):
                u = torch.tensor(list(fixed_scales),
                                 dtype=base_env._u_applied.dtype,
                                 device=base_env._u_applied.device)
                u = u.unsqueeze(0).expand(n, 6).contiguous()
                base_env._u_target.copy_(u)
                base_env._u_applied.copy_(u)
        base_env._apply_global_6d_action = fixed_apply_action
        print(f"[viz] controller pinned to scales {fixed_scales}", flush=True)

    # ── Build action provider (zero_action vs policy) ──
    action_provider = None  # default: pass zeros into env.step (overridden by patch)
    if args.checkpoint is not None and not args.zero_action:
        from rl_games.torch_runner import Runner
        agent_cfg["params"]["seed"] = int(args.seed)
        agent_cfg["params"]["config"]["num_actors"] = E
        agent_cfg["params"]["load_checkpoint"] = True
        agent_cfg["params"]["load_path"] = os.path.abspath(args.checkpoint)
        agent_cfg["params"]["config"]["save_best_after"] = 10**9
        agent_cfg["params"]["config"]["save_frequency"] = 10**9
        runner = Runner()
        runner.load(agent_cfg)
        player = runner.create_player()
        player.restore(os.path.abspath(args.checkpoint))
        player.reset()
        print(f"[viz] policy loaded from {args.checkpoint}", flush=True)

        obs0 = env.reset()
        if isinstance(obs0, tuple):
            obs0 = obs0[0]
        pol = obs0["obs"] if isinstance(obs0, dict) and "obs" in obs0 else obs0
        _ = player.get_batch_size(player.obs_to_torch(pol), 1)
        if player.is_rnn:
            player.init_rnn()

        def policy_provider(obs_x):
            return player.get_action(player.obs_to_torch(obs_x),
                                     is_deterministic=True)
        action_provider = policy_provider
    else:
        # zero_action / pinned-scales path: just reset once and step zeros
        obs0 = env.reset()
        if isinstance(obs0, tuple):
            obs0 = obs0[0]

    # ── Diagnostics: read spawn block id + xyz ──
    try:
        sb = base_env._spawn_block_id.detach().cpu().numpy().astype(int)
        spawn_xyz = base_env.robot.data.root_pos_w[0].detach().cpu().numpy()
        print(f"[viz] post-reset  spawn_block_id={sb.tolist()}  "
              f"spawn_xyz={tuple(spawn_xyz.round(4).tolist())}", flush=True)
    except Exception as exc:
        print(f"[viz] spawn diagnostics read failed: {exc}", flush=True)
        spawn_xyz = np.zeros(3)
        sb = np.array([-1])

    # Physics warmup (settle on heightmap before recording).
    n_phys_warm = int(args.steps_warmup_phys // max(decimation, 1))
    print(f"[viz] physics warmup: {n_phys_warm} policy steps "
          f"({n_phys_warm * decimation} substeps)", flush=True)
    for _ in range(n_phys_warm):
        a = torch.zeros((E, env_cfg.action_space), dtype=torch.float32,
                        device=base_env.device)
        env.step(a)

    # Render warmup (RTX raytrace converges).
    print(f"[viz] render warmup: {args.n_warmup}", flush=True)
    for _ in range(args.n_warmup):
        base_env.sim.render()

    # ── Black-frame rebound fix: re-attach render product + annotator on the live
    #    post-warmup camera prim (the flat / non-baseline-wave all-black failure case). ──
    if getattr(args, "reattach", True):
        try:
            annot = attach_rgb_annotator(cam_path, args.width, args.height)
            for _ in range(max(10, args.n_warmup // 2)):
                base_env.sim.render()
            _probe = annotator_to_uint8_rgb(annot.get_data())
            _pmax = int(_probe.max()) if (_probe is not None and _probe.size) else 0
            print(f"[viz] re-attached annotator after warmup (probe max={_pmax})", flush=True)
        except Exception as _e:
            print(f"[viz] reattach FAILED: {type(_e).__name__}: {_e}", flush=True)

    # ── HUD per-frame metric capture — HONEST BODY-PCA (GO_R1_TRACKING_AND_TOP10) ──
    # net_yaw = body PRINCIPAL-AXIS net rotation (NOT head ∫ω_z — the head-wag artifact, retired);
    # tip-over/roll = body-median over spine segments; v_fwd = -Δx_world/T; body-wave = std of
    # body-wave joint angles. Captured per frame → sidecar 'per_frame'.
    _ctrl = base_env.controller
    try:
        _body_ids = _ctrl.body_joint_ids.detach().cpu().numpy()
        _body_wave_ids = _body_ids[1:] if len(_body_ids) > 1 else _body_ids   # drop head
    except Exception:
        _body_wave_ids = np.array([], dtype=int)
    _bn = list(base_env.robot.data.body_names)
    _spine_ids = [_bn.index(f"link_body_{i}") for i in range(64) if f"link_body_{i}" in _bn]
    print(f"[viz] body-PCA HUD: {len(_spine_ids)} spine segments", flush=True)
    hud_rows = []   # per frame: dict(x, spine_xy(S,2), seg_upz(S), roll_deg, bodywave_std)
    _ALLLINK = bool(getattr(args, "dump_raw_alllink", False))   # GO_OFFLINE_RENDERER §4
    _alllink_pos, _alllink_quat = [], []                        # per-frame full (nb,3)/(nb,4)

    # Capture loop
    n_frames = int(round(args.duration_sec * args.video_fps))
    print(f"[viz] capturing {n_frames} frames at {args.video_fps} fps "
          f"({args.duration_sec:.1f} s)", flush=True)

    chase = (args.camera == "chase")
    chase_offset = np.asarray(args.chase_offset, dtype=np.float64)
    smoothed_target = None
    if chase:
        root_w = base_env.robot.data.root_pos_w[0].detach().cpu().numpy().astype(np.float64)
        smoothed_target = root_w.copy()
        eye_now = smoothed_target + chase_offset
        update_camera_xform(cam_xform_op, tuple(eye_now), tuple(smoothed_target))
        print(f"[viz] chase init  target={tuple(smoothed_target.round(3))}  "
              f"eye={tuple(eye_now.round(3))}", flush=True)

    # Per-step physics metrics (independent of RGB pipeline — these
    # remain valid even if videos fail visual audit, mirroring the
    # quantitative-data approach used by render_robustness_diagnostic.py).
    DT_POL = decimation * sim_dt
    prev_x = float(base_env.robot.data.root_pos_w[0, 0].item())
    x0 = prev_x
    F_max_seen = 0.0
    vz_sq_sum = 0.0
    z_min = float(base_env.robot.data.root_pos_w[0, 2].item())
    z_max = z_min
    n_dones = 0
    n_valid_steps = 0

    frames: list[np.ndarray] = []
    n_failed = 0
    obs = obs0  # already reset; never reset again inside the loop

    for f in range(n_frames):
        if action_provider is not None:
            pol = obs["obs"] if isinstance(obs, dict) and "obs" in obs else obs
            actions = action_provider(pol)
        else:
            actions = torch.zeros((E, env_cfg.action_space),
                                  dtype=torch.float32, device=base_env.device)
        out = env.step(actions)
        if isinstance(out, tuple) and len(out) == 5:
            obs, _r, _term, _trunc, _info = out
        elif isinstance(out, tuple) and len(out) == 4:
            obs, _r, _done, _info = out
        else:
            obs = out

        if chase and smoothed_target is not None:
            try:
                root_w = base_env.robot.data.root_pos_w[0].detach().cpu().numpy().astype(np.float64)
                smoothed_target = (args.chase_alpha * smoothed_target
                                   + (1.0 - args.chase_alpha) * root_w)
                eye_now = smoothed_target + chase_offset
                update_camera_xform(cam_xform_op, tuple(eye_now), tuple(smoothed_target))
            except Exception:
                pass

        base_env.sim.render()
        rgb_raw = annot.get_data()
        rgb = annotator_to_uint8_rgb(rgb_raw)
        # Content-aware validity check (mandated by docs/RENDERING_PIPELINE_DEBUG_NOTES.md §2):
        # Phase 2 black-frame race produced non-empty zero buffers that the
        # `rgb is None or rgb.size == 0` check let through. Now also require
        # rgb.max() > 0 — i.e. the buffer must contain non-zero pixels.
        # If invalid, retry sim.render() + get_data() a few times before
        # substituting the LAST VALID frame.
        first_valid = (rgb is not None and rgb.size > 0 and int(rgb.max()) > 0)
        if not first_valid:
            for _r in range(4):
                base_env.sim.render()
                rgb_r = annotator_to_uint8_rgb(annot.get_data())
                if rgb_r is not None and rgb_r.size > 0 and int(rgb_r.max()) > 0:
                    rgb = rgb_r
                    first_valid = True
                    break
        if not first_valid:
            n_failed += 1
            if frames:
                rgb = frames[-1].copy()
            else:
                rgb = np.zeros((args.height, args.width, 3), dtype=np.uint8)
        frames.append(rgb)

        # ── INLINE EARLY-ABORT (2026-06-22, RENDERING_PIPELINE_DEBUG_NOTES): if the first EARLY_N+1
        #    frames are ALL uniformly black/white/blank, the RTX buffer race lost this boot. ABORT NOW
        #    (os._exit) — do NOT capture+encode the full clip then hang on Omniverse teardown to the
        #    watchdog deadline (~12 min wasted PER bad render). The driver re-boots on the missing mp4,
        #    so a failure now costs ~boot+0.6 s, not ~12 min. Color-agnostic (matches the hardened audit:
        #    mean<8 black / mean>240 white / std<6 blank). Try-wrapped so a check bug can NEVER break a
        #    GOOD render; only fires when ALL early frames are definitively bad. ──
        EARLY_N = 12
        if f == EARLY_N:
            try:
                # GO_OFFLINE_RENDERER_ZFIX §4: extend early-abort to REGION (partial/gradient-white) +
                # TEMPORAL (strobe) so a half-white or strobing boot also aborts in ~13 frames, not after
                # the full clip. Color-agnostic; fires only when ALL/most early frames are definitively bad.
                _L, _S, _imb = [], [], []
                for _fk in frames[:EARLY_N + 1]:
                    _a = np.asarray(_fk, dtype=np.float32)[..., :3]
                    _lum = 0.299 * _a[..., 0] + 0.587 * _a[..., 1] + 0.114 * _a[..., 2]
                    _L.append(float(_lum.mean())); _S.append(float(_lum.std()))
                    _hh, _ww = _lum.shape; _st = (_lum > 250)
                    _qf = [float(_st[:_hh // 2, :_ww // 2].mean()), float(_st[:_hh // 2, _ww // 2:].mean()),
                           float(_st[_hh // 2:, :_ww // 2].mean()), float(_st[_hh // 2:, _ww // 2:].mean())]
                    _imb.append(max(_qf) - min(_qf))
                _L = np.asarray(_L); _S = np.asarray(_S); _imb = np.asarray(_imb)
                _uniform_bad = bool(((_L < 8) | (_L > 240) | (_S < 6)).all())       # all black/white/blank
                _partial_bad = bool((_imb > 0.30).all())                            # all partial/gradient-white
                _dj = np.abs(np.diff(_L))
                _strobe_bad = bool(len(_dj) and (_dj > 25).mean() > 0.6)            # most transitions strobe
                if _uniform_bad or _partial_bad or _strobe_bad:
                    _why = ("black/white/blank" if _uniform_bad else
                            "partial/gradient-white" if _partial_bad else "strobe/flicker")
                    print(f"[viz] EARLY-ABORT: first {EARLY_N + 1} frames bad [{_why}] "
                          f"(lum {_L.min():.0f}-{_L.max():.0f}, std_min {_S.min():.1f}, sat_imb_max {_imb.max():.2f}, "
                          f"strobe_frac {(_dj > 25).mean() if len(_dj) else 0:.2f}) — RTX race lost this boot; "
                          f"fast-exit for driver re-boot (no full-capture hang).", flush=True)
                    sys.stdout.flush(); sys.stderr.flush()
                    os._exit(2)
            except Exception as _eab:
                print(f"[viz] early-abort check skipped ({type(_eab).__name__}: {_eab})", flush=True)

        # Physics-side metrics
        cur_x = float(base_env.robot.data.root_pos_w[0, 0].item())
        cur_z = float(base_env.robot.data.root_pos_w[0, 2].item())
        vz    = float(base_env.robot.data.root_lin_vel_w[0, 2].item())
        try:
            cs = base_env.scene["contact_sensor"]
            F_arr = cs.data.net_forces_w[0].detach().cpu().numpy()
            Fmag  = float(np.linalg.norm(F_arr, axis=-1).sum())
            if Fmag > F_max_seen:
                F_max_seen = Fmag
        except Exception:
            pass
        vz_sq_sum += vz * vz
        z_min = min(z_min, cur_z); z_max = max(z_max, cur_z)
        dx_raw = -(cur_x - prev_x)
        if abs(dx_raw) > 0.05:
            n_dones += 1
        else:
            n_valid_steps += 1
        prev_x = cur_x

        # ── HUD per-frame metrics — HONEST BODY-PCA (body-level, head-wag-robust) ──
        try:
            rd = base_env.robot.data
            _bpw = rd.body_pos_w[0].detach().cpu().numpy()           # (nb,3)
            _sxy = _bpw[_spine_ids, :2].astype(float)                # (S,2) spine world XY
            _bq = rd.body_quat_w[0].detach().cpu().numpy()           # (nb,4) wxyz
            if _ALLLINK:
                _alllink_pos.append(_bpw.astype(np.float32).copy())
                _alllink_quat.append(_bq.astype(np.float32).copy())
            _sq = _bq[_spine_ids, :]
            _w0, _qx, _qy, _qz = _sq[:, 0], _sq[:, 1], _sq[:, 2], _sq[:, 3]
            _seg_up = 1.0 - 2.0 * (_qx ** 2 + _qy ** 2)              # per-seg uprightness
            _seg_roll = np.arctan2(2.0 * (_w0 * _qx + _qy * _qz), 1.0 - 2.0 * (_qx ** 2 + _qy ** 2))
            _roll = math.degrees(float(np.median(_seg_roll)))        # body-median roll
            _upr = float(np.median(_seg_up))                          # body-median uprightness
            _vx = float(rd.root_lin_vel_w[0, 0].item())
            _bw = float(rd.joint_pos[0, _body_wave_ids].detach().cpu().numpy().std()) if len(_body_wave_ids) else 0.0
            hud_rows.append(dict(f=f, x=cur_x, spine_xy=_sxy.tolist(), roll_deg=_roll,
                                 v_fwd_inst=-_vx, upright=_upr, bodywave_std=_bw))
        except Exception as _e:
            hud_rows.append(dict(f=f, x=cur_x, spine_xy=None, roll_deg=0.0,
                                 v_fwd_inst=0.0, upright=1.0, bodywave_std=0.0))
            if _ALLLINK:   # keep the all-link series frame-aligned even on a rare read failure
                _nbq = len(_bn)
                _alllink_pos.append(np.zeros((_nbq, 3), np.float32))
                _alllink_quat.append(np.zeros((_nbq, 4), np.float32))

        if (f + 1) % 25 == 0 or f == n_frames - 1:
            print(f"[viz] captured {f+1}/{n_frames} frames  failed={n_failed}",
                  flush=True)

    # ── §4 all-link pose dump: full per-link world pose of THIS render's physics, so the offline
    #    renderer reproduces the IDENTICAL motion (same-run Omniverse-vs-offline). Additive, default OFF. ──
    if _ALLLINK and _alllink_pos:
        _alp = os.path.splitext(out_path)[0] + "_alllink.npz"
        _bp = np.stack(_alllink_pos, 0)[:, None, :, :]    # (T,1,nb,3)
        _bqq = np.stack(_alllink_quat, 0)[:, None, :, :]  # (T,1,nb,4)
        _Tn = _bp.shape[0]
        _homog = args.homogeneous_lambda_mm is not None
        np.savez_compressed(
            _alp, body_pos_w=_bp, body_quat_w=_bqq, body_names=np.array(_bn),
            alive=np.ones((_Tn, 1), bool),
            x0=np.array([float(x0)], np.float32), z0=np.array([float(z_min)], np.float32),
            dt_pol=np.float32(DT_POL),
            lambda_mm=np.float32(args.homogeneous_lambda_mm if _homog else 0.0),
            amp_mm=np.float32(args.homogeneous_amp_mm if _homog else 0.0),
            terrain_seed=np.int64(args.homogeneous_seed if _homog else 0),
            arena_size_m=np.float32(env_cfg.transition_arena_size_m),
            image_size=np.int64(env_cfg.transition_image_size),
            freq_hz=np.float32(args.freq_hz),
            force_wave_abs=np.float32(args.absolute_wave_number if args.absolute_wave_number is not None else np.nan),
            warmup_steps=np.int64(_n_warm_pol), n_envs=np.int64(1),
            central_spawn=np.bool_(bool(getattr(args, "central_spawn", False))),
            y_strata=np.array(env_cfg.transition_spawn_y_strata_centres, dtype=np.float32))
        print(f"[viz] wrote all-link -> {_alp}  (body_pos_w {_bp.shape})", flush=True)

    net_forward = -(prev_x - x0)
    duration = n_frames * DT_POL
    v_fwd  = net_forward / max(duration, 1e-6)
    vz_rms = math.sqrt(vz_sq_sum / max(n_frames, 1))
    z_p2p  = z_max - z_min

    if n_failed > 0:
        print(f"[viz] WARNING: {n_failed}/{n_frames} frames failed content check (zero-content RGB).",
              flush=True)

    # Hard-fail if too many frames are zero-content. Threshold = 10% of total.
    # This makes the bash retry path see the failure via grep "FAIL at L" pattern
    # AND prevents an unusable MP4 from being written. Aligns with
    # docs/RENDERING_PIPELINE_DEBUG_NOTES.md §2 mandatory check #2.
    if n_failed > max(10, int(0.10 * n_frames)):
        print(f"[viz] FAIL at L3 — {n_failed}/{n_frames} frames are zero-content. "
              "Aborting MP4 write so bash retry can re-render.", flush=True)
        return 4

    # Encode MP4
    try:
        import imageio.v2 as imageio  # type: ignore
        with imageio.get_writer(out_path, fps=int(args.video_fps),
                                codec="libx264", quality=8) as w:
            for img in frames:
                w.append_data(img)
        print(f"[viz] MP4 -> {out_path}", flush=True)
    except Exception as exc:
        print(f"[viz] imageio failed ({type(exc).__name__}: {exc}); cv2 fallback",
              flush=True)
        try:
            import cv2  # type: ignore
            h_, w_ = frames[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(out_path, fourcc,
                                     float(args.video_fps), (w_, h_))
            for img in frames:
                bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                writer.write(bgr)
            writer.release()
            print(f"[viz] MP4 (cv2) -> {out_path}", flush=True)
        except Exception as exc2:
            print(f"[viz] cv2 also failed: {type(exc2).__name__}: {exc2}",
                  flush=True)
            return 3

    # JSON sidecar
    if json_path:
        meta = {
            "policy_name":     args.policy_name,
            "checkpoint":      args.checkpoint,
            "task":            args.task,
            "seed":            args.seed,
            "spawn_seed":      args.spawn_seed,
            "pin_block":       args.pin_block,
            "pin_block_label": BLOCK_LABELS[args.pin_block] if args.pin_block is not None else None,
            "pin_lambda_mm":   BLOCK_LAMBDA[args.pin_block] if args.pin_block is not None else None,
            "pin_amp_mm":      BLOCK_AMP[args.pin_block]    if args.pin_block is not None else None,
            "pin_severity_S":  BLOCK_SEV[args.pin_block]    if args.pin_block is not None else None,
            "pin_y_strat":     args.pin_y_strat,
            "no_jitter":       bool(args.no_jitter),
            "spawn_xyz":       [float(spawn_xyz[0]), float(spawn_xyz[1]), float(spawn_xyz[2])],
            "spawn_block_id_observed": int(sb[0]) if len(sb) else -1,
            "homogeneous_lambda_mm": args.homogeneous_lambda_mm,
            "homogeneous_amp_mm":    args.homogeneous_amp_mm if args.homogeneous_lambda_mm is not None else None,
            "homogeneous_seed":      args.homogeneous_seed   if args.homogeneous_lambda_mm is not None else None,
            "absolute_wave_number":  args.absolute_wave_number,
            "scales": {
                "body_amp": args.body_amp_scale, "leg_yaw": args.leg_yaw_scale,
                "leg_pitch": args.leg_pitch_scale, "wave": args.wave_scale,
                "imp": args.imp_scale, "freq": args.freq_scale,
            },
            "camera": args.camera,
            "chase_offset": list(args.chase_offset) if chase else None,
            "video": {
                "path":         out_path,
                "width":        args.width,
                "height":       args.height,
                "fps":          args.video_fps,
                "n_frames":     n_frames,
                "duration_s":   duration,
                "failed_frames": n_failed,
            },
            "metrics": {
                "v_fwd_mps": v_fwd,
                "F_max_N":   F_max_seen,
                "vz_rms_mps": vz_rms,
                "z_p2p_m":   z_p2p,
                "n_dones":   n_dones,
                "n_valid_steps": n_valid_steps,
            },
        }
        # ── HUD: clip-level HONEST BODY-PCA metrics + per-frame trace ──
        def _body_pca_heading_deg(sxy):
            """sxy (S,2) -> body principal-axis heading (deg), sign by head->tail."""
            x = sxy[:, 0]; y = sxy[:, 1]
            xm = x - x.mean(); ym = y - y.mean()
            a = (xm * xm).mean(); c = (ym * ym).mean(); b = (xm * ym).mean()
            th = 0.5 * math.atan2(2.0 * b, a - c)
            vx, vy = math.cos(th), math.sin(th)
            htx = sxy[0, 0] - sxy[-1, 0]; hty = sxy[0, 1] - sxy[-1, 1]
            if vx * htx + vy * hty < 0:
                vx, vy = -vx, -vy
            return math.degrees(math.atan2(vy, vx))
        _headings = []
        for r in hud_rows:
            if r.get("spine_xy") is not None:
                _headings.append(_body_pca_heading_deg(np.asarray(r["spine_xy"], dtype=np.float64)))
            else:
                _headings.append(float("nan"))
        _h = np.array(_headings, dtype=np.float64)
        _h_valid = _h[~np.isnan(_h)]
        # net BODY rotation = unwrapped heading change start->end (no integration)
        if len(_h_valid) >= 2:
            _hu = np.degrees(np.unwrap(np.radians(_h_valid)))
            _net_yaw_deg = float(_hu[-1] - _hu[0])
            _cum_yaw_valid = _hu - _hu[0]
        else:
            _net_yaw_deg = 0.0; _cum_yaw_valid = np.zeros(len(_h_valid))
        # re-expand cum yaw to all frames (nan-fill carried forward)
        _cum_yaw = np.zeros(len(hud_rows)); _j = 0
        for i in range(len(hud_rows)):
            if not np.isnan(_h[i]):
                _cum_yaw[i] = _cum_yaw_valid[_j]; _j += 1
            elif i > 0:
                _cum_yaw[i] = _cum_yaw[i - 1]
        _roll = np.array([r["roll_deg"] for r in hud_rows], dtype=np.float64)
        _bw = np.array([r["bodywave_std"] for r in hud_rows], dtype=np.float64)
        _upr = np.array([r["upright"] for r in hud_rows], dtype=np.float64)
        _roll_rms_deg = float(np.sqrt(np.mean(_roll ** 2))) if len(_roll) else 0.0
        meta["hud_metrics"] = {
            "net_yaw_deg":   _net_yaw_deg,                    # BODY-PCA net rotation (honest)
            "roll_rms_deg":  _roll_rms_deg,                   # body-median roll RMS
            "fwd_signed_m":  net_forward,                     # = -Δx_world
            "v_fwd_mps":     v_fwd,
            "min_upright":   float(_upr.min()) if len(_upr) else 1.0,  # body-median tip-over
            "bodywave_realized_rms_rad": float(_bw.mean()),
            "dt_pol_s":      DT_POL, "metric": "body_pca",
            "terrain": {"lambda_mm": args.homogeneous_lambda_mm,
                        "amp_mm": args.homogeneous_amp_mm if args.homogeneous_lambda_mm is not None else None},
        }
        meta["per_frame"] = [
            dict(f=r["f"], net_yaw_cum_deg=float(_cum_yaw[i]),
                 roll_deg=r["roll_deg"], v_fwd_inst_mps=r["v_fwd_inst"],
                 fwd_disp_m=float(-(r["x"] - x0)), upright=r["upright"],
                 bodywave_std_rad=r["bodywave_std"])
            for i, r in enumerate(hud_rows)
        ]
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        print(f"[viz] JSON sidecar -> {json_path}  (net_yaw={_net_yaw_deg:+.1f} "
              f"roll_rms={_roll_rms_deg:.1f} fwd={net_forward:+.3f})", flush=True)

    print(f"[viz] report -> {out_path}", flush=True)
    return 0


if __name__ == "__main__":
    rc = main()
    sim_app.close()
    sys.exit(rc)
