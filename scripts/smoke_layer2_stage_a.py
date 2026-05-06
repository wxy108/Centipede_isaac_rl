#!/usr/bin/env python3
"""
Stage A smoke test for the Layer 2 DirectRLEnv, instrumented.

What this script measures and why
=================================
The slowness of the first run was kernel-launch dominated, not physics
dominated. To diagnose where the time actually goes, this script:

  - Wall-clocks every policy step.
  - Wall-clocks the controller subroutine separately from the rest of
    env.step (i.e., physics + render + obs/rew/done).
  - Counts exact GPU<->CPU syncs (printed every 5 steps).
  - Optionally records an MP4 of the cloned env grid.

Use the timing summary at the end to decide whether further controller
optimization is worth it, or whether the right move is to scale up
num_envs (which amortizes launch overhead across more parallel work).

Usage
-----
    python scripts/smoke_layer2_stage_a.py --headless --num-envs 8 --num-steps 30
    python scripts/smoke_layer2_stage_a.py --headless --num-envs 64 --num-steps 30
    python scripts/smoke_layer2_stage_a.py --headless --num-envs 16 \
        --num-steps 100 --video videos/smoke_stage_a.mp4
"""

import argparse
import os
import statistics
import sys
import time

# ------------------------------------------------------------------
# CLI parsing FIRST — must inject --enable_cameras before AppLauncher
# reads sys.argv if --video is requested.
# ------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--num-envs", type=int, default=8,
                    help="Number of parallel envs.")
parser.add_argument("--num-steps", type=int, default=30,
                    help="Number of policy steps to run.")
parser.add_argument("--video", default=None,
                    help="Output MP4 path (e.g. videos/smoke.mp4). "
                         "If set, --enable_cameras is auto-injected.")
parser.add_argument("--video-length", type=int, default=200,
                    help="Max frames per recorded clip.")

_uses_video = any(a == "--video" or a.startswith("--video=") for a in sys.argv[1:])
if _uses_video and "--enable_cameras" not in sys.argv:
    sys.argv.append("--enable_cameras")
    print("[smoke] --video given -> injected --enable_cameras", flush=True)

from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
sim_app = app_launcher.app

# ── Force non-progressive renderer ────────────────────────────────
# Default `/rtx/rendermode` on this build is `PathTracing`, which is
# progressive: every camera/scene change resets the sample
# accumulator, producing 1-2 frames of noise/near-black output. With
# the gymnasium RecordVideo path that means alternating black + real
# frames, i.e. the "flashing" symptom. Every working video pipeline
# in this project (optimize_impedance_bayesian.py:235,
# optimize_impedance_biological.py:412, render_top_trials.py:155)
# forces RaytracedLighting at startup. We do the same. See
# NOTES_RENDERING.md sections 0 and 1 for full rationale.
import carb
_carb_settings = carb.settings.get_settings()
_carb_settings.set_string("/rtx/rendermode", "RaytracedLighting")
print("[smoke] forced /rtx/rendermode = RaytracedLighting "
      "(non-progressive, prevents flashing video)", flush=True)

# Imports that depend on the app being live
import gymnasium
import torch

from envs.centipede_env import CentipedeEnv
from envs.centipede_env_cfg import CentipedeEnvCfg


def _fmt_ms(seconds: float) -> str:
    return f"{seconds * 1000:7.1f}ms"


def main():
    cfg = CentipedeEnvCfg()
    cfg.scene.num_envs = args.num_envs

    print(f"[smoke] num_envs={cfg.scene.num_envs} action_dim={cfg.action_space} "
          f"obs_dim={cfg.observation_space} decimation={cfg.decimation} "
          f"sim_dt={cfg.sim.dt}s", flush=True)

    render_mode = "rgb_array" if args.video else None
    env = CentipedeEnv(cfg=cfg, render_mode=render_mode)
    print(f"[smoke] Env instantiated. num_envs={env.num_envs} "
          f"device={env.device}", flush=True)

    if args.video:
        video_folder = os.path.dirname(args.video) or "."
        os.makedirs(video_folder, exist_ok=True)
        prefix = os.path.splitext(os.path.basename(args.video))[0]
        env = gymnasium.wrappers.RecordVideo(
            env,
            video_folder=video_folder,
            name_prefix=prefix,
            video_length=args.video_length,
            episode_trigger=lambda ep: True,   # record every episode
            disable_logger=True,
        )
        print(f"[smoke] Recording video to {video_folder}/{prefix}*.mp4 "
              f"(<= {args.video_length} frames)", flush=True)

    obs, _info = env.reset()
    if isinstance(obs, dict):
        obs_shape = obs["policy"].shape
    else:
        obs_shape = obs.shape
    print(f"[smoke] Reset OK. obs shape={obs_shape}", flush=True)

    actions = torch.zeros(
        (args.num_envs, cfg.action_space),
        dtype=torch.float32, device=env.unwrapped.device,
    )
    print(f"[smoke] Stepping {args.num_steps} policy steps "
          f"(= {args.num_steps * cfg.decimation} physics steps, "
          f"~{args.num_steps * cfg.decimation * cfg.sim.dt:.2f}s sim time)",
          flush=True)

    # ── Per-step timing instrumentation ───────────────────────────
    times_total: list[float] = []
    times_ctrl: list[float] = []     # filled if env.unwrapped exposes _last_controller_dt

    t_loop_start = time.perf_counter()
    for step_idx in range(args.num_steps):
        t0 = time.perf_counter()
        obs, reward, terminated, truncated, _info = env.step(actions)
        # Force any pending async GPU work to complete before stopping the
        # clock — without this, we under-count GPU work that hasn't drained.
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        t1 = time.perf_counter()
        dt_step = t1 - t0
        times_total.append(dt_step)

        # Optional: pull controller time from the env if it's tracking it.
        ctrl_dt = getattr(env.unwrapped, "_last_controller_dt_total", None)
        if ctrl_dt is not None:
            times_ctrl.append(ctrl_dt)
            env.unwrapped._last_controller_dt_total = 0.0  # reset accumulator

        if step_idx % 5 == 0 or step_idx == args.num_steps - 1:
            recent = times_total[-5:]
            avg_recent = statistics.mean(recent)
            ctrl_str = ""
            if times_ctrl and len(times_ctrl) >= 1:
                ctrl_str = f"  ctrl={_fmt_ms(times_ctrl[-1])}"
            print(f"[smoke] step={step_idx:3d}  "
                  f"this={_fmt_ms(dt_step)}  "
                  f"avg5={_fmt_ms(avg_recent)}{ctrl_str}",
                  flush=True)
    t_loop_end = time.perf_counter()

    # ── Summary ───────────────────────────────────────────────────
    total_wall = t_loop_end - t_loop_start
    sim_dt_per_step = cfg.decimation * cfg.sim.dt
    sim_time_total = sim_dt_per_step * len(times_total)

    print("", flush=True)
    print("================ TIMING SUMMARY ================", flush=True)
    print(f"  num_envs:              {args.num_envs}", flush=True)
    print(f"  policy steps:          {len(times_total)}", flush=True)
    print(f"  sim time elapsed:      {sim_time_total:.3f} s", flush=True)
    print(f"  wall-clock total:      {total_wall:.3f} s", flush=True)
    print(f"  realtime factor:       {sim_time_total / total_wall:6.2f}x", flush=True)
    print(f"  mean step:             {_fmt_ms(statistics.mean(times_total))}", flush=True)
    print(f"  median step:           {_fmt_ms(statistics.median(times_total))}", flush=True)
    print(f"  min step:              {_fmt_ms(min(times_total))}", flush=True)
    print(f"  max step:              {_fmt_ms(max(times_total))}", flush=True)
    if times_ctrl:
        ctrl_mean = statistics.mean(times_ctrl)
        ctrl_share = ctrl_mean / statistics.mean(times_total) * 100.0
        print(f"  controller mean:       {_fmt_ms(ctrl_mean)}  "
              f"({ctrl_share:.0f}% of step)", flush=True)
        print(f"  remainder (physics+render+obs/rew/done): "
              f"{_fmt_ms(statistics.mean(times_total) - ctrl_mean)}", flush=True)
    env_steps_per_s = (cfg.decimation * args.num_envs * len(times_total)
                       / total_wall)
    print(f"  env-steps/s (physics): {env_steps_per_s:9.0f}", flush=True)
    print("================================================", flush=True)
    print("[smoke] DONE", flush=True)

    env.close()
    sim_app.close()


if __name__ == "__main__":
    main()
