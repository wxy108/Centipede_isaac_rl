"""Evaluate one policy or one fixed open-loop wave on one homogeneous terrain."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from isaaclab.app import AppLauncher


ROOT = Path(__file__).resolve().parents[1]
TERRAINS = {
    "flat": (1000.0, 0.0),
    "l120a4": (120.0, 4.0),
    "l90a8": (90.0, 8.0),
    "l60a12": (60.0, 12.0),
    "l45a16": (45.0, 16.0),
    "l30a16": (30.0, 16.0),
}

parser = argparse.ArgumentParser()
parser.add_argument("--terrain", choices=tuple(TERRAINS), default="flat")
parser.add_argument("--terrain-seed", type=int, default=244949)
parser.add_argument("--policy-seed", type=int, choices=(161803, 141421, 173205), default=161803)
parser.add_argument("--checkpoint", type=Path)
parser.add_argument("--fixed-wave", type=float)
parser.add_argument("--num-envs", type=int, default=32)
parser.add_argument("--steps", type=int, default=800)
parser.add_argument("--out", type=Path, default=ROOT / "outputs" / "evaluate")
parser.add_argument(
    "--video",
    action="store_true",
    help="Record exact link poses and render a deterministic offline verification video.",
)
parser.add_argument("--video-fps", type=int, default=20)
parser.add_argument("--video-width", type=int, default=960)
parser.add_argument("--video-height", type=int, default=540)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
launcher = AppLauncher(args)
simulation_app = launcher.app

sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from rl_games.common import env_configurations, vecenv  # noqa: E402
from rl_games.torch_runner import Runner  # noqa: E402
from isaaclab_rl.rl_games import RlGamesGpuEnv, RlGamesVecEnvWrapper  # noqa: E402

from centipede.env import CentipedeEnv  # noqa: E402
from centipede.env_cfg import CentipedeEnvCfg  # noqa: E402
from centipede.model import BODY_LENGTH_M  # noqa: E402


def _spine_x(base):
    ids = torch.as_tensor(base.model.spine, dtype=torch.long, device=base.device)
    return base.robot.data.body_pos_w[:, ids, 0].mean(dim=1)


def _policy(wrapped, checkpoint):
    agent = json.loads((ROOT / "config" / "rl_games.json").read_text(encoding="utf-8"))
    agent["params"]["load_checkpoint"] = True
    agent["params"]["load_path"] = str(checkpoint)
    agent["params"]["config"]["num_actors"] = int(args.num_envs)
    vecenv.register(
        "CentipedeEval", lambda config_name, num_actors, **kwargs:
        RlGamesGpuEnv(config_name, num_actors, **kwargs)
    )
    env_configurations.register(
        "rlgpu", {"vecenv_type": "CentipedeEval", "env_creator": lambda **kwargs: wrapped}
    )
    runner = Runner()
    runner.load(agent)
    player = runner.create_player()
    player.restore(str(checkpoint))
    player.reset()
    return player


def main():
    lambda_mm, amplitude_mm = TERRAINS[args.terrain]
    cfg = CentipedeEnvCfg()
    cfg.scene.num_envs = int(args.num_envs)
    cfg.terrain_mode = "homogeneous"
    cfg.terrain_label = args.terrain
    cfg.terrain_lambda_mm = lambda_mm
    cfg.terrain_amplitude_mm = amplitude_mm
    cfg.terrain_seed = int(args.terrain_seed)
    cfg.fixed_wave = args.fixed_wave
    requested_duration_s = float(args.steps) * float(cfg.decimation) * float(cfg.sim.dt)
    cfg.episode_length_s = max(
        float(cfg.episode_length_s),
        requested_duration_s + float(cfg.decimation) * float(cfg.sim.dt),
    )
    # Verification videos are rendered after simulation from exact world poses.
    # Keeping the Isaac run camera-free avoids the known Windows RTX startup race.
    base = CentipedeEnv(cfg=cfg, render_mode=None)
    env = base
    run_name = (
        f"open_loop_w{args.fixed_wave:g}" if args.fixed_wave is not None
        else f"policy_seed_{args.policy_seed}"
    )
    out = args.out.resolve() / args.terrain / f"terrain_seed_{args.terrain_seed}" / run_name
    out.mkdir(parents=True, exist_ok=True)

    if args.fixed_wave is None:
        checkpoint = args.checkpoint or ROOT / "checkpoints" / f"policy_seed_{args.policy_seed}.pth"
        wrapped = RlGamesVecEnvWrapper(env, "cuda:0", 5.0, 1.0)
        player = _policy(wrapped, checkpoint.resolve())
        obs = wrapped.reset()
        if isinstance(obs, dict):
            obs = obs["obs"]
        player.get_batch_size(obs, 1)
    else:
        wrapped = None
        player = None
        obs, _ = env.reset()

    initial_x = _spine_x(base).clone()
    final_x = initial_x.clone()
    alive = torch.ones(base.num_envs, dtype=torch.bool, device=base.device)
    first_done = torch.full((base.num_envs,), -1, dtype=torch.long, device=base.device)
    return_sum = torch.zeros(base.num_envs, dtype=torch.float32, device=base.device)
    wave_trace = []
    reward_trace = []
    pose_positions = []
    pose_quaternions = []
    pose_alive = []
    pose_wave = []
    pose_time_s = []
    next_video_time_s = 0.0

    def capture_pose(time_s):
        pose_positions.append(base.robot.data.body_pos_w[0].detach().cpu().numpy().copy())
        pose_quaternions.append(base.robot.data.body_quat_w[0].detach().cpu().numpy().copy())
        pose_alive.append(bool(alive[0].item()))
        pose_wave.append(float(base.controller.realized_wave[0].item()))
        pose_time_s.append(float(time_s))

    if args.video:
        if int(args.video_fps) <= 0:
            raise ValueError("--video-fps must be positive")
        capture_pose(0.0)
        next_video_time_s = 1.0 / float(args.video_fps)

    for step in range(int(args.steps)):
        with torch.inference_mode():
            if player is None:
                actions = torch.zeros((base.num_envs, 1), dtype=torch.float32, device=base.device)
                obs, reward, terminated, truncated, _ = env.step(actions)
                done = terminated | truncated
            else:
                action = player.get_action(player.obs_to_torch(obs), is_deterministic=True)
                obs, reward, done, _ = wrapped.step(action)
            reward = reward.to(base.device)
            done = done.to(base.device).bool()
            still_alive = alive & ~done
            current_x = _spine_x(base)
            final_x[still_alive] = current_x[still_alive]
            newly_done = alive & done
            first_done[newly_done] = step
            alive &= ~done
            return_sum += reward
            wave_trace.append(base.controller.realized_wave.detach().cpu().numpy())
            reward_trace.append(reward.detach().cpu().numpy())
            elapsed_s = float(step + 1) * float(base.step_dt)
            if args.video and elapsed_s + 1e-12 >= next_video_time_s:
                capture_pose(elapsed_s)
                while next_video_time_s <= elapsed_s + 1e-12:
                    next_video_time_s += 1.0 / float(args.video_fps)

    duration_s = float(args.steps) * float(base.step_dt)
    speed = torch.full((base.num_envs,), float("nan"), device=base.device)
    speed[alive] = -(final_x[alive] - initial_x[alive]) / duration_s / BODY_LENGTH_M
    result = {
        "contract": {"observation_dim": 34, "action_dim": 1, "period_ms": 12, "reward": "pure_forward_spine_centroid"},
        "terrain": args.terrain,
        "terrain_seed": int(args.terrain_seed),
        "mode": "open_loop" if player is None else "closed_loop",
        "fixed_wave": args.fixed_wave,
        "policy_seed": None if player is None else int(args.policy_seed),
        "num_envs": int(base.num_envs),
        "parallel_conditions_not_independent_replicates": True,
        "steps": int(args.steps),
        "duration_s": duration_s,
        "survivors": int(alive.sum().item()),
        "survival_rate": float(alive.float().mean().item()),
        "survivor_speed_bl_cycle": speed[alive].detach().cpu().tolist(),
        "survivor_speed_median_bl_cycle": float(torch.nanmedian(speed).item()) if alive.any() else None,
        "first_done_step": first_done.detach().cpu().tolist(),
        "episode_return": return_sum.detach().cpu().tolist(),
    }
    (out / "metrics.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    np.savez_compressed(
        out / "traces.npz",
        wave=np.stack(wave_trace),
        reward=np.stack(reward_trace),
        alive=alive.detach().cpu().numpy(),
        first_done_step=first_done.detach().cpu().numpy(),
    )
    render_job = None
    if args.video:
        trace_path = out / "all_link_world_poses.npz"
        np.savez_compressed(
            trace_path,
            body_names=np.asarray(base.robot.body_names),
            body_pos_w=np.stack(pose_positions)[:, None, :, :],
            body_quat_w=np.stack(pose_quaternions)[:, None, :, :],
            alive=np.asarray(pose_alive, dtype=bool)[:, None],
            realized_wave=np.asarray(pose_wave, dtype=np.float32)[:, None],
            time_s=np.asarray(pose_time_s, dtype=np.float64),
            lambda_mm=np.asarray(lambda_mm, dtype=np.float64),
            amp_mm=np.asarray(amplitude_mm, dtype=np.float64),
            terrain_seed=np.asarray(int(args.terrain_seed), dtype=np.int64),
            arena_size_m=np.asarray(3.0, dtype=np.float64),
            image_size=np.asarray(1536, dtype=np.int64),
            dt_pol=np.asarray(float(base.step_dt), dtype=np.float64),
        )
        video_dir = out / "video"
        video_dir.mkdir(parents=True, exist_ok=True)
        render_job = {
            "trace": trace_path,
            "mp4": video_dir / "offline_45deg.mp4",
            "report": video_dir / "offline_45deg.audit.json",
        }
    print(json.dumps(result, indent=2))
    if wrapped is not None:
        wrapped.close()
    else:
        env.close()
    if render_job is not None:
        command = [
            sys.executable,
            str(ROOT / "scripts" / "render_offline.py"),
            "--alllink-npz",
            str(render_job["trace"]),
            "--out-mp4",
            str(render_job["mp4"]),
            "--report-json",
            str(render_job["report"]),
            "--fps",
            str(args.video_fps),
            "--width",
            str(args.video_width),
            "--height",
            str(args.video_height),
            "--terrain",
        ]
        completed = subprocess.run(command, cwd=ROOT, check=False)
        if completed.returncode != 0:
            raise RuntimeError(
                f"Offline video audit failed with exit code {completed.returncode}: "
                f"{render_job['report']}"
            )
        print(f"[video] {render_job['mp4']}", flush=True)
        print(f"[video audit] {render_job['report']}", flush=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
