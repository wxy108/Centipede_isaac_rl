"""Train the frozen 34-D observation / 1-D wave policy from scratch."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from isaaclab.app import AppLauncher


ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=161803)
parser.add_argument("--epochs", type=int, default=24)
parser.add_argument("--num-envs", type=int, default=32)
parser.add_argument("--out", type=Path, default=ROOT / "outputs" / "train")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
launcher = AppLauncher(args)
simulation_app = launcher.app

sys.path.insert(0, str(ROOT))

from rl_games.common import env_configurations, vecenv  # noqa: E402
from rl_games.common.algo_observer import IsaacAlgoObserver  # noqa: E402
from rl_games.torch_runner import Runner  # noqa: E402
from isaaclab_rl.rl_games import RlGamesGpuEnv, RlGamesVecEnvWrapper  # noqa: E402

from centipede.env import CentipedeEnv  # noqa: E402
from centipede.env_cfg import CentipedeEnvCfg  # noqa: E402


def main():
    agent = json.loads((ROOT / "config" / "rl_games.json").read_text(encoding="utf-8"))
    params = agent["params"]
    params["seed"] = int(args.seed)
    params["config"]["max_epochs"] = int(args.epochs)
    params["config"]["num_actors"] = int(args.num_envs)
    params["config"]["train_dir"] = str(args.out.resolve())
    params["config"]["full_experiment_name"] = f"seed_{args.seed}"

    cfg = CentipedeEnvCfg()
    cfg.seed = int(args.seed)
    cfg.scene.num_envs = int(args.num_envs)
    cfg.terrain_mode = "tri"
    env = CentipedeEnv(cfg=cfg)
    wrapped = RlGamesVecEnvWrapper(env, "cuda:0", 5.0, 1.0)
    vecenv.register(
        "CentipedeIsaac", lambda config_name, num_actors, **kwargs:
        RlGamesGpuEnv(config_name, num_actors, **kwargs)
    )
    env_configurations.register(
        "rlgpu", {"vecenv_type": "CentipedeIsaac", "env_creator": lambda **kwargs: wrapped}
    )
    runner = Runner(IsaacAlgoObserver())
    runner.load(agent)
    runner.reset()
    try:
        runner.run({"train": True, "play": False})
    finally:
        wrapped.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
