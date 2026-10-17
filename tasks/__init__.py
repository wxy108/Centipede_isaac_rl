"""
Stage B-v0 Pass 4/5 (2026-05-07): centipede RL task registration.

Registers gymnasium task IDs that point CentipedeEnv (the
DirectRLEnv subclass in envs/) at IsaacLab's rl_games trainer.
Registration is EXPLICIT: callers (e.g. scripts/rl/train_stage_b_smoke.py)
import `register_centipede_tasks` and call it once at startup. We
deliberately avoid auto-registering at import time so that running
sanity scripts that do NOT need rl_games keeps a clean global state.
"""

from __future__ import annotations

import os

import gymnasium as gym


_AGENT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "agents")


def _resolve_yaml(name: str) -> str:
    """Return absolute path to an rl_games YAML config in tasks/agents/.
    rl_games' Runner.load() accepts dict OR path; we pass the path so
    the YAML is parsed inside rl_games (matches IsaacLab's pattern)."""
    return os.path.join(_AGENT_DIR, name)


def register_centipede_tasks() -> None:
    """Idempotently register the Stage B-v0 task IDs.

    Tasks:
      Isaac-Centipede-Stage-B-Transition-v0
        Smoke configuration: 10 PPO epochs, intended num_envs=16. Used
        for end-to-end pipeline checks. yaml ->
        rl_games_stage_b_smoke_cfg.yaml.

      Isaac-Centipede-Stage-B-Train-v0     (Pass 5, 2026-05-07)
        Long-training configuration: 300 PPO epochs, intended
        num_envs=32, ~4 h wall time on RTX 5070 Laptop. Same env class
        and CentipedeEnvCfg defaults; just a different rl_games agent
        yaml. yaml -> rl_games_stage_b_train_cfg.yaml.
    """
    common_entry = {
        "entry_point": "envs.centipede_env:CentipedeEnv",
        "disable_env_checker": True,
    }

    smoke_id = "Isaac-Centipede-Stage-B-Transition-v0"
    if smoke_id not in gym.registry:
        gym.register(
            id=smoke_id,
            **common_entry,
            kwargs={
                "env_cfg_entry_point": "envs.centipede_env_cfg:CentipedeEnvCfg",
                "rl_games_cfg_entry_point": _resolve_yaml(
                    "rl_games_stage_b_smoke_cfg.yaml"
                ),
            },
        )

    train_id = "Isaac-Centipede-Stage-B-Train-v0"
    if train_id not in gym.registry:
        gym.register(
            id=train_id,
            **common_entry,
            kwargs={
                "env_cfg_entry_point": "envs.centipede_env_cfg:CentipedeEnvCfg",
                "rl_games_cfg_entry_point": _resolve_yaml(
                    "rl_games_stage_b_train_cfg.yaml"
                ),
            },
        )
