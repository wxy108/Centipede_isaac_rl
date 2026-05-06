"""
Configuration classes for the Centipede DirectRLEnv.

Mirrors the scene setup from scripts/test_walk.py (single-env reference)
but uses Isaac Lab's InteractiveScene to clone the centipede across
num_envs parallel environments.
"""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass


# =============================================================================
# Paths
# =============================================================================

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
CENTIPEDE_USD_PATH = os.path.join(PROJECT_ROOT, "assets", "centipede.usd")
CONTROLLER_CONFIG_PATH = os.path.join(PROJECT_ROOT, "configs", "farms_controller.yaml")


# =============================================================================
# Robot articulation cfg
# =============================================================================
# Actuator groups match scripts/test_walk.py:
#   active joints: effort-mode, stiffness=damping=0 (controller sets torque)
#   foot joints:   passive damping (matches MuJoCo XML)

CENTIPEDE_ROBOT_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=CENTIPEDE_USD_PATH,
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.025),
    ),
    actuators={
        "active": ImplicitActuatorCfg(
            joint_names_expr=[
                "joint_body_.*",
                "joint_pitch_body_.*",
                "joint_leg_.*",
            ],
            effort_limit=10.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=0.0,
        ),
        "foot_passive": ImplicitActuatorCfg(
            joint_names_expr=["joint_foot_.*"],
            effort_limit=1.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=0.006,
        ),
    },
)


# =============================================================================
# Scene cfg
# =============================================================================

@configclass
class CentipedeSceneCfg(InteractiveSceneCfg):
    """N parallel centipedes on a shared flat ground plane."""

    num_envs: int = 64
    env_spacing: float = 1.5  # metres between cloned envs

    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DistantLightCfg(intensity=2000.0),
    )

    robot: ArticulationCfg = CENTIPEDE_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )


# =============================================================================
# Env cfg
# =============================================================================

@configclass
class CentipedeEnvCfg(DirectRLEnvCfg):
    """Top-level config for the Centipede Layer 2 RL env.

    Stage A (this commit): plumbing only. Zero actions, placeholder
    observations and rewards. Once the smoke test passes, Stage B fills
    in the real Layer 2 action / obs / reward / termination logic.
    """

    # Physics dt and decimation matched to the project's existing
    # Isaac Lab single-env baseline (scripts/test_walk.py reads
    # simulation.timestep = 0.004 from configs/farms_controller.yaml).
    # The MuJoCo reference runs at 0.5 ms for scientific fidelity, but
    # Isaac Lab side has always been 4 ms — and at 4 ms the centipede's
    # tuned gain set is stable. Policy step = 12 * 4 ms = 48 ms, which
    # rounds to ~20 Hz policy update cadence.
    decimation: int = 12
    episode_length_s: float = 8.0

    sim: SimulationCfg = SimulationCfg(
        dt=0.004,                 # 4 ms physics step (matches test_walk.py)
        render_interval=12,       # match decimation: 1 render per policy step
    )

    scene: CentipedeSceneCfg = CentipedeSceneCfg()

    # ── Action / observation spaces ──
    # Stage A placeholders. Stage B sets:
    #   action_space      = 21  (19 per-joint amp scales + wave_number + body_kp)
    #   observation_space = ~50 lean obs (root state, body joints, contacts, phases)
    action_space: int = 21
    observation_space: int = 50
    state_space: int = 0
