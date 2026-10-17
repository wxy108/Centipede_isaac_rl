"""Single supported Isaac Lab environment configuration."""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.envs.common import ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_PATH = PROJECT_ROOT / "assets" / "centipede_rebuild_v1_renamed.usd"
CONTROLLER_PATH = PROJECT_ROOT / "config" / "controller.yaml"
GAIT_MAP_PATH = PROJECT_ROOT / "config" / "gait_map.json"


ROBOT_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(ASSET_PATH),
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.025)),
    actuators={
        "active": ImplicitActuatorCfg(
            joint_names_expr=[
                "body_yaw_.*", "body_pitch_.*", "body_roll_.*",
                "legbase_.*", "knee_.*",
            ],
            effort_limit=10.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=0.0,
            armature=0.0,
        )
    },
)


@configclass
class CentipedeSceneCfg(InteractiveSceneCfg):
    num_envs: int = 32
    env_spacing: float = 1.5
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane", spawn=sim_utils.GroundPlaneCfg()
    )
    light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DistantLightCfg(intensity=2000.0)
    )
    robot: ArticulationCfg = ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link_body_0/foot_.*",
        update_period=0.0,
        history_length=6,
        track_air_time=True,
        force_threshold=0.005,
        debug_vis=False,
    )


@configclass
class CentipedeEnvCfg(DirectRLEnvCfg):
    decimation: int = 12
    episode_length_s: float = 9.6
    is_finite_horizon: bool = False
    action_space: int = 1
    observation_space: int = 34
    state_space: int = 0
    sim: SimulationCfg = SimulationCfg(dt=0.001, render_interval=12)
    viewer: ViewerCfg = ViewerCfg(
        eye=(0.32, 0.32, 0.18),
        lookat=(0.0, 0.0, 0.025),
        origin_type="world",
        resolution=(1280, 720),
    )
    scene: CentipedeSceneCfg = CentipedeSceneCfg()

    # Runtime mode. Training uses tri; evaluation and open-loop sweeps use homogeneous.
    terrain_mode: str = "tri"
    terrain_label: str = "l60a12"
    terrain_lambda_mm: float = 60.0
    terrain_amplitude_mm: float = 12.0
    terrain_seed: int = 223607
    terrain_mix_seed: int = 20260729
    spawn_clearance_m: float = 0.025
    spawn_footprint_radius_m: float = 0.075
    observation_window_s: float = 1.0
    fixed_wave: float | None = None
