"""One-path Centipede environment for the frozen 34-D/1-D experiment."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import torch
from isaaclab.envs import DirectRLEnv

from .controller import ReferenceGenerator
from .env_cfg import CONTROLLER_PATH, GAIT_MAP_PATH, CentipedeEnvCfg
from .gait import GaitMap, WaveActionLaw
from .model import BODY_LENGTH_M, ModelIndex
from .terrain import (
    ARENA_SIZE_M,
    TRI_X_M,
    TRI_Y_M,
    build_mesh,
    footprint_max,
    homogeneous_terrain,
    tri_terrain,
)


CONTACT_SEGMENTS = 19
CONTACT_SCALE_N = 0.05
CONTACT_SPIKE_K = 3.0
MAIN_SPINE_LINKS = 21
ROOT_ANGULAR_SCALE = 0.25
FAIL_TIP_COS = 0.259

OBSERVATION_NAMES = (
    "realized_wave", "cpg_sin", "cpg_cos",
    "root_vx", "root_vy", "root_vz",
    "root_wx_scaled", "root_wy_scaled", "root_wz_scaled",
    "gravity_x", "gravity_y", "gravity_z",
    "contact_std_dct0", "contact_std_dct1", "contact_std_dct2",
    "contact_spike_dct0", "contact_spike_dct1", "contact_spike_dct2",
    "omega_crest_log", "omega_spike_rate", "total_load_std", "total_load_spike_rate",
    "spine_y_dct0", "spine_y_dct1", "spine_y_dct2", "spine_y_dct3",
    "spine_z_dct0", "spine_z_dct1", "spine_z_dct2", "spine_z_dct3",
    "curvature_y_rms", "curvature_z_rms", "edge_cv", "tortuosity",
)

IMPEDANCE = {
    "body_yaw": (0.00996598695234778, 6.998080672480617e-08),
    "head_pitch": (0.03041804430974008, 0.00768829695690118),
    "body_pitch": (0.0006, 0.0009),
    "body_roll": (0.07536, 0.0036062445840513925),
    "legbase_yaw": (0.004293438984912552, 4.249695808879143e-06),
    "legbase_pitch": (0.000728032484289019, 9.855345633124885e-07),
    "legbase_roll": (0.00024, 2.94e-08),
    "knee": (0.00586, 0.000433),
}


def _dct_basis(samples: int, modes: int, device):
    rows = []
    for mode in range(modes):
        scale = math.sqrt(1.0 / samples) if mode == 0 else math.sqrt(2.0 / samples)
        rows.append([
            scale * math.cos(math.pi * (sample + 0.5) * mode / samples)
            for sample in range(samples)
        ])
    return torch.tensor(rows, dtype=torch.float32, device=device)


def _rotate_inverse(q: torch.Tensor, v: torch.Tensor):
    q = q / q.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    q_vec = q[:, None, 1:]
    q_w = q[:, None, :1]
    twice_cross = 2.0 * torch.cross(q_vec.expand_as(v), v, dim=-1)
    return v - q_w * twice_cross + torch.cross(q_vec.expand_as(v), twice_cross, dim=-1)


class CentipedeEnv(DirectRLEnv):
    """The complete supported runtime; no historical compatibility branches."""

    cfg: CentipedeEnvCfg

    def __init__(self, cfg: CentipedeEnvCfg, render_mode=None, **kwargs):
        if cfg.action_space != 1 or cfg.observation_space != 34:
            raise ValueError("Clean runtime requires action_space=1 and observation_space=34")
        if cfg.decimation != 12 or float(cfg.sim.dt) != 0.001:
            raise ValueError("Clean runtime requires a 12 ms policy period over 1 ms physics")
        super().__init__(cfg, render_mode, **kwargs)

        self.model = ModelIndex(self.robot)
        self.controller = ReferenceGenerator(
            self.robot, CONTROLLER_PATH, self.num_envs, self.device
        )
        self.gait_map = GaitMap(GAIT_MAP_PATH, self.device)
        self.actions = torch.zeros((self.num_envs, 1), dtype=torch.float32, device=self.device)
        initial_wave = float(cfg.fixed_wave) if cfg.fixed_wave is not None else WaveActionLaw.reset_wave
        if not WaveActionLaw.lower <= initial_wave <= WaveActionLaw.upper:
            raise ValueError(f"fixed_wave must be in [1,4], got {initial_wave}")
        self._wave_state = [initial_wave] * self.num_envs
        self._sim_time = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        self._pre_spine_x = torch.zeros_like(self._sim_time)
        self._spawn_rng = np.random.default_rng(int(cfg.terrain_mix_seed))

        window = int(round(float(cfg.observation_window_s) / float(cfg.sim.dt)))
        if window != 1000:
            raise ValueError(f"The frozen contact window must be 1000 substeps, got {window}")
        self._contact = torch.zeros(
            (self.num_envs, window, CONTACT_SEGMENTS), dtype=torch.float32, device=self.device
        )
        self._omega = torch.zeros((self.num_envs, window), dtype=torch.float32, device=self.device)
        self._total_load = torch.zeros_like(self._omega)
        self._ring_pos = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._ring_filled = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._foot_ids = None
        self._dct19 = _dct_basis(19, 3, self.device)
        self._dct21 = _dct_basis(21, 4, self.device)

        self._family_ids = self._make_family_ids()
        self._kp = torch.zeros(
            (self.num_envs, self.robot.num_joints), dtype=torch.float32, device=self.device
        )
        self._kv = torch.zeros_like(self._kp)
        for family, ids in self._family_ids.items():
            kp, kv = IMPEDANCE[family]
            self._kp[:, ids] = kp
            self._kv[:, ids] = kv
        self._active_ids = torch.cat(list(self._family_ids.values())).unique(sorted=True)
        if self._active_ids.numel() != 211:
            raise RuntimeError(f"Expected 211 active joints, got {self._active_ids.numel()}")
        self._gains_dirty = True

        if render_mode == "rgb_array":
            for _ in range(30):
                self.sim.render()

    def _setup_scene(self):
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetPath().pathString.endswith("/Robot/worldBody"):
                prim.SetActive(False)
        self.robot = self.scene.articulations["robot"]

        mode = str(self.cfg.terrain_mode)
        if mode == "tri":
            source = tri_terrain(int(self.cfg.terrain_seed))
        elif mode == "homogeneous":
            source = homogeneous_terrain(
                float(self.cfg.terrain_lambda_mm),
                float(self.cfg.terrain_amplitude_mm),
                int(self.cfg.terrain_seed),
            )
        else:
            raise ValueError(f"terrain_mode must be 'tri' or 'homogeneous', got {mode!r}")
        self._terrain_source = source
        self._terrain_collision = build_mesh(stage, "/World/CentipedeTerrain", source)
        self.scene.filter_collisions(
            global_prim_paths=["/World/GroundPlane", "/World/CentipedeTerrain"]
        )

    def _make_family_ids(self):
        def ids(values):
            return torch.as_tensor(values, dtype=torch.long, device=self.device)

        pitch = self.model.body_pitch
        legs = self.model.legs
        return {
            "body_yaw": ids(self.model.body_yaw),
            "head_pitch": ids(pitch[:1]),
            "body_pitch": ids(pitch[1:]),
            "body_roll": ids(self.model.body_roll),
            "legbase_yaw": ids(legs[:, :, 0].reshape(-1)),
            "legbase_pitch": ids(legs[:, :, 1].reshape(-1)),
            "legbase_roll": ids(legs[:, :, 2].reshape(-1)),
            "knee": ids(legs[:, :, 3].reshape(-1)),
        }

    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.to(device=self.device, dtype=torch.float32).clamp(-1.0, 1.0)
        spine_ids = torch.as_tensor(self.model.spine, dtype=torch.long, device=self.device)
        self._pre_spine_x = self.robot.data.body_pos_w[:, spine_ids, 0].mean(dim=1).clone()

        if self.cfg.fixed_wave is None:
            action_values = self.actions[:, 0].detach().cpu().tolist()
            self._wave_state = [
                WaveActionLaw.step(previous, action)
                for previous, action in zip(self._wave_state, action_values)
            ]
        else:
            self._wave_state = [float(self.cfg.fixed_wave)] * self.num_envs
        wave = torch.tensor(self._wave_state, dtype=torch.float32, device=self.device)
        gait = self.gait_map.at(wave)
        self.controller.set_gait(
            wave, gait["body_amp"], gait["leg_yaw_amp"], gait["leg_pitch_amp"]
        )
        self._kp[:, self._family_ids["body_yaw"]] = gait["body_yaw_kp"].unsqueeze(1)
        self._kv[:, self._family_ids["body_yaw"]] = gait["body_yaw_kv"].unsqueeze(1)
        self._kp[:, self._family_ids["legbase_yaw"]] = gait["legbase_yaw_kp"].unsqueeze(1)
        self._kv[:, self._family_ids["legbase_yaw"]] = gait["legbase_yaw_kv"].unsqueeze(1)
        self._gains_dirty = True

    def _apply_action(self):
        self.controller.step(self._sim_time, float(self.cfg.sim.dt))
        ids = self._active_ids
        if self._gains_dirty:
            self.robot.write_joint_stiffness_to_sim(self._kp[:, ids], joint_ids=ids)
            self.robot.write_joint_damping_to_sim(self._kv[:, ids], joint_ids=ids)
            actuator = self.robot.actuators.get("active")
            if actuator is not None:
                actuator.stiffness[:] = self.robot.data.joint_stiffness[:, actuator.joint_indices]
                actuator.damping[:] = self.robot.data.joint_damping[:, actuator.joint_indices]
            self._gains_dirty = False
        self.robot.set_joint_position_target(
            self.controller.position_target[:, ids], joint_ids=ids
        )
        self.robot.set_joint_velocity_target(
            self.controller.velocity_target[:, ids], joint_ids=ids
        )
        self.robot.set_joint_effort_target(
            torch.zeros((self.num_envs, ids.numel()), dtype=torch.float32, device=self.device),
            joint_ids=ids,
        )
        self._sim_time += float(self.cfg.sim.dt)
        self._accumulate_contact_window()

    def _resolve_feet(self):
        if self._foot_ids is not None:
            return True
        sensor = self.scene.sensors.get("contact_sensor")
        if sensor is None:
            return False
        names = list(sensor.body_names)
        grid = torch.empty((CONTACT_SEGMENTS, 2), dtype=torch.long, device=self.device)
        for segment in range(CONTACT_SEGMENTS):
            for side in range(2):
                name = f"foot_{segment}_{side}"
                if name not in names:
                    return False
                grid[segment, side] = names.index(name)
        self._foot_ids = grid
        return True

    def _accumulate_contact_window(self):
        if not self._resolve_feet():
            return
        sensor = self.scene.sensors["contact_sensor"]
        magnitude = sensor.data.net_forces_w.norm(dim=-1)
        paired = magnitude[:, self._foot_ids].sum(dim=-1)
        contact = (paired / CONTACT_SCALE_N).clamp(0.0, 2.0)
        total = paired.sum(dim=1) / CONTACT_SCALE_N
        omega = self.robot.data.root_ang_vel_b.norm(dim=-1)
        rows = torch.arange(self.num_envs, dtype=torch.long, device=self.device)
        self._contact[rows, self._ring_pos] = contact
        self._omega[rows, self._ring_pos] = omega
        self._total_load[rows, self._ring_pos] = total
        self._ring_pos = (self._ring_pos + 1) % self._contact.shape[1]
        self._ring_filled = torch.clamp(self._ring_filled + 1, max=self._contact.shape[1])

    def _get_observations(self):
        if not hasattr(self, "controller"):
            return {"policy": torch.zeros((self.num_envs, 34), device=self.device)}

        wave = self.controller.realized_wave.to(torch.float32).unsqueeze(1)
        if self.controller.body_phase is None or not self.controller.cpg_initialized:
            cpg = torch.zeros((self.num_envs, 2), dtype=torch.float32, device=self.device)
            cpg[:, 1] = 1.0
        else:
            phase = self.controller.body_phase[:, 0].to(torch.float32)
            cpg = torch.stack((torch.sin(phase), torch.cos(phase)), dim=1)
        root = torch.cat((
            self.robot.data.root_lin_vel_b.to(torch.float32),
            self.robot.data.root_ang_vel_b.to(torch.float32) * ROOT_ANGULAR_SCALE,
            self.robot.data.projected_gravity_b.to(torch.float32),
        ), dim=1)

        width = self._contact.shape[1]
        full = (self._ring_filled >= width).to(torch.float32).unsqueeze(1)
        contact_median = torch.sort(self._contact, dim=1).values[:, width // 2]
        omega_median = torch.sort(self._omega, dim=1).values[:, width // 2]
        total_median = torch.sort(self._total_load, dim=1).values[:, width // 2]
        contact_std = self._contact.std(dim=1, unbiased=False)
        contact_spike = (
            self._contact > (CONTACT_SPIKE_K * contact_median).unsqueeze(1)
        ).to(torch.float32).mean(dim=1)
        omega_rms = torch.sqrt((self._omega ** 2).mean(dim=1) + 1e-12)
        global4 = torch.stack((
            torch.log1p(self._omega.max(dim=1).values / (omega_rms + 1e-9)),
            (self._omega > (CONTACT_SPIKE_K * omega_median).unsqueeze(1)).float().mean(dim=1),
            self._total_load.std(dim=1, unbiased=False),
            (self._total_load > (CONTACT_SPIKE_K * total_median).unsqueeze(1)).float().mean(dim=1),
        ), dim=1)
        contact10 = torch.cat((
            contact_std @ self._dct19.T,
            contact_spike @ self._dct19.T,
            global4,
        ), dim=1) * full

        main_ids = torch.as_tensor(self.model.main_body, dtype=torch.long, device=self.device)
        main_world = self.robot.data.body_pos_w[:, main_ids].to(torch.float32)
        relative_world = main_world - main_world[:, :1]
        relative_body = _rotate_inverse(
            self.robot.data.root_quat_w.to(torch.float32), relative_world
        )
        parameter = torch.linspace(0.0, 1.0, MAIN_SPINE_LINKS, device=self.device).view(
            1, MAIN_SPINE_LINKS, 1
        )
        chord = relative_body[:, -1:]
        chord_length = chord[:, 0].norm(dim=1).clamp_min(1e-4)
        residual = (relative_body - parameter * chord) / chord_length[:, None, None]
        curvature = torch.diff(residual, n=2, dim=1)
        edge_length = torch.diff(relative_body, dim=1).norm(dim=2)
        edge_mean = edge_length.mean(dim=1).clamp_min(1e-6)
        spine12 = torch.cat((
            residual[:, :, 1] @ self._dct21.T,
            residual[:, :, 2] @ self._dct21.T,
            torch.sqrt((curvature[:, :, 1] ** 2).mean(dim=1)).unsqueeze(1),
            torch.sqrt((curvature[:, :, 2] ** 2).mean(dim=1)).unsqueeze(1),
            (edge_length.std(dim=1, unbiased=False) / edge_mean).unsqueeze(1),
            (edge_length.sum(dim=1) / chord_length).unsqueeze(1),
        ), dim=1)
        observation = torch.cat((wave, cpg, root, contact10, spine12), dim=1)
        if observation.shape != (self.num_envs, len(OBSERVATION_NAMES)):
            raise RuntimeError(f"Observation shape drift: {tuple(observation.shape)}")
        if not torch.isfinite(observation).all():
            raise RuntimeError("Observation contains NaN or Inf")
        return {"policy": observation.to(torch.float32)}

    def _failure_mask(self):
        root_pos = self.robot.data.root_pos_w
        root_vel = self.robot.data.root_lin_vel_w
        gravity = self.robot.data.projected_gravity_b
        return (
            (gravity[:, 2].abs() < FAIL_TIP_COS)
            | (root_pos[:, 2] < 0.0)
            | ~torch.isfinite(root_pos).all(dim=1)
            | ~torch.isfinite(root_vel).all(dim=1)
        )

    def _get_rewards(self):
        if not hasattr(self, "model"):
            return torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        spine_ids = torch.as_tensor(self.model.spine, dtype=torch.long, device=self.device)
        post_x = self.robot.data.body_pos_w[:, spine_ids, 0].mean(dim=1)
        reward = -(post_x - self._pre_spine_x) / BODY_LENGTH_M
        terminal = self._failure_mask() | (self._sim_time >= float(self.cfg.episode_length_s))
        reward = reward.masked_fill(terminal, 0.0).to(torch.float32)
        self.extras["log"] = {
            "reward/forward_spine_displacement_bl": reward.detach(),
            "wave/realized": self.controller.realized_wave.detach(),
            "failure/rate": self._failure_mask().float().mean().detach(),
        }
        return reward

    def _get_dones(self):
        if not hasattr(self, "_sim_time"):
            zeros = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            return zeros, zeros
        return self._failure_mask(), self._sim_time >= float(self.cfg.episode_length_s)

    def _reset_idx(self, env_ids):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        default_root = self.robot.data.default_root_state[env_ids].clone()
        default_root[:, :3] += self.scene.env_origins[env_ids]
        ids_cpu = env_ids.detach().cpu().tolist()
        for row, env_id in enumerate(ids_cpu):
            if str(self.cfg.terrain_mode) == "tri":
                terrain_id = int(self._spawn_rng.integers(0, len(TRI_Y_M)))
                x_m, y_m = TRI_X_M, TRI_Y_M[terrain_id]
            else:
                x_m = 0.0
                y_m = -1.0 if int(env_id) % 2 == 0 else 1.0
            z_m = footprint_max(
                self._terrain_collision,
                x_m,
                y_m,
                float(self.cfg.spawn_footprint_radius_m),
                ARENA_SIZE_M,
            ) + float(self.cfg.spawn_clearance_m)
            default_root[row, 0:3] = torch.tensor(
                (x_m, y_m, z_m), dtype=default_root.dtype, device=default_root.device
            )
        self.robot.write_root_pose_to_sim(default_root[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root[:, 7:13], env_ids)
        self.robot.write_joint_state_to_sim(
            self.robot.data.default_joint_pos[env_ids].clone(),
            self.robot.data.default_joint_vel[env_ids].clone(),
            None,
            env_ids,
        )

        if hasattr(self, "_sim_time"):
            self._sim_time[env_ids] = 0.0
            initial_wave = (
                float(self.cfg.fixed_wave)
                if self.cfg.fixed_wave is not None
                else WaveActionLaw.reset_wave
            )
            for env_id in ids_cpu:
                self._wave_state[int(env_id)] = initial_wave
            self.controller.reset(env_ids)
            self._contact[env_ids] = 0.0
            self._omega[env_ids] = 0.0
            self._total_load[env_ids] = 0.0
            self._ring_pos[env_ids] = 0
            self._ring_filled[env_ids] = 0
            self._gains_dirty = True

