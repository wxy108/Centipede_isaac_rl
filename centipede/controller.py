"""Torch-vectorized open-loop CPG reference generator.

The controller emits joint position and velocity targets only. PhysX implicit
PD is the sole feedback and force path.
"""

from __future__ import annotations

import math
from pathlib import Path

import torch
import yaml

from .model import ModelIndex, N_BODY_YAW, N_LEG_DOF, N_LEG_PAIRS


def quaternion_yaw(q: torch.Tensor) -> torch.Tensor:
    w, x, y, z = q.unbind(dim=-1)
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def dct_blend(t: torch.Tensor, indices: torch.Tensor, settle: float, ramp: float, n: int):
    """Sequential cosine fade used by the frozen gait startup."""
    t = t.to(torch.float64).unsqueeze(1)
    if ramp <= 0.0:
        return (t >= settle).to(torch.float64).expand(-1, len(indices)).clone()
    duration = ramp * 0.5
    starts = (settle + indices.to(torch.float64) * duration / max(n - 1, 1)).unsqueeze(0)
    elapsed = torch.clamp(t - starts, min=0.0)
    value = 0.5 * (1.0 - torch.cos(math.pi * elapsed / duration))
    value = torch.where(elapsed >= duration, torch.ones_like(value), value)
    return torch.where(t <= starts, torch.zeros_like(value), value)


def kuramoto_chain(phi, omega, gain, delta, dt):
    envs, count = phi.shape
    rate = omega.unsqueeze(1).expand(envs, count).clone()
    forward = torch.sin(phi[:, :-1] - (phi[:, 1:] + delta))
    backward = torch.sin(phi[:, 1:] - (phi[:, :-1] - delta))
    rate[:, 0] += gain * forward[:, 0]
    if count > 2:
        rate[:, 1:-1] += gain.unsqueeze(1) * (backward[:, :-1] + forward[:, 1:])
    rate[:, -1] += gain * backward[:, -1]
    return phi + dt * rate


def kuramoto_legs(phi, anchor_phi, omega, gain, dt):
    return phi + dt * (omega.unsqueeze(1) + gain * torch.sin(phi - anchor_phi))


class ReferenceGenerator:
    """Frozen 1 Hz, 19-segment CPG with per-environment wave commands."""

    def __init__(self, articulation, config_path, num_envs: int, device):
        config_path = Path(config_path).resolve()
        cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if "impedance" in cfg:
            raise ValueError("Controller config must use reference_targets, not impedance")

        self.articulation = articulation
        self.config_path = str(config_path)
        self.num_envs = int(num_envs)
        self.device = device
        self.dtype = torch.float32
        self.index = ModelIndex(articulation)
        self.num_joints = articulation.num_joints

        def ids(value):
            return torch.as_tensor(value, dtype=torch.long, device=device)

        self.body_ids = ids(self.index.body_yaw)
        self.pitch_ids = ids(self.index.body_pitch)
        self.roll_ids = ids(self.index.body_roll)
        self.leg_ids = ids(self.index.legs.reshape(-1))
        self.head_body_id = int(self.index.head_body)

        body = cfg["body_wave"]
        self.body_amplitude = float(body["amplitude"])
        self.frequency_hz = float(body["frequency"])
        self.base_wave_number = float(body["wave_number"])
        self.wave_speed = float(body["speed"])

        cpg = cfg["cpg"]
        if not bool(cpg.get("enabled", True)):
            raise ValueError("The frozen runtime requires CPG enabled")
        self.body_coupling = float(cpg["coupling_omega"]) * 40.0
        self.leg_coupling = float(cpg["leg_coupling_omega"]) * 80.0

        targets = cfg["reference_targets"]
        allowed = {
            "head_pitch_offset", "head_pitch_joints", "settle_time", "ramp_time",
            "head_yaw_gain", "head_yaw_target_clip", "head_yaw_rate",
        }
        unknown = set(targets) - allowed
        if unknown:
            raise ValueError(f"Unknown reference-target keys: {sorted(unknown)}")
        self.settle_time = float(targets["settle_time"])
        self.ramp_time = float(targets["ramp_time"])
        self.head_pitch_offset = float(targets["head_pitch_offset"])
        self.head_pitch_joints = int(targets["head_pitch_joints"])
        self.head_yaw_gain = float(targets["head_yaw_gain"])
        self.head_yaw_clip = float(targets["head_yaw_target_clip"])
        self.head_yaw_rate = float(targets["head_yaw_rate"])

        pitch_target = torch.zeros(len(self.pitch_ids), dtype=self.dtype, device=device)
        pitch_target[: self.head_pitch_joints] = self.head_pitch_offset
        self.pitch_target = pitch_target

        leg = cfg["leg_wave"]
        amplitudes = torch.tensor(leg["amplitudes"][:N_LEG_DOF], dtype=self.dtype, device=device)
        phase_offsets = torch.tensor(leg["phase_offsets"][:N_LEG_DOF], dtype=self.dtype, device=device)
        dc_offsets = torch.tensor(leg["dc_offsets"][:N_LEG_DOF], dtype=self.dtype, device=device)
        active_dofs = set(int(x) for x in leg["active_dofs"])

        self.leg_sign = torch.tensor(
            [sign for _ in range(N_LEG_PAIRS) for sign in (1.0, -1.0) for _ in range(N_LEG_DOF)],
            dtype=self.dtype, device=device,
        )
        self.leg_amplitude = amplitudes.repeat(N_LEG_PAIRS * 2)
        self.leg_amplitude_reference = float(leg["amplitudes"][0])
        self.leg_phase_offset = phase_offsets.repeat(N_LEG_PAIRS * 2)
        self.leg_dc = dc_offsets.repeat(N_LEG_PAIRS * 2)
        self.leg_active = torch.tensor(
            [dof in active_dofs for _ in range(N_LEG_PAIRS * 2) for dof in range(N_LEG_DOF)],
            dtype=torch.bool, device=device,
        )
        self.leg_segment = torch.arange(N_LEG_PAIRS, device=device).repeat_interleave(2 * N_LEG_DOF)
        self.leg_dof = torch.arange(N_LEG_DOF, device=device).repeat(N_LEG_PAIRS * 2)

        phase = [
            2.0 * math.pi * self.base_wave_number * self.wave_speed * i / (N_BODY_YAW - 1)
            for i in range(N_BODY_YAW)
        ]
        self.spatial_phase = torch.tensor(phase, dtype=self.dtype, device=device)

        self.body_scale = torch.ones((num_envs,), dtype=self.dtype, device=device)
        self.leg_yaw_scale = torch.ones_like(self.body_scale)
        self.leg_pitch_scale = torch.ones_like(self.body_scale)
        self.wave_scale = torch.ones_like(self.body_scale)
        self.body_phase = None
        self.leg_phase = None
        self.cpg_initialized = False
        self.head_yaw_reference = None
        self.previous_body_target = None
        self.previous_pitch_target = None
        self.previous_leg_target = None

    @property
    def realized_wave(self):
        return self.wave_scale * self.base_wave_number

    def set_gait(self, wave, body_scale, leg_yaw_scale, leg_pitch_scale):
        for name, value in (
            ("wave", wave), ("body_scale", body_scale),
            ("leg_yaw_scale", leg_yaw_scale), ("leg_pitch_scale", leg_pitch_scale),
        ):
            if value.shape != (self.num_envs,):
                raise ValueError(f"{name} must have shape ({self.num_envs},), got {tuple(value.shape)}")
        self.wave_scale.copy_(wave.to(self.dtype) / self.base_wave_number)
        self.body_scale.copy_(body_scale.to(self.dtype))
        self.leg_yaw_scale.copy_(leg_yaw_scale.to(self.dtype))
        self.leg_pitch_scale.copy_(leg_pitch_scale.to(self.dtype))

    def _omega(self):
        return torch.full(
            (self.num_envs,), 2.0 * math.pi * self.frequency_hz,
            dtype=self.dtype, device=self.device,
        )

    def _spatial(self):
        return self.wave_scale.unsqueeze(1) * self.spatial_phase.unsqueeze(0)

    def _delta(self):
        base = 2.0 * math.pi * self.base_wave_number * self.wave_speed / (N_BODY_YAW - 1)
        return base * self.wave_scale.unsqueeze(1)

    def _leg_phase_anchor(self, body_phase):
        """Sample each leg at the center of its two neighboring body-yaw joints."""
        return body_phase[:, :N_LEG_PAIRS] + 0.5 * self._delta()

    def _initialize_phases(self, time_s):
        self.body_phase = self._omega().unsqueeze(1) * time_s.unsqueeze(1) - self._spatial()
        self.leg_phase = self._leg_phase_anchor(self.body_phase).clone()
        self.cpg_initialized = True

    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, dtype=torch.long, device=self.device)
        else:
            env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device).reshape(-1)
        if not len(env_ids):
            return
        if self.body_phase is None:
            self.body_phase = torch.zeros(
                (self.num_envs, N_BODY_YAW), dtype=self.dtype, device=self.device
            )
            self.leg_phase = torch.zeros(
                (self.num_envs, N_LEG_PAIRS), dtype=self.dtype, device=self.device
            )
        self.body_phase[env_ids] = -self._spatial()[env_ids]
        self.leg_phase[env_ids] = self._leg_phase_anchor(self.body_phase)[env_ids]
        if self.head_yaw_reference is None:
            self.head_yaw_reference = torch.full(
                (self.num_envs,), float("nan"), dtype=self.dtype, device=self.device
            )
        self.head_yaw_reference[env_ids] = float("nan")
        for previous in (
            self.previous_body_target, self.previous_pitch_target, self.previous_leg_target
        ):
            if previous is not None:
                previous[env_ids] = 0.0
        self.body_scale[env_ids] = 1.0
        self.leg_yaw_scale[env_ids] = 1.0
        self.leg_pitch_scale[env_ids] = 1.0
        self.wave_scale[env_ids] = 1.0

    def step(self, time_s: torch.Tensor, dt: float):
        time_s = time_s.to(dtype=self.dtype, device=self.device)
        if not self.cpg_initialized and bool((time_s >= self.settle_time).any().item()):
            self._initialize_phases(time_s)
        if self.cpg_initialized:
            body_gain = self.body_amplitude * self.body_coupling
            self.body_phase = kuramoto_chain(
                self.body_phase,
                self._omega(),
                torch.full((self.num_envs,), body_gain, dtype=self.dtype, device=self.device),
                self._delta(),
                dt,
            )
            self.leg_phase = kuramoto_legs(
                self.leg_phase,
                self._leg_phase_anchor(self.body_phase),
                self._omega(),
                self.leg_amplitude_reference * self.leg_coupling,
                dt,
            )

        head_quat = self.articulation.data.body_quat_w[:, self.head_body_id].to(self.dtype)
        yaw = quaternion_yaw(head_quat)
        if self.head_yaw_reference is None:
            self.head_yaw_reference = yaw.clone()
        else:
            self.head_yaw_reference = torch.where(
                torch.isnan(self.head_yaw_reference), yaw, self.head_yaw_reference
            )
        if self.head_yaw_rate:
            self.head_yaw_reference = torch.where(
                time_s >= self.settle_time,
                self.head_yaw_reference + self.head_yaw_rate * dt,
                self.head_yaw_reference,
            )
        yaw_error = ((yaw - self.head_yaw_reference + math.pi) % (2.0 * math.pi)) - math.pi
        head_target = torch.clamp(
            self.head_yaw_gain * yaw_error, -self.head_yaw_clip, self.head_yaw_clip
        )

        body_indices = torch.arange(1, N_BODY_YAW, device=self.device)
        body_blend = dct_blend(
            time_s, body_indices, self.settle_time, self.ramp_time, N_BODY_YAW
        ).to(self.dtype)
        phases = (
            self.body_phase[:, 1:]
            if self.cpg_initialized
            else (self._omega().unsqueeze(1) * time_s.unsqueeze(1) - self._spatial())[:, 1:]
        )
        body_target = torch.empty(
            (self.num_envs, N_BODY_YAW), dtype=self.dtype, device=self.device
        )
        body_target[:, 0] = head_target
        body_target[:, 1:] = (
            body_blend * self.body_scale.unsqueeze(1) * self.body_amplitude * torch.sin(phases)
        )
        body_velocity = self._target_velocity(
            body_target, self.previous_body_target, dt
        )
        self.previous_body_target = body_target.clone()

        pitch_blend = dct_blend(
            time_s, torch.tensor([0], device=self.device),
            self.settle_time, self.ramp_time, N_BODY_YAW,
        ).to(self.dtype)
        pitch_target = pitch_blend * self.pitch_target.unsqueeze(0)
        pitch_velocity = self._target_velocity(
            pitch_target, self.previous_pitch_target, dt
        )
        self.previous_pitch_target = pitch_target.clone()

        leg_indices = torch.arange(N_LEG_PAIRS, device=self.device)
        leg_blend = dct_blend(
            time_s, leg_indices, self.settle_time, self.ramp_time, N_LEG_PAIRS
        ).to(self.dtype)
        leg_phase = (
            self.leg_phase
            if self.cpg_initialized
            else self._leg_phase_anchor(
                self._omega().unsqueeze(1) * time_s.unsqueeze(1) - self._spatial()
            )
        )
        flat_phase = leg_phase[:, self.leg_segment] + self.leg_phase_offset.unsqueeze(0)
        flat_blend = leg_blend[:, self.leg_segment]
        runtime = torch.ones(
            (self.num_envs, len(self.leg_amplitude)), dtype=self.dtype, device=self.device
        )
        runtime[:, self.leg_dof == 0] = self.leg_yaw_scale.unsqueeze(1)
        runtime[:, self.leg_dof == 1] = self.leg_pitch_scale.unsqueeze(1)
        wave_target = (
            flat_blend * self.leg_sign.unsqueeze(0) * self.leg_amplitude.unsqueeze(0)
            * runtime * torch.sin(flat_phase) + self.leg_dc.unsqueeze(0)
        )
        leg_target = torch.where(
            self.leg_active.unsqueeze(0) & (flat_blend > 0.0),
            wave_target,
            self.leg_dc.unsqueeze(0),
        )
        leg_velocity = self._target_velocity(leg_target, self.previous_leg_target, dt)
        self.previous_leg_target = leg_target.clone()

        position = torch.zeros(
            (self.num_envs, self.num_joints), dtype=self.dtype, device=self.device
        )
        velocity = torch.zeros_like(position)
        position[:, self.body_ids] = body_target
        velocity[:, self.body_ids] = body_velocity
        position[:, self.pitch_ids] = pitch_target
        velocity[:, self.pitch_ids] = pitch_velocity
        position[:, self.leg_ids] = leg_target
        velocity[:, self.leg_ids] = leg_velocity
        self.position_target = position
        self.velocity_target = velocity

    @staticmethod
    def _target_velocity(current, previous, dt):
        if previous is None or dt <= 1e-12:
            return torch.zeros_like(current)
        return (current - previous) / dt
