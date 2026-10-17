from __future__ import annotations

import hashlib
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

from centipede.controller import ReferenceGenerator
from centipede.gait import GaitMap, WaveActionLaw
from centipede.terrain import to_mujoco_collision, tri_terrain


ROOT = Path(__file__).resolve().parents[1]
GOLDEN_CONTROLLER_SHA256 = "ffa91f526dc739bb23647021cb0d7420b2c1f5931d54c22bfb61e660cfb2a982"
TRI_SOURCE_SHA256 = "024fc9f8eb1cb4a62df0be6515c0e6f3e1931e3edd1109ed3d51f88de2c466d4"
TRI_COLLISION_SHA256 = "dd8ab150ab1bb33ac6ff2fcfb0a816beea309047885192ec9cbd161a65f9142d"


def _array_hash(array):
    value = np.ascontiguousarray(array)
    digest = hashlib.sha256()
    digest.update(str(value.dtype).encode("ascii"))
    digest.update(str(list(value.shape)).replace("'", '"').encode("ascii"))
    digest.update(value.view(np.uint8).tobytes())
    return digest.hexdigest()


class DummyArticulation:
    def __init__(self, envs=4):
        self.joint_names = (
            [f"body_yaw_{i}" for i in range(1, 20)]
            + [f"body_pitch_{i}" for i in range(20)]
            + [f"body_roll_{i}" for i in range(20)]
            + [f"{family}_{segment}_{side}"
               for segment in range(19)
               for side in ("L", "R")
               for family in ("legbase_yaw", "legbase_pitch", "legbase_roll", "knee")]
        )
        self.body_names = (
            [f"link_body_{i}" for i in range(21)]
            + [f"link_spine_extra_{i}" for i in range(40)]
        )
        self.num_joints = len(self.joint_names)
        quat = torch.zeros((envs, len(self.body_names), 4), dtype=torch.float32)
        quat[:, :, 0] = 1.0
        self.data = SimpleNamespace(body_quat_w=quat)


def test_wave_action_bounds_and_center():
    assert WaveActionLaw.target(-1.0) == 1.0
    assert WaveActionLaw.target(0.0) == 2.5
    assert WaveActionLaw.target(1.0) == 4.0
    assert WaveActionLaw.step(2.5, 0.0) == 2.5
    assert WaveActionLaw.step(2.5, 1.0) == 2.542


def test_gait_map_contract():
    gait = GaitMap(ROOT / "config" / "gait_map.json", "cpu")
    assert gait.waves.tolist() == [1.0 + 0.25 * i for i in range(13)]
    result = gait.at(torch.tensor([1.0, 2.5, 4.0]))
    assert torch.equal(result["leg_pitch_amp"], torch.full((3,), 1.15))
    assert float(result["body_yaw_kp"][1]) == torch.tensor(0.00996598695234778).item()


def test_leg_phase_samples_segment_center():
    controller = ReferenceGenerator(
        DummyArticulation(), ROOT / "config" / "controller.yaml", 4, "cpu"
    )
    wave = torch.tensor([1.0, 2.0, 3.0, 4.0])
    ones = torch.ones(4)
    controller.set_gait(wave, ones, ones, ones)
    controller._initialize_phases(torch.zeros(4))
    offset = controller.leg_phase - controller.body_phase[:, :19]
    assert torch.allclose(offset, 0.5 * controller._delta().expand_as(offset))


def test_controller_matches_segment_center_golden():
    controller = ReferenceGenerator(
        DummyArticulation(), ROOT / "config" / "controller.yaml", 4, "cpu"
    )
    gait_map = GaitMap(ROOT / "config" / "gait_map.json", "cpu")
    states = [2.5] * 4
    actions = (-1.0, -0.25, 0.5, 1.0)
    digest = hashlib.sha256()
    for substep in range(1800):
        if substep % 12 == 0:
            states = [
                WaveActionLaw.step(wave, action)
                for wave, action in zip(states, actions)
            ]
            wave = torch.tensor(states)
            mapped = gait_map.at(wave)
            controller.set_gait(
                wave, mapped["body_amp"], mapped["leg_yaw_amp"], mapped["leg_pitch_amp"]
            )
        controller.step(torch.full((4,), substep * 0.001), 0.001)
        digest.update(controller.position_target.numpy().tobytes())
        digest.update(controller.velocity_target.numpy().tobytes())
    assert digest.hexdigest() == GOLDEN_CONTROLLER_SHA256


def test_tri_terrain_contract():
    source = tri_terrain(223607)
    collision = to_mujoco_collision(source)
    assert _array_hash(source) == TRI_SOURCE_SHA256
    assert _array_hash(collision) == TRI_COLLISION_SHA256


def test_checkpoint_shapes():
    for seed in (161803, 141421, 173205):
        checkpoint = torch.load(
            ROOT / "checkpoints" / f"policy_seed_{seed}.pth",
            map_location="cpu",
            weights_only=True,
        )
        state = checkpoint["model"]
        assert checkpoint["epoch"] == 24
        assert tuple(state["running_mean_std.running_mean"].shape) == (34,)
        assert tuple(state["a2c_network.actor_mlp.0.weight"].shape) == (256, 34)
        assert tuple(state["a2c_network.mu.weight"].shape) == (1, 128)
