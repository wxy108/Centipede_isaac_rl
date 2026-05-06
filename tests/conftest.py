"""
Pytest configuration and shared fixtures for the parity tests.

These fixtures let us run controller logic *without* spinning up Isaac Sim,
by providing a MockArticulation that implements just the surface area the
controllers touch. This keeps the parity test suite fast (single-digit
seconds) and runnable on the laptop or any dev machine.
"""

from __future__ import annotations

import math
import numpy as np
import pytest
import torch
import yaml

from controllers.kinematics import (
    N_BODY_SEGS, N_BODY_JOINTS, N_LEGS, N_LEG_DOF, N_PITCH, N_ROLL,
    body_joint_name, pitch_joint_name, roll_joint_name,
    leg_joint_name, body_link_name,
)


# =============================================================================
# Joint / body name lists for the centipede articulation.
# =============================================================================

def make_centipede_joint_names(simplified: bool = True) -> list[str]:
    """
    Build the FARMS centipede's joint name list in the same order Isaac Lab
    would expose them. simplified=True drops the body pitch/roll joints
    (matching centipede_simple.xml).
    """
    names: list[str] = []
    # Body yaw joints (always present): joint_body_1 .. joint_body_19
    for i in range(1, N_BODY_JOINTS + 1):
        names.append(body_joint_name(i))
    # Body pitch/roll (only in full MJCF)
    if not simplified:
        for i in range(N_PITCH):
            names.append(pitch_joint_name(i))
        for i in range(N_ROLL):
            names.append(roll_joint_name(i))
    # Leg joints: per leg, two sides, four DOFs
    for n in range(N_LEGS):
        for side in ("L", "R"):
            for dof in range(N_LEG_DOF):
                names.append(leg_joint_name(n, side, dof))
    return names


def make_centipede_body_names() -> list[str]:
    """link_body_0 .. link_body_20"""
    return [body_link_name(i) for i in range(N_BODY_SEGS)]


# =============================================================================
# Mock articulation. Implements only the surface area both controllers use:
#   .device, .num_joints, .num_bodies, .joint_names, .body_names
#   .data.joint_pos, .data.joint_vel, .data.body_quat_w
#   .set_joint_effort_target(tensor), .write_data_to_sim()
# All tensors are torch tensors so the numpy controller's `.cpu().numpy()`
# calls work unchanged.
# =============================================================================


class _MockData:
    def __init__(self, num_envs: int, num_joints: int, num_bodies: int, device: str):
        self.joint_pos = torch.zeros(
            (num_envs, num_joints), dtype=torch.float32, device=device
        )
        self.joint_vel = torch.zeros(
            (num_envs, num_joints), dtype=torch.float32, device=device
        )
        # quaternion (w, x, y, z), identity = (1, 0, 0, 0)
        self.body_quat_w = torch.zeros(
            (num_envs, num_bodies, 4), dtype=torch.float32, device=device
        )
        self.body_quat_w[:, :, 0] = 1.0  # w = 1 → identity rotation


class MockArticulation:
    """
    Drop-in mock for isaaclab.assets.Articulation, sufficient for controller
    parity tests. Captures the most recent torque tensor written via
    set_joint_effort_target so tests can inspect it.
    """

    def __init__(
        self,
        joint_names: list[str],
        body_names: list[str],
        num_envs: int = 1,
        device: str = "cpu",
    ):
        self.joint_names = list(joint_names)
        self.body_names = list(body_names)
        self.num_envs = num_envs
        self.num_joints = len(self.joint_names)
        self.num_bodies = len(self.body_names)
        self.device = device
        self.data = _MockData(num_envs, self.num_joints, self.num_bodies, device)
        self.last_effort_target: torch.Tensor | None = None

    def set_joint_effort_target(self, tensor: torch.Tensor) -> None:
        self.last_effort_target = tensor.detach().clone()

    def write_data_to_sim(self) -> None:
        pass


# =============================================================================
# Pytest fixtures
# =============================================================================

PROJECT_ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "farms_controller.yaml"


@pytest.fixture(scope="session")
def config_path() -> str:
    """Absolute path to the canonical farms_controller.yaml."""
    assert DEFAULT_CONFIG.exists(), f"Missing config: {DEFAULT_CONFIG}"
    return str(DEFAULT_CONFIG)


@pytest.fixture(scope="session")
def config_dict() -> dict:
    """Parsed YAML config — useful for tests that don't need a controller."""
    with open(DEFAULT_CONFIG, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


@pytest.fixture
def mock_articulation_simple() -> MockArticulation:
    """
    Mock articulation matching centipede_simple.xml: body yaw + leg joints,
    no body pitch/roll. Single env, CPU. Matches what the current
    optimizer-tuned gains were trained against.
    """
    joint_names = make_centipede_joint_names(simplified=True)
    body_names = make_centipede_body_names()
    return MockArticulation(joint_names, body_names, num_envs=1, device="cpu")


@pytest.fixture
def mock_articulation_full() -> MockArticulation:
    """
    Mock articulation matching the full MJCF: body yaw + pitch + roll + leg.
    Used to exercise pitch/roll branches of the controller.
    """
    joint_names = make_centipede_joint_names(simplified=False)
    body_names = make_centipede_body_names()
    return MockArticulation(joint_names, body_names, num_envs=1, device="cpu")


@pytest.fixture
def rng() -> np.random.Generator:
    """Fixed-seed numpy RNG for reproducible parity test inputs."""
    return np.random.default_rng(seed=42)


@pytest.fixture
def torch_rng() -> torch.Generator:
    """Fixed-seed torch RNG for reproducible parity test inputs."""
    g = torch.Generator(device="cpu")
    g.manual_seed(42)
    return g
