"""
Parity tests: torch port must reproduce the numpy reference.

The numpy controller (controllers/impedance_controller.py) is the parity
oracle. Each test below pins one piece of the torch port against the
numpy reference for a known input.

Run all parity tests:
    pytest tests/test_controller_parity.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import torch

import controllers.impedance_controller as ref
import controllers.impedance_controller_torch as port


# =============================================================================
# Smoke tests
# =============================================================================

def test_imports_succeed():
    assert hasattr(ref, "ImpedanceTravelingWaveController")
    assert hasattr(port, "ImpedanceTravelingWaveControllerTorch")


def test_module_constants_consistent():
    from controllers.kinematics import (
        N_BODY_JOINTS, N_LEGS, N_LEG_DOF, N_PITCH, N_ROLL,
    )
    assert N_BODY_JOINTS == 19
    assert N_LEGS == 19
    assert N_LEG_DOF == 4


# =============================================================================
# Section 1: stateless helpers
# =============================================================================

@pytest.mark.parametrize("yaw_true", [-1.5, -0.3, 0.0, 0.3, 1.5, 3.0, -3.0])
def test_quat_to_yaw_matches_numpy(yaw_true: float):
    half = yaw_true * 0.5
    q = np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)
    yaw_ref = ref.quat_to_yaw(q)
    q_t = torch.tensor(q, dtype=torch.float64).unsqueeze(0)
    yaw_port = port.quat_to_yaw(q_t).item()
    def wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi
    assert math.isclose(wrap(yaw_ref), wrap(yaw_port), abs_tol=1e-10), (
        f"quat_to_yaw mismatch: ref={yaw_ref}, port={yaw_port}"
    )


def test_pd_torque_matches_formula(rng):
    n_envs, n_joints = 4, 19
    target = torch.from_numpy(rng.standard_normal((n_envs, n_joints)))
    q = torch.from_numpy(rng.standard_normal((n_envs, n_joints)))
    target_dot = torch.from_numpy(rng.standard_normal((n_envs, n_joints)))
    qdot = torch.from_numpy(rng.standard_normal((n_envs, n_joints)))
    kp = torch.from_numpy(rng.uniform(0.01, 1.0, size=(n_envs, n_joints)))
    kv = torch.from_numpy(rng.uniform(0.001, 0.1, size=(n_envs, n_joints)))
    tau_port = port.pd_torque(kp, kv, target, q, target_dot, qdot)
    tau_ref = kp * (target - q) + kv * (target_dot - qdot)
    assert torch.allclose(tau_port, tau_ref, atol=1e-12, rtol=0.0)


def test_kuramoto_body_step_matches_numpy(rng):
    N = 19
    omega = 2.0 * math.pi * 1.0
    cpg_omega = -2.5
    body_amp = 0.4
    n_wave, speed = 3.0, 1.0
    delta = 2.0 * math.pi * n_wave * speed / max(N - 1, 1)
    dt = 5e-4
    coupling_gain = body_amp * cpg_omega
    phi_np = rng.uniform(-math.pi, math.pi, size=N)
    fdot = np.full(N, omega)
    fwd = np.sin(phi_np[:-1] - (phi_np[1:] + delta))
    bwd = np.sin(phi_np[1:] - (phi_np[:-1] - delta))
    fdot[0] += coupling_gain * fwd[0]
    fdot[1:-1] += coupling_gain * (bwd[:-1] + fwd[1:])
    fdot[-1] += coupling_gain * bwd[-1]
    phi_ref = phi_np + fdot * dt
    phi_t = torch.from_numpy(phi_np).unsqueeze(0)
    phi_new = port.kuramoto_body_step(
        phi_t, omega=omega, coupling_gain=coupling_gain, delta=delta, dt=dt,
    )
    assert phi_new.shape == (1, N)
    assert torch.allclose(phi_new[0], torch.from_numpy(phi_ref),
                          atol=1e-12, rtol=0.0)


def test_kuramoto_body_step_batched_independence(rng):
    N = 19
    omega = 2.0 * math.pi * 1.0
    delta = 2.0 * math.pi * 3.0 / max(N - 1, 1)
    dt = 5e-4
    gain = 0.4 * -2.5
    phi_a = rng.uniform(-math.pi, math.pi, size=N)
    phi_b = rng.uniform(-math.pi, math.pi, size=N)
    out_a = port.kuramoto_body_step(
        torch.from_numpy(phi_a).unsqueeze(0), omega, gain, delta, dt
    )[0]
    out_b = port.kuramoto_body_step(
        torch.from_numpy(phi_b).unsqueeze(0), omega, gain, delta, dt
    )[0]
    stacked = torch.stack(
        [torch.from_numpy(phi_a), torch.from_numpy(phi_b)], dim=0
    )
    out_stacked = port.kuramoto_body_step(stacked, omega, gain, delta, dt)
    assert torch.allclose(out_stacked[0], out_a, atol=1e-12, rtol=0.0)
    assert torch.allclose(out_stacked[1], out_b, atol=1e-12, rtol=0.0)


def test_kuramoto_leg_step_matches_numpy(rng):
    from controllers.kinematics import N_LEGS
    omega = 2.0 * math.pi * 1.0
    gain_leg = 0.6 * -2.5
    dt = 5e-4
    phi_leg_np = rng.uniform(-math.pi, math.pi, size=N_LEGS)
    phi_body_np = rng.uniform(-math.pi, math.pi, size=N_LEGS)
    fdot = omega + gain_leg * np.sin(phi_leg_np - phi_body_np)
    phi_ref = phi_leg_np + fdot * dt
    out = port.kuramoto_leg_step(
        torch.from_numpy(phi_leg_np).unsqueeze(0),
        torch.from_numpy(phi_body_np).unsqueeze(0),
        omega=omega, coupling_gain_leg=gain_leg, dt=dt,
    )
    assert out.shape == (1, N_LEGS)
    assert torch.allclose(out[0], torch.from_numpy(phi_ref),
                          atol=1e-12, rtol=0.0)


def test_head_amp_scale_parity():
    from controllers.kinematics import N_BODY_JOINTS
    n = N_BODY_JOINTS
    H, R, amin = 1, 2, 0.0
    scale_ref = np.ones(n, dtype=float)
    for i in range(min(H, n)):
        scale_ref[i] = 0.0
    if R > 0:
        for k in range(1, R + 1):
            i = H + k - 1
            if i >= n:
                break
            s = amin + (1.0 - amin) * 0.5 * (
                1.0 - math.cos(math.pi * k / (R + 1)))
            scale_ref[i] = s
    scale_port = port.build_body_amp_scale(
        n=n, head_hold_joints=H, head_ramp_joints=R, head_ramp_min=amin,
    )
    assert torch.allclose(scale_port, torch.from_numpy(scale_ref),
                          atol=1e-12, rtol=0.0)


def test_head_amp_scale_edge_cases():
    from controllers.kinematics import N_BODY_JOINTS
    scale = port.build_body_amp_scale(N_BODY_JOINTS, 0, 0, 0.0)
    assert torch.allclose(scale, torch.ones(N_BODY_JOINTS, dtype=torch.float64))
    scale = port.build_body_amp_scale(N_BODY_JOINTS, N_BODY_JOINTS, 0, 0.0)
    assert torch.allclose(scale, torch.zeros(N_BODY_JOINTS, dtype=torch.float64))
    scale = port.build_body_amp_scale(5, 1, 100, 0.0)
    expected = np.zeros(5, dtype=float)
    for k in range(1, 101):
        i = 1 + k - 1
        if i >= 5:
            break
        expected[i] = 0.5 * (1.0 - math.cos(math.pi * k / 101))
    assert torch.allclose(scale, torch.from_numpy(expected),
                          atol=1e-12, rtol=0.0)


@pytest.mark.parametrize("t", [0.0, 0.5, 1.0, 1.05, 1.5, 2.0, 5.0])
def test_seg_blend_parity(t):
    from controllers.kinematics import N_BODY_JOINTS
    settle_time = 1.0
    ramp_time = 1.0
    n_seg = N_BODY_JOINTS
    indices = np.arange(1, N_BODY_JOINTS)
    if t < settle_time:
        blend_ref = np.zeros(len(indices), dtype=float)
    elif ramp_time <= 0.0:
        blend_ref = np.ones(len(indices), dtype=float)
    else:
        per_seg = ramp_time * 0.5
        stagger = per_seg / max(n_seg - 1, 1)
        starts = settle_time + indices * stagger
        elapsed = np.maximum(t - starts, 0.0)
        blend_ref = np.where(
            elapsed >= per_seg, 1.0,
            0.5 * (1.0 - np.cos(np.pi * elapsed / per_seg))
        )
        blend_ref[t <= starts] = 0.0
    blend_port = port.seg_blend(
        t=t, indices=torch.from_numpy(indices),
        settle_time=settle_time, ramp_time=ramp_time, n_seg=n_seg,
    )
    assert torch.allclose(blend_port, torch.from_numpy(blend_ref),
                          atol=1e-12, rtol=0.0)


# =============================================================================
# Section 2: integrated controller parity
# =============================================================================

def _make_paired_mocks(num_envs=1):
    """Return (art_ref, art_port) mocks with identical structure."""
    from tests.conftest import (
        MockArticulation, make_centipede_joint_names, make_centipede_body_names,
    )
    joint_names = make_centipede_joint_names(simplified=True)
    body_names = make_centipede_body_names()
    art_ref = MockArticulation(joint_names, body_names, num_envs=num_envs, device="cpu")
    art_port = MockArticulation(joint_names, body_names, num_envs=num_envs, device="cpu")
    return art_ref, art_port


def _set_joint_state(art, joint_pos, joint_vel):
    art.data.joint_pos[:] = torch.from_numpy(joint_pos).to(dtype=art.data.joint_pos.dtype)
    art.data.joint_vel[:] = torch.from_numpy(joint_vel).to(dtype=art.data.joint_vel.dtype)


def _set_head_quat_yaw(art, env_idx, yaw):
    half = float(yaw) * 0.5
    q = torch.tensor([math.cos(half), 0.0, 0.0, math.sin(half)],
                     dtype=art.data.body_quat_w.dtype)
    art.data.body_quat_w[env_idx, 0] = q


def test_heading_servo_parity(config_path, capsys):
    """With a known head yaw and head_yaw_gain>0, joint-0 torque must match."""
    art_ref, art_port = _make_paired_mocks(num_envs=1)
    n_joints = art_ref.num_joints
    pos = np.zeros((1, n_joints), dtype=np.float32)
    vel = np.zeros((1, n_joints), dtype=np.float32)
    _set_joint_state(art_ref, pos, vel)
    _set_joint_state(art_port, pos, vel)
    _set_head_quat_yaw(art_ref, 0, 0.3)
    _set_head_quat_yaw(art_port, 0, 0.3)

    capsys.readouterr()
    ctrl_ref = ref.ImpedanceTravelingWaveController(art_ref, config_path, env_idx=0)
    ctrl_port = port.ImpedanceTravelingWaveControllerTorch(
        art_port, config_path, num_envs=1, device="cpu", dtype=torch.float64,
    )
    capsys.readouterr()
    ctrl_ref.head_yaw_gain = 1.0
    ctrl_port.head_yaw_gain = 1.0

    dt = 5e-4
    ctrl_ref.step(0.0, dt)
    tau_ref_step1 = art_ref.last_effort_target.clone().squeeze(0)
    tau_port_step1 = ctrl_port.step(0.0, dt).squeeze(0)
    body_id_0 = int(ctrl_ref.idx.body_joint_ids[0])
    assert torch.allclose(
        tau_port_step1[body_id_0].to(torch.float32),
        tau_ref_step1[body_id_0],
        atol=1e-6, rtol=0.0,
    ), f"joint-0 torque mismatch: ref={tau_ref_step1[body_id_0]}, port={tau_port_step1[body_id_0]}"


def test_full_step_torque_parity(config_path, capsys):
    """Multi-step rollout: full torque output matches numpy reference per-step."""
    art_ref, art_port = _make_paired_mocks(num_envs=1)
    rng_local = np.random.default_rng(seed=123)
    n_joints = art_ref.num_joints
    pos = (rng_local.standard_normal((1, n_joints)).astype(np.float32) * 0.02)
    vel = (rng_local.standard_normal((1, n_joints)).astype(np.float32) * 0.02)
    _set_joint_state(art_ref, pos, vel)
    _set_joint_state(art_port, pos, vel)

    capsys.readouterr()
    ctrl_ref = ref.ImpedanceTravelingWaveController(art_ref, config_path, env_idx=0)
    ctrl_port = port.ImpedanceTravelingWaveControllerTorch(
        art_port, config_path, num_envs=1, device="cpu", dtype=torch.float64,
    )
    capsys.readouterr()

    # Compress settle/ramp so the 50-step rollout exercises the full gait
    # transition: settle (~10 steps) -> ramp -> active wave with CPG init.
    for ctrl in (ctrl_ref, ctrl_port):
        ctrl.settle_time = 0.005
        ctrl.ramp_time = 0.005

    dt = 5e-4
    n_steps = 50
    max_diff = 0.0
    max_tau = 0.0
    for step_idx in range(n_steps):
        t = step_idx * dt
        ctrl_ref.step(t, dt)
        tau_ref = art_ref.last_effort_target.squeeze(0)
        tau_port = ctrl_port.step(t, dt).squeeze(0)
        diff = (tau_port.to(torch.float32) - tau_ref).abs().max().item()
        max_diff = max(max_diff, diff)
        max_tau = max(max_tau, tau_ref.abs().max().item())
        assert diff < 1e-5, (
            f"Step {step_idx} (t={t:.4f}): "
            f"max diff = {diff}, ref_max = {tau_ref.abs().max().item()}"
        )
    # Non-triviality: at least one step must produce a nonzero torque,
    # otherwise the test is comparing 0 to 0 and proves nothing.
    assert max_tau > 1e-6, "Test inputs produced near-zero torque"


def test_step_n_envs_independence(config_path, capsys):
    """With num_envs=2 and different states, each env matches solo run."""
    from tests.conftest import (
        MockArticulation, make_centipede_joint_names, make_centipede_body_names,
    )
    art_a, art_b = _make_paired_mocks(num_envs=1)
    art_multi = MockArticulation(
        make_centipede_joint_names(simplified=True),
        make_centipede_body_names(),
        num_envs=2, device="cpu",
    )

    rng_local = np.random.default_rng(seed=7)
    n_joints = art_multi.num_joints
    pos_a = rng_local.standard_normal((1, n_joints)).astype(np.float32) * 0.02
    vel_a = rng_local.standard_normal((1, n_joints)).astype(np.float32) * 0.02
    pos_b = rng_local.standard_normal((1, n_joints)).astype(np.float32) * 0.02
    vel_b = rng_local.standard_normal((1, n_joints)).astype(np.float32) * 0.02

    _set_joint_state(art_a, pos_a, vel_a)
    _set_joint_state(art_b, pos_b, vel_b)
    art_multi.data.joint_pos[0] = torch.from_numpy(pos_a).squeeze(0)
    art_multi.data.joint_pos[1] = torch.from_numpy(pos_b).squeeze(0)
    art_multi.data.joint_vel[0] = torch.from_numpy(vel_a).squeeze(0)
    art_multi.data.joint_vel[1] = torch.from_numpy(vel_b).squeeze(0)

    capsys.readouterr()
    ctrl_a = port.ImpedanceTravelingWaveControllerTorch(
        art_a, config_path, num_envs=1, device="cpu", dtype=torch.float64,
    )
    ctrl_b = port.ImpedanceTravelingWaveControllerTorch(
        art_b, config_path, num_envs=1, device="cpu", dtype=torch.float64,
    )
    ctrl_multi = port.ImpedanceTravelingWaveControllerTorch(
        art_multi, config_path, num_envs=2, device="cpu", dtype=torch.float64,
    )
    capsys.readouterr()

    dt = 5e-4
    for step_idx in range(10):
        t = step_idx * dt
        tau_a = ctrl_a.step(t, dt).squeeze(0)
        tau_b = ctrl_b.step(t, dt).squeeze(0)
        tau_multi = ctrl_multi.step(t, dt)
        assert torch.allclose(tau_multi[0], tau_a, atol=1e-12, rtol=0.0), (
            f"Step {step_idx}: env 0 of multi-env diverged from solo"
        )
        assert torch.allclose(tau_multi[1], tau_b, atol=1e-12, rtol=0.0), (
            f"Step {step_idx}: env 1 of multi-env diverged from solo"
        )
