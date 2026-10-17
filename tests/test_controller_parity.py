"""
Parity tests: the torch module's STATELESS HELPERS must reproduce the numpy reference.

SCOPE NARROWED BY S2b (2026-07-16, PI-signed; DISPATCH_LOG 2026-07-16T19:34Z)
-----------------------------------------------------------------------------
This file was born as "the torch port reproduces the numpy reference's TORQUE".
That contract is VOID: S1 deleted the env's explicit torque path, S2a made an env-side
impedance table the only gain source, and S2b deleted `pd_torque`, every kp/kv, the
effort clamps and the `tau` return from controllers/impedance_controller_torch.py.
There is no torque left in the torch controller to compare.

DELETED with the feature (do not resurrect without resurrecting the torque path):
  * test_pd_torque_matches_formula   -- port.pd_torque no longer exists.
  * test_heading_servo_parity        -- compared joint-0 TORQUE against the numpy ref.
  * test_full_step_torque_parity     -- compared the full tau rollout against the numpy ref.
Note these could not run regardless: they build the numpy reference from the shared
config fixture, and S2b made configs/farms_controller.yaml gain-free (the numpy ref now
raises there by design and reads configs/farms_controller_numpy_legacy.yaml instead).

WHAT STILL HAS PARITY VALUE, and is still tested here:
  * the stateless helpers -- quat_to_yaw, kuramoto_body_step, kuramoto_leg_step,
    build_body_amp_scale, seg_blend. These are the CPG maths and they are untouched by S2b.
  * test_step_n_envs_independence -- re-expressed on the controller's live output (the
    published TARGETS) instead of torque. Still torch-vs-torch; no numpy oracle needed.

The controller's own S2b bit-identity evidence (targets unchanged vs the pre-S2b snapshot
over a 400-step rollout + partial reset) is separate; see the S2b report.

Run:
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


def test_leg_phase_anchor_uses_segment_midpoint():
    controller = object.__new__(port.ImpedanceTravelingWaveControllerTorch)
    controller.n_wave_scalar = 3.0
    controller.speed_scalar = 1.0
    controller.wave_number_scale_runtime = torch.tensor([0.5, 1.0, 4.0 / 3.0])
    body = torch.zeros((3, 19))
    anchor = controller._leg_phase_anchor(body)
    expected = 0.5 * controller._cpg_delta().expand_as(anchor)
    assert torch.allclose(anchor, expected)


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


# test_pd_torque_matches_formula: DELETED by S2b (2026-07-16). port.pd_torque is gone --
# the torch controller computes no torque; PhysX's implicit PD does, from the env's
# impedance table + this controller's targets. There is nothing left to compare.


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
# Section 2: integrated controller behaviour
#
# S2b (2026-07-16): the two TORQUE-parity tests that lived here
# (test_heading_servo_parity, test_full_step_torque_parity) are DELETED -- the torch
# controller returns None and holds no gains, so there is no torque to compare against
# the numpy reference. What remains is the multi-env consistency check, now asserted on
# the controller's actual output: the published position/velocity TARGETS.
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
        # S2b: step() returns None; the output is the published target stash.
        assert ctrl_a.step(t, dt) is None, "step() must return None since S2b"
        ctrl_b.step(t, dt)
        ctrl_multi.step(t, dt)
        for _nm in ("_pd_pos_target", "_pd_vel_target"):
            solo_a = getattr(ctrl_a, _nm).squeeze(0)
            solo_b = getattr(ctrl_b, _nm).squeeze(0)
            multi = getattr(ctrl_multi, _nm)
            assert torch.allclose(multi[0], solo_a, atol=1e-12, rtol=0.0), (
                f"Step {step_idx}: env 0 of multi-env diverged from solo ({_nm})"
            )
            assert torch.allclose(multi[1], solo_b, atol=1e-12, rtol=0.0), (
                f"Step {step_idx}: env 1 of multi-env diverged from solo ({_nm})"
        )
