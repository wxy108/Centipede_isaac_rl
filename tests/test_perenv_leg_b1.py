"""
NUMPY-reference leg-gain positive control.

HISTORY / WHY THE FILENAME IS STALE (S2b, 2026-07-16, PI-signed)
-----------------------------------------------------------------
This file was written for ⛳B1 (Stage A v3, 2026-06-17): the protected edit that added
`leg_imp_scale_runtime (num_envs, N_LEG_DOF)` to controllers/impedance_controller_torch.py.
S2b DELETED that feature -- along with every other gain and the torque itself -- from the
torch controller, which is now a TARGETS-ONLY CPG reference generator (gains live in the
env's impedance table -> PhysX). Three of this file's four tests died with it:

  * test_default_bit_identical_to_pre_edit_snapshot  -- compared TORQUE against the
    .before_perenv_leg_2026-06-17 snapshot. No torque exists to compare.
  * test_leg_imp_scale_positive_control_torch        -- drove ctrl.set_leg_imp_scale_runtime().
    Both the setter and the tensor are gone (AttributeError).
  * test_setter_shape_validation                     -- same setter.

WHAT SURVIVES (below): the NUMPY positive control. controllers/impedance_controller.py is
NOT part of S2b -- it still computes torque and is still used by the standalone optimizer
scripts (optimize_impedance_*.py, render_top_trials.py, test_walk.py, ...). This test closes
the GATE_B1 false alarm by proving a leg-gain change through the real apply_params path
(controller.leg_kp[0]=v; _leg_kp_flat=tile) actually moves numpy joint torque, and that the
change is confined to hip_yaw leg DOFs.

NOTE the config: S2b made configs/farms_controller.yaml GAIN-FREE (it configures the
targets-only torch controller). The numpy ref needs its gains, so it reads
configs/farms_controller_numpy_legacy.yaml -- byte-identical gains to pre-S2b. Pointing the
numpy ref at the gain-free file now RAISES by design (it used to silently default body_kp
to 5.0, a 35x error).

Run:
    python -m pytest tests/test_perenv_leg_b1.py -v
"""
from __future__ import annotations

import importlib.machinery
import importlib.util
from pathlib import Path

import numpy as np
import pytest
import torch

import controllers.impedance_controller as ref
import controllers.impedance_controller_torch as port
from controllers.kinematics import N_LEG_DOF, N_LEGS
from tests.conftest import (
    MockArticulation, make_centipede_joint_names, make_centipede_body_names,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = (PROJECT_ROOT / "controllers"
            / "impedance_controller_torch.py.before_perenv_leg_2026-06-17")
DT = 5e-4


def _mock(num_envs=1):
    return MockArticulation(
        make_centipede_joint_names(simplified=True),
        make_centipede_body_names(),
        num_envs=num_envs, device="cpu",
    )


def _set_state(art, pos, vel):
    art.data.joint_pos[:] = torch.as_tensor(pos, dtype=art.data.joint_pos.dtype)
    art.data.joint_vel[:] = torch.as_tensor(vel, dtype=art.data.joint_vel.dtype)


def _torch_ctrl(mod, art, config_path, num_envs):
    return mod.ImpedanceTravelingWaveControllerTorch(
        art, config_path, num_envs=num_envs, device="cpu", dtype=torch.float64,
    )


def _load_snapshot_module():
    assert SNAPSHOT.exists(), f"Missing pre-edit snapshot: {SNAPSHOT}"
    loader = importlib.machinery.SourceFileLoader("port_old_b1", str(SNAPSHOT))
    spec = importlib.util.spec_from_loader("port_old_b1", loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# 1. BIT-IDENTITY: edited @ default leg_imp_scale==1.0 == pre-edit snapshot
# ---------------------------------------------------------------------------
def test_numpy_leg_gain_drives_torque_positive_control():
    # S2b: NOT the `config_path` fixture (that is the gain-free torch config, which the
    # numpy controller now rejects by design). The numpy ref reads its own legacy config.
    config_path = str(PROJECT_ROOT / "configs" / "farms_controller_numpy_legacy.yaml")
    rng = np.random.default_rng(7)
    n_joints = _mock(1).num_joints
    pos = rng.standard_normal((1, n_joints)).astype(np.float32) * 0.05
    vel = rng.standard_normal((1, n_joints)).astype(np.float32) * 0.05

    def run(hip_yaw_kp):
        art = _mock(1)
        _set_state(art, pos, vel)
        c = ref.ImpedanceTravelingWaveController(art, config_path, env_idx=0)
        c.settle_time = 0.005
        c.ramp_time = 0.005
        # Mimic optimize_impedance_biological.apply_params (:1206, :1216):
        # set hip_yaw_kp on the numpy leg_kp vector, then REBUILD _leg_kp_flat.
        c.leg_kp[0] = float(hip_yaw_kp)
        c._leg_kp_flat = np.tile(c.leg_kp, N_LEGS * 2)
        last = None
        for k in range(30):
            c.step(k * DT, DT)
            last = art.last_effort_target.clone().squeeze(0).cpu().numpy().copy()
        return last, c

    a, ca = run(0.05)
    b, _ = run(0.50)  # 10x hip_yaw stiffness, all else identical

    leg_ids = np.asarray(ca._leg_flat_ids)
    dof = np.tile(np.arange(N_LEG_DOF), N_LEGS * 2)
    hipyaw_ids = leg_ids[dof == 0]

    assert not np.allclose(a, b), (
        "hip_yaw_kp change did NOT move joint torque -> leg search would be inert")
    diff_idx = set(np.where(~np.isclose(a, b, atol=1e-9))[0].tolist())
    assert diff_idx.issubset(set(leg_ids.tolist())), "torque change leaked outside legs"
    assert diff_idx.issubset(set(hipyaw_ids.tolist())), "only hip_yaw DOFs should change"
    assert np.abs(a[hipyaw_ids]).max() > 1e-6, "hip_yaw torque ~0 (vacuous)"
