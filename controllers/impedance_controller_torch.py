"""
impedance_controller_torch.py - the CPG reference generator (torch-vectorized).
====================================================================================

WHAT THIS MODULE IS (since S2b, 2026-07-16 -- PI-signed, DISPATCH_LOG 2026-07-16T19:34Z)
---------------------------------------------------------------------------------------
A REFERENCE GENERATOR and NOTHING ELSE. It owns the Kuramoto CPG state and turns it
into per-joint TARGETS. It computes no gain, no torque, and no force.

    step(t, dt) -> None      # publishes self._pd_pos_target / self._pd_vel_target

  * GAINS live in the env's impedance TABLE (env.set_joint_impedance -> PhysX
    stiffness/damping). S2a made that the single source; PI: "一个关节只留一组kp kv
    作为PD gains，其他全去掉".
  * FORCE is computed by PhysX's implicit PD from (target, Kp, Kv). S1 deleted the
    explicit torque path from the env.
  * S2b therefore DELETED, from this file: `pd_torque`, every kp/kv vector and its
    YAML read, the 10 *_scale_runtime GAIN scales and their setters, the effort
    clamps, the `tau` tensor, and the `_pd_kp`/`_pd_kv` stash. They computed numbers
    that nothing consumed. PI: "已经删除的就删彻底，防止误导后面人".

CONSEQUENCE worth knowing: this controller now reads NO joint state. Only the head's
body_quat_w (the heading servo) is read; joint_pos/joint_vel were feedback for the
deleted PD law. Feedback closes in PhysX, not here. The CPG is open-loop in q.

DESIGN PRINCIPLES
-----------------
1.  All internal state has an explicit leading num_envs dimension.
2.  step() is PURE COMPUTE + CPG state advance. It does NOT write to the sim;
    that is the env's job (env._apply_action_implicit_pd).
3.  The CPG is STATEFUL: step() integrates the Kuramoto phase every call and must
    be called every physics step even though its targets are consumed per-step.

NOT the numpy reference's port any more. controllers/impedance_controller.py still
computes torque and is still used by the standalone optimizer scripts, but the
torque-parity relationship this file was born with is VOID -- there is no torque
here to compare. Only the stateless helpers (quat_to_yaw / seg_blend / kuramoto_*)
remain parity-comparable.

CONTENTS
  [x]  Helper: quat_to_yaw           (stateless)
  [x]  Helper: kuramoto_body_step    (per-env-broadcast aware)
  [x]  Helper: kuramoto_leg_step     (per-env-broadcast aware)
  [x]  Helper: build_body_amp_scale  (head-end cosine taper)
  [x]  Helper: seg_blend             (sequential per-segment blend)
  [x]  Class: __init__               (target/CPG parameter loading, state allocation)
  [x]  Class: heading servo          (target)
  [x]  Class: body wave / pitch / legs -> targets
  [x]  Class: step()                 (advances CPG, publishes _pd_pos/_pd_vel_target)
  [x]  Class: reset_target_history(), set_frequency()
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import torch
import yaml

from controllers.kinematics import (
    IsaacLabModelIndex,
    N_BODY_JOINTS, N_LEGS, N_LEG_DOF,
)


# =============================================================================
# Stateless helpers (parity-tested)
# =============================================================================

def quat_to_yaw(quat_wxyz: torch.Tensor) -> torch.Tensor:
    """Extract yaw (rotation about world-Z) from (w, x, y, z) quaternion(s)."""
    if quat_wxyz.shape[-1] != 4:
        raise ValueError(f"Expected last dim 4 (w,x,y,z), got {quat_wxyz.shape}")
    w = quat_wxyz[..., 0]
    x = quat_wxyz[..., 1]
    y = quat_wxyz[..., 2]
    z = quat_wxyz[..., 3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return torch.atan2(siny_cosp, cosy_cosp)


def _to_phi_tensor(x, phi: torch.Tensor) -> torch.Tensor:
    if torch.is_tensor(x):
        return x.to(dtype=phi.dtype, device=phi.device)
    return torch.as_tensor(x, dtype=phi.dtype, device=phi.device)


def kuramoto_body_step(phi, omega, coupling_gain, delta, dt):
    """Forward-Euler step of the Kuramoto chain for body yaw phases.

    phi: (num_envs, N). Returns phi_new of same shape.

    omega and coupling_gain may each be a Python scalar, a 0-dim tensor,
    or a 1-dim (num_envs,) tensor. Per-env (1-dim) inputs are broadcast
    against phi's trailing N dim via an explicit unsqueeze.
    """
    if phi.dim() != 2:
        raise ValueError(f"Expected phi shape (num_envs, N), got {phi.shape}")
    num_envs, N = phi.shape
    if N < 1:
        raise ValueError("N must be >= 1")
    omega_t = _to_phi_tensor(omega, phi)
    gain_t = _to_phi_tensor(coupling_gain, phi)

    # Build phi_dot of shape (num_envs, N).
    if omega_t.dim() == 1:
        phi_dot = omega_t.unsqueeze(-1).expand(num_envs, N).clone()
    else:
        phi_dot = phi.new_full(phi.shape, omega_t.item())

    if N >= 2:
        if gain_t.dim() == 1:
            gain_2d = gain_t.unsqueeze(-1)  # (num_envs, 1)
        else:
            gain_2d = gain_t  # scalar 0-dim
        fwd = torch.sin(phi[:, :-1] - (phi[:, 1:] + delta))
        bwd = torch.sin(phi[:, 1:] - (phi[:, :-1] - delta))
        phi_dot[:, 0] = phi_dot[:, 0] + gain_t * fwd[:, 0]
        if N >= 3:
            phi_dot[:, 1:-1] = phi_dot[:, 1:-1] + gain_2d * (bwd[:, :-1] + fwd[:, 1:])
        phi_dot[:, -1] = phi_dot[:, -1] + gain_t * bwd[:, -1]
    return phi + dt * phi_dot


def kuramoto_leg_step(phi_leg, phi_body_subset, omega, coupling_gain_leg, dt):
    """Forward-Euler step for leg Kuramoto coupling.

    omega and coupling_gain_leg can be scalar or (num_envs,); the latter
    are broadcast across leg index via an unsqueeze.
    """
    if phi_leg.shape != phi_body_subset.shape:
        raise ValueError(
            f"phi_leg {phi_leg.shape} and phi_body_subset {phi_body_subset.shape} must match"
        )
    omega_t = _to_phi_tensor(omega, phi_leg)
    gain_t = _to_phi_tensor(coupling_gain_leg, phi_leg)
    if omega_t.dim() == 1:
        omega_t = omega_t.unsqueeze(-1)
    if gain_t.dim() == 1:
        gain_t = gain_t.unsqueeze(-1)
    fdot = omega_t + gain_t * torch.sin(phi_leg - phi_body_subset)
    return phi_leg + dt * fdot


def build_body_amp_scale(n, head_hold_joints, head_ramp_joints,
                         head_ramp_min=0.0, dtype=torch.float64, device=None):
    """Cosine head-end amplitude taper, returns (n,) tensor."""
    scale = torch.ones(n, dtype=dtype, device=device)
    H = max(int(head_hold_joints), 0)
    R = max(int(head_ramp_joints), 0)
    amin = float(head_ramp_min)
    if H > 0:
        scale[: min(H, n)] = 0.0
    if R > 0:
        ks = torch.arange(1, R + 1, dtype=dtype, device=device)
        ramp_vals = amin + (1.0 - amin) * 0.5 * (
            1.0 - torch.cos(math.pi * ks / (R + 1))
        )
        idxs_start = H
        idxs_end = min(H + R, n)
        n_to_write = idxs_end - idxs_start
        if n_to_write > 0:
            scale[idxs_start:idxs_end] = ramp_vals[:n_to_write]
    return scale


def seg_blend(t, indices, settle_time, ramp_time, n_seg):
    """Sequential per-segment cosine blend factor for each i in indices.

    t can be:
      - Python float / 0-dim tensor / scalar  -> returns (M,) blend tensor.
      - 1-dim tensor of shape (num_envs,)     -> returns (num_envs, M).

    Stage B addition (2026-05-07): per-env time support. Equations are
    unchanged; only the broadcasting is lifted so that envs at different
    elapsed-since-reset times each see their own correct blend.
    """
    if not torch.is_tensor(indices):
        indices = torch.as_tensor(indices)
    out_dtype = torch.float64
    out_device = indices.device

    # Detect per-env tensor mode.
    per_env = (
        torch.is_tensor(t) and t.dim() == 1 and t.numel() > 0
    )

    if per_env:
        n_idx = int(indices.shape[0])
        n_env = int(t.shape[0])
        t_b = t.to(dtype=out_dtype, device=out_device).unsqueeze(1)  # (E, 1)
        if ramp_time <= 0.0:
            return (t_b >= settle_time).to(dtype=out_dtype).expand(n_env, n_idx).clone()
        per_seg = ramp_time * 0.5
        stagger = per_seg / max(n_seg - 1, 1)
        indices_f = indices.to(dtype=out_dtype)
        starts = (settle_time + indices_f * stagger).unsqueeze(0)   # (1, M)
        elapsed = torch.clamp(t_b - starts, min=0.0)                # (E, M)
        cos_blend = 0.5 * (1.0 - torch.cos(math.pi * elapsed / per_seg))
        blend = torch.where(elapsed >= per_seg, torch.ones_like(elapsed), cos_blend)
        blend = torch.where(t_b <= starts, torch.zeros_like(blend), blend)
        return blend

    # ── scalar-t path (existing behaviour, byte-identical) ──
    n_idx = indices.shape
    if t < settle_time:
        return torch.zeros(n_idx, dtype=out_dtype, device=out_device)
    if ramp_time <= 0.0:
        return torch.ones(n_idx, dtype=out_dtype, device=out_device)
    per_seg = ramp_time * 0.5
    stagger = per_seg / max(n_seg - 1, 1)
    indices_f = indices.to(dtype=out_dtype)
    starts = settle_time + indices_f * stagger
    elapsed = torch.clamp(t - starts, min=0.0)
    cos_blend = 0.5 * (1.0 - torch.cos(math.pi * elapsed / per_seg))
    blend = torch.where(elapsed >= per_seg, torch.ones_like(elapsed), cos_blend)
    blend = torch.where(t <= starts, torch.zeros_like(blend), blend)
    return blend


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# =============================================================================
# Full controller class
# =============================================================================


class ImpedanceTravelingWaveControllerTorch:
    """Torch-vectorized Kuramoto-CPG REFERENCE GENERATOR for Isaac Lab.

    Targets only -- no gains, no torque (S2b 2026-07-16). See module docstring.

    Public interface:
        step(t, dt) -> None            # sets self._pd_pos_target / self._pd_vel_target
        reset_envs(env_ids, t_at_reset) -> None
        reset_target_history() -> None
        set_frequency(freq) -> None
    """

    # -- The YAML block this controller reads ------------------------------------------
    # RENAMED 2026-07-16 from `impedance:` (PI ruling: "改名impedance变成implicit内置的求解器了").
    # The old name was historical and misleading: the block carries NO impedance any more, only
    # the TARGETS this controller emits + the settle/ramp clock. The impedance moved to
    # env.set_joint_impedance({family: (Kp, Kv)}) -> PhysX stiffness/damping, and the "solver"
    # IS the PhysX built-in implicit PD. Legal keys = _TARGETS_YAML_KEYS below.
    _TARGETS_YAML_BLOCK = "reference_targets"
    _TARGETS_YAML_BLOCK_OLD = "impedance"
    _TARGETS_YAML_KEYS = (
        "head_pitch_offset", "head_pitch_joints", "settle_time", "ramp_time",
        "head_yaw_gain", "head_yaw_target_clip", "head_yaw_rate",
    )

    # S2b (2026-07-16): gain keys that fed the DELETED gain chain. A config that still carries one
    # is a config whose author believes it sets a gain -- it does not, and silently ignoring it is
    # exactly the failure this project forbids (same doctrine as the S1/S2a guards).
    _S2B_DEAD_YAML_KEYS = (
        "body_kp", "body_kv", "body_kp_head", "body_kv_head",
        "pitch_kp", "pitch_kv", "roll_kp", "roll_kv",
        "head_pitch_kp", "head_pitch_kv", "leg",
    )

    @classmethod
    def _validate_targets_yaml(cls, cfg, config_path):
        """FAIL LOUDLY on a config written for the pre-2026-07-16 controller.

        Runs BEFORE the articulation is touched, so a bad config costs nothing and can be checked
        without booting Isaac. Two failure modes, both of which would otherwise be SILENT:

          (a) OLD BLOCK NAME `impedance:` -- cfg.get() would return {} and every target key would
              take its DEFAULT: head_pitch_offset 0.0 instead of -0.25 (the head-up hold VANISHES),
              head_yaw_gain 1.0 instead of 0.0 (a heading servo meant to be OFF switches ON),
              settle/ramp 1.0. The run would look fine and be wrong.
          (b) A GAIN KEY inside the block -- nothing reads it any more, so the stiffness its author
              believes they set would simply not happen.

        Same doctrine as the S1/S2a env guards and the split of configs/farms_controller.yaml.
        """
        if cls._TARGETS_YAML_BLOCK_OLD in cfg:
            raise ValueError(
                "controller YAML %r has an `impedance:` block. That block was RENAMED to "
                "`reference_targets:` on 2026-07-16 (PI-signed; DISPATCH_LOG 2026-07-16T19:34Z -> "
                "22:05Z), because the name had become false: it carries NO impedance.\n"
                "This controller is the SOLE reference generator -- it emits position/velocity "
                "TARGETS only, reads no joint state and computes no torque. Impedance is now set in "
                "PHYSICAL units via env.set_joint_impedance({family: (Kp, Kv)}) (families = "
                "env.s2_family_ids()), which writes PhysX stiffness/damping directly, and all "
                "feedback closes inside the PhysX implicit PD solver. Torque caps: "
                "env.set_joint_effort_limit({family: tau_max_Nm|None}).\n"
                "  TO FIX -> rename the block to `reference_targets:`; the keys are unchanged "
                "(%s). Reference copy: configs/farms_controller.yaml.\n"
                "  Full story -> docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md ; "
                "outputs/CURRENT_RESULT.md ; DISPATCH_LOG.md 2026-07-16T19:34Z -> 22:05Z.\n"
                "  NOTE -> configs/farms_controller_numpy_legacy.yaml KEEPS its `impedance:` block "
                "on purpose: its consumer (controllers/impedance_controller.py, the numpy "
                "reference) still computes torque from real gains, so there the name is still "
                "accurate. If you passed THAT file here, you want the numpy controller, not this "
                "one." % (config_path, ", ".join(cls._TARGETS_YAML_KEYS))
            )
        imp = cfg.get(cls._TARGETS_YAML_BLOCK, {})
        _dead = [k for k in cls._S2B_DEAD_YAML_KEYS if k in imp]
        if _dead:
            raise ValueError(
                "controller YAML %r carries gain key(s) %s in its `reference_targets:` block, which "
                "S2b DELETED (2026-07-16, PI-signed; DISPATCH_LOG 2026-07-16T19:34Z).\n"
                "This controller computes TARGETS ONLY -- it holds no kp/kv and no torque. Gains "
                "are set in PHYSICAL units via env.set_joint_impedance({family: (Kp, Kv)}) "
                "(families = env.s2_family_ids()), which writes PhysX stiffness/damping directly, "
                "and the PhysX implicit PD solver closes the loop.\n"
                "  TO FIX -> delete these keys. The legal keys are TARGET/timing only: %s.\n"
                "  If you meant to drive the NUMPY reference controller "
                "(controllers/impedance_controller.py), it still needs these gains -- use "
                "configs/farms_controller_numpy_legacy.yaml and that class, not this one."
                % (config_path, _dead, ", ".join(cls._TARGETS_YAML_KEYS))
            )
        return imp

    def __init__(
        self,
        articulation,
        config_path,
        num_envs=1,
        device=None,
        dtype=torch.float64,
    ):
        cfg = _load_yaml(config_path)
        # Config validation FIRST -- before the articulation, the index resolution or any tensor.
        # Catches a pre-2026-07-16 config (old `impedance:` block name, or a gain key inside the
        # renamed block) loudly and for free. See _validate_targets_yaml.
        self._validate_targets_yaml(cfg, config_path)
        self.articulation = articulation
        self.num_envs = int(num_envs)
        self.device = device or getattr(articulation, "device", "cpu")
        self.dtype = dtype
        self.config_path = config_path

        # ── Index resolution ──
        self.idx = IsaacLabModelIndex(articulation)
        self.num_joints = articulation.num_joints
        # Block A (2026-07-07): PER-INSTANCE leg-DOF count from the articulation
        # (4 for deployed/renamed-4DOF, 2 for the leaner rebuild). Replaces the
        # module-level N_LEG_DOF in every leg tensor shape, tiling stride, and
        # shape check below. For a 4-DOF model n_leg_dof == 4 == the old constant
        # ⇒ every leg array is byte-identical to pre-Block-A.
        self.n_leg_dof = int(self.idx.n_leg_dof)

        def _to_long(arr):
            return torch.as_tensor(arr, dtype=torch.long, device=self.device)

        self.body_joint_ids = _to_long(self.idx.body_joint_ids)
        self.pitch_joint_ids = _to_long(self.idx.pitch_joint_ids)
        self.roll_joint_ids = _to_long(self.idx.roll_joint_ids)
        self.leg_flat_ids = _to_long(self.idx.leg_joint_ids.reshape(-1))
        self.head_body_id = int(self.idx.head_body_id)
        self.has_pitch = bool(self.idx.has_pitch)
        self.has_roll = bool(self.idx.has_roll)
        self.disabled_joint_indices = []

        # ── Body wave params (per-env tensors so the RL env can modulate) ──
        bw = cfg["body_wave"]
        self.body_amp = self._param_per_env(float(bw["amplitude"]))
        self.freq = self._param_per_env(float(bw["frequency"]))
        self.n_wave_scalar = float(bw["wave_number"])
        self.speed_scalar = float(bw["speed"])

        self.head_hold_joints = int(bw.get("head_hold_joints", 0))
        self.head_ramp_joints = int(bw.get("head_ramp_joints", 2))
        self.head_ramp_min = float(bw.get("head_ramp_min", 0.0))

        self.body_amp_scale = build_body_amp_scale(
            N_BODY_JOINTS,
            self.head_hold_joints,
            self.head_ramp_joints,
            self.head_ramp_min,
            dtype=dtype,
            device=self.device,
        )

        # ── CPG ──
        cpg_cfg = cfg.get("cpg", {})
        self.use_cpg = bool(cpg_cfg.get("enabled", True))
        self.cpg_omega = float(cpg_cfg.get("coupling_omega", -2.5))
        self.cpg_leg_omega = float(cpg_cfg.get("leg_coupling_omega", self.cpg_omega))

        self.spatial_phases = torch.tensor(
            [2.0 * math.pi * self.n_wave_scalar * self.speed_scalar * i
             / max(N_BODY_JOINTS - 1, 1)
             for i in range(N_BODY_JOINTS)],
            dtype=dtype, device=self.device,
        )

        self.body_phases = None
        self.leg_phases = None
        self._cpg_initialized = False

        # ── Stage B Step 1 (2026-05-07): per-env RL action modulation ──
        # All buffers default to 1.0 so a zero RL action is byte-identical
        # to the pre-Stage-B-Step-1 baseline:
        #   body_amp_scale_runtime[e, j] multiplies the body-wave amplitude
        #     for env e at body joint j (anatomic: j=0 -> joint_body_1,
        #     j=1 -> joint_body_2, ...; j=0 is the head-yaw joint and is
        #     stored-but-unused since the body wave is only applied to
        #     joints 1..N-1, see step() body block).
        #   wave_number_scale_runtime[e] multiplies n_wave_scalar to give
        #     the effective wave number used in CPG coupling delta and
        #     spatial-phase initialisation for env e.
        #
        # S2b (2026-07-16): the GAIN scales that used to live alongside these
        # (body_kp/body_kv/pitch_imp/pitch_kp/pitch_kv/roll_kp/roll_kv/leg_imp/
        # leg_kp/leg_kv _scale_runtime -- the env's _S2_DEAD_SCALES list) are
        # DELETED. Only TARGET-shaping scales (amp / wave-number / frequency)
        # survive, because only targets leave this module.
        self.body_amp_scale_runtime = torch.ones(
            (self.num_envs, N_BODY_JOINTS), dtype=dtype, device=self.device
        )
        self.wave_number_scale_runtime = torch.ones(
            (self.num_envs,), dtype=dtype, device=self.device
        )

        # ── Stage B-v0 (2026-05-07): global_6d action mode runtime fields ──
        # Default 1.0, so a zero RL action reduces every equation to baseline
        # byte-for-byte.
        #   leg_yaw_amp_scale_runtime[e]  multiplies leg DOF-0 amplitude.
        #   leg_pitch_amp_scale_runtime[e] multiplies leg DOF-1 amplitude.
        #   freq_scale_runtime[e]      multiplies self.freq inside _omega().
        self.leg_yaw_amp_scale_runtime = torch.ones(
            (self.num_envs,), dtype=dtype, device=self.device
        )
        self.leg_pitch_amp_scale_runtime = torch.ones(
            (self.num_envs,), dtype=dtype, device=self.device
        )
        self.freq_scale_runtime = torch.ones(
            (self.num_envs,), dtype=dtype, device=self.device
        )
        # ── 2026-05-13: fast wave-number retargeting state ──
        # Phase-state retargeting (Option B in
        # outputs/analysis/cpg_fast_wavenumber_retargeting_design.md).
        # When enabled, _apply_phase_rebase shifts body_phases each
        # step so the chain tracks wave_number_scale_runtime within a
        # rate-limited per-step delta. Default OFF — when disabled,
        # _apply_phase_rebase is never called and the controller path
        # is identical to the pre-2026-05-13 Pass-8 / Stage-E baseline.
        # The audit counter must read 0 across any flag-OFF rollout.
        self._fast_wave_retargeting_enabled = False
        self._fast_wave_retargeting_max_rate_per_s = 1.0
        # Tracks the wave_number_scale_runtime value that body_phases
        # is currently calibrated to. Lazily allocated on the first
        # call to _apply_phase_rebase (so the very first rebase step
        # produces a zero shift) and reset in reset_envs.
        self._last_rebase_wave_scale = None
        self._rebase_call_count = 0

        # ── 2026-05-13: FARMS-equivalent CPG optimisation overrides ──
        # All four values are identity-by-default: behaviour is
        # bit-identical to the pre-override baseline when
        # body_coupling_scale = leg_coupling_scale = 1.0,
        # substeps = 1, update_mode = "jacobi".
        # See outputs/analysis/farms_equivalent_cpg_optimization/.
        self._cpg_body_coupling_scale = 1.0
        self._cpg_leg_coupling_scale = 1.0
        self._cpg_substeps = 1
        self._cpg_update_mode = "jacobi"
        self._cpg_integrate_call_count = 0

        # ── Targets + timing (the `reference_targets:` YAML block) ──
        # RENAMED 2026-07-16 from `impedance:` -- see _validate_targets_yaml, which already ran
        # (top of __init__) and has rejected the old name and any leftover gain key. Nothing in
        # this block is a gain: it configures the head-up hold, the heading servo, and the clock
        # that fades the emitted targets in. Impedance = env.set_joint_impedance() -> PhysX.
        imp = cfg.get(self._TARGETS_YAML_BLOCK, {})

        self.settle_time = float(imp.get("settle_time", 1.0))
        self.ramp_time = float(imp.get("ramp_time", 1.0))

        # head_pitch_offset is a TARGET (the head-up hold), NOT a gain — it survives S2b.
        # head_pitch_joints is load-bearing beyond this file: it defines the head/body_pitch
        # family split in env.s2_family_ids().
        self.head_pitch_offset = float(imp.get("head_pitch_offset", 0.0))
        self.head_pitch_joints = int(imp.get("head_pitch_joints", 1))

        if self.has_pitch:
            # pitch_target_final[0] = head_pitch_offset (the head's static hold);
            # pitch_target_final[1:] == 0 — the 19 spine-pitch joints have NO CPG signal
            # (PI: "脊柱pitch本身根本没有cpg信号，只有body yaw和leg base pitch和yaw才有cpg信号").
            n_p = len(self.pitch_joint_ids)
            self.pitch_target_final = torch.zeros(n_p, dtype=dtype, device=self.device)
            for k in range(min(self.head_pitch_joints, n_p)):
                self.pitch_target_final[k] = self.head_pitch_offset

        # ── Heading servo ──
        self.head_yaw_gain = float(imp.get("head_yaw_gain", 1.0))
        self.head_yaw_target_clip = float(imp.get("head_yaw_target_clip", 0.5))
        self.head_yaw_rate = float(imp.get("head_yaw_rate", 0.0))
        self.head_yaw_ref = None

        # ── Leg wave params ──
        lw = cfg["leg_wave"]
        leg_amps = np.array(lw["amplitudes"], dtype=float)
        leg_phase_offsets = np.array(lw["phase_offsets"], dtype=float)
        leg_dc_offsets = np.array(lw["dc_offsets"], dtype=float)
        active_dofs_set = set(lw["active_dofs"])

        # Block A (2026-07-07): SLICE the per-DOF config leg arrays to the
        # per-instance leg-DOF count. configs/farms_controller.yaml stays length-4
        # (UNCHANGED); the 4-DOF models take [:4] (identity ⇒ byte-identical), the
        # leaner rebuild takes [:2] = (legbase_yaw, legbase_pitch). active_dofs_set
        # is (0, 1) ⊂ both, so no ACTIVE DOF is dropped by the slice.
        _nl = self.n_leg_dof
        leg_amps = leg_amps[:_nl]
        leg_phase_offsets = leg_phase_offsets[:_nl]
        leg_dc_offsets = leg_dc_offsets[:_nl]

        signs_np = np.array([1.0, -1.0])
        leg_signs_np = np.tile(np.repeat(signs_np, self.n_leg_dof), N_LEGS)
        leg_dc_flat_np = np.tile(leg_dc_offsets, N_LEGS * 2)
        leg_active_flat_np = np.tile(
            np.array([d in active_dofs_set for d in range(self.n_leg_dof)], dtype=bool),
            N_LEGS * 2,
        )
        leg_phase_off_flat_np = np.tile(leg_phase_offsets, N_LEGS * 2)
        leg_amp_flat_np = np.tile(leg_amps, N_LEGS * 2)
        leg_seg_idx_np = np.repeat(np.arange(N_LEGS), 2 * self.n_leg_dof)
        # Stage B-v0 (2026-05-07): per-index DOF id (0..N_LEG_DOF-1).
        # Used by step() to apply per-DOF leg amplitude scaling
        # (leg_yaw_amp_scale_runtime for DOF=0, leg_pitch_amp_scale_runtime
        # for DOF=1) in the global_6d action mode.
        leg_dof_idx_flat_np = np.tile(np.arange(self.n_leg_dof), N_LEGS * 2)

        def _to_dt(arr):
            return torch.as_tensor(arr, dtype=dtype, device=self.device)

        self.leg_signs = _to_dt(leg_signs_np)
        self.leg_dc_flat = _to_dt(leg_dc_flat_np)
        self.leg_active_flat = torch.as_tensor(leg_active_flat_np, dtype=torch.bool, device=self.device)
        self.leg_phase_off_flat = _to_dt(leg_phase_off_flat_np)
        self.leg_amp_flat = _to_dt(leg_amp_flat_np)
        self.leg_seg_idx = torch.as_tensor(leg_seg_idx_np, dtype=torch.long, device=self.device)
        self.leg_dof_idx_flat = torch.as_tensor(leg_dof_idx_flat_np, dtype=torch.long, device=self.device)
        self.leg_amp_ref = float(leg_amps[0]) if len(leg_amps) > 0 else 1.0

        self.prev_body_yaw_targets = None
        self.prev_pitch_targets = None
        self.prev_leg_targets_flat = None

    # ------------------------------------------------------------------
    def _param_per_env(self, value):
        if torch.is_tensor(value):
            t = value.to(dtype=self.dtype, device=self.device)
            if t.dim() == 0:
                return t.expand(self.num_envs).clone()
            if t.shape == (self.num_envs,):
                return t.clone()
            raise ValueError(f"Param tensor shape {t.shape} != ({self.num_envs},)")
        return torch.full((self.num_envs,), float(value),
                          dtype=self.dtype, device=self.device)

    def _omega(self):
        # Stage B-v0 (2026-05-07): per-env frequency scale.
        # freq_scale_runtime defaults to 1.0, so by IEEE 754
        # float multiplication this is bit-equivalent to
        # 2π * self.freq (the parity baseline) at default.
        return 2.0 * math.pi * self.freq * self.freq_scale_runtime

    def _cpg_delta(self):
        """CPG coupling phase offset.

        Stage B Step 1 (2026-05-07): now per-env via wave_number_scale_runtime.
        Returns shape (num_envs, 1). When wave_number_scale_runtime[e] = 1.0
        (default / zero RL action) the value is bit-equivalent to the
        pre-Stage-B-Step-1 Python scalar `2π * n_wave * speed / (N-1)`,
        because IEEE 754 multiplication by 1.0 is exact. kuramoto_body_step
        broadcasts (E, 1) cleanly against phi[:, 1:] (E, N-1).
        """
        base = (2.0 * math.pi * self.n_wave_scalar * self.speed_scalar
                / max(N_BODY_JOINTS - 1, 1))
        return base * self.wave_number_scale_runtime.unsqueeze(1)

    def _spatial_phases_per_env(self):
        """Per-env spatial phase template used by CPG initialisation,
        partial reset, and the non-CPG fallback in step().

        Stage B Step 1 (2026-05-07): per-env via wave_number_scale_runtime.
        Returns shape (num_envs, N_BODY_JOINTS). When the runtime scale is
        1.0, the result is bit-equivalent to the legacy
        `self.spatial_phases.unsqueeze(0)` value (1.0 multiply is exact).
        """
        return (self.wave_number_scale_runtime.unsqueeze(1)
                * self.spatial_phases.unsqueeze(0))

    def _leg_phase_anchor(self, body_phases):
        """Sample leg roots halfway between adjacent body-yaw phase samples."""
        return body_phases[:, :N_LEGS] + 0.5 * self._cpg_delta()

    def _assert_dead_gain_scale(self, name, value):
        """S2b: a GAIN-modulation channel that S2b deleted. The caller signature is kept so
        every existing action-mapping call site still type-checks, but a NON-IDENTITY value
        now RAISES instead of being written to a tensor nothing reads.

        Why raise rather than ignore: this channel used to reach physics (6D action ch-4
        body_imp_scale -> body_k{p,v}_scale_runtime -> the PD law). The gain source is now
        the env's impedance TABLE, so silently accepting the scale would drop the caller's
        modulation with no diagnostic — the exact silent-physics-change the S1/S2a guards
        exist to prevent. (This supersedes the env's per-substep _S2_DEAD_SCALES check for
        this path: it fires once per policy step instead of 10 CUDA syncs per substep.)
        """
        if value is None:
            return
        if not bool(torch.all(torch.as_tensor(value) == 1.0)):
            raise ValueError(
                "%s is a GAIN scale, which S2b DELETED (2026-07-16, PI-signed): this controller "
                "produces TARGETS ONLY and holds no kp/kv, so a value != 1.0 would silently have "
                "NO effect on physics.\n"
                "To modulate impedance, modulate the TABLE: env.set_joint_impedance({family: (Kp, Kv)}) "
                "in physical units. To keep this action channel inert, pass 1.0 (the campaign pins it "
                "via --design-a-fixed-imp-freq / --design-c-fix-all-non-wave)." % name
            )

    # ------------------------------------------------------------------
    # Stage B Step 1: RL action -> controller modulation
    # ------------------------------------------------------------------
    def apply_action_modulation(
        self,
        body_amp_scale: torch.Tensor,
        wave_number_scale: torch.Tensor,
        body_kp_scale: torch.Tensor,
    ) -> None:
        """Write RL-mapped scales into the runtime modulation buffers.

        Stage B Step 1 (2026-05-07). The caller is responsible for clamping
        / bounding values to safe ranges; this method does NO clipping. It
        only casts dtype / device and copies in-place.

        Args:
            body_amp_scale: shape (num_envs, N_BODY_JOINTS). Per-env per-joint
                multiplier on the body-wave amplitude. Index j corresponds
                anatomically to joint_body_(j+1); index 0 is stored but
                unused (head-yaw joint has heading servo, not wave amp).
            wave_number_scale: shape (num_envs,). Per-env multiplier on the
                effective n_wave (used in CPG coupling delta and spatial
                phase template).
            body_kp_scale: shape (num_envs,). DEAD since S2b (it modulated a gain).
                MUST be 1.0 — any other value RAISES rather than being silently
                dropped. Kept in the signature so the legacy body_21d action
                mapping (env._apply_body_21d_action, action ch-20) still calls
                cleanly and still fails loudly if a policy actually uses ch-20.
        """
        if body_amp_scale.shape != (self.num_envs, N_BODY_JOINTS):
            raise ValueError(
                f"body_amp_scale shape {tuple(body_amp_scale.shape)} != "
                f"({self.num_envs}, {N_BODY_JOINTS})"
            )
        if wave_number_scale.shape != (self.num_envs,):
            raise ValueError(
                f"wave_number_scale shape {tuple(wave_number_scale.shape)} != "
                f"({self.num_envs},)"
            )
        if body_kp_scale.shape != (self.num_envs,):
            raise ValueError(
                f"body_kp_scale shape {tuple(body_kp_scale.shape)} != "
                f"({self.num_envs},)"
            )
        self._assert_dead_gain_scale("apply_action_modulation(body_kp_scale=)", body_kp_scale)
        self.body_amp_scale_runtime.copy_(
            body_amp_scale.to(dtype=self.dtype, device=self.device)
        )
        self.wave_number_scale_runtime.copy_(
            wave_number_scale.to(dtype=self.dtype, device=self.device)
        )

    # ------------------------------------------------------------------
    # Stage B-v0 (2026-05-07): global_6d action mode
    # ------------------------------------------------------------------
    def apply_global_6d_modulation(
        self,
        body_amp_scale: torch.Tensor,
        leg_yaw_amp_scale: torch.Tensor,
        leg_pitch_amp_scale: torch.Tensor,
        wave_number_scale: torch.Tensor,
        body_imp_scale: torch.Tensor,
        freq_scale: torch.Tensor,
    ) -> None:
        """Write the global_6d-mode TARGET scales into the runtime buffers.

        All six tensors must be shape (num_envs,). body_amp_scale is broadcast
        across all 19 body joints (anatomic): all joints share the same amp scale.

        Equation reduces to baseline byte-for-byte when every argument is the
        all-ones tensor.

        Args:
            body_amp_scale:        (num_envs,) global body amplitude scale.
            leg_yaw_amp_scale:     (num_envs,) leg DOF-0 amplitude scale.
            leg_pitch_amp_scale:   (num_envs,) leg DOF-1 amplitude scale.
            wave_number_scale:     (num_envs,) wave-number scale.
            body_imp_scale:        (num_envs,) DEAD since S2b — it fanned out to the
                                   body-yaw kp+kv GAIN scales, which no longer exist.
                                   MUST be 1.0; anything else RAISES (see
                                   _assert_dead_gain_scale). The campaign already pins
                                   action ch-4 via --design-a-fixed-imp-freq /
                                   --design-c-fix-all-non-wave, and Stage E's action is
                                   wave-only, so this costs nothing today.
            freq_scale:            (num_envs,) gait frequency scale.
        """
        E = self.num_envs
        for name, t in (
            ("body_amp_scale", body_amp_scale),
            ("leg_yaw_amp_scale", leg_yaw_amp_scale),
            ("leg_pitch_amp_scale", leg_pitch_amp_scale),
            ("wave_number_scale", wave_number_scale),
            ("body_imp_scale", body_imp_scale),
            ("freq_scale", freq_scale),
        ):
            if t.shape != (E,):
                raise ValueError(
                    f"{name} shape {tuple(t.shape)} != ({E},)"
                )
        self._assert_dead_gain_scale("apply_global_6d_modulation(body_imp_scale=)", body_imp_scale)
        body_amp_e = body_amp_scale.to(dtype=self.dtype, device=self.device)
        # Broadcast (E,) -> (E, 19) so every body joint shares the global
        # amplitude scale. Index 0 (head joint) is stored-but-unused in
        # the body wave equation (head uses heading servo).
        self.body_amp_scale_runtime.copy_(
            body_amp_e.unsqueeze(1).expand(E, N_BODY_JOINTS)
        )
        self.leg_yaw_amp_scale_runtime.copy_(
            leg_yaw_amp_scale.to(dtype=self.dtype, device=self.device)
        )
        self.leg_pitch_amp_scale_runtime.copy_(
            leg_pitch_amp_scale.to(dtype=self.dtype, device=self.device)
        )
        self.wave_number_scale_runtime.copy_(
            wave_number_scale.to(dtype=self.dtype, device=self.device)
        )
        self.freq_scale_runtime.copy_(
            freq_scale.to(dtype=self.dtype, device=self.device)
        )

    # ------------------------------------------------------------------
    # 2026-05-13: fast wave-number retargeting (Option B in
    # outputs/analysis/cpg_fast_wavenumber_retargeting_design.md)
    # ------------------------------------------------------------------
    def set_cpg_overrides(self, body_coupling_scale=1.0,
                          leg_coupling_scale=1.0, substeps=1,
                          update_mode="jacobi"):
        """Set the FARMS-equivalent CPG optimisation overrides.

        Default values (1.0, 1.0, 1, "jacobi") reproduce the
        pre-override controller behaviour bit-for-bit. Documented in
        `outputs/analysis/farms_equivalent_cpg_optimization/`.

        body_coupling_scale  multiplies `cpg_omega` inside
            `_integrate_cpg_phases` (i.e. scales the body Kuramoto
            coupling gain `g_body = body_amp · cpg_omega`).
        leg_coupling_scale   multiplies `cpg_leg_omega` for the leg
            Kuramoto step (scales `g_leg = leg_amp_ref · cpg_leg_omega`).
        substeps             number of internal Euler substeps per
            controller.step() invocation; substep dt = dt / substeps.
            substeps = 1 reproduces the original cadence.
        update_mode          "jacobi" (default; vectorised simultaneous
            update from old phases) or "sequential" (head→tail
            Gauss-Seidel update). FARMS uses "sequential"; IsaacLab
            historical default is "jacobi".
        """
        self._cpg_body_coupling_scale = float(body_coupling_scale)
        self._cpg_leg_coupling_scale = float(leg_coupling_scale)
        n = int(substeps)
        if n < 1:
            raise ValueError(f"cpg_substeps must be >= 1, got {n}")
        self._cpg_substeps = n
        if update_mode not in ("jacobi", "sequential"):
            raise ValueError(
                f"cpg_update_mode must be 'jacobi' or 'sequential', "
                f"got {update_mode!r}"
            )
        self._cpg_update_mode = str(update_mode)

    def set_fast_retargeting(self, enabled, max_rate_per_s=1.0):
        """Enable/disable phase-state retargeting + set per-step rate cap.

        When enabled, `_apply_phase_rebase` is called inside
        `_integrate_cpg_phases` (BEFORE the Kuramoto step) each `dt`.
        It shifts `body_phases` toward the current
        `wave_number_scale_runtime` gradient at a per-step
        `Δscale ≤ max_rate_per_s · dt`.

        When disabled (default), `_apply_phase_rebase` is never called
        and the controller's CPG path is identical to the
        pre-2026-05-13 baseline. The audit counter
        `self._rebase_call_count` must therefore stay at 0 during any
        flag-OFF rollout (verified by the §6.3 parity test).
        """
        self._fast_wave_retargeting_enabled = bool(enabled)
        self._fast_wave_retargeting_max_rate_per_s = float(max_rate_per_s)

    def _wrap_pi(self, x):
        return ((x + math.pi) % (2.0 * math.pi)) - math.pi

    # ------------------------------------------------------------------
    def reset_envs(self, env_ids=None, t_at_reset=0.0):
        """Reset CPG / heading / target-history state for the listed envs.

        Stage B Foundation (2026-05-07): partial-reset support. Equations
        unchanged; this only manages STATE so per-env asynchronous reset
        produces the same wave-fade-in behaviour each env had at startup.

        Args:
            env_ids: 1-D long tensor / list / None. None means all envs.
            t_at_reset: scalar float OR (len(env_ids),) tensor — the
                controller-clock value the listed envs are restarting
                from. Almost always 0.0 (they just reset).
        """
        # Resolve env_ids.
        if env_ids is None:
            ids = torch.arange(self.num_envs, dtype=torch.long, device=self.device)
        else:
            if torch.is_tensor(env_ids):
                ids = env_ids.to(dtype=torch.long, device=self.device).reshape(-1)
            else:
                ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device).reshape(-1)
        if ids.numel() == 0:
            return

        # Build per-env t0 for the listed ids.
        if torch.is_tensor(t_at_reset):
            t0_vec = t_at_reset.to(dtype=self.dtype, device=self.device).reshape(-1)
            assert t0_vec.numel() == ids.numel(), (
                f"t_at_reset shape {tuple(t0_vec.shape)} != env_ids shape {tuple(ids.shape)}"
            )
        else:
            t0_vec = torch.full((ids.numel(),), float(t_at_reset),
                                dtype=self.dtype, device=self.device)

        # ── 1) CPG phases ──
        # If body_phases hasn't been allocated yet, allocate at default
        # zero and let _init_cpg_phases handle full re-init at next step.
        if self.body_phases is None:
            self.body_phases = torch.zeros((self.num_envs, N_BODY_JOINTS),
                                            dtype=self.dtype, device=self.device)
        if self.leg_phases is None:
            self.leg_phases = torch.zeros((self.num_envs, N_LEGS),
                                           dtype=self.dtype, device=self.device)

        # Per-env phase initialisation: phi_i(t0) = omega_e * t0 - spatial_phases_i
        # Stage B Step 1: spatial_phases is now per-env (modulated by
        # wave_number_scale_runtime). At scale=1.0, values are bit-identical.
        omega_e = self._omega()[ids].unsqueeze(1)                            # (k, 1)
        spat_e = self._spatial_phases_per_env()[ids]                         # (k, N)
        new_body = omega_e * t0_vec.unsqueeze(1) - spat_e
        self.body_phases[ids] = new_body
        self.leg_phases[ids] = new_body[:, :N_LEGS] + 0.5 * self._cpg_delta()[ids]

        # ── 2) Heading reference: clear so next step() re-latches from yaw_world ──
        if self.head_yaw_ref is None:
            self.head_yaw_ref = torch.full((self.num_envs,), float("nan"),
                                            dtype=self.dtype, device=self.device)
        self.head_yaw_ref[ids] = float("nan")   # sentinel for "re-latch on next read"

        # ── 3) Per-env CPG-init flag is global; if these are the only
        #       envs running, ensure the global flag is True so step()
        #       enters the CPG branch on its next call. ──
        # If only some envs reset and CPG was already initialised globally,
        # leave _cpg_initialized as-is. If CPG wasn't initialised, the
        # next step() will call _init_cpg_phases for ALL envs (which is
        # correct — the un-reset envs still need their phase init).
        if not self._cpg_initialized:
            # Will be picked up on next step(); per-env body_phases for
            # the reset envs is already correct, others will be re-init.
            pass

        # ── 4) Per-env prev-target history (target_dot finite difference) ──
        self.reset_target_history(ids)

        # ── 5) Stage B Step 1 (2026-05-07): reset RL action modulation ──
        # back to baseline (1.0) for the listed envs. The next
        # _pre_physics_step will write fresh per-env values from the new
        # action, but during the brief window before that we want the
        # controller to behave as if the RL action were zero.
        # S2b (2026-07-16): only TARGET scales are reset here — the gain/effort
        # scales this block also reset are DELETED.
        self.body_amp_scale_runtime[ids] = 1.0
        self.wave_number_scale_runtime[ids] = 1.0
        self.leg_yaw_amp_scale_runtime[ids] = 1.0
        self.leg_pitch_amp_scale_runtime[ids] = 1.0
        self.freq_scale_runtime[ids] = 1.0

        # 2026-05-13: reset the rebase tracker so the next step
        # treats the post-reset wave_number_scale_runtime (= 1.0) as
        # the baseline; no spurious delta on the first post-reset
        # step. See §4.4 and §6.3 of the design packet.
        if self._last_rebase_wave_scale is not None:
            self._last_rebase_wave_scale[ids] = 1.0

    # ------------------------------------------------------------------
    def reset_target_history(self, env_ids=None):
        if env_ids is None:
            self.prev_body_yaw_targets = None
            self.prev_pitch_targets = None
            self.prev_leg_targets_flat = None
        else:
            if self.prev_body_yaw_targets is not None:
                self.prev_body_yaw_targets[env_ids] = 0.0
            if self.prev_pitch_targets is not None:
                self.prev_pitch_targets[env_ids] = 0.0
            if self.prev_leg_targets_flat is not None:
                self.prev_leg_targets_flat[env_ids] = 0.0

    def set_frequency(self, freq, env_ids=None):
        new = self._param_per_env(freq)
        if env_ids is None:
            self.freq = new
            self.body_phases = None
            self.leg_phases = None
            self._cpg_initialized = False
            self.head_yaw_ref = None
            self.reset_target_history()
        else:
            self.freq[env_ids] = new[env_ids]
            if self.body_phases is not None:
                self.body_phases[env_ids] = 0.0
            if self.leg_phases is not None:
                self.leg_phases[env_ids] = 0.0
            if self.head_yaw_ref is not None:
                self.head_yaw_ref[env_ids] = 0.0
            self.reset_target_history(env_ids)

    def _init_cpg_phases(self, t0):
        """Initialise body and leg phases from t0.

        Stage B (2026-05-07): t0 may be scalar OR (num_envs,) tensor.
        Per-env initialisation produces correct wave alignment after
        async resets at different times.
        """
        omega = self._omega()                           # (E,)
        if torch.is_tensor(t0) and t0.dim() == 1:
            t0_b = t0.to(dtype=self.dtype, device=self.device).unsqueeze(1)  # (E, 1)
        else:
            t0_scalar = float(t0.item()) if torch.is_tensor(t0) else float(t0)
            t0_b = torch.full((self.num_envs, 1), t0_scalar,
                              dtype=self.dtype, device=self.device)
        # Stage B Step 1: per-env spatial phases (wave-number-modulated).
        self.body_phases = (
            omega.unsqueeze(1) * t0_b - self._spatial_phases_per_env()
        ).clone()
        self.leg_phases = self._leg_phase_anchor(self.body_phases).clone()
        self._cpg_initialized = True

    def _apply_phase_rebase(self, dt):
        """Shift body_phases toward the current wave_number_scale_runtime
        gradient at a per-step rate-limited delta. See §4.4 of
        outputs/analysis/cpg_fast_wavenumber_retargeting_design.md.

        Equation:
          delta_target  = wave_number_scale_runtime - _last_rebase_wave_scale
          delta_applied = clamp(delta_target, ±max_rate_per_s · dt)
          body_phases  -= delta_applied · spatial_phases  (broadcast over joints)
          _last_rebase_wave_scale += delta_applied

        Leg phases are NOT rebased here — see §4.3 of the design
        packet for the non-rebase choice and §6.6 for the leg-body
        coordination audit that quantifies the transient impact.
        """
        # Lazy allocate the tracker — first call captures the current
        # wave_number_scale_runtime as the baseline, so the very first
        # rebase step produces a zero shift (no spurious transient
        # right after the flag is flipped on).
        if self._last_rebase_wave_scale is None:
            self._last_rebase_wave_scale = self.wave_number_scale_runtime.clone()
            self._rebase_call_count += 1
            return
        delta_target = (
            self.wave_number_scale_runtime - self._last_rebase_wave_scale
        )                                                            # (E,)
        dt_t = torch.as_tensor(float(dt), dtype=self.dtype, device=self.device)
        max_step = self._fast_wave_retargeting_max_rate_per_s * dt_t  # scalar
        delta_applied = torch.clamp(
            delta_target, min=-max_step, max=max_step
        )                                                            # (E,)
        # Per-env spatial-template shift broadcast over joints.
        # shape: (E, 1) * (N,) → (E, N).
        shift = delta_applied.unsqueeze(1) * self.spatial_phases.unsqueeze(0)
        self.body_phases = self.body_phases - shift
        self._last_rebase_wave_scale = (
            self._last_rebase_wave_scale + delta_applied
        )
        self._rebase_call_count += 1

    def _kuramoto_body_step_sequential(self, phi, omega, coupling_gain,
                                       delta, dt):
        """Gauss-Seidel (head→tail) variant of the body Kuramoto step.

        Each interior segment reads its already-updated backward
        neighbour for the same step. Mirrors FARMS's
        compute_body_wave iteration order (joint_i = 0..N−1 with
        in-place writes to oscillator_body_phases). Used when
        cpg_update_mode == "sequential".

        Equation form is identical to kuramoto_body_step; only
        evaluation order changes.
        """
        E, N = phi.shape
        omega_t = _to_phi_tensor(omega, phi)
        gain_t = _to_phi_tensor(coupling_gain, phi)
        if omega_t.dim() == 0:
            omega_2d = omega_t.expand(E)
        else:
            omega_2d = omega_t
        if gain_t.dim() == 0:
            gain_2d = gain_t.expand(E)
        else:
            gain_2d = gain_t.view(-1)
        # in-place sequential update; clone to avoid surprising the
        # caller if phi is shared.
        phi_new = phi.clone()
        for i in range(N):
            phi_self = phi_new[:, i]
            # forward neighbour (uses OLD phi[i+1] — already updated
            # values would be earlier in the chain, so phi[i+1] is
            # still old here when i < N-1)
            if i < N - 1:
                fwd = torch.sin(phi_self - (phi_new[:, i + 1] + delta.view(-1)))
            else:
                fwd = torch.zeros(E, dtype=phi.dtype, device=phi.device)
            # backward neighbour (uses NEW phi[i-1] — Gauss-Seidel)
            if i > 0:
                bwd = torch.sin(phi_self - (phi_new[:, i - 1] - delta.view(-1)))
            else:
                bwd = torch.zeros(E, dtype=phi.dtype, device=phi.device)
            phi_dot_i = omega_2d + gain_2d * (fwd + bwd)
            phi_new[:, i] = phi_self + dt * phi_dot_i
        return phi_new

    def _integrate_cpg_phases(self, dt):
        # 2026-05-13: phase-state retargeting (Option B). When enabled,
        # shift body_phases toward the current wave_number_scale_runtime
        # gradient BEFORE the Kuramoto step runs. When disabled, this
        # branch is skipped entirely and the controller path is
        # identical to the pre-2026-05-13 baseline (the audit counter
        # self._rebase_call_count must read 0 across any flag-OFF run).
        if self._fast_wave_retargeting_enabled:
            self._apply_phase_rebase(dt)
        # 2026-05-13: FARMS-equivalent CPG overrides.
        # At defaults (body_scale=leg_scale=1.0, substeps=1,
        # mode='jacobi') the call below is bit-identical to the
        # pre-override version (g_body·1.0 = g_body; dt/1 = dt; jacobi
        # branch is the original kuramoto_body_step).
        body_scale = self._cpg_body_coupling_scale
        leg_scale = self._cpg_leg_coupling_scale
        n_sub = self._cpg_substeps
        mode = self._cpg_update_mode
        coupling_gain = self.body_amp * self.cpg_omega * body_scale
        leg_gain = float(self.leg_amp_ref) * self.cpg_leg_omega * leg_scale
        sub_dt = dt / float(n_sub)
        for _sub in range(n_sub):
            if mode == "sequential":
                self.body_phases = self._kuramoto_body_step_sequential(
                    self.body_phases,
                    omega=self._omega(),
                    coupling_gain=coupling_gain,
                    delta=self._cpg_delta(),
                    dt=sub_dt,
                )
            else:
                self.body_phases = kuramoto_body_step(
                    self.body_phases,
                    omega=self._omega(),
                    coupling_gain=coupling_gain,
                    delta=self._cpg_delta(),
                    dt=sub_dt,
                )
            self.leg_phases = kuramoto_leg_step(
                self.leg_phases,
                phi_body_subset=self._leg_phase_anchor(self.body_phases),
                omega=self._omega(),
                coupling_gain_leg=leg_gain,
                dt=sub_dt,
            )
        self._cpg_integrate_call_count += 1

    # ------------------------------------------------------------------
    def step(self, t, dt):
        """Advance the CPG and publish this step's joint TARGETS. Returns None.

        t can be scalar (single global clock — Stage A) or a (num_envs,) tensor
        (per-env clock — Stage B). Target equations unchanged; only broadcasting differs.

        SIDE EFFECTS (this is the whole point of the call):
          - integrates the Kuramoto body/leg phases by dt (STATEFUL — must be called
            every physics step);
          - latches/advances the heading-servo reference;
          - updates the prev-target history used for the finite-difference vel target;
          - sets self._pd_pos_target / self._pd_vel_target, both (num_envs, num_joints),
            which env._apply_action_implicit_pd() writes to PhysX.

        RETURNS None. S2b (2026-07-16, PI-signed) deleted the torque: gains live in the
        env's impedance table and PhysX's implicit PD computes the force. The old
        `tau` return was already being DISCARDED by the env (S1); returning it only
        invited callers to believe it was a force. Un-migrated callers that do
        `tau = ctrl.step(...)` now fail on None immediately rather than silently
        applying a torque that no longer drives anything.
        """
        art = self.articulation
        E = self.num_envs

        # ── Normalize t ──
        # tensor mode is engaged when t is a 1-D tensor of length E.
        t_is_per_env = (
            torch.is_tensor(t) and t.dim() == 1 and t.numel() == E
        )
        if t_is_per_env:
            t_env = t.to(dtype=self.dtype, device=self.device)         # (E,)
            t_min = float(t_env.min())                                  # any-env-past-settle?
        else:
            t_scalar = float(t.item()) if torch.is_tensor(t) else float(t)
            t_env = torch.full((E,), t_scalar, dtype=self.dtype, device=self.device)
            t_min = t_scalar

        # S2b: joint_pos / joint_vel are NO LONGER read. They were the feedback terms of the
        # deleted PD law (q, qdot); the targets below depend only on the clock and the CPG phase.
        # Feedback now closes inside PhysX. The heading servo still needs the head's world quat.
        head_quat = art.data.body_quat_w[:, self.head_body_id].to(dtype=self.dtype)

        # CPG initialisation + integration — fixed 2026-05-07 for the
        # async-reset case:
        #   - Initialise once ANY env first reaches settle_time. Use
        #     t_env so each env's initial phase reflects its own clock;
        #     envs still in settle get the t=0 init they would have
        #     gotten anyway (because their _sim_time is small).
        #   - Once initialised, INTEGRATE EVERY STEP regardless of
        #     t_min. Earlier code gated integration on t_min >=
        #     settle_time which silently froze the CPG whenever a
        #     freshly-reset env dropped t_min below settle. seg_blend
        #     already gives per-env wave amplitude = 0 during settle,
        #     so integrating-but-not-applying is correct and lossless.
        if self.use_cpg:
            if not self._cpg_initialized:
                if t_is_per_env:
                    any_past_settle = bool((t_env >= self.settle_time).any().item())
                else:
                    any_past_settle = (t_min >= self.settle_time)
                if any_past_settle:
                    self._init_cpg_phases(t_env)
            if self._cpg_initialized:
                self._integrate_cpg_phases(dt)

        # ── BODY YAW ──
        yaw_world = quat_to_yaw(head_quat)
        # head_yaw_ref handling supports the NaN sentinel set by reset_envs:
        # any NaN entries are re-latched from yaw_world per-env.
        if self.head_yaw_ref is None:
            self.head_yaw_ref = yaw_world.clone()
        else:
            nan_mask = torch.isnan(self.head_yaw_ref)
            if bool(nan_mask.any().item()):
                self.head_yaw_ref = torch.where(
                    nan_mask, yaw_world, self.head_yaw_ref
                )
        # Heading rate integration is per-env (only past-settle envs accumulate)
        if self.head_yaw_rate != 0.0:
            past_settle = (t_env >= self.settle_time)
            self.head_yaw_ref = torch.where(
                past_settle,
                self.head_yaw_ref + self.head_yaw_rate * dt,
                self.head_yaw_ref,
            )
        err = self._wrap_pi(yaw_world - self.head_yaw_ref)
        head_target = torch.clamp(
            self.head_yaw_gain * err,
            -self.head_yaw_target_clip,
            self.head_yaw_target_clip,
        )

        # ── seg_blend supports per-env tensor t ──
        body_indices = torch.arange(1, N_BODY_JOINTS, device=self.device)
        blends_per_env = seg_blend(t_env, body_indices, self.settle_time,
                                    self.ramp_time, N_BODY_JOINTS)
        # blends_per_env shape: (E, N_BODY_JOINTS - 1)
        blends_per_env = blends_per_env.to(dtype=self.dtype)

        if self.use_cpg and self._cpg_initialized:
            phases = self.body_phases[:, 1:]
        else:
            # Non-CPG fallback: phases derived from t_env per-env.
            # Stage B Step 1: spatial phases are now per-env.
            base = (self._omega().unsqueeze(1) * t_env.unsqueeze(1)
                    - self._spatial_phases_per_env())
            phases = base[:, 1:]

        # Stage B Step 1: body_amp_scale_runtime[:, 1:] (E, N-1) is the
        # per-env per-joint amplitude modulation set by the RL action.
        # Defaults to 1.0 -> equation reduces to baseline byte-for-byte.
        targets_body = (
            blends_per_env
            * self.body_amp_scale[1:].unsqueeze(0)
            * self.body_amp_scale_runtime[:, 1:]
            * self.body_amp.unsqueeze(1)
            * torch.sin(phases)
        )

        all_body_targets = torch.empty((E, N_BODY_JOINTS), dtype=self.dtype, device=self.device)
        all_body_targets[:, 0] = head_target
        all_body_targets[:, 1:] = targets_body

        if self.prev_body_yaw_targets is not None and dt > 1e-12:
            body_target_dot = (all_body_targets - self.prev_body_yaw_targets) / dt
        else:
            body_target_dot = torch.zeros_like(all_body_targets)
        self.prev_body_yaw_targets = all_body_targets.clone()

        # ── PITCH ──
        if self.has_pitch:
            # head_blend_t is per-env when t_env is per-env (E,1) → (E,)
            head_blend_per_env = seg_blend(
                t_env, torch.tensor([0], device=self.device),
                self.settle_time, self.ramp_time, N_BODY_JOINTS,
            ).to(dtype=self.dtype).reshape(-1)   # (E,)
            # broadcast to (E, n_pitch) by outer product with pitch_target_final
            pitch_targets = head_blend_per_env.unsqueeze(1) * self.pitch_target_final.unsqueeze(0)
            if self.prev_pitch_targets is not None and dt > 1e-12:
                pitch_target_dot = (pitch_targets - self.prev_pitch_targets) / dt
            else:
                pitch_target_dot = torch.zeros_like(pitch_targets)
            self.prev_pitch_targets = pitch_targets.clone()

        # ── ROLL ── (target ≡ 0, target_dot ≡ 0 — held by the table's Kp/Kv about q=0)
        # S2b: no code. Roll is impedance-HELD and was never waved: its pos/vel targets are
        # identically zero, which is exactly what the zero-initialised publish buffers below
        # already carry at roll_joint_ids. The deleted block computed only its torque.
        # (has_roll / roll_joint_ids survive — env.s2_family_ids() enumerates the body_roll
        # family from them.)

        # ── LEGS ──
        leg_indices = torch.arange(N_LEGS, device=self.device)
        leg_blends_per_env = seg_blend(t_env, leg_indices, self.settle_time,
                                        self.ramp_time, N_LEGS)
        leg_blends_per_env = leg_blends_per_env.to(dtype=self.dtype)  # (E, N_LEGS)

        if self.use_cpg and self._cpg_initialized:
            leg_base_phases = self.leg_phases
        else:
            # Stage B Step 1: per-env spatial phases.
            base = (self._omega().unsqueeze(1) * t_env.unsqueeze(1)
                    - self._spatial_phases_per_env())
            leg_base_phases = self._leg_phase_anchor(base)

        base_flat = leg_base_phases[:, self.leg_seg_idx]
        full_phases = base_flat + self.leg_phase_off_flat.unsqueeze(0)
        wave_flat = torch.sin(full_phases)
        # blend_flat shape: (E, 152) — broadcast leg_blends_per_env via leg_seg_idx
        blend_flat = leg_blends_per_env[:, self.leg_seg_idx]

        active_mask = self.leg_active_flat.unsqueeze(0) & (blend_flat > 0)
        # Stage B-v0 (2026-05-07): per-env per-DOF leg amplitude scaling.
        # leg_amp_runtime[e, k] is 1.0 by default for ALL k (parity), and
        # becomes:
        #   leg_yaw_amp_scale_runtime[e]   when leg_dof_idx_flat[k] == 0
        #   leg_pitch_amp_scale_runtime[e] when leg_dof_idx_flat[k] == 1
        # DOFs 2 and 3 are passive in the active gait and stay at 1.0.
        leg_amp_runtime = torch.ones((E, self.leg_amp_flat.shape[0]),
                                      dtype=self.dtype, device=self.device)
        mask_dof0 = (self.leg_dof_idx_flat == 0)
        mask_dof1 = (self.leg_dof_idx_flat == 1)
        if mask_dof0.any():
            leg_amp_runtime[:, mask_dof0] = self.leg_yaw_amp_scale_runtime.unsqueeze(1)
        if mask_dof1.any():
            leg_amp_runtime[:, mask_dof1] = self.leg_pitch_amp_scale_runtime.unsqueeze(1)
        wave_term = (
            blend_flat
            * self.leg_signs.unsqueeze(0)
            * self.leg_amp_flat.unsqueeze(0)
            * leg_amp_runtime
            * wave_flat
            + self.leg_dc_flat.unsqueeze(0)
        )
        dc_only = self.leg_dc_flat.unsqueeze(0).expand(E, -1)
        targets_flat = torch.where(active_mask, wave_term, dc_only)

        if self.prev_leg_targets_flat is not None and dt > 1e-12:
            leg_target_dot = (targets_flat - self.prev_leg_targets_flat) / dt
        else:
            leg_target_dot = torch.zeros_like(targets_flat)
        self.prev_leg_targets_flat = targets_flat.clone()

        # ── PUBLISH the targets ──
        # S2b: unconditional (the old `_export_pd` flag is gone). Targets are now this
        # module's ONLY output, so there is nothing left for the flag to gate: with it OFF
        # step() would compute a whole gait and publish nothing. The _pd_kp / _pd_kv half of
        # the old stash is deleted — env._apply_action_implicit_pd reads only the targets
        # ("ctrl._pd_kp / ctrl._pd_kv are deliberately NOT read", envs/centipede_env.py).
        # Joints with no family below (e.g. roll, and any passive joint) keep target 0/0.
        _pp = torch.zeros((E, self.num_joints), dtype=self.dtype, device=self.device)
        _vv = torch.zeros((E, self.num_joints), dtype=self.dtype, device=self.device)
        _pp[:, self.body_joint_ids] = all_body_targets
        _vv[:, self.body_joint_ids] = body_target_dot
        if self.has_pitch:
            _pp[:, self.pitch_joint_ids] = pitch_targets
            _vv[:, self.pitch_joint_ids] = pitch_target_dot
        _pp[:, self.leg_flat_ids] = targets_flat
        _vv[:, self.leg_flat_ids] = leg_target_dot
        if self.disabled_joint_indices:
            _dis = torch.as_tensor(self.disabled_joint_indices, dtype=torch.long, device=self.device)
            _pp[:, _dis] = 0.0
            _vv[:, _dis] = 0.0
        self._pd_pos_target = _pp
        self._pd_vel_target = _vv
        return None
