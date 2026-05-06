"""
impedance_controller.py — Impedance traveling-wave controller for Isaac Lab
============================================================================
Port of the MuJoCo ImpedanceTravelingWaveController to Isaac Lab.

VECTORIZED VERSION: all joint loops replaced with numpy batch operations.
GPU↔CPU transfers minimized to one read + one write per step.

The control LAW is identical to MuJoCo — only the data access changed.
"""

import math
import numpy as np
import torch
import yaml

from controllers.kinematics import (
    IsaacLabModelIndex,
    N_BODY_JOINTS, N_LEGS, N_LEG_DOF, N_PITCH, N_ROLL, ACTIVE_DOFS,
)


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def quat_to_yaw(quat_wxyz):
    """Extract yaw from (w,x,y,z) quaternion — same as atan2(R[1,0], R[0,0])."""
    w, x, y, z = quat_wxyz
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ImpedanceTravelingWaveController:
    """
    Vectorized impedance traveling wave controller for Isaac Lab.

    All per-joint Python for-loops are replaced with numpy array operations.
    Only the heading servo (1 joint) and CPG integration (19 oscillators)
    remain as scalar loops.
    """

    def __init__(self, articulation, config_path,
                 body_kp=None, body_kv=None,
                 pitch_kp=None, pitch_kv=None,
                 roll_kp=None, roll_kv=None,
                 env_idx=0):
        cfg = load_config(config_path)
        self.articulation = articulation
        self.env_idx = env_idx
        self.device = articulation.device

        # ── index resolution ──
        self.idx = IsaacLabModelIndex(articulation)

        # ── Disabled joints (workaround for PhysX compound-joint bug) ──
        # In the original MJCF, multi-DOF compound joints (body-pitch +
        # body-roll, leg-yaw + leg-roll) caused PhysX to lock one DOF
        # at body 10's cluster. We worked around it by zeroing the
        # commanded torque on those joints. With the simplified MJCF
        # (centipede_simple.xml — see scripts/simplify_mjcf.py) all
        # compound joints are removed, so the workaround is no longer
        # needed and this list is empty by default. Leaving the
        # mechanism in place so it can be re-populated externally for
        # future debugging.
        self.disabled_joint_indices = []

        # ── body wave params ──
        bw = cfg["body_wave"]
        self.body_amp = float(bw["amplitude"])
        self.freq     = float(bw["frequency"])
        self.n_wave   = float(bw["wave_number"])
        self.speed    = float(bw["speed"])
        self.omega    = 2.0 * math.pi * self.freq

        # ── head-end spatial envelope ──
        self.head_hold_joints = int(bw.get("head_hold_joints", 0))
        self.head_ramp_joints = int(bw.get("head_ramp_joints", 2))
        self.head_ramp_min    = float(bw.get("head_ramp_min", 0.0))
        self._build_body_amp_scale()

        # ── CPG ──
        cpg_cfg = cfg.get("cpg", {})
        self.use_cpg       = bool(cpg_cfg.get("enabled", True))
        self.cpg_omega     = float(cpg_cfg.get("coupling_omega", -5.0))
        self.cpg_leg_omega = float(cpg_cfg.get("leg_coupling_omega", self.cpg_omega))
        self.cpg_delta = (2.0 * math.pi * self.n_wave * self.speed
                          / max(N_BODY_JOINTS - 1, 1))
        self.body_phases = None
        self.leg_phases  = None
        self._cpg_initialized = False

        # ── impedance gains ──
        imp = cfg.get("impedance", {})
        self.body_kp = body_kp if body_kp is not None else float(imp.get("body_kp", 5.0))
        self.body_kv = body_kv if body_kv is not None else float(imp.get("body_kv", 0.05))
        self.body_kp_head = float(imp.get("body_kp_head", self.body_kp))
        self.body_kv_head = float(imp.get("body_kv_head", self.body_kv))
        self._build_body_kp_vec()

        self.pitch_kp = pitch_kp if pitch_kp is not None else float(imp.get("pitch_kp", 0.005))
        self.pitch_kv = pitch_kv if pitch_kv is not None else float(imp.get("pitch_kv", 0.002))
        self.roll_kp  = roll_kp if roll_kp is not None else float(imp.get("roll_kp", 0.005))
        self.roll_kv  = roll_kv if roll_kv is not None else float(imp.get("roll_kv", 0.002))

        self.settle_time = float(imp.get("settle_time", 1.0))
        self.ramp_time   = float(imp.get("ramp_time", 1.0))

        # Leg gains
        leg_imp = imp.get("leg", {})
        self.leg_kp = np.array(leg_imp.get("kp", [0.25]*4), dtype=float)
        self.leg_kv = np.array(leg_imp.get("kv", [0.05]*4), dtype=float)

        # ── leg wave params ──
        lw = cfg["leg_wave"]
        self.leg_amps          = np.array(lw["amplitudes"], dtype=float)
        self.leg_phase_offsets = np.array(lw["phase_offsets"], dtype=float)
        self.leg_dc_offsets    = np.array(lw["dc_offsets"], dtype=float)
        self.active_dofs       = set(lw["active_dofs"])

        # ── heading servo ──
        self.head_yaw_gain       = float(imp.get("head_yaw_gain", 1.0))
        self.head_yaw_target_clip = float(imp.get("head_yaw_target_clip", 0.5))
        self.head_yaw_rate       = float(imp.get("head_yaw_rate", 0.0))
        self.head_yaw_ref        = None

        # ── head pitch offset ──
        self.head_pitch_offset = float(imp.get("head_pitch_offset", 0.0))
        self.head_pitch_joints = int(imp.get("head_pitch_joints", 1))
        self.head_pitch_kp = float(imp.get("head_pitch_kp", self.pitch_kp * 20.0))
        self.head_pitch_kv = float(imp.get("head_pitch_kv", self.pitch_kv * 20.0))

        # Pitch gain/target vectors
        if self.idx.has_pitch:
            n_p = len(self.idx.pitch_joint_ids)
            self.pitch_kp_vec = np.full(n_p, self.pitch_kp, dtype=float)
            self.pitch_kv_vec = np.full(n_p, self.pitch_kv, dtype=float)
            self.pitch_target_final = np.zeros(n_p, dtype=float)
            for k in range(min(self.head_pitch_joints, n_p)):
                self.pitch_target_final[k] = self.head_pitch_offset
                self.pitch_kp_vec[k] = self.head_pitch_kp
                self.pitch_kv_vec[k] = self.head_pitch_kv

        # ── Pre-compute spatial phase offsets for vectorized body/leg targets ──
        self._spatial_phases = np.array([
            2.0 * math.pi * self.n_wave * self.speed * i / max(N_BODY_JOINTS - 1, 1)
            for i in range(N_BODY_JOINTS)
        ], dtype=float)

        # ── Pre-build flat index arrays for vectorized leg torque ──
        # leg_joint_ids shape: (N_LEGS, 2, N_LEG_DOF) → flatten for batch indexing
        self._leg_flat_ids = self.idx.leg_joint_ids.reshape(-1)  # (N_LEGS*2*4,)
        # Per-DOF kp/kv tiled to match flat layout: [dof0,dof1,dof2,dof3] × N_LEGS×2
        self._leg_kp_flat = np.tile(self.leg_kp, N_LEGS * 2)     # (N_LEGS*2*4,)
        self._leg_kv_flat = np.tile(self.leg_kv, N_LEGS * 2)

        # Pre-build sign array: +1 for L, -1 for R, repeated per DOF
        signs = np.array([1.0, -1.0])  # [L, R]
        self._leg_signs = np.repeat(signs, N_LEG_DOF)          # (8,) per leg
        self._leg_signs = np.tile(self._leg_signs, N_LEGS)     # (N_LEGS*2*4,)

        # DC offsets tiled
        self._leg_dc_flat = np.tile(self.leg_dc_offsets, N_LEGS * 2)

        # Active DOF mask: True where dof is in active_dofs
        dof_mask = np.array([d in self.active_dofs for d in range(N_LEG_DOF)], dtype=bool)
        self._leg_active_flat = np.tile(dof_mask, N_LEGS * 2)

        # Phase offsets per DOF, tiled
        self._leg_phase_off_flat = np.tile(self.leg_phase_offsets, N_LEGS * 2)

        # Amplitude per DOF, tiled
        self._leg_amp_flat = np.tile(self.leg_amps, N_LEGS * 2)

        # Leg base phase index: which leg index each flat entry belongs to
        # Each leg has 2 sides × 4 DOF = 8 entries
        self._leg_seg_idx = np.repeat(np.arange(N_LEGS), 2 * N_LEG_DOF)

        # ── Pre-allocate torque tensor on GPU (avoids per-step allocation) ──
        n_joints = articulation.num_joints
        self.torque_buf = np.zeros(n_joints, dtype=np.float32)
        self._torque_tensor = torch.zeros(1, n_joints, dtype=torch.float32,
                                          device=self.device)

        # ── Trajectory-tracking PD: cache previous-step targets so we
        #    can compute target velocity numerically as
        #        target_dot ≈ (target_now − target_prev) / dt
        #    and use   τ = kp·(target − q) + kv·(target_dot − qdot)
        #    instead of the regulator law τ = kp·(target − q) − kv·qdot.
        #    On the first step (prev = None) target_dot is taken as zero
        #    — this is the safe assumption and only affects step #1.
        #    To reset across episodes, call self.reset_target_history(). ──
        self._prev_body_yaw_targets   = None
        self._prev_pitch_targets      = None
        self._prev_leg_targets_flat   = None

        print(f"[IsaacLab ImpedanceController] Initialized (VECTORIZED, "
              f"PD-tracking): "
              f"body_kp={self.body_kp:.4f}, freq={self.freq:.2f}Hz, "
              f"CPG={'ON' if self.use_cpg else 'OFF'}")

    def reset_target_history(self):
        """Clear cached previous-step targets so the next step starts
        fresh (target_dot=0). Call this from episode-reset code."""
        self._prev_body_yaw_targets   = None
        self._prev_pitch_targets      = None
        self._prev_leg_targets_flat   = None

    def set_frequency(self, freq: float):
        """Update the body-wave frequency mid-trial and reset all phase
        and history state so the new frequency takes effect cleanly.

        Used by the three-evaluation optimizer to test a single gain set
        at multiple frequencies (e.g., flat-2Hz, rough-1Hz, rough-2Hz)
        within a single Isaac Sim session. Without resetting CPG /
        target-history state, residual phases from the previous frequency
        would corrupt the first second or so of the new evaluation.
        """
        self.freq  = float(freq)
        self.omega = 2.0 * math.pi * self.freq
        # cpg_delta does NOT depend on frequency (it's a spatial inter-
        # segment phase lag), so it stays the same.
        # Reset all phase / target-history state.
        self.body_phases       = None
        self.leg_phases        = None
        self._cpg_initialized  = False
        self.head_yaw_ref      = None
        self.reset_target_history()

    # ──────────────────────────────────────────────────────────────
    # Envelope / gain helpers
    # ──────────────────────────────────────────────────────────────

    def _build_body_amp_scale(self):
        n = N_BODY_JOINTS
        scale = np.ones(n, dtype=float)
        H = max(int(self.head_hold_joints), 0)
        R = max(int(self.head_ramp_joints), 0)
        amin = float(self.head_ramp_min)
        for i in range(min(H, n)):
            scale[i] = 0.0
        if R > 0:
            for k in range(1, R + 1):
                i = H + k - 1
                if i >= n:
                    break
                s = amin + (1.0 - amin) * 0.5 * (
                    1.0 - math.cos(math.pi * k / (R + 1)))
                scale[i] = s
        self.body_amp_scale = scale

    def _build_body_kp_vec(self):
        n = N_BODY_JOINTS
        kp_vec = np.full(n, self.body_kp, dtype=float)
        kv_vec = np.full(n, self.body_kv, dtype=float)
        H = max(int(self.head_hold_joints), 0)
        for i in range(min(H, n)):
            kp_vec[i] = self.body_kp_head
            kv_vec[i] = self.body_kv_head
        self.body_kp_vec = kp_vec
        self.body_kv_vec = kv_vec

    # ──────────────────────────────────────────────────────────────
    # Blend (vectorized for arrays of segment indices)
    # ──────────────────────────────────────────────────────────────

    def _seg_blend_vec(self, t, indices, n_seg=None):
        """Vectorized blend: returns array of blend factors for all indices."""
        if n_seg is None:
            n_seg = N_BODY_JOINTS
        if t < self.settle_time:
            return np.zeros(len(indices), dtype=float)
        if self.ramp_time <= 0.0:
            return np.ones(len(indices), dtype=float)
        per_seg = self.ramp_time * 0.5
        stagger = per_seg / max(n_seg - 1, 1)
        starts = self.settle_time + indices * stagger
        elapsed = np.maximum(t - starts, 0.0)
        blend = np.where(
            elapsed >= per_seg,
            1.0,
            0.5 * (1.0 - np.cos(np.pi * elapsed / per_seg))
        )
        blend[t <= starts] = 0.0
        return blend

    def _seg_blend(self, t, i, n_seg=None):
        """Scalar version for single index (heading servo)."""
        if n_seg is None:
            n_seg = N_BODY_JOINTS
        if t < self.settle_time:
            return 0.0
        if self.ramp_time <= 0.0:
            return 1.0
        per_seg = self.ramp_time * 0.5
        stagger = per_seg / max(n_seg - 1, 1)
        start_i = self.settle_time + i * stagger
        if t <= start_i:
            return 0.0
        elapsed = t - start_i
        if elapsed >= per_seg:
            return 1.0
        return 0.5 * (1.0 - math.cos(math.pi * elapsed / per_seg))

    # ──────────────────────────────────────────────────────────────
    # CPG (Kuramoto chain) — kept as scalar loop (only 19 iterations)
    # ──────────────────────────────────────────────────────────────

    def _init_cpg_phases(self, t0):
        body = self.omega * t0 - self._spatial_phases
        self.body_phases = body.copy()
        self.leg_phases  = body[:N_LEGS].copy()
        self._cpg_initialized = True

    def _integrate_cpg_phases(self, dt):
        phi = self.body_phases
        d   = self.cpg_delta
        N   = N_BODY_JOINTS
        gain_body = self.body_amp * self.cpg_omega

        # Vectorized nearest-neighbor coupling
        new_phi = np.empty_like(phi)
        if N == 1:
            new_phi[0] = phi[0] + self.omega * dt
        else:
            # Interior: coupling from both neighbors
            fdot = np.full(N, self.omega)
            # Forward neighbor coupling: sin(phi[i] - (phi[i+1] + d))
            fwd = np.sin(phi[:-1] - (phi[1:] + d))
            # Backward neighbor coupling: sin(phi[i] - (phi[i-1] - d))
            bwd = np.sin(phi[1:] - (phi[:-1] - d))
            # Head (i=0): only forward
            fdot[0] += gain_body * fwd[0]
            # Interior (i=1..N-2): both
            fdot[1:-1] += gain_body * (bwd[:-1] + fwd[1:])
            # Tail (i=N-1): only backward
            fdot[-1] += gain_body * bwd[-1]
            new_phi = phi + fdot * dt
        self.body_phases = new_phi

        # Leg phases: vectorized coupling to body
        leg_amp_ref = float(self.leg_amps[0]) if len(self.leg_amps) > 0 else 1.0
        gain_leg = leg_amp_ref * self.cpg_leg_omega
        lphi = self.leg_phases
        fdot_leg = self.omega + gain_leg * np.sin(lphi - new_phi[:N_LEGS])
        self.leg_phases = lphi + fdot_leg * dt

    # ──────────────────────────────────────────────────────────────
    # Main step — VECTORIZED
    # ──────────────────────────────────────────────────────────────

    def step(self, t: float, dt: float):
        """Compute and apply torques for one simulation step."""
        art = self.articulation
        ei = self.env_idx

        # Single GPU→CPU read for all joints
        all_pos = art.data.joint_pos[ei].cpu().numpy()
        all_vel = art.data.joint_vel[ei].cpu().numpy()

        # Zero torque buffer
        self.torque_buf[:] = 0.0

        # ── CPG integration ──
        if self.use_cpg and t >= self.settle_time:
            if not self._cpg_initialized:
                self._init_cpg_phases(t)
            self._integrate_cpg_phases(dt)

        # ══════════════════════════════════════════════════════════
        # BODY YAW — vectorized (except heading servo at i=0)
        # ══════════════════════════════════════════════════════════
        body_ids = self.idx.body_joint_ids  # (19,)

        # --- Heading servo (i=0) — scalar ---
        head_quat = art.data.body_quat_w[ei, self.idx.head_body_id].cpu().numpy()
        yaw_world = quat_to_yaw(head_quat)
        if self.head_yaw_ref is None:
            self.head_yaw_ref = yaw_world
        if self.head_yaw_rate != 0.0 and t >= self.settle_time:
            self.head_yaw_ref += self.head_yaw_rate * dt
        err = yaw_world - self.head_yaw_ref
        if err > math.pi:  err -= 2.0 * math.pi
        if err < -math.pi: err += 2.0 * math.pi
        head_target = np.clip(self.head_yaw_gain * err,
                              -self.head_yaw_target_clip,
                               self.head_yaw_target_clip)

        # --- Body joints i=1..18 — vectorized ---
        body_indices = np.arange(1, N_BODY_JOINTS)  # [1, 2, ..., 18]
        blends = self._seg_blend_vec(t, body_indices)

        if self.use_cpg and self._cpg_initialized:
            phases = self.body_phases[1:]
        else:
            phases = self.omega * t - self._spatial_phases[1:]

        targets_body = blends * self.body_amp_scale[1:] * self.body_amp * np.sin(phases)

        # Combine heading + body targets
        all_body_targets = np.empty(N_BODY_JOINTS, dtype=float)
        all_body_targets[0] = head_target
        all_body_targets[1:] = targets_body
        # Expose to external observers (e.g. SensorRecorder for tracking RMSE).
        self.last_body_yaw_targets = all_body_targets.copy()

        # Batch torque computation for all 19 body yaw joints
        # ── PD-TRACKING form: τ = kp·(target − q) + kv·(target_dot − qdot)
        #    — see reset_target_history(). target_dot is computed via
        #    finite difference from the previous step's target. ──
        body_q    = all_pos[body_ids]
        body_qdot = all_vel[body_ids]
        if self._prev_body_yaw_targets is not None and dt > 1e-12:
            body_target_dot = (all_body_targets - self._prev_body_yaw_targets) / dt
        else:
            body_target_dot = np.zeros_like(all_body_targets)
        self._prev_body_yaw_targets = all_body_targets.copy()
        body_tau  = (self.body_kp_vec * (all_body_targets - body_q)
                     + self.body_kv_vec * (body_target_dot - body_qdot))
        self.torque_buf[body_ids] = body_tau

        # ══════════════════════════════════════════════════════════
        # PITCH — fully vectorized
        # ══════════════════════════════════════════════════════════
        if self.idx.has_pitch:
            pitch_ids = self.idx.pitch_joint_ids
            head_blend = self._seg_blend(t, 0)
            # Store on self so SensorRecorder can read pitch tracking targets.
            self.pitch_targets = self.pitch_target_final * head_blend
            pitch_q    = all_pos[pitch_ids]
            pitch_qdot = all_vel[pitch_ids]
            # ── PD-TRACKING — target_dot via finite difference ──
            if self._prev_pitch_targets is not None and dt > 1e-12:
                pitch_target_dot = (self.pitch_targets - self._prev_pitch_targets) / dt
            else:
                pitch_target_dot = np.zeros_like(self.pitch_targets)
            self._prev_pitch_targets = self.pitch_targets.copy()
            pitch_tau  = (self.pitch_kp_vec * (self.pitch_targets - pitch_q)
                          + self.pitch_kv_vec * (pitch_target_dot - pitch_qdot))
            self.torque_buf[pitch_ids] = pitch_tau

        # ══════════════════════════════════════════════════════════
        # ROLL — fully vectorized
        # ══════════════════════════════════════════════════════════
        if self.idx.has_roll:
            roll_ids = self.idx.roll_joint_ids
            roll_q    = all_pos[roll_ids]
            roll_qdot = all_vel[roll_ids]
            roll_tau  = self.roll_kp * (0.0 - roll_q) - self.roll_kv * roll_qdot
            self.torque_buf[roll_ids] = roll_tau

        # ══════════════════════════════════════════════════════════
        # LEGS — fully vectorized across all N_LEGS × 2 × 4 = 152 joints
        # ══════════════════════════════════════════════════════════
        # Compute per-leg blend factors
        leg_indices = np.arange(N_LEGS)
        leg_blends = self._seg_blend_vec(t, leg_indices, n_seg=N_LEGS)

        # Base phases per leg
        if self.use_cpg and self._cpg_initialized:
            leg_base_phases = self.leg_phases  # (N_LEGS,)
        else:
            leg_base_phases = self.omega * t - self._spatial_phases[:N_LEGS]

        # Expand base phases to flat layout: each leg has 2*4=8 entries
        base_flat = leg_base_phases[self._leg_seg_idx]  # (N_LEGS*2*4,)

        # Full phases with per-DOF offsets
        full_phases = base_flat + self._leg_phase_off_flat

        # Wave values
        wave_flat = np.sin(full_phases)

        # Blend expanded to flat
        blend_flat = leg_blends[self._leg_seg_idx]

        # Targets: active DOFs get wave, inactive get dc_offset
        targets_flat = np.where(
            self._leg_active_flat & (blend_flat > 0),
            blend_flat * self._leg_signs * self._leg_amp_flat * wave_flat + self._leg_dc_flat,
            self._leg_dc_flat
        )
        # Expose targets reshaped to (N_LEGS, 2, N_LEG_DOF) for external
        # observers (e.g. SensorRecorder for leg tracking RMSE).
        self.last_leg_targets = targets_flat.reshape(N_LEGS, 2, N_LEG_DOF).copy()

        # Batch torque
        # ── PD-TRACKING — target_dot via finite difference ──
        leg_q    = all_pos[self._leg_flat_ids]
        leg_qdot = all_vel[self._leg_flat_ids]
        if self._prev_leg_targets_flat is not None and dt > 1e-12:
            leg_target_dot = (targets_flat - self._prev_leg_targets_flat) / dt
        else:
            leg_target_dot = np.zeros_like(targets_flat)
        self._prev_leg_targets_flat = targets_flat.copy()
        leg_tau  = (self._leg_kp_flat * (targets_flat - leg_q)
                    + self._leg_kv_flat * (leg_target_dot - leg_qdot))
        self.torque_buf[self._leg_flat_ids] = leg_tau

        # ══════════════════════════════════════════════════════════
        # DISABLED-JOINT MASK (applied AFTER all torque scatter — must
        # come last so it overrides body-yaw, pitch, roll, and leg
        # torque writes alike).
        # ══════════════════════════════════════════════════════════
        # See __init__ for rationale. PhysX's articulation solver
        # locks several multi-DOF compound joints at body 10 of the
        # centipede; the controller's stiff impedance ramps massive
        # torques into those locked DOFs and the reaction destabilises
        # the whole robot. Zeroing those torques makes body 10's
        # neighbourhood passive and the rest of the gait stable.
        if hasattr(self, "disabled_joint_indices") and self.disabled_joint_indices:
            for jid in self.disabled_joint_indices:
                self.torque_buf[jid] = 0.0

        # ══════════════════════════════════════════════════════════
        # APPLY — single CPU→GPU transfer
        # ══════════════════════════════════════════════════════════
        self._torque_tensor[0] = torch.from_numpy(self.torque_buf)
        art.set_joint_effort_target(self._torque_tensor)
        art.write_data_to_sim()
