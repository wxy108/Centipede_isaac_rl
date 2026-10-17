"""
CentipedeEnv — Stage A skeleton.

A DirectRLEnv subclass that:
  - Clones N centipede articulations across env namespaces.
  - Instantiates the torch-vectorized impedance + Kuramoto-CPG controller
    on the cloned articulation.
  - Calls controller.step(t, dt) every physics step, applies returned
    torques via set_joint_effort_target.
  - Returns ZERO actions / placeholder obs / placeholder reward /
    timeout-only termination.

Stage A goal: confirm the env loads, the centipede spawns N times, the
controller runs without crashing, and we can step the env for a full
episode without errors. NO RL is happening yet; that comes in Stage B.
"""

from __future__ import annotations

import math
import os
import sys

import torch

from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv

from envs.centipede_env_cfg import (
    CentipedeEnvCfg,
    CONTROLLER_CONFIG_PATH,
    PROJECT_ROOT,
)
from controllers.impedance_controller_torch import (
    ImpedanceTravelingWaveControllerTorch,
)


# =============================================================================
# Stage B Step 1 (2026-05-07): RL action -> controller scale mapping
# =============================================================================
# Conservative bounded scales. Zero action maps to scale = 1.0 for ALL
# parameters. Asymmetric body_kp bound [0.7, 1.5] is handled by the
# half-aware tanh map below (symmetric maps would force midpoint=1.1
# and break the zero-action=baseline invariant).

_BODY_AMP_LO, _BODY_AMP_HI = 0.7, 1.3
_WAVE_NUM_LO, _WAVE_NUM_HI = 0.8, 1.2
_BODY_KP_LO,  _BODY_KP_HI  = 0.7, 1.5

# =============================================================================
# Stage B-v0 (2026-05-07): RL action -> controller scale mapping (global_6d)
# =============================================================================
# Six-dimensional global action layout, baseline-centered:
#   a0 body yaw amp scale       in [_G6_BODY_AMP_LO, _G6_BODY_AMP_HI]
#   a1 leg yaw amp scale        in [_G6_LEG_YAW_AMP_LO, _G6_LEG_YAW_AMP_HI]
#   a2 leg pitch amp scale      in [_G6_LEG_PITCH_AMP_LO, _G6_LEG_PITCH_AMP_HI]
#   a3 body wave number scale   in [_G6_WAVE_NUM_LO, _G6_WAVE_NUM_HI]
#   a4 body yaw impedance scale in [_G6_BODY_IMP_LO, _G6_BODY_IMP_HI]   <-- STRUCTURALLY DEAD, see below
#   a5 frequency scale          in [_G6_FREQ_LO, _G6_FREQ_HI]
# All ranges contain 1.0, so a=0 -> scale=1.0 -> baseline behaviour.
# These are the conservative initial ranges; later experiments may
# widen them (see Stage B-v0 spec doc).
_G6_BODY_AMP_LO,       _G6_BODY_AMP_HI       = 0.3, 1.5
_G6_LEG_YAW_AMP_LO,    _G6_LEG_YAW_AMP_HI    = 0.3, 1.5
_G6_LEG_PITCH_AMP_LO,  _G6_LEG_PITCH_AMP_HI  = 0.3, 1.5
_G6_WAVE_NUM_LO,       _G6_WAVE_NUM_HI       = 0.5, 2.5
_G6_BODY_IMP_LO,       _G6_BODY_IMP_HI       = 0.3, 3.0   # DEAD RANGE -- see _S2_DEAD_ACTION_NOTE
_G6_FREQ_LO,           _G6_FREQ_HI           = 0.7, 1.3

# ═════════════════════════════════════════════════════════════════════════════════════════════
# _S2_DEAD_ACTION_NOTE -- READ THIS BEFORE TOUCHING ACTION ch-4 / ch-20 OR THE IMPEDANCE REWARD
# ═════════════════════════════════════════════════════════════════════════════════════════════
# The impedance ACTION channel is structurally dead as of 2026-07-16 (implicit-only unification,
# S1/S2a/S2b, PI-signed). This note is the one place the whole story is written down; the reward
# term (`imp_term` / `r_imp`), the log key (`raw/body_imp_scale`) and the two action decoders all
# point here.
#
# WHAT IT USED TO DO
#   global_6d action ch-4 ("body_imp_scale") and legacy body_21d action ch-20 ("body_kp_scale")
#   were mapped through _bounded_scale into the ranges above, low-passed into _u_applied[:, 4],
#   and handed to controller.apply_global_6d_modulation(body_imp_scale=...). The controller fanned
#   that scale out onto its body-yaw kp/kv vectors and used them in its own PD law
#   (tau = kp*(q_ref - q) + kv*(qd_ref - qd)), which the env then applied as an explicit torque.
#   So the policy really could stiffen/soften the spine in-episode. That is the plant every
#   impedance-related reward/log in this file was written for.
#
# WHY IT IS DEAD NOW (this is an ARCHITECTURE change, not a tuning change)
#   S1 deleted the explicit torque path. S2a made ONE joint family = exactly ONE (Kp, Kv) in
#   PHYSICAL units, written straight to PhysX (env.set_joint_impedance -> write_joint_stiffness_to_sim
#   / write_joint_damping_to_sim in _apply_action_implicit_pd). S2b then deleted the controller's
#   entire gain/torque machinery: `controller.body_kp_scale_runtime` NO LONGER EXISTS, the
#   controller holds no kp/kv, computes no torque, reads no joint state, and returns None from
#   step(). It is now purely a REFERENCE GENERATOR emitting position/velocity TARGETS; ALL feedback
#   closes inside PhysX's implicit PD solver. There is no longer any wire from a per-step action
#   scale to a stiffness.
#
# WHAT HAPPENS IF A POLICY MOVES ch-4 TODAY -- IT RAISES, IT DOES NOT SILENTLY NO-OP
#   controller.apply_global_6d_modulation / apply_action_modulation call _assert_dead_gain_scale,
#   which raises ValueError on any value != 1.0; env._s2_assert_scales_inert() is the second line
#   of defence (_S2_DEAD_SCALES). This is deliberate (S1 fail-loudly doctrine): a silently dropped
#   modulation is a run whose author believes the policy is tuning stiffness when it is not.
#   CONSEQUENCE, worth knowing before you launch anything:
#     - global_6d WITHOUT --design-a-fixed-imp-freq (or --design-c-fix-all-non-wave) RAISES as soon
#       as the policy emits a non-zero ch-4, i.e. almost immediately. The campaign CLI in CLAUDE.md
#       already pins ch-4, and Stage E's action is wave-only, so this costs the current campaign
#       nothing -- but the flag is now load-bearing for NOT CRASHING, not just for isolating an axis.
#     - action_mode "body_21d" (_apply_body_21d_action, ch-20 -> body_kp_scale) is UNRUNNABLE for the
#       same reason: a policy will never emit exactly 0.0 there. It is legacy and was already not the
#       training mode.
#
# WHAT A FUTURE RL RUN MUST DO INSTEAD (if it wants to modulate impedance)
#   Modulate the TABLE, not a scale: env.set_joint_impedance({family: (Kp, Kv)}) in physical N*m/rad,
#   families = env.s2_family_ids() (8 families = all 211 DOF). Per-family values may be a scalar (all
#   envs) or a length-E vector (per-env), so a per-env sweep works; a missing/unknown family raises.
#   Torque caps: env.set_joint_effort_limit({family: tau_max_Nm | None}) -- family-exact; the
#   controller's old effort clamp is gone (under implicit it clamped a `tau` the solver discards).
#   NOTE this is a PER-CALL (host-side) entry, not a per-step action channel: re-wiring a policy to
#   drive it is a real design task, not a flag. That is the honest cost of the new architecture.
#
# WHERE THE STORY IS WRITTEN DOWN
#   docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md   (what changed + how to migrate)
#   outputs/CURRENT_RESULT.md                    (the S2 result marker + its retractions/scope limits)
#   DISPATCH_LOG.md 2026-07-16T19:34Z -> 22:05Z  (PI sign-off, S1/S2a/S2b execution records)
#   SOURCE: env.set_joint_impedance / env.set_joint_effort_limit / env._apply_action_implicit_pd
#           / env._S2_DEAD_SCALES / env._S2_DELETED_MEMBERS ;
#           controllers/impedance_controller_torch.py::_assert_dead_gain_scale
# ═════════════════════════════════════════════════════════════════════════════════════════════

# =============================================================================
# Pass 14 (2026-05-17): obs-v2 windowed terrain-severity feature constants
# =============================================================================
# Pooled transition-arena normalization stats (mean, std) per feature, in
# the order: acc_fwd_rms_w, F_rate_p99_w, F_max_p95_w, contact_entropy_std_w.
# Source: outputs/analysis/extended_sensor_obs_packet_prep_2026-05-17/
# data/T5_final_normalization.csv (pooled across 3 seeds × N=16 + 1 N=32
# probes on the actual RL transition arena under fixed all-1.0 controller;
# total 9600 samples per feature).
_PASS14_OBS_V2_MEANS = (0.2592, 0.3514, 0.01568, 0.06335)
_PASS14_OBS_V2_STDS  = (0.1148, 0.5418, 0.00257, 0.05321)
_PASS14_OBS_V2_CLIP_SIGMAS = 3.0
_PASS14_OBS_V2_N_FEATURES  = 4

# =============================================================================
# Pass 17 (2026-05-19): K=12 sensor → severity-belief estimator constants
# =============================================================================
# All 12 values verbatim from
#   outputs/analysis/overnight_estimator_pass16_forensic_2026-05-19/
#   data/k12_deploy.json
# Feature order MUST match _K12_FEATURE_NAMES ordering (z-score and ridge
# coefs are aligned to that order). Window size W = 20 policy steps
# (= 0.96 s at dt_pol = 0.048 s). Ridge:
#   R_belief_raw = intercept + sum_k coef_z[k] * (feat[k] - mu[k]) / sigma[k]
# then R_norm = clip((R_raw - R_LOW)/(R_HIGH - R_LOW), 0, 1)
# then R_belief = smoothstep(R_norm) = R_norm² (3 - 2 R_norm) ∈ [0, 1]
_K12_WINDOW_W              = 20
_K12_INTERCEPT             = 0.18578775227069855
_K12_R_LOW                 = 0.112
_K12_R_HIGH                = 0.243
_PASS17_F_THRESHOLD_N      = 1.0e-4    # contact binary threshold (Newtons)
_PASS17_N_FOOT             = 38        # 19 segments × 2 sides
_PASS17_N_SEG_BODY         = 21        # link_body_0 .. link_body_20

_K12_FEATURE_NAMES = (
    "contact_imbalance_mean_w20",      # 0
    "duty_mean_w20",                   # 1
    "air_time_fraction_w20",           # 2
    "n_active_mean_w20",               # 3
    "duty_std_w20",                    # 4
    "F_argmax_frac_w20",               # 5
    "F_link_p95_w20",                  # 6
    "contact_switch_rate_w20",         # 7
    "mean_contact_dur_w20",            # 8
    "seg_pitch_spatial_var_mean_w20",  # 9
    "contact_dur_p95_w20",             # 10
    "roll_var_w20",                    # 11
)
_K12_FEATURE_MU = (
    +2.78419, +0.03539, +0.96461, +1.34475, +0.07365, +0.43684,
    +0.00336, +0.03166, +0.00595, +0.01702, +0.02013, +1.72609,
)
_K12_FEATURE_SIGMA = (
    +1.48228, +0.02478, +0.02478, +0.94157, +0.06102, +0.31806,
    +0.00301, +0.02498, +0.00975, +0.02758, +0.05114, +2.90813,
)
_K12_FEATURE_COEFZ = (
    +0.03809, +0.02868, -0.02864, +0.02799, -0.03164, +0.00616,
    -0.02424, -0.05322, -0.00424, +0.00728, +0.00040, +0.01345,
)
assert len(_K12_FEATURE_NAMES) == 12
assert len(_K12_FEATURE_MU)    == 12
assert len(_K12_FEATURE_SIGMA) == 12
assert len(_K12_FEATURE_COEFZ) == 12

# =============================================================================
# P4 v3.4 Round A (2026-05-31): PGZ severity-belief constants (proj_g_z-only)
# =============================================================================
# Offline OLS of spawn-block severity on the SINGLE sensor feature proj_g_z over
# the 4 k12_feature_sweep traces (21,248 alive rows); Pearson r=+0.4267 (matches
# outputs/analysis/k12_feature_sweep_2026-05-21/RANKING.md; proj_g_z LOSO +0.4153).
# Provenance + fit: scripts/sanity/p4_v34_roundA_fit_pgz_belief_2026-05-31.py and
# outputs/analysis/k12_feature_sweep_2026-05-21/pgz_deploy.json.
# Deploy form (parallel to K=12), RUNTIME INPUT = online-sensed proj_g_z ONLY:
#   R_raw  = _PGZ_OLS_A + _PGZ_OLS_B * proj_g_z          (b>0 => monotone increasing)
#   R_norm = clip((R_raw - _PGZ_R_LOW)/(_PGZ_R_HIGH - _PGZ_R_LOW), 0, 1)
#   R_belief_pgz = R_norm^2 (3 - 2 R_norm)  in [0, 1]
# No terrain ground-truth at runtime (Constraint 2 is a runtime rule; identical
# legality posture to K=12, likewise fitted offline against severity).
_PGZ_OLS_A   = 8.09050200
_PGZ_OLS_B   = 7.87771769
_PGZ_R_LOW   = 0.21323118
_PGZ_R_HIGH  = 0.35598686

# =============================================================================
# P4 v3.4 Round B (2026-05-31): belief-conditioned attitude-var robustness reward
# =============================================================================
# r_20 = -(W_FRAC * pass16_attitude_var_weight) * (1 + KAPPA*R_belief)
#        * attitude_var_w * gate
#      = -0.125 * (1 + 0.5*R_belief) * attitude_var_w * gate   (pass16 weight 0.5)
# Reuses the pass16 windowed attitude variance (_attitude_var_w) + its gate +
# the K=12 sensor belief (_R_belief). NO wave/applied_abs/force/speed in this
# term — strict-emergent: the reward never mentions wave. Sensor-only.
_PASS20_KAPPA  = 0.5    # belief gain: w_belief = 1.0 + KAPPA * R_belief  in [1.0, 1.5]
_PASS20_W_FRAC = 0.25   # r_20 coef = W_FRAC * pass16_attitude_var_weight  (=0.125 @ 0.5)


def _quat_wxyz_to_roll_pitch(q: torch.Tensor):
    """Quaternion (..., 4) in wxyz convention → (roll, pitch) each (...,).
    Mirror of the trace collector's helper at
    scripts/sanity/collect_extended_trace_on_transition.py:quat_wxyz_to_roll_pitch.
    """
    w = q[..., 0]; x = q[..., 1]; y = q[..., 2]; z = q[..., 3]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)
    sinp = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0)
    pitch = torch.asin(sinp)
    return roll, pitch

# =============================================================================
# Stage B-v0 (Pass 2, 2026-05-07): 86-D contact-rich observation constants
# =============================================================================
_N_SEG = 19          # number of body segments / leg pairs / foot pairs
_F_SCALE = 0.05      # contact force normalisation [N] (mg-scale robot)
_F_EPS = 1e-6        # safe denom for ratio features

# --- Stage E (2026-07-12) windowed-terrain-obs (WTO) layout ---
_NWO_SELF_DIM = 9     # realized_wave(1)+cpg sin/cos(2)+root_ang_vel_b(3)+proj_g(3)
_NWO_WIN_DIM  = 42    # 19 contact_std + 19 contact_spike + omega{crest,spike}(2) + total{std,spike}(2)
_NWO_OBS_DIM  = 51    # _NWO_SELF_DIM + _NWO_WIN_DIM  (REPLACES the 86-D obs when new_windowed_obs)
_NWO_SPIKE_K  = 3.0   # spike_rate threshold = K * per-window median
_TOPO_OBS_DIM = 34    # wave/phase(3)+root lin/ang/gravity(9)+contact DCT(10)+spine spectrum(12)
_TOPO_MAIN_N  = 21    # link_body_0..link_body_20
_TOPO_ROOT_ANG_SCALE = 0.25


def _orthonormal_dct2_basis(n: int, k: int, device) -> torch.Tensor:
    """Return the first k orthonormal DCT-II rows as a (k,n) tensor."""
    rows = []
    for mode in range(k):
        scale = math.sqrt(1.0 / n) if mode == 0 else math.sqrt(2.0 / n)
        rows.append([
            scale * math.cos(math.pi * (sample + 0.5) * mode / n)
            for sample in range(n)
        ])
    return torch.tensor(rows, dtype=torch.float32, device=device)


def _quat_wxyz_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate batched vectors from world into the frame represented by q."""
    q = q / q.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    q_vec = q[:, None, 1:]
    q_w = q[:, None, :1]
    twice_cross = 2.0 * torch.cross(q_vec.expand_as(v), v, dim=-1)
    return v - q_w * twice_cross + torch.cross(
        q_vec.expand_as(v), twice_cross, dim=-1)

# Side-imbalance compression bins (segment ranges, end-exclusive).
# Sums to 19 segments. The middle bin is wider (3) to absorb the trunk.
_SIDE_ASYM_BINS = (
    (0, 2),    # bin 0: seg 0-1
    (2, 4),    # bin 1: seg 2-3
    (4, 6),    # bin 2: seg 4-5
    (6, 8),    # bin 3: seg 6-7
    (8, 11),   # bin 4: seg 8-10  (wider trunk bin)
    (11, 13),  # bin 5: seg 11-12
    (13, 15),  # bin 6: seg 13-14
    (15, 17),  # bin 7: seg 15-16
    (17, 19),  # bin 8: seg 17-18
)
# Front-rear cumulative split points (end-exclusive front index).
_FRONT_REAR_SPLITS = (5, 9, 11, 15)
# Local 4-region groupings A/B/C/D (end-exclusive).
_LOCAL_REGIONS = ((0, 5), (5, 10), (10, 15), (15, 19))
# Global-summary regions (end-exclusive).
_GLOBAL_FRONT  = (0, 6)
_GLOBAL_MID    = (6, 13)
_GLOBAL_REAR   = (13, 19)
# Front/rear regions for the global side-asym diff (segments 0-5 vs 13-18).
_GLOBAL_SIDE_FRONT = (0, 6)
_GLOBAL_SIDE_REAR  = (13, 19)


def _bounded_scale(action: torch.Tensor, lo: float, hi: float) -> torch.Tensor:
    """Map raw action ∈ ℝ -> scale ∈ [lo, hi] with action=0 -> 1.0 exactly.

    Requires lo ≤ 1.0 ≤ hi. Uses tanh for smooth saturation, with
    asymmetric half-widths (1-lo, hi-1) so an asymmetric range like
    [0.7, 1.5] still has its midpoint at 1.0 (not 1.1).

    Properties:
      - action = 0       -> scale = 1.0 exactly (tanh(0) = 0)
      - action -> +inf   -> scale -> hi
      - action -> -inf   -> scale -> lo
      - smooth + bounded everywhere
    """
    t = torch.tanh(action)
    neg_half = 1.0 - lo
    pos_half = hi - 1.0
    return 1.0 + torch.where(t >= 0, pos_half * t, neg_half * t)


# --------------------------------------------------------------------------------------------------
# _S2_DEAD_ACTION_CH -- which RAW action index is structurally dead, per action_mode (2026-07-16).
# The single source of truth for the r_act dead-channel exclusion
# (cfg.exclude_dead_action_channels_from_r_act). Full story: _S2_DEAD_ACTION_NOTE at the top.
#   global_6d ch-4  = body_imp_scale  -> controller.body_kp_scale_runtime  (DELETED by S2b)
#   body_21d  ch-20 = body_kp_scale   -> same dead wire (this mode is UNRUNNABLE anyway)
# Indexed (not .get()) on purpose: an unknown action_mode must KeyError, never silently tax nothing.
_S2_DEAD_ACTION_CH = {
    "global_6d": (4,),
    "body_21d":  (20,),
}

# --------------------------------------------------------------------------------------------------
# S1/S2a TOMBSTONES (2026-07-16). These members were DELETED by the implicit-only unification
# (PI-signed; DISPATCH_LOG 2026-07-16T19:34Z). Listed here ONLY so a stale caller gets a DIRECTED
# error instead of a bare AttributeError -- NOTHING here restores any mechanism or any physics.
#
# Why the tombstone raises RuntimeError and not AttributeError: `getattr(env, "_suppress_active_pitch",
# None)` SILENTLY returns None when the attribute is merely absent -- that exact call printed
# "suppress=None" in scripts/analysis/stage1_droop_harness_2026-07-05.py:125 and would have mislabelled
# a run rather than stopping it. getattr()'s default only swallows AttributeError, so a RuntimeError
# still propagates. (Module-level, not class-level, so __getattr__ can never recurse into itself.)
_S2_DELETED_MEMBERS = {
    "enable_passive_spring":   "the passive structural spring (the K = Kp + k_p / weighted-target FOLD, rest = q0)",
    "suppress_active_pitch":   "the active-pitch zeroing + keep_head_up harness",
    "_passive_spring_torque":  "the explicit passive-spring torque law tau = k(q0-q) - c*qd",
    "_suppress_active_pitch":  "the suppress_active_pitch state flag",
    "_suppress_keep_head_up":  "the keep_head_up state flag",
    "_passive_spring_enabled": "the passive-spring state flag",
    "_passive_k":              "the passive-spring stiffness buffer",
    "_passive_c":              "the passive-spring damping buffer",
    "_passive_q0":             "the passive-spring rest pose",
}


class CentipedeEnv(DirectRLEnv):
    """Centipede Layer 2 RL env, Stage A skeleton."""

    cfg: CentipedeEnvCfg

    # =========================================================================
    # Construction & scene
    # =========================================================================

    def __init__(self, cfg: CentipedeEnvCfg, render_mode: str | None = None, **kwargs):
        # Stage B-v0: assert action_mode <-> action_space consistency BEFORE
        # super().__init__ so the gym space is built with a sane shape.
        _expected = {"global_6d": 6, "body_21d": 21}
        _mode = getattr(cfg, "action_mode", "global_6d")
        if _mode not in _expected:
            raise ValueError(
                f"cfg.action_mode = {_mode!r} not in {tuple(_expected)}"
            )
        if cfg.action_space != _expected[_mode]:
            raise ValueError(
                f"cfg.action_space ({cfg.action_space}) does not match "
                f"cfg.action_mode {_mode!r} (expected {_expected[_mode]}). "
                "Set both consistently before constructing CentipedeEnv."
            )
        super().__init__(cfg, render_mode, **kwargs)

        # Controller (torch-vectorized).
        # Use float32 to match Isaac Lab tensors and GPU performance.
        self.controller = ImpedanceTravelingWaveControllerTorch(
            self.robot,
            CONTROLLER_CONFIG_PATH,
            num_envs=self.num_envs,
            device=self.device,
            dtype=torch.float32,
        )

        # 2026-05-13: propagate fast wave-number retargeting flag from
        # cfg → controller. Default OFF — `_apply_phase_rebase` is
        # never called when the flag is False, so the controller path
        # is identical to the pre-2026-05-13 baseline (see
        # outputs/analysis/cpg_fast_wavenumber_retargeting_design.md).
        # getattr with a default keeps older env_cfg objects (without
        # these fields) loading cleanly.
        self.controller.set_fast_retargeting(
            enabled=bool(getattr(cfg, "fast_wave_retargeting_enabled", False)),
            max_rate_per_s=float(
                getattr(cfg, "fast_wave_retargeting_max_rate_per_s", 1.0)
            ),
        )

        # 2026-05-13: FARMS-equivalent CPG optimisation overrides.
        # Default values (1.0, 1.0, 1, "jacobi") reproduce the existing
        # controller behaviour bit-for-bit. See env_cfg comment block
        # and outputs/analysis/farms_equivalent_cpg_optimization/.
        self.controller.set_cpg_overrides(
            body_coupling_scale=float(
                getattr(cfg, "cpg_body_coupling_scale", 1.0)
            ),
            leg_coupling_scale=float(
                getattr(cfg, "cpg_leg_coupling_scale", 1.0)
            ),
            substeps=int(getattr(cfg, "cpg_substeps", 1)),
            update_mode=str(getattr(cfg, "cpg_update_mode", "jacobi")),
        )

        # ── RTX renderer warmup for video capture ──────────────────
        # DirectRLEnv.render() returns np.zeros for the first ~20
        # frames while the RTX pipeline initialises (see direct_rl_env.py:
        # 'initially the renderer is warming up and returns empty data').
        # When wrapped by gymnasium's RecordVideo, those zeros become
        # black frames in the MP4. Pre-tick the renderer here, gated on
        # render_mode so this cost is zero for non-video runs.
        if render_mode == "rgb_array":
            n_warmup = 30
            print(
                f"[CentipedeEnv] warming up RTX renderer ({n_warmup} frames) "
                "before exposing env to video recorder ...",
                flush=True,
            )
            for _ in range(n_warmup):
                self.sim.render()
            print("[CentipedeEnv] renderer warmup complete.", flush=True)

        # Action buffer held between policy decimation calls.
        self.actions = torch.zeros(
            (self.num_envs, self.cfg.action_space),
            dtype=torch.float32,
            device=self.device,
        )
        # Stage B Step 1: previous-action cache for future action-rate
        # penalty / smoothness reward (not used in Stage B Step 1 itself,
        # but reserved here so _pre_physics_step can read it on the very
        # first call without an AttributeError).
        self._prev_actions = torch.zeros_like(self.actions)

        # GO_FLING_CAUSE_DEFINITIVE_2026-06-29 §2: physics-substep contact-impulse logging.
        # Default OFF. enable_substep_impulse_dump() is NEVER called on a normal run, so the
        # scene.update wrapper is never installed and the _pre_physics_step reset is a no-op
        # ⇒ the runtime path is byte-identical to the pre-edit env (flag-OFF bit-identity proof).
        self._dump_substep_impulse = False

        # S2a (PI-signed 2026-07-16: "不要有所谓的被动弹簧" / "一个关节只留一组kp kv作为PD gains，其他全去掉"):
        # the PASSIVE structural spring and the suppress_active_pitch droop harness are DELETED. Every joint's
        # impedance is now exactly ONE (Kp, Kv) from the S2 table, written straight to PhysX. There is NO default
        # and NO fallback: set_joint_impedance() must be called before stepping, or _apply_action raises.
        self._s2_Kp = None
        self._s2_Kv = None
        self._s2_table = None
        self._s2_active_ids = None
        self._s2_effort = None

        # P4 v3.4 windowed-speed reward (2026-06-03): per-env trailing v_forward circular buffer.
        # Default OFF ⇒ never allocated/used ⇒ byte-identical. ON ⇒ r_v uses the trailing W-mean.
        if bool(getattr(self.cfg, "windowed_vfwd_reward_enabled", False)):
            self._vfwd_W = int(getattr(self.cfg, "vfwd_window_steps", 80))
            self._vfwd_buf = torch.zeros(self.num_envs, self._vfwd_W, dtype=torch.float32, device=self.device)
            self._vfwd_cnt = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
            self._vfwd_pos = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
            self._vfwd_last_windowed = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
            print(f"[windowed-vfwd] ENABLED (W={self._vfwd_W}): r_v uses trailing {self._vfwd_W}-step "
                  f"MEAN of v_forward (signed, dense, per-env, cleared on reset).", flush=True)

        # Stage B-v0 Pass 6 (2026-05-07): per-env SCALED-PARAMETER history
        # for the physically-meaningful action-rate penalty. The reward
        # term r_rate is now:
        #   rate_term = ||u_t - u_{t-1}||^2
        # where u is the 6-D scaled-physical-knob vector
        #   [body_amp, leg_yaw_amp, leg_pitch_amp, wave_num, body_imp, freq]
        # read from the controller's runtime fields. At baseline (zero
        # action) every runtime scale equals 1.0, so initialising the
        # history to ones makes rate_term=0 at the first post-reset
        # step. The buffer is reset to 1.0 in _reset_idx for partial-
        # reset correctness (mirroring controller.reset_envs which also
        # resets all runtime scales to 1.0).
        self._prev_scaled_action = torch.ones(
            (self.num_envs, 6), dtype=torch.float32, device=self.device
        )

        # Stage B-v0 Pass 8 (2026-05-08, audit-driven): first-order
        # low-pass on the 6-D global-gait target parameters.
        #
        # The latest-policy and CPG-continuity audits showed that the
        # Pass 6 trained policy commands u_target with mean per-step
        # rate |du|/step ≈ 0.13–0.65 PER DIM (largest = body_imp at
        # 0.65 on its [0.30, 3.00] range = 24% step-jump). Zero-action
        # gives 0 step-rate. The discontinuity propagates through the
        # body-yaw target equation
        #   q_ref ∝ body_amp_scale_runtime * sin(phases)
        # and (HISTORICAL — this PD law lived in the controller and was DELETED by S2b
        # 2026-07-16; gains now come from the env impedance table and PhysX computes the
        # force. Kept only to explain the Pass-6 measurement below, which predates S2.)
        #   τ ∝ body_kp_scale_runtime * (q_ref - q) + body_kv_scale_runtime * (...)
        # producing applied_torque mean +1544% over zero-action and
        # |omega_b| mean +98%. Per-block u_target ranges are 0.05–0.19,
        # 3–13× SMALLER than per-step std, so the policy is currently
        # noise-around-a-mean rather than terrain-conditioned feedback.
        #
        # First-order low-pass (single shared tau across all 6 dims):
        #   u_applied_{t+1} = u_applied_t + alpha * (u_target_t - u_applied_t)
        #   alpha = 1 - exp(-dt_pol / tau_u)
        # tau_u = 0.30 s gives alpha ≈ 0.148 at dt_pol = 0.048 s. The
        # policy still commands u_target each step; the controller
        # consumes u_applied. Per-step rate of u_applied shrinks by ~7×
        # (body_imp 0.65 → 0.096), which:
        #   (a) brings cross-step variation below cross-block variation,
        #       leaving room for terrain-conditioned modulation to emerge
        #   (b) shrinks q_ref / qdot_ref / torque step-jumps in the same
        #       proportion (linear in the leading factor), reducing the
        #       visible jerk and the +1544% torque excess
        #   (c) preserves global_6d action space, observation layout,
        #       reward formula, action ranges, controller equations,
        #       and YAML gains UNCHANGED
        #
        # Rationale for tau choice (full justification in the audit-driven
        # write-up): 0.30 s ≈ 30% of the 1 Hz baseline gait period;
        # 95% settling = 3·tau = 0.9 s ≈ one cycle. Tighter (≤0.1 s)
        # gives only 2.6× rate reduction; looser (≥0.5 s) makes
        # adaptation slower than half a gait cycle.
        #
        # Reward rate-term: kept in u_applied (= what's read from
        # runtime fields). Smoothing automatically shrinks the rate-term
        # ≈50×, removing it from the dominant-penalty list and letting
        # the policy explore. We deliberately do NOT add a u_target
        # rate penalty: bounds_loss + learnable sigma already constrain
        # raw-action noise.
        dt_pol = float(self.cfg.sim.dt) * int(self.cfg.decimation)
        self._u_smooth_tau_s = 0.30
        self._u_smooth_alpha = 1.0 - math.exp(
            -dt_pol / max(self._u_smooth_tau_s, 1e-6)
        )
        self._u_applied = torch.ones(
            (self.num_envs, 6), dtype=torch.float32, device=self.device
        )
        # u_target snapshot (last commanded target, pre-smoothing).
        # Logged for analysis; not consumed by controller.
        self._u_target = torch.ones(
            (self.num_envs, 6), dtype=torch.float32, device=self.device
        )

        # P4 interface-smoothing (2026-06-01, behind default-OFF
        # cfg.wave_smooth_interface_enabled). When ON, the WAVE channel
        # (idx 3) is re-smoothed with a slower EMA (tau=1.0 s) + a per-step
        # rate limiter, in applied_abs units (scale * base), exactly mirroring
        # the eval-time WaveSmoother in
        # scripts/sanity/p4_v34_interface_smoothing_eval_2026-06-01.py. The
        # five non-wave channels keep the tau=0.30 EMA above. Buffers are
        # allocated unconditionally with torch.full (NO rng draw), so the
        # flag-OFF path stays byte-identical to Pass 8.
        self._wave_smooth_base = 3.0
        self._wave_smooth_tau_s = 1.0
        self._wave_smooth_alpha = 1.0 - math.exp(
            -dt_pol / max(self._wave_smooth_tau_s, 1e-6)
        )
        self._wave_smooth_max_dabs_per_s = 0.5
        self._wave_smooth_max_dabs_abs = self._wave_smooth_max_dabs_per_s * dt_pol
        self._wave_ema_abs = torch.full(
            (self.num_envs,), self._wave_smooth_base,
            dtype=torch.float32, device=self.device,
        )
        self._wave_abs = torch.full(
            (self.num_envs,), self._wave_smooth_base,
            dtype=torch.float32, device=self.device,
        )

        # Per-env elapsed sim time tensor (used for episode timeouts / dones).
        self._sim_time = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        # Host-side global sim-time scalar for the controller. Avoids a
        # GPU->CPU sync per physics step. Per-env time-since-reset is a
        # Stage B concern; for Stage A all envs share the same clock.
        self._t_host = 0.0
        # Diagnostic: accumulated wall-clock time spent inside the
        # controller during the current policy step. The smoke script
        # reads and resets this between policy steps. Cheap (one float
        # add per physics step), no impact on training.
        self._last_controller_dt_total = 0.0

        # ── Stage B-v0 Pass 3 refined (2026-05-07): random-block spawn state ──
        # Per-env spawn block ID (which of the four terrain blocks the
        # robot started in). Stored for DIAGNOSTICS ONLY — never fed
        # into the policy observation. Initialised to -1; populated at
        # each call to _reset_idx in transition mode.
        self._spawn_block_id = torch.full(
            (self.num_envs,), -1, dtype=torch.long, device=self.device
        )
        # Host-side numpy RNG for spawn sampling. Cheap (one-shot per
        # reset, not per-step) and gives reproducible spawns when seeded.
        # We deliberately use a separate seed from the terrain seed so
        # that one can change spawn distribution without re-baking the
        # heightmap. Note: transition_spawn_seed lives on cfg (env cfg),
        # not cfg.scene — see CentipedeEnvCfg comment for why.
        import numpy as _np
        self._spawn_rng = _np.random.default_rng(
            int(getattr(cfg, "transition_spawn_seed", 1234))
        )

        # ── Stage B-v0 (Pass 2, 2026-05-07): 86-D observation state ──
        # Per-env per-segment contact load history for the temporal-
        # derivative slot of the observation. Reset to zero in
        # _reset_idx for the listed envs to prevent spurious spikes
        # right after a reset.
        self._previous_F_sum = torch.zeros(
            (self.num_envs, _N_SEG), dtype=torch.float32, device=self.device
        )
        # Lazy-init handles for foot index resolution. Populated on the
        # first call to _get_observations() after the contact sensor's
        # body_names are available (post-init). Holds:
        #   self._foot_idx_grid: (19, 2) long, sensor index per (seg, side)
        # The (seg, side) ordering is anatomic (foot_<seg>_<side> by name),
        # NOT raw articulation order — see project memory entry on
        # USD-flat-hierarchy.
        self._foot_idx_grid = None
        # Body-yaw bin slices (anatomical joint_body_1..joint_body_19) for
        # the compressed q/qdot observation. These reference indices INTO
        # controller.body_joint_ids (which is exact-name-resolved):
        #   bin0: joint_body_1..4   -> body_joint_ids[0:4]   (4 joints)
        #   bin1: joint_body_5..9   -> body_joint_ids[4:9]   (5 joints)
        #   bin2: joint_body_10..14 -> body_joint_ids[9:14]  (5 joints)
        #   bin3: joint_body_15..19 -> body_joint_ids[14:19] (5 joints)
        self._body_yaw_bins = ((0, 4), (4, 9), (9, 14), (14, 19))

        # --- Stage E (2026-07-12): NEW windowed-terrain observation (WTO) ---
        # DEFAULT OFF (cfg.new_windowed_obs=False): every WTO path below is gated on
        # that flag and never runs, so the OFF obs is byte-identical to the pre-edit
        # 86-D obs (fingerprint-reproduced). When ON, the policy obs is the ~51-D
        # self-state + WINDOWED-terrain vector that REPLACES the 86-D obs (caller sets
        # cfg.observation_space=51, or +oracle_terrain_obs_dim). LEGAL ONLY: every
        # channel is a proprioceptive / contact SELF-signal (no severity/block/x/lambda/amp/LUT).
        # The sensing window is a FIXED WALL-CLOCK ring of the per-physics-substep signals
        # (windowed_obs_window_seconds, default 1.0s), DECOUPLED from the decimation/update
        # rate: at decim 250 the ring == one policy interval, but the knob lets us hold the
        # window fixed while sweeping decim (isolate rate) or vice-versa.
        _old_wto = bool(getattr(cfg, "new_windowed_obs", False))
        _topo_v2 = bool(getattr(cfg, "topology_obs_v2_enabled", False))
        if _old_wto and _topo_v2:
            raise ValueError(
                "new_windowed_obs and topology_obs_v2_enabled are mutually exclusive")
        self._nwo_enabled = _old_wto or _topo_v2
        if self._nwo_enabled:
            _win_s = float(getattr(cfg, "windowed_obs_window_seconds", 1.0))
            self._nwo_W = max(1, int(round(_win_s / float(self.cfg.sim.dt))))
            self._nwo_sensor = None
            self._nwo_cbuf = torch.zeros(
                (self.num_envs, self._nwo_W, _N_SEG), dtype=torch.float32, device=self.device)
            self._nwo_obuf = torch.zeros(
                (self.num_envs, self._nwo_W), dtype=torch.float32, device=self.device)
            self._nwo_tbuf = torch.zeros(
                (self.num_envs, self._nwo_W), dtype=torch.float32, device=self.device)
            self._nwo_pos = torch.zeros(
                (self.num_envs,), dtype=torch.long, device=self.device)
            self._nwo_filled = torch.zeros(
                (self.num_envs,), dtype=torch.long, device=self.device)
            if _topo_v2:
                self._topo_main_body_ids = None
                self._topo_dct19 = _orthonormal_dct2_basis(
                    _N_SEG, 3, self.device)
                self._topo_dct21 = _orthonormal_dct2_basis(
                    _TOPO_MAIN_N, 4, self.device)
            _obs_dim = _TOPO_OBS_DIM if _topo_v2 else _NWO_OBS_DIM
            _label = "TOPOLOGY OBS V2" if _topo_v2 else "Stage-E WTO"
            print(f"[{_label}] windowed observation ENABLED: window={_win_s:.3f}s "
                  f"(W={self._nwo_W} substeps @ dt={float(self.cfg.sim.dt)*1e3:.1f}ms), "
                  f"obs={_obs_dim}-D REPLACES 86-D (window decoupled from decimation).", flush=True)
        # Stage E oracle-terrain DIAGNOSTIC channel (DO_NOT_SHIP; default OFF).
        # When ON, the ground-truth terrain descriptor (set by the caller via
        # set_oracle_terrain_descriptor) is APPENDED to the obs -- diagnostic UPPER BOUND
        # only: NEVER shipped, NEVER warm-started from (CLAUDE.md forbidden list).
        self._oracle_enabled = bool(getattr(cfg, "oracle_terrain_obs", False))
        self._oracle_dim = int(getattr(cfg, "oracle_terrain_obs_dim", 3))
        if self._oracle_enabled:
            self._oracle_terrain_descriptor = torch.zeros(
                (self.num_envs, self._oracle_dim), dtype=torch.float32, device=self.device)
            print(f"[Stage-E ORACLE] DO_NOT_SHIP oracle_terrain_obs ENABLED: appends "
                  f"{self._oracle_dim}-D ground-truth terrain descriptor (diagnostic upper bound).", flush=True)

        # ── Pass 14 (2026-05-17): obs-v2 windowed severity buffers ──
        # Pre-allocate the 5-feature ring buffer + per-env state
        # regardless of the cfg flag. When pass14_obs_v2_enabled=False
        # the update path is gated off in _get_observations so these
        # tensors remain zeros and do not affect any computation.
        # Total memory: 5 buffers × num_envs × W × 4 bytes (e.g. 16×30
        # = 9.6 KB at N=16, W=30) — negligible. Pre-allocating keeps
        # _get_observations branch-free on the hot path.
        self._pass14_W = int(getattr(cfg, "pass14_obs_v2_window_steps", 30))
        self._pass14_acc_fwd_buf = torch.zeros(
            (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
        self._pass14_acc_z_buf   = torch.zeros(
            (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
        self._pass14_F_rate_buf  = torch.zeros(
            (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
        self._pass14_F_max_buf   = torch.zeros(
            (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
        self._pass14_entropy_buf = torch.zeros(
            (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
        self._pass14_buf_idx     = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)
        self._pass14_buf_filled  = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)
        self._pass14_prev_v_fwd  = torch.zeros(
            (self.num_envs,), dtype=torch.float32, device=self.device)
        self._pass14_prev_v_z    = torch.zeros(
            (self.num_envs,), dtype=torch.float32, device=self.device)
        self._pass14_prev_F_total = torch.zeros(
            (self.num_envs,), dtype=torch.float32, device=self.device)
        self._pass14_has_prev    = torch.zeros(
            (self.num_envs,), dtype=torch.bool, device=self.device)
        # Stashes for _get_rewards to log via extras["log"]; updated
        # only when the flag is ON (when OFF they remain zeros).
        self._pass14_last_raw    = torch.zeros(
            (self.num_envs, 5), dtype=torch.float32, device=self.device)
        self._pass14_last_z      = torch.zeros(
            (self.num_envs, 4), dtype=torch.float32, device=self.device)

        # ── Pass 16 (2026-05-19) — overnight_adaptive_wave_campaign ──
        # Windowed buffers for attitude-variance + per-link force-peak.
        # Pre-allocated regardless of flag (zero RAM cost when OFF; ~10 KB
        # at N=96 W=30); the update path in _get_rewards is gated.
        self._p16_attitude_W = int(getattr(cfg, "pass16_attitude_window_steps", 30))
        self._p16_force_W    = int(getattr(cfg, "pass16_force_window_steps", 30))
        self._p16_roll_buf  = torch.zeros(
            (self.num_envs, self._p16_attitude_W),
            dtype=torch.float32, device=self.device)
        self._p16_pitch_buf = torch.zeros(
            (self.num_envs, self._p16_attitude_W),
            dtype=torch.float32, device=self.device)
        self._p16_att_idx   = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)
        self._p16_att_filled = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)
        self._p16_fmax_buf  = torch.zeros(
            (self.num_envs, self._p16_force_W),
            dtype=torch.float32, device=self.device)
        self._p16_fmax_idx  = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)
        self._p16_fmax_filled = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)

        # ── P4 v3.4 Round C — C0 impact buffer (2026-05-31) ──
        # rC0-OWNED windowed F_max ring buffer (does NOT reuse Pass14/16 buffers
        # — those are pass-flag-gated). Allocated ONLY when rC0_impact_enabled, so
        # when OFF there is zero new state (flag-OFF bit-identity). Filled every
        # step from s["F_max"]; force_raw_Fmax = MEAN over the filled window (B2).
        if bool(getattr(cfg, "rC0_impact_enabled", False)):
            self._rC0_fmax_W = int(getattr(cfg, "rC0_fmax_window_steps", 30))
            self._rC0_fmax_buf = torch.zeros(
                (self.num_envs, self._rC0_fmax_W),
                dtype=torch.float32, device=self.device)
            self._rC0_fmax_idx = torch.zeros(
                (self.num_envs,), dtype=torch.long, device=self.device)
            self._rC0_fmax_filled = torch.zeros(
                (self.num_envs,), dtype=torch.long, device=self.device)

        # ── Pass 17 (2026-05-19) — K=12 sensor → severity belief ──
        # Pre-allocated regardless of flag. Memory cost: 96 envs × 20 W × 38
        # feet × 4 B × 2 buffers ≈ 583 KB. Plus 5 scalar buffers
        # (E × W × 4 B × 5) ≈ 38 KB. Tiny.
        self._k12_W = int(_K12_WINDOW_W)
        # Rolling buffers (E, W, ...) updated each policy step in
        # `_compute_k12_R_belief`.
        self._k12_F_foot_buf = torch.zeros(
            (self.num_envs, self._k12_W, _PASS17_N_FOOT),
            dtype=torch.float32, device=self.device)
        self._k12_contact_buf = torch.zeros(
            (self.num_envs, self._k12_W, _PASS17_N_FOOT),
            dtype=torch.float32, device=self.device)
        self._k12_ci_buf = torch.zeros(
            (self.num_envs, self._k12_W),
            dtype=torch.float32, device=self.device)
        self._k12_roll_a_buf = torch.zeros(
            (self.num_envs, self._k12_W),
            dtype=torch.float32, device=self.device)
        self._k12_seg_pitch_sp_var_buf = torch.zeros(
            (self.num_envs, self._k12_W),
            dtype=torch.float32, device=self.device)
        # Per-env circular index + filled count (per Pass-16 pattern).
        self._k12_idx = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)
        self._k12_filled = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device)
        # Per-foot contact-duration persistent state (NOT in rolling buf —
        # the K=12 feature mean_contact_dur_w20 is the snapshot at t_end).
        self._k12_dur_state = torch.zeros(
            (self.num_envs, _PASS17_N_FOOT),
            dtype=torch.float32, device=self.device)
        # 21 per-segment body ids resolved at first call (sensor / art may
        # not be ready in __init__).
        self._k12_seg_body_ids = None
        # Latest computed beliefs / target (filled by _compute_k12_R_belief).
        self._R_belief     = torch.zeros(
            (self.num_envs,), dtype=torch.float32, device=self.device)
        self._R_belief_raw = torch.zeros(
            (self.num_envs,), dtype=torch.float32, device=self.device)
        self._applied_abs_target_k12 = torch.zeros(
            (self.num_envs,), dtype=torch.float32, device=self.device)
        # K=12 ridge coefs as device tensors (precomputed for hot loop).
        self._k12_mu_t    = torch.tensor(_K12_FEATURE_MU,
            dtype=torch.float32, device=self.device)
        self._k12_sigma_t = torch.tensor(_K12_FEATURE_SIGMA,
            dtype=torch.float32, device=self.device).clamp_min(1e-12)
        self._k12_coefz_t = torch.tensor(_K12_FEATURE_COEFZ,
            dtype=torch.float32, device=self.device)
        # Mutual exclusion + dependency assertions.
        _sws = bool(getattr(cfg, "pass17_severity_weighted_speed_enabled", False))
        _imit = bool(getattr(cfg, "pass18_k12_imitation_enabled", False))
        _est  = bool(getattr(cfg, "pass17_k12_estimator_enabled", False))
        # P4 v3.4 Round A (2026-05-31): PGZ is an alternative sensor-only belief.
        _pgz  = bool(getattr(cfg, "pass17_projgz_estimator_enabled", False))
        # P4 v3.4 Round B (2026-05-31): belief-robustness locks belief to K=12;
        # PGZ is circular (proj_g_z is attitude; r_20 penalizes attitude variance).
        _p20  = bool(getattr(cfg, "pass20_belief_robustness_enabled", False))
        if _p20 and _pgz:
            raise ValueError(
                "pass20_belief_robustness_enabled with pass17_projgz_estimator_enabled "
                "is circular (proj_g_z is an attitude signal; the r_20 penalty is "
                "attitude variance). Round B locks the belief to K=12 — disable "
                "pass17_projgz_estimator.")
        if _est and _pgz:
            raise ValueError(
                "pass17_k12_estimator_enabled and pass17_projgz_estimator_enabled "
                "are MUTUALLY EXCLUSIVE belief sources. Enable at most one per run "
                "(Round A A/B: exactly one belief variant per run).")
        if _sws and _imit:
            raise ValueError(
                "Pass 17 severity-weighted-speed and Pass 18 K=12 imitation "
                "are mutually exclusive. Enable at most one per run.")
        if (_sws or _imit) and not (_est or _pgz):
            raise ValueError(
                "pass17_severity_weighted_speed_enabled or "
                "pass18_k12_imitation_enabled requires a belief source: "
                "pass17_k12_estimator_enabled OR pass17_projgz_estimator_enabled "
                "must be True so R_belief is computed before either reward uses it.")
        # P4 v3.4 Round C C1 (2026-06-01): rC1 reuses the K=12 sensor belief.
        # _compute_k12_R_belief advances a persistent per-step contact-duration
        # buffer ON EACH CALL; rC1 calls it inside the C0 impact block, while the
        # legacy pass17_k12 / pass20 path calls it again later in _get_rewards.
        # Co-enabling would double-advance the buffer and corrupt the belief, so
        # rC1 is MUTUALLY EXCLUSIVE with the legacy K=12 callers (Round C uses the
        # clean rC* namespace with legacy belief OFF). rC1 also requires the rC0
        # impact term it multiplies.
        _rc1 = bool(getattr(cfg, "rC1_belief_impact_enabled", False))
        if _rc1:
            if _est or _p20:
                raise ValueError(
                    "rC1_belief_impact_enabled is MUTUALLY EXCLUSIVE with the legacy "
                    "K=12 belief callers (pass17_k12_estimator_enabled / "
                    "pass20_belief_robustness_enabled): _compute_k12_R_belief advances "
                    "a persistent buffer per call and rC1 calls it in the C0 impact "
                    "block, so co-enabling double-advances the buffer.")
            if not (bool(getattr(cfg, "roundC_enabled", False)) and
                    bool(getattr(cfg, "rC0_impact_enabled", False))):
                raise ValueError(
                    "rC1_belief_impact_enabled requires roundC_enabled AND "
                    "rC0_impact_enabled (C1 multiplies the C0 impact excess by "
                    "belief_weight; without the C0 term there is nothing to weight).")

        # ── Pass 19 (2026-05-21) — observation enrichment ──
        # Two mutually-exclusive paths:
        #   pass19_extended_trace_enabled:  log-only 6 new keys per step
        #   pass19_obs_extension_enabled:   append 155 dims to policy obs
        # Buffers pre-allocated regardless of flag (tiny RAM); update path
        # in _get_observations is gated.
        _p19_trace = bool(getattr(cfg, "pass19_extended_trace_enabled", False))
        _p19_obs   = bool(getattr(cfg, "pass19_obs_extension_enabled",  False))
        _p14_obs   = bool(getattr(cfg, "pass14_obs_v2_enabled",         False))
        if _p19_trace and _p19_obs:
            raise ValueError(
                "pass19_extended_trace_enabled and pass19_obs_extension_enabled "
                "are mutually exclusive. Enable at most one per run.")
        if _p19_obs and _p14_obs:
            raise ValueError(
                "pass19_obs_extension_enabled and pass14_obs_v2_enabled are "
                "mutually exclusive (both modify the policy observation dim). "
                "Enable at most one per run.")
        # P4-pre β-light (2026-05-29): 3 scalar substrate channels for v12 obs.
        # Requires pass19_obs_extension. Mutually exclusive with pass14_obs_v2.
        _p4_pre = bool(getattr(cfg, "p4_pre_runtime_torque_qdot_enabled", False))
        if _p4_pre and not _p19_obs:
            raise ValueError(
                "p4_pre_runtime_torque_qdot_enabled requires "
                "pass19_obs_extension_enabled = True.")
        if _p4_pre and _p14_obs:
            raise ValueError(
                "p4_pre_runtime_torque_qdot_enabled and pass14_obs_v2_enabled "
                "are mutually exclusive.")
        # Per-foot contact duration counter (frames in contact; resets on
        # contact break). Shape (E, 38). Integer count stored as float32
        # for cheap obs concatenation; clamp at 1e6 to avoid overflow.
        self._pass19_contact_dur_counter = torch.zeros(
            (self.num_envs, _PASS17_N_FOOT),
            dtype=torch.float32, device=self.device,
        )
        # proj_g_z 5-tap history for 1st/2nd finite difference. Layout:
        # history[:, 0] = oldest, history[:, 4] = newest. Filled flag
        # counts how many real samples have been pushed.
        self._pass19_proj_g_z_hist = torch.zeros(
            (self.num_envs, 5), dtype=torch.float32, device=self.device,
        )
        self._pass19_proj_g_z_filled = torch.zeros(
            (self.num_envs,), dtype=torch.long, device=self.device,
        )
        # Per-segment body ids for the 19 link_body_0..18 anatomical
        # segments (resolved lazily on first _get_observations call,
        # same pattern as _foot_idx_grid).
        self._pass19_seg_body_ids = None
        # Last-computed feature stashes for _get_rewards to log when
        # pass19_extended_trace_enabled. Persistent across the
        # _get_observations → _get_rewards transition.
        self._pass19_last_foot_force_vector = torch.zeros(
            (self.num_envs, _PASS17_N_FOOT, 3),
            dtype=torch.float32, device=self.device,
        )
        self._pass19_last_foot_force_mag = torch.zeros(
            (self.num_envs, _PASS17_N_FOOT),
            dtype=torch.float32, device=self.device,
        )
        self._pass19_last_foot_contact_state = torch.zeros(
            (self.num_envs, _PASS17_N_FOOT),
            dtype=torch.float32, device=self.device,
        )
        self._pass19_last_foot_contact_dur = torch.zeros(
            (self.num_envs, _PASS17_N_FOOT),
            dtype=torch.float32, device=self.device,
        )
        self._pass19_last_seg_pitch = torch.zeros(
            (self.num_envs, _N_SEG),
            dtype=torch.float32, device=self.device,
        )
        self._pass19_last_seg_roll = torch.zeros(
            (self.num_envs, _N_SEG),
            dtype=torch.float32, device=self.device,
        )
        # Last [proj_g_z, dproj_g_z/dt, d²proj_g_z/dt²] stash for the
        # 3-vec obs slice and for logging.
        self._pass19_last_proj_g_z_triplet = torch.zeros(
            (self.num_envs, 3),
            dtype=torch.float32, device=self.device,
        )

    def _setup_scene(self):
        """Post-spawn, pre-init scene setup.

        InteractiveScene has already spawned the centipede USD at
        ``/World/envs/env_<i>/Robot`` for each env (via the configclass
        scene cfg). Two leftover artifacts from MJCF -> USD conversion
        need to be handled here, BEFORE ``sim.reset()`` triggers the
        articulation initialization:

        1. The MJCF ``<worldbody>`` is imported as a sibling prim
           ``Robot/worldBody`` containing the floor + sensors. PhysX
           sees this as a SECOND articulation root inside ``Robot/``,
           which crashes ``Articulation._initialize_impl`` with
           "Failed to find a single articulation". We deactivate it.

        2. (Optional, future) Same treatment for any stray collision
           planes. Skipped for now — once the worldBody subtree is
           inactive, the floor collision is gone with it.

        Stage B-v0 Pass 3 (2026-05-07): when cfg.scene.terrain_kind ==
        "transition", we additionally bake the flat→rough λ=18mm
        heightmap into a USD mesh under /World/TransitionTerrain
        (GLOBAL prim, NOT inside env_*) and swap the ground-plane
        collision filter. The terrain is shared across envs in this
        Pass 3a; the per-env upgrade lives in Pass 3b.
        """
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        n_deactivated = 0
        for prim in stage.Traverse():
            path = prim.GetPath().pathString
            if path.endswith("/Robot/worldBody"):
                prim.SetActive(False)
                n_deactivated += 1
        print(
            f"[CentipedeEnv] Deactivated {n_deactivated} worldBody "
            f"subtree(s) under /World/envs/env_*/Robot/",
            flush=True,
        )

        # Expose the InteractiveScene-created articulation as self.robot.
        self.robot = self.scene.articulations["robot"]

        terrain_kind = getattr(self.cfg, "terrain_kind", "flat")
        if terrain_kind == "transition":
            self._setup_transition_terrain(stage)
            # Filter against the new global terrain mesh, NOT the GroundPlane
            # (which is still authored but will not block collisions of the
            # robot above the heightfield given the spawn z; we keep it for
            # under-floor safety).
            self.scene.filter_collisions(
                global_prim_paths=["/World/GroundPlane", "/World/TransitionTerrain"]
            )
        elif terrain_kind == "flat":
            self.scene.filter_collisions(global_prim_paths=["/World/GroundPlane"])
        else:
            raise ValueError(
                f"cfg.scene.terrain_kind = {terrain_kind!r}; "
                "expected 'flat' or 'transition'."
            )

    def _setup_transition_terrain(self, stage) -> None:
        """Build the flat→rough λ=18mm transition heightmap and bake it
        into a USD triangle mesh + PhysX collision under
        /World/TransitionTerrain. Stores the heightmap on self for the
        per-env spawn-z lookup.

        Reuses the existing arena.py partition-of-unity blending; see
        scripts/optimization/arena.py::generate_flat_to_rough_transition_heightmap.
        """
        # Local imports — these modules require pxr (kit-deferred) and
        # are only needed when transition mode is active.
        sys.path.insert(0, os.path.join(PROJECT_ROOT, "scripts", "optimization"))
        from arena import generate_flat_to_rough_transition_heightmap  # noqa: E402
        from terrain import build_terrain_mesh, to_mujoco_terrain  # noqa: E402

        # Transition fields live on the ENV cfg (self.cfg), NOT on the
        # scene cfg — see CentipedeEnvCfg comment for why.
        # 2026-05-19 (training-arena-param refactor): pass the per-block
        # (λ, amp) tuples from env_cfg directly to the heightmap builder.
        # Defaults reproduce the prior call site bit-identical.
        ec = self.cfg
        h_m, blocks = generate_flat_to_rough_transition_heightmap(
            arena_size_m=float(ec.transition_arena_size_m),
            seed=int(ec.transition_seed),
            image_size=int(ec.transition_image_size),
            transition_wavelengths_mm=tuple(float(v) for v in ec.transition_wavelengths_mm),
            transition_amplitudes_m=tuple(float(v) for v in ec.transition_amplitudes_m),
        )
        # Cache for spawn-z and diagnostic lookups.
        self._transition_heightmap_m = h_m
        self._transition_arena_size_m = float(ec.transition_arena_size_m)
        self._transition_collision_heightmap_m = None
        if bool(getattr(ec, "transition_spawn_use_collision_patch", False)):
            self._transition_collision_heightmap_m, _ = to_mujoco_terrain(h_m)
        # Bake the mesh.
        prim_path = "/World/TransitionTerrain"
        from pxr import UsdGeom  # noqa: E402  -- kit is up by now
        UsdGeom.Xform.Define(stage, "/World")  # idempotent
        build_terrain_mesh(
            stage=stage,
            prim_path=prim_path,
            heightmap_m=h_m,
            world_size=float(ec.transition_arena_size_m),
            mesh_resolution=int(ec.transition_image_size),
            mujoco_compat=True,
        )
        # 2026-05-19 (training-arena-param refactor, §1 amendment):
        # report the actual tuples used to build the heightmap, not the
        # legacy scalar transition_rough_wavelength_mm / _amplitude_m
        # which are no longer plumbed through `generate_flat_to_rough_
        # transition_heightmap`. Values are read from self.cfg.
        _wls   = tuple(float(v) for v in ec.transition_wavelengths_mm)
        _amps  = tuple(float(v) for v in ec.transition_amplitudes_m)
        _sev_S = tuple(float(v) for v in ec.transition_block_severity_S)
        print(f"[CentipedeEnv] TransitionTerrain mesh built at {prim_path} "
              f"(arena {ec.transition_arena_size_m:.2f} m, "
              f"seed={ec.transition_seed}).", flush=True)
        print(f"  transition_wavelengths_mm    = "
              f"({', '.join(f'{v:.1f}' for v in _wls)})", flush=True)
        print(f"  transition_amplitudes_m      = "
              f"({', '.join(f'{v:.4f}' for v in _amps)})", flush=True)
        print(f"  transition_block_severity_S  = "
              f"({', '.join(f'{v:.3f}' for v in _sev_S)})", flush=True)

    # =========================================================================
    # Policy → physics
    # =========================================================================

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        """Called once per policy step (every `decimation` physics steps).

        Dispatches on cfg.action_mode:
          "global_6d"  -> _apply_global_6d_action  (Stage B-v0 default)
          "body_21d"   -> _apply_body_21d_action   (Stage B Step 1 legacy)
        """
        # DirectRLEnv asks _get_dones() before _get_rewards(). Invalidate the previous
        # transition here so _get_dones() cannot reuse its cached failure mask on the
        # next policy step. _get_rewards() repopulates the cache before wrappers read it.
        self._step_state = None
        # GO_FLING_CAUSE_DEFINITIVE_2026-06-29 §2: zero the substep impulse accumulators at the
        # start of each policy step (before the decimation loop). Default OFF -> attribute False
        # -> no-op -> byte-identical. ON: the wrapped scene.update (one call per physics substep)
        # then sums ∫(r×F) dt over this step's substeps; the cell reads it after step() returns.
        if getattr(self, "_dump_substep_impulse", False):
            self._reset_substep_accum()
        # Cache previous action for diagnostic / action-rate penalty.
        self._prev_actions = self.actions
        self.actions = actions.clone()

        mode = getattr(self.cfg, "action_mode", "global_6d")
        if mode == "global_6d":
            self._apply_global_6d_action(actions)
        elif mode == "body_21d":
            self._apply_body_21d_action(actions)
        else:
            raise ValueError(
                f"Unknown cfg.action_mode = {mode!r}; "
                "expected 'global_6d' or 'body_21d'."
            )

    # ───────────────────────────────────────────────────────────────────────
    # GO_FLING_CAUSE_DEFINITIVE_2026-06-29 §2 — physics-substep contact-impulse
    # instrumentation (default OFF; the three methods below are dead code on a
    # normal run because enable_substep_impulse_dump() is never called, so the
    # scene.update wrapper is never installed and _pre_physics_step's reset is a
    # no-op ⇒ byte-identical). The policy-rate contact snapshot (one ~4 ms
    # substep per 48 ms step) badly under-samples the impulsive foot-catch; this
    # reads contact + foot positions at every physics substep so the per-step
    # angular impulse  J_z = Σ_substeps Σ_feet (r_foot − r_com) × F_foot · dt
    # can close the body angular-momentum balance ΔL_z == J_z. J is taken about
    # the INSTANTANEOUS system COM (same reference as the offline Lz_multibody),
    # where gravity contributes zero yaw torque, so the z-balance is clean.
    # ───────────────────────────────────────────────────────────────────────
    def enable_substep_impulse_dump(self) -> None:
        """Turn ON physics-substep contact-impulse logging (idempotent).

        Wraps ``self.scene.update`` (called once per physics substep, AFTER the
        sensor buffers refresh) so each substep's external foot-contact force is
        integrated. Called by the analysis cell only; never on a shippable run.
        """
        if getattr(self, "_dump_substep_impulse", False):
            return
        # Prefer the all-body contact sensor (GO_FLING_CAUSE_DEFINITIVE deeper fix) when the cell installed it —
        # it captures the NON-FOOT (body-segment) contacts that carry 17-42% of the weight on rough terrain, which
        # the feet-only sensor misses (so feet-only cannot close the Newton balance on rough). Fall back to feet.
        _sensors = getattr(self.scene, "sensors", {})
        cs = _sensors.get("contact_sensor_all", None) if hasattr(self.scene, "sensors") else None
        _all_body = cs is not None
        if cs is None:
            cs = _sensors.get("contact_sensor", None) if hasattr(self.scene, "sensors") else None
        if cs is None:
            cs = self.scene["contact_sensor"]
        # map each contact-sensor body (sensor body order) -> articulation body index,
        # so positions in body_pos_w line up with net_forces_w columns.
        cs_names = [str(x) for x in cs.body_names]
        art_names = list(self.robot.data.body_names)
        foot_idx = [art_names.index(n) for n in cs_names]
        self._si_contact_sensor = cs
        self._si_foot_idx = torch.tensor(foot_idx, device=self.device, dtype=torch.long)
        self._si_foot_names = cs_names
        self._si_allbody = bool(_all_body)
        self._si_mass = self.robot.data.default_mass.to(self.device).float()      # (E, nb)
        self._si_M = self._si_mass.sum(dim=1, keepdim=True).clamp_min(1e-12)      # (E, 1)
        self._si_dt = float(self.physics_dt)
        nf = len(foot_idx)
        E = self.num_envs
        self._substep_Jz = torch.zeros(E, device=self.device)                    # (E,)  Σ(r×F)_z dt about COM
        self._substep_Jz_perfoot = torch.zeros(E, nf, device=self.device)        # (E,nf) per-body contribution
        self._substep_Jfoot = torch.zeros(E, nf, 3, device=self.device)          # (E,nf,3) per-body linear impulse
        self._substep_Jcontact = torch.zeros(E, 3, device=self.device)           # (E,3) total contact linear impulse
        self._substep_Jfric = torch.zeros(E, 3, device=self.device)             # (E,3) total tangential (friction) impulse
        self._substep_count = 0
        _orig_update = self.scene.update

        def _wrapped_scene_update(dt, *, _orig=_orig_update):
            _orig(dt=dt)
            if getattr(self, "_dump_substep_impulse", False):
                self._accumulate_substep_impulse()

        self._si_orig_scene_update = _orig_update
        self.scene.update = _wrapped_scene_update
        self._dump_substep_impulse = True
        print(
            f"[CentipedeEnv] SUBSTEP-IMPULSE dump ENABLED — sensor="
            f"{'contact_sensor_all (ALL bodies)' if _all_body else 'contact_sensor (FEET only)'}, "
            f"n_contact_bodies={nf}, physics_dt={self._si_dt:.4f}s, decimation={int(self.cfg.decimation)} "
            f"(J accumulated about instantaneous system COM).",
            flush=True,
        )

    def enable_contact_friction_dump(self, on: bool = True) -> None:
        """GO_FRICTION_DIRECT_V2 T1: also accumulate per-substep FRICTION (tangential) impulse via
        RigidContactView.get_friction_data (needs the all-body sensor filter_prim_paths_expr non-empty).
        Default OFF -> the friction block in _accumulate_substep_impulse is skipped -> subimp_Jcontact bit-identical."""
        self._dump_contact_friction = bool(on)

    def _reset_substep_accum(self) -> None:
        """Zero the per-policy-step substep impulse accumulators (called from _pre_physics_step)."""
        self._substep_Jz.zero_()
        self._substep_Jz_perfoot.zero_()
        self._substep_Jfoot.zero_()
        self._substep_Jcontact.zero_()
        if hasattr(self, "_substep_Jfric"):
            self._substep_Jfric.zero_()
        self._substep_count = 0

    def _accumulate_substep_impulse(self) -> None:
        """Integrate one physics substep's external foot-contact impulse (called per substep)."""
        cs = self._si_contact_sensor
        nf_w = cs.data.net_forces_w                                   # (E, nf, 3) world contact force
        if nf_w is None:
            return
        pos = self.robot.data.body_pos_w                             # (E, nb, 3) all-link world pos
        foot_pos = pos.index_select(1, self._si_foot_idx)            # (E, nf, 3)
        com = (self._si_mass.unsqueeze(-1) * pos).sum(dim=1) / self._si_M   # (E, 3) instantaneous system COM
        d = foot_pos - com.unsqueeze(1)                             # (E, nf, 3) moment arm about COM
        # (d × F)_z = d_x·F_y − d_y·F_x
        tau_z = d[..., 0] * nf_w[..., 1] - d[..., 1] * nf_w[..., 0]  # (E, nf)
        dt = self._si_dt
        self._substep_Jz += tau_z.sum(dim=1) * dt                    # (E,)
        self._substep_Jz_perfoot += tau_z * dt                       # (E, nf)
        self._substep_Jfoot += nf_w * dt                             # (E, nf, 3)
        self._substep_Jcontact += nf_w.sum(dim=1) * dt               # (E, 3)
        # GO_FRICTION_DIRECT_V2 T1: per-substep FRICTION (tangential) impulse via RigidContactView.get_friction_data.
        # Default OFF -> byte-identical. Needs the all-body sensor built with filter_prim_paths_expr non-empty
        # (make_allbody_contact_sensor(with_friction=True)); else filter_count==0 and get_friction_data is undefined/hangs.
        if getattr(self, "_dump_contact_friction", False):
            view = cs.contact_physx_view
            nfil = int(view.filter_count)
            if not getattr(self, "_fric_boot_checked", False):
                # BOOT GATE (T1): filter_count>=1 + 120s watchdog on the FIRST get_friction_data so a hang costs ONE boot.
                assert nfil >= 1, f"[E3friction] filter_count={nfil} < 1 — all-body sensor built without filter_prim_paths_expr; get_friction_data undefined (T1D node 1)"
                import threading, os as _os
                _wd = threading.Timer(120.0, lambda: (print("[E3friction] get_friction_data WATCHDOG 120s -> os._exit(3) (INFEASIBILITY signal)", flush=True), _os._exit(3)))
                _wd.start()
                ff, _fp, fcount, fstart = view.get_friction_data(dt)
                _wd.cancel()
                print(f"[E3friction] BOOT OK filter_count={nfil} shapes ff={tuple(ff.shape)} fcount={tuple(fcount.shape)} fstart={tuple(fstart.shape)} dtype={ff.dtype}", flush=True)
                self._fric_boot_checked = True
            else:
                ff, _fp, fcount, fstart = view.get_friction_data(dt)
            counts = fcount.reshape(-1).to(torch.long)
            starts = fstart.reshape(-1).to(torch.long)
            n_rows = counts.numel()
            agg = torch.zeros((n_rows, 3), device=self.device, dtype=ff.dtype)
            if int(counts.sum()) > 0:
                row_ids = torch.repeat_interleave(torch.arange(n_rows, device=self.device), counts)
                blk = counts.cumsum(0) - counts
                deltas = torch.arange(row_ids.numel(), device=self.device) - blk.repeat_interleave(counts)
                flat_idx = starts[row_ids] + deltas
                agg.index_add_(0, row_ids, ff.index_select(0, flat_idx).to(agg.dtype))
            fmat = agg.view(self.num_envs, nf_w.shape[1], nfil, 3).sum(dim=2)
            self._substep_Jfric += fmat.sum(dim=1) * dt
        self._substep_count += 1

    # ───────────────────────────────────────────────────────────────────────
    # ═══════════════════════════════════════════════════════════════════════
    # S2a — THE SINGLE IMPEDANCE PARAMETER ENTRY (PI-signed 2026-07-16, DISPATCH_LOG 19:34Z)
    #
    # PI ruling, verbatim: "一个关节就两个,kp和kv,直接喂进implicit参数,作为controller唯一的参数源"
    #                      "不要有所谓的被动弹簧"  /  "即使数值用一样的,也要分开,后面我们肯定还要单独调整"
    #
    # ONE joint family -> exactly (Kp, Kv) in PHYSICAL units (N·m/rad, N·m·s/rad), written STRAIGHT to PhysX.
    # The CPG is the SOLE reference generator (pos/vel TARGETS only) and supplies NO gains.
    #
    # DELETED here (PI "其他全去掉"), each of which was a hidden parameter source:
    #   • enable_passive_spring() + _passive_{k,c,q0} + the K=Kp+k_p / weighted-average-target FOLD
    #   • suppress_active_pitch() + _suppress_{active_pitch,keep_head_up} + the pitch Kp/Kv zeroing
    #   • the env's consumption of ctrl._pd_kp / ctrl._pd_kv (and therefore the whole YAML-base × gains-json-scale
    #     × *_imp_scale_runtime chain behind them, which is now DEAD for physics — S2b deletes it from the
    #     controller and proves that deletion inert, S1-style).
    #   • the head's DOUBLE parameterisation (head_Kp_eff = head_pitch_kp × pitch_kp_scale) -> ONE "head_pitch".
    #
    # WHY DELETING THE SPRING IS SAFE — MEASURED, not argued (q0 probe, DISPATCH_LOG 2026-07-16T20:40Z):
    #   robot.data.default_joint_pos == 0 on ALL 211 DOF, and the CPG's spine-pitch target is identically 0
    #   (pitch_target_final zeros except the head; the per-step blend is MULTIPLICATIVE). The deleted passive
    #   channel (rest = q0) and the surviving CPG channel (rest = target) therefore rest at the SAME point, so
    #   removing the spring does NOT shift the rest pose. Had q0 been non-zero, "全去掉" would have silently
    #   moved the robot's pose.
    #
    # NOT bit-identical, BY DESIGN: this CHANGES the spine-pitch stiffness (was K = 0 + bio_pitch_k 6e-4 via the
    #   fold; now Kp_body_pitch, a free parameter). The bio-calibrated 6e-4 / 7.2e-5 were PASSIVE-TISSUE constants
    #   (droop rigs); reused as ACTUATOR gains they are tuning parameters, not bio constants. That is a category
    #   change and is stated, not slid through (it un-anchors feedback_body_pitch_always_bio_calib_2026-07-10).
    # ═══════════════════════════════════════════════════════════════════════
    S2_FAMILIES = ("body_yaw", "head_pitch", "body_pitch", "body_roll",
                   "legbase_yaw", "legbase_pitch", "legbase_roll", "knee")

    # the controller gain-scale channels that S2 makes DEAD. Any non-1.0 value means a caller still believes it
    # can modulate impedance through the old chain -> its effect would VANISH silently. Fail loudly (S1 doctrine).
    _S2_DEAD_SCALES = ("body_kp_scale_runtime", "body_kv_scale_runtime", "pitch_imp_scale_runtime",
                       "pitch_kp_scale_runtime", "pitch_kv_scale_runtime", "roll_kp_scale_runtime",
                       "roll_kv_scale_runtime", "leg_imp_scale_runtime", "leg_kp_scale_runtime",
                       "leg_kv_scale_runtime")

    def __getattr__(self, name):
        """Fail LOUDLY and DIRECTED on the S1/S2a-deleted members.

        Python calls __getattr__ ONLY when normal attribute lookup has already failed, so this costs
        nothing on any live path and can never shadow a real attribute. Unknown names keep raising the
        ordinary AttributeError, so hasattr()/pickle/deepcopy probing is unaffected.
        """
        if name in _S2_DELETED_MEMBERS:
            raise RuntimeError(
                "CentipedeEnv.%s was DELETED by the implicit-only unification (S1/S2a, 2026-07-16, "
                "PI-signed; DISPATCH_LOG 2026-07-16T19:34Z). It was %s.\n"
                "There is no passive spring and no explicit torque path any more: every joint family has "
                "exactly ONE (Kp, Kv) in physical units, written straight to PhysX.\n"
                "  TO MIGRATE -> env.set_joint_impedance({family: (Kp, Kv), ...}) for every family in "
                "env.s2_family_ids(); torque caps -> env.set_joint_effort_limit({family: tau_max_Nm|None}). "
                "Reference wiring + the full table: docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md.\n"
                "  TO REPRODUCE A HISTORICAL explicit / passive-spring RESULT -> do NOT migrate. A migrated "
                "script computes DIFFERENT physics (implicit vs explicit is MEASURED at -37.2%% bl_per_cycle "
                "at amp_mult 1.0), so it will not reproduce the old number. Check out the pre-surgery files "
                "instead: outputs/analysis/implicit_unify_2026-07-16/backup_pre_surgery/ (or the snapshot "
                "envs/centipede_env.py.before_S1_explicit_delete_2026-07-16)."
                % (name, _S2_DELETED_MEMBERS[name])
            )
        raise AttributeError("%r object has no attribute %r" % (type(self).__name__, name))

    # ---- the dead gain-scale guard (S2a), hoisted OFF the per-substep path (2026-07-16) -------------
    def _s2_scales_fingerprint(self):
        """HOST-ONLY identity of the dead-scale chain: (id, in-place version) per scale + the effort flag.

        No CUDA sync. torch bumps Tensor._version on EVERY in-place mutation, and the controller writes
        these with .copy_() (impedance_controller_torch.py, apply_global_6d_modulation), so a 6D-action
        write IS seen; id() changes on any rebind. An UNCHANGED fingerprint therefore proves the tensors
        are the same objects and unmutated since the last full check => still all-1.0, without touching
        the GPU. If a future torch drops the private _version, the fallback is a fresh object() -> the
        fingerprint never matches -> the full check runs every step (slow, but never a false negative).
        """
        ctrl = self.controller
        fp = []
        for _nm in self._S2_DEAD_SCALES:
            _s = getattr(ctrl, _nm, None)
            if _s is None:
                fp.append(None)
                continue
            _v = getattr(_s, "_version", None)
            fp.append((id(_s), _v if _v is not None else object()))
        fp.append(bool(getattr(ctrl, "effort_limit_enabled", False)))
        return tuple(fp)

    def _s2_assert_scales_inert(self):
        """FULL check of the dead gain-scale chain + the dead effort clamp.

        ONE fused device->host sync for all 10 scales (was: 10 syncs, EVERY substep).

        S2b (2026-07-16): the controller ATTRIBUTES checked here no longer EXIST -- S2b deleted the 10
        gain scales, the effort tensors and `effort_limit_enabled` outright, so on a stock controller
        every getattr below returns the default and this whole method is a no-op. It is deliberately
        RETAINED as defence-in-depth, for two reasons:
          1. Python lets a caller CREATE a deleted attribute by plain assignment
             (`ctrl.body_kp_scale_runtime = x`) -- several legacy scripts do exactly that -- and the
             attribute would then be just as dead and just as silent as before. This still catches it.
          2. The FIRST line of defence now lives in the controller's own setters
             (_assert_dead_gain_scale, raising from apply_action_modulation / apply_global_6d_modulation),
             which fire once per POLICY step at the interface the caller actually touches, and
             set_freesearch_runtime / set_{pitch,leg}_imp_scale_runtime are gone entirely (AttributeError).
        """
        ctrl = self.controller
        _names, _flags = [], []
        for _nm in self._S2_DEAD_SCALES:
            _s = getattr(ctrl, _nm, None)
            if _s is None:
                continue
            _names.append(_nm)
            _flags.append(torch.all(torch.as_tensor(_s, device=self.device) == 1.0))
        if _flags and not bool(torch.all(torch.stack(_flags))):      # <-- the ONLY sync
            _bad = [_n for _n, _f in zip(_names, _flags) if not bool(_f)]
            raise RuntimeError(
                "S2: controller.%s is not 1.0, but S2 made it DEAD -- the impedance table is the only gain "
                "source, so this modulation would silently have NO effect on physics.\n"
                "If a policy/sweep must modulate impedance, it has to modulate the TABLE "
                "(env.set_joint_impedance) -- not this scale. (S2a 2026-07-16, PI-signed; same "
                "fail-loudly doctrine as the S1 explicit-path guard.)" % ", controller.".join(_bad)
            )
        # The controller's effort clamp WAS dead under implicit: it clamped `tau`, which the implicit path
        # DISCARDS (it writes an all-zero effort target). A caller that set it believes it has a torque cap
        # and does NOT. The real cap is the PhysX effort limit -> env.set_joint_effort_limit().
        # S2b DELETED the clamp, the effort tensors and this flag from the controller; see the docstring.
        if getattr(ctrl, "effort_limit_enabled", False):
            raise RuntimeError(
                "S2: controller.effort_limit_enabled is True, but under implicit PD the controller's effort clamp "
                "is INERT -- it clamps the controller's `tau`, which the implicit path discards. A run relying on it "
                "would be SILENTLY UNCAPPED.\n"
                "Use env.set_joint_effort_limit({family: tau_max_Nm_or_None}) -- a family-exact PhysX limit -- and "
                "stop passing effort_* / enable_effort to controller.set_freesearch_runtime()."
            )

    def s2_family_ids(self):
        """family -> LongTensor of joint ids, enumerated from the controller's OWN id sets (the same
        enumeration the q0 probe measured). Families absent on this instance (a 2-DOF leaner model has no
        legbase_roll/knee; a model without pitch/roll has neither) are simply not in the dict."""
        ctrl = self.controller
        dev = self.device
        fam = {}
        fam["body_yaw"] = ctrl.body_joint_ids.to(dev)
        if getattr(ctrl, "has_pitch", False):
            _p = ctrl.pitch_joint_ids.to(dev)
            _nh = max(0, min(int(getattr(ctrl, "head_pitch_joints", 0)), int(_p.numel())))
            if _nh:
                fam["head_pitch"] = _p[:_nh]
            if int(_p.numel()) > _nh:
                fam["body_pitch"] = _p[_nh:]
        if getattr(ctrl, "has_roll", False):
            fam["body_roll"] = ctrl.roll_joint_ids.to(dev)
        _lids = ctrl.leg_flat_ids.to(dev)
        _lcls = ctrl.leg_dof_idx_flat.to(dev)
        _lbl = ("legbase_yaw", "legbase_pitch", "legbase_roll", "knee")
        for j in range(int(getattr(ctrl, "n_leg_dof", 4))):
            _sel = _lids[_lcls == j]
            if _sel.numel():
                fam[_lbl[j]] = _sel
        return fam

    def set_joint_impedance(self, table):
        """S2a: THE parameter entry. `table` maps EVERY family present on this model -> (Kp, Kv) in PHYSICAL
        units (N·m/rad, N·m·s/rad). Written straight to PhysX on the next step; nothing scales it.

        Each Kp/Kv is EITHER a scalar (applied to every env) OR a length-num_envs sequence — the per-env form
        is what keeps the vectorized sweep alive (N entries in the impedance json -> N envs in ONE boot).

        Families stay SEPARATE even when the numbers are equal (PI: "即使数值用一样的,也要分开").
        A missing or unknown family RAISES: a typo must not silently leave a joint on a stale/zero gain — a
        silent default is exactly the failure mode S2 exists to delete."""
        dev = self.device
        fam = self.s2_family_ids()
        _unknown = sorted(set(table) - set(fam))
        if _unknown:
            raise ValueError(
                "set_joint_impedance: unknown family %s. This model has exactly %s. (Known S2 families: %s.)"
                % (_unknown, sorted(fam), list(self.S2_FAMILIES))
            )
        _missing = sorted(set(fam) - set(table))
        if _missing:
            raise ValueError(
                "set_joint_impedance: no (Kp, Kv) given for %s. S2 requires EVERY family explicitly — an "
                "omitted family would fall back to a hidden default, which is the entire thing S2 deletes." % _missing
            )

        def _col(x, _f, _what):
            """scalar -> (E,) ; length-E sequence -> (E,) ; anything else raises."""
            t = torch.as_tensor(x, dtype=torch.float32, device=dev).reshape(-1)
            if int(t.numel()) == 1:
                t = t.expand(self.num_envs)
            elif int(t.numel()) != self.num_envs:
                raise ValueError(
                    "set_joint_impedance: %s %s has %d values but there are %d envs. Give a scalar (all envs) "
                    "or exactly one value per env." % (_f, _what, int(t.numel()), self.num_envs)
                )
            if not bool(torch.all(torch.isfinite(t))) or bool(torch.any(t < 0.0)):
                raise ValueError("set_joint_impedance: %s %s must be finite and >= 0; got %r" % (_f, _what, x))
            return t.contiguous()

        n_dof = int(self.robot.data.joint_pos.shape[1])
        Kp = torch.zeros((self.num_envs, n_dof), dtype=torch.float32, device=dev)
        Kv = torch.zeros((self.num_envs, n_dof), dtype=torch.float32, device=dev)
        _tbl = {}
        for _f, _ids in fam.items():
            _kp_in, _kv_in = table[_f]
            _kp = _col(_kp_in, _f, "Kp")
            _kv = _col(_kv_in, _f, "Kv")
            Kp[:, _ids] = _kp.unsqueeze(1)       # (E,1) broadcasts across the family's joints
            Kv[:, _ids] = _kv.unsqueeze(1)
            _tbl[_f] = (_kp.detach().cpu().tolist(), _kv.detach().cpu().tolist())
        self._s2_Kp = Kp
        self._s2_Kv = Kv
        self._s2_table = _tbl
        self._s2_active_ids = torch.cat([fam[k] for k in sorted(fam)]).unique()
        self._impl_gains_written = False   # force a re-write to PhysX on the next _apply_action
        # SET-TIME copy of the dead-scale / dead-effort guard: catch a caller that set a scale BEFORE
        # installing the table, at install time, instead of one Isaac boot + one step later.
        self._s2_scales_fp_ok = None
        self._s2_assert_scales_inert()
        self._s2_scales_fp_ok = self._s2_scales_fingerprint()
        print("[CentipedeEnv] S2 IMPEDANCE TABLE installed — the ONLY gain source (physical units, %d env(s)):"
              % self.num_envs, flush=True)
        for _f in self.S2_FAMILIES:
            if _f in _tbl:
                _kp_l, _kv_l = _tbl[_f]
                _u = (len(set(_kp_l)) == 1 and len(set(_kv_l)) == 1)
                if _u:
                    _s = "Kp=%-12.6g Kv=%-12.6g" % (_kp_l[0], _kv_l[0])
                else:
                    _s = "Kp=[%.4g..%.4g] Kv=[%.4g..%.4g] (per-env)" % (min(_kp_l), max(_kp_l), min(_kv_l), max(_kv_l))
                print("[CentipedeEnv]   %-14s n=%3d  %s" % (_f, int(fam[_f].numel()), _s), flush=True)

    def set_joint_effort_limit(self, table):
        """S2a-effort (2026-07-16): per-FAMILY PhysX torque cap — the real actuator strength under implicit PD.

        `table` maps family -> tau_max in N·m (scalar or length-num_envs), or **None = leave that family UNCAPPED**
        (its PhysX limit is not written at all, so the uncapped baseline stays bit-exact). Families may be omitted
        entirely; omitted == None.

        WHY THIS EXISTS (both bugs were found by the independent audit 2026-07-16 and are REAL):
          1. The controller's own effort clamp is DEAD under implicit PD — it clamps `tau`, which the implicit path
             DISCARDS. Anything relying on it is silently uncapped. `_apply_action_implicit_pd` now RAISES on it.
          2. The old `--physx-override-json effort_limit_nm` path selected joints by NAME SUBSTRING with an
             `unmatched -> max(spec.values())` fallback. The head joint is named `body_pitch_0`, so (a) no substring
             could select head_pitch alone and (b) a `body_pitch` cap ALSO capped the head; any unmatched joint
             silently got the LOOSEST bound. This path is family-EXACT (same enumeration as the impedance table),
             writes ONLY the families given, and has no fallback.
        S3's stated lever is tau_max, so this had to be correct before S3 — not after."""
        dev = self.device
        fam = self.s2_family_ids()
        _unknown = sorted(set(table) - set(fam))
        if _unknown:
            raise ValueError(
                "set_joint_effort_limit: unknown family %s. This model has exactly %s." % (_unknown, sorted(fam))
            )
        _capped = {k: v for k, v in table.items() if v is not None}
        if not _capped:
            self._s2_effort = None
            print("[CentipedeEnv] S2 EFFORT: every family UNCAPPED (no PhysX effort limit written).", flush=True)
            return
        _ids_all, _val_all = [], []
        for _f, _v in _capped.items():
            t = torch.as_tensor(_v, dtype=torch.float32, device=dev).reshape(-1)
            if int(t.numel()) == 1:
                t = t.expand(self.num_envs)
            elif int(t.numel()) != self.num_envs:
                raise ValueError("set_joint_effort_limit: %s has %d values but there are %d envs."
                                 % (_f, int(t.numel()), self.num_envs))
            if not bool(torch.all(torch.isfinite(t))) or bool(torch.any(t <= 0.0)):
                raise ValueError("set_joint_effort_limit: %s tau_max must be finite and > 0; got %r" % (_f, _v))
            _ids_all.append(fam[_f])
            _val_all.append(t.unsqueeze(1).expand(self.num_envs, int(fam[_f].numel())))
        ids = torch.cat(_ids_all)
        vals = torch.cat(_val_all, dim=1)
        # write ONLY the capped families' joints -> the uncapped ones keep their USD limit untouched
        self.robot.write_joint_effort_limit_to_sim(vals, joint_ids=ids)
        self._s2_effort = {k: (None if v is None else float(torch.as_tensor(v).reshape(-1)[0]))
                           for k, v in table.items()}
        print("[CentipedeEnv] S2 EFFORT LIMITS (PhysX, family-exact, N·m):", flush=True)
        for _f in self.S2_FAMILIES:
            if _f in fam:
                _v = table.get(_f)
                print("[CentipedeEnv]   %-14s n=%3d  tau_max=%s"
                      % (_f, int(fam[_f].numel()), "UNCAPPED" if _v is None else ("%.6g" % float(
                          torch.as_tensor(_v).reshape(-1)[0]))), flush=True)

    def _apply_action_implicit_pd(self):
        """S2a: write the impedance TABLE straight to PhysX, and the CPG's targets on top of it.

        The CPG supplies TARGETS ONLY. ctrl._pd_kp / ctrl._pd_kv are deliberately NOT read: the table is the
        single source of gains (PI: "作为controller唯一的参数源")."""
        ctrl = self.controller
        dev = self.device
        if self._s2_Kp is None:
            raise RuntimeError(
                "S2: no impedance table installed. Call env.set_joint_impedance({family: (Kp, Kv), ...}) for "
                "every family in env.s2_family_ids() before stepping.\n"
                "The passive spring, suppress_active_pitch and the gains-json scale chain were DELETED "
                "(S2a 2026-07-16, PI-signed), so there is NO default gain to fall back on. This is deliberate: "
                "a silent default is exactly what S2 removes."
            )
        # The old impedance-modulation channels must stay inert (6D action ch-4 body_imp_scale ->
        # ctrl.body_k{p,v}_scale_runtime was the live one) and the controller's effort clamp must stay off,
        # or their effect would vanish SILENTLY. Both are checked by _s2_assert_scales_inert().
        #
        # HOISTED (2026-07-16): this used to run ~10 `torch.all(...)` per SUBSTEP -- 10 device->host syncs
        # x decimation, every control step. Now a host-only fingerprint (object id + torch's in-place
        # _version counter + the effort flag) gates it: unchanged fingerprint => provably still all-1.0 =>
        # ZERO syncs. The full check (ONE fused sync) runs only when something actually changed -- which
        # still includes a 6D-action .copy_() write, i.e. the fail-loudly guarantee is UNCHANGED. It stays
        # on the _apply_action path on purpose: a sweep that monkey-patches _apply_global_6d_action (the
        # documented sweep-runner pattern) would bypass any check installed at the action site.
        _fp = self._s2_scales_fingerprint()
        if _fp != getattr(self, "_s2_scales_fp_ok", None):
            self._s2_assert_scales_inert()
            self._s2_scales_fp_ok = _fp
        pos = ctrl._pd_pos_target
        vel = ctrl._pd_vel_target
        ids = self._s2_active_ids
        # write per-joint stiffness/damping ONCE (persists across reset; juror-1-I)
        if not getattr(self, "_impl_gains_written", False):
            self.robot.write_joint_stiffness_to_sim(self._s2_Kp[:, ids], joint_ids=ids)
            self.robot.write_joint_damping_to_sim(self._s2_Kv[:, ids], joint_ids=ids)
            # mirror into the actuator-model buffers so data.applied_torque is populated (IsaacLab #128)
            try:
                _act = self.robot.actuators["active"]
                _act.stiffness[:] = self.robot.data.joint_stiffness[:, _act.joint_indices]
                _act.damping[:] = self.robot.data.joint_damping[:, _act.joint_indices]
            except Exception:
                pass
            self._impl_gains_written = True
        # targets change every step (cheap buffer fill); effort feed-forward := 0
        self.robot.set_joint_position_target(pos[:, ids], joint_ids=ids)
        self.robot.set_joint_velocity_target(vel[:, ids], joint_ids=ids)
        self.robot.set_joint_effort_target(
            torch.zeros((self.num_envs, int(ids.numel())), dtype=pos.dtype, device=dev),
            joint_ids=ids,
        )

    def _apply_body_21d_action(self, actions: torch.Tensor) -> None:
        """Stage B Step 1 21-dim action mapping (preserved as a legacy mode).

        actions[:, 0:19]  -> per-joint body amplitude scale (joint_body_1..19)
        actions[:, 19]    -> wave-number scale
        actions[:, 20]    -> body_kp scale
        Bounds: body_amp [0.7,1.3], wave_num [0.8,1.2], body_kp [0.7,1.5].

        *** THIS ACTION MODE IS UNRUNNABLE since 2026-07-16. ch-20 is a GAIN scale, and the
        implicit-only unification deleted every route from a gain scale to physics; the
        controller's apply_action_modulation() RAISES on body_kp_scale != 1.0, which a policy
        will hit immediately (only the exact action 0.0 maps to 1.0). This was already a legacy
        mode (global_6d is the training mode), so it is left intact-but-failing rather than
        silently reinterpreted. To modulate impedance, modulate the TABLE:
        env.set_joint_impedance({family: (Kp, Kv)}). See _S2_DEAD_ACTION_NOTE at the top. ***
        """
        amp_action  = actions[:, 0:19]   # (E, 19)
        wave_action = actions[:, 19]     # (E,)
        kp_action   = actions[:, 20]     # (E,)

        body_amp_scale  = _bounded_scale(amp_action,  _BODY_AMP_LO, _BODY_AMP_HI)
        wave_num_scale  = _bounded_scale(wave_action, _WAVE_NUM_LO, _WAVE_NUM_HI)
        body_kp_scale   = _bounded_scale(kp_action,   _BODY_KP_LO,  _BODY_KP_HI)

        self.controller.apply_action_modulation(
            body_amp_scale=body_amp_scale,
            wave_number_scale=wave_num_scale,
            body_kp_scale=body_kp_scale,
        )

    def _apply_global_6d_action(self, actions: torch.Tensor) -> None:
        """Stage B-v0 global six-knob CPG/impedance action mapping.

        actions[:, 0]  -> body yaw amplitude scale         in [_G6_BODY_AMP_LO, _G6_BODY_AMP_HI]
        actions[:, 1]  -> leg yaw (DOF-0) amplitude scale  in [_G6_LEG_YAW_AMP_LO, _G6_LEG_YAW_AMP_HI]
        actions[:, 2]  -> leg pitch (DOF-1) amplitude scale in [_G6_LEG_PITCH_AMP_LO, _G6_LEG_PITCH_AMP_HI]
        actions[:, 3]  -> body wave-number scale           in [_G6_WAVE_NUM_LO, _G6_WAVE_NUM_HI]
        actions[:, 4]  -> body yaw impedance scale (kp+kv) in [_G6_BODY_IMP_LO, _G6_BODY_IMP_HI]
                          *** STRUCTURALLY DEAD since 2026-07-16 — this channel no longer reaches
                          physics, and a value != 1.0 RAISES (it does not silently no-op). Pin it
                          with --design-a-fixed-imp-freq / --design-c-fix-all-non-wave, which the
                          campaign CLI already does. See _S2_DEAD_ACTION_NOTE at the top. ***
        actions[:, 5]  -> gait frequency scale             in [_G6_FREQ_LO, _G6_FREQ_HI]
        a=0 maps to scale=1.0 for every component (baseline preserved).
        """
        body_amp_s     = _bounded_scale(actions[:, 0], _G6_BODY_AMP_LO,      _G6_BODY_AMP_HI)
        leg_yaw_amp_s  = _bounded_scale(actions[:, 1], _G6_LEG_YAW_AMP_LO,   _G6_LEG_YAW_AMP_HI)
        leg_pitch_amp_s= _bounded_scale(actions[:, 2], _G6_LEG_PITCH_AMP_LO, _G6_LEG_PITCH_AMP_HI)
        _g6_wave_hi = float(getattr(self.cfg, "g6_wave_num_hi_override", -1.0))
        _g6_wave_hi = _G6_WAVE_NUM_HI if _g6_wave_hi <= 0.0 else _g6_wave_hi
        wave_num_s     = _bounded_scale(actions[:, 3], _G6_WAVE_NUM_LO,      _g6_wave_hi)
        # P4 v3.4 Phase 1 (2026-06-02): default-OFF ENV-SIDE wave-baseline recenter.
        # Shift the wave action zero-point so action=0 -> applied_abs = wave_baseline_abs
        # (i.e. scale = wave_baseline_abs / n_wave). Controller n_wave is UNTOUCHED;
        # logging (applied_abs = scale * pass15_baseline_n_wave) stays truthful because
        # we shift the SCALE itself. Default == pass15_baseline_n_wave (3.0) ⇒ shift 0
        # ⇒ byte-identical. Additive (action-independent) ⇒ action sensitivity preserved.
        _wave_base_abs = float(getattr(self.cfg, "wave_baseline_abs", 3.0))
        _nwave_ref = float(self.cfg.pass15_baseline_n_wave)
        if _wave_base_abs != _nwave_ref:
            wave_num_s = wave_num_s + (_wave_base_abs / _nwave_ref - 1.0)
        # ch-4 is DEAD (see _S2_DEAD_ACTION_NOTE): the decode is preserved bit-identically, but the
        # value only survives to the controller's _assert_dead_gain_scale, which RAISES on anything
        # != 1.0. The Design A pin below is what keeps a global_6d run alive; without it this line's
        # output is non-1.0 the moment the policy moves ch-4, and the step raises.
        body_imp_s     = _bounded_scale(actions[:, 4], _G6_BODY_IMP_LO,      _G6_BODY_IMP_HI)
        freq_s         = _bounded_scale(actions[:, 5], _G6_FREQ_LO,          _G6_FREQ_HI)

        # Design A 2026-05-16: fix body_imp + freq at 1.0 to isolate the
        # wave-number action axis. Default OFF preserves all prior
        # behaviour byte-identically.
        if self.cfg.designA_fixed_imp_freq_enabled:
            body_imp_s = torch.ones_like(body_imp_s)
            freq_s     = torch.ones_like(freq_s)

        # Design B+ 2026-05-16: narrow body_amp / leg_yaw / leg_pitch to
        # [lo, hi] (default [0.9, 1.1]) to remove the leg/body-amplitude
        # substitute axes identified in PHASE 1 leg-axis confound audit.
        # Applied AFTER Design A override and AFTER _bounded_scale, so
        # the action -> scale map is unchanged; only the scale endpoints
        # are clamped before smoothing. Default OFF preserves prior
        # behaviour byte-identically.
        if self.cfg.designBplus_narrow_legs_enabled:
            _lo = float(self.cfg.designBplus_narrow_lo)
            _hi = float(self.cfg.designBplus_narrow_hi)
            body_amp_s      = body_amp_s.clamp(_lo, _hi)
            leg_yaw_amp_s   = leg_yaw_amp_s.clamp(_lo, _hi)
            leg_pitch_amp_s = leg_pitch_amp_s.clamp(_lo, _hi)

        # Design C 2026-05-17: fix ALL non-wave action axes exactly at
        # 1.0 (body_amp, leg_yaw, leg_pitch). Combined with Design A's
        # body_imp + freq pin, this leaves u_wave_num as the ONLY free
        # cross-env adaptation axis. Triggered by overnight PHASE 1
        # residual-axis audit which found u_leg_pitch
        # micro-perturbations remained terrain-correlated under B+
        # despite a 3x std reduction. Default OFF preserves prior
        # behaviour byte-identically.
        if self.cfg.designC_fix_all_non_wave_enabled:
            body_amp_s      = torch.ones_like(body_amp_s)
            leg_yaw_amp_s   = torch.ones_like(leg_yaw_amp_s)
            leg_pitch_amp_s = torch.ones_like(leg_pitch_amp_s)

        # Stage B-v0 Pass 8 (2026-05-08): first-order low-pass on the
        # 6-D global-gait target parameters BEFORE writing them into
        # the controller. The policy commands u_target; the controller
        # consumes u_applied. See Pass 8 block in __init__ for full
        # rationale and tau_u choice.
        u_target = torch.stack(
            [body_amp_s, leg_yaw_amp_s, leg_pitch_amp_s,
             wave_num_s, body_imp_s, freq_s],
            dim=1,
        ).to(dtype=self._u_applied.dtype)
        self._u_target = u_target.detach().clone()
        self._u_applied = self._u_applied + self._u_smooth_alpha * (
            u_target - self._u_applied
        )

        # P4 interface-smoothing (default OFF): override the WAVE channel
        # (idx 3) with a slower tau=1.0 EMA + 0.5/s per-step rate limiter in
        # applied_abs units, driven by the raw commanded target u_target[:, 3].
        # This sits AFTER the tau=0.30 EMA and BEFORE the single controller
        # call, so the controller (and physics in _apply_action) consume the
        # smoothed wave scale with no re-call. Numerically identical to the
        # eval-time WaveSmoother. When OFF, the block is skipped and
        # self._u_applied is byte-identical to Pass 8.
        if self.cfg.wave_smooth_interface_enabled:
            tgt_abs = u_target[:, 3] * self._wave_smooth_base
            self._wave_ema_abs = self._wave_ema_abs + self._wave_smooth_alpha * (
                tgt_abs - self._wave_ema_abs
            )
            d_wave = torch.clamp(
                self._wave_ema_abs - self._wave_abs,
                -self._wave_smooth_max_dabs_abs,
                self._wave_smooth_max_dabs_abs,
            )
            self._wave_abs = self._wave_abs + d_wave
            self._u_applied[:, 3] = self._wave_abs / self._wave_smooth_base

        self.controller.apply_global_6d_modulation(
            body_amp_scale=self._u_applied[:, 0],
            leg_yaw_amp_scale=self._u_applied[:, 1],
            leg_pitch_amp_scale=self._u_applied[:, 2],
            wave_number_scale=self._u_applied[:, 3],
            body_imp_scale=self._u_applied[:, 4],
            freq_scale=self._u_applied[:, 5],
        )

    def _apply_action(self) -> None:
        """Called every physics step, between policy decisions.

        Stage B Foundation (2026-05-07): controller stepping is now
        driven by the per-env _sim_time tensor (was a single host scalar
        _t_host in Stage A). This makes per-env asynchronous reset
        produce correct controller behaviour: a freshly-reset env sees
        t=0 and goes through settle/ramp from scratch, while the other
        envs keep advancing.

        Reads articulation state, runs the controller, writes torques.
        Time fed to the controller is the per-env elapsed time since
        the last reset; we advance _sim_time by physics dt each call.
        """
        import time as _time
        dt = self.cfg.sim.dt
        _t0 = _time.perf_counter()
        # Per-env time as the controller's time argument (Stage B).
        # The torch controller now accepts (num_envs,) tensor t.
        # ══ IMPLICIT-ONLY FORCE PATH ══ (PI-signed 2026-07-16; DISPATCH_LOG 2026-07-16T19:34Z) ═══════════════
        # The EXPLICIT torque path is DELETED. PhysX's implicit PD computes every force; the CPG is the SOLE
        # reference generator, supplying only (pos_target, vel_target) via the controller's _pd_* stash.
        # `controller.step()` is still called every step -- it is STATEFUL (integrates the Kuramoto phase) and it
        # populates that stash.
        # S2b (2026-07-16) finished the job inside the controller: it no longer computes gains or `tau` at all and
        # step() now returns None. (Kp, Kv) come from the env's impedance table (set_joint_impedance -> PhysX).
        #
        # FAIL LOUDLY, never silently: 169 of 182 env-using scripts never set `_implicit_pd`. Silently running them
        # implicit would change their physics without a word (implicit != explicit is MEASURED: bl -37.2% at
        # amp_mult 1.0, marker RESULT-2026-07-16T0350Z-cap2x2), which is exactly the "simulation 出错" this change
        # exists to prevent. An un-migrated caller gets an error, not corrupt data.
        if not getattr(self, "_implicit_pd", False):
            raise RuntimeError(
                "CentipedeEnv is IMPLICIT-ONLY as of 2026-07-16 (PI-signed; DISPATCH_LOG 2026-07-16T19:34Z). "
                "The explicit torque path (set_joint_effort_target of the controller's tau) has been DELETED, so "
                "this caller -- which never set `_implicit_pd` -- would otherwise run SILENTLY DIFFERENT physics "
                "(implicit vs explicit is measured at -37.2% bl_per_cycle at amp_mult 1.0). Failing instead of "
                "corrupting the run. TO MIGRATE: set `base_env._implicit_pd = True` and install the impedance "
                "table via `base_env.set_joint_impedance({family: (Kp, Kv)})` for every family in "
                "`base_env.s2_family_ids()` before stepping; see the --implicit-pd / --impedance-json wiring in "
                "scripts/sanity/stage_a_vec_sweep_cell_compliance_2026-07-13.py for the reference. "
                "(`controller.set_export_pd()` no longer exists -- S2b made the target stash unconditional, "
                "since targets are now the controller's only output.)"
            )
        self.controller.step(self._sim_time, dt)   # CPG -> _pd_pos/_pd_vel_target stash; returns None (S2b)
        self._apply_action_implicit_pd()
        # Diagnostic accumulator. Drained by the smoke script.
        self._last_controller_dt_total += _time.perf_counter() - _t0

        # Advance both clocks: host-side scalar (kept for Stage A
        # back-compat with smoke_layer2_stage_a.py and any external
        # caller that still reads it) and per-env tensor.
        self._t_host += dt
        self._sim_time = self._sim_time + dt
        # Stage E (2026-07-12): accumulate this substep's windowed-terrain signals
        # into the fixed wall-clock ring (default OFF -> no-op -> byte-identical).
        if bool(getattr(self, "_nwo_enabled", False)):
            self._nwo_accumulate_substep()

    # =========================================================================
    # Observation / reward / termination — placeholders for Stage A
    # =========================================================================

    def _get_observations(self) -> dict:
        """Build the policy observation.

        Stage B-v0 (Pass 2, 2026-05-07): in `global_6d` action mode this
        returns the 86-D contact-rich vector specified in the project
        prompt. In the legacy `body_21d` mode (which has its own action
        wiring sanity that does NOT inspect the observation), we return
        zeros at observation_space dim — preserves the existing test
        contract without injecting an obs schema mismatch.

        The 86-D layout is documented on cfg.observation_space and the
        comments below mark each slice. Slices are END-EXCLUSIVE.

        IMPORTANT — anatomical ordering:
          - Body yaw uses controller.body_joint_ids (exact-name resolved
            in controllers/kinematics.py), so the 4 compressed bins map
            to joint_body_1..joint_body_19 in anatomical order, NOT
            articulation index order.
          - Foot contact uses self._foot_idx_grid which is resolved from
            the contact sensor's body_names by matching f"foot_{seg}_{side}",
            so F_pair[:, seg, side] is anatomical too. Side labels are
            kept as `side0` / `side1` (NOT `left` / `right`) since the
            MJCF naming convention has not been independently verified.
        """
        # Stage E (2026-07-12): NEW windowed-terrain obs REPLACES the 86-D obs
        # when cfg.new_windowed_obs (default OFF -> this branch skipped -> 86-D path).
        if bool(getattr(self.cfg, "topology_obs_v2_enabled", False)):
            return self._get_topology_observation()
        if bool(getattr(self.cfg, "new_windowed_obs", False)):
            return self._get_windowed_observation()
        if self.cfg.action_mode != "global_6d":
            # Legacy body_21d path: zeros at the cfg-declared dim.
            return {"policy": torch.zeros(
                (self.num_envs, self.cfg.observation_space),
                dtype=torch.float32, device=self.device,
            )}

        E = self.num_envs
        ctrl = self.controller
        art = self.robot

        # ── Lazy-init foot_idx_grid from sensor body_names ──
        sensor = self.scene.sensors.get("contact_sensor", None) if hasattr(self.scene, "sensors") else None
        if sensor is None:
            try:
                sensor = self.scene["contact_sensor"]
            except Exception:  # noqa: BLE001
                sensor = None
        if sensor is not None and self._foot_idx_grid is None:
            body_names = list(sensor.body_names)
            grid = torch.zeros((_N_SEG, 2), dtype=torch.long, device=self.device)
            for seg in range(_N_SEG):
                for side in range(2):
                    name = f"foot_{seg}_{side}"
                    if name not in body_names:
                        raise RuntimeError(
                            f"Contact sensor body_names is missing {name!r}; "
                            f"got {body_names[:6]}... (len={len(body_names)})."
                        )
                    grid[seg, side] = body_names.index(name)
            self._foot_idx_grid = grid

        # ── A. Root state (9 dims, slice 0:9) ──
        root_lin_vel_b      = art.data.root_lin_vel_b               # (E, 3)
        root_ang_vel_b      = art.data.root_ang_vel_b               # (E, 3)
        projected_gravity_b = art.data.projected_gravity_b          # (E, 3)

        # ── B. Compressed body yaw q, qdot (8 dims, slice 9:17) ──
        body_q   = art.data.joint_pos[:, ctrl.body_joint_ids]       # (E, 19)
        body_qd  = art.data.joint_vel[:, ctrl.body_joint_ids]       # (E, 19)
        bin_q  = torch.empty((E, 4), dtype=torch.float32, device=self.device)
        bin_qd = torch.empty((E, 4), dtype=torch.float32, device=self.device)
        for k, (lo, hi) in enumerate(self._body_yaw_bins):
            bin_q[:,  k] = body_q[:,  lo:hi].mean(dim=1)
            bin_qd[:, k] = body_qd[:, lo:hi].mean(dim=1)

        # ── Contact data (E, 38) ──
        if sensor is not None and self._foot_idx_grid is not None:
            net_forces = sensor.data.net_forces_w                   # (E, 38, 3)
            Fmag = net_forces.norm(dim=-1)                          # (E, 38)
            F_pair  = Fmag[:, self._foot_idx_grid]                  # (E, 19, 2)
            F_side0 = F_pair[..., 0]                                # (E, 19)
            F_side1 = F_pair[..., 1]                                # (E, 19)
            F_sum   = F_side0 + F_side1                             # (E, 19)
        else:
            zero19 = torch.zeros((E, _N_SEG), dtype=torch.float32, device=self.device)
            F_side0 = zero19
            F_side1 = zero19
            F_sum   = zero19

        # ── C. Per-segment contact load (19 dims, slice 17:36) ──
        contact_load = (F_sum / _F_SCALE).clamp_(min=0.0, max=2.0)  # (E, 19)

        # ── D. Contact distribution features (19 dims, slice 36:55) ──
        side_asym = (F_side0 - F_side1) / (F_sum + _F_EPS)          # (E, 19)
        # D.1 9-bin compressed side imbalance (mean over each bin).
        cd_side_bins = torch.empty((E, 9), dtype=torch.float32, device=self.device)
        for k, (lo, hi) in enumerate(_SIDE_ASYM_BINS):
            cd_side_bins[:, k] = side_asym[:, lo:hi].mean(dim=1)
        # D.2 Front-rear cumulative imbalance (4 dims).
        cd_front_rear = torch.empty((E, 4), dtype=torch.float32, device=self.device)
        for k, split in enumerate(_FRONT_REAR_SPLITS):
            front = F_sum[:, :split].sum(dim=1)
            rear  = F_sum[:, split:].sum(dim=1)
            cd_front_rear[:, k] = (front - rear) / (front + rear + _F_EPS)
        # D.3 Local load gradients (4 dims).
        A = F_sum[:, _LOCAL_REGIONS[0][0]:_LOCAL_REGIONS[0][1]].sum(dim=1)
        B = F_sum[:, _LOCAL_REGIONS[1][0]:_LOCAL_REGIONS[1][1]].sum(dim=1)
        C = F_sum[:, _LOCAL_REGIONS[2][0]:_LOCAL_REGIONS[2][1]].sum(dim=1)
        D = F_sum[:, _LOCAL_REGIONS[3][0]:_LOCAL_REGIONS[3][1]].sum(dim=1)
        cd_local = torch.stack(
            [
                (A - B) / (A + B + _F_EPS),
                (B - C) / (B + C + _F_EPS),
                (C - D) / (C + D + _F_EPS),
                (A - D) / (A + D + _F_EPS),
            ],
            dim=1,
        )                                                            # (E, 4)
        # D.4 Global distribution (2 dims).
        sum0 = F_side0.sum(dim=1)
        sum1 = F_side1.sum(dim=1)
        cd_global_side = (sum0 - sum1) / (sum0 + sum1 + _F_EPS)      # (E,)
        cd_front_rear_side_diff = (
            side_asym[:, _GLOBAL_SIDE_FRONT[0]:_GLOBAL_SIDE_FRONT[1]].mean(dim=1)
            - side_asym[:, _GLOBAL_SIDE_REAR[0]:_GLOBAL_SIDE_REAR[1]].mean(dim=1)
        )                                                            # (E,)
        cd_global = torch.stack([cd_global_side, cd_front_rear_side_diff], dim=1)
        contact_dist = torch.cat([cd_side_bins, cd_front_rear, cd_local, cd_global], dim=1)
        # contact_dist shape: (E, 9 + 4 + 4 + 2) = (E, 19)

        # ── E. Contact temporal derivative (19 dims, slice 55:74) ──
        dF_sum = F_sum - self._previous_F_sum
        contact_deriv = (dF_sum / _F_SCALE).clamp_(min=-2.0, max=2.0)
        # Store F_sum for the next obs step.
        self._previous_F_sum = F_sum.detach().clone()

        # ── Pass 14 (2026-05-17): obs-v2 windowed severity features ──
        # Computed ONLY when pass14_obs_v2_enabled=True. When OFF, the
        # entire block is skipped (parity guarantee with the 86-D obs).
        # When ON, 4 features are appended to the obs at slices 86:90;
        # a fifth feature (acc_z_rms_w) is logged but NOT appended.
        # Only run when the flag is ON AND the env is fully initialized.
        # super().__init__() in DirectRLEnv calls _get_observations
        # before our env's own __init__ post-super() block runs (gym
        # space build). At that early point, self.controller and the
        # pass14 buffers don't yet exist, and our new code would hang
        # or crash. We detect that state by checking for
        # `self.controller` (set immediately after super().__init__()
        # — see env.__init__ line ~165). If absent, return a zero
        # pass14_obs so the gym observation_space check succeeds with
        # the declared 90-D shape.
        if self.cfg.pass14_obs_v2_enabled and not hasattr(self, "controller"):
            pass14_obs = torch.zeros(
                (E, _PASS14_OBS_V2_N_FEATURES), dtype=torch.float32, device=self.device)
        elif self.cfg.pass14_obs_v2_enabled:
            # Lazy-init the pass14 buffers. They are also allocated at
            # the end of env.__init__, but the lazy path provides a
            # safety net in case _get_observations is called before
            # the unconditional allocation runs.
            if not hasattr(self, "_pass14_acc_fwd_buf"):
                self._pass14_W = int(getattr(
                    self.cfg, "pass14_obs_v2_window_steps", 30))
                self._pass14_acc_fwd_buf = torch.zeros(
                    (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
                self._pass14_acc_z_buf   = torch.zeros(
                    (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
                self._pass14_F_rate_buf  = torch.zeros(
                    (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
                self._pass14_F_max_buf   = torch.zeros(
                    (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
                self._pass14_entropy_buf = torch.zeros(
                    (self.num_envs, self._pass14_W), dtype=torch.float32, device=self.device)
                self._pass14_buf_idx     = torch.zeros(
                    (self.num_envs,), dtype=torch.long, device=self.device)
                self._pass14_buf_filled  = torch.zeros(
                    (self.num_envs,), dtype=torch.long, device=self.device)
                self._pass14_prev_v_fwd  = torch.zeros(
                    (self.num_envs,), dtype=torch.float32, device=self.device)
                self._pass14_prev_v_z    = torch.zeros(
                    (self.num_envs,), dtype=torch.float32, device=self.device)
                self._pass14_prev_F_total = torch.zeros(
                    (self.num_envs,), dtype=torch.float32, device=self.device)
                self._pass14_has_prev    = torch.zeros(
                    (self.num_envs,), dtype=torch.bool, device=self.device)
                self._pass14_last_raw    = torch.zeros(
                    (self.num_envs, 5), dtype=torch.float32, device=self.device)
                self._pass14_last_z      = torch.zeros(
                    (self.num_envs, 4), dtype=torch.float32, device=self.device)
            v_b      = art.data.root_lin_vel_b
            v_b_fwd  = v_b[:, 0]
            v_b_z    = v_b[:, 2]

            # All-sensor-body contact magnitudes (matches scripted_v3
            # aggregator in scripts/sanity/run_scripted_controller_v3.py:283-300
            # which uses ALL sensor bodies, not just the foot grid).
            if sensor is not None:
                F_per_body = sensor.data.net_forces_w.norm(dim=-1)  # (E, n_bodies)
                F_total38  = F_per_body.sum(dim=-1)                  # (E,)
                F_max38    = F_per_body.max(dim=-1).values           # (E,)
                _eps = 1e-12
                F_safe = torch.clamp(F_per_body, min=_eps)
                F_norm = F_safe / F_safe.sum(dim=-1, keepdim=True)
                H_e    = -(F_norm * torch.log(F_norm + _eps)).sum(dim=-1)
                n_bodies_f = float(F_per_body.shape[-1])
                # BISECT FIX 2026-05-18: replacing `np.log(...)` (numpy)
                # with `math.log(...)` (stdlib) eliminates a deadlock
                # that hung _get_observations inside Omniverse's CUDA
                # stream context. The numpy call appeared to trigger
                # a CUDA sync that conflicted with PhysX's stream;
                # math.log is pure C-Python and avoids it.
                entropy_i  = H_e / float(math.log(max(n_bodies_f, 2.0)))
            else:
                F_total38 = torch.zeros(E, dtype=torch.float32, device=self.device)
                F_max38   = torch.zeros(E, dtype=torch.float32, device=self.device)
                entropy_i = torch.zeros(E, dtype=torch.float32, device=self.device)

            # Instantaneous derivatives — zero on first post-reset step.
            dt_pol = float(self.cfg.sim.dt) * float(self.cfg.decimation)
            has_prev = self._pass14_has_prev
            zeros_E = torch.zeros_like(v_b_fwd)
            acc_fwd_i = torch.where(
                has_prev, (v_b_fwd - self._pass14_prev_v_fwd) / max(dt_pol, 1e-9), zeros_E)
            acc_z_i   = torch.where(
                has_prev, (v_b_z   - self._pass14_prev_v_z)   / max(dt_pol, 1e-9), zeros_E)
            F_rate_i  = torch.where(
                has_prev, torch.abs(F_total38 - self._pass14_prev_F_total) / max(dt_pol, 1e-9), zeros_E)

            # Persist for next step.
            self._pass14_prev_v_fwd.copy_(v_b_fwd.detach())
            self._pass14_prev_v_z.copy_(v_b_z.detach())
            self._pass14_prev_F_total.copy_(F_total38.detach())
            self._pass14_has_prev.fill_(True)

            # Scatter into per-env ring buffer.
            W_int = self._pass14_W
            batch_arange = torch.arange(E, dtype=torch.long, device=self.device)
            cur_idx = self._pass14_buf_idx
            self._pass14_acc_fwd_buf[batch_arange, cur_idx] = acc_fwd_i.to(torch.float32)
            self._pass14_acc_z_buf[batch_arange, cur_idx]   = acc_z_i.to(torch.float32)
            self._pass14_F_rate_buf[batch_arange, cur_idx]  = F_rate_i.to(torch.float32)
            self._pass14_F_max_buf[batch_arange, cur_idx]   = F_max38.to(torch.float32)
            self._pass14_entropy_buf[batch_arange, cur_idx] = entropy_i.to(torch.float32)
            self._pass14_buf_idx     = (self._pass14_buf_idx + 1) % W_int
            self._pass14_buf_filled  = torch.clamp(
                self._pass14_buf_filled + 1, max=W_int)

            # Windowed aggregates over the full ring (zeros in unfilled
            # slots; masked below via buf_filled).
            # 2026-05-18 BISECT FIX: torch.quantile was triggering a
            # hang under Pass 14 ON. Replaced with topk-based quantile
            # computation (sort-equivalent, no quantile-internal CUDA
            # sync that conflicted with Omniverse's mid-construction
            # CUDA stream).
            acc_fwd_rms_w_raw = torch.sqrt((self._pass14_acc_fwd_buf ** 2).mean(dim=-1))
            acc_z_rms_w_raw   = torch.sqrt((self._pass14_acc_z_buf   ** 2).mean(dim=-1))
            # torch.quantile(buf, 0.99, dim=-1) replaced by k-th largest
            # element where k = ceil((1-0.99)*W) = ceil(0.01*30) = 1.
            # (= the max). For p95: k = ceil(0.05*30) = 2 → 2nd largest.
            W_int_p99 = max(1, int(round((1.0 - 0.99) * W_int)))
            W_int_p95 = max(1, int(round((1.0 - 0.95) * W_int)))
            F_rate_p99_w_raw  = self._pass14_F_rate_buf.topk(W_int_p99, dim=-1).values[:, -1]
            F_max_p95_w_raw   = self._pass14_F_max_buf.topk(W_int_p95,  dim=-1).values[:, -1]
            entropy_std_w_raw = self._pass14_entropy_buf.std(dim=-1, unbiased=False)

            # Mask: output 0 (= pooled-mean in z-space) until the per-env
            # ring is fully filled (typically 1.44 s after reset at W=30).
            full_mask = (self._pass14_buf_filled >= W_int).to(torch.float32)

            # Z-score with pooled (mean, std) and clip to ±_CLIP_SIGMAS.
            def _z_then_clip(raw, mu, sigma):
                z = (raw - mu) / max(sigma, 1e-12)
                return torch.clamp(z,
                                   -_PASS14_OBS_V2_CLIP_SIGMAS,
                                   +_PASS14_OBS_V2_CLIP_SIGMAS)

            acc_fwd_rms_w_z = _z_then_clip(
                acc_fwd_rms_w_raw, _PASS14_OBS_V2_MEANS[0], _PASS14_OBS_V2_STDS[0]) * full_mask
            F_rate_p99_w_z  = _z_then_clip(
                F_rate_p99_w_raw,  _PASS14_OBS_V2_MEANS[1], _PASS14_OBS_V2_STDS[1]) * full_mask
            F_max_p95_w_z   = _z_then_clip(
                F_max_p95_w_raw,   _PASS14_OBS_V2_MEANS[2], _PASS14_OBS_V2_STDS[2]) * full_mask
            entropy_std_w_z = _z_then_clip(
                entropy_std_w_raw, _PASS14_OBS_V2_MEANS[3], _PASS14_OBS_V2_STDS[3]) * full_mask

            pass14_obs = torch.stack(
                [acc_fwd_rms_w_z, F_rate_p99_w_z, F_max_p95_w_z, entropy_std_w_z],
                dim=1).to(torch.float32)  # (E, 4)

            # Stash raw + z values for _get_rewards to log via extras["log"].
            # Order in _pass14_last_raw: [acc_fwd, acc_z (log-only), F_rate, F_max, entropy]
            self._pass14_last_raw[:, 0] = acc_fwd_rms_w_raw.detach()
            self._pass14_last_raw[:, 1] = acc_z_rms_w_raw.detach()
            self._pass14_last_raw[:, 2] = F_rate_p99_w_raw.detach()
            self._pass14_last_raw[:, 3] = F_max_p95_w_raw.detach()
            self._pass14_last_raw[:, 4] = entropy_std_w_raw.detach()
            self._pass14_last_z[:, 0]   = acc_fwd_rms_w_z.detach()
            self._pass14_last_z[:, 1]   = F_rate_p99_w_z.detach()
            self._pass14_last_z[:, 2]   = F_max_p95_w_z.detach()
            self._pass14_last_z[:, 3]   = entropy_std_w_z.detach()
        else:
            pass14_obs = None

        # ── Pass 19 (2026-05-21) — observation enrichment ──
        # Compute 155 enriched-channel features when EITHER pass19 flag
        # is True. When OFF: entire block is skipped (parity guarantee).
        # Memory: features stashed onto self._pass19_last_* for trace
        # logging in _get_rewards; tensor obs slice appended to obs_list
        # only when pass19_obs_extension_enabled.
        _p19_trace_on = bool(self.cfg.pass19_extended_trace_enabled)
        _p19_obs_on   = bool(self.cfg.pass19_obs_extension_enabled)
        if (_p19_trace_on or _p19_obs_on) and hasattr(self, "controller"):
            # ── Resolve the 19 per-segment body ids on first call ──
            if self._pass19_seg_body_ids is None and sensor is not None:
                # Use the articulation's body_names (same source as the
                # K=12 _PASS17_N_SEG_BODY=21 resolver, but we only take
                # the first 19 link_body_X for anatomical consistency
                # with the 19-segment foot grid).
                body_names_art = list(art.data.body_names)
                seg_ids = []
                for seg_i in range(_N_SEG):
                    name = f"link_body_{seg_i}"
                    if name not in body_names_art:
                        raise RuntimeError(
                            f"Articulation body_names missing {name!r} "
                            f"(needed for pass19 per-segment pitch/roll). "
                            f"Got {body_names_art[:6]}... "
                            f"(len={len(body_names_art)})."
                        )
                    seg_ids.append(body_names_art.index(name))
                self._pass19_seg_body_ids = torch.tensor(
                    seg_ids, dtype=torch.long, device=self.device,
                )
            # ── Per-foot force vector + magnitude (38,3) and (38,) ──
            # Re-use the (E, 38, 3) net_forces tensor already fetched
            # above; reshape via _foot_idx_grid to anatomical (seg, side)
            # then flatten to length-38. Fallback to zeros when sensor
            # is not yet available (rare; init-time guard).
            if sensor is not None and self._foot_idx_grid is not None:
                flat_idx = self._foot_idx_grid.reshape(-1)              # (38,)
                p19_foot_vec = net_forces[:, flat_idx, :]               # (E, 38, 3)
                p19_foot_mag = Fmag[:, flat_idx]                        # (E, 38)
                _p19_contact_state = (p19_foot_mag > _PASS17_F_THRESHOLD_N).to(torch.float32)
            else:
                p19_foot_vec = torch.zeros(
                    (E, _PASS17_N_FOOT, 3), dtype=torch.float32, device=self.device)
                p19_foot_mag = torch.zeros(
                    (E, _PASS17_N_FOOT), dtype=torch.float32, device=self.device)
                _p19_contact_state = torch.zeros(
                    (E, _PASS17_N_FOOT), dtype=torch.float32, device=self.device)
            # ── Per-foot contact duration counter ──
            # In contact: counter += 1; out of contact: counter = 0.
            # Persists across steps via self._pass19_contact_dur_counter;
            # reset to 0 per env in _reset_idx.
            _in_contact = _p19_contact_state.to(torch.bool)
            self._pass19_contact_dur_counter = torch.where(
                _in_contact,
                self._pass19_contact_dur_counter + 1.0,
                torch.zeros_like(self._pass19_contact_dur_counter),
            ).clamp_(max=1e6)
            # ── Per-segment pitch + roll (19,) each ──
            # Use body_link_quat_w (same source as K=12). Slice to the
            # 19 anatomical body segments.
            if self._pass19_seg_body_ids is not None:
                # art.data.body_link_quat_w is wxyz convention per IsaacLab
                # docs and matches the existing _quat_wxyz_to_roll_pitch.
                seg_quat = art.data.body_link_quat_w[:, self._pass19_seg_body_ids, :]  # (E, 19, 4)
                p19_seg_roll, p19_seg_pitch = _quat_wxyz_to_roll_pitch(seg_quat)       # each (E, 19)
            else:
                p19_seg_roll  = torch.zeros((E, _N_SEG), dtype=torch.float32, device=self.device)
                p19_seg_pitch = torch.zeros((E, _N_SEG), dtype=torch.float32, device=self.device)
            # ── proj_g_z + 1st/2nd time-derivative (5-tap finite diff) ──
            # Push current proj_g_z (already computed at line ~893,
            # projected_gravity_b[:, 2]) into the rolling history.
            _p19_dt = float(self.cfg.sim.dt) * float(self.cfg.decimation)
            _p19_dt = max(_p19_dt, 1e-6)
            _p19_pgz_now = projected_gravity_b[:, 2].to(torch.float32)              # (E,)
            # Shift history left and append newest.
            self._pass19_proj_g_z_hist[:, :-1] = self._pass19_proj_g_z_hist[:, 1:].clone()
            self._pass19_proj_g_z_hist[:, -1]  = _p19_pgz_now.detach()
            self._pass19_proj_g_z_filled = torch.clamp(self._pass19_proj_g_z_filled + 1, max=5)
            # 5-tap central derivative coefficients (Savitzky-Golay-like):
            #   f'  ≈ (1·f[0] - 8·f[1] + 0·f[2] + 8·f[3] - 1·f[4]) / (12·dt)
            #   f'' ≈ (-1·f[0] + 16·f[1] - 30·f[2] + 16·f[3] - 1·f[4]) / (12·dt²)
            # Computed using the LATEST 5 samples; when < 5 are filled,
            # output 0 to avoid spurious spikes after reset.
            _p19_full = (self._pass19_proj_g_z_filled >= 5).to(torch.float32)
            _h = self._pass19_proj_g_z_hist
            _p19_dpgz_dt = (
                1.0 * _h[:, 0] - 8.0 * _h[:, 1] + 0.0 * _h[:, 2]
                + 8.0 * _h[:, 3] - 1.0 * _h[:, 4]
            ) / (12.0 * _p19_dt) * _p19_full
            _p19_d2pgz_dt2 = (
                -1.0 * _h[:, 0] + 16.0 * _h[:, 1] - 30.0 * _h[:, 2]
                + 16.0 * _h[:, 3] - 1.0 * _h[:, 4]
            ) / (12.0 * _p19_dt * _p19_dt) * _p19_full
            p19_pgz_triplet = torch.stack([_p19_pgz_now, _p19_dpgz_dt, _p19_d2pgz_dt2], dim=1)  # (E, 3)
            # ── Stash raw features for trace logging in _get_rewards ──
            self._pass19_last_foot_force_vector.copy_(p19_foot_vec.detach())
            self._pass19_last_foot_force_mag.copy_(p19_foot_mag.detach())
            self._pass19_last_foot_contact_state.copy_(_p19_contact_state.detach())
            self._pass19_last_foot_contact_dur.copy_(self._pass19_contact_dur_counter.detach())
            self._pass19_last_seg_pitch.copy_(p19_seg_pitch.detach())
            self._pass19_last_seg_roll.copy_(p19_seg_roll.detach())
            self._pass19_last_proj_g_z_triplet.copy_(p19_pgz_triplet.detach())
            # ── Build the 155-D obs slice (only used if obs-extension flag is on) ──
            if _p19_obs_on:
                _p19_dur_W = float(self.cfg.pass19_contact_dur_normalize_W)
                _p19_dur_W = max(_p19_dur_W, 1.0)
                _dur_norm = (self._pass19_contact_dur_counter / _p19_dur_W).clamp_(max=1.0)
                pass19_obs = torch.cat([
                    p19_foot_mag,            # (E, 38)  slice 86:124
                    _p19_contact_state,      # (E, 38)  slice 124:162
                    _dur_norm,               # (E, 38)  slice 162:200
                    p19_seg_pitch,           # (E, 19)  slice 200:219
                    p19_seg_roll,            # (E, 19)  slice 219:238
                    p19_pgz_triplet,         # (E, 3)   slice 238:241
                ], dim=1).to(torch.float32)
            else:
                pass19_obs = None
        else:
            pass19_obs = None

        # ── Pass 19 β-light (2026-05-29): 3 scalar substrate channels ──
        # When p4_pre_runtime_torque_qdot_enabled is True (AND pass19 is on,
        # asserted in __init__), append power_j04, tau_abs_j29, qdot_j30 at
        # slice 241:244 via runtime art.data reads. Hard-coded joint indices
        # per offline build_channel_list zero-based convention; PC3 verifies
        # alignment with art.data.joint_names at runtime.
        if bool(self.cfg.p4_pre_runtime_torque_qdot_enabled):
            _art = self.robot
            _tau = _art.data.applied_torque
            _qd  = _art.data.joint_vel
            if _tau is None or _qd is None:
                raise RuntimeError(
                    "p4_pre_runtime_torque_qdot_enabled is True but "
                    "art.data.applied_torque or art.data.joint_vel is None.")
            _J_POWER, _J_TORQUE, _J_QDOT = 4, 29, 30  # zero-based per offline NPZ
            _power_j04   = torch.abs(_tau[:, _J_POWER] * _qd[:, _J_POWER]).unsqueeze(1)
            _tau_abs_j29 = torch.abs(_tau[:, _J_TORQUE]).unsqueeze(1)
            _qdot_j30    = _qd[:, _J_QDOT].unsqueeze(1)  # NOT absolute (per offline)
            p4_pre_obs   = torch.cat(
                [_power_j04, _tau_abs_j29, _qdot_j30], dim=1).to(torch.float32)
        else:
            p4_pre_obs = None

        # ── F. Previous global action (6 dims, slice 74:80) ──
        prev_action_obs = self._prev_actions.to(dtype=torch.float32)  # (E, 6) in global_6d

        # ── G. Global CPG phase sin/cos (2 dims, slice 80:82) ──
        if ctrl.body_phases is not None and ctrl._cpg_initialized:
            phi = ctrl.body_phases[:, 0].to(dtype=torch.float32)      # head segment
            cpg_obs = torch.stack([torch.sin(phi), torch.cos(phi)], dim=1)
        else:
            cpg_obs = torch.zeros((E, 2), dtype=torch.float32, device=self.device)
            cpg_obs[:, 1] = 1.0  # cos(0) = 1

        # ── H. Global contact summaries (4 dims, slice 82:86) ──
        cs_front  = contact_load[:, _GLOBAL_FRONT[0]:_GLOBAL_FRONT[1]].mean(dim=1)
        cs_middle = contact_load[:, _GLOBAL_MID[0]:_GLOBAL_MID[1]].mean(dim=1)
        cs_rear   = contact_load[:, _GLOBAL_REAR[0]:_GLOBAL_REAR[1]].mean(dim=1)
        cs_total  = contact_load.mean(dim=1)
        contact_summary = torch.stack([cs_front, cs_middle, cs_rear, cs_total], dim=1)

        # ── Concatenate ──
        obs_list = [
            root_lin_vel_b.to(dtype=torch.float32),               # 0:3
            root_ang_vel_b.to(dtype=torch.float32),               # 3:6
            projected_gravity_b.to(dtype=torch.float32),          # 6:9
            bin_q,                                                # 9:13
            bin_qd,                                               # 13:17
            contact_load.to(dtype=torch.float32),                 # 17:36
            contact_dist.to(dtype=torch.float32),                 # 36:55
            contact_deriv.to(dtype=torch.float32),                # 55:74
            prev_action_obs,                                      # 74:80
            cpg_obs,                                              # 80:82
            contact_summary.to(dtype=torch.float32),              # 82:86
        ]
        # Pass 14 (2026-05-17): optional 4 windowed severity features.
        # Slices 86:90 when pass14_obs_v2_enabled=True; absent otherwise.
        if pass14_obs is not None:
            obs_list.append(pass14_obs)                            # 86:90
        # Pass 19 (2026-05-21): optional 155 enriched-sensor features.
        # Slices 86:241 when pass19_obs_extension_enabled=True; absent
        # otherwise. Mutually exclusive with pass14_obs_v2 (asserted in
        # __init__).
        if pass19_obs is not None:
            obs_list.append(pass19_obs)                            # 86:241
        # P4-pre β-light (2026-05-29): 3 scalar runtime channels for v12 obs.
        # Slices 241:244 when both pass19_obs and p4_pre_runtime_torque_qdot on.
        if p4_pre_obs is not None:
            obs_list.append(p4_pre_obs)                            # 241:244
        obs = torch.cat(obs_list, dim=1)
        # Defensive: ensure no NaN/Inf escapes (the controller's parity
        # path is finite at default scales; contact / phase paths above
        # use _F_EPS / clamps; if any path slipped, we want a clear NaN
        # rather than silent garbage downstream).
        # We do NOT clamp here — preserving the raw values — but a
        # downstream sanity check (check_stage_b_observation.py)
        # asserts finiteness explicitly.
        # Stage E oracle-terrain append (DO_NOT_SHIP; default OFF -> skipped -> identical).
        if bool(getattr(self.cfg, "oracle_terrain_obs", False)):
            obs = torch.cat([obs, self._oracle_obs_slice(self.num_envs)], dim=1)
        return {"policy": obs}

    # =========================================================================
    # Stage E (2026-07-12) windowed-terrain-obs (WTO) helpers -- gated OFF by default
    # =========================================================================
    def _nwo_accumulate_substep(self) -> None:
        """Push ONE physics-substep's per-segment contact + IMU signals into the fixed
        wall-clock ring buffer. Called once per physics substep from _apply_action ONLY
        when cfg.new_windowed_obs. The ring persists across policy steps (window decoupled
        from decimation) and is reduced at the policy step by _get_windowed_observation.
        Per-env ring index + filled count mirror the Pass-14 async-reset pattern (zeroed
        per env in _reset_idx)."""
        art = self.robot
        if self._nwo_sensor is None:
            s = None
            if hasattr(self.scene, "sensors"):
                s = self.scene.sensors.get("contact_sensor", None)
            if s is None:
                try:
                    s = self.scene["contact_sensor"]
                except Exception:  # noqa: BLE001
                    s = None
            self._nwo_sensor = s
        sensor = self._nwo_sensor
        if sensor is not None and self._foot_idx_grid is None:
            body_names = list(sensor.body_names)
            grid = torch.zeros((_N_SEG, 2), dtype=torch.long, device=self.device)
            for seg in range(_N_SEG):
                for side in range(2):
                    name = f"foot_{seg}_{side}"
                    if name not in body_names:
                        return  # grid not resolvable yet; skip this substep (obs path will raise)
                    grid[seg, side] = body_names.index(name)
            self._foot_idx_grid = grid
        if sensor is None or self._foot_idx_grid is None:
            return
        Fmag = sensor.data.net_forces_w.norm(dim=-1)                    # (E, 38)
        F_sum = Fmag[:, self._foot_idx_grid].sum(dim=-1)               # (E, 19)
        contact_load = (F_sum / _F_SCALE).clamp(min=0.0, max=2.0)      # (E, 19)
        total_load = (F_sum.sum(dim=1) / _F_SCALE)                     # (E,)
        omega_mag = art.data.root_ang_vel_b.norm(dim=-1)              # (E,)
        idxE = torch.arange(self.num_envs, dtype=torch.long, device=self.device)
        pos = self._nwo_pos
        self._nwo_cbuf[idxE, pos, :] = contact_load.to(torch.float32)
        self._nwo_obuf[idxE, pos]    = omega_mag.to(torch.float32)
        self._nwo_tbuf[idxE, pos]    = total_load.to(torch.float32)
        self._nwo_pos = (pos + 1) % self._nwo_W
        self._nwo_filled = torch.clamp(self._nwo_filled + 1, max=self._nwo_W)

    def _get_topology_observation(self) -> dict:
        """Build the 34-D default-off topology-aware observation.

        The contact spectrum is the first three orthonormal spatial DCT
        coefficients of each 19-segment temporal-statistic profile plus the
        four existing global window summaries.  The spine spectrum is computed
        from link_body_0..20 positions after translation removal, inverse root
        rotation, chord detrending, and body-length normalization.
        """
        E = self.num_envs
        dev = self.device
        if not hasattr(self, "controller") or not hasattr(self, "_nwo_cbuf"):
            base = torch.zeros((E, _TOPO_OBS_DIM), dtype=torch.float32, device=dev)
            if bool(getattr(self.cfg, "oracle_terrain_obs", False)):
                base = torch.cat([base, self._oracle_obs_slice(E)], dim=1)
            return {"policy": base}

        art = self.robot
        ctrl = self.controller
        realized_wave = (
            ctrl.wave_number_scale_runtime * float(ctrl.n_wave_scalar)
        ).to(torch.float32).unsqueeze(1)
        if ctrl.body_phases is not None and ctrl._cpg_initialized:
            phi = ctrl.body_phases[:, 0].to(torch.float32)
            cpg = torch.stack([torch.sin(phi), torch.cos(phi)], dim=1)
        else:
            cpg = torch.zeros((E, 2), dtype=torch.float32, device=dev)
            cpg[:, 1] = 1.0
        root_state = torch.cat(
            [
                art.data.root_lin_vel_b.to(torch.float32),
                art.data.root_ang_vel_b.to(torch.float32) * _TOPO_ROOT_ANG_SCALE,
                art.data.projected_gravity_b.to(torch.float32),
            ],
            dim=1,
        )

        W = self._nwo_W
        full = (self._nwo_filled >= W).to(torch.float32).unsqueeze(1)
        cbuf = self._nwo_cbuf
        obuf = self._nwo_obuf
        tbuf = self._nwo_tbuf
        c_med = torch.sort(cbuf, dim=1).values[:, W // 2, :]
        o_med = torch.sort(obuf, dim=1).values[:, W // 2]
        t_med = torch.sort(tbuf, dim=1).values[:, W // 2]
        c_std = cbuf.std(dim=1, unbiased=False)
        c_spike = (
            cbuf > (_NWO_SPIKE_K * c_med).unsqueeze(1)
        ).to(torch.float32).mean(dim=1)
        o_rms = torch.sqrt((obuf ** 2).mean(dim=1) + 1e-12)
        # Crest factor is unbounded and saturated the policy clip even on flat ground.
        global4 = torch.stack(
            [
                torch.log1p(obuf.max(dim=1).values / (o_rms + 1e-9)),
                (obuf > (_NWO_SPIKE_K * o_med).unsqueeze(1))
                .to(torch.float32).mean(dim=1),
                tbuf.std(dim=1, unbiased=False),
                (tbuf > (_NWO_SPIKE_K * t_med).unsqueeze(1))
                .to(torch.float32).mean(dim=1),
            ],
            dim=1,
        )
        contact10 = torch.cat(
            [
                c_std @ self._topo_dct19.T,
                c_spike @ self._topo_dct19.T,
                global4,
            ],
            dim=1,
        ) * full

        if self._topo_main_body_ids is None:
            names = list(art.data.body_names)
            ids = []
            for index in range(_TOPO_MAIN_N):
                name = f"link_body_{index}"
                if name not in names:
                    raise RuntimeError(
                        f"topology_obs_v2 requires {name!r}; got "
                        f"{names[:6]}... (len={len(names)})")
                ids.append(names.index(name))
            self._topo_main_body_ids = torch.tensor(
                ids, dtype=torch.long, device=dev)
        main_w = art.data.body_pos_w[:, self._topo_main_body_ids, :].to(torch.float32)
        rel_w = main_w - main_w[:, :1, :]
        rel_b = _quat_wxyz_rotate_inverse(
            art.data.root_quat_w.to(torch.float32), rel_w)
        s = torch.linspace(
            0.0, 1.0, _TOPO_MAIN_N, dtype=torch.float32, device=dev
        ).view(1, _TOPO_MAIN_N, 1)
        chord = rel_b[:, -1:, :]
        chord_len = chord[:, 0, :].norm(dim=1).clamp_min(1e-4)
        residual = (rel_b - s * chord) / chord_len[:, None, None]
        y_modes = residual[:, :, 1] @ self._topo_dct21.T
        z_modes = residual[:, :, 2] @ self._topo_dct21.T
        curvature = torch.diff(residual, n=2, dim=1)
        edges = torch.diff(rel_b, dim=1)
        edge_len = edges.norm(dim=2)
        edge_mean = edge_len.mean(dim=1).clamp_min(1e-6)
        spine12 = torch.cat(
            [
                y_modes,
                z_modes,
                torch.sqrt((curvature[:, :, 1] ** 2).mean(dim=1)).unsqueeze(1),
                torch.sqrt((curvature[:, :, 2] ** 2).mean(dim=1)).unsqueeze(1),
                (edge_len.std(dim=1, unbiased=False) / edge_mean).unsqueeze(1),
                (edge_len.sum(dim=1) / chord_len).unsqueeze(1),
            ],
            dim=1,
        )
        obs = torch.cat([realized_wave, cpg, root_state, contact10, spine12], dim=1)
        if obs.shape[1] != _TOPO_OBS_DIM:
            raise RuntimeError(
                f"topology_obs_v2 width {obs.shape[1]} != {_TOPO_OBS_DIM}")
        if not torch.isfinite(obs).all():
            raise RuntimeError("topology_obs_v2 produced non-finite values")
        if bool(getattr(self.cfg, "oracle_terrain_obs", False)):
            obs = torch.cat([obs, self._oracle_obs_slice(E)], dim=1)
        return {"policy": obs.to(torch.float32)}

    def _get_windowed_observation(self) -> dict:
        """Build the ~51-D self-state + WINDOWED-terrain obs that REPLACES the 86-D obs
        when cfg.new_windowed_obs. Reduces the fixed wall-clock ring (filled by
        _nwo_accumulate_substep over the last windowed_obs_window_seconds) into per-channel
        windowed statistics. Windowed features are masked to 0 until the per-env ring is
        fully filled (first window after reset); the self-state is always valid."""
        E = self.num_envs
        dev = self.device
        # Early-init guard: super().__init__ calls _get_observations before controller /
        # ring buffers exist (gym space build). Return zeros at the declared dim.
        if not hasattr(self, "controller") or not hasattr(self, "_nwo_cbuf"):
            base = torch.zeros((E, _NWO_OBS_DIM), dtype=torch.float32, device=dev)
            if bool(getattr(self.cfg, "oracle_terrain_obs", False)):
                base = torch.cat([base, self._oracle_obs_slice(E)], dim=1)
            return {"policy": base}
        art = self.robot
        ctrl = self.controller
        # -- SELF-STATE (9) --
        realized_wave = (ctrl.wave_number_scale_runtime
                         * float(ctrl.n_wave_scalar)).to(torch.float32).unsqueeze(1)   # (E,1) applied_abs
        if ctrl.body_phases is not None and ctrl._cpg_initialized:
            phi = ctrl.body_phases[:, 0].to(torch.float32)
            cpg = torch.stack([torch.sin(phi), torch.cos(phi)], dim=1)                 # (E,2)
        else:
            cpg = torch.zeros((E, 2), dtype=torch.float32, device=dev); cpg[:, 1] = 1.0
        ang = art.data.root_ang_vel_b.to(torch.float32)                                # (E,3)
        grav = art.data.projected_gravity_b.to(torch.float32)                          # (E,3)
        self_state = torch.cat([realized_wave, cpg, ang, grav], dim=1)                 # (E,9)
        # -- WINDOWED TERRAIN (42) over the fixed wall-clock ring --
        W = self._nwo_W
        full = (self._nwo_filled >= W).to(torch.float32).unsqueeze(1)                  # (E,1)
        cbuf = self._nwo_cbuf; obuf = self._nwo_obuf; tbuf = self._nwo_tbuf
        # sort-based medians (avoid torch.quantile CUDA-sync hang seen under Omniverse).
        c_med = torch.sort(cbuf, dim=1).values[:, W // 2, :]                           # (E,19)
        o_med = torch.sort(obuf, dim=1).values[:, W // 2]                              # (E,)
        t_med = torch.sort(tbuf, dim=1).values[:, W // 2]                              # (E,)
        c_std = cbuf.std(dim=1, unbiased=False)                                        # (E,19)
        c_spike = (cbuf > (_NWO_SPIKE_K * c_med).unsqueeze(1)).to(torch.float32).mean(dim=1)   # (E,19)
        o_rms = torch.sqrt((obuf ** 2).mean(dim=1) + 1e-12)                            # (E,)
        o_peak = obuf.max(dim=1).values                                               # (E,)
        o_crest = (o_peak / (o_rms + 1e-9)).unsqueeze(1)                               # (E,1)
        o_spike = (obuf > (_NWO_SPIKE_K * o_med).unsqueeze(1)).to(torch.float32).mean(dim=1).unsqueeze(1)  # (E,1)
        t_std = tbuf.std(dim=1, unbiased=False).unsqueeze(1)                           # (E,1)
        t_spike = (tbuf > (_NWO_SPIKE_K * t_med).unsqueeze(1)).to(torch.float32).mean(dim=1).unsqueeze(1)  # (E,1)
        windowed = torch.cat([c_std, c_spike, o_crest, o_spike, t_std, t_spike], dim=1) * full  # (E,42)
        obs = torch.cat([self_state, windowed], dim=1).to(torch.float32)               # (E,51)
        if bool(getattr(self.cfg, "oracle_terrain_obs", False)):
            obs = torch.cat([obs, self._oracle_obs_slice(E)], dim=1)
        return {"policy": obs}

    def _oracle_obs_slice(self, E: int) -> torch.Tensor:
        """Stage E DO_NOT_SHIP diagnostic: the ground-truth terrain descriptor to APPEND
        to the obs (zeros if unset). Upper-bound probe only -- NEVER shipped/warm-started."""
        d = getattr(self, "_oracle_terrain_descriptor", None)
        if d is None:
            return torch.zeros((E, int(getattr(self.cfg, "oracle_terrain_obs_dim", 3))),
                               dtype=torch.float32, device=self.device)
        return d.to(torch.float32)

    def set_oracle_terrain_descriptor(self, desc) -> None:
        """Stage E DO_NOT_SHIP: set the per-env (E, oracle_dim) ground-truth terrain
        descriptor (normalized lambda, amp, a004-optimal wave#) appended when
        cfg.oracle_terrain_obs is ON. Caller supplies it from spawn labels."""
        self._oracle_terrain_descriptor = desc.to(torch.float32).to(self.device)

    # =========================================================================
    # Reward + termination (Stage B-v0, Pass 4)
    # =========================================================================
    # Minimal reward + failure detection per the project spec:
    #     r = w_v · v_forward + w_alive
    #         − w_ori (roll² + pitch²)
    #         − w_ang ‖root_ang_vel_b‖²
    #         − w_act ‖a‖² − w_rate ‖a − a_prev‖²
    #         − w_tau mean|τ|
    #         − w_fail · fail
    # Forward direction = -world_x  (verified empirically from the
    # rough λ=18 mm walking demo: root_x decreased over time at ~17 mm/s).
    # Failure = any of:
    #     - root_z < FAIL_ROOT_Z_M  (fell through floor / collapsed)
    #     - |projected_gravity_b[2]| < cos(75°)  (tipped > 75°)
    #     - any non-finite obs / root state
    # We use projected_gravity_b (cheap, no Euler decomposition) instead
    # of computing roll/pitch separately. roll² + pitch² in the reward
    # is replaced by the equivalent (1 - g_b_z)² which has the same
    # shape near upright (small-angle) and is bounded at large tips.
    # 2026-05-07 Pass 6 (audit-driven rebalance after the
    # check_stage_b_reward_magnitudes audit revealed that the Pass 5
    # weight set produced NEGATIVE total reward at the zero-action
    # baseline gait — i.e., even a perfect policy reproducing the
    # hand-tuned CPG would lose reward, so PPO had no gradient toward
    # locomotion). Audit findings (filtered means, per policy step):
    #   - At zero action: forward=+0.034, alive=+0.020, ang=-0.169
    #     (5x forward!), all other terms ~0. Net = -0.116/step.
    #     The angular-velocity penalty alone dominated baseline gait.
    #   - At trained saturation: act=-0.798 (161x fwd), rate=-6.476
    #     (1304x fwd!), ang=-4.084 (822x fwd), forward=+0.005.
    #     Penalty/reward ratio ~2300:1, total reward -11.5/step.
    #   - Action saturation: 99.9% of trained policy steps had
    #     |raw_a|_max > 2 (deep tanh saturation), 99.7% > 3.
    #     The raw-action-space rate penalty squared the saturation
    #     amplitude, dominating everything else.
    #
    # Pass 6 changes (each justified by the audit numbers, not by
    # arbitrary scaling):
    #   w_ang  0.05  -> 0.01  : at zero action E[||w_b||^2] = 3.38;
    #                            need w_ang*3.38 < (forward+alive)
    #                            = 0.054 -> w_ang < 0.016. We pick
    #                            0.01 (60% of ceiling) so baseline
    #                            ang penalty = -0.034 leaves +0.020
    #                            net per step.
    #   w_act  0.008 -> 0.002 : trained ||a||^2 ~ 100 due to clip 5.
    #                            Old 0.008*100 = -0.80 swamped fwd by
    #                            161x. New 0.002*100 = -0.20, still
    #                            40x forward (plenty of deterrent)
    #                            but no longer crushing.
    #   rate term: RAW -> SCALED: was ||raw_a - raw_a_prev||^2 in
    #              [-5, +5]^6 space. At trained ~104. After switching
    #              to ||u_t - u_{t-1}||^2 in scaled-physical-knob
    #              space (range e.g. [0.3, 3.0] for body_imp), trained
    #              value ~ 3.13 — a 33x reduction. We KEEP w_rate=0.05.
    #              At zero action u stays constant at 1.0 -> rate=0.
    #              Implementation: env caches self._prev_scaled_action
    #              (E, 6), reset to 1.0 in _reset_idx so the post-reset
    #              first-step rate is zero.
    #   All other weights unchanged.
    #
    # Predicted Pass 6 per-step at zero action: +0.034 + 0.020 - 0.034
    #   = +0.020/step (vs Pass 5 -0.116/step).
    # Predicted Pass 6 per-step at trained-style saturation:
    #   +0.005 + 0.020 - 0.107 - 0.82 - 0.20 - 0.21 - 0.012 - 0.006
    #   - 0.027 = -1.36/step (vs Pass 5 -11.5/step).
    # The slope from broken-saturation toward baseline-gait now points
    # toward locomotion with usable magnitude.
    _REWARD_W_V         = 1.0
    _REWARD_W_ALIVE     = 0.02
    _REWARD_W_ORI       = 0.5
    _REWARD_W_ANG       = 0.01      # Pass 6: 0.05 -> 0.01 (audit-derived)
    _REWARD_W_ACT       = 0.002     # Pass 6: 0.008 -> 0.002 (audit-derived)
    _REWARD_W_RATE      = 0.10      # M2 2026-05-15: R1 doubled for chatter at 40/80 (was 0.05)
    _REWARD_W_TAU       = 0.003
    _REWARD_W_Z_VEL     = 0.03
    _REWARD_W_IMP       = 0.01
    _REWARD_W_WAVE      = 0.001     # M2 2026-05-15: R1 reduced wave_num=1.0 anchor (was 0.005)
    _REWARD_W_FAIL      = 3.0
    # ── Pass 10 (2026-05-08): mechanical-power / effort-aware shaping ──
    # NEW reward term, calibrated 2026-05-08 against zero-action baseline
    # under Pass 9b transition arena (see
    # outputs/sanity/pass10_effort_calibration.txt and
    # outputs/analysis/pass10_calibration_review.md).
    #
    #   r_effort = - W_EFFORT * sigmoid((v_forward - V_MID) / V_SIGMA)
    #              * sum_{actuated DOFs} |tau_i * qdot_i|
    #
    # NOTE on naming: this is an EFFORT-AWARE / MECHANICAL-POWER term, not
    # a literal per-step Cost-of-Transport. True CoT is mechanical work
    # divided by displacement; per-step CoT is numerically unstable when
    # displacement → 0. This per-step power penalty creates CoT-like
    # pressure when combined with the existing forward-velocity reward
    # W_V * v_forward — together they approximate CoT minimization in
    # expectation while remaining PPO-stable.
    #
    # Calibration: at zero-action baseline (gated power 0.155 W,
    # v_forward 0.0278 m/s), this term contributes ~ -0.0056 / step
    # (= 1/5 of the forward term). Trained Pass 8 receives ~ -0.020 /
    # step (3.6× stronger), providing the gradient toward efficient gait.
    # Do-nothing safety: under the FULL Pass 10 reward (existing 11
    # terms + this term), all four diagnostic conditions (low_body_amp,
    # low_leg_amp, low_freq, low_impedance) score strictly worse than
    # zero-action baseline (W_IMP and W_ACT regularizers close the
    # marginal low-impedance cheat the simplified check flagged).
    # Pass 10 v2 (2026-05-08): halved from 0.0358 → 0.018 after v1
    # 15-epoch smoke showed v_forward dropped to 74% of baseline and
    # effort/m WORSENED from Pass 8's 3.08× to 4.09× baseline. The 0.0358
    # calibration (1/5 of forward at zero-action) was too strong at the
    # active operating point because effort scales ∝ amp·freq·stiffness
    # (steeper than v_forward), making "slow down" an easier PPO descent
    # than "find efficient gait". 0.018 ≈ 1/10 of forward at baseline,
    # bringing the trained-operating-point effort/forward ratio from
    # ~80% to ~40%, restoring forward-term dominance.
    _REWARD_W_EFFORT    = 0.018     # Pass 10 v2: halved from 0.0358
    _GATE_V_MID         = 0.005     # m/s — sigmoid midpoint
    _GATE_V_SIGMA       = 0.005     # m/s — sigmoid scale

    # ── Pass 11 (2026-05-09): windowed CoT hinge ──
    # Calibration source:
    #   outputs/sanity/pass11_cot_calibration.txt  (H=42 row)
    #   outputs/analysis/pass11_calibration_review.md
    # At zero-action baseline (H=42): CoT_window mean = 285.333.
    # At Pass 8 trained (H=42):       CoT_window mean = 1061.902.
    # Threshold = α · CoT_zero_mean = 1.5 · 285.333 = 428.0.
    # excess at trained = 1061.902 − 428.0 = 633.902.
    # gate_x at trained mean = 0.9313.
    # target r_cot at trained = −0.005 / step.
    # → W_COT = 0.005 / (0.9313 · 633.902) ≈ 8.470e−6  (mean-based formula).
    # 2026-05-09: empirical re-calibration after first audit revealed
    # the mean-based formula under-shoots realized r_cot by ~2.43× due
    # to (1) hinge non-linearity over the heavy-tailed CoT distribution
    # — Jensen's inequality on max(0, x−θ) — and (2) the buf_filled
    # mask suppressing gate_x for the first H=42 steps after each env
    # reset (≈ 25% of an 8 s episode at episode_length_s=8.0). Realized
    # r_cot at trained was −0.00206 vs target −0.005. Scaling factor
    # 0.005 / 0.00206 = 2.43 gives W_COT_v2 = 8.470e−6 · 2.43 ≈ 2.057e−5.
    # See outputs/analysis/pass11_audit_review.md for full diagnosis.
    _REWARD_W_COT       = 2.057e-5  # Pass 11 v1.1: empirical-calibrated weight
    _COT_H_STEPS        = 42        # window length (≈ 2 gait cycles at 1 Hz)
    _COT_THRESHOLD      = 428.0     # = 1.5 · mean(CoT_window) at zero-action, H=42
    _COT_DX_MIN         = 0.010     # m, gate threshold on dx_window
    _COT_DX_SIGMA       = 0.005     # m, gate sharpness
    _COT_EPSILON_E      = 1e-9      # J, denominator FP-safety floor
    _COT_BODY_MASS_KG   = 0.005     # centipede mass (matches calibration)
    _COT_GRAVITY        = 9.81      # m/s², standard

    # ── Pass 12 (2026-05-09): CoT-geometry repair constants ──
    # All gated by separate default-False flags; full design and
    # calibration in outputs/analysis/cot_reward_candidate_designs.md.
    # Empirical anchors (cleaned audits this session):
    #   Pass 8 reference v_forward aggregate = 0.0207 m/s
    #   Pass 11 FINAL    v_forward aggregate = 0.0198 m/s
    #   Pass 8 effort/m  = 3.78x zero, FINAL = 3.69x zero
    #   FINAL leg_pitch_amp drift target = 0.70 (default 1.0)
    #   FINAL CoT_window mean/p50 ratio   = 7110x (heavy-tailed)

    # Candidate A — speed floor
    _PASS12_V_FLOOR        = 0.018      # m/s, ≈ 90 % of Pass 8 trained mean
    _PASS12_W_V_FLOOR      = 2.0        # 2x the W_V slope (W_V = 1.0)

    # Candidate B — structural guard (resists CPG-structure collapse)
    _PASS12_BODY_AMP_MIN     = 0.6       # body_amp_scale lower bound
    _PASS12_FREQ_MIN         = 0.85      # freq_scale lower bound
    _PASS12_LEG_PITCH_AMP_MIN = 0.75     # added 2026-05-09 vs FINAL drift to 0.70
    _PASS12_W_STRUCT         = 0.30      # ≈ 6x W_IMP; one-sided activation

    # Candidate C — angular penalty split (roll/pitch vs yaw)
    _PASS12_W_ANG_RP       = 0.02       # M1 2026-05-15: 4x->2x (R3 at 4x catastrophized PPO ω_x^2+ω_y^2~=2.5; halve to ~R0 magnitude)
    _PASS12_W_ANG_YAW      = 0.0025     # 1/4 W_ANG (allow body wave yaw)
    # M5-A 2026-05-16: cap per-step ω² to prevent impact-transient
    # catastrophization. CAP_RP_SQ = 2.0 ⇒ sustained |ω_rp| > 1.4 rad/s
    # accrues max penalty; CAP_YAW_SQ = 9.0 ⇒ sustained |ω_yaw| > 3.0
    # rad/s accrues max penalty. Trained PPO impact spikes at ω_x² ≈ 1892
    # (deterministic across R0/R3/M1/M2) are clipped to 2.0 — a 946×
    # reduction at the peak frame. See
    # outputs/analysis/wave_outcome_landscape_reward_constraints_40_80/REPORT.md.
    _PASS12_CAP_RP_SQ      = 2.0
    _PASS12_CAP_YAW_SQ     = 9.0

    # C09-A REVISED (2026-05-16): progress-gated peak-force penalty.
    # Linear excess form (NOT squared), calibrated to the actual
    # per-step F_max distribution observed across M5-A trained policies
    # (mean F_max ≈ 0.012 N, p99 ≈ 0.025 N, deterministic spike 0.144 N).
    # Packet's original constants (F_SAFE=0.075, squared, W=0.10) were
    # vacuous on this distribution; see PHASE 4 readiness audit at
    # outputs/analysis/overnight_m5a_multiseed_c09a_40_80/PHASE4_C09A_READINESS_AUDIT.md.
    # Predicted mean r_fmax ≈ -0.0005 / step at trained M5-A traces;
    # worst-frame penalty bounded by F_CAP at -0.044.
    _C09_F_SAFE_N      = 0.012     # N; ~p25 of trained F_max; above this incurs penalty
    _C09_F_MAX_CAP_N   = 0.10      # N; caps the impact-spike contribution
    _C09_W_FMAX        = 0.50      # weight (linear excess form, not squared)
    _C09_V_GATE        = 0.005     # m/s; below this, no penalty (prevents zero-action gaming)

    # Candidate D — bounded CoT (replaces hinge when enabled with Pass 11)
    _PASS12_COT_CAP        = 100.0      # per-env saturation cap (dimless)
    _PASS12_COT_SCALE      = 200.0      # tanh transition zone
    _PASS12_W_COT_V2       = 6.4e-5     # ≈ 3x W_COT but bounded; calibrated
                                        # so mean(r_cot) ≈ -0.005/step at
                                        # Pass 8 trained policy operating point

    # Pass 12 RELAXED variant — "functional guard" (2026-05-09 follow-up)
    # Biological-hypothesis-aware version: NO body_amp guard, NO body_imp
    # guard. Only leg_pitch and freq are guarded (and at lower thresholds
    # than B2-strict). See outputs/analysis/pass12_relaxed_candidate_design.md
    # for the full derivation. Active when cfg.pass12_functional_guard_enabled.
    _PASS12_LEG_PITCH_MIN_RELAXED = 0.65   # B2 strict was 0.75. Pass 11 FINAL had 0.703.
    _PASS12_FREQ_MIN_RELAXED      = 0.80   # B2 strict was 0.85. Pass 11 FINAL had 1.04.
    _PASS12_W_FUNC                = 0.30   # same scale as W_STRUCT (B2)

    # ── Pass 13 (2026-05-11): robustness reward constants ──
    # r_robust_pass13 = + W_ROBUST_PASS13 * severity_S * v_forward
    # Severity S = amp_mm / λ_mm per terrain block, indexed by spawn_block_id.
    # 2026-05-19 (training-arena-param refactor): the per-block severity
    # tuple now lives on the ENV cfg (`transition_block_severity_S`) so
    # the LUT stays consistent with whichever (λ, amp) tuples were used
    # to build the heightmap. The class-level fallback below preserves the
    # historical default verbatim and is used only when self.cfg lacks the
    # field — kept for legacy / static-analysis callers; live env code
    # reads self.cfg.transition_block_severity_S directly.
    _REWARD_W_ROBUST_PASS13 = 1.5
    _BLOCK_SEVERITY_S = (10.0/18.0,      # rough_a
                         10.0/18.0,      # rough_b
                         10.0/30.0,      # medium
                         8.0/50.0,       # intermediate
                         6.0/60.0,       # long_wave
                         0.0)            # flat

    # 2026-05-07 (Pass 4 fix-1): lowered from 0.005 to 0.0 after the
    # _debug_settle_v2.py Pass A diagnostic on FLAT terrain showed the
    # centipede's natural settled rest height is ~4.4 mm above ground —
    # so the previous 5 mm threshold was firing on a perfectly upright
    # standing pose, producing a 3-step terminate-reset cycle that wiped
    # runtime scales and prevented _sim_time from accumulating to the
    # 8 s episode_length_s. Threshold 0.0 still catches the real failure
    # mode (root sinking below ground) but no longer fires on normal
    # rest. Tip detection (|g_z| < 0.259) and NaN check still catch the
    # other physical-failure cases. See debug log
    # outputs/sanity/_debug_settle_v2_pass_a.csv.
    _FAIL_ROOT_Z_M      = 0.0
    # cos(75°) ≈ 0.259. body z-axis projected onto world up; when
    # robot is upright g_b ≈ (0, 0, -1). |g_b[2]| < 0.259 means the
    # body z-axis is more than 75° from world up.
    _FAIL_TIP_COS       = 0.259

    def _compute_step_state(self) -> dict:
        """Compute the per-step quantities shared between _get_rewards
        and _get_dones. Cached on self._step_state so each helper
        sees the SAME failure mask / torque / etc.

        Called eagerly at the top of _get_rewards. _get_dones reads
        self._step_state if present; if not, recomputes (defensive).
        """
        art = self.robot
        # Root state — already in world / body frames courtesy of Isaac Lab.
        root_lin_vel_w = art.data.root_lin_vel_w               # (E, 3)
        root_ang_vel_b = art.data.root_ang_vel_b               # (E, 3)
        root_pos_w     = art.data.root_pos_w                   # (E, 3)
        proj_g_b       = art.data.projected_gravity_b          # (E, 3)
        # Forward velocity = -world_x_velocity (verified, see project
        # memory: rough λ=18 mm demo had root_x decreasing).
        v_forward = -root_lin_vel_w[:, 0]                      # (E,)
        # Tip detection from projected gravity. |g_b[2]| ≈ 1 when
        # upright (gravity points along body -z). It drops as the
        # body z-axis tilts away from world up. Magnitude < 0.259
        # corresponds to > 75° tilt.
        g_b_z = proj_g_b[:, 2]                                 # (E,)
        tipped = g_b_z.abs() < self._FAIL_TIP_COS
        # Root-z failure (fell through floor; for transition terrain
        # the local heightmap can be ±10 mm, so 5 mm threshold catches
        # collapse but not normal stance-on-rough).
        too_low = root_pos_w[:, 2] < self._FAIL_ROOT_Z_M
        # NaN/Inf safety net (defensive; unlikely with our smoothstep
        # blend + parity-tested controller, but cheap to check).
        nan_root = ~torch.isfinite(root_pos_w).all(dim=1)
        nan_vel  = ~torch.isfinite(root_lin_vel_w).all(dim=1)
        fail = tipped | too_low | nan_root | nan_vel           # (E,) bool

        # Torque buffer (effort target was just written; prefer
        # applied_torque if available, else fall back to the target).
        tau = getattr(art.data, "applied_torque", None)
        if tau is None:
            tau = getattr(art.data, "joint_effort_target", None)
        if tau is None:
            tau = torch.zeros((self.num_envs, 1), dtype=torch.float32, device=self.device)

        # C09-A 2026-05-16: per-step max contact force across bodies.
        # Read from contact_sensor.net_forces_w (already accessed by
        # _get_observations); compute |F|_max over the 38 body slots.
        sensor = None
        if hasattr(self.scene, "sensors"):
            sensor = self.scene.sensors.get("contact_sensor", None)
        if sensor is None:
            try:
                sensor = self.scene["contact_sensor"]
            except Exception:  # noqa: BLE001
                sensor = None
        if sensor is not None:
            net_forces = sensor.data.net_forces_w               # (E, 38, 3)
            Fmag = net_forces.norm(dim=-1)                      # (E, 38)
            F_max = Fmag.max(dim=-1).values                     # (E,)
        else:
            F_max = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)

        state = {
            "v_forward": v_forward,
            "root_pos_w": root_pos_w,
            "proj_g_b":   proj_g_b,
            "g_b_z":      g_b_z,
            "root_ang_vel_b": root_ang_vel_b,
            "tau":        tau,
            "fail":       fail,
            "tipped":     tipped,
            "too_low":    too_low,
            "nan_state":  nan_root | nan_vel,
            "F_max":      F_max,
        }
        self._step_state = state
        return state

    def _windowed_vfwd_update(self, v_forward: torch.Tensor) -> torch.Tensor:
        """P4 v3.4 (2026-06-03): trailing W-step MEAN of SIGNED v_forward, per env. Called once per
        step from _get_rewards (dense). Circular buffer; mean over min(count, W) valid entries (the
        unwritten/zeroed slots contribute 0 to the sum and are excluded by the count). The buffer is
        cleared for done envs in _reset_idx (reset-then-fill), so a window never spans episodes.
        Only invoked when windowed_vfwd_reward_enabled."""
        W = self._vfwd_W
        idx = torch.arange(self.num_envs, device=self.device)
        self._vfwd_buf[idx, self._vfwd_pos] = v_forward.to(torch.float32)
        self._vfwd_pos = (self._vfwd_pos + 1) % W
        self._vfwd_cnt = torch.clamp(self._vfwd_cnt + 1, max=W)
        win = self._vfwd_buf.sum(dim=1) / self._vfwd_cnt.clamp(min=1).to(torch.float32)
        self._vfwd_last_windowed = win
        return win

    def _get_rewards(self) -> torch.Tensor:
        """Stage B-v0 reward (Pass 5, 2026-05-07).

        Composition:
          + W_V     * v_forward                        forward velocity
          + W_ALIVE                                    alive bonus
          - W_ORI   * (1 + g_b_z)^2                    orientation upright
          - W_ANG   * ||omega_b||^2                    angular vel penalty
          - W_ACT   * ||a||^2                          action magnitude
          - W_RATE  * ||a - a_prev||^2                 action rate
          - W_TAU   * mean(|tau|)                      torque magnitude
          - W_Z_VEL * v_z_w^2                          vertical velocity
          - W_IMP   * (body_imp_scale - 1)^2           impedance reg  [IDENTICALLY 0 since
                                                         2026-07-16 -- see _S2_DEAD_ACTION_NOTE]
          - W_WAVE  * (wave_num_scale - 1)^2           wave-num reg
          - W_FAIL  * fail                             failure boolean

        Per-component values are logged via self.extras['log'][...]
        (tensors of shape (E,)) so rl_games' tensorboard observer can
        plot each term independently.
        """
        s = self._compute_step_state()
        # ── Existing terms ──
        ori_term  = (1.0 + s["g_b_z"]) ** 2
        ang_term  = (s["root_ang_vel_b"] ** 2).sum(dim=1)
        a = self.actions.to(dtype=torch.float32)
        a_prev = self._prev_actions.to(dtype=torch.float32)
        # ══ r_act TAXES A DEAD AXIS ══ (recorded 2026-07-16; PI: "default off做好备注，留痕") ══════
        # THE FACT, stated plainly so no future session has to re-derive it:
        #   act_term sums the RAW policy action over ALL 6 dims, and ch-4 (body_imp_scale) has been
        #   STRUCTURALLY DEAD SINCE 2026-07-16 (implicit-only unification S1/S2a/S2b, PI-signed).
        #   `controller.body_kp_scale_runtime` no longer exists; there is NO wire from ch-4 to any
        #   stiffness. Full story: _S2_DEAD_ACTION_NOTE at the top of this file.
        #   => from 2026-07-16 onward, `r_act = -W_ACT * ||a||^2` (W_ACT = 0.002) has been applying a
        #   real, non-zero PPO gradient to an axis that CANNOT affect physics. --design-a-fixed-imp-freq
        #   pins the DECODED scale to 1.0; it does NOT zero the RAW action, and this term reads the RAW
        #   action. So the pin does not remove the tax. Both statements are true at once and that is
        #   exactly the trap: the run looks "isolated to wave" while still paying for ch-4.
        #
        # THE FLAG (cfg.exclude_dead_action_channels_from_r_act, CLI --exclude-dead-action-channels-from-r-act):
        #   OFF (DEFAULT)  -> the `else` branch below is the ORIGINAL expression, character-for-character
        #                     unchanged: `act_term  = (a ** 2).sum(dim=1)`. Same tensor, same ops, same
        #                     order; the flag test is a host-side Python bool that adds no tensor op, no
        #                     RNG draw and no device sync. Flag-OFF is therefore BIT-IDENTICAL to every
        #                     pre-flag run BY CONSTRUCTION, not by measurement -- which is precisely why
        #                     OFF is the default: every checkpoint, reward curve and eval CSV in this
        #                     project's history was produced with the dead channel TAXED, and silently
        #                     un-taxing it would make new numbers incomparable to all of them.
        #   ON             -> ch-4 (global_6d) / ch-20 (body_21d) is zeroed BEFORE the square-sum, i.e.
        #                     the dead axis stops contributing to the action cost. This changes TWO
        #                     things, both of which a caller must own:
        #                       1. r_total (via r_act, line ~2995) -> a DIFFERENT objective => a NEW
        #                          experiment. Do NOT mix ON-runs into a comparison against OFF-runs.
        #                       2. the log key `raw/action_norm_sq` (line ~3775) -> downstream analysis
        #                          / eval-CSV columns read it; its meaning becomes "5-dim norm".
        #                     What it does NOT do: it does not narrow the action space (still 6-wide,
        #                     cfg.action_space = 6), does not remove the channel, and does not touch the
        #                     rate term (rate_term already contributes identically 0 on ch-4 because
        #                     scaled_u's ch-4 slot is hard-coded to ones -- see the block below).
        #
        # WHY IT WAS NOT SIMPLY FIXED: bit-identity is this project's protected-file contract
        #   (CLAUDE.md: "edit ONLY behind a default-OFF flag ... + flag-OFF bit-identity"). The tax is
        #   small but it is NOT zero, so removing it unconditionally would silently re-baseline the
        #   campaign. The PI's call (2026-07-16) was: keep the default OFF, annotate, leave a trace.
        #
        # POINTERS: outputs/CURRENT_RESULT.md (the S2 result marker + its retractions/scope limits) ;
        #   DISPATCH_LOG.md 2026-07-16T19:34Z -> 22:55Z (PI sign-off + the S1/S2a/S2b execution records;
        #   the 22:55Z entry is where this exact defect was first written down as a PI decision owed) ;
        #   docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md ; SOURCE: _S2_DEAD_ACTION_NOTE / _S2_DEAD_ACTION_CH
        #   / env.set_joint_impedance / controllers/impedance_controller_torch.py::_assert_dead_gain_scale.
        # ═══════════════════════════════════════════════════════════════════════════════════════════
        if self.cfg.exclude_dead_action_channels_from_r_act:
            # KeyError on an unknown mode is DELIBERATE (fail-loudly doctrine): a mode whose dead
            # channels we do not know must stop the run, never silently tax the full vector.
            _dead_ch = _S2_DEAD_ACTION_CH[getattr(self.cfg, "action_mode", "global_6d")]
            _a_taxed = a.clone()
            _a_taxed[:, list(_dead_ch)] = 0.0   # list() => unambiguous advanced indexing
            act_term = (_a_taxed ** 2).sum(dim=1)
        else:
            act_term  = (a ** 2).sum(dim=1)

        # ── Pass 6: action-rate term in SCALED-PARAMETER space ──
        # Read the current 6-D scaled-physical-knob vector from the
        # controller's runtime fields (same order used by the global_6d
        # decoder, so the rate term is interpretable in physical units).
        # body_amp_scale_runtime is shape (E, 19) since global_6d
        # broadcasts a single body amplitude across all 19 body-yaw
        # joints; we read joint 0 as a representative (under global_6d
        # all 19 are equal; under legacy body_21d they may differ but
        # body_21d isn't this Pass's training mode).
        scaled_u = torch.stack(
            [
                self.controller.body_amp_scale_runtime[:, 0],
                self.controller.leg_yaw_amp_scale_runtime,
                self.controller.leg_pitch_amp_scale_runtime,
                self.controller.wave_number_scale_runtime,
                # ch-4 = the body_imp (gain) scale -- STRUCTURALLY DEAD (full story:
                # _S2_DEAD_ACTION_NOTE at the top of this file). S2b (2026-07-16) DELETED
                # controller.body_kp_scale_runtime, so there is nothing left to read; this
                # slot is hard-coded to the constant the S2 guard already forces.
                # BIT-IDENTICAL: _apply_action_implicit_pd RAISES unless every dead gain
                # scale == 1.0, so any run that reaches this reward had exactly 1.0 here
                # => this dimension's contribution to rate_term was already identically 0.
                # It is kept (rather than dropping to a 5-vector) so scaled_u stays (E, 6):
                # _prev_scaled_action, raw/scaled_action_l2 and the 6-D action bookkeeping
                # all assume 6, and a reshape would change logged values for no gain.
                torch.ones(self.num_envs, dtype=self.controller.freq_scale_runtime.dtype,
                           device=self.device),
                self.controller.freq_scale_runtime,
            ],
            dim=1,
        ).to(dtype=torch.float32)                    # (E, 6)
        rate_term = ((scaled_u - self._prev_scaled_action) ** 2).sum(dim=1)
        # Cache for next step (must happen AFTER rate computation).
        self._prev_scaled_action = scaled_u.detach().clone()

        tau_term  = s["tau"].abs().mean(dim=1)
        fail_term = s["fail"].to(dtype=torch.float32)
        alive_term = torch.ones_like(s["v_forward"])

        # ── New Pass 5 terms ──
        # Vertical velocity squared. We use root_lin_vel_w[:, 2] (world
        # frame z); this directly penalizes hopping/bouncing.
        z_vel = self.robot.data.root_lin_vel_w[:, 2]      # (E,)
        z_vel_term = z_vel ** 2

        # ── IMPEDANCE REGULARIZER — STRUCTURALLY ALWAYS ZERO SINCE 2026-07-16 ──────────────
        # THIS IS AN RL-TRAINING-SIDE TERM. It is KEPT DELIBERATELY, and it REGULARIZES NOTHING.
        # If you are a fresh session trying to understand this file, read _S2_DEAD_ACTION_NOTE at
        # the top — it has the full story. The short version:
        #
        # WHAT IT USED TO DO: penalize the policy for pulling the 6D action's ch-4 impedance scale
        #   away from baseline 1.0. That scale reached physics through
        #   `controller.body_kp_scale_runtime` -> the controller's own PD law -> an explicit torque,
        #   so the term was a real tax on real in-episode spine stiffening.
        # WHY IT IS NOW IDENTICALLY 0: the implicit-only unification (S1/S2a/S2b, PI-signed) DELETED
        #   `controller.body_kp_scale_runtime` and the whole explicit torque path. Impedance is now
        #   ONE (Kp, Kv) per joint family in physical units, written straight to PhysX by
        #   env.set_joint_impedance(); feedback closes inside PhysX's implicit PD solver. There is no
        #   wire left from ch-4 to a stiffness, so `body_imp_scale` is not read from anywhere any
        #   more — it is HARD-CODED to ones here, making imp_term == 0 exactly, by construction and
        #   not by luck. (The env's _S2_DEAD_SCALES / _s2_assert_scales_inert() fingerprint guard
        #   forces the underlying scales to 1.0 or RAISES, so no run can reach this line with a
        #   different value.)
        # WHY KEEP IT: (1) bit-identity — W_IMP * 0 == the value every S2-era run already scored, so
        #   deleting vs keeping is numerically identical and keeping avoids churning r_total's
        #   composition; (2) `reward/09_impedance_reg` and `raw/body_imp_scale` are LOG KEYS that
        #   downstream analysis / eval-CSV column derivation read, and the flag-OFF bit-identity rule
        #   in this file treats a log key-set change as a breaking change.
        # WHAT A FUTURE RL RUN MUST DO INSTEAD: to modulate impedance you must modulate the TABLE —
        #   env.set_joint_impedance({family: (Kp, Kv)}) in physical N*m/rad, families =
        #   env.s2_family_ids() — NOT a scale. Reviving THIS term as-is would regularize a constant.
        #   A regularizer for a table-driven impedance has to be written against the table's own
        #   quantity (e.g. |Kp - Kp_baseline|), and the policy needs a real wire to it first.
        # POINTERS: docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md ; outputs/CURRENT_RESULT.md ;
        #   DISPATCH_LOG.md 2026-07-16T19:34Z -> 22:05Z ; source = env.set_joint_impedance /
        #   env.set_joint_effort_limit / env._apply_action_implicit_pd.
        # ───────────────────────────────────────────────────────────────────────────────────
        body_imp_scale = torch.ones(self.num_envs, dtype=torch.float32, device=self.device)
        imp_term = (body_imp_scale - 1.0) ** 2                     # == 0 for every env, always

        # Wave-number regularization toward baseline 1.0.
        wave_num_scale = self.controller.wave_number_scale_runtime  # (E,)
        wave_term = (wave_num_scale - 1.0) ** 2

        # ── Cache actuated joint indices on first call ──
        # body yaw + pitch + leg (foot is passive — kp=0, only damping —
        # and is excluded from the power sum). Used by both Pass 10 and
        # Pass 11 mechanical-work computations.
        if not hasattr(self, "_pass10_active_idx"):
            _body_idx, _pitch_idx, _leg_idx = [], [], []
            for _i, _name in enumerate(self.robot.joint_names):
                # P3 rename (2026-06-17): match BOTH the legacy joint_* prefixes
                # AND the function-renamed prefixes (body_pitch_/body_yaw_/
                # legbase_*/knee_). Additive: legacy names never match the new
                # prefixes, so the deployed model's mask is byte-identical.
                if _name.startswith("joint_pitch_body_") or _name.startswith("body_pitch_"):
                    _pitch_idx.append(_i)
                elif _name.startswith("joint_body_") or _name.startswith("body_yaw_"):
                    _body_idx.append(_i)
                elif (_name.startswith("joint_leg_") or _name.startswith("legbase_")
                      or _name.startswith("knee_")):
                    _leg_idx.append(_i)
            self._pass10_active_idx = torch.tensor(
                _body_idx + _pitch_idx + _leg_idx,
                dtype=torch.long, device=self.device,
            )
            print(
                f"[CentipedeEnv] Pass 10/11 actuated-DOF mask: "
                f"{self._pass10_active_idx.numel()} DOFs "
                f"(body={len(_body_idx)}, pitch={len(_pitch_idx)}, "
                f"leg={len(_leg_idx)}) cached",
                flush=True,
            )

        # Pre-compute power_t and gate_t — reused below by both Pass 10
        # (per-step penalty) and Pass 11 (windowed work integration).
        qdot = self.robot.data.joint_vel                              # (E, n_joints)
        tau_active  = s["tau"][:, self._pass10_active_idx].abs()      # (E, n_active)
        qdot_active = qdot[:, self._pass10_active_idx].abs()          # (E, n_active)
        power_t = (tau_active * qdot_active).sum(dim=1)               # (E,)
        gate_t = torch.sigmoid(
            (s["v_forward"] - self._GATE_V_MID) / self._GATE_V_SIGMA
        )                                                             # (E,)

        # ── Pass 10 (2026-05-08, ablation only): per-step mechanical-
        #    power penalty. Disabled by default in Pass 8 / Pass 11; can
        #    be re-enabled via cfg.pass10_effort_enabled = True for
        #    ablation comparisons against the failed v1/v2 smoke runs.
        if self.cfg.pass10_effort_enabled:
            effort_term = gate_t * power_t                            # (E,)
        else:
            effort_term = torch.zeros_like(power_t)

        # ── Pass 11 (2026-05-09): windowed work-per-distance CoT term ──
        # CoT_window = (Σ_H |τ·q̇|·dt) / (m·g · Σ_H max(0, dx_forward))
        # gate_x = sigmoid((dx_window - dx_min) / dx_sigma)
        # r_cot  = − W_COT · gate_x · max(0, CoT_window − α·CoT_zero)
        # All constants live as class attributes below the reward
        # weights (see _COT_*). Calibrated 2026-05-09 against
        # zero-action baseline at H=42 policy steps; see
        # outputs/sanity/pass11_cot_calibration.txt and
        # outputs/analysis/pass11_calibration_review.md.
        if self.cfg.pass11_cot_enabled:
            H = self._COT_H_STEPS
            dt_pol = float(self.cfg.decimation) * float(self.cfg.sim.dt)

            # Lazy buffer init (first call after env creation).
            if not hasattr(self, "_pass11_initialized"):
                self._pass11_work_buf = torch.zeros(
                    (self.num_envs, H), dtype=torch.float32,
                    device=self.device,
                )
                self._pass11_dx_buf = torch.zeros(
                    (self.num_envs, H), dtype=torch.float32,
                    device=self.device,
                )
                self._pass11_buf_idx = 0
                self._pass11_buf_count = torch.zeros(
                    self.num_envs, dtype=torch.long, device=self.device,
                )
                self._pass11_prev_root_x = (
                    self.robot.data.root_pos_w[:, 0].detach().clone()
                )
                self._pass11_initialized = True
                print(
                    f"[CentipedeEnv] Pass 11 CoT-term initialized "
                    f"(H={H}, dx_min={self._COT_DX_MIN} m, "
                    f"alpha=1.5, threshold={self._COT_THRESHOLD:.2f}, "
                    f"W_COT={self._REWARD_W_COT:.2e})",
                    flush=True,
                )

            # Per-step work and forward displacement.
            work_step = power_t * dt_pol                              # (E,)  [J]
            cur_x = self.robot.data.root_pos_w[:, 0]
            dx_raw = -(cur_x - self._pass11_prev_root_x)              # forward = -x
            # Filter reset jumps (|dx_raw| > 0.05 m / 0.048 s ≈ 1 m/s)
            # and clamp backward motion to zero — same convention as
            # the calibration script.
            reset_jump = dx_raw.abs() > 0.05
            dx_step = torch.where(
                reset_jump,
                torch.zeros_like(dx_raw),
                torch.clamp(dx_raw, min=0.0),
            )
            self._pass11_prev_root_x = cur_x.detach().clone()

            # Write into the circular buffer at the global step index.
            write_idx = self._pass11_buf_idx % H
            self._pass11_work_buf[:, write_idx] = work_step
            self._pass11_dx_buf[:, write_idx]   = dx_step
            self._pass11_buf_idx += 1
            self._pass11_buf_count = torch.clamp(
                self._pass11_buf_count + 1, max=H,
            )

            # Windowed sums and dimensionless CoT.
            work_window = self._pass11_work_buf.sum(dim=1)            # (E,) [J]
            dx_window   = self._pass11_dx_buf.sum(dim=1)              # (E,) [m]
            denom = torch.clamp(
                self._COT_BODY_MASS_KG * self._COT_GRAVITY * dx_window,
                min=self._COT_EPSILON_E,
            )
            cot_window = work_window / denom                          # (E,)

            # Gate: sigmoid on dx_window (NOT instantaneous v_forward),
            # multiplied by a "buffer is full" 0/1 mask so the term is
            # 0 for the first H steps after env creation / reset.
            gate_x = torch.sigmoid(
                (dx_window - self._COT_DX_MIN) / self._COT_DX_SIGMA
            )
            buf_filled = (self._pass11_buf_count >= H).to(
                dtype=torch.float32
            )
            gate_x = gate_x * buf_filled

            # ReLU hinge: penalize only when CoT_window exceeds threshold.
            excess = torch.clamp(
                cot_window - self._COT_THRESHOLD, min=0.0,
            )
            cot_term = gate_x * excess                                # (E,)
            # Pass 12 Candidate D: bounded-CoT replacement.
            # When pass12_cot_bounded_enabled is True, swap the unbounded
            # hinge for a saturating tanh form so per-env contribution is
            # capped and the gradient is robust to dx_window→0 outliers.
            if self.cfg.pass12_cot_bounded_enabled:
                bounded_excess = self._PASS12_COT_CAP * torch.tanh(
                    excess / self._PASS12_COT_SCALE
                )
                cot_term = gate_x * bounded_excess                    # (E,)
                _w_cot_active = self._PASS12_W_COT_V2
            else:
                _w_cot_active = self._REWARD_W_COT
        else:
            cot_window = torch.zeros(self.num_envs, dtype=torch.float32,
                                       device=self.device)
            dx_window = torch.zeros(self.num_envs, dtype=torch.float32,
                                       device=self.device)
            work_window = torch.zeros(self.num_envs, dtype=torch.float32,
                                       device=self.device)
            gate_x = torch.zeros(self.num_envs, dtype=torch.float32,
                                       device=self.device)
            cot_term = torch.zeros(self.num_envs, dtype=torch.float32,
                                       device=self.device)
            _w_cot_active = self._REWARD_W_COT  # unused (cot_term ≡ 0)

        # ── Per-component reward (per env). Sign-applied at compose time. ──
        # P4 v3.4 windowed-speed reward (2026-06-03): when enabled, r_v uses the trailing-window MEAN
        # of v_forward (resolves the wave->speed effect the per-step signal cannot; PART-0 W). The raw
        # per-step v_forward (s["v_forward"]) is UNTOUCHED elsewhere (logging, floors, gates, etc.).
        # Default OFF ⇒ _vfwd_for_rv = s["v_forward"] ⇒ byte-identical.
        if bool(getattr(self.cfg, "windowed_vfwd_reward_enabled", False)):
            _vfwd_for_rv = self._windowed_vfwd_update(s["v_forward"])
        else:
            _vfwd_for_rv = s["v_forward"]
        r_v     =  self._REWARD_W_V     * _vfwd_for_rv
        # P4 v3.4 Phase 1 (2026-06-02): default-OFF r_v ablation lever. Scales the
        # forward-velocity reward. 1.0 ⇒ multiply SKIPPED ⇒ byte-identical to pre-edit.
        _rv_scale = float(getattr(self.cfg, "rv_weight_scale", 1.0))
        if _rv_scale != 1.0:
            r_v = r_v * _rv_scale
        r_alive =  self._REWARD_W_ALIVE * alive_term
        r_ori   = -self._REWARD_W_ORI   * ori_term
        r_ang   = -self._REWARD_W_ANG   * ang_term
        r_act   = -self._REWARD_W_ACT   * act_term
        # P4 v3.4 Phase 1 (2026-06-02): default-OFF r_act ablation lever. Scales the
        # action-norm penalty (the tax on the negative action needed to lower wave).
        # 1.0 ⇒ multiply SKIPPED ⇒ byte-identical. r_rate is left UNTOUCHED.
        _ract_scale = float(getattr(self.cfg, "ract_weight_scale", 1.0))
        if _ract_scale != 1.0:
            r_act = r_act * _ract_scale
        r_rate  = -self._REWARD_W_RATE  * rate_term
        r_tau   = -self._REWARD_W_TAU   * tau_term
        r_zvel  = -self._REWARD_W_Z_VEL * z_vel_term
        # r_imp == 0 for every env, ALWAYS: imp_term is structurally zero since the implicit-only
        # unification deleted the impedance action channel's route to physics (2026-07-16). It stays
        # in r_total because -W_IMP * 0 == 0 is bit-identical to removing it, and `reward/09_impedance_reg`
        # is a log key read downstream. It regularizes NOTHING. See the imp_term block above and
        # _S2_DEAD_ACTION_NOTE at the top of this file before reviving or re-weighting it.
        r_imp   = -self._REWARD_W_IMP   * imp_term
        r_wave  = -self._REWARD_W_WAVE  * wave_term
        # P4 v3.4 Round C C1: neutralize the legacy wave_scale→1.0 (applied_abs→3.0)
        # anchor so the emergence test isn't given a free push toward abs=3.0
        # (emergence-contamination, not "fights high wave" — the anchor is actually
        # bio-prior-aligned on rough). Default OFF ⇒ r_wave unchanged.
        if bool(getattr(self.cfg, "roundc_zero_wave_anchor", False)):
            r_wave = torch.zeros_like(r_wave)
        r_fail  = -self._REWARD_W_FAIL  * fail_term
        # Pass 10: mechanical-power / effort-aware term (sign-applied).
        # When cfg.pass10_effort_enabled is False, effort_term ≡ 0 → r_effort ≡ 0.
        # Route-A dose (2026-06-07): weight is CLI-tunable; default == _REWARD_W_EFFORT
        # (0.018) so an unscaled run is bit-identical, and OFF (effort_term≡0) is unaffected.
        _w_effort = float(getattr(self.cfg, "reward_w_effort", self._REWARD_W_EFFORT))
        r_effort = -_w_effort * effort_term
        # Pass 11: windowed CoT hinge term (sign-applied).
        # When cfg.pass11_cot_enabled is False, cot_term ≡ 0 → r_cot ≡ 0.
        # _w_cot_active is _REWARD_W_COT by default, or _PASS12_W_COT_V2
        # when pass12_cot_bounded_enabled is True (set inside the block above).
        r_cot = -_w_cot_active * cot_term

        # ── Pass 12 (2026-05-09): independent default-False reward shapers ──
        # All three are zero when their flag is False, so default behavior
        # is bit-identical to Pass 11 (which is bit-identical to Pass 8 +
        # cot_p11). See outputs/analysis/cot_reward_candidate_designs.md
        # for the per-candidate derivation.

        # Pass 12 Candidate A — speed floor.
        if self.cfg.pass12_speed_floor_enabled:
            v_def = torch.clamp(
                self._PASS12_V_FLOOR - s["v_forward"], min=0.0,
            )
            r_speed_floor = -self._PASS12_W_V_FLOOR * v_def
        else:
            r_speed_floor = torch.zeros_like(s["v_forward"])

        # Route-A L2 (2026-06-05, GO_ROUTEA_EMERGENCE): WINDOWED + CLI-tunable
        # speed floor. Default-OFF; when the flag is False this block is skipped
        # and r_speed_floor is unchanged ⇒ reward bit-identical to pre-edit.
        # When ON, the one-sided hinge is driven off the TRAILING-WINDOW mean
        # v_fwd (self._vfwd_last_windowed, populated at the windowed-r_v update
        # earlier this step) so it prices SUSTAINED stalls, not gait-cycle dips;
        # threshold + weight are CLI-tunable (defaults == the legacy constants).
        # Reads v_fwd ONLY — no wave/severity/applied_abs → W1/W2-legal. Adds to
        # r_speed_floor (the legacy per-step term is OFF in route-A arms).
        if bool(getattr(self.cfg, "pass12_speed_floor_windowed_enabled", False)):
            _sf_thresh = float(getattr(self.cfg, "speed_floor_thresh", self._PASS12_V_FLOOR))
            _sf_weight = float(getattr(self.cfg, "speed_floor_weight", self._PASS12_W_V_FLOOR))
            _vfwd_win = getattr(self, "_vfwd_last_windowed", None)
            if _vfwd_win is None:
                # windowed buffer inactive (windowed_vfwd_reward_enabled is OFF):
                # misconfiguration for this term. Fall back to per-step v_fwd so
                # the run does not crash; route-A base always has windowed r_v ON.
                _vfwd_win = s["v_forward"]
            v_def_w = torch.clamp(_sf_thresh - _vfwd_win, min=0.0)
            r_speed_floor = r_speed_floor + (-_sf_weight * v_def_w)

        # Pass 12 Candidate B — structural guard (CPG-structure preservation).
        # Reads body_amp_scale_runtime[:, 0], freq_scale_runtime,
        # leg_pitch_amp_scale_runtime — the latter is the FINAL-drift target.
        if self.cfg.pass12_structural_guard_enabled:
            _body_amp_scale_p12  = self.controller.body_amp_scale_runtime[:, 0]
            _freq_scale_p12      = self.controller.freq_scale_runtime
            _leg_pitch_scale_p12 = self.controller.leg_pitch_amp_scale_runtime
            amp_def  = torch.clamp(
                self._PASS12_BODY_AMP_MIN - _body_amp_scale_p12, min=0.0,
            )
            freq_def_term = torch.clamp(
                self._PASS12_FREQ_MIN - _freq_scale_p12, min=0.0,
            )
            leg_def  = torch.clamp(
                self._PASS12_LEG_PITCH_AMP_MIN - _leg_pitch_scale_p12, min=0.0,
            )
            struct_term = (amp_def ** 2) + (freq_def_term ** 2) + (leg_def ** 2)
            r_struct_guard = -self._PASS12_W_STRUCT * struct_term
        else:
            r_struct_guard = torch.zeros_like(s["v_forward"])

        # Pass 12 RELAXED — functional guard (NO body_amp / body_imp guard).
        # Only protects leg_pitch_amp and freq, at lower thresholds than B2.
        # Allows biologically-plausible body-wave / soft-body adaptation.
        # See outputs/analysis/pass12_relaxed_candidate_design.md.
        if self.cfg.pass12_functional_guard_enabled:
            _freq_scale_p12r      = self.controller.freq_scale_runtime
            _leg_pitch_scale_p12r = self.controller.leg_pitch_amp_scale_runtime
            leg_def_r  = torch.clamp(
                self._PASS12_LEG_PITCH_MIN_RELAXED - _leg_pitch_scale_p12r, min=0.0,
            )
            freq_def_r = torch.clamp(
                self._PASS12_FREQ_MIN_RELAXED - _freq_scale_p12r, min=0.0,
            )
            func_term = (leg_def_r ** 2) + (freq_def_r ** 2)
            r_functional_guard = -self._PASS12_W_FUNC * func_term
        else:
            r_functional_guard = torch.zeros_like(s["v_forward"])

        # Pass 12 Candidate C — angular penalty split (yaw allowed, RP heavy).
        # When True, REPLACES the existing r_ang (zeroed below) with a split
        # that distinguishes roll/pitch instability from body-wave yaw.
        if self.cfg.pass12_ang_split_enabled:
            _omega = s["root_ang_vel_b"]                              # (E, 3)
            ang_rp_raw = (_omega[:, 0] ** 2) + (_omega[:, 1] ** 2)
            ang_y_raw  = _omega[:, 2] ** 2
            # M5-A 2026-05-16: clip per-step ω² to bound impact transients.
            ang_rp = torch.clamp(ang_rp_raw, max=self._PASS12_CAP_RP_SQ)
            ang_y  = torch.clamp(ang_y_raw,  max=self._PASS12_CAP_YAW_SQ)
            r_ang_split = (
                -self._PASS12_W_ANG_RP * ang_rp
                -self._PASS12_W_ANG_YAW * ang_y
            )
            # M5-A diagnostics: keep unclipped + clipped tensors available
            # for the log block below.
            self._m5a_ang_rp_raw = ang_rp_raw.detach()
            self._m5a_ang_y_raw  = ang_y_raw.detach()
            self._m5a_ang_rp_clipped = ang_rp.detach()
            self._m5a_ang_y_clipped  = ang_y.detach()
            # Single source of truth: zero the original r_ang to avoid
            # double-counting. Original log key reflects the substitution.
            r_ang = torch.zeros_like(s["v_forward"])
        else:
            r_ang_split = torch.zeros_like(s["v_forward"])
            self._m5a_ang_rp_raw = None
            self._m5a_ang_y_raw  = None
            self._m5a_ang_rp_clipped = None
            self._m5a_ang_y_clipped  = None

        # ── P4 v3.4 trend-suppression Phase 1 (2026-06-02): r_ang ablation ──
        # Scale the (resolved) angular-velocity penalty by cfg.rang_weight_scale.
        # DEFAULT 1.0 ⇒ the multiply is SKIPPED ⇒ byte-identical to pre-edit
        # (verified by flag-OFF SHA match). Arms: 0.5 (halve r_ang), 0.0 (zero).
        # In C1 (pass12_ang_split OFF) r_ang above is the live W_ANG-scaled term;
        # under ang_split ON r_ang is already zeroed so this is a no-op.
        _rang_scale = float(getattr(self.cfg, "rang_weight_scale", 1.0))
        if _rang_scale != 1.0:
            r_ang = r_ang * _rang_scale

        # ── C09-A REVISED (2026-05-16): progress-gated peak-force penalty.
        # Always active (no Pass-N flag; the patch IS the change). Linear
        # excess form: r_fmax = -W * clamp(F_max, F_CAP) - F_SAFE)+ * gate(v>V_GATE).
        F_max_c09 = s["F_max"]                                              # (E,) raw N
        F_max_clipped = F_max_c09.clamp(max=self._C09_F_MAX_CAP_N)          # (E,)
        F_excess = (F_max_clipped - self._C09_F_SAFE_N).clamp(min=0.0)      # (E,)
        c09a_gate = (s["v_forward"] > self._C09_V_GATE).float()             # (E,)
        r_fmax = -self._C09_W_FMAX * F_excess * c09a_gate                   # (E,)
        # Diagnostic tensors for log block
        self._c09a_F_max_raw = F_max_c09.detach()
        self._c09a_F_excess  = F_excess.detach()
        self._c09a_gate      = c09a_gate.detach()

        # ── P4 v3.4 Round C — C0 impact-first reward (2026-05-31) ──
        # When (roundC_enabled AND rC0_impact_enabled): gate the legacy always-on
        # r_fmax to 0 and replace it with rC0_impact. force_raw_Fmax = MEAN F_max
        # over a W-step window (PI decision B2: windowed MEAN, NOT peak — matches
        # the fixed-wave sweeps' per-cell mean benefit; avoids the per-step-peak
        # pathology). gate_progress_valid: locomoting forward + upright, so
        # "minimize force by freezing/crawling/falling" is NOT a win. Sensor-only;
        # NEVER references wave/applied_abs (Constraint 2). When OFF, legacy r_fmax
        # is unchanged and rC0_impact ≡ 0 ⇒ r_total bit-identical to baseline.
        _rC0_on = (bool(getattr(self.cfg, "roundC_enabled", False))
                   and bool(getattr(self.cfg, "rC0_impact_enabled", False)))
        if _rC0_on:
            W_rc0 = self._rC0_fmax_W
            _arange_rc0 = torch.arange(self.num_envs, device=self.device)
            _idx_rc0 = self._rC0_fmax_idx
            self._rC0_fmax_buf[_arange_rc0, _idx_rc0] = F_max_c09  # raw N, same signal as C09-A
            self._rC0_fmax_idx = (_idx_rc0 + 1) % W_rc0
            self._rC0_fmax_filled = torch.minimum(
                self._rC0_fmax_filled + 1,
                torch.full_like(self._rC0_fmax_filled, W_rc0))
            # MEAN over the FILLED portion (B2): sum / n_filled, clamp n>=1.
            _n_filled = torch.clamp(self._rC0_fmax_filled.float(), min=1.0)       # (E,)
            force_raw_Fmax = self._rC0_fmax_buf.sum(dim=1) / _n_filled            # (E,) windowed mean
            _rC0_excess = (force_raw_Fmax.clamp(max=self._C09_F_MAX_CAP_N)
                           - self._C09_F_SAFE_N).clamp(min=0.0)                   # reuse C09 caps
            # validity gate: forward(>V_CRAWL) AND upright. V_CRAWL(0.008)>V_GATE(0.005)
            # so the C09 V_GATE conjunct is redundant and dropped (Gate-2 c).
            _v_fwd = s["v_forward"]                                               # = -root_lin_vel_b[0]
            _proj_gz = s["proj_g_b"][:, 2]                                        # sensed attitude
            gate_progress_valid = (
                (_v_fwd > float(self.cfg.rC0_V_CRAWL))
                & (_proj_gz < float(self.cfg.rC0_TIP_PROJ_GZ))
            ).float()
            # require buffer half-filled before firing (parity w/ Pass16 _gate_f)
            gate_progress_valid = gate_progress_valid * (
                self._rC0_fmax_filled >= max(W_rc0 // 2, 1)).float()
            # ── P4 v3.4 Round C — C1: belief-conditioned impact (emergence test) ──
            # When rC1 on, multiply the impact penalty by belief_weight =
            # (1 + kappa·R_belief), R_belief = K12 SENSOR belief (no ground-truth).
            # Reward STILL never mentions wave. When OFF, belief_weight ≡ 1.0 so
            # rC1 reduces EXACTLY to rC0 (C0 bit-identity preserved).
            _rC1_on = bool(getattr(self.cfg, "rC1_belief_impact_enabled", False))
            if _rC1_on:
                _R_belief_c1, _ = self._compute_k12_R_belief(self.robot)         # sensor-only (E,)
                belief_weight = 1.0 + float(self.cfg.rC1_kappa) * _R_belief_c1   # (E,) in [1, 1+kappa]
            else:
                _R_belief_c1 = torch.zeros_like(_rC0_excess)
                belief_weight = torch.ones_like(_rC0_excess)
            rC0_impact = (-float(self.cfg.rC0_impact_weight)
                          * belief_weight * _rC0_excess * gate_progress_valid)   # (E,) C0 when weight=1
            legacy_r_fmax_gated = torch.zeros_like(r_fmax)                        # gate legacy to 0
            self._rC0_force_raw_Fmax = force_raw_Fmax.detach()
            self._rC0_gate           = gate_progress_valid.detach()
            self._rC1_belief_weight  = belief_weight.detach()
            self._rC1_active         = float(_rC1_on)
        else:
            rC0_impact = torch.zeros_like(r_fmax)
            legacy_r_fmax_gated = r_fmax                                          # unchanged
            self._rC0_force_raw_Fmax = torch.zeros_like(r_fmax)
            self._rC0_gate           = torch.zeros_like(r_fmax)
            self._rC1_belief_weight  = torch.ones_like(r_fmax)
            self._rC1_active         = 0.0
        self._rC0_impact_val = rC0_impact.detach()
        self._legacy_r_fmax_gated_val = legacy_r_fmax_gated.detach()
        self._rC0_active = float(_rC0_on)

        # ── Pass 13 (2026-05-11): robustness reward — severity-weighted
        #    forward progress. Encourages per-step progress on hard terrains
        #    without referencing wave_num/body_imp directly.
        #
        #    r_robust_pass13 = + W_ROBUST_PASS13 * severity_S(spawn_block) * v_forward
        #
        #    severity_S is the per-env terrain severity, derived once from
        #    self._spawn_block_id using a cached lookup tensor that matches
        #    cfg.transition_block_labels order.
        #
        #    When pass13_robustness_reward_enabled is False, this term ≡ 0 and
        #    the total reward is byte-identical to Pass 12 RELAXED behavior.
        if self.cfg.pass13_robustness_reward_enabled:
            if not hasattr(self, "_pass13_S_lut"):
                # 2026-05-19 (training-arena-param refactor): prefer the
                # cfg-side per-block severity tuple if present, so the LUT
                # tracks whatever (λ, amp) the env was actually built on.
                # Class constant retained as a safety fallback.
                _sev_tuple = tuple(
                    getattr(self.cfg, "transition_block_severity_S",
                            self._BLOCK_SEVERITY_S)
                )
                self._pass13_S_lut = torch.tensor(
                    _sev_tuple,
                    dtype=s["v_forward"].dtype, device=self.device,
                )
            # spawn_block_id can be -1 (pre-reset transient); clamp to 0
            # (= rough_a in label order) so the lookup is safe. Any env in
            # the -1 transient is also pre-reward; the value is effectively
            # ignored downstream because the reward gets reset on next step.
            S_idx = torch.clamp(self._spawn_block_id, min=0)
            S_per_env = self._pass13_S_lut[S_idx]                # (E,)
            r_robust_pass13 = (
                self._REWARD_W_ROBUST_PASS13 * S_per_env * s["v_forward"]
            )
        else:
            S_per_env = torch.zeros_like(s["v_forward"])
            r_robust_pass13 = torch.zeros_like(s["v_forward"])

        # ── Pass 15 (2026-05-18): reward recalibration ───────────────
        # Two independent default-OFF terms approved by ChatGPT after the
        # wave-scale audit. Both identically 0 when their flag is False;
        # total reward is byte-identical to pre-Pass-15 when both are
        # False. See cfg docstring for design rationale.
        #
        # Read applied_abs = wave_number_scale_runtime * baseline_n_wave.
        # Logged per-env; consumed by both Pass 15 terms when active.
        _wave_scale_for_pass15 = self.controller.wave_number_scale_runtime
        applied_abs_pass15 = (
            _wave_scale_for_pass15 * float(self.cfg.pass15_baseline_n_wave)
        )
        #
        # Term A — v_forward floor (quadratic hinge on v < v_target).
        if self.cfg.pass15_v_fwd_floor_enabled:
            v_deficit = torch.clamp(
                float(self.cfg.pass15_v_target_mps) - s["v_forward"],
                min=0.0,
            )
            r_v_fwd_floor_pass15 = (
                - float(self.cfg.pass15_w_v_floor) * v_deficit * v_deficit
            )
        else:
            r_v_fwd_floor_pass15 = torch.zeros_like(s["v_forward"])
        #
        # Term B — applied_abs soft cap (quadratic hinge on abs > threshold).
        if self.cfg.pass15_wave_softcap_enabled:
            abs_excess = torch.clamp(
                applied_abs_pass15 - float(
                    self.cfg.pass15_applied_abs_softcap_threshold),
                min=0.0,
            )
            r_wave_softcap_pass15 = (
                - float(self.cfg.pass15_w_softcap) * abs_excess * abs_excess
            )
        else:
            r_wave_softcap_pass15 = torch.zeros_like(s["v_forward"])

        # ── Pass 16 (2026-05-19) — overnight_adaptive_wave_campaign ──
        # Two independent default-OFF functional-outcome reward terms.
        # NEITHER depends on terrain ground truth, spawn pose, or LUT command.
        # Update circular buffers each call; compute windowed reductions only
        # when flag is True. Buffers are pre-allocated in __init__.
        #
        # Term A — attitude variance:
        #   Insert current root roll & pitch (from projected_gravity_b, the same
        #   reduction used elsewhere) into the circular buffers, then compute
        #   the window variance and subtract weight * (roll_var + pitch_var).
        # P4 v3.4 Round B (2026-05-31): the attitude-variance WINDOW must also be
        # maintained when pass20_belief_robustness is on (r_20 reuses _attitude_var_w
        # + _gate). The pass16 REWARD term stays gated on pass16 alone, so flag-OFF
        # (both off) and pass16-only behavior are byte-identical to before.
        _need_attitude_var = bool(self.cfg.pass16_attitude_var_enabled) or \
            bool(getattr(self.cfg, "pass20_belief_robustness_enabled", False))
        if _need_attitude_var:
            # Root attitude this step. proj_g_b is already computed earlier in
            # _get_rewards (it's part of `s`). We use proj_g_b directly.
            _proj_g_b = self.robot.data.projected_gravity_b
            _gx = _proj_g_b[:, 0]; _gy = _proj_g_b[:, 1]; _gz = _proj_g_b[:, 2]
            _roll_now  = torch.atan2(-_gy, -_gz)
            _pitch_now = torch.asin(torch.clamp(_gx, -1.0, 1.0))
            # Write into circular buffer at p16_att_idx
            W_att = self._p16_attitude_W
            _idx = self._p16_att_idx
            _arange = torch.arange(self.num_envs, device=self.device)
            self._p16_roll_buf[_arange, _idx]  = _roll_now
            self._p16_pitch_buf[_arange, _idx] = _pitch_now
            self._p16_att_idx     = (_idx + 1) % W_att
            self._p16_att_filled  = torch.minimum(
                self._p16_att_filled + 1,
                torch.full_like(self._p16_att_filled, W_att))
            # Compute variance only when buffer has enough samples
            _filled_mask = self._p16_att_filled.float()
            _n = torch.clamp(_filled_mask, min=1.0)
            # Mean over filled portion (treat un-filled rows as 0; since buffer
            # is zeroed at reset and we mask the count, the mean is correct).
            _roll_mean  = self._p16_roll_buf.sum(dim=1)  / _n
            _pitch_mean = self._p16_pitch_buf.sum(dim=1) / _n
            _roll_var  = ((self._p16_roll_buf  - _roll_mean.unsqueeze(1))**2).sum(dim=1) / _n
            _pitch_var = ((self._p16_pitch_buf - _pitch_mean.unsqueeze(1))**2).sum(dim=1) / _n
            _attitude_var_w = _roll_var + _pitch_var
            # Only fire when buffer has >= half capacity (avoid first-fill noise)
            _gate = (self._p16_att_filled >= max(W_att // 2, 1)).float()
        else:
            _attitude_var_w = torch.zeros_like(s["v_forward"])
            _gate = torch.zeros_like(s["v_forward"])
        if self.cfg.pass16_attitude_var_enabled:
            r_attitude_var_pass16 = -float(self.cfg.pass16_attitude_var_weight) * _attitude_var_w * _gate
        else:
            r_attitude_var_pass16 = torch.zeros_like(s["v_forward"])

        # Term B — per-link force peak. Uses cs.data.net_forces_w max across
        # bodies (the same source the Pass 14 obs uses), windowed p99.
        if self.cfg.pass16_force_peak_enabled:
            try:
                _sensor = self.scene["contact_sensor"]
                _F_arr = _sensor.data.net_forces_w
                _F_per_body = _F_arr.norm(dim=-1)
                _F_max_now = _F_per_body.max(dim=-1).values
            except Exception:
                _F_max_now = torch.zeros_like(s["v_forward"])
            W_f = self._p16_force_W
            _idx_f = self._p16_fmax_idx
            _arange_f = torch.arange(self.num_envs, device=self.device)
            self._p16_fmax_buf[_arange_f, _idx_f] = _F_max_now
            self._p16_fmax_idx    = (_idx_f + 1) % W_f
            self._p16_fmax_filled = torch.minimum(
                self._p16_fmax_filled + 1,
                torch.full_like(self._p16_fmax_filled, W_f))
            # p99 approximated as quantile over filled samples. Easier: use max
            # of the buffer (since p99 ≈ max for small W).
            _F_max_p99 = self._p16_fmax_buf.max(dim=1).values
            _F_excess = torch.clamp(
                _F_max_p99 - float(self.cfg.pass16_force_safe_N), min=0.0)
            _gate_f = (self._p16_fmax_filled >= max(W_f // 2, 1)).float()
            r_force_peak_pass16 = -float(self.cfg.pass16_force_peak_weight) * _F_excess * _F_excess * _gate_f
        else:
            r_force_peak_pass16 = torch.zeros_like(s["v_forward"])
            _F_max_p99 = torch.zeros_like(s["v_forward"])

        # ── Pass 17 / Pass 18 (2026-05-19): K=12 sensor → severity-belief
        #    estimator + severity-weighted speed (A) / K=12 imitation (B). ──
        # R_belief is computed each step from 12 windowed sensor features.
        # When pass17_k12_estimator_enabled is OFF, the helper returns zeros
        # and both reward terms are identically 0.
        if self.cfg.pass17_k12_estimator_enabled or \
                getattr(self.cfg, "pass20_belief_robustness_enabled", False):
            # P4 v3.4 Round B (2026-05-31): pass20 LOCKS the belief to K=12 and
            # auto-computes it (PGZ is circular for an attitude-variance penalty).
            _R_belief, _R_belief_raw = self._compute_k12_R_belief(self.robot)
        elif getattr(self.cfg, "pass17_projgz_estimator_enabled", False):
            # P4 v3.4 Round A (2026-05-31): PGZ belief from ONLINE-SENSED proj_g_z
            # ONLY (no terrain ground-truth). Supplies R_belief to the SAME r_25
            # term at the SAME weight ⇒ controlled A/B vs the K=12 belief.
            _R_belief, _R_belief_raw = self._compute_pgz_R_belief(self.robot)
        else:
            _R_belief = torch.zeros_like(s["v_forward"])
            _R_belief_raw = torch.zeros_like(s["v_forward"])

        if self.cfg.pass17_severity_weighted_speed_enabled:
            r_25_severity_weighted_speed = (
                float(self.cfg.pass17_severity_weighted_speed_weight)
                * s["v_forward"] * _R_belief
            )
        else:
            r_25_severity_weighted_speed = torch.zeros_like(s["v_forward"])

        if self.cfg.pass18_k12_imitation_enabled:
            # wave_number_scale_runtime is (E,) on the controller. Multiply by
            # baseline n_wave (3.0) to get applied_abs in the same units the
            # K=12 selector dry-run produced (∈ [1.5, 7.5] policy range).
            #
            # 2026-05-20 OPTION-B target-formula parameter recalibration
            # (user-authorized; no fresh ChatGPT packet — change is a
            # constant tweak, not a structural reward redesign):
            #   target ∈ [2.0, 4.5]  (was [1.5, 3.5] until 2026-05-20)
            # Reason: at w_B=0.05 with target ∈ [1.5, 3.5], the trained
            # policy's natural applied_abs band (3.0-3.2) was ABOVE target
            # everywhere; r_26 gradient pulled flat (Δ≈0.66) much harder
            # than rough (Δ≈0.16), reversing the bio-prior sign at ep30
            # (Spearman r = -0.77). The new bounds [2.0, 4.5] center the
            # baseline band INSIDE the target range so the gradient pulls
            # rough UP and flat DOWN as intended. Snapshot at
            # envs/centipede_env.py.before_target_formula_mod_2026-05-20.
            _applied_abs_chosen = self.controller.wave_number_scale_runtime * 3.0
            _applied_abs_target = 2.0 + 2.5 * _R_belief                 # ∈ [2.0, 4.5]
            self._applied_abs_target_k12 = _applied_abs_target.detach()
            _imit_diff = _applied_abs_chosen - _applied_abs_target
            r_26_k12_imitation = (
                -float(self.cfg.pass18_k12_imitation_weight)
                * _imit_diff * _imit_diff
            )
        else:
            r_26_k12_imitation = torch.zeros_like(s["v_forward"])
            _applied_abs_target = torch.zeros_like(s["v_forward"])

        # ── P4 v3.4 Round B (2026-05-31): belief-conditioned attitude-var robustness.
        #    Penalize body attitude variance MORE when the K=12 SENSOR belief says
        #    the terrain is rough. The REWARD NEVER MENTIONS WAVE — if the policy
        #    raises body wave to stay stable on sensed-rough ground, that is EMERGENT.
        #    Reuses _attitude_var_w + _gate (computed above when pass20 on) and the
        #    K=12 _R_belief. Sensor-only: NO terrain ground-truth, NO wave/applied_abs,
        #    NO force/speed/contact in this term.
        if getattr(self.cfg, "pass20_belief_robustness_enabled", False):
            _w_belief20 = 1.0 + _PASS20_KAPPA * _R_belief                 # in [1.0, 1.5]
            _pass20_coef = _PASS20_W_FRAC * float(self.cfg.pass16_attitude_var_weight)
            r_20_belief_robustness = (
                -_pass20_coef * _w_belief20 * _attitude_var_w * _gate)
        else:
            r_20_belief_robustness = torch.zeros_like(s["v_forward"])

        # ── P4 v3.4 Arm 2 (2026-06-03): stability (low-tilt) reward ──────────
        # Penalize body-TILT = sqrt(proj_g_x² + proj_g_y²) (proj_g_b is the gravity
        # unit-vector in the body frame = root ORIENTATION only ⇒ proprioception;
        # NO terrain label / severity / block / x / amplitude / λ / LUT). UNIFORM
        # weight: auto-conditions because tilt is ~14× larger on rough, so the
        # penalty is far larger there ⇒ drives more wave on rough (higher wave →
        # less tilt, causal) WITHOUT any forbidden severity input. Default OFF ⇒
        # 0 ⇒ byte-identical.
        _pg_b_tilt = self.robot.data.projected_gravity_b          # (E, 3), body-frame IMU
        _tilt_mag = torch.sqrt(_pg_b_tilt[:, 0] ** 2 + _pg_b_tilt[:, 1] ** 2)
        if bool(getattr(self.cfg, "stability_tilt_reward_enabled", False)):
            r_stability_tilt = -float(self.cfg.stability_tilt_weight) * _tilt_mag
        else:
            r_stability_tilt = torch.zeros_like(s["v_forward"])

        r_total = (r_v + r_alive + r_ori + r_ang + r_act + r_rate
                   + r_tau + r_zvel + r_imp + r_wave + r_fail
                   + r_effort + r_cot
                   + r_speed_floor + r_struct_guard + r_ang_split
                   + r_functional_guard
                   + r_robust_pass13
                   + legacy_r_fmax_gated          # P4 v3.4 Round C: == r_fmax when OFF, 0 when rC0 ON
                   + rC0_impact                   # P4 v3.4 Round C C0 impact term (0 when OFF)
                   + r_v_fwd_floor_pass15
                   + r_wave_softcap_pass15
                   + r_attitude_var_pass16
                   + r_force_peak_pass16
                   + r_25_severity_weighted_speed
                   + r_26_k12_imitation
                   + r_20_belief_robustness
                   + r_stability_tilt)            # P4 v3.4 Arm 2 (0 when OFF)

        # ── SPEED-ONLY reward override (2026-06-04, DO_NOT_SHIP oracle/credit probe) ──
        # Replace the composed r_total with ONLY {r_v (windowed), r_fail, r_ori,
        # r_alive} — the sweep's own forward-speed objective + a safety floor that
        # does not bias the wave choice. Overriding the FULL sum (rather than zeroing
        # ~24 terms individually) is robust: no term can be missed regardless of which
        # Pass flags are on. r_v is already windowed when windowed_vfwd_reward_enabled.
        # Default OFF => override skipped => r_total byte-identical to baseline.
        if bool(getattr(self.cfg, "reward_speed_only_enabled", False)):
            self._speedonly_excluded = (r_total - (r_v + r_fail + r_ori + r_alive)).detach()
            r_total = r_v + r_fail + r_ori + r_alive
            # ── STEP B reward-atlas additive re-add (2026-06-13, PI-signed GO_REWARD_ATLAS) ──
            # The override above strips the FULL composed sum. To run an atlas variant
            # ("speed-only base + ONE legal reward term") the flagged term must survive the
            # strip. Re-add ONLY the explicitly-flagged LEGAL-menu terms; each r_X is already
            # EXACTLY 0 when its own flag is OFF (effort/cot: *_term≡0 upstream, :~2168/:~2174;
            # the rest: else→zeros_like). So with NO atlas flag set, _atlas_extra≡0 and r_total
            # stays byte-identical to the prior speed-only base (the frozen node) — proven by the
            # flag-OFF bit-identity control. jitter is re-added separately below. FORBIDDEN terms
            # (wave_softcap/robust/sev-weighted/k12/belief) and the always-on pass12 guards
            # (struct/ang_split/functional) are NOT re-added. Adds no extras['log'] key (no eval
            # CSV column change). Default OFF (reward_speed_only_enabled False) ⇒ block skipped.
            _atlas_extra = (r_effort + r_cot + r_speed_floor
                            + r_v_fwd_floor_pass15 + r_attitude_var_pass16
                            + r_force_peak_pass16 + r_stability_tilt)
            r_total = r_total + _atlas_extra
            self._speedonly_atlas_extra = _atlas_extra.detach()
            if not getattr(self, "_speedonly_printed", False):
                self._speedonly_printed = True
                print(f"[reward-speed-only] ON (DO_NOT_SHIP): r_total = r_v + r_fail + r_ori + "
                      f"r_alive + atlas-flagged legal terms. step-mean r_v={r_v.mean().item():+.5f} "
                      f"r_fail={r_fail.mean().item():+.5f} r_ori={r_ori.mean().item():+.5f} "
                      f"r_alive={r_alive.mean().item():+.5f} | atlas_extra "
                      f"mean={self._speedonly_atlas_extra.mean().item():+.5f} | EXCLUDED "
                      f"stripped-terms mean={self._speedonly_excluded.mean().item():+.5f}", flush=True)

        # ── GO r_jitter BUILD (2026-06-09): CV-type forward-speed-choppiness ──
        # r_jitter = -W_JITTER * clip(CV_w, 0, CV_CAP); CV_w = std_w / max(mean_w,
        # V_FLOOR) over the SAME trailing window windowed r_v maintains (_vfwd_buf,
        # updated earlier this step). V_FLOOR = 0.010 (= the probe's frac<10
        # threshold) => standing still yields CV~0, NOT a blow-up; CV_CAP = 3.0
        # bounds transients. std over VALID slots mirrors _windowed_vfwd_update
        # masking (zero-filled unwritten slots excluded via _vfwd_cnt). Reads v_fwd
        # ONLY — no wave/applied_abs/severity/block/x/amp/λ/LUT. Placed AFTER the
        # speed-only override so it survives the r_total replacement (the
        # directive's base). Default OFF => block skipped => byte-identical.
        if bool(getattr(self.cfg, "jitter_penalty_enabled", False)):
            _jit_w = float(getattr(self.cfg, "jitter_penalty_weight", 0.05))
            if (bool(getattr(self.cfg, "windowed_vfwd_reward_enabled", False))
                    and getattr(self, "_vfwd_buf", None) is not None):
                _jit_cnt = self._vfwd_cnt.clamp(min=1).to(torch.float32)
                _jit_mean = self._vfwd_last_windowed
                _jit_msq = (self._vfwd_buf * self._vfwd_buf).sum(dim=1) / _jit_cnt
                _jit_std = torch.sqrt(torch.clamp(_jit_msq - _jit_mean * _jit_mean, min=0.0))
                _jit_cv = torch.clamp(_jit_std / torch.clamp(_jit_mean, min=0.010),
                                      min=0.0, max=3.0)
            else:
                # misconfiguration guard (windowed r_v OFF => no buffer): CV := 0 so
                # the run does not crash; the screen base always has windowed ON.
                _jit_cv = torch.zeros_like(s["v_forward"])
            self._last_r_jitter = -_jit_w * _jit_cv
            self._last_jitter_cv = _jit_cv
            r_total = r_total + self._last_r_jitter
            if not getattr(self, "_jitter_printed", False):
                self._jitter_printed = True
                print(f"[jitter] ON W_JITTER={_jit_w} "
                      f"W={int(getattr(self.cfg, 'vfwd_window_steps', 80))} "
                      f"V_FLOOR=0.010 CV_CAP=3.0 | first-step mean CV_w="
                      f"{_jit_cv.mean().item():.3f} r_jitter="
                      f"{self._last_r_jitter.mean().item():+.5f}", flush=True)

        # ── Per-component logging into extras['log'] ──
        # rl_games' IsaacAlgoObserver picks up extras['log'] (renamed
        # to 'episode' by the wrapper) and writes scalar means to
        # tensorboard. We pass per-env (E,) tensors; rl_games handles
        # mean reduction internally.
        log = self.extras.get("log", {})
        # r_jitter logging is INSIDE the flag so flag-OFF adds NO new log keys
        # (eval CSV columns derive from log keys => key-set change would break
        # flag-OFF bit-identity).
        if bool(getattr(self.cfg, "jitter_penalty_enabled", False)):
            log["reward/31_jitter"] = self._last_r_jitter.detach()
            log["raw/jitter_cv_w"] = self._last_jitter_cv.detach()
        log["reward/01_forward_velocity"]   = r_v.detach()
        log["reward/02_alive"]              = r_alive.detach()
        log["reward/03_orientation"]        = r_ori.detach()
        log["reward/04_angular_velocity"]   = r_ang.detach()
        log["reward/05_action_norm"]        = r_act.detach()
        log["reward/06_action_rate"]        = r_rate.detach()
        log["reward/07_torque"]             = r_tau.detach()
        log["reward/08_vertical_velocity"]  = r_zvel.detach()
        log["reward/09_impedance_reg"]      = r_imp.detach()
        log["reward/10_wave_num_reg"]       = r_wave.detach()
        log["reward/11_failure"]            = r_fail.detach()
        log["reward/12_total"]              = r_total.detach()
        # P4 v3.4 Arm 2: stability (low-tilt) reward term + raw tilt magnitude.
        log["reward/30_stability_tilt"]     = r_stability_tilt.detach()
        log["raw/tilt_mag"]                 = _tilt_mag.detach()
        # Pass 10: effort-aware term + raw power / gate diagnostics.
        log["reward/13_effort_pass10"]      = r_effort.detach()
        log["raw/power_t_W"]                = power_t.detach()
        log["raw/gate_t"]                   = gate_t.detach()
        # Pass 11: windowed CoT hinge term + diagnostics.
        log["reward/14_cot_pass11"]         = r_cot.detach()
        log["raw/cot_window"]               = cot_window.detach()
        log["raw/work_window_J"]            = work_window.detach()
        log["raw/dx_window_m"]              = dx_window.detach()
        log["raw/gate_x_pass11"]            = gate_x.detach()
        # Pass 12: candidate-specific reward shapers (each ≡ 0 when its flag is False).
        log["reward/15_speed_floor_pass12"]      = r_speed_floor.detach()
        log["reward/16_structural_guard_pass12"] = r_struct_guard.detach()
        log["reward/17_ang_split_pass12"]        = r_ang_split.detach()
        log["reward/18_functional_guard_pass12"] = r_functional_guard.detach()
        # M5-A 2026-05-16: clipped-ω² diagnostics (only when ang_split is on).
        if self.cfg.pass12_ang_split_enabled and self._m5a_ang_rp_raw is not None:
            log["raw/m5a_ang_rp_sq_unclipped"]  = self._m5a_ang_rp_raw
            log["raw/m5a_ang_rp_sq_clipped"]    = self._m5a_ang_rp_clipped
            log["raw/m5a_ang_yaw_sq_unclipped"] = self._m5a_ang_y_raw
            log["raw/m5a_ang_yaw_sq_clipped"]   = self._m5a_ang_y_clipped
            log["raw/m5a_frac_clipped_rp"] = (
                self._m5a_ang_rp_raw >= self._PASS12_CAP_RP_SQ
            ).float()
            log["raw/m5a_frac_clipped_yaw"] = (
                self._m5a_ang_y_raw  >= self._PASS12_CAP_YAW_SQ
            ).float()
        # Pass 13: robustness reward + per-env severity diagnostic.
        log["reward/19_robust_pass13"]           = r_robust_pass13.detach()
        log["raw/severity_S_per_env"]            = S_per_env.detach()
        # C09-A REVISED 2026-05-16: F_max progress-gated penalty + diagnostics.
        # P4 v3.4 Round C (Gate-2 d#1): reward/20 logs the ACTUALLY-SUMMED legacy
        # value (== r_fmax when OFF, 0 when rC0 ON) so reward_decomposition
        # reconciles to reward/12_total; the original pre-gate r_fmax is preserved
        # at reward/20b for diagnostic continuity. rC0 impact at reward/28.
        log["reward/20_impact_c09a"]             = self._legacy_r_fmax_gated_val
        log["reward/20b_impact_c09a_pregate"]    = r_fmax.detach()
        log["reward/28_impact_rC0"]              = self._rC0_impact_val
        log["raw/rC0_active"]                    = torch.full(
            (self.num_envs,), self._rC0_active, dtype=torch.float32, device=self.device)
        log["raw/rC0_force_raw_Fmax"]            = self._rC0_force_raw_Fmax
        log["raw/rC0_gate"]                      = self._rC0_gate
        # P4 v3.4 Round C C1: when rC1 on, reward/28 above carries the belief-weighted
        # impact (rC0_impact = -w·belief_weight·excess·gate). reward/29 mirrors it for
        # the C1 decomposition; raw/rC1_belief_weight + raw/rC1_active for diagnostics.
        log["reward/29_belief_weighted_impact"]  = self._rC0_impact_val
        log["raw/rC1_belief_weight"]             = self._rC1_belief_weight
        log["raw/rC1_active"]                    = torch.full(
            (self.num_envs,), self._rC1_active, dtype=torch.float32, device=self.device)
        log["raw/c09a_F_max"]                    = self._c09a_F_max_raw
        log["raw/c09a_F_excess"]                 = self._c09a_F_excess
        log["raw/c09a_gate"]                     = self._c09a_gate
        # Pass 15 2026-05-18: v_forward floor + applied_abs soft cap.
        # Both identically 0 when their flag is False.
        log["reward/21_v_fwd_floor_pass15"]      = r_v_fwd_floor_pass15.detach()
        log["reward/22_wave_softcap_pass15"]     = r_wave_softcap_pass15.detach()
        # Pass 16 2026-05-19: attitude variance + force peak (functional outcomes).
        # Both identically 0 when their flag is False.
        log["reward/23_attitude_var_pass16"]     = r_attitude_var_pass16.detach()
        log["reward/24_force_peak_pass16"]       = r_force_peak_pass16.detach()
        log["raw/pass16_attitude_var_w"]         = _attitude_var_w.detach()
        log["raw/pass16_F_max_p99_w"]            = _F_max_p99.detach()
        log["reward/27_belief_robustness_pass20"] = r_20_belief_robustness.detach()
        log["raw/pass20_belief_robustness_active"] = torch.full(
            (self.num_envs,),
            float(getattr(self.cfg, "pass20_belief_robustness_enabled", False)),
            dtype=torch.float32, device=self.device)
        log["raw/pass16_attitude_active"]        = torch.full(
            (self.num_envs,), float(self.cfg.pass16_attitude_var_enabled),
            dtype=torch.float32, device=self.device,
        )
        log["raw/pass16_force_active"]           = torch.full(
            (self.num_envs,), float(self.cfg.pass16_force_peak_enabled),
            dtype=torch.float32, device=self.device,
        )
        log["raw/pass15_v_fwd_floor_active"]     = torch.full(
            (self.num_envs,),
            float(self.cfg.pass15_v_fwd_floor_enabled),
            dtype=torch.float32, device=self.device,
        )
        log["raw/pass15_wave_softcap_active"]    = torch.full(
            (self.num_envs,),
            float(self.cfg.pass15_wave_softcap_enabled),
            dtype=torch.float32, device=self.device,
        )
        log["raw/applied_abs_per_env"]           = applied_abs_pass15.detach()
        # Pass 17 2026-05-19: K=12 estimator + severity-weighted speed reward.
        # Pass 18 2026-05-19: K=12 imitation reward.
        log["reward/25_severity_weighted_speed_pass17"] = r_25_severity_weighted_speed.detach()
        log["reward/26_k12_imitation_pass18"]           = r_26_k12_imitation.detach()
        log["raw/R_belief"]                              = _R_belief.detach()
        log["raw/R_belief_raw"]                          = _R_belief_raw.detach()
        log["raw/applied_abs_target_pass18"]             = _applied_abs_target.detach()
        log["raw/k12_buffer_filled"]                     = self._k12_filled.float()
        log["raw/pass17_estimator_active"]               = torch.full(
            (self.num_envs,),
            float(self.cfg.pass17_k12_estimator_enabled),
            dtype=torch.float32, device=self.device,
        )
        log["raw/pass17_severity_weighted_speed_active"] = torch.full(
            (self.num_envs,),
            float(self.cfg.pass17_severity_weighted_speed_enabled),
            dtype=torch.float32, device=self.device,
        )
        log["raw/pass18_k12_imitation_active"]           = torch.full(
            (self.num_envs,),
            float(self.cfg.pass18_k12_imitation_enabled),
            dtype=torch.float32, device=self.device,
        )
        # Design A 2026-05-16: flag mirror for trace filtering.
        log["raw/designA_fixed_imp_freq_active"] = torch.full(
            (self.num_envs,),
            float(self.cfg.designA_fixed_imp_freq_enabled),
            dtype=torch.float32, device=self.device,
        )
        # Design B+ 2026-05-16: flag mirror for trace filtering.
        log["raw/designBplus_narrow_legs_active"] = torch.full(
            (self.num_envs,),
            float(self.cfg.designBplus_narrow_legs_enabled),
            dtype=torch.float32, device=self.device,
        )
        # Design C 2026-05-17: flag mirror for trace filtering.
        log["raw/designC_fix_all_non_wave_active"] = torch.full(
            (self.num_envs,),
            float(self.cfg.designC_fix_all_non_wave_enabled),
            dtype=torch.float32, device=self.device,
        )
        # Pass 14 2026-05-17: obs-v2 windowed-severity feature logs
        # (always written; reflect last _get_observations call's values).
        # raw/* are the pre-z-score windowed quantities; z/* are the
        # ±3σ-clipped z-scored obs values. The fifth feature
        # (acc_z_rms_w) is LOG-ONLY and NOT in the obs vector.
        log["raw/pass14_obs_v2_active"] = torch.full(
            (self.num_envs,), float(self.cfg.pass14_obs_v2_enabled),
            dtype=torch.float32, device=self.device,
        )
        if hasattr(self, "_pass14_last_raw"):
            log["raw/obs_v2_acc_fwd_rms_w"]   = self._pass14_last_raw[:, 0].detach()
            log["raw/obs_v2_acc_z_rms_w"]     = self._pass14_last_raw[:, 1].detach()  # LOG-only
            log["raw/obs_v2_F_rate_p99_w"]    = self._pass14_last_raw[:, 2].detach()
            log["raw/obs_v2_F_max_p95_w"]     = self._pass14_last_raw[:, 3].detach()
            log["raw/obs_v2_entropy_std_w"]   = self._pass14_last_raw[:, 4].detach()
            log["raw/obs_v2_z_acc_fwd_rms"]   = self._pass14_last_z[:, 0].detach()
            log["raw/obs_v2_z_F_rate_p99"]    = self._pass14_last_z[:, 1].detach()
            log["raw/obs_v2_z_F_max_p95"]     = self._pass14_last_z[:, 2].detach()
            log["raw/obs_v2_z_entropy_std"]   = self._pass14_last_z[:, 3].detach()
        # Pass 12 diagnostic: which W_COT was used in r_cot above.
        # Stored as a (E,) constant so the audit can pull it via env.extras.
        log["raw/w_cot_active"] = torch.full(
            (self.num_envs,), float(_w_cot_active),
            dtype=torch.float32, device=self.device,
        )
        # Also expose the unweighted scalars so the user can spot
        # where the magnitude is going without back-computing W*x:
        log["raw/v_forward_mps"]            = s["v_forward"].detach()
        log["raw/z_vel_mps"]                = z_vel.detach()
        # raw/body_imp_scale is CONSTANT 1.0 on every env, every step, since 2026-07-16 — it is no
        # longer a measurement of anything. It used to report controller.body_kp_scale_runtime (the
        # 6D action's ch-4 impedance modulation as actually consumed by the controller's PD law);
        # the implicit-only unification DELETED that attribute and the explicit torque path with it,
        # so the value here is hard-coded ones (see the imp_term block in _get_rewards). The key is
        # KEPT because downstream analysis / eval-CSV columns read it and a log key-set change breaks
        # flag-OFF bit-identity. DO NOT read a flat 1.0 here as "the policy chose baseline impedance"
        # — the policy has no impedance channel at all. The live impedance is env.set_joint_impedance()
        # -> PhysX; to log it, log the TABLE. See _S2_DEAD_ACTION_NOTE at the top of this file.
        log["raw/body_imp_scale"]           = body_imp_scale.detach()
        log["raw/wave_num_scale"]           = wave_num_scale.detach()
        # Pass 10 (2026-05-08): expose the four remaining u_target dim
        # scales so per-epoch tensorboard can detect body_amp / freq /
        # leg-amp collapse during training. Pure diagnostic — no
        # reward effect. body_amp_scale_runtime is (E, 19) in
        # global_6d (broadcast); read joint 0 as representative.
        body_amp_scale = self.controller.body_amp_scale_runtime[:, 0]
        leg_yaw_amp_scale   = self.controller.leg_yaw_amp_scale_runtime
        leg_pitch_amp_scale = self.controller.leg_pitch_amp_scale_runtime
        freq_scale          = self.controller.freq_scale_runtime
        log["raw/body_amp_scale"]       = body_amp_scale.detach()
        log["raw/leg_yaw_amp_scale"]    = leg_yaw_amp_scale.detach()
        log["raw/leg_pitch_amp_scale"]  = leg_pitch_amp_scale.detach()
        log["raw/freq_scale"]           = freq_scale.detach()
        # Lower-bound proximity fractions — collapse detectors. Bounds
        # are (in envs/centipede_env.py): body_amp [0.3, 1.5], freq
        # [0.7, 1.3]. We tag a sample as "near LO" if it's within
        # ~10% of the lower bound.
        log["raw/body_amp_at_lo_frac"] = (
            (body_amp_scale < 0.40).to(dtype=torch.float32).detach()
        )
        log["raw/freq_at_lo_frac"] = (
            (freq_scale < 0.75).to(dtype=torch.float32).detach()
        )
        log["raw/tau_mean_Nm"]              = s["tau"].abs().mean(dim=1).detach()
        log["raw/action_norm_sq"]           = act_term.detach()
        # Pass 6: rate is now in scaled-parameter space (||u_t-u_{t-1}||²).
        log["raw/action_rate_sq"]           = rate_term.detach()
        log["raw/scaled_action_l2"]         = scaled_u.norm(dim=1).detach()
        # Pass 8: log u_target and u_applied separately so the next
        # audit can directly compare the policy-commanded global-gait
        # parameters against the controller-consumed (smoothed) ones.
        # Both are (E, 6); we log L2 magnitudes here. Per-dim values
        # are still recoverable from the audit script's per-step
        # rollout (it reads runtime fields = u_applied directly).
        if hasattr(self, "_u_target") and hasattr(self, "_u_applied"):
            log["raw/u_target_l2"]      = self._u_target.norm(dim=1).detach()
            log["raw/u_applied_l2"]     = self._u_applied.norm(dim=1).detach()
            # Smoothing-effect ratio per env: ||u_target - u_applied|| /
            # max(||u_target - 1||, eps) — closer to 0 means u_applied
            # has caught up; closer to 1 means u_target just stepped
            # and u_applied lags behind.
            target_minus_one = (self._u_target - 1.0).norm(dim=1)
            target_minus_applied = (
                self._u_target - self._u_applied
            ).norm(dim=1)
            log["raw/u_smoothing_lag_ratio"] = (
                target_minus_applied / (target_minus_one + 1e-6)
            ).detach()
        # Pass 19 (2026-05-21): extended-trace log keys.
        # Written only when pass19_extended_trace_enabled is True; flag-OFF
        # path is bit-identical to the pre-edit env. The 6 keys feed
        # offline K=12 v2 refit (§1.main of obs-enrichment packet v3).
        if bool(self.cfg.pass19_extended_trace_enabled):
            log["raw/foot_force_vector"]  = self._pass19_last_foot_force_vector.detach()
            log["raw/foot_force_mag"]     = self._pass19_last_foot_force_mag.detach()
            log["raw/foot_contact_state"] = self._pass19_last_foot_contact_state.detach()
            log["raw/foot_contact_dur"]   = self._pass19_last_foot_contact_dur.detach()
            log["raw/seg_pitch"]          = self._pass19_last_seg_pitch.detach()
            log["raw/seg_roll"]           = self._pass19_last_seg_roll.detach()
            # Also log the proj_g_z triplet for offline analysis (NOT
            # one of the 6 §1.main keys; bonus diagnostic).
            log["raw/proj_g_z_triplet"]   = self._pass19_last_proj_g_z_triplet.detach()
        self.extras["log"] = log

        return r_total.to(dtype=torch.float32)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Stage B-v0 termination: failure (fall / tip / NaN) OR timeout."""
        s = getattr(self, "_step_state", None)
        if s is None:
            s = self._compute_step_state()
        terminated = s["fail"]
        truncated  = self._sim_time >= self.cfg.episode_length_s
        return terminated, truncated

    # =========================================================================
    # P4 v3.4 Round A (2026-05-31): PGZ sensor-only severity-belief (proj_g_z)
    # =========================================================================

    def _compute_pgz_R_belief(self, art):
        """R_belief from ONLINE-SENSED proj_g_z ONLY (no terrain ground-truth).

        Runtime input is art.data.projected_gravity_b[:, 2] — the SAME sensed
        projected-gravity-z the reward/obs already read (e.g. env.py:1671, 2171).
        Constants _PGZ_OLS_A/_B/_R_LOW/_R_HIGH were fitted OFFLINE against spawn-
        block severity (Constraint 2 is a runtime rule — NO ground-truth tensor
        enters here). Returns (R_belief (E,), R_belief_raw (E,)).
        """
        proj_g_z = art.data.projected_gravity_b[:, 2].to(dtype=torch.float32)  # (E,)
        R_raw = _PGZ_OLS_A + _PGZ_OLS_B * proj_g_z
        R_norm = torch.clamp(
            (R_raw - _PGZ_R_LOW) / (_PGZ_R_HIGH - _PGZ_R_LOW), 0.0, 1.0)
        R_belief = R_norm * R_norm * (3.0 - 2.0 * R_norm)  # smoothstep in [0,1]
        return R_belief, R_raw

    # =========================================================================
    # Pass 17 (2026-05-19): K=12 sensor → severity-belief estimator
    # =========================================================================

    def _compute_k12_R_belief(self, art):
        """Compute R_belief from 12 sensor-derived windowed features.

        Mirrors the offline pipeline that fitted the K=12 ridge model
        (multi_window_12s_audit.py + build_k12_estimator_and_selector.py)
        bit-for-bit on feature formulas, so the deployed coefs apply
        without re-fitting. Returns (R_belief (E,), R_belief_raw (E,)).

        Inputs (all sensor-derived; no terrain labels):
          - contact_sensor.net_forces_w  (E, 38, 3) → per-foot force mag
          - articulation.body_link_quat_w  → per-segment roll/pitch
          - articulation.root_ang_vel_b   → root roll-rate scalar

        Buffer pattern: Pass-16 style circular indexing with per-env idx
        + filled count. When buffer not full (< W=20 steps post-reset),
        R_belief is gated to 0 to avoid noisy initial-fill values.
        """
        E   = self.num_envs
        W   = self._k12_W
        dev = self.device
        dt  = float(self.cfg.decimation) * float(self.cfg.sim.dt)  # dt_pol

        # ── Resolve sensor + per-segment body ids (lazy) ──
        sensor = None
        if hasattr(self.scene, "sensors"):
            sensor = self.scene.sensors.get("contact_sensor", None)
        if sensor is None:
            try:
                sensor = self.scene["contact_sensor"]
            except Exception:  # noqa: BLE001
                sensor = None
        if sensor is None or self._foot_idx_grid is None:
            return (torch.zeros(E, dtype=torch.float32, device=dev),
                    torch.zeros(E, dtype=torch.float32, device=dev))
        if self._k12_seg_body_ids is None:
            body_names = list(art.body_names)
            seg_ids = []
            for i in range(_PASS17_N_SEG_BODY):
                name = f"link_body_{i}"
                if name in body_names:
                    seg_ids.append(body_names.index(name))
            if not seg_ids:
                # Fallback: zeros forever (will be visible in raw/R_belief log).
                return (torch.zeros(E, dtype=torch.float32, device=dev),
                        torch.zeros(E, dtype=torch.float32, device=dev))
            self._k12_seg_body_ids = torch.tensor(
                seg_ids, dtype=torch.long, device=dev)

        # ── Per-step sensor read ──
        F_arr      = sensor.data.net_forces_w                  # (E, 38, 3)
        F_per_body = F_arr.norm(dim=-1)                        # (E, 38)
        F_pair     = F_per_body[:, self._foot_idx_grid]        # (E, 19, 2)
        F_flat     = F_pair.reshape(E, _PASS17_N_FOOT)         # (E, 38)
        in_contact = (F_flat > _PASS17_F_THRESHOLD_N).float()  # (E, 38)

        # Persistent contact-duration state: += dt while in contact;
        # reset to 0 on lift-off. Same formula as the trace collector.
        self._k12_dur_state = torch.where(
            in_contact > 0,
            self._k12_dur_state + dt,
            torch.zeros_like(self._k12_dur_state))

        seg_quat = art.data.body_link_quat_w[:, self._k12_seg_body_ids, :]  # (E, n_seg, 4)
        sR, sP   = _quat_wxyz_to_roll_pitch(seg_quat)          # each (E, n_seg)
        seg_pitch_sp_var = sP.var(dim=1, unbiased=False)        # (E,)

        F_mean_pf = F_flat.mean(dim=1).clamp_min(1e-9)         # (E,)
        F_std_pf  = F_flat.std(dim=1, unbiased=False)          # (E,)
        ci_t      = F_std_pf / F_mean_pf                       # (E,) contact_imbalance

        roll_a_t  = art.data.root_ang_vel_b[:, 0]              # (E,) root roll-rate

        # ── Push into rolling buffers (env-level lockstep — all envs share
        #     the same circular index in this implementation, since they
        #     all step together. Per-env idx is kept for reset isolation.)
        idx = self._k12_idx  # (E,) — all values equal in lockstep
        _arange = torch.arange(E, device=dev)
        self._k12_F_foot_buf[_arange, idx, :]   = F_flat
        self._k12_contact_buf[_arange, idx, :]  = in_contact
        self._k12_ci_buf[_arange, idx]          = ci_t
        self._k12_roll_a_buf[_arange, idx]      = roll_a_t
        self._k12_seg_pitch_sp_var_buf[_arange, idx] = seg_pitch_sp_var
        self._k12_idx    = (idx + 1) % W
        self._k12_filled = torch.minimum(
            self._k12_filled + 1, torch.full_like(self._k12_filled, W))

        # ── Compute 12 windowed features from buffers ──
        # Reductions over the full (E, W, ...) buffer. Buffers are zeroed at
        # reset, so envs that have fewer than W samples produce biased
        # values; we gate the final R_belief output to 0 when not full.
        fF = self._k12_F_foot_buf      # (E, W, 38)
        fC = self._k12_contact_buf     # (E, W, 38)

        # 0: contact_imbalance_mean_w20 — windowed mean of per-step ci
        f0 = self._k12_ci_buf.mean(dim=1)
        # 1: duty_mean_w20 — mean over feet of (mean over W of contact)
        duty_pf = fC.mean(dim=1)        # (E, 38)
        f1 = duty_pf.mean(dim=1)
        # 2: air_time_fraction_w20 — 1 - duty_mean
        f2 = 1.0 - f1
        # 3: n_active_mean_w20 — mean over W of (per-step active feet count)
        n_active = fC.sum(dim=2)        # (E, W)
        f3 = n_active.mean(dim=1)
        # 4: duty_std_w20 — spatial std across feet of per-foot duty
        f4 = duty_pf.std(dim=1, unbiased=False)
        # 5: F_argmax_frac_w20 — argmax over feet of (max-over-W per-foot) / 38
        max_per_foot = fF.max(dim=1).values     # (E, 38)
        f5 = max_per_foot.argmax(dim=1).float() / float(_PASS17_N_FOOT)
        # 6: F_link_p95_w20 — p95 of all (W × 38) per-foot force values
        f6 = torch.quantile(fF.reshape(E, -1), 0.95, dim=1)
        # 7: contact_switch_rate_w20 — mean over feet of (|diff over W|.sum) / (W-1)
        if W >= 2:
            trans = (fC[:, 1:, :] - fC[:, :-1, :]).abs().sum(dim=1)  # (E, 38)
            f7 = trans.mean(dim=1) / float(max(W - 1, 1))
        else:
            f7 = torch.zeros(E, dtype=torch.float32, device=dev)
        # 8: mean_contact_dur_w20 — snapshot mean of dur_state
        f8 = self._k12_dur_state.mean(dim=1)
        # 9: seg_pitch_spatial_var_mean_w20 — windowed mean of per-step seg_pitch_sp_var
        f9 = self._k12_seg_pitch_sp_var_buf.mean(dim=1)
        # 10: contact_dur_p95_w20 — snapshot p95 over feet of dur_state
        f10 = torch.quantile(self._k12_dur_state, 0.95, dim=1)
        # 11: roll_var_w20 — windowed variance of root roll-rate
        roll_mean = self._k12_roll_a_buf.mean(dim=1)
        f11 = ((self._k12_roll_a_buf - roll_mean.unsqueeze(1))**2).mean(dim=1)

        feats = torch.stack([f0, f1, f2, f3, f4, f5,
                             f6, f7, f8, f9, f10, f11], dim=1)   # (E, 12)
        # Z-score + ridge sum + smoothstep.
        z = (feats - self._k12_mu_t.unsqueeze(0)) / self._k12_sigma_t.unsqueeze(0)
        R_raw = _K12_INTERCEPT + (z * self._k12_coefz_t.unsqueeze(0)).sum(dim=1)
        R_norm = ((R_raw - _K12_R_LOW) / (_K12_R_HIGH - _K12_R_LOW)).clamp(0.0, 1.0)
        R_belief = R_norm * R_norm * (3.0 - 2.0 * R_norm)        # smoothstep
        # Gate to 0 when buffer is not yet full (≥ W samples since reset).
        fill_mask = (self._k12_filled >= W).float()
        R_belief = R_belief * fill_mask

        self._R_belief     = R_belief
        self._R_belief_raw = R_raw
        return R_belief, R_raw

    # =========================================================================
    # Reset
    # =========================================================================

    def _reset_idx(self, env_ids):
        """Reset the given envs. Called by DirectRLEnv on init and on
        episode end for each env that's done.
        """
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self.robot._ALL_INDICES
        super()._reset_idx(env_ids)

        # Reset robot pose to default.
        default_root = self.robot.data.default_root_state[env_ids].clone()
        default_root[:, 0:3] += self.scene.env_origins[env_ids]

        # Stage B-v0 Pass 3 refined (2026-05-07): single randomized
        # block-spawn mode for transition terrain. Per-env, sample one
        # of the four block centers, jitter, clamp to arena, lookup
        # spawn_z from the heightmap. Transition fields live on the ENV
        # cfg (self.cfg), NOT scene cfg — see CentipedeEnvCfg.
        if getattr(self.cfg, "terrain_kind", "flat") == "transition" \
                and getattr(self, "_transition_heightmap_m", None) is not None:
            ec = self.cfg
            from arena import heightmap_patch_max_at, heightmap_value_at  # noqa: E402  -- kit is up
            block_centers = list(ec.transition_block_centers_x)  # tuple -> list
            n_blocks = len(block_centers)
            arena_half = float(ec.transition_arena_size_m) * 0.5
            margin = float(ec.transition_spawn_margin_m)
            jitter_x = float(ec.transition_spawn_jitter_x)
            jitter_y = float(ec.transition_spawn_jitter_y)
            clearance = float(ec.transition_spawn_clearance_m)
            # Pass 9 stratified-spawn lattice. Default 4 y-strata at
            # ±0.6, ±0.2 m. We keep getattr() with safe defaults so
            # earlier configs (without these fields) still work.
            y_strata_centres = list(getattr(
                ec, "transition_spawn_y_strata_centres",
                (-0.6, -0.2, +0.2, +0.6),
            ))
            n_y_strata = int(getattr(
                ec, "transition_spawn_n_y_strata", len(y_strata_centres)
            ))
            # Spawn-drift fix (2026-06-24, default-OFF flag): the y-stratum is normally drawn from the
            # global call-counter `_spawn_rng`, which DRIFTS when a tipped env auto-resets mid-eval (the
            # counter is keyed to call order, not env identity) -> later envs get the wrong stratum. With
            # `spawn_stratum_by_envid` ON, pin the stratum to `env_id % n_y_strata` (identity-stable, immune
            # to partial resets). Flag OFF = the original counter path, byte-unchanged. The RNG is still
            # advanced identically below so block_idx (n_blocks==1 -> 0) and jitter are unchanged.
            _strat_by_envid = bool(getattr(ec, "spawn_stratum_by_envid", False))

            # env_ids is a tensor; iterate on the host side. Each reset
            # batch is small relative to physics-step costs.
            env_ids_cpu = env_ids.detach().cpu().tolist()
            for row, env_id in enumerate(env_ids_cpu):
                # Pass 9: stratified (block, y-stratum) sampling. Each
                # reset draws an independent (block_idx, y_strat_idx)
                # uniformly over the 6 * n_y_strata grid; within the
                # cell we jitter ±jitter_x in x and ±jitter_y in y.
                block_idx = int(self._spawn_rng.integers(0, n_blocks))
                _rng_y_strat = int(self._spawn_rng.integers(0, n_y_strata))   # advance RNG identically (cadence preserved)
                y_strat_idx = (env_id % n_y_strata) if _strat_by_envid else _rng_y_strat
                cx = float(block_centers[block_idx])
                cy = float(y_strata_centres[y_strat_idx])
                jx = float(self._spawn_rng.uniform(-jitter_x, +jitter_x))
                jy = float(self._spawn_rng.uniform(-jitter_y, +jitter_y))
                spawn_x = max(-arena_half + margin,
                              min(+arena_half - margin, cx + jx))
                spawn_y = max(-arena_half + margin,
                              min(+arena_half - margin, cy + jy))
                # SPAWN SPREAD (2026-07-17, default-OFF): when spawn_spread_pitch>0, override the sampled
                # (x,y) with this env's slot on a ceil(sqrt(num_envs))^2 lattice so NO reset ever stacks at
                # the shared central point (stacked co-located robots corrupt the shared PhysX GPU scene at
                # N>32). Homogeneous arena => every slot is the same lambda/amp terrain, different phase.
                # pitch<=0 => this block is skipped => byte-identical to the stacked central-spawn path.
                _spr_pitch = float(getattr(ec, 'spawn_spread_pitch', 0.0))
                if _spr_pitch > 0.0:
                    import math as _sm2
                    _Gsp = int(_sm2.ceil(_sm2.sqrt(self.num_envs)))
                    _rr, _cc = env_id // _Gsp, env_id % _Gsp
                    spawn_x = max(-arena_half + margin,
                                  min(+arena_half - margin, (_cc - (_Gsp - 1) / 2.0) * _spr_pitch))
                    spawn_y = max(-arena_half + margin,
                                  min(+arena_half - margin, (_rr - (_Gsp - 1) / 2.0) * _spr_pitch))
                # Lookup heightmap at the sampled xy, add clearance.
                if bool(getattr(ec, "transition_spawn_use_collision_patch", False)):
                    collision_hm = getattr(self, "_transition_collision_heightmap_m", None)
                    if collision_hm is None:
                        raise RuntimeError("collision-coordinate spawn heightmap was not initialized")
                    h0 = heightmap_patch_max_at(
                        collision_hm,
                        self._transition_arena_size_m,
                        spawn_x, spawn_y,
                        float(getattr(ec, "transition_spawn_footprint_radius_m", 0.075)),
                    )
                else:
                    h0 = heightmap_value_at(
                        self._transition_heightmap_m,
                        self._transition_arena_size_m,
                        spawn_x, spawn_y,
                    )
                spawn_z = float(h0) + clearance
                # Write WORLD-frame xyz directly (NOT env_origin-offset).
                # All envs share the global terrain mesh, so we land
                # each robot at its sampled world position regardless
                # of its env_origin grid slot.
                default_root[row, 0] = spawn_x
                default_root[row, 1] = spawn_y
                default_root[row, 2] = spawn_z
                # Diagnostic-only block tag (NOT in policy obs).
                self._spawn_block_id[env_id] = block_idx

        self.robot.write_root_pose_to_sim(default_root[:, 0:7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root[:, 7:13], env_ids)

        default_joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        default_joint_vel = self.robot.data.default_joint_vel[env_ids].clone()
        self.robot.write_joint_state_to_sim(
            default_joint_pos, default_joint_vel, None, env_ids
        )

        # Reset per-env time tensor.
        self._sim_time[env_ids] = 0.0

        # Stage B-v0 (Pass 2, 2026-05-07): zero the per-env contact-load
        # history so the observation's temporal-derivative slot does not
        # show a spurious spike right after a reset (otherwise dF =
        # F_now - F_old_other_episode could be huge).
        if hasattr(self, "_previous_F_sum"):
            self._previous_F_sum[env_ids] = 0.0

        # P4 v3.4 windowed-speed reward (2026-06-03): clear the per-env v_forward window on reset so
        # a trailing window never averages across episodes (reset-then-fill). Guarded ⇒ no-op when OFF.
        if hasattr(self, "_vfwd_buf"):
            self._vfwd_buf[env_ids] = 0.0
            self._vfwd_cnt[env_ids] = 0
            self._vfwd_pos[env_ids] = 0

        # Pass 14 (2026-05-17): zero the obs-v2 windowed ring buffers
        # and per-env prev-state for resetting envs. Without this, the
        # first W steps of the next episode would mix stale (prior-
        # episode) buffer values with new readings — the windowed
        # masks below guard against using the partial fill, but
        # explicit zeroing makes diagnostics clean.
        if hasattr(self, "_pass14_acc_fwd_buf"):
            self._pass14_acc_fwd_buf[env_ids] = 0.0
            self._pass14_acc_z_buf[env_ids]   = 0.0
            self._pass14_F_rate_buf[env_ids]  = 0.0
            self._pass14_F_max_buf[env_ids]   = 0.0
            self._pass14_entropy_buf[env_ids] = 0.0
            self._pass14_buf_idx[env_ids]     = 0
            self._pass14_buf_filled[env_ids]  = 0
            self._pass14_prev_v_fwd[env_ids]  = 0.0
            self._pass14_prev_v_z[env_ids]    = 0.0
            self._pass14_prev_F_total[env_ids] = 0.0
            self._pass14_has_prev[env_ids]    = False
            self._pass14_last_raw[env_ids]    = 0.0
            self._pass14_last_z[env_ids]      = 0.0

        # Stage E (2026-07-12): zero the windowed-terrain ring + per-env index/filled for
        # resetting envs (no cross-episode window contamination). Guarded -> no-op OFF.
        if hasattr(self, "_nwo_cbuf"):
            self._nwo_cbuf[env_ids]   = 0.0
            self._nwo_obuf[env_ids]   = 0.0
            self._nwo_tbuf[env_ids]   = 0.0
            self._nwo_pos[env_ids]    = 0
            self._nwo_filled[env_ids] = 0

        # Pass 16 2026-05-19: zero attitude + force-peak windowed buffers
        # for resetting envs.
        if hasattr(self, "_p16_roll_buf"):
            self._p16_roll_buf[env_ids]    = 0.0
            self._p16_pitch_buf[env_ids]   = 0.0
            self._p16_att_idx[env_ids]     = 0
            self._p16_att_filled[env_ids]  = 0
            self._p16_fmax_buf[env_ids]    = 0.0
            self._p16_fmax_idx[env_ids]    = 0
            self._p16_fmax_filled[env_ids] = 0

        # P4 v3.4 Round C C0 (Gate-2 b#3): zero the rC0 windowed-F_max buffer for
        # resetting envs, so buffer.mean does NOT carry a pre-reset episode's force
        # across the reset boundary (cross-episode leak).
        if hasattr(self, "_rC0_fmax_buf"):
            self._rC0_fmax_buf[env_ids]    = 0.0
            self._rC0_fmax_idx[env_ids]    = 0
            self._rC0_fmax_filled[env_ids] = 0

        # Pass 17 2026-05-19: zero K=12 estimator buffers + contact-duration
        # persistent state for resetting envs.
        if hasattr(self, "_k12_F_foot_buf"):
            self._k12_F_foot_buf[env_ids]           = 0.0
            self._k12_contact_buf[env_ids]          = 0.0
            self._k12_ci_buf[env_ids]               = 0.0
            self._k12_roll_a_buf[env_ids]           = 0.0
            self._k12_seg_pitch_sp_var_buf[env_ids] = 0.0
            self._k12_idx[env_ids]                  = 0
            self._k12_filled[env_ids]               = 0
            self._k12_dur_state[env_ids]            = 0.0
            self._R_belief[env_ids]                 = 0.0
            self._R_belief_raw[env_ids]             = 0.0
            self._applied_abs_target_k12[env_ids]   = 0.0

        # Pass 19 2026-05-21: zero pass19 contact-duration counter +
        # proj_g_z 5-tap history + last-feature stashes for resetting
        # envs. Without this, the post-reset first-5-step proj_g_z
        # derivatives would mix stale (prior-episode) samples and the
        # contact_dur counter would carry over.
        if hasattr(self, "_pass19_contact_dur_counter"):
            self._pass19_contact_dur_counter[env_ids]     = 0.0
            self._pass19_proj_g_z_hist[env_ids]           = 0.0
            self._pass19_proj_g_z_filled[env_ids]         = 0
            self._pass19_last_foot_force_vector[env_ids]  = 0.0
            self._pass19_last_foot_force_mag[env_ids]     = 0.0
            self._pass19_last_foot_contact_state[env_ids] = 0.0
            self._pass19_last_foot_contact_dur[env_ids]   = 0.0
            self._pass19_last_seg_pitch[env_ids]          = 0.0
            self._pass19_last_seg_roll[env_ids]           = 0.0
            self._pass19_last_proj_g_z_triplet[env_ids]   = 0.0

        # Stage B-v0 Pass 6 (2026-05-07): reset the scaled-parameter
        # history to 1.0 for the listed envs so the action-rate penalty
        # doesn't show a spurious spike right after a reset. The
        # controller's runtime scales were just reset to 1.0 (above),
        # so prev=1.0 keeps r_rate=0 at the first post-reset step.
        if hasattr(self, "_prev_scaled_action"):
            self._prev_scaled_action[env_ids] = 1.0

        # Stage B-v0 Pass 8 (2026-05-08): reset the smoothed u_applied
        # AND u_target snapshots to 1.0 for the listed envs. controller.
        # reset_envs has just reset all runtime scales to 1.0; we
        # mirror that here so the next env.step's smoothing recursion
        # starts from baseline.
        if hasattr(self, "_u_applied"):
            self._u_applied[env_ids] = 1.0
        if hasattr(self, "_u_target"):
            self._u_target[env_ids] = 1.0
        # P4 interface-smoothing: restart the wave slow-EMA + rate-limited
        # state at baseline applied_abs (scale 1.0 * base) for the resetting
        # envs, mirroring WaveSmoother.reset_envs so the parity test and the
        # training resets stay consistent. hasattr-guarded so flag-OFF /
        # pre-allocation states are safe.
        if hasattr(self, "_wave_ema_abs"):
            self._wave_ema_abs[env_ids] = self._wave_smooth_base
        if hasattr(self, "_wave_abs"):
            self._wave_abs[env_ids] = self._wave_smooth_base
        # P4 v3.4 Phase 1 (2026-06-02): wave-baseline recenter — reset the wave channel
        # (idx 3) to wave_baseline_abs/n_wave so reset also starts at applied_abs=
        # wave_baseline_abs. Default 3.0 ⇒ skipped ⇒ byte-identical. Other 5 channels
        # remain at 1.0. Overrides the 1.0 / _wave_smooth_base inits above for idx 3.
        _wave_base_abs_r = float(getattr(self.cfg, "wave_baseline_abs", 3.0))
        _nwave_ref_r = float(self.cfg.pass15_baseline_n_wave)
        if _wave_base_abs_r != _nwave_ref_r:
            _ws0 = _wave_base_abs_r / _nwave_ref_r
            if hasattr(self, "_u_applied"):
                self._u_applied[env_ids, 3] = _ws0
            if hasattr(self, "_u_target"):
                self._u_target[env_ids, 3] = _ws0
            if hasattr(self, "_wave_ema_abs"):
                self._wave_ema_abs[env_ids] = _wave_base_abs_r
            if hasattr(self, "_wave_abs"):
                self._wave_abs[env_ids] = _wave_base_abs_r

        # Stage B-v0 Pass 11 (2026-05-09): zero the windowed-CoT buffers
        # for the resetting envs so the gate stays off until each env's
        # buffer is filled again. The pass11_prev_root_x for those envs
        # is set to the post-reset root_x so the next step's dx_step is
        # not a spurious spawn-translation jump.
        if hasattr(self, "_pass11_initialized"):
            self._pass11_work_buf[env_ids]   = 0.0
            self._pass11_dx_buf[env_ids]     = 0.0
            self._pass11_buf_count[env_ids]  = 0
            self._pass11_prev_root_x[env_ids] = (
                self.robot.data.root_pos_w[env_ids, 0].detach()
            )

        # Stage B Foundation (2026-05-07): partial-reset support.
        # controller.reset_envs() resets body_phases / leg_phases /
        # head_yaw_ref / prev-target-history ONLY for the listed envs.
        # Per-env t_at_reset is 0.0 for all listed envs (they were just
        # reset; their _sim_time was just zeroed). Equations unchanged;
        # this is pure state management.
        self.controller.reset_envs(env_ids, t_at_reset=0.0)

        # When ALL envs reset together we also zero the host scalar
        # clock for back-compat with any external caller that reads it
        # (e.g. the Stage A smoke harness). The per-env tensor is the
        # authoritative clock from now on.
        try:
            n_resetting = len(env_ids)
        except TypeError:
            n_resetting = int(env_ids.numel()) if hasattr(env_ids, "numel") else 0
        if n_resetting >= self.num_envs:
            self._t_host = 0.0
