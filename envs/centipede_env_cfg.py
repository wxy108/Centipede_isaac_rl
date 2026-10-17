"""
Configuration classes for the Centipede DirectRLEnv.

Mirrors the scene setup from scripts/test_walk.py (single-env reference)
but uses Isaac Lab's InteractiveScene to clone the centipede across
num_envs parallel environments.
"""

from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.envs.common import ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass


# =============================================================================
# Paths
# =============================================================================

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
CENTIPEDE_USD_PATH = os.path.join(PROJECT_ROOT, "assets", "centipede.usd")
CONTROLLER_CONFIG_PATH = os.environ.get("CENTIPEDE_CONTROLLER_YAML") or os.path.join(PROJECT_ROOT, "configs", "farms_controller.yaml")


# =============================================================================
# Contact-sensor factory (Stage B Step 2, 2026-05-07)
# =============================================================================

def make_centipede_contact_sensor(
    update_period: float = 0.0,
    history_length: int = 6,
    track_air_time: bool = True,
    force_threshold: float = 0.005,
) -> "ContactSensorCfg":
    """Build a foot-contact ContactSensorCfg with a USD-verified prim_path.

    The MJCF -> USD conversion of centipede_simple.xml FLATTENS the
    kinematic tree at the USD level: every link_body_<n>, link_leg_*,
    and foot_<n>_<m> rigid body becomes a DIRECT child of the floating
    root link_body_0. (Kinematics is preserved via joints, not USD
    nesting.) Verified 2026-05-07 by
    scripts/sanity/inspect_foot_prim_paths.py — all 38 foot prims share
    the template /centipede_simple/link_body_0/foot_<n>_<m> at depth 3.
    Once the asset is spawned at {ENV_REGEX_NS}/Robot the path becomes
    /World/envs/env_<i>/Robot/link_body_0/foot_<n>_<m>.

    isaaclab.sim.utils.find_matching_prims walks one path-segment at a
    time, so the prim_path below is split into 6 strict per-token
    regexes, with `foot_.*` matching exactly the 38 foot prims as
    direct children of link_body_0.

    PhysxContactReportAPI is auto-applied to every rigid body under
    /Robot because CENTIPEDE_ROBOT_CFG.spawn has
    activate_contact_sensors=True. The foot prims (rigid bodies with
    collision geometry per the MJCF) therefore carry the API.

    Defaults:
      update_period=0.0  -> evaluate every physics step (sim dt = 4 ms).
      history_length=6   -> ~24 ms of history at 4 ms dt.
      track_air_time=True -> populate last_air_time / current_contact_time
                              for stance/swing detection in Step 3.
      force_threshold=0.005
                          -> centipede mass is mg-scale; default 1.0 N
                              would never trigger air-time accounting.
                              Only affects the air-time event clock,
                              NOT net_forces_w.
    """
    return ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link_body_0/foot_.*",
        update_period=update_period,
        history_length=history_length,
        track_air_time=track_air_time,
        force_threshold=force_threshold,
        debug_vis=False,
    )


def make_allbody_contact_sensor(
    update_period: float = 0.0,
    history_length: int = 6,
    with_friction: bool = False,
) -> "ContactSensorCfg":
    """GO_FLING_CAUSE_DEFINITIVE_2026-06-29 §2 (deeper fix) — contact sensor over ALL robot rigid bodies,
    not just the 38 feet.

    The feet-only `contact_sensor` captures only ~58-83 % of the weight on rough terrain: the body segments
    drape onto terrain crests (16 mm bumps) and carry the rest, so the foot-only contact log cannot close the
    Newton momentum balance (linear OR angular). The flattened MJCF->USD makes every link a DIRECT child of
    link_body_0, so `link_body_0/.*` matches all 250 non-root rigid bodies (link_body_1..18 + legs + feet);
    `PhysxContactReportAPI` is already on every body (CENTIPEDE_ROBOT_CFG.spawn.activate_contact_sensors=True),
    so this is a PASSIVE reader — it adds NO dynamics, the robot behaves identically, only more contact data is
    read. Default OFF: the scene field below is None unless the analysis cell sets it before constructing the env."""
    return ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link_body_0/.*",
        update_period=update_period,
        history_length=history_length,
        track_air_time=False,
        debug_vis=False,
        filter_prim_paths_expr=(["/World/GroundPlane", "/World/TransitionTerrain"] if with_friction else []),
        max_contact_data_count_per_prim=(16 if with_friction else 4),
    )


# =============================================================================
# Robot articulation cfg
# =============================================================================
# Actuator groups match scripts/test_walk.py:
#   active joints: effort-mode, stiffness=damping=0 (controller sets torque)
#   foot joints:   passive damping (matches MuJoCo XML)

CENTIPEDE_ROBOT_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=CENTIPEDE_USD_PATH,
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.025),
    ),
    actuators={
        "active": ImplicitActuatorCfg(
            joint_names_expr=[
                "joint_body_.*",
                "joint_pitch_body_.*",
                "joint_leg_.*",
            ],
            effort_limit=10.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=0.0,
        ),
        "foot_passive": ImplicitActuatorCfg(
            joint_names_expr=["joint_foot_.*"],
            effort_limit=1.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=0.006,
        ),
    },
)


# =============================================================================
# Scene cfg
# =============================================================================

@configclass
class CentipedeSceneCfg(InteractiveSceneCfg):
    """N parallel centipedes on a shared flat ground plane."""

    num_envs: int = 64
    # env_spacing is the cell size used by InteractiveScene's cloner.
    #
    # 2026-05-07 Stage B Foundation history:
    #   - Tried env_spacing = 0.6 (visually compact for the cm-scale
    #     centipede). This CRASHED Isaac Sim 5.0's
    #     `isaacsim.core.cloner.replicationAttachEndFn` with a heap
    #     corruption (0xc0000374) on this RTX 5070 Laptop / Win11
    #     build, even at num_envs=4. The crash is INSIDE the cloner
    #     during physics-replication init, so it's NOT about robot
    #     extent vs spacing — likely a PhysX scene-cell minimum that
    #     0.6 m violates on this build.
    #   - Reverted to 1.5 m, which is empirically stable at num_envs=16
    #     in the smoke harness. If you want tighter spacing for video,
    #     use scripts/render_centipede_demo.py --mode stage-a-env-video
    #     and override eye/lookat — the env-side default stays safe.
    #
    # If you want to experiment, override `cfg.scene.env_spacing = X`
    # from your launching script; but expect 5.0 cloner instabilities
    # below ~1.0 m on this hardware.
    env_spacing: float = 1.5  # metres between cloned envs (5.0-safe)

    # NOTE: terrain_kind, transition_*, transition_spawn_* are NOT defined
    # on this CentipedeSceneCfg because InteractiveScene._add_entities_from_cfg
    # iterates every field of the scene cfg and tries to spawn each one
    # as a scene entity (Articulation / Sensor / AssetBaseCfg / ...). A
    # plain-typed field like `terrain_kind: str = "flat"` would crash
    # with `Unknown asset config type`. They live on CentipedeEnvCfg
    # (parent of this scene cfg) instead, and CentipedeEnv reads them
    # via self.cfg.terrain_kind etc.

    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DistantLightCfg(intensity=2000.0),
    )

    robot: ArticulationCfg = CENTIPEDE_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    # ── Foot contact sensor (Stage B-v0, default ON since Pass 2) ─────────
    #
    # Default value is built by `make_centipede_contact_sensor()` (factory
    # defined above this class). The verified prim_path resolves to
    # /World/envs/env_.*/Robot/link_body_0/foot_.* — the MJCF -> USD
    # converter flattens all rigid bodies (link_body_*, link_leg_*,
    # foot_*) as direct children of the floating root link_body_0
    # (verified 2026-05-07 via inspect_foot_prim_paths.py: 38 / 38
    # feet match this template at depth 3).
    #
    # InteractiveScene auto-instantiates this sensor; the env reads
    # `env.scene["contact_sensor"].data.net_forces_w` for the 86-D
    # observation's contact-load / distribution / derivative blocks.
    #
    # To opt OUT (e.g. for an action-wiring smoke that doesn't need
    # sensor overhead), set `cfg.scene.contact_sensor = None` BEFORE
    # constructing CentipedeEnv. InteractiveScene explicitly skips
    # None cfg values (interactive_scene._add_entities_from_cfg
    # line 726). The 86-D observation will degrade to zeros for the
    # contact-related slots in that case.
    contact_sensor: ContactSensorCfg | None = make_centipede_contact_sensor()

    # GO_FLING_CAUSE_DEFINITIVE_2026-06-29 §2 (deeper fix): OPTIONAL all-body contact sensor. Default None ->
    # InteractiveScene skips it (line 726) -> byte-identical. The analysis cell sets it to
    # make_allbody_contact_sensor() before constructing the env (under --dump-substep-impulse) so the substep
    # impulse accounting captures the NON-FOOT (body-segment) contacts that carry 17-42 % of the weight on rough
    # terrain. Separate from `contact_sensor` (the foot sensor the 86-D obs reads) -> obs UNCHANGED. Passive reader.
    contact_sensor_all: ContactSensorCfg | None = None


# =============================================================================
# Env cfg
# =============================================================================

@configclass
class CentipedeEnvCfg(DirectRLEnvCfg):
    """Top-level config for the Centipede Layer 2 RL env.

    Stage A (this commit): plumbing only. Zero actions, placeholder
    observations and rewards. Once the smoke test passes, Stage B fills
    in the real Layer 2 action / obs / reward / termination logic.
    """

    # Physics dt and decimation. 4 ms / decimation 12 (48 ms control period).
    # History: Stage A v5 Part 0 (2026-06-20) tried dt 0.001 (1 ms, decim 48)
    # for MuJoCo-closer fidelity, but the PI REVERTED to 4 ms on 2026-06-21
    # (GO_REVERT_4MS_RERUN_BENCH): 1 ms gave no benefit for the robust v5
    # candidates (flat & 5 Hz are dt-identical; only marginal rough cases
    # differ) and cost ~4x wall-clock — not worth it for the bench or Stage-E
    # RL. The candidate selection was re-validated as 4ms↔1ms consistent
    # (charter §11a gate). Snapshots: .before_dt1ms_2026-06-20 (the 1ms edit),
    # .before_revert4ms_2026-06-21 (this revert). 4 ms is the working setting;
    # keep a single 1 ms spot-check on the final locked controller only.
    decimation: int = 12
    episode_length_s: float = 8.0

    sim: SimulationCfg = SimulationCfg(
        dt=0.004,                 # 4 ms physics step (REVERTED from 1ms 2026-06-21; see below)
        render_interval=12,       # match decimation: 1 render per policy step
    )

    # Viewport camera for video capture / GUI viewer.
    # 2026-05-06: override DirectRLEnvCfg's default eye=(7.5, 7.5, 7.5)
    # which is sized for ~1m-class robots and produced black smoke MP4s
    # on Isaac Sim 5.0 (cm-scale centipede was a few pixels at 13 m).
    # Diagnostic confirmation: scripts/sanity/check_rendering.py with
    # explicit camera at eye=(0.30, 0.30, 0.18), lookat=(0, 0, 0.025)
    # produced PNG with mean=237, std=12.86, 100% non-zero pixels.
    # eye=(2.5, 2.5, 1.8) is a 3/4 elevated framing for a 16-env
    # 4x4 grid at 1.5 m spacing (~4.5 m corner-to-corner).
    viewer: ViewerCfg = ViewerCfg(
        eye=(2.5, 2.5, 1.8),
        lookat=(0.0, 0.0, 0.05),
        origin_type="world",
        resolution=(1280, 720),
    )

    scene: CentipedeSceneCfg = CentipedeSceneCfg()

    # ── Stage 0.1 model rebuild (2026-06-17): default-OFF asset override ──
    # When None (DEFAULT) the deployed assets/centipede.usd is spawned →
    # deployed behavior is BYTE-IDENTICAL (this field is never read on the
    # default path). When set to a USD path, resolve_asset_override() points
    # the robot articulation at that USD instead. This is the minimal form of
    # the P2 `asset_variant` selector (GO_STAGE_0_1_MODEL_REBUILD directive):
    # P2 reuses this same field + method rather than building a 2nd mechanism.
    #
    # The override is applied via the explicit resolve_asset_override() method,
    # NOT __post_init__ — an automatic bump in __post_init__ conflicts with
    # @configclass's machinery and hangs env.reset (see the observation_space
    # note further down). Callers set the field then call the method BEFORE
    # gym.make / CentipedeEnv(cfg=...).
    usd_path_override: str | None = None

    # ── Stage B-v0 Pass 3 (2026-05-07): terrain selector ──
    # These fields live on the ENV cfg (not the scene cfg) because
    # InteractiveScene._add_entities_from_cfg iterates scene-cfg fields
    # and tries to spawn each as an asset; plain-typed fields would
    # raise "Unknown asset config type". CentipedeEnv reads them via
    # self.cfg.<name>.
    #
    # terrain_kind:
    #   "flat"       (default) → use the GroundPlaneCfg in scene.ground
    #   "transition" → bake a 4-block flat → rough λ=18 mm heightmap
    #                   under /World/TransitionTerrain at scene setup;
    #                   randomized per-env block-spawn on reset.
    terrain_kind: str = "flat"

    # ── Transition-mode parameters (terrain_kind == 'transition') ──
    # 6-block flat → rough heightmap with a WAVELENGTH GRADIENT, blended
    # via cubic smoothstep partition-of-unity weights (C1 continuous —
    # NO slope kink at block centers; see arena.generate_arena_heightmap
    # weight_kind='smoothstep').
    #
    # Pass 9 (2026-05-08): arena bumped 1.8 → 2.2 m and spawn switched
    # from "random block + ±30 mm y jitter" (which packed every robot
    # into a y ≈ 0 strip) to a STRATIFIED 6 × 4 spawn lattice (block ×
    # y-stratum) with small jitter inside each cell. The arena enlarge
    # gives 100 mm more buffer on each side and pushes block centers
    # slightly outward; the y-stratification spreads the population
    # across the full y range instead of concentrating it on a thin
    # central strip.
    #
    # Block layout for arena_size_m=2.2 m, n_blocks=(6,1) — block centres
    # at i * (arena/6) + (arena/12) - arena/2 for i=0..5:
    #     block 0  x ≈ -0.917 m   λ=18 mm   amp=10 mm    rough_a
    #     block 1  x ≈ -0.550 m   λ=18 mm   amp=10 mm    rough_b (different seed)
    #     block 2  x ≈ -0.183 m   λ=30 mm   amp=10 mm    medium
    #     block 3  x ≈ +0.183 m   λ=50 mm   amp= 8 mm    intermediate
    #     block 4  x ≈ +0.550 m   λ=60 mm   amp= 6 mm    gentle long-wave
    #     block 5  x ≈ +0.917 m    -        amp= 0 mm    flat (h ≡ 0)
    transition_arena_size_m:   float = 2.2     # Pass 9: 1.8 → 2.2
    transition_rough_wavelength_mm: float = 18.0
    transition_rough_amplitude_m:   float = 0.010
    transition_seed:           int   = 42
    transition_image_size:     int   = 1024

    # 2026-05-19 (training-arena-param refactor): per-block (λ, amp) tuples
    # now live as first-class env_cfg fields. Defaults preserve the prior
    # behaviour bit-identical (these were previously the function defaults
    # in scripts/optimization/arena.py:generate_flat_to_rough_transition_
    # heightmap). When sweep_consistent_v1 training is requested via
    # train_stage_b_smoke.py CLI overrides, the train script writes the
    # new tuples here BEFORE the env is constructed; envs/centipede_env.py
    # reads them in _build_transition_terrain. NO arena.py edit required:
    # arena.generate_flat_to_rough_transition_heightmap already accepts the
    # transition_wavelengths_mm / transition_amplitudes_m kwargs.
    # transition_block_severity_S mirrors the per-spawn-block severity LUT
    # previously hardcoded at envs/centipede_env.py:_BLOCK_SEVERITY_S. The
    # 6-element tuple is (5 rough blocks + 1 auto-appended flat); when the
    # rough tuples are overridden the severity tuple MUST be overridden in
    # lockstep to keep per-block reward / step1b / audit labels consistent.
    transition_wavelengths_mm:    tuple = (18.0, 18.0, 30.0, 50.0, 60.0)
    transition_amplitudes_m:      tuple = (0.010, 0.010, 0.010, 0.008, 0.006)
    transition_block_severity_S:  tuple = (10.0/18.0, 10.0/18.0, 10.0/30.0,
                                            8.0/50.0,  6.0/60.0,  0.0)

    # ── Stratified spawn mode (Pass 9, 2026-05-08) ──
    # On each reset, every env independently samples
    #   (block_idx, y_stratum_idx) ~ Uniform({0..5} × {0..n_y_strata-1})
    # giving 6 * n_y_strata distinct spawn cells. Within each cell, a
    # small jitter is added so reset-to-reset placements aren't pixel-
    # identical. This:
    #   - preserves per-reset terrain randomization (policy still sees
    #     every block),
    #   - spreads the population across the full y range (not just y≈0),
    #   - keeps spawn margin from the arena edge,
    #   - leaves room for the centipede's natural ~17-30 mm/s walk
    #     during a 6-8 s episode without leaving the safe zone.
    #
    # Block IDs are stored on env._spawn_block_id[E] for diagnostics
    # ONLY; they are NEVER fed into the policy observation.
    transition_block_centers_x: tuple = (-0.917, -0.550, -0.183,
                                          +0.183, +0.550, +0.917)
    transition_block_labels:    tuple = ("rough_a", "rough_b", "medium",
                                          "intermediate", "long_wave", "flat")
    transition_spawn_jitter_x:  float = 0.025   # ±25 mm around block centre
    transition_spawn_jitter_y:  float = 0.100   # Pass 9: 30 → 100 mm jitter
                                                 # WITHIN each y-stratum.
    transition_spawn_clearance_m: float = 0.025
    # Default OFF: preserve the historical point-sample/raw-height spawn path.
    # When enabled, spawn above the highest collision-mesh point under the robot.
    transition_spawn_use_collision_patch: bool = False
    transition_spawn_footprint_radius_m: float = 0.075
    transition_spawn_margin_m:    float = 0.10   # 100 mm safe-zone margin
                                                 # from arena edge.
                                                 # arena 2.2 m → safe zone
                                                 # is ±1.0 m. Leftmost
                                                 # block centre -0.917 m
                                                 # sits 83 mm inside, so
                                                 # ±25 mm x-jitter never
                                                 # clamps.
    transition_spawn_seed:     int   = 1234     # separate from terrain seed

    # Pass 9 stratified-spawn lattice. n_y_strata=4 with default centres
    # at ±0.6, ±0.2 m gives 24 distinct (block × y-strat) cells. With
    # within-strata y-jitter ±0.1 m, adjacent strata don't visually
    # overlap (centre gap 0.4 m ≫ 2 × 0.1 m). y_strata_centres are
    # spaced symmetrically and contained within the safe zone (±1.0 m),
    # so jitter never has to clamp.
    transition_spawn_n_y_strata:        int   = 4
    transition_spawn_y_strata_centres:  tuple = (-0.6, -0.2, +0.2, +0.6)

    # ── Action mode (Stage B-v0, 2026-05-07) ──
    # action_mode selects how raw policy actions are mapped onto the
    # torch controller's runtime modulation buffers:
    #
    #   "global_6d"  (default for Stage B-v0) — six low-dimensional
    #     CPG/impedance knobs:
    #       a0 body amp, a1 leg yaw amp, a2 leg pitch amp,
    #       a3 wave number, a4 body imp (kp+kv), a5 freq.
    #     action_space MUST be 6 in this mode.
    #
    #   "body_21d"  (Stage B Step 1 legacy) — preserves the
    #     19 per-joint body amp scales + wave_number + body_kp scale
    #     mapping. Useful for ablation tests against global_6d. The
    #     existing scripts/sanity/check_stage_b_action_wiring.py runs
    #     this mode explicitly. action_space MUST be 21 in this mode.
    #
    # action_space MUST be set consistently with action_mode by the
    # caller; CentipedeEnv.__init__ asserts it.
    action_mode: str = "global_6d"
    action_space: int = 6

    # Stage B-v0 (Pass 2, 2026-05-07): 86-D contact-rich observation.
    # Layout (slices owned by CentipedeEnv._get_observations):
    #     0:3    root_lin_vel_b               (3)
    #     3:6    root_ang_vel_b               (3)
    #     6:9    projected_gravity_b          (3)
    #     9:13   compressed body yaw q, 4 bins (4)
    #    13:17   compressed body yaw qdot, 4 bins (4)
    #    17:36   per-segment contact load     (19)
    #    36:55   contact distribution features (19)
    #    55:74   contact temporal derivative  (19)
    #    74:80   previous global action       (6)
    #    80:82   global CPG phase sin/cos     (2)
    #    82:86   global contact summaries     (4)
    # Total = 86.
    observation_space: int = 86
    state_space: int = 0

    # ── Pass 10 / Pass 11 enable flags (2026-05-09) ──
    # Both default False so the canonical Pass 8 reward is restored
    # whenever a task config / training script does not opt in. This
    # preserves backward compatibility with the Pass 8 best smoke
    # checkpoint (2026-05-08_15-27-26): replaying it under default
    # cfg gives the same reward stream it was trained on.
    #
    # pass10_effort_enabled:
    #   When True, adds the per-step mechanical-power penalty
    #     r_effort = − W_EFFORT · sigmoid((v_forward−V_MID)/V_SIGMA)
    #                · Σ_actuated |τ·q̇|
    #   that was used in Pass 10 v1 / v2 smoke training.
    #   Kept available for ablation comparisons; v1/v2 results stand
    #   on disk under outputs/rl_smoke/stage_b_transition_global6d_cot/
    #   and …_v2/ as failed-ablation evidence.
    #
    # pass11_cot_enabled:
    #   When True, adds the windowed-CoT hinge penalty
    #     r_cot = − W_COT · gate_x · max(0, CoT_window − α·CoT_zero)
    #   on top of the existing 11-term Pass 8 reward. Designed for
    #   Stage 2 fine-tuning warm-started from the Pass 8 best smoke
    #   checkpoint. Calibrated parameters (H=42, α=1.5, W_COT=8.47e-6)
    #   are class constants in CentipedeEnv (envs/centipede_env.py).
    #
    # The two flags are independent: setting both True simultaneously
    # is supported but not the recommended mode.
    pass10_effort_enabled: bool = False
    pass11_cot_enabled:    bool = False
    # Route-A dose (2026-06-07): CLI-tunable effort weight. Default == legacy
    # _REWARD_W_EFFORT (0.018) so an unscaled run is bit-identical; only matters
    # when pass10_effort_enabled is True (else effort_term ≡ 0 → r_effort ≡ 0).
    reward_w_effort: float = 0.018

    # ── Pass 12 enable flags (2026-05-09) ──
    # All default False so existing Pass 8 / Pass 11 behavior is bit-preserved.
    # Each flag is independent and may be combined.
    #
    # pass12_speed_floor_enabled:
    #     adds   r_speed_floor = - W_V_FLOOR * max(0, V_FLOOR - v_forward)
    #     prevents lowering CoT_window by simply moving slower.
    #
    # pass12_structural_guard_enabled:
    #     adds   r_struct = - W_STRUCT * (
    #               max(0, BODY_AMP_MIN - body_amp_scale)^2
    #             + max(0, FREQ_MIN     - freq_scale)^2
    #             + max(0, LEG_PITCH_AMP_MIN - leg_pitch_amp_scale)^2 )
    #     resists CPG-structure collapse seen in kl004 FINAL drift
    #     (leg_pitch_amp drifted 0.90 → 0.70).
    #
    # pass12_ang_split_enabled:
    #     REPLACES r_ang = -W_ANG * ||omega_b||^2 with
    #              r_ang_split = -W_ANG_RP * (omega_x^2 + omega_y^2)
    #                            -W_ANG_YAW * omega_z^2
    #     to avoid penalizing centipede body-wave yaw as instability.
    #     When True, original r_ang term is zeroed (single source of truth).
    #
    # pass12_cot_bounded_enabled:
    #     when ALSO pass11_cot_enabled is True, REPLACES the unbounded
    #     hinge max(0, CoT_window - threshold) with
    #         bounded_excess = COT_CAP * tanh(max(0, excess) / COT_SCALE)
    #     and uses W_COT_V2 instead of W_COT. Caps per-env contribution
    #     to stabilize gradient variance over the heavy-tailed CoT
    #     distribution (mean/p50 ratio observed = 7000-8000x in BEST/FINAL).
    #
    # See outputs/analysis/cot_reward_candidate_designs.md for full
    # derivation, calibration, and expected failure modes.
    pass12_speed_floor_enabled:        bool = False
    pass12_structural_guard_enabled:   bool = False
    pass12_ang_split_enabled:          bool = True     # R3 2026-05-14: L11 fix (yaw 1/4, RP 4x)
    pass12_cot_bounded_enabled:        bool = False

    # ── Route-A L2 (2026-06-05): WINDOWED + CLI-tunable speed floor ──
    #   GO_ROUTEA_EMERGENCE_2026-06-05. Default-OFF gated variant of the
    #   legacy per-step speed floor (pass12_speed_floor_enabled). When ON,
    #   drives the one-sided hinge off the TRAILING-WINDOW mean v_fwd
    #   (self._vfwd_last_windowed; requires windowed_vfwd_reward_enabled)
    #   so it prices SUSTAINED stalls, not gait-cycle dips. Threshold +
    #   weight are CLI-tunable; defaults == the legacy constants
    #   (_PASS12_V_FLOOR=0.018, _PASS12_W_V_FLOOR=2.0) so a flag-ON run
    #   with no overrides matches the legacy magnitude. Reads v_fwd ONLY
    #   (no wave/severity/applied_abs) → W1/W2-legal. Flag-OFF ⇒ the new
    #   block is unreachable ⇒ reward bit-identical to pre-edit.
    pass12_speed_floor_windowed_enabled: bool = False
    speed_floor_thresh:                  float = 0.018   # m/s, == _PASS12_V_FLOOR
    speed_floor_weight:                  float = 2.0     # == _PASS12_W_V_FLOOR

    # ── Pass 12 RELAXED variant (2026-05-09 follow-up review) ──
    # Biological rationale: real centipedes on rough terrain can adopt low
    # body wave / soft body strategies. The B2-strict structural guard
    # forced body_amp ≥ 0.6 and freq ≥ 0.85 — the body_amp floor was too
    # restrictive (audit shows low_body_amp at body_amp=0.348 preserves
    # 95% of zero-action speed; low_impedance with body_imp=0.348 is
    # actually FASTER than zero-action). Relaxed variant drops the
    # body_amp guard entirely, keeps a softer leg/freq guard.
    #
    # When pass12_functional_guard_enabled is True:
    #   r_functional = - W_FUNC * (
    #         max(0, LEG_PITCH_MIN_RELAXED - leg_pitch_amp_scale)^2
    #       + max(0, FREQ_MIN_RELAXED      - freq_scale)^2
    #   )
    # No body_amp guard; no body_imp guard. Relies on bounded CoT and
    # the audit's functional classification to tell adaptation from
    # collapse.
    #
    # Mutually-conceptually-exclusive with pass12_structural_guard_enabled
    # (the strict B2 variant). Setting both True is supported but means
    # the strict guard's body_amp/freq/leg_pitch terms are added on top
    # of the relaxed terms — generally not the intended use. Pick one.
    pass12_functional_guard_enabled:   bool = True     # R3 2026-05-14: L10 mitigation (leg_pitch>=0.65, freq>=0.80)

    # ── Pass 13 robustness reward (2026-05-11) ──
    # When True, adds a severity-weighted forward-progress bonus:
    #     r_robust_pass13 = + W_ROBUST_PASS13 * severity_S * v_forward
    # where severity_S is per-env from the terrain block label
    # (amp_mm / λ_mm; flat=0, rough=0.556).
    #
    # Motivation: Phase 2 robustness sweep (2026-05-11) classified as
    # Case A — higher wave_num (=2.0) shows flatter performance curves
    # across terrain severity (retention 0.65–0.70 vs wave=1.0's 0.57,
    # decline slope −14 mm/s/S vs −38). But absolute speed is lower
    # at every severity. To encourage RL to explore the robust regime,
    # this term bonuses any progress on hard terrains without
    # referencing wave_num/body_imp directly.
    #
    # See docs/PHASE_3_ROBUSTNESS_REWARD_DESIGN.md for derivation,
    # safety analysis, and audit plan.
    #
    # Default OFF; flag=False reproduces Pass 8/9 reward exactly.
    pass13_robustness_reward_enabled:  bool = False

    # ── 2026-05-13: low-level CPG fast wave-number retargeting ──
    # Phase-state retargeting (Option B in
    # outputs/analysis/cpg_fast_wavenumber_retargeting_design.md).
    # When enabled, controller._apply_phase_rebase shifts body_phases
    # each step so the chain tracks wave_number_scale_runtime within
    # a rate-limited per-step delta — the high-level scripted
    # severity → applied_abs mapping then reaches the body within an
    # episode instead of being stalled by the ~10 s Kuramoto chain
    # reorganisation time.
    #
    # Default OFF — when False, controller path is identical to the
    # pre-2026-05-13 Pass-8 / Stage-E baseline. The controller's
    # `_rebase_call_count` audit counter must read 0 across any
    # flag-OFF rollout (verified by §6.3 default-OFF parity).
    #
    # max_rate_per_s sets the per-step rebase cap; the empirical
    # default 1.0 is the OUTPUT of the §6.5 rate-limit sweep over
    # {0.25, 0.5, 1.0} on the s01 condition. The §6.5 sweep selects
    # the largest rate that simultaneously satisfies the realisation-
    # lag bound, q_ref jump ≤ 0.50 rad, tau ≤ 1.5× baseline,
    # F_max ≤ 5·W, zero tip_risk, zero no_progress.
    fast_wave_retargeting_enabled:        bool  = False
    fast_wave_retargeting_max_rate_per_s: float = 1.0

    # ── 2026-05-13: FARMS-equivalent CPG optimization campaign ──
    # Default-OFF (all four values = identity behaviour) overrides on
    # the CPG integration. See
    # outputs/analysis/farms_equivalent_cpg_optimization/PHASE0_FORENSIC_PLAN.md
    # for the hypothesis table that motivates each knob.
    #
    # cpg_body_coupling_scale  multiplies cpg_omega in
    #   _integrate_cpg_phases. 1.0 = no change. The FARMS-equivalent
    #   value, given FARMS coupling g = body_amp·omega = 0.2·(−200) =
    #   −40 1/s and IsaacLab default g = 0.4·(−2.5) = −1.0 1/s, is
    #   approximately 40×.
    # cpg_leg_coupling_scale  multiplies cpg_leg_omega in
    #   _integrate_cpg_phases. 1.0 = no change. FARMS-equivalent for
    #   leg DOF 0 is approximately 80× (FARMS g_leg = 0.6·(−200) =
    #   −120 1/s; IsaacLab g_leg = 0.6·(−2.5) = −1.5 1/s).
    # cpg_substeps  number of internal CPG Euler substeps per
    #   controller.step() invocation. 1 = no substepping. Tested
    #   experimentally; PHASE 0 expects null result since per-step
    #   convergence rate is already equivalent to FARMS.
    # cpg_update_mode  "jacobi" (current vectorised behaviour;
    #   simultaneous update from old phase values) or "sequential"
    #   (Gauss-Seidel: head→tail, each segment reads its already-updated
    #   neighbour). FARMS uses sequential; IsaacLab uses Jacobi. Test
    #   in PHASE 5 if PHASE 3 doesn't close the gap.
    cpg_body_coupling_scale:              float = 1.0
    cpg_leg_coupling_scale:               float = 1.0
    cpg_substeps:                         int   = 1
    cpg_update_mode:                      str   = "jacobi"

    # Design A 2026-05-16: fix u_body_imp = u_freq = 1.0 to isolate
    # the wave-number action axis and remove the stiffness confound
    # (per wave_trend_reward_calibration_fixed_stiffness_40_80 PHASE 6).
    # Default OFF preserves all prior behaviour byte-identically.
    designA_fixed_imp_freq_enabled:       bool  = False

    # Design B+ 2026-05-16: narrow body_amp / leg_yaw / leg_pitch
    # to [0.9, 1.1] after Design A override, to remove the residual
    # leg/body-amplitude substitute axes identified in the PHASE 1
    # leg-axis confound audit (corr(u_body_amp, F_p99) = +0.49,
    # corr(u_leg_yaw, F_p99) = +0.55, corr(u_leg_pitch, F_p99) =
    # -0.50 under Design A). Default OFF preserves prior behaviour.
    # Intended to be combined with designA_fixed_imp_freq_enabled.
    designBplus_narrow_legs_enabled:      bool  = False
    designBplus_narrow_lo:                float = 0.9
    designBplus_narrow_hi:                float = 1.1

    # Design C 2026-05-17: fix ALL non-wave action axes exactly at 1.0
    # (body_amp, leg_yaw, leg_pitch) AND inherit Design A's pin of
    # body_imp + freq at 1.0. Only u_wave_num remains free. Triggered
    # by overnight PHASE 1 residual-axis audit which found that even
    # under B+'s 3-7x std reduction, u_leg_pitch micro-perturbations
    # remained more correlated with F_p99 than u_wave_num itself
    # (|corr(u_leg_pitch, F_p99)| = 0.451 > |corr(u_wave, F_p99)| =
    # 0.354 at B+ 10-iter seed 42). Default OFF preserves prior
    # behaviour byte-identically. Intended to be combined with
    # designA_fixed_imp_freq_enabled.
    designC_fix_all_non_wave_enabled:     bool  = False

    # ── r_act dead-channel exclusion (2026-07-16, PI-ordered "default off做好备注，留痕") ────────
    # When True, the action-magnitude reward term (r_act = -W_ACT * ||a||^2) STOPS summing the
    # STRUCTURALLY-DEAD action channel: ch-4 (body_imp_scale) under action_mode "global_6d",
    # ch-20 (body_kp_scale) under legacy "body_21d". See _S2_DEAD_ACTION_NOTE at the top of
    # envs/centipede_env.py for why those channels are dead (implicit-only unification, S1/S2a/S2b).
    #
    # WHY THIS FLAG EXISTS: Design A pins the DECODED scale to 1.0 but does NOT zero the RAW action,
    # and act_term sums the RAW action -> from 2026-07-16 onward PPO carries a real (small,
    # W_ACT = 0.002) gradient on an axis that can no longer reach physics.
    # DEFAULT OFF = bit-identical to every pre-flag run (the OFF branch is the original expression,
    # unchanged). Flipping it ON changes r_total AND the `raw/action_norm_sq` log key -> it is a NEW
    # experiment, not a bug fix; it must not be mixed into a comparison against pre-flag runs.
    exclude_dead_action_channels_from_r_act: bool = False

    # P4 interface-smoothing (2026-06-01): when True, the WAVE action channel
    # (idx 3) is re-smoothed in the env with a slower EMA (tau=1.0 s) + a
    # 0.5/s per-step rate limiter (applied_abs units), mirroring the eval-time
    # WaveSmoother probe that unmasked a terrain-conditioned wave response.
    # The five non-wave channels keep the tau=0.30 EMA. Default OFF preserves
    # all prior behaviour byte-identically.
    wave_smooth_interface_enabled:        bool  = False

    # -- Stage E (2026-07-12, PI-directed): narrow the wave-number action HI bound so
    # the RL action maps applied_abs into [1.5, override*3.0] instead of the full
    # [1.5,7.5]. Sentinel <= 0 -> use the module global _G6_WAVE_NUM_HI (2.5) =>
    # byte-identical. Set 1.16667 -> applied_abs max = 3.5 (a004-useful range).
    g6_wave_num_hi_override:               float = -1.0

    # ── Pass 14 (2026-05-17): obs-v2 windowed terrain-severity features ──
    # When `pass14_obs_v2_enabled` is True, the policy observation grows
    # 86 → 90 by appending 4 windowed severity features:
    #     86: acc_fwd_rms_w
    #     87: F_rate_p99_w
    #     88: F_max_p95_w
    #     89: contact_entropy_std_w
    # Computed in `_get_observations` from a per-env ring-buffer of length
    # `pass14_obs_v2_window_steps` policy steps (default 30 = 1.44 s at
    # dt_pol=0.048 s; matches the scripted_v3 R_raw window in
    # scripts/sanity/run_scripted_controller_v3.py:283-358).
    # Z-scored using transition-arena pooled stats from
    #   outputs/analysis/extended_sensor_obs_packet_prep_2026-05-17/
    #   data/T5_final_normalization.csv
    # and clipped to ±3σ. The fifth scripted-controller feature
    # `acc_z_rms_w` is NOT appended to obs (cross-seed corr was unstable
    # in extended validation), but IS logged via extras["log"] as a
    # diagnostic. Reconstructed R_smooth is NOT appended either; PPO
    # learns its own weighting from the 4 raw features.
    # Default OFF preserves the prior 86-D observation byte-identically.
    pass14_obs_v2_enabled:        bool = False

    # --- Stage E (2026-07-12): NEW windowed-terrain observation (WTO) ---
    # When `new_windowed_obs` is True the policy obs becomes the ~51-D self-state +
    # WINDOWED-terrain vector that REPLACES the 86-D obs; the caller MUST also set
    # `observation_space = 51` (or 51 + oracle_terrain_obs_dim when the oracle is ON).
    # `windowed_obs_window_seconds` is the FIXED wall-clock sensing window for the
    # per-substep ring (DECOUPLED from decimation): at decim 250 it equals the 1 s
    # policy interval, but the knob lets us hold the window fixed while sweeping the
    # update rate (isolate window-vs-rate). Default OFF -> obs byte-identical to 86-D.
    new_windowed_obs:             bool  = False
    windowed_obs_window_seconds:  float = 1.0
    # Observation redesign (2026-07-28), default OFF.  Replaces both the
    # legacy 86-D vector and Stage-E 51-D WTO with a 34-D legal vector:
    # applied wave/CPG (3), root linear/angular velocity + gravity (9),
    # a 10-D spatial-spectrum compression of the existing contact window,
    # and a 12-D instantaneous body-frame spine-shape spectrum.  The final
    # two blocks contain no terrain ID, world position, lambda, amplitude,
    # severity, or lookup-table value.  Caller must set observation_space=34.
    # Mutually exclusive with new_windowed_obs; both share only the fixed
    # wall-clock contact ring implementation.
    topology_obs_v2_enabled:      bool  = False
    # DO_NOT_SHIP diagnostic (CLAUDE.md forbidden-to-ship): when True the ground-truth
    # terrain descriptor (normalized lambda, amp, a004-optimal wave#) is APPENDED to the
    # obs as a DIAGNOSTIC UPPER BOUND. NEVER shipped, NEVER warm-started from. Caller
    # sets it per-env via set_oracle_terrain_descriptor and bumps observation_space by
    # oracle_terrain_obs_dim.
    oracle_terrain_obs:           bool  = False
    oracle_terrain_obs_dim:       int   = 3
    pass14_obs_v2_window_steps:   int  = 30
    # NOTE on observation_space: when pass14_obs_v2_enabled is True, the
    # caller MUST also set `env_cfg.observation_space = 90` before
    # passing env_cfg to gym.make. This is intentional: an automatic
    # __post_init__ bump conflicted with @configclass's own machinery
    # and caused env.reset to hang. The caller pattern is documented in
    # `scripts/sanity/check_pass14_obs_v2.py` and (when implemented) the
    # train script: set the flag, then explicitly bump observation_space.

    # ── Pass 15 (2026-05-18): reward recalibration after wave-scale audit ──
    # Two independent default-OFF reward terms approved by ChatGPT for the
    # wave-scale alignment audit follow-up. Both target the Pass 14 30iter
    # observation that the policy correctly raised applied_abs on rough
    # terrain (3.15 -> 3.74 mean) but pushed past the within-policy
    # v_forward optimum (~3.5-3.9) and into mild crawl on rough_a
    # (v_forward 0.0130 vs baseline 0.0165 m/s).
    #
    # Term A — v_forward floor:
    #     r_v_floor = - pass15_w_v_floor * max(0, pass15_v_target_mps - v_forward) ^ 2
    # Quadratic hinge: zero penalty above the floor; grows quadratically
    # below. Calibrated so worst-case full-stop penalty is comparable to
    # baseline step_reward magnitude (~0.008-0.030).
    #
    # Term B — applied_abs soft cap:
    #     applied_abs = wave_number_scale_runtime * pass15_baseline_n_wave
    #     r_wave_softcap = - pass15_w_softcap *
    #         max(0, applied_abs - pass15_applied_abs_softcap_threshold) ^ 2
    # Threshold 4.0 (NOT 3.5): offline within-block bins show v_forward
    # peaks at applied_abs ~3.5-3.9 on rough/medium; a cap at 3.5 would
    # push the policy away from its optimum. ChatGPT-approved threshold.
    #
    # Both terms are identically 0 when their flag is False; reward is
    # byte-identical to pre-Pass-15 behavior when both flags are False.
    # Audit evidence:
    #   outputs/analysis/pass15_reward_recalibration_packet_2026-05-18/
    pass15_v_fwd_floor_enabled:               bool  = False
    pass15_wave_softcap_enabled:              bool  = False
    pass15_v_target_mps:                      float = 0.015
    pass15_w_v_floor:                         float = 50.0
    pass15_applied_abs_softcap_threshold:     float = 4.0
    pass15_w_softcap:                         float = 0.1
    pass15_baseline_n_wave:                   float = 3.0

    # ── Pass 16 (2026-05-19) — overnight_adaptive_wave_campaign ─────────
    # Two independent default-OFF FUNCTIONAL-OUTCOME reward terms.
    # NEITHER depends on terrain ground truth, spawn pose, block_id, true
    # severity, or LUT command. Both are pure window-aggregate penalties
    # on observed body mechanics.
    #
    # Term A — attitude-variance penalty:
    #   attitude_var_w = var_w(roll) + var_w(pitch)   (1.44 s window @ 30 steps)
    #   r_attitude_var = - pass16_attitude_var_weight * attitude_var_w
    # Calibrated so typical baseline contribution is ~-0.005 to -0.020 per step
    # (comparable in magnitude to pass15 floor/cap).
    #
    # Term B — per-link force-peak penalty:
    #   F_max_per_link_p99_w = p99 over [t-W+1, t] of F_max-across-links
    #   r_force_peak = - pass16_force_peak_weight *
    #       max(0, F_max_per_link_p99_w - pass16_force_safe_N) ^ 2
    # Hinge at body-weight scale (BW ≈ 0.049 N for m=0.005 kg). Fires only on
    # actual force spikes; typical baseline contribution near 0 in flat blocks,
    # up to ~-0.02 on rough blocks.
    #
    # Both terms are identically 0 when their flag is False; reward is
    # byte-identical to pre-Pass-16 behavior when both flags are False.
    # Stacking order: Pass 8 baseline + Pass 10/11/12/13/14/15 + Pass 16.
    # Audit evidence:
    #   outputs/analysis/overnight_adaptive_wave_campaign_2026-05-19/
    # ── Pass 17 (2026-05-19): K=12 sensor → severity estimator + severity-weighted speed reward ──
    # K=12 ridge model (from outputs/analysis/overnight_estimator_pass16_forensic_2026-05-19/
    # data/k12_deploy.json) is computed each policy step in env.py over a
    # W=20 step (~0.96 s) sliding window of 12 sensor-derived features.
    # NO terrain labels (severity_S, block_id, x, amp, λ, LUT target) are
    # used as inputs at runtime — only contact-sensor force, contact state,
    # per-segment body link quat, root angular velocity. The estimator's
    # OFFLINE training target was severity_S but at deploy time only sensors
    # are read; coefs + intercept + z-score (mu, sigma) are hard-coded in
    # envs/centipede_env.py as _K12_FEATURE_* constants.
    #
    # When pass17_k12_estimator_enabled is True, env writes:
    #   raw/R_belief        ∈ [0, 1]  (smoothstep-bounded)
    #   raw/R_belief_raw    raw ridge output (unbounded scalar)
    # No reward impact unless pass17_severity_weighted_speed_enabled is True.
    #
    # When pass17_severity_weighted_speed_enabled is True:
    #   r_25_severity_weighted_speed = + w_A * v_forward * R_belief
    # which rewards forward progress MORE on rough terrain (R_belief high).
    # Default OFF; ≡ 0 when flag False. Bio-prior derived (per packet §3).
    pass17_k12_estimator_enabled:             bool  = False
    pass17_severity_weighted_speed_enabled:   bool  = False
    pass17_severity_weighted_speed_weight:    float = 0.5    # w_A, packet default

    # ── P4 v3.4 Round A (2026-05-31): PGZ alternative belief (sensor-only) ──
    # When pass17_projgz_estimator_enabled is True, R_belief is computed from
    # ONLINE-SENSED proj_g_z ONLY (no terrain ground-truth) via a documented
    # monotone OLS+smoothstep transform (constants fitted offline against spawn-
    # block severity; proj_g_z LOSO r=+0.4153 > K=12 +0.193). It REPLACES the
    # K=12 belief as the source for r_25 (same pass17_severity_weighted_speed_weight
    # ⇒ controlled A/B). MUTUALLY EXCLUSIVE with pass17_k12_estimator_enabled
    # (asserted in env.__init__). Default OFF ⇒ reward graph byte-identical to
    # baseline. pass18 stays OFF in Round A.
    pass17_projgz_estimator_enabled:          bool  = False

    # ── P4 v3.4 Round B (2026-05-31): belief-conditioned attitude-var robustness ──
    # When pass20_belief_robustness_enabled is True, adds
    #   r_20 = -0.125 * (1 + 0.5*R_belief) * attitude_var_w * gate
    # reusing the pass16 windowed attitude variance + the K=12 sensor belief. The
    # REWARD NEVER MENTIONS WAVE — a strict-emergent test of whether the policy
    # raises body wave to stay stable on SENSED-rough terrain. Belief LOCKED to
    # K=12 (PGZ is circular: proj_g_z is itself an attitude signal). When on, it
    # auto-computes the K=12 R_belief + the attitude-var window. Default OFF ⇒
    # reward graph byte-identical to baseline. Sensor-only (no terrain ground-truth).
    pass20_belief_robustness_enabled:         bool  = False

    # ── Pass 18 (2026-05-19): K=12 imitation target reward ──
    # When pass18_k12_imitation_enabled is True (requires pass17_k12_estimator_enabled):
    #   applied_abs_chosen = wave_number_scale_runtime * 3.0
    #   applied_abs_target = 1.5 + 2.0 * R_belief                   ∈ [1.5, 3.5]
    #   r_26_k12_imitation = - w_B * (applied_abs_chosen - applied_abs_target)^2
    #
    # MUTUALLY EXCLUSIVE with pass17_severity_weighted_speed_enabled
    # (assertion in env.__init__). w_B default is 0.02 per auditor 2026-05-19
    # calibration: at max deviation 2.0, per-step penalty = 0.08, comparable
    # to existing Pass 15 wave_softcap penalty magnitude (~0.04). Default OFF.
    pass18_k12_imitation_enabled:             bool  = False
    pass18_k12_imitation_weight:              float = 0.02   # w_B, auditor-calibrated

    pass16_attitude_var_enabled:              bool  = False
    pass16_attitude_window_steps:             int   = 30
    pass16_attitude_var_weight:               float = 0.5
    pass16_force_peak_enabled:                bool  = False
    pass16_force_window_steps:                int   = 30
    pass16_force_peak_weight:                 float = 5.0
    pass16_force_safe_N:                      float = 0.05

    # ── Pass 19 (2026-05-21): observation enrichment ──
    # Two mutually-exclusive paths for adding per-link / per-segment
    # sensor channels. Both default OFF; flag-OFF preserves byte-identity
    # with the pre-edit obs/reward chain.
    #
    # pass19_extended_trace_enabled:
    #     Log-only path (§1.main of obs-enrichment packet v3). When True,
    #     env writes 6 new log keys per policy step into env.extras["log"]:
    #         raw/foot_force_vector   (E, 38, 3)
    #         raw/foot_force_mag      (E, 38)
    #         raw/seg_pitch           (E, 19)
    #         raw/seg_roll            (E, 19)
    #         raw/foot_contact_state  (E, 38) binary
    #         raw/foot_contact_dur    (E, 38) rolling counter
    #     Policy obs unchanged at 86-D. Used for offline K=12 v2 refit.
    #
    # pass19_obs_extension_enabled:
    #     In-obs path (§1.alt of obs-enrichment packet v3). When True,
    #     env appends 155 new dims to the policy obs at slices 86:241:
    #         86:124   per-foot force magnitude        (38)
    #        124:162   per-foot contact state binary   (38)
    #        162:200   per-foot contact duration /20   (38)
    #        200:219   per-segment pitch               (19)
    #        219:238   per-segment roll                (19)
    #        238:241   [proj_g_z, d/dt, d²/dt²] 5-tap  (3)
    #     New obs dim = 86 + 155 = 241. The CALLER must also bump
    #     env_cfg.observation_space = 241 BEFORE env init (same pattern
    #     as pass14_obs_v2_enabled → observation_space = 90).
    #
    # MUTUAL EXCLUSION: at most one of the two flags True at a time
    # (asserted in env.__init__). pass14_obs_v2_enabled is ALSO mutually
    # exclusive with pass19_obs_extension_enabled (same assertion).
    pass19_extended_trace_enabled:            bool  = False
    pass19_obs_extension_enabled:             bool  = False
    pass19_contact_dur_normalize_W:           int   = 20      # divisor for foot_contact_dur in obs

    # p4_pre_runtime_torque_qdot_enabled (2026-05-29, P4-pre β-light):
    #     when True AND pass19_obs_extension_enabled is True, append 3 scalar
    #     channels (joint_power_abs_j04, applied_torque_abs_j29, joint_vel_j30)
    #     to the policy observation at slice 241:244 via runtime computation
    #     from art.data.applied_torque and art.data.joint_vel.
    #
    #     Default False — no observation surface change when off.
    #     Mutually exclusive with pass14_obs_v2_enabled (same as pass19).
    #     Requires pass19_obs_extension_enabled = True (asserted in __init__).
    #     Total obs dim with both pass19 + p4_pre on: 86 + 155 + 3 = 244.
    p4_pre_runtime_torque_qdot_enabled:       bool  = False

    # ── P4 v3.4 Round C — C0 impact-first reward floor (2026-05-31) ──
    # NEW reward structure (impact-first, no belief at C0). All default-OFF ⇒
    # reward graph byte-identical to baseline. When (roundC_enabled AND
    # rC0_impact_enabled): gate the always-on legacy r_fmax (C09-A) to 0 and add
    #   rC0_impact = -rC0_impact_weight · force_raw_Fmax_excess · gate_progress_valid
    # where force_raw_Fmax = MEAN F_max over a W-step window (PI decision B2:
    # windowed MEAN, NOT max/peak — matches the sweeps' per-cell mean benefit and
    # avoids the per-step-peak pathology that drove the policy to lower wave).
    # Sensor-only (s["F_max"]/v_forward/proj_g_z); NEVER rewards wave/applied_abs.
    roundC_enabled:                           bool  = False   # master flag
    rC0_impact_enabled:                       bool  = False   # C0 impact term
    roundc_zero_wave_anchor:                  bool  = False   # C1 use only; declared now
    rC0_impact_weight:                        float = 0.50    # PI-signed decision #5
    rC0_fmax_window_steps:                    int   = 30      # W; ~1.44s at dt_pol 0.048
    rC0_V_CRAWL:                              float = 0.008   # m/s anti-crawl floor (forward = -vx)
    rC0_TIP_PROJ_GZ:                          float = -0.5    # proj_g_z upright threshold (more upright than -0.259 fail)

    # ── P4 v3.4 Round C — C1 belief-conditioned impact (THE emergence test) ──
    # When (roundC_enabled AND rC1_belief_impact_enabled): the C0 impact term is
    # multiplied by belief_weight = (1 + rC1_kappa * R_belief), R_belief = K12
    # SENSOR belief (sensor-only, no ground-truth). Reward STILL never mentions
    # wave — tests whether the policy DISCOVERS higher wave (which lowers impact on
    # rough) to satisfy the belief-amplified impact penalty. Requires
    # rC0_impact_enabled (C1 reuses C0's force buffer/excess/gate). Engage
    # roundc_zero_wave_anchor at C1 to neutralize the legacy r_wave 3.0-anchor
    # (emergence-contamination, not "fights high wave"). Default OFF ⇒ baseline.
    rC1_belief_impact_enabled:                bool  = False   # C1 emergence test
    rC1_kappa:                                float = 0.5     # belief gain; PI sweep {0.5, 1.0}

    # ── P4 v3.4 trend-suppression Phase 1 (2026-06-02): r_ang ablation lever ──
    # Multiplicative scale on the angular-velocity penalty r_ang (env:2098).
    # DEFAULT 1.0 ⇒ the env SKIPS the multiply (byte-identical to pre-edit).
    # Arms: 1.0 (OFF/baseline), 0.5 (halved), 0.0 (zeroed). In C1 (pass12_ang_split
    # OFF) r_ang is the live W_ANG-scaled penalty; under ang_split ON r_ang is
    # already zeroed so this is a no-op (correct — the lever targets C1's r_ang).
    rang_weight_scale:                        float = 1.0     # 1.0=OFF; 0.5/0.0=ablation arms
    # r_v (forward-velocity reward) multiplicative scale. 1.0=OFF (byte-identical).
    rv_weight_scale:                          float = 1.0     # 1.0=OFF; 2.0=test (do not >=3x first test)
    # r_act (action-norm penalty) multiplicative scale. 1.0=OFF (byte-identical).
    ract_weight_scale:                        float = 1.0     # 1.0=OFF; 0.0=remove action-norm tax
    # ENV-SIDE wave-baseline recenter: action=0 AND reset -> applied_abs=this value
    # (scale=this/pass15_baseline_n_wave). Controller n_wave FROZEN; logging truthful.
    # Default == pass15_baseline_n_wave (3.0) ⇒ byte-identical.
    wave_baseline_abs:                        float = 3.0     # 3.0=OFF; 2.0=test
    # P4 v3.4 windowed-speed reward (2026-06-03, PART-0 W=80 primary / W=40 control). Default OFF
    # (byte-identical). ON: r_v uses a trailing W-step MEAN of v_forward (signed, dense, per-env,
    # cleared on episode reset) so the wave->speed effect the per-step signal cannot resolve
    # (Q2 SNR~0.1-0.6 per-step vs 2-9 windowed) becomes learnable. W is a CLI param.
    windowed_vfwd_reward_enabled:             bool  = False
    vfwd_window_steps:                        int   = 80
    # P4 v3.4 Arm 2 (2026-06-03): stability (low-tilt) reward. Default OFF (byte-identical).
    # ON: subtract stability_tilt_weight * tilt from r_total, where tilt = sqrt(proj_g_x²+proj_g_y²)
    # (body-frame gravity = orientation only; proprioception, NO terrain label). UNIFORM weight —
    # auto-conditions because tilt is ~14× larger on rough (legal severity-by-physics). Crediting
    # stability is meant to flip Arm-1's INVERTED conditioning (rough→low wave) toward the bio-prior
    # (rough→high wave), since higher wave → less tilt (causal).
    stability_tilt_reward_enabled:            bool  = False
    stability_tilt_weight:                    float = 0.2
    # ── SPEED-ONLY reward (2026-06-04, DO_NOT_SHIP oracle/credit probe) ──
    # ON: r_total = r_v + r_fail + r_ori + r_alive ONLY — strip EVERY modulation tax/
    # anchor (r_ang/r_act/r_rate/r_tau/r_zvel/r_imp/r_wave/...) AND all pass-gated
    # shapers, by overriding the composed sum (robust: cannot miss a term). r_v stays
    # windowed when windowed_vfwd_reward_enabled. Safety floor {r_fail,r_ori,r_alive}
    # retained (PI 2026-06-04) — none biases the wave choice. Default OFF => byte-identical.
    reward_speed_only_enabled:                bool  = False
    # ── GO r_jitter BUILD (2026-06-09): CV-type forward-speed-choppiness penalty ──
    # ON: r_total -= jitter_penalty_weight * clip(CV_w, 0, 3.0) where CV_w =
    # std_w / max(mean_w, 0.010) over the SAME trailing window windowed r_v keeps
    # (_vfwd_buf; requires windowed_vfwd_reward_enabled). CV-type (speed-normalized):
    # raw std is gamed by standing still (there is NO stall termination). Reads
    # v_fwd ONLY — no wave/severity/block/x/amp/λ/LUT. Probe-motivated: open-loop
    # choppiness→wave incentive is severity-graded 0.0/1.6/5.5/30.6 pts at amp
    # 0/6/10/16 mm, ZERO on flat (P4_v34_jitter_selfcond_2026-06-09). Added AFTER
    # the speed-only override so it survives the r_total replacement. Default OFF
    # => byte-identical.
    jitter_penalty_enabled:                   bool  = False
    jitter_penalty_weight:                    float = 0.05

    # ── Stage 0.1 model rebuild (2026-06-17): asset-override resolver ──
    def resolve_asset_override(self) -> str:
        """Point the robot articulation at ``usd_path_override`` if set.

        No-op when ``usd_path_override`` is None (the default) → the deployed
        ``assets/centipede.usd`` is spawned and behavior is byte-identical.
        Call this AFTER constructing the cfg and BEFORE gym.make /
        CentipedeEnv(cfg=...). Returns the USD path actually in effect.

        Deliberately a plain method (not __post_init__): an automatic bump in
        __post_init__ conflicts with @configclass machinery and hangs
        env.reset (see the observation_space note above).
        """
        if self.usd_path_override:
            self.scene.robot.spawn.usd_path = self.usd_path_override
            # Rebuild variant (centipede_rebuild_v1): feet are WELDED (no
            # joint_foot_* → the deployed `foot_passive` actuator group's regex
            # matches 0 joints → Isaac Lab raises) and body-roll now exists as
            # 20 single-DOF hinges that should be actuated like yaw/pitch. Drop
            # foot_passive and add joint_roll_body_ to the active group. This is
            # the directive's P2 per-variant actuator wiring, centralized here so
            # every script that selects the variant gets it (no 2nd mechanism).
            if "rebuild" in self.usd_path_override.lower():
                self.scene.robot.actuators.pop("foot_passive", None)
                if "renamed" in self.usd_path_override.lower():
                    if "leaner" in self.usd_path_override.lower():
                        # Block A (2026-07-07): LEANER 2-DOF-leg rebuild. legbase_roll +
                        # knee + body_roll are gone. body_pitch stays in the 'active' group
                        # (stiffness=0/damping=0 here) — but since S2a (2026-07-16) that is
                        # only the SUBSTRATE the impedance table writes onto: every family's
                        # (Kp, Kv) now goes straight to PhysX via env.set_joint_impedance(),
                        # so the LOCKED bio spine-pitch k=6e-4 is simply Kp of the
                        # `body_pitch` family (the panel pairs it with Kv=9e-4=1.5k, the c
                        # the suppressed spine-pitch joints used to run). The passive spring,
                        # suppress_active_pitch and the explicit effort_target torque path
                        # they rode on are DELETED — a joint left out of an actuator group
                        # would get no Kp/Kv at all, which is why membership still matters.
                        # See docs/IMPLICIT_ONLY_MIGRATION_2026-07-16.md. tarsus_yaw is a
                        # distal PASSIVE yaw joint given its OWN group preserving the MJCF
                        # spring (stiffness 5e-4, damping 1e-4). Per-model adaptive
                        # (Option B): the 4-DOF renamed branch below is UNCHANGED.
                        self.scene.robot.actuators["active"].joint_names_expr = [
                            "body_yaw_.*", "body_pitch_.*",
                            "legbase_yaw_.*", "legbase_pitch_.*",
                        ]
                        self.scene.robot.actuators["tarsus_passive"] = ImplicitActuatorCfg(
                            joint_names_expr=["tarsus_yaw_.*"],
                            effort_limit=1.0, velocity_limit=100.0,
                            stiffness=5.0e-4, damping=1.0e-4,
                        )
                    else:
                        # P3 function-renamed 4-DOF rebuild: UNCHANGED (back-compat).
                        self.scene.robot.actuators["active"].joint_names_expr = [
                            "body_yaw_.*", "body_pitch_.*", "body_roll_.*",
                            "legbase_.*", "knee_.*",
                        ]
                else:
                    self.scene.robot.actuators["active"].joint_names_expr = [
                        "joint_body_.*", "joint_pitch_body_.*",
                        "joint_roll_body_.*", "joint_leg_.*",
                    ]
        return self.scene.robot.spawn.usd_path
