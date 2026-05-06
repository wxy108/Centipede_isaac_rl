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

import torch

from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv

from envs.centipede_env_cfg import (
    CentipedeEnvCfg,
    CONTROLLER_CONFIG_PATH,
)
from controllers.impedance_controller_torch import (
    ImpedanceTravelingWaveControllerTorch,
)


class CentipedeEnv(DirectRLEnv):
    """Centipede Layer 2 RL env, Stage A skeleton."""

    cfg: CentipedeEnvCfg

    # =========================================================================
    # Construction & scene
    # =========================================================================

    def __init__(self, cfg: CentipedeEnvCfg, render_mode: str | None = None, **kwargs):
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

        # Action buffer held between policy decimation calls.
        self.actions = torch.zeros(
            (self.num_envs, self.cfg.action_space),
            dtype=torch.float32,
            device=self.device,
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

        # Filter collisions between cloned envs but allow them with the
        # shared ground plane.
        self.scene.filter_collisions(global_prim_paths=["/World/GroundPlane"])

    # =========================================================================
    # Policy → physics
    # =========================================================================

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        """Called once per policy step (every `decimation` physics steps).

        Stage A: cache actions, ignore them. Stage B will map them onto
        controller.body_amp / body_kp / n_wave_scalar.
        """
        self.actions = actions.clone()

    def _apply_action(self) -> None:
        """Called every physics step, between policy decisions.

        Reads articulation state, runs the controller, writes torques.
        Time fed to the controller is the per-env elapsed time since
        the last reset; we advance _sim_time by physics dt each call.
        """
        import time as _time
        dt = self.cfg.sim.dt
        # Host-side time scalar (no GPU sync) — Stage A approximation.
        _t0 = _time.perf_counter()
        tau = self.controller.step(self._t_host, dt)
        self.robot.set_joint_effort_target(tau)
        # Diagnostic accumulator. Drained by the smoke script.
        self._last_controller_dt_total += _time.perf_counter() - _t0

        # Advance both clocks: host-side scalar (controller input) and
        # per-env tensor (episode timeout).
        self._t_host += dt
        self._sim_time = self._sim_time + dt

    # =========================================================================
    # Observation / reward / termination — placeholders for Stage A
    # =========================================================================

    def _get_observations(self) -> dict:
        """Stage A: zeros. Stage B will return root state + body state +
        contact + CPG phases.
        """
        obs = torch.zeros(
            (self.num_envs, self.cfg.observation_space),
            dtype=torch.float32,
            device=self.device,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        """Stage A: zeros. Stage B will compute forward-velocity reward
        with bio-anchored penalties and action-magnitude penalty.
        """
        return torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Stage A: timeout only. Stage B adds fall detection.

        Returns (terminated, truncated). For DirectRLEnv, both are
        boolean tensors of shape (num_envs,).
        """
        truncated = self._sim_time >= self.cfg.episode_length_s
        terminated = torch.zeros_like(truncated)
        return terminated, truncated

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
        self.robot.write_root_pose_to_sim(default_root[:, 0:7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root[:, 7:13], env_ids)

        default_joint_pos = self.robot.data.default_joint_pos[env_ids].clone()
        default_joint_vel = self.robot.data.default_joint_vel[env_ids].clone()
        self.robot.write_joint_state_to_sim(
            default_joint_pos, default_joint_vel, None, env_ids
        )

        # Reset per-env time tensor.
        self._sim_time[env_ids] = 0.0

        # Reset controller history for these envs.
        self.controller.reset_target_history(env_ids)

        # When ALL envs reset together, also reset the host clock and the
        # globally-shared CPG / heading state. Per-env CPG reset is a
        # Stage B refinement.
        if len(env_ids) == self.num_envs:
            self._t_host = 0.0
            self.controller.body_phases = None
            self.controller.leg_phases = None
            self.controller._cpg_initialized = False
            self.controller.head_yaw_ref = None
