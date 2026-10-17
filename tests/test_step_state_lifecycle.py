"""CPU-only regression for CentipedeEnv's per-policy-step failure cache.

The live module imports Isaac Lab, so this test extracts only the two small
methods under test from the source file. It therefore exercises their real
Python bodies without starting AppLauncher, Isaac Sim, PhysX, or CUDA.
"""

from __future__ import annotations

import ast
import textwrap
from pathlib import Path

import torch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ENV_SOURCE = PROJECT_ROOT / "envs" / "centipede_env.py"


def _load_centipede_method(name: str):
    source = ENV_SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(ENV_SOURCE))
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "CentipedeEnv"
    )
    method = next(
        node for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    lines = source.splitlines(keepends=True)
    method_source = textwrap.dedent(
        "".join(lines[method.lineno - 1:method.end_lineno])
    )
    namespace = {"torch": torch}
    exec(compile(method_source, str(ENV_SOURCE), "exec"), namespace)
    return namespace[name]


class _Cfg:
    action_mode = "global_6d"
    episode_length_s = 10.0


class _LifecycleHarness:
    _pre_physics_step = _load_centipede_method("_pre_physics_step")
    _get_dones = _load_centipede_method("_get_dones")

    def __init__(self):
        self.cfg = _Cfg()
        self.actions = torch.zeros((2, 6), dtype=torch.float32)
        self._sim_time = torch.zeros(2, dtype=torch.float32)
        self._dump_substep_impulse = False
        self._step_state = {
            "fail": torch.tensor([True, False], dtype=torch.bool),
        }
        self.next_fail = torch.tensor([False, True], dtype=torch.bool)
        self.compute_calls = 0
        self.state_seen_by_action_dispatch = object()

    def _apply_global_6d_action(self, _actions):
        self.state_seen_by_action_dispatch = self._step_state

    def _apply_body_21d_action(self, _actions):
        raise AssertionError("global_6d fixture dispatched to body_21d")

    def _compute_step_state(self):
        self.compute_calls += 1
        state = {"fail": self.next_fail.clone()}
        self._step_state = state
        return state


def test_pre_physics_step_invalidates_previous_failure_before_done_query():
    env = _LifecycleHarness()
    previous_fail = env._step_state["fail"].clone()

    env._pre_physics_step(torch.ones((2, 6), dtype=torch.float32))

    # Invalidation must precede action dispatch, which is the earliest work in
    # the policy step that can affect the following physics transition.
    assert env.state_seen_by_action_dispatch is None
    assert env._step_state is None

    terminated, truncated = env._get_dones()

    assert env.compute_calls == 1
    assert terminated.tolist() == [False, True]
    assert truncated.tolist() == [False, False]
    assert terminated.tolist() != previous_fail.tolist()

