"""Exact joint and body indexing for the frozen renamed 4-DOF model."""

from __future__ import annotations

import numpy as np


N_BODY_SEGMENTS = 21
N_BODY_YAW = 19
N_BODY_PITCH = 20
N_BODY_ROLL = 20
N_LEG_PAIRS = 19
N_LEG_DOF = 4
BODY_LENGTH_M = 0.1025

LEG_FAMILIES = ("legbase_yaw", "legbase_pitch", "legbase_roll", "knee")


class ModelIndex:
    """Resolve the one supported model by exact names, never regular expressions."""

    def __init__(self, articulation):
        joints = {name: index for index, name in enumerate(articulation.joint_names)}
        bodies = {name: index for index, name in enumerate(articulation.body_names)}

        def joint(name: str) -> int:
            if name not in joints:
                raise ValueError(f"Frozen model is missing joint {name!r}")
            return joints[name]

        def body(name: str) -> int:
            if name not in bodies:
                raise ValueError(f"Frozen model is missing body {name!r}")
            return bodies[name]

        self.body_yaw = np.asarray(
            [joint(f"body_yaw_{i}") for i in range(1, N_BODY_YAW + 1)], dtype=int
        )
        self.body_pitch = np.asarray(
            [joint(f"body_pitch_{i}") for i in range(N_BODY_PITCH)], dtype=int
        )
        self.body_roll = np.asarray(
            [joint(f"body_roll_{i}") for i in range(N_BODY_ROLL)], dtype=int
        )
        self.legs = np.empty((N_LEG_PAIRS, 2, N_LEG_DOF), dtype=int)
        for segment in range(N_LEG_PAIRS):
            for side_index, side in enumerate(("L", "R")):
                for dof, family in enumerate(LEG_FAMILIES):
                    self.legs[segment, side_index, dof] = joint(
                        f"{family}_{segment}_{side}"
                    )

        self.head_body = body("link_body_0")
        self.main_body = np.asarray(
            [body(f"link_body_{i}") for i in range(N_BODY_SEGMENTS)], dtype=int
        )
        self.spine = np.asarray(
            [
                index
                for index, name in enumerate(articulation.body_names)
                if "link" in name.lower()
                and "leg" not in name.lower()
                and "tarsus" not in name.lower()
            ],
            dtype=int,
        )
        if len(self.spine) != 61:
            raise ValueError(f"Frozen metric requires 61 spine links, found {len(self.spine)}")

        active = np.concatenate(
            [self.body_yaw, self.body_pitch, self.body_roll, self.legs.reshape(-1)]
        )
        if len(active) != 211 or len(np.unique(active)) != 211:
            raise ValueError("Frozen model must expose 211 unique active joints")

