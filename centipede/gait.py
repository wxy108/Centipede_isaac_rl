"""The frozen 1-D action law and wave-conditioned six-parameter gait map."""

from __future__ import annotations

import json
import math
from pathlib import Path

import torch


class WaveActionLaw:
    """Exact flow of the rate-limited first-order command law at 12 ms."""

    lower = 1.0
    upper = 4.0
    reset_wave = 2.5
    tau_s = 0.25
    rate_per_s = 3.5
    dt_s = 0.012

    @classmethod
    def target(cls, action: float) -> float:
        return 2.5 + 1.5 * min(1.0, max(-1.0, float(action)))

    @classmethod
    def step(cls, previous: float, action: float) -> float:
        target = cls.target(action)
        error = target - float(previous)
        if error == 0.0:
            return float(previous)
        sign = 1.0 if error > 0.0 else -1.0
        magnitude = abs(error)
        release_error = cls.rate_per_s * cls.tau_s
        if magnitude <= release_error:
            increment = magnitude * (1.0 - math.exp(-cls.dt_s / cls.tau_s))
        else:
            saturated_time = (magnitude - release_error) / cls.rate_per_s
            if cls.dt_s <= saturated_time:
                increment = cls.rate_per_s * cls.dt_s
            else:
                increment = magnitude - release_error * math.exp(
                    -(cls.dt_s - saturated_time) / cls.tau_s
                )
        return min(cls.upper, max(cls.lower, float(previous) + sign * increment))


class GaitMap:
    fields = (
        "body_amp", "leg_yaw_amp", "body_yaw_kp",
        "body_yaw_kv", "legbase_yaw_kp", "legbase_yaw_kv",
    )

    def __init__(self, path, device):
        self.path = str(Path(path).resolve())
        doc = json.loads(Path(path).read_text(encoding="utf-8"))
        rows = sorted(doc["wave_map"], key=lambda row: float(row["wave"]))
        waves = [float(row["wave"]) for row in rows]
        expected = [1.0 + 0.25 * i for i in range(13)]
        if waves != expected:
            raise ValueError(f"Gait map must contain the frozen 13 knots, got {waves}")
        missing = [(row["wave"], name) for row in rows for name in self.fields if name not in row]
        if missing:
            raise ValueError(f"Gait map fields are incomplete: {missing[:4]}")
        self.pitch_scale = float(doc["pitch_scale_locked"])
        self.waves = torch.tensor(waves, dtype=torch.float32, device=device)
        self.values = {
            name: torch.tensor(
                [float(row[name]) for row in rows], dtype=torch.float32, device=device
            )
            for name in self.fields
        }

    def at(self, wave: torch.Tensor):
        wave = wave.to(dtype=torch.float32, device=self.waves.device)
        upper = torch.bucketize(wave, self.waves).clamp(1, len(self.waves) - 1)
        lower = upper - 1
        alpha = (wave - self.waves[lower]) / (self.waves[upper] - self.waves[lower])
        result = {}
        for name, values in self.values.items():
            result[name] = values[lower] + alpha * (values[upper] - values[lower])
        result["leg_pitch_amp"] = torch.full_like(wave, self.pitch_scale)
        return result

