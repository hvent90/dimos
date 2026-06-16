# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Speed-scheduled plant-gain inversion for nonlinear twist bases.

The constant-K feedforward (``cmd = desired / K_hat``) assumes a linear
plant. Some bases (the Go2 especially) have a steady-state gain that varies
2-3x across the speed range, so a single K mis-compensates everywhere except
one operating point. This module interpolates K per axis from the
characterization's per-amplitude fits (``dynamics_by_amplitude``) so the
inversion tracks the operating point: ``cmd = desired / K(|desired|)``.

Defensive by design — the per-amplitude fits are noisy (low r^2) and some
collapse near the motion floor, so degenerate points are dropped and the
interpolated gain is floored to bound the inverse.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

_AXES = ("vx", "vy", "wz")
# Floor on the interpolated gain so 1/K can't explode (max ~5x amplification).
_K_FLOOR = 0.2
# Drop characterization points whose gain collapsed (floor / degenerate fit).
_MIN_VALID_K = 0.1
# Fixed-point iterations to solve cmd * K(cmd) = desired. K is indexed by the
# COMMAND amplitude, but we know the DESIRED velocity, so refine the command a
# few times (matters where K varies fast / non-monotonically, e.g. Go2 wz).
_INVERT_ITERS = 4


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


@dataclass(frozen=True)
class AxisGainCurve:
    """Monotone-in-amplitude breakpoints of K(amp) for one axis."""

    amp: tuple[float, ...]
    gain: tuple[float, ...]
    tau: tuple[float, ...]

    def k_at(self, v: float) -> float:
        """Interpolated steady-state gain at command magnitude ``|v|``,
        floored so the inverse stays bounded. np.interp clamps to the end
        breakpoints outside the measured range."""
        return max(_K_FLOOR, float(np.interp(abs(v), self.amp, self.gain)))

    def tau_at(self, v: float) -> float:
        return float(np.interp(abs(v), self.amp, self.tau))

    def invert(self, desired: float) -> float:
        """Command that produces ``desired`` velocity: solve cmd*K(cmd)=desired
        by fixed-point iteration (K is indexed by command amplitude)."""
        cmd = desired
        for _ in range(_INVERT_ITERS):
            cmd = desired / self.k_at(cmd)
        return cmd


@dataclass(frozen=True)
class GainSchedule:
    vx: AxisGainCurve
    vy: AxisGainCurve
    wz: AxisGainCurve

    def axis(self, name: str) -> AxisGainCurve:
        curve: AxisGainCurve = getattr(self, name)
        return curve

    @staticmethod
    def from_dynamics(dynamics: dict[str, list[dict[str, Any]]] | None) -> GainSchedule | None:
        """Build from a characterization artifact's ``dynamics_by_amplitude``
        section. Returns None if any axis has no usable points (caller falls
        back to the constant-K compensator)."""
        if not dynamics:
            return None
        curves: dict[str, AxisGainCurve] = {}
        for axis in _AXES:
            by_amp: dict[float, list[tuple[float, float]]] = {}
            for row in dynamics.get(axis) or []:
                gain = float(row["K"])
                if gain < _MIN_VALID_K:  # collapsed near the floor — unusable
                    continue
                by_amp.setdefault(float(row["amp"]), []).append((gain, float(row["tau"])))
            if not by_amp:
                return None
            amps = sorted(by_amp)
            gains = tuple(float(np.mean([g for g, _ in by_amp[a]])) for a in amps)
            taus = tuple(float(np.mean([t for _, t in by_amp[a]])) for a in amps)
            curves[axis] = AxisGainCurve(tuple(amps), gains, taus)
        return GainSchedule(vx=curves["vx"], vy=curves["vy"], wz=curves["wz"])


class ScheduledGainCompensator:
    """Per-axis gain inversion with K scheduled by command magnitude.

    Drop-in for :class:`FeedforwardGainCompensator`: same ``compute`` /
    ``reset`` surface so the task can swap it in transparently.
    """

    def __init__(self, schedule: GainSchedule, output_limit: tuple[float, float, float]) -> None:
        self._schedule = schedule
        self._limit = output_limit

    def compute(
        self,
        desired_vx: float,
        desired_vy: float,
        desired_wz: float,
        actual_vx: float = 0.0,
        actual_vy: float = 0.0,
        actual_wz: float = 0.0,
    ) -> tuple[float, float, float]:
        ox, oy, oz = self._limit
        return (
            _clamp(self._schedule.vx.invert(desired_vx), ox),
            _clamp(self._schedule.vy.invert(desired_vy), oy),
            _clamp(self._schedule.wz.invert(desired_wz), oz),
        )

    def reset(self) -> None:
        pass


__all__ = [
    "AxisGainCurve",
    "GainSchedule",
    "ScheduledGainCompensator",
]
