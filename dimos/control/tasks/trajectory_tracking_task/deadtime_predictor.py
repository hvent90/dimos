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

"""Dead-time-compensated pose prediction for the trajectory tracker.

The tracker already previews the feedforward reference by each velocity axis'
dead time. This module handles the feedback side: a measured pose can reflect
commands that are still working through the velocity plant, so proportional
feedback reacts late. The modified Smith form used here advances the measured
pose by the nominal model's motion over the recent in-flight command window:

    predicted_pose = measured_pose + (model_pose(now) - model_pose(now - H))

Only the model delta is used, so long-term model drift is not fed back as an
absolute pose. The block is strictly opt-in.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from dimos.control.tasks.trajectory_tracking_task.config import PerAxis, TrackingConfig
from dimos.control.tasks.trajectory_tracking_task.gain_schedule import GainSchedule
from dimos.utils.trigonometry import angle_diff

Pose2 = tuple[float, float, float]
Twist3 = tuple[float, float, float]

_MODEL_DT = 0.01
_EPS = 1e-9


@dataclass(frozen=True)
class DeadtimePredictorConfig:
    """Model and horizon for feedback dead-time compensation."""

    k_hat: PerAxis
    tau: PerAxis
    deadtime: PerAxis
    horizon_s: float
    schedule: GainSchedule | None = None
    blend: float = 1.0
    mode: Literal["full", "yaw_only"] = "full"
    model_dt: float = _MODEL_DT


def build_deadtime_predictor(
    tracking: TrackingConfig,
    *,
    feedback_lag_s: float = 0.0,
    blend: float = 1.0,
    mode: Literal["full", "yaw_only"] = "full",
) -> DeadtimePosePredictor:
    """Build a predictor from the same FOPDT fit the tracker already uses."""
    horizon_s = max(tracking.deadtime.x, tracking.deadtime.y, tracking.deadtime.yaw)
    horizon_s += max(0.0, feedback_lag_s)
    return DeadtimePosePredictor(
        DeadtimePredictorConfig(
            k_hat=tracking.k_hat,
            tau=tracking.tau,
            deadtime=tracking.deadtime,
            horizon_s=horizon_s,
            schedule=tracking.schedule,
            blend=_clamp01(blend),
            mode=mode,
        )
    )


class DeadtimePosePredictor:
    """Nominal FOPDT command-history predictor for one holonomic twist base."""

    def __init__(self, cfg: DeadtimePredictorConfig) -> None:
        self.cfg = cfg
        self._t: float | None = None
        self._pose: Pose2 = (0.0, 0.0, 0.0)
        self._vel: Twist3 = (0.0, 0.0, 0.0)
        self._last_command: Twist3 = (0.0, 0.0, 0.0)
        self._commands: list[tuple[float, Twist3]] = []
        self._history: list[tuple[float, Pose2]] = []

    def reset(self, t: float | None = None, pose: Pose2 = (0.0, 0.0, 0.0)) -> None:
        """Clear state. If ``t`` is supplied, seed the model at ``pose``."""
        self._t = t
        self._pose = pose
        self._vel = (0.0, 0.0, 0.0)
        self._last_command = (0.0, 0.0, 0.0)
        self._commands = []
        self._history = []
        if t is not None:
            self._commands.append((t, self._last_command))
            self._history.append((t, pose))

    def seed(self, t: float, pose: Pose2) -> None:
        if self._t is None:
            self.reset(t, pose)

    def predict(self, measured_pose: Pose2, t: float) -> Pose2:
        """Return ``measured_pose`` advanced by the recent nominal model delta."""
        self.seed(t, measured_pose)
        self._advance_to(t)
        if self.cfg.horizon_s <= _EPS or len(self._history) < 2:
            return measured_pose
        delayed = self._pose_at(t - self.cfg.horizon_s)
        dx = self._pose[0] - delayed[0]
        dy = self._pose[1] - delayed[1]
        dyaw = angle_diff(self._pose[2], delayed[2])
        blend = _clamp01(self.cfg.blend)
        if self.cfg.mode == "yaw_only":
            dx = 0.0
            dy = 0.0
        return (
            measured_pose[0] + blend * dx,
            measured_pose[1] + blend * dy,
            measured_pose[2] + blend * dyaw,
        )

    def record_command(self, t: float, command: Twist3) -> None:
        """Advance the nominal model to ``t`` and hold ``command`` from there."""
        self.seed(t, self._pose)
        self._advance_to(t)
        self._last_command = command
        if not self._commands or t > self._commands[-1][0] + _EPS:
            self._commands.append((t, command))
        else:
            self._commands[-1] = (t, command)
        self._trim(t)

    def _advance_to(self, t: float) -> None:
        if self._t is None:
            return
        while self._t + _EPS < t:
            dt = min(self.cfg.model_dt, t - self._t)
            self._step(dt)
            self._t += dt
            self._history.append((self._t, self._pose))

    def _step(self, dt: float) -> None:
        vx, vy, wz = self._vel
        x, y, yaw = self._pose
        cmd_x = self._command_at(self._t_or_zero() - self.cfg.deadtime.x)[0]
        cmd_y = self._command_at(self._t_or_zero() - self.cfg.deadtime.y)[1]
        cmd_yaw = self._command_at(self._t_or_zero() - self.cfg.deadtime.yaw)[2]
        kx, taux = self._axis_model("vx", cmd_x)
        ky, tauy = self._axis_model("vy", cmd_y)
        kyaw, tauyaw = self._axis_model("wz", cmd_yaw)
        vx = self._axis_step(vx, cmd_x, kx, taux, dt)
        vy = self._axis_step(vy, cmd_y, ky, tauy, dt)
        wz = self._axis_step(wz, cmd_yaw, kyaw, tauyaw, dt)
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw = (yaw + wz * dt + math.pi) % (2.0 * math.pi) - math.pi
        self._vel = (vx, vy, wz)
        self._pose = (x, y, yaw)

    def _t_or_zero(self) -> float:
        assert self._t is not None
        return self._t

    @staticmethod
    def _axis_step(y: float, u: float, k: float, tau: float, dt: float) -> float:
        tau = max(tau, _EPS)
        alpha = 1.0 - math.exp(-dt / tau)
        return y + alpha * (k * u - y)

    def _axis_model(self, axis: Literal["vx", "vy", "wz"], command: float) -> tuple[float, float]:
        if self.cfg.schedule is not None:
            curve = self.cfg.schedule.axis(axis)
            return curve.k_at(command), curve.tau_at(command)
        if axis == "vx":
            return self.cfg.k_hat.x, self.cfg.tau.x
        if axis == "vy":
            return self.cfg.k_hat.y, self.cfg.tau.y
        return self.cfg.k_hat.yaw, self.cfg.tau.yaw

    def _command_at(self, t: float) -> Twist3:
        if not self._commands or t < self._commands[0][0]:
            return (0.0, 0.0, 0.0)
        lo = 0
        hi = len(self._commands) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            if self._commands[mid][0] <= t:
                lo = mid + 1
            else:
                hi = mid - 1
        return self._commands[max(0, hi)][1]

    def _pose_at(self, t: float) -> Pose2:
        if not self._history or t <= self._history[0][0]:
            return self._history[0][1] if self._history else self._pose
        if t >= self._history[-1][0]:
            return self._history[-1][1]
        lo = 0
        hi = len(self._history) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            if self._history[mid][0] <= t:
                lo = mid + 1
            else:
                hi = mid - 1
        t0, p0 = self._history[hi]
        t1, p1 = self._history[hi + 1]
        frac = (t - t0) / max(t1 - t0, _EPS)
        return (
            p0[0] + frac * (p1[0] - p0[0]),
            p0[1] + frac * (p1[1] - p0[1]),
            p0[2] + frac * angle_diff(p1[2], p0[2]),
        )

    def _trim(self, t: float) -> None:
        keep_after = t - self.cfg.horizon_s - max(self.cfg.deadtime.as_tuple()) - 1.0
        while len(self._commands) > 2 and self._commands[1][0] < keep_after:
            self._commands.pop(0)
        while len(self._history) > 2 and self._history[1][0] < keep_after:
            self._history.pop(0)


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


__all__ = [
    "DeadtimePosePredictor",
    "DeadtimePredictorConfig",
    "build_deadtime_predictor",
]
