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

"""Waypoint path -> time-parameterized reference trajectory.

Curvature-aware, accel-limited speed profile along the path arc length:
the speed is capped to sqrt(A_LAT_MAX / curvature) so corners are taken at
a speed the base can actually hold (no overshoot), and reduces to a plain
trapezoid on straight paths. Heading is profiled independently (the base is
holonomic) — either tracking the path tangent or holding a fixed heading.
``sample(t)`` is a cheap table lookup, so the controller can evaluate the
reference at any time (including per-axis dead-time previews).

The corner slowdown is delegated to
:class:`~dimos.control.tasks.velocity_profiler.VelocityProfiler`. All limits
come from the constants module (85% planning margins) so the firmware
command limiter never engages.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from dimos.control.tasks.trajectory_tracking_task.constants import (
    A_LAT_MAX,
    PLAN_MAX_ACC,
    PLAN_MAX_VEL,
)
from dimos.control.tasks.velocity_profiler import VelocityProfiler
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.trigonometry import angle_diff

# Below this path length there is no room for a meaningful profile;
# treat the trajectory as a pure rotate/hold at the endpoint.
_MIN_PATH_LENGTH_M = 1e-6

# Arc-length resample step for curvature estimation. Matches the path
# waypoint step and the omega central-difference step so curvature is
# well-conditioned regardless of input waypoint density.
_DS_GRID_M = 0.05
# Below this length, curvature is not meaningful — fall back to a plain
# trapezoid (also covers the rotate/hold degenerate case).
_MIN_PROFILE_LENGTH_M = 2.0 * _DS_GRID_M
# Floor for dt = ds / v_avg so the time integral stays finite at the
# forced-rest endpoints.
_EPS_SPEED = 1e-3


@dataclass(frozen=True)
class TrajectorySample:
    """Reference state at time ``t`` (world frame)."""

    t: float
    x: float
    y: float
    yaw: float
    vx_world: float
    vy_world: float
    omega: float


@dataclass(frozen=True)
class TrapezoidalProfile:
    """Closed-form trapezoidal s(t) along a 1-D distance.

    Degenerates to a triangle profile when the distance is too short to
    reach cruise speed.
    """

    distance: float
    v_cruise: float
    accel: float
    t_accel: float
    t_cruise: float

    @staticmethod
    def plan(distance: float, v_max: float, a_max: float) -> TrapezoidalProfile:
        distance = abs(distance)
        if distance < _MIN_PATH_LENGTH_M or v_max <= 0.0 or a_max <= 0.0:
            return TrapezoidalProfile(distance, 0.0, a_max, 0.0, 0.0)
        v_peak_triangle = math.sqrt(distance * a_max)
        v_cruise = min(v_max, v_peak_triangle)
        t_accel = v_cruise / a_max
        d_accel = 0.5 * a_max * t_accel * t_accel
        t_cruise = max(0.0, (distance - 2.0 * d_accel) / v_cruise)
        return TrapezoidalProfile(distance, v_cruise, a_max, t_accel, t_cruise)

    @property
    def duration(self) -> float:
        return 2.0 * self.t_accel + self.t_cruise

    def sample(self, t: float) -> tuple[float, float]:
        """Return (s, v) at time t; clamps to the endpoints."""
        if t <= 0.0:
            return 0.0, 0.0
        if t >= self.duration:
            return self.distance, 0.0
        a, ta, tc = self.accel, self.t_accel, self.t_cruise
        if t < ta:
            return 0.5 * a * t * t, a * t
        d_accel = 0.5 * a * ta * ta
        if t < ta + tc:
            return d_accel + self.v_cruise * (t - ta), self.v_cruise
        t_dec = t - ta - tc
        return (
            d_accel + self.v_cruise * tc + self.v_cruise * t_dec - 0.5 * a * t_dec * t_dec,
            self.v_cruise - a * t_dec,
        )


def _accel_pass_1d(s: np.ndarray, v: np.ndarray, a: float, forward: bool) -> np.ndarray:
    """Forward/backward acceleration-limited pass on a 1-D arc-length grid.

    Same recurrence as VelocityProfiler._acceleration_pass, applied here so
    a forced-rest endpoint propagates into a proper ramp instead of a speed
    discontinuity. ``v[i] <= sqrt(v[j]^2 + 2*a*ds)``.
    """
    out = v.copy()
    rng = range(1, len(s)) if forward else range(len(s) - 2, -1, -1)
    for i in rng:
        j = i - 1 if forward else i + 1
        ds = abs(float(s[i] - s[j]))
        if ds > _MIN_PATH_LENGTH_M:
            out[i] = min(out[i], math.sqrt(out[j] ** 2 + 2.0 * a * ds))
    return out


class ArcLengthProfile:
    """Curvature-aware s(t)/v(t) over a path's arc length.

    Duck-typed with :class:`TrapezoidalProfile` (``.sample(t) -> (s, v)``,
    ``.duration``, ``.distance``, ``.v_cruise``) so it drops into
    :class:`TimedTrajectory` unchanged. The corner slowdown comes entirely
    from :class:`~dimos.control.tasks.velocity_profiler.VelocityProfiler`
    (curvature -> centripetal-accel cap -> accel/decel passes); this class
    only forces rest at the two ends and inverts the speed profile to time.
    """

    def __init__(self, s_grid: np.ndarray, v_grid: np.ndarray, t_grid: np.ndarray) -> None:
        self._s = s_grid
        self._v = v_grid
        self._t = t_grid

    @property
    def distance(self) -> float:
        return float(self._s[-1])

    @property
    def duration(self) -> float:
        return float(self._t[-1])

    @property
    def v_cruise(self) -> float:
        return float(self._v.max())

    def sample(self, t: float) -> tuple[float, float]:
        if t <= 0.0:
            return 0.0, 0.0
        if t >= self._t[-1]:
            return float(self._s[-1]), 0.0
        s = float(np.interp(t, self._t, self._s))
        v = float(np.interp(t, self._t, self._v))
        return s, v


def _build_speed_profile(
    points: np.ndarray, cum_dist: np.ndarray, v_max: float, a_max: float
) -> ArcLengthProfile | TrapezoidalProfile:
    """Curvature-aware speed profile over the path, rest-to-rest.

    Resamples to a uniform arc-length grid, runs the curvature + accel/decel
    profiler, forces both endpoints to rest, then integrates to a time grid.
    Falls back to a plain trapezoid for paths too short to resolve curvature
    (where the result would be a trapezoid anyway).
    """
    total = float(cum_dist[-1])
    if total < _MIN_PROFILE_LENGTH_M or v_max <= 0.0 or a_max <= 0.0:
        return TrapezoidalProfile.plan(total, v_max, a_max)

    n = max(2, int(total / _DS_GRID_M) + 1)
    s_grid = np.linspace(0.0, total, n)
    x_grid = np.interp(s_grid, cum_dist, points[:, 0])
    y_grid = np.interp(s_grid, cum_dist, points[:, 1])
    grid_path = Path(
        poses=[
            PoseStamped(position=Vector3(float(x), float(y), 0.0))
            for x, y in zip(x_grid, y_grid, strict=True)
        ]
    )

    profiler = VelocityProfiler(
        max_linear_speed=v_max,
        max_linear_accel=a_max,
        max_linear_decel=a_max,
        max_centripetal_accel=A_LAT_MAX,
        min_speed=_EPS_SPEED,
    )
    v_grid = profiler.compute_profile(grid_path).astype(float)

    # The profiler deliberately does not zero the endpoints; force rest and
    # propagate it with the same accel/decel recurrence so the start/end ramps
    # respect a_max instead of jumping.
    v_grid[0] = 0.0
    v_grid[-1] = 0.0
    v_grid = _accel_pass_1d(s_grid, v_grid, a_max, forward=True)
    v_grid = _accel_pass_1d(s_grid, v_grid, a_max, forward=False)

    # Invert v(s) to t(s): dt = ds / v_avg over each grid interval.
    ds = np.diff(s_grid)
    v_avg = np.maximum(0.5 * (v_grid[:-1] + v_grid[1:]), _EPS_SPEED)
    t_grid = np.concatenate([[0.0], np.cumsum(ds / v_avg)])
    return ArcLengthProfile(s_grid, v_grid, t_grid)


class TimedTrajectory:
    """Time-parameterized reference built from a waypoint Path."""

    def __init__(
        self,
        points: np.ndarray,
        cum_dist: np.ndarray,
        profile: ArcLengthProfile | TrapezoidalProfile,
        yaw_start: float,
        yaw_end: float,
        heading_mode: str,
        yaw_profile: TrapezoidalProfile,
    ) -> None:
        self._points = points
        self._cum_dist = cum_dist
        self._profile = profile
        self._yaw_start = yaw_start
        self._yaw_end = yaw_end
        self._heading_mode = heading_mode
        self._yaw_profile = yaw_profile

    @staticmethod
    def from_path(
        path: Path,
        max_speed: float | None = None,
        heading_mode: str = "tangent",
        fixed_heading: float = 0.0,
    ) -> TimedTrajectory:
        """Build from waypoints. ``max_speed`` caps the cruise speed below
        the planning margin (it can never raise it above PLAN_MAX_VEL)."""
        if heading_mode not in ("tangent", "fixed"):
            raise ValueError(f"unknown heading_mode {heading_mode!r}")
        if len(path.poses) < 1:
            raise ValueError("path has no poses")

        points = np.array([[p.position.x, p.position.y] for p in path.poses], dtype=float)
        if len(points) == 1:
            points = np.vstack([points, points])
        seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cum_dist = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(cum_dist[-1])

        # The profile is along-path; vx/vy split per segment direction can
        # momentarily load one axis fully, so plan with the tighter of the
        # two linear axes. Curvature-aware: slows into corners so the base
        # can hold the path; reduces to a plain trapezoid on straight paths.
        v_max = min(PLAN_MAX_VEL.x, PLAN_MAX_VEL.y)
        a_max = min(PLAN_MAX_ACC.x, PLAN_MAX_ACC.y)
        if max_speed is not None:
            v_max = min(v_max, max_speed)
        profile = _build_speed_profile(points, cum_dist, v_max, a_max)

        if heading_mode == "fixed":
            yaw_start = fixed_heading
            yaw_end = fixed_heading
        else:
            yaw_start = TimedTrajectory._tangent_yaw(points, cum_dist, 0.0)
            yaw_end = TimedTrajectory._tangent_yaw(points, cum_dist, total)
        # Yaw rate budget for the independent heading profile. In tangent
        # mode yaw follows the path tangent continuously; this profile is
        # only used for the fixed-mode swing (start -> fixed heading is the
        # caller's job; here it covers initial-yaw alignment hold).
        yaw_span = abs(angle_diff(yaw_end, yaw_start))
        yaw_profile = TrapezoidalProfile.plan(yaw_span, PLAN_MAX_VEL.yaw, PLAN_MAX_ACC.yaw)

        return TimedTrajectory(
            points, cum_dist, profile, yaw_start, yaw_end, heading_mode, yaw_profile
        )

    @property
    def duration(self) -> float:
        return self._profile.duration

    @property
    def length(self) -> float:
        return self._profile.distance

    @property
    def max_speed(self) -> float:
        return self._profile.v_cruise

    @staticmethod
    def _tangent_yaw(points: np.ndarray, cum_dist: np.ndarray, s: float) -> float:
        i = int(np.searchsorted(cum_dist, s, side="right")) - 1
        i = max(0, min(i, len(points) - 2))
        d = points[i + 1] - points[i]
        if float(np.hypot(d[0], d[1])) < _MIN_PATH_LENGTH_M:
            return 0.0
        return float(math.atan2(d[1], d[0]))

    def _interp(self, s: float) -> tuple[float, float, float]:
        """Position + tangent yaw at arc length s."""
        s = max(0.0, min(s, float(self._cum_dist[-1])))
        x = float(np.interp(s, self._cum_dist, self._points[:, 0]))
        y = float(np.interp(s, self._cum_dist, self._points[:, 1]))
        return x, y, self._tangent_yaw(self._points, self._cum_dist, s)

    def sample(self, t: float) -> TrajectorySample:
        s, v = self._profile.sample(t)
        x, y, tangent = self._interp(s)

        if self._heading_mode == "fixed":
            yaw = self._yaw_start
            omega = 0.0
        else:
            yaw = tangent if self.length > _MIN_PATH_LENGTH_M else self._yaw_start
            # Tangent yaw rate = curvature * speed; estimate curvature by a
            # short central difference along s.
            ds = 0.05
            _, _, yaw_ahead = self._interp(s + ds)
            _, _, yaw_behind = self._interp(s - ds)
            omega = angle_diff(yaw_ahead, yaw_behind) / (2.0 * ds) * v

        return TrajectorySample(
            t=t,
            x=x,
            y=y,
            yaw=yaw,
            vx_world=v * math.cos(tangent),
            vy_world=v * math.sin(tangent),
            omega=omega,
        )

    def end_sample(self) -> TrajectorySample:
        return self.sample(self.duration)
