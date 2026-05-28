# Copyright 2026 Dimensional Inc.
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

"""Reference governor math: per-waypoint Maximum Velocity Constraints +
forward/backward accel-limited solver.

The reference-governor algorithm here gives a per-waypoint velocity
profile that keeps tracking error inside an ``±e_max`` corridor under a
given FOPDT plant. ``solve_profile()`` composes the constraint
upper-bounds (``min`` across a list of :class:`VelocityConstraint`
implementations) and runs the existing forward/backward accel passes
from :class:`VelocityProfiler`.

The four built-in MVCs each express one physical/actuator/error limit:

    GeometricMVC   v <= v_max                         (absolute platform cap)
    SaturationMVC  v <= omega_max / |kappa|           (turn-rate saturation)
    LateralMVC     v <= sqrt(a_lat_max / |kappa|)     (centripetal accel cap)
    PrecisionMVC   v <= e_max / max(tau+L per chan)   (FOPDT tracking budget)

Add a new constraint by writing one more class implementing
:class:`VelocityConstraint` and including it in the solver's constraint
list — no solver change needed.

Live consumer: :class:`PrecisionPathFollowerTask`, which calls
:func:`solve_profile` once per path and again on each ``e_max`` update,
atomically swapping its cached profile.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.velocity_profiler import VelocityProfiler
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.benchmarking.tuning import PlantModelDC

# ---------------------------------------------------------------------------
# PathSpeedCap method contract — the consumption seam in PathFollowerTask.
# ---------------------------------------------------------------------------


@runtime_checkable
class PathSpeedCapProtocol(Protocol):
    """Duck-type contract for objects that can be installed as the
    follower's ``_profile_cap``. Mirrors the shape of
    :class:`dimos.utils.benchmarking.velocity_profile.PathSpeedCap`.
    """

    def for_path(self, path: Path) -> None: ...

    def speed_limit_at(self, x: float, y: float) -> float: ...

    def cap(
        self, x: float, y: float, vx: float, vy: float, wz: float
    ) -> tuple[float, float, float]: ...


# ---------------------------------------------------------------------------
# Velocity constraint generators — per-waypoint pure-function upper bounds.
# ---------------------------------------------------------------------------


@dataclass
class ConstraintContext:
    """Path-derived context passed to each constraint's upper_bound.

    Bundling curvatures (computed once per path) avoids each curvature-
    dependent constraint recomputing the same numbers.
    """

    path: Path
    curvatures: NDArray[np.float64]  # length len(path.poses); abs curvature in 1/m
    plant: PlantModelDC


class VelocityConstraint(Protocol):
    name: str

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float: ...


_INF = float("inf")
_KAPPA_EPS = 1e-6


@dataclass
class GeometricMVC:
    """Static linear-speed cap (artifact's ``velocity_profile.max_linear_speed``)."""

    v_max: float
    name: str = "geometric"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        return float(self.v_max)


@dataclass
class SaturationMVC:
    """Turn-rate saturation: v <= omega_max / |kappa|. HARD cap — above
    this the controller can't track the geometry at all."""

    omega_max: float
    name: str = "saturation"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        kappa = float(ctx.curvatures[s_idx])
        if kappa < _KAPPA_EPS:
            return _INF
        return self.omega_max / kappa


@dataclass
class LateralMVC:
    """Lateral-comfort cap: v <= sqrt(a_lat_max / |kappa|)."""

    a_lat_max: float
    name: str = "lateral"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        kappa = float(ctx.curvatures[s_idx])
        if kappa < _KAPPA_EPS:
            return _INF
        return float(np.sqrt(self.a_lat_max / kappa))


@dataclass
class PrecisionMVC:
    """Precision cap derived from the FOPDT plant CTE floor:

        v <= e_max / max(tau_vx + L_vx, tau_wz + L_wz)

    The straight-line characterization fits only `(tau_vx + L_vx) * v` as
    the CTE floor; that holds on straight segments. On curved segments
    the heading-tracking lag (a function of `tau_wz + L_wz`) dominates
    instead. Empirically (slalom/smooth_corner/figure_eight sim runs)
    using only the vx channel under-predicts CTE on curved paths by ~2x;
    taking max(vx_chan, wz_chan) halves the residual at the cost of a
    proportionally lower v.

    e_max read via a callable so callers can hot-update the runtime
    corridor half-width without rebuilding the constraint. The bound is
    constant across waypoints (κ-independent); the min() in the solver
    handles composition with the κ-dependent caps.
    """

    e_max_provider: Callable[[], float]
    min_e_max: float = 0.005  # 5 mm floor: keeps v from collapsing to 0
    name: str = "precision"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        e_max = max(float(self.e_max_provider()), self.min_e_max)
        tau_plus_L = max(
            float(ctx.plant.vx.tau + ctx.plant.vx.L),
            float(ctx.plant.wz.tau + ctx.plant.wz.L),
        )
        if tau_plus_L < 1e-9:
            return _INF
        return e_max / tau_plus_L


# ---------------------------------------------------------------------------
# Solver: compose constraints, then reuse existing accel passes.
# ---------------------------------------------------------------------------


def _path_pts(path: Path) -> NDArray[np.float64]:
    return np.array([[p.position.x, p.position.y] for p in path.poses], dtype=float)


def solve_profile(
    path: Path,
    plant: PlantModelDC,
    constraints: Sequence[VelocityConstraint],
    *,
    accel_max: float,
    decel_max: float,
    min_speed: float,
    curvatures: NDArray[np.float64] | None = None,
    pts: NDArray[np.float64] | None = None,
) -> NDArray[np.float64]:
    """Per-waypoint MVC = min(constraints), then forward/backward accel
    passes via the existing :class:`VelocityProfiler` helpers. ``pts``
    and ``curvatures`` can be passed in to avoid recomputing on hot
    paths (e_max updates with a fixed path)."""
    n = len(path.poses)
    if n < 2:
        return np.array([min_speed], dtype=float)

    if pts is None:
        pts = _path_pts(path)
    if curvatures is None:
        # _compute_curvatures depends only on pts (it's stateless w.r.t. profiler config).
        curvatures = VelocityProfiler()._compute_curvatures(pts)

    ctx = ConstraintContext(path=path, curvatures=curvatures, plant=plant)
    mvc = np.empty(n, dtype=float)
    for i in range(n):
        mvc[i] = min(c.upper_bound(ctx, i) for c in constraints)

    # Reuse the forward/backward accel passes verbatim.
    profiler = VelocityProfiler(
        max_linear_accel=accel_max,
        max_linear_decel=decel_max,
    )
    v = profiler._acceleration_pass(pts, mvc, forward=True)
    v = profiler._acceleration_pass(pts, v, forward=False)
    return np.maximum(v, min_speed)


__all__ = [
    "ConstraintContext",
    "GeometricMVC",
    "LateralMVC",
    "PathSpeedCapProtocol",
    "PrecisionMVC",
    "SaturationMVC",
    "VelocityConstraint",
    "solve_profile",
]
