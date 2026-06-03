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

"""Costmap-driven autonomous e_max governor.

Consumes ``/global_costmap`` (``OccupancyGrid``) + ``/go2/odom``
(``PoseStamped``), measures local clearance around the robot (or a
look-ahead point), maps it to an ``e_max`` corridor half-width via a
piecewise-linear curve, and publishes ``Float32`` on ``e_max``. The
``ControlCoordinator``'s existing ``_on_e_max`` forwarder broadcasts
the value to :meth:`PrecisionPathFollowerTask.set_e_max`, which
re-solves the velocity profile in place.

Open space → high ``e_max`` (robot drives faster, loose tracking).
Cluttered space → low ``e_max`` (slower, tight tracking).

Coexists with :class:`KeyboardTeleop`: both publish on the same topic
and the coord forwards the last value verbatim. A keystroke
(``0``-``9``) instantly overrides the auto value; the next costmap
tick reapplies the auto value.
"""

from __future__ import annotations

import math
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.gradient import gradient
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ---------------------------------------------------------------------------
# Pure helpers — testable without instantiating the Module. The Module is
# thin glue around these.
# ---------------------------------------------------------------------------


def sample_point(pose: PoseStamped, lookahead_m: float) -> tuple[float, float]:
    """World-frame XY where clearance is sampled.

    ``lookahead_m == 0`` → robot pose. ``lookahead_m > 0`` → project that
    distance along the robot's yaw heading."""
    if lookahead_m == 0.0:
        return pose.position.x, pose.position.y
    yaw = pose.orientation.euler[2]
    return (
        pose.position.x + lookahead_m * math.cos(yaw),
        pose.position.y + lookahead_m * math.sin(yaw),
    )


def sample_path_window(
    path: Path,
    robot_xy: tuple[float, float],
    lookahead_m: float,
    step_m: float,
) -> list[tuple[float, float]]:
    """Sample XY points along ``path`` starting at the waypoint nearest
    the robot, walking forward until total accumulated distance reaches
    ``lookahead_m``. Points are spaced ``step_m`` apart via linear
    interpolation between waypoints. Empty path or zero-length window →
    empty list. The nearest waypoint is always emitted first (distance
    0)."""
    if not path.poses or lookahead_m <= 0.0 or step_m <= 0.0:
        return []

    pts = [(p.position.x, p.position.y) for p in path.poses]
    rx, ry = robot_xy

    best_i = 0
    best_d2 = float("inf")
    for i, (x, y) in enumerate(pts):
        d2 = (x - rx) ** 2 + (y - ry) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i

    samples: list[tuple[float, float]] = [pts[best_i]]
    accumulated = 0.0
    next_emit = step_m

    for i in range(best_i, len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-9:
            continue
        seg_start = accumulated
        while next_emit <= accumulated + seg_len + 1e-9:
            t = (next_emit - seg_start) / seg_len
            samples.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            if next_emit >= lookahead_m - 1e-9:
                return samples
            next_emit += step_m
        accumulated += seg_len
        if accumulated >= lookahead_m:
            break

    return samples


def min_clearance_along(
    gradient_grid: OccupancyGrid,
    points: list[tuple[float, float]],
    d_far: float,
) -> float | None:
    """Read the distance-transform cell at each XY point; return the
    minimum clearance in meters across all in-bounds samples. ``None``
    if every sample fell outside the grid."""
    min_c: float | None = None
    for sx, sy in points:
        idx = gradient_grid.world_to_grid(Vector3(sx, sy, 0.0))
        ix, iy = int(idx.x), int(idx.y)
        if not (0 <= ix < gradient_grid.width and 0 <= iy < gradient_grid.height):
            continue
        cell = int(gradient_grid.grid[iy, ix])
        clearance = d_far * (1.0 - cell / 100.0)
        if min_c is None or clearance < min_c:
            min_c = clearance
    return min_c


def clearance_to_e_max(
    clearance: float,
    d_near: float,
    d_far: float,
    e_max_low: float,
    e_max_high: float,
) -> float:
    """Piecewise-linear: clamp below d_near, clamp above d_far, lerp between."""
    if clearance <= d_near:
        return e_max_low
    if clearance >= d_far:
        return e_max_high
    t = (clearance - d_near) / (d_far - d_near)
    return e_max_low + t * (e_max_high - e_max_low)


def compute_e_max_from_costmap(
    costmap: OccupancyGrid,
    pose: PoseStamped,
    *,
    d_near: float,
    d_far: float,
    e_max_low: float,
    e_max_high: float,
    lookahead_m: float,
    obstacle_threshold: int,
    path: Path | None = None,
    path_lookahead_m: float = 3.0,
    path_sample_step_m: float = 0.10,
) -> float | None:
    """End-to-end: costmap + pose → e_max (m). Pure function — testable
    without a Module.

    Path-aware mode (``path`` is non-empty): sample clearance at every
    ``path_sample_step_m`` along the next ``path_lookahead_m`` of the
    path starting from the waypoint nearest the robot, take the
    minimum, map that to e_max. Anticipates pinch corners before the
    robot reaches them.

    Heading-based fallback (no path, or empty path): single sample at
    ``sample_point(pose, lookahead_m)``. Used in standalone-spy runs
    and during the brief window before the first goal is published.

    Returns ``None`` if every relevant sample falls outside the grid."""
    gradient_grid = gradient(costmap, obstacle_threshold=obstacle_threshold, max_distance=d_far)

    if path is not None and path.poses:
        samples = sample_path_window(
            path,
            (pose.position.x, pose.position.y),
            lookahead_m=path_lookahead_m,
            step_m=path_sample_step_m,
        )
        clearance = min_clearance_along(gradient_grid, samples, d_far)
    else:
        sx, sy = sample_point(pose, lookahead_m)
        clearance = min_clearance_along(gradient_grid, [(sx, sy)], d_far)

    if clearance is None:
        return None
    return clearance_to_e_max(clearance, d_near, d_far, e_max_low, e_max_high)


class CostmapPrecisionGovernorConfig(ModuleConfig):
    """Knobs for :class:`CostmapPrecisionGovernor`. All in SI units."""

    # Clearance (m) below which we clamp to the tight-corridor floor.
    d_near: float = 0.30
    # Clearance (m) above which we clamp to the open-space ceiling.
    d_far: float = 1.0
    # e_max (m) emitted at or below d_near.
    e_max_low: float = 0.1
    # e_max (m) emitted at or above d_far.
    e_max_high: float = 0.4
    # Minimum |Δe_max| since last publish required to emit again.
    # Suppresses thrashing the task's solve_profile() on every costmap.
    hysteresis_delta: float = 0.02
    # Heading-based fallback lookahead (m). Used only when no Path has
    # been seen yet (standalone-spy, pre-first-goal). Once a Path is
    # cached, path-window sampling takes over.
    lookahead_m: float = 0.50
    # Path-window sampling: scan the next path_lookahead_m of the
    # planned path starting at the waypoint nearest the robot, sampling
    # every path_sample_step_m, and take the MIN clearance. Anticipates
    # pinch corners before the robot reaches them.
    path_lookahead_m: float = 3.0
    path_sample_step_m: float = 0.10
    # Costmap cell value threshold treated as an obstacle by gradient().
    obstacle_threshold: int = 50
    # Emit one value on first costmap so the consuming task isn't stuck
    # at its compile-time default.
    publish_initial: bool = True


class CostmapPrecisionGovernor(Module):
    """Autonomous ``e_max`` publisher driven by local costmap clearance.

    Stream contract — fed by the precision-nav blueprint via the same
    LCM topics the planner already consumes:

    - ``global_costmap: In[OccupancyGrid]`` — reactive; recompute on
      every new map.
    - ``odom: In[PoseStamped]`` — latest pose stored, used to anchor
      the clearance sample.
    - ``e_max: Out[Float32]`` — published when the clearance-derived
      e_max changes by more than ``hysteresis_delta``.

    The math reuses :func:`dimos.mapping.occupancy.gradient.gradient`:
    one SciPy distance-transform per costmap, then a single cell read
    at the sample point.
    """

    config: CostmapPrecisionGovernorConfig

    global_costmap: In[OccupancyGrid]
    odom: In[PoseStamped]
    path: In[Path]
    e_max: Out[Float32]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_odom: PoseStamped | None = None
        self._latest_path: Path | None = None
        self._last_published: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))

    def _on_odom(self, msg: PoseStamped) -> None:
        # Cheap callback — just stash; clearance read happens on costmap.
        self._latest_odom = msg

    def _on_path(self, msg: Path) -> None:
        # Stash; the next costmap tick switches to path-window sampling.
        self._latest_path = msg

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        if self._latest_odom is None:
            # No pose yet — nothing to anchor the clearance read at.
            return
        cfg = self.config
        new_e_max = compute_e_max_from_costmap(
            msg,
            self._latest_odom,
            d_near=cfg.d_near,
            d_far=cfg.d_far,
            e_max_low=cfg.e_max_low,
            e_max_high=cfg.e_max_high,
            lookahead_m=cfg.lookahead_m,
            obstacle_threshold=cfg.obstacle_threshold,
            path=self._latest_path,
            path_lookahead_m=cfg.path_lookahead_m,
            path_sample_step_m=cfg.path_sample_step_m,
        )
        if new_e_max is None:
            return  # All samples fell outside the grid.
        if self._should_publish(new_e_max):
            self.e_max.publish(Float32(data=new_e_max))
            self._last_published = new_e_max

    def _should_publish(self, new_e_max: float) -> bool:
        if self._last_published is None:
            return self.config.publish_initial
        return abs(new_e_max - self._last_published) > self.config.hysteresis_delta


__all__ = [
    "CostmapPrecisionGovernor",
    "CostmapPrecisionGovernorConfig",
    "clearance_to_e_max",
    "compute_e_max_from_costmap",
    "min_clearance_along",
    "sample_path_window",
    "sample_point",
]
