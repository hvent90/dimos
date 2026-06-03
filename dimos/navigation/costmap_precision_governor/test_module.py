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

"""Pure unit tests for the costmap-clutter-governor math. No Module
instantiation, no LCM, no fixture data — synthetic costmaps only."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.costmap_precision_governor.module import (
    clearance_to_e_max,
    compute_e_max_from_costmap,
    min_clearance_along,
    sample_path_window,
    sample_point,
)


def _path(points: list[tuple[float, float]]) -> Path:
    return Path(poses=[_pose(x, y) for x, y in points])


# Common config used across tests so the hand-picked numbers line up.
D_NEAR = 0.30
D_FAR = 1.50
E_LOW = 0.05
E_HIGH = 0.90


def _pose(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _free_grid(side_m: float = 5.0, resolution: float = 0.05) -> OccupancyGrid:
    """All-FREE grid centred on the world origin."""
    n = int(side_m / resolution)
    grid = np.zeros((n, n), dtype=np.int8)
    from dimos.msgs.geometry_msgs.Pose import Pose

    origin = Pose(position=Vector3(-side_m / 2.0, -side_m / 2.0, 0.0))
    return OccupancyGrid(grid=grid, resolution=resolution, origin=origin)


def _grid_with_obstacle_at(
    obstacle_xy: tuple[float, float],
    side_m: float = 5.0,
    resolution: float = 0.05,
) -> OccupancyGrid:
    """Empty grid with a single OCCUPIED cell at the given world point."""
    g = _free_grid(side_m, resolution)
    idx = g.world_to_grid(Vector3(obstacle_xy[0], obstacle_xy[1], 0.0))
    g.grid[int(idx.y), int(idx.x)] = 100
    return g


# ---------------------------------------------------------------------------
# clearance_to_e_max — pure piecewise-linear curve
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "clearance, expected",
    [
        (0.0, E_LOW),  # well below d_near → floor
        (D_NEAR, E_LOW),  # exactly at d_near → still floor
        (D_FAR, E_HIGH),  # exactly at d_far → ceiling
        (10.0, E_HIGH),  # well above d_far → ceiling
        ((D_NEAR + D_FAR) / 2.0, (E_LOW + E_HIGH) / 2.0),  # midpoint → linear midpoint
    ],
)
def test_clearance_to_e_max_piecewise(clearance: float, expected: float) -> None:
    got = clearance_to_e_max(clearance, D_NEAR, D_FAR, E_LOW, E_HIGH)
    assert got == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# sample_point — lookahead projection along yaw
# ---------------------------------------------------------------------------


def test_sample_point_no_lookahead_returns_pose_xy() -> None:
    p = _pose(1.0, 2.0, yaw=0.5)
    assert sample_point(p, 0.0) == (1.0, 2.0)


@pytest.mark.parametrize(
    "yaw, expected_dx, expected_dy",
    [
        (0.0, 0.5, 0.0),  # +x
        (math.pi / 2, 0.0, 0.5),  # +y
        (math.pi, -0.5, 0.0),  # -x
        (-math.pi / 2, 0.0, -0.5),  # -y
    ],
)
def test_sample_point_projects_along_yaw(
    yaw: float, expected_dx: float, expected_dy: float
) -> None:
    p = _pose(0.0, 0.0, yaw=yaw)
    sx, sy = sample_point(p, lookahead_m=0.5)
    assert sx == pytest.approx(expected_dx, abs=1e-9)
    assert sy == pytest.approx(expected_dy, abs=1e-9)


# ---------------------------------------------------------------------------
# compute_e_max_from_costmap — end-to-end pure pipeline
# ---------------------------------------------------------------------------


def _kwargs(lookahead_m: float = 0.0) -> dict:
    return dict(
        d_near=D_NEAR,
        d_far=D_FAR,
        e_max_low=E_LOW,
        e_max_high=E_HIGH,
        lookahead_m=lookahead_m,
        obstacle_threshold=50,
    )


def test_open_space_yields_e_max_high() -> None:
    # All FREE → clearance saturates at d_far → e_max_high.
    grid = _free_grid()
    e = compute_e_max_from_costmap(grid, _pose(0.0, 0.0), **_kwargs())
    assert e == pytest.approx(E_HIGH, abs=1e-6)


def test_obstacle_at_robot_yields_e_max_low() -> None:
    # Obstacle in the SAME cell as the robot → clearance ~0 → e_max_low.
    grid = _grid_with_obstacle_at((0.0, 0.0))
    e = compute_e_max_from_costmap(grid, _pose(0.0, 0.0), **_kwargs())
    assert e == pytest.approx(E_LOW, abs=1e-6)


def test_lookahead_sees_obstacle_ahead_of_robot() -> None:
    # Robot at origin facing +x; obstacle at (1.0, 0) — robot itself is in
    # open space but the lookahead point (0.5, 0) is much closer to the
    # obstacle than the robot is, so e_max should drop below the open-
    # space ceiling.
    grid = _grid_with_obstacle_at((1.0, 0.0))
    e_no_lookahead = compute_e_max_from_costmap(grid, _pose(0.0, 0.0), **_kwargs(0.0))
    e_with_lookahead = compute_e_max_from_costmap(grid, _pose(0.0, 0.0, yaw=0.0), **_kwargs(0.5))
    assert e_with_lookahead < e_no_lookahead


def test_returns_none_when_sample_point_outside_grid() -> None:
    # Grid is 5m wide centred at origin → bounds roughly [-2.5, +2.5]m.
    # Pose at (10, 10) is well outside.
    grid = _free_grid(side_m=5.0)
    e = compute_e_max_from_costmap(grid, _pose(10.0, 10.0), **_kwargs())
    assert e is None


# ---------------------------------------------------------------------------
# Hysteresis — exercise _should_publish via a tiny stand-in
# ---------------------------------------------------------------------------


class _HysteresisStub:
    """Mirrors CostmapPrecisionGovernor._should_publish without pulling in
    the whole Module machinery."""

    def __init__(self, delta: float, publish_initial: bool = True) -> None:
        self.delta = delta
        self.publish_initial = publish_initial
        self.last_published: float | None = None

    def should_publish(self, new_e_max: float) -> bool:
        if self.last_published is None:
            return self.publish_initial
        return abs(new_e_max - self.last_published) > self.delta


def test_hysteresis_suppresses_small_changes() -> None:
    h = _HysteresisStub(delta=0.02)
    # Initial publish: always (publish_initial=True).
    assert h.should_publish(0.50) is True
    h.last_published = 0.50
    # Small change (< delta): suppressed.
    assert h.should_publish(0.505) is False
    # Large change (> delta): published.
    assert h.should_publish(0.60) is True


def test_hysteresis_skips_initial_when_publish_initial_false() -> None:
    h = _HysteresisStub(delta=0.02, publish_initial=False)
    assert h.should_publish(0.50) is False


# ---------------------------------------------------------------------------
# sample_path_window — nearest-waypoint anchor + walk-forward
# ---------------------------------------------------------------------------


def test_sample_path_window_empty_path_returns_empty() -> None:
    assert sample_path_window(_path([]), (0.0, 0.0), lookahead_m=3.0, step_m=0.1) == []


def test_sample_path_window_straight_line_emits_evenly_spaced() -> None:
    # Path along +x from (0,0) → (5,0). Robot at origin, 1.0 m window,
    # 0.25 m step. Expect samples at 0.0, 0.25, 0.50, 0.75, 1.0.
    path = _path([(x, 0.0) for x in np.linspace(0.0, 5.0, 51)])
    samples = sample_path_window(path, (0.0, 0.0), lookahead_m=1.0, step_m=0.25)
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    assert xs == pytest.approx([0.0, 0.25, 0.50, 0.75, 1.00], abs=1e-6)
    assert all(y == pytest.approx(0.0, abs=1e-9) for y in ys)


def test_sample_path_window_starts_at_nearest_waypoint() -> None:
    # Path along +x. Robot at (2.0, 0). Nearest waypoint is x=2.0 → samples
    # start there, not at the path origin.
    path = _path([(x, 0.0) for x in np.linspace(0.0, 5.0, 51)])
    samples = sample_path_window(path, (2.0, 0.0), lookahead_m=0.5, step_m=0.25)
    assert samples[0][0] == pytest.approx(2.0, abs=1e-6)
    # Subsequent samples walk forward, not backward.
    assert samples[-1][0] > samples[0][0]


def test_sample_path_window_diagonal_path_interpolates_correctly() -> None:
    # Two-waypoint path on the 45° diagonal: (0,0) → (10,10). 1 m window
    # at 0.5 m step from (0,0) → samples on the diagonal at distances
    # 0, 0.5, 1.0 (so XY = (0,0), (0.354,0.354), (0.707,0.707)).
    path = _path([(0.0, 0.0), (10.0, 10.0)])
    samples = sample_path_window(path, (0.0, 0.0), lookahead_m=1.0, step_m=0.5)
    expected = [
        (0.0, 0.0),
        (0.5 / math.sqrt(2), 0.5 / math.sqrt(2)),
        (1.0 / math.sqrt(2), 1.0 / math.sqrt(2)),
    ]
    assert len(samples) == 3
    for got, exp in zip(samples, expected, strict=False):
        assert got[0] == pytest.approx(exp[0], abs=1e-6)
        assert got[1] == pytest.approx(exp[1], abs=1e-6)


# ---------------------------------------------------------------------------
# min_clearance_along — pick the tightest pinch over a sample set
# ---------------------------------------------------------------------------


def test_min_clearance_along_picks_tightest_sample() -> None:
    # Obstacle at (1.0, 0.0). Sample one point next to it, one in open
    # space. min should reflect the close one.
    grid = _grid_with_obstacle_at((1.0, 0.0))
    g = gradient(grid, obstacle_threshold=50, max_distance=D_FAR)
    far_sample = (0.0, -2.0)  # well away from obstacle
    near_sample = (0.9, 0.0)  # 0.1 m from obstacle
    c_far = min_clearance_along(g, [far_sample], D_FAR)
    c_min = min_clearance_along(g, [far_sample, near_sample], D_FAR)
    assert c_far is not None and c_min is not None
    assert c_min < c_far
    assert c_min == pytest.approx(0.1, abs=0.05)  # ~one-cell precision


def test_min_clearance_along_all_outside_grid_returns_none() -> None:
    grid = _free_grid(side_m=5.0)
    g = gradient(grid, obstacle_threshold=50, max_distance=D_FAR)
    assert min_clearance_along(g, [(10.0, 10.0), (-10.0, -10.0)], D_FAR) is None


# ---------------------------------------------------------------------------
# compute_e_max_from_costmap with path-window — anticipates corners
# ---------------------------------------------------------------------------


def test_path_window_sees_obstacle_robot_doesnt() -> None:
    # Robot at origin, in open space relative to its own pose. Obstacle
    # 2 m ahead at (2.0, 0). Heading-based fallback (lookahead=0) thinks
    # we're in open space → high e_max. Path-window mode walks the path
    # forward, hits the obstacle within its 3 m lookahead, → low e_max.
    grid = _grid_with_obstacle_at((2.0, 0.0))
    path = _path([(x, 0.0) for x in np.linspace(0.0, 4.0, 41)])

    e_no_path = compute_e_max_from_costmap(grid, _pose(0.0, 0.0), **_kwargs(0.0))
    e_with_path = compute_e_max_from_costmap(
        grid,
        _pose(0.0, 0.0),
        **_kwargs(0.0),
        path=path,
        path_lookahead_m=3.0,
        path_sample_step_m=0.1,
    )
    assert e_no_path == pytest.approx(E_HIGH, abs=1e-3)
    assert e_with_path < e_no_path
    assert e_with_path == pytest.approx(E_LOW, abs=0.05)


def test_empty_path_falls_back_to_heading_lookahead() -> None:
    # Empty path → identical result to no-path call (heading-based).
    grid = _grid_with_obstacle_at((1.0, 0.0))
    e_no_path = compute_e_max_from_costmap(grid, _pose(0.0, 0.0, yaw=0.0), **_kwargs(0.5))
    e_empty_path = compute_e_max_from_costmap(
        grid,
        _pose(0.0, 0.0, yaw=0.0),
        **_kwargs(0.5),
        path=_path([]),
    )
    assert e_no_path == pytest.approx(e_empty_path, abs=1e-9)
