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

"""Closed-loop dynamic-obstacle tests for the wavefront local planner.

Each test runs a small simulation: a moving obstacle is painted into a fresh
costmap every tick, the planner replans, and the robot advances along the new
local path. We then assert the run was collision-free (the robot never came
within its body radius of an obstacle), that it rerouted when blocked, that it
did not oscillate, and that it recovered to the goal once clear. Pure Python +
numpy, no sim/hardware/network.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from dimos.navigation.nav_3d.wavefront_local_planner.local_planner import (
    WavefrontParams,
    plan_path,
)

RES = 0.1
ORIGIN = (0.0, 0.0)


@dataclass
class Box:
    """An axis-aligned obstacle box, centre (x, y), half-extents (hx, hy)."""

    x: float
    y: float
    hx: float = 0.3
    hy: float = 0.3

    def paint(self, cost: np.ndarray) -> None:
        c0 = round((self.x - self.hx) / RES)
        c1 = round((self.x + self.hx) / RES)
        r0 = round((self.y - self.hy) / RES)
        r1 = round((self.y + self.hy) / RES)
        r0, r1 = max(r0, 0), min(r1, cost.shape[0])
        c0, c1 = max(c0, 0), min(c1, cost.shape[1])
        cost[r0:r1, c0:c1] = 100

    def clearance(self, x: float, y: float) -> float:
        """Distance from world point (x, y) to this box (0 if inside)."""
        dx = max(self.x - self.hx - x, 0.0, x - (self.x + self.hx))
        dy = max(self.y - self.hy - y, 0.0, y - (self.y + self.hy))
        return math.hypot(dx, dy)


@dataclass
class DynResult:
    poses: list[tuple[float, float, float]]  # robot trajectory (x, y, yaw)
    min_clearance: float
    reached: bool
    max_lateral: float  # furthest the robot strayed from the straight goal line
    heading_reversals: int  # tick-to-tick near-reversals (oscillation metric)
    stalls: int  # ticks where the planner returned no usable path
    clearances: list[float] = field(default_factory=list)


def _follow(poses: list[tuple[float, float, float]], step: float) -> tuple[float, float, float]:
    """Point a distance ``step`` along the local path from its start."""
    if len(poses) < 2:
        return poses[0] if poses else (0.0, 0.0, 0.0)
    acc = 0.0
    prev = poses[0]
    for cur in poses[1:]:
        seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        if acc + seg >= step and seg > 1e-9:
            r = (step - acc) / seg
            x = prev[0] + r * (cur[0] - prev[0])
            y = prev[1] + r * (cur[1] - prev[1])
            return x, y, math.atan2(cur[1] - prev[1], cur[0] - prev[0])
        acc += seg
        prev = cur
    return poses[-1]


def simulate_dynamic(
    grid_shape: tuple[int, int],
    static_walls,
    obstacle_at,  # tick -> list[Box]
    start: tuple[float, float],
    goal: tuple[float, float],
    *,
    ticks: int = 120,
    robot_step: float = 0.12,
    params: WavefrontParams | None = None,
    global_path=None,
) -> DynResult:
    """Run the closed loop and collect safety/progress metrics."""
    params = params or WavefrontParams()
    height, width = grid_shape
    base = np.zeros((height, width), dtype=np.float64)
    for w in static_walls:
        w.paint(base)
    if global_path is None:
        global_path = [
            (start[0] + (goal[0] - start[0]) * t, start[1] + (goal[1] - start[1]) * t)
            for t in np.linspace(0.0, 1.0, 40)
        ]
    line = np.asarray(goal) - np.asarray(start)
    line_len = float(np.hypot(*line)) or 1.0
    line_dir = line / line_len

    robot = (start[0], start[1], math.atan2(line_dir[1], line_dir[0]))
    traj = [robot]
    clearances: list[float] = []
    reversals = 0
    stalls = 0
    prev_heading: float | None = None
    prev_local: list[tuple[float, float]] | None = None
    max_lateral = 0.0
    reached = False

    for t in range(ticks):
        cost = base.copy()
        boxes = obstacle_at(t)
        for b in boxes:
            b.paint(cost)
        poses = plan_path(cost, RES, ORIGIN, global_path, robot, params, previous_path=prev_local)
        prev_local = [(x, y) for x, y, _ in poses] if len(poses) >= 2 else None
        # clearance of the robot's *current* position to every obstacle this tick
        clr = min((b.clearance(robot[0], robot[1]) for b in boxes), default=math.inf)
        clearances.append(clr)
        if len(poses) < 2:
            stalls += 1
        else:
            nxt = _follow(poses, robot_step)
            heading = math.atan2(nxt[1] - robot[1], nxt[0] - robot[0])
            if prev_heading is not None:
                d = abs(
                    math.atan2(math.sin(heading - prev_heading), math.cos(heading - prev_heading))
                )
                if d > math.radians(120):
                    reversals += 1
            prev_heading = heading
            robot = nxt
            traj.append(robot)
        # lateral deviation from the straight start->goal line
        rel = np.asarray(robot[:2]) - np.asarray(start)
        max_lateral = max(max_lateral, abs(float(rel[0] * line_dir[1] - rel[1] * line_dir[0])))
        if math.hypot(robot[0] - goal[0], robot[1] - goal[1]) < 0.4:
            reached = True
            break

    return DynResult(
        poses=traj,
        min_clearance=min(clearances) if clearances else math.inf,
        reached=reached,
        max_lateral=max_lateral,
        heading_reversals=reversals,
        stalls=stalls,
        clearances=clearances,
    )


RADIUS = 0.25  # default vehicle_width 0.5 -> body radius


def _assert_safe(res: DynResult, name: str, *, expect_reach: bool = True) -> None:
    assert res.min_clearance >= RADIUS - 0.03, (
        f"{name}: robot came within {res.min_clearance:.2f} m of an obstacle (< body radius)"
    )
    assert res.heading_reversals <= 3, f"{name}: oscillated ({res.heading_reversals} reversals)"
    if expect_reach:
        assert res.reached, f"{name}: never reached the goal"


def test_obstacle_crossing_the_path():
    """A box patrols vertically across the robot's straight path."""

    def at(t):
        y = 1.0 + 5.0 * (0.5 - 0.5 * math.cos(t * 0.08))  # sweeps 1..6 and back
        return [Box(6.0, y)]

    res = simulate_dynamic((90, 130), [], at, (1.0, 3.5), (11.5, 3.5))
    _assert_safe(res, "crossing")


def test_obstacle_blocks_then_clears():
    """A box sits on the path, then moves off it; the robot detours then rejoins."""

    def at(t):
        y = 3.5 if t < 45 else 3.5 + min((t - 45) * 0.1, 3.0)  # parks on path, then leaves
        return [Box(6.0, y)]

    res = simulate_dynamic((90, 130), [], at, (1.0, 3.5), (11.5, 3.5))
    _assert_safe(res, "blocks_then_clears")
    assert res.max_lateral > 0.4, "should have detoured around the parked obstacle"


def test_obstacle_head_on():
    """A box drives toward the robot along the path; the robot must sidestep."""

    def at(t):
        x = max(11.0 - t * 0.13, 1.0)
        return [Box(x, 3.5)]

    res = simulate_dynamic((90, 130), [], at, (1.0, 3.5), (11.5, 3.5))
    _assert_safe(res, "head_on")
    assert res.max_lateral > 0.3, "should have sidestepped the oncoming obstacle"


def test_two_obstacles_in_a_corridor():
    """Two boxes patrol inside a corridor the robot must traverse."""
    walls = [Box(6.5, 0.9, hx=6.5, hy=0.9), Box(6.5, 6.1, hx=6.5, hy=0.9)]  # top/bottom walls

    def at(t):
        y1 = 2.4 + 1.2 * math.sin(t * 0.09)
        y2 = 4.6 - 1.2 * math.sin(t * 0.11 + 1.0)
        return [Box(4.5, y1, hx=0.25, hy=0.25), Box(8.5, y2, hx=0.25, hy=0.25)]

    res = simulate_dynamic((70, 130), walls, at, (0.6, 3.5), (12.0, 3.5), ticks=160)
    _assert_safe(res, "corridor")


# --- refinements found via the dynamic tests --------------------------------
def _wiggle(t):
    """Two boxes flanking the path, wiggling fast in opposite phase — the route's
    best side flips every tick, baiting a stateless planner into oscillating."""
    return [Box(5.0, 3.5 + 0.9 * math.sin(t * 0.4)), Box(7.5, 3.5 - 0.9 * math.sin(t * 0.4 + 1.0))]


def test_commitment_weight_stops_oscillation_under_wiggling_obstacles():
    """Without temporal commitment the planner flip-flops (and clips an obstacle);
    with it, the route stays committed — no oscillation and collision-free."""
    base = simulate_dynamic((90, 130), [], _wiggle, (1.0, 3.5), (11.5, 3.5), ticks=140)
    committed = simulate_dynamic(
        (90, 130),
        [],
        _wiggle,
        (1.0, 3.5),
        (11.5, 3.5),
        ticks=140,
        params=WavefrontParams(commitment_weight=3.0),
    )
    # Baseline genuinely misbehaves (this is the bug the refinement targets).
    assert base.heading_reversals >= 4, "expected the baseline to oscillate here"
    # Commitment fixes it.
    assert committed.heading_reversals <= 1, (
        f"commitment did not stop oscillation ({committed.heading_reversals} reversals)"
    )
    assert committed.min_clearance >= RADIUS - 0.05, "commitment run clipped an obstacle"
    assert committed.reached, "commitment run did not reach the goal"


def test_safety_margin_increases_clearance():
    """A static box partly blocks the path; a safety margin makes the planner bow
    further around it, so the robot keeps measurably more clearance."""

    def blocker(_t):
        return [Box(6.0, 3.0, hx=0.35, hy=0.9)]  # straddles the y=3.5 path from below

    plain = simulate_dynamic((90, 130), [], blocker, (1.0, 3.5), (11.5, 3.5))
    margin = simulate_dynamic(
        (90, 130),
        [],
        blocker,
        (1.0, 3.5),
        (11.5, 3.5),
        params=WavefrontParams(safety_margin=0.25),
    )
    assert plain.reached and margin.reached
    assert margin.min_clearance > plain.min_clearance + 0.1, (
        f"safety margin did not add clearance (plain {plain.min_clearance:.2f} "
        f"-> margin {margin.min_clearance:.2f})"
    )
