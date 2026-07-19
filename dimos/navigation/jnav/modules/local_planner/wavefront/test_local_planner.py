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

"""Hermetic tests for the wavefront local planner (pure core, no DimOS).

Scenario grids are built in cell units with a known resolution/origin so the
asserts can talk in world metres. Each test paints obstacles into the costmap,
plans, and checks a concrete property of the resulting pose list.
"""

from __future__ import annotations

import math

import numpy as np

from dimos.navigation.jnav.modules.local_planner.wavefront.local_planner import (
    WavefrontParams,
    plan_path,
)

RES = 0.1  # m/cell
ORIGIN = (0.0, 0.0)


def _blank(width: int, height: int) -> np.ndarray:
    """Free (cost 0) grid, shape (height, width)."""
    return np.zeros((height, width), dtype=np.float64)


def _cost_at(cost: np.ndarray, x: float, y: float) -> float:
    col = round((x - ORIGIN[0]) / RES)
    row = round((y - ORIGIN[1]) / RES)
    row = min(max(row, 0), cost.shape[0] - 1)
    col = min(max(col, 0), cost.shape[1] - 1)
    return float(cost[row, col])


def _straight_path(start: tuple[float, float], goal: tuple[float, float], n: int = 40):
    return [
        (start[0] + (goal[0] - start[0]) * t, start[1] + (goal[1] - start[1]) * t)
        for t in np.linspace(0.0, 1.0, n)
    ]


def _reaches(poses, goal, tol=0.3) -> bool:
    return bool(math.hypot(poses[-1][0] - goal[0], poses[-1][1] - goal[1]) <= tol)


def test_clear_costmap_tracks_global_path():
    """With no obstacles the local path should hug the (straight) global path."""
    cost = _blank(60, 40)
    start, goal = (0.5, 2.0), (5.0, 2.0)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0))
    assert poses, "planner returned an empty path"
    assert _reaches(poses, goal), f"did not reach goal: {poses[-1]}"
    max_dev = max(abs(y - 2.0) for _, y, _ in poses)
    assert max_dev < 0.2, f"strayed from the straight path by {max_dev:.2f} m"


def test_wall_detours_and_stays_off_obstacles():
    """A wall across the straight path -> detour, and every pose stays clear."""
    cost = _blank(60, 50)
    # Vertical wall at x ~ 2.5 m with a gap nowhere (full block from y=1.0..3.0),
    # leaving room to go around above (y > 3.2) or below (y < 0.8).
    wall_col = int(2.5 / RES)
    cost[int(0.8 / RES) : int(3.2 / RES), wall_col - 1 : wall_col + 2] = 100
    start, goal = (0.5, 2.0), (4.5, 2.0)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0))
    assert poses, "planner returned an empty path"
    # Must actually go around: some pose leaves the y≈2.0 corridor.
    assert max(abs(y - 2.0) for _, y, _ in poses) > 0.5, "never detoured around the wall"
    # No pose may sit on a lethal cell.
    worst = max(_cost_at(cost, x, y) for x, y, _ in poses)
    assert worst < 50, f"a pose landed on an obstacle (cost {worst})"
    assert _reaches(poses, goal, tol=0.4), f"did not reach goal: {poses[-1]}"


def test_u_trap_makes_progress_no_permanent_stall():
    """A U-shaped trap opening away from the goal must not park the robot.

    The classic APF local-minimum: a pocket whose mouth faces the start. The
    wavefront navigation function has no minimum but the goal, so the planner
    routes straight out of the cup and around rather than stalling inside it.
    """
    cost = _blank(80, 60)
    # U opening to the left (toward start); goal is to the right past the cup.
    top = int(4.0 / RES)
    bot = int(2.0 / RES)
    right = int(4.5 / RES)
    left = int(2.5 / RES)
    cost[bot:top, right - 1 : right + 1] = 100  # back wall
    cost[top - 1 : top + 1, left:right] = 100  # top wall
    cost[bot - 1 : bot + 1, left:right] = 100  # bottom wall
    start, goal = (1.0, 3.0), (6.5, 3.0)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0))
    assert poses, "planner returned an empty path"
    # Net progress toward the goal: end must be closer than start.
    start_d = math.hypot(start[0] - goal[0], start[1] - goal[1])
    end_d = math.hypot(poses[-1][0] - goal[0], poses[-1][1] - goal[1])
    assert end_d < start_d - 0.5, f"no progress out of the trap ({start_d:.2f}->{end_d:.2f})"
    # And it never sat on the obstacle.
    worst = max(_cost_at(cost, x, y) for x, y, _ in poses)
    assert worst < 50, f"a pose landed on an obstacle (cost {worst})"


def test_no_oscillation_path_is_smooth():
    """Successive heading changes stay bounded -> no flip-flopping."""
    cost = _blank(60, 50)
    wall_col = int(2.5 / RES)
    cost[int(0.8 / RES) : int(3.2 / RES), wall_col - 1 : wall_col + 2] = 100
    start, goal = (0.5, 2.0), (4.5, 2.0)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0))
    # Heading change between consecutive segments should rarely be sharp.
    headings = []
    for i in range(len(poses) - 1):
        dx = poses[i + 1][0] - poses[i][0]
        dy = poses[i + 1][1] - poses[i][1]
        if math.hypot(dx, dy) > 1e-6:
            headings.append(math.atan2(dy, dx))
    # A genuine smooth go-around turns gradually (each 5cm step barely changes
    # heading); oscillation is near-180deg reversals between consecutive steps.
    flips = 0
    for i in range(len(headings) - 1):
        d = abs(
            math.atan2(
                math.sin(headings[i + 1] - headings[i]), math.cos(headings[i + 1] - headings[i])
            )
        )
        if d > math.radians(120):
            flips += 1
    assert flips <= 1, f"path oscillates: {flips} near-reversals between steps"


def test_face_forward_yaw_matches_travel_direction():
    """face_forward_weight≈1 -> each pose yaw ≈ its travel direction."""
    cost = _blank(60, 30)
    start, goal = (0.5, 1.5), (5.0, 1.5)
    params = WavefrontParams(face_forward_weight=1.0, omnidirectional=False)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0), params)
    assert len(poses) > 3
    # Straight east path -> yaw ≈ 0 for interior poses.
    for x, y, yaw in poses[2:-2]:
        assert abs(math.atan2(math.sin(yaw), math.cos(yaw))) < math.radians(15), (
            f"pose at ({x:.2f},{y:.2f}) faces {math.degrees(yaw):.0f}deg, not forward"
        )


def test_omnidirectional_faces_goal_not_travel():
    """omnidirectional=True -> yaw points at the goal even when travelling sideways."""
    cost = _blank(40, 80)
    # Travel mostly north (+y); goal is north so goal-dir ~ +90deg while travelling.
    start, goal = (2.0, 0.5), (2.0, 6.0)
    params = WavefrontParams(omnidirectional=True)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0), params)
    assert len(poses) > 3
    for x, y, yaw in poses[1:-2]:
        goal_yaw = math.atan2(goal[1] - y, goal[0] - x)
        d = abs(math.atan2(math.sin(yaw - goal_yaw), math.cos(yaw - goal_yaw)))
        assert d < math.radians(15), f"omni pose faces {math.degrees(yaw):.0f}, not the goal"


def _min_clearance(cost: np.ndarray, poses) -> float:
    """Smallest distance (m) from any pose to a lethal cell."""
    obs = np.argwhere(cost >= 50).astype(float) * RES  # (row=y, col=x)
    if obs.size == 0:
        return math.inf
    return min(float(np.min(np.hypot(obs[:, 1] - x, obs[:, 0] - y))) for x, y, _ in poses)


def _gapped_wall() -> tuple[np.ndarray, tuple, tuple]:
    """Full-height wall at x≈4 m with a single 0.4 m gap; start left, goal right."""
    cost = _blank(80, 60)
    cost[:, 38:42] = 100  # vertical wall cols 38..41 (x 3.8..4.1), full height
    cost[28:32, 38:42] = 0  # gap rows 28..31 (y 2.8..3.1) -> 0.4 m tall
    return cost, (1.0, 3.0), (7.0, 3.0)


def test_vehicle_fits_through_wide_enough_gap():
    """A narrow robot passes through a 0.4 m gap and keeps >= its body radius clear."""
    cost, start, goal = _gapped_wall()
    params = WavefrontParams(vehicle_width=0.2)  # radius 0.1 m
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0), params)
    assert poses, "narrow vehicle should find a path through the gap"
    assert _reaches(poses, goal, tol=0.4), f"did not reach goal: {poses[-1]}"
    # Body never overlaps a wall: centre stays at least ~one radius away.
    assert _min_clearance(cost, poses) >= 0.1 - 1e-6, "vehicle clipped the gap walls"


def test_vehicle_too_wide_for_gap_does_not_pass_through():
    """A robot wider than the gap can't squeeze through -> it stops on the near
    side (best effort), never clipping the walls and never reaching the far side."""
    cost, start, goal = _gapped_wall()
    params = WavefrontParams(vehicle_width=0.6)  # needs 0.6 m, gap is 0.4 m
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0), params)
    assert poses, "best effort should still drive the robot up to the wall"
    assert not _reaches(poses, goal, tol=0.5), "a too-wide vehicle must not pass the gap"
    # Stays on the start side of the wall (wall at x≈3.8-4.1 m) and stays clear.
    assert max(x for x, _, _ in poses) < 3.8, "vehicle crossed a gap it cannot fit"
    assert _min_clearance(cost, poses) >= 0.3 - 1e-6, "vehicle clipped the wall"


def test_vehicle_width_keeps_clearance_from_a_wall():
    """Wider vehicle -> path bows further from an obstacle (more clearance)."""
    cost = _blank(70, 50)
    cost[int(0.8 / RES) : int(3.2 / RES), 24:27] = 100  # wall across the straight path
    start, goal = (0.5, 2.0), (4.5, 2.0)
    sp = _straight_path(start, goal)
    narrow = plan_path(
        cost, RES, ORIGIN, sp, (*start, 0.0), WavefrontParams(vehicle_width=0.2)
    )
    wide = plan_path(cost, RES, ORIGIN, sp, (*start, 0.0), WavefrontParams(vehicle_width=0.8))
    assert narrow and wide
    assert _min_clearance(cost, wide) >= 0.4 - 1e-6, "wide vehicle came too close to the wall"
    assert _min_clearance(cost, wide) > _min_clearance(cost, narrow), (
        "wider vehicle should keep more clearance than a narrow one"
    )


def test_walled_off_goal_is_best_effort_not_frozen():
    """A goal boxed in by walls -> the robot drives to the closest reachable
    point (makes progress) instead of freezing, and never touches an obstacle."""
    cost = _blank(90, 60)
    # Box around the goal at (4.5, 3.0), open nowhere.
    cost[22:40, 38:40] = 100  # left wall
    cost[22:40, 52:54] = 100  # right wall
    cost[22:24, 38:54] = 100  # bottom wall
    cost[38:40, 38:54] = 100  # top wall
    start, goal = (0.5, 3.0), (4.5, 3.0)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0))
    assert poses, "should make a best-effort approach, not return nothing"
    start_d = math.hypot(start[0] - goal[0], start[1] - goal[1])
    end_d = math.hypot(poses[-1][0] - goal[0], poses[-1][1] - goal[1])
    assert end_d < start_d - 0.5, "best effort should get meaningfully closer to the goal"
    assert not _reaches(poses, goal, tol=0.3), "the goal is walled off; must not 'reach' it"
    assert max(_cost_at(cost, x, y) for x, y, _ in poses) < 50, "a pose hit the box wall"


def test_goal_inside_obstacle_ends_off_the_obstacle():
    """A goal sitting inside an obstacle -> the path ends at the nearest safe cell,
    never with a pose on a lethal cell."""
    cost = _blank(90, 60)
    cost[24:36, 46:54] = 100  # block; goal is inside it
    start, goal = (0.5, 3.0), (5.0, 3.0)
    poses = plan_path(cost, RES, ORIGIN, _straight_path(start, goal), (*start, 0.0))
    assert poses
    assert max(_cost_at(cost, x, y) for x, y, _ in poses) < 50, "final pose sits in the obstacle"
    assert _min_clearance(cost, poses) >= 0.25 - 1e-6, "path comes inside the body radius"


# --- wp4 mechanism: follow the global route, don't beeline at a far goal ------
# wp4's room is sealed on the side facing the robot; its only door is a long loop
# away and is *not* in the local costmap window, so the room is a disconnected
# island. The global planner (MLS) routes through the door, so the local planner
# must *follow that route* rather than aim at the goal and stall at the near wall
# (the observed "stalls 1.68 m short" failure).
def test_follows_global_route_when_goal_region_is_sealed_in_costmap():
    """Goal region sealed in this costmap; route heads west toward an off-window
    door. The planner must follow the route (head west), not beeline north into
    the sealing wall and stall."""
    cost = _blank(100, 70)  # 10 x 7 m
    cost[40:42, 0:100] = 100  # full-width wall at y≈4.0 -> room above is an island
    robot = (6.0, 2.0, 0.0)
    # MLS route: west along the open corridor to the (off-window) door, then up
    # into the room to the goal. Only the westward corridor leg is in-costmap.
    route = [(6.0, 2.0), (3.0, 2.0), (0.3, 2.0), (0.3, 5.5), (6.0, 5.5)]
    poses = plan_path(cost, RES, ORIGIN, route, robot)
    assert poses, "should follow the route, not freeze"
    end = poses[-1]
    assert end[0] < 4.0, f"did not follow the route west (ended at {end[:2]}); beelined the goal"
    assert max(y for _, y, _ in poses) < 4.0, "crossed the sealing wall toward the goal"
    assert max(_cost_at(cost, x, y) for x, y, _ in poses) < 50, "a pose hit the wall"


def test_reaches_goal_via_in_costmap_route_loop():
    """Goal reachable only by looping around a barrier; the loop is in the costmap
    and the route describes it -> the planner follows it all the way to the goal."""
    cost = _blank(100, 70)
    cost[30:70, 48:52] = 100  # barrier x≈4.8-5.2 from y≈3.0 up; open below y≈3.0
    robot = (1.0, 5.0, 0.0)
    goal = (9.0, 5.0)
    route = [(1.0, 5.0), (1.0, 1.5), (9.0, 1.5), (9.0, 5.0)]  # down, across, up
    poses = plan_path(cost, RES, ORIGIN, route, robot)
    assert poses and _reaches(poses, goal, tol=0.5), f"did not reach via the loop: {poses[-1]}"
    assert max(_cost_at(cost, x, y) for x, y, _ in poses) < 50, "a pose hit the barrier"


# --- wp4 regression: bounded carrot must not shortcut a loop through a MISSING wall -
# The wp4 failure that survived every costmap fix: the room's north wall is occluded,
# so it is *absent* from the local costmap (the corridor between robot and goal reads
# free), while the global route still loops ~20 m to the far south door. An unbounded
# carrot ("furthest reachable point") then jumps to the goal — reachable by a straight
# shortcut across the (physically real but un-mapped) wall — and the robot beelines into
# it and stalls. carrot_lookahead caps the carrot to a few metres of route arc-length so
# the planner follows the loop toward the door instead. See test_cross_wall_dim_city.
def test_unbounded_carrot_shortcuts_a_loop_to_a_near_goal():
    """Baseline (no cap): goal is ~1.5 m away in a straight line but ~18 m along the
    loop, and the straight line is free in the costmap -> the planner beelines it."""
    cost = _blank(120, 90)  # 12 x 9 m, all free (the wall is missing from the costmap)
    robot = (5.0, 6.5, -math.pi / 2)
    # Loop route: west, south to a far door, back east+north to the goal, which sits
    # just ~1.5 m south of the robot (a straight, in-costmap shortcut).
    route = [
        (5.0, 6.5), (3.0, 7.0), (1.0, 7.0), (1.0, 2.0), (3.0, 1.0),
        (5.0, 1.0), (7.0, 1.0), (7.0, 4.0), (5.2, 5.0),
    ]
    goal = route[-1]
    poses = plan_path(cost, RES, ORIGIN, route, robot)  # carrot_lookahead defaults to 0
    # Unbounded: pins the goal and takes the ~1.5 m straight shortcut (short path).
    assert _reaches(poses, goal, tol=0.4), f"unbounded should beeline the goal: {poses[-1]}"
    assert len(poses) < 40, "unbounded should be a short beeline, not the long loop"


def test_carrot_lookahead_follows_the_loop_instead_of_shortcutting():
    """The fix: with a bounded carrot the planner follows the route's outbound leg
    (west/south toward the far door) rather than shortcutting straight to the goal."""
    cost = _blank(120, 90)
    robot = (5.0, 6.5, -math.pi / 2)
    route = [
        (5.0, 6.5), (3.0, 7.0), (1.0, 7.0), (1.0, 2.0), (3.0, 1.0),
        (5.0, 1.0), (7.0, 1.0), (7.0, 4.0), (5.2, 5.0),
    ]
    params = WavefrontParams(carrot_lookahead=4.0)
    poses = plan_path(cost, RES, ORIGIN, route, robot, params)
    assert poses, "should follow the route, not freeze"
    end = poses[-1]
    # The carrot sits on the outbound (west) leg, not at the near goal to the south.
    assert end[0] < 4.0, f"did not follow the route west (ended at {end[:2]}); shortcut the goal"
    assert end[1] > 6.0, f"headed south toward the near goal instead of west: {end[:2]}"
