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

"""Constant-length local path: route-tail extension, carrot gap tolerance, and the
solve adoption gate.

The course recordings showed the local path length oscillating 0.3-5.9 m: it pinned
(and shrank) at every intermediate leg goal, and single-cell reachability flicker
collapsed the carrot to under a metre for a tick. The path should hold its (speed-
scaled) horizon everywhere except approaching the route's TRUE final goal.
"""

from __future__ import annotations

import math
import time
from types import SimpleNamespace

import numpy as np

from dimos.navigation.jnav.modules.local_planner.wavefront.local_planner import (
    WavefrontLocalPlanner,
    WavefrontLocalPlannerConfig,
    WavefrontParams,
    _obstacle_distance,
    plan_path,
)

RES = 0.1
ORIGIN = (0.0, 0.0)


def _free_grid(width: int, height: int) -> np.ndarray:
    return np.zeros((height, width), dtype=np.float64)


def _straight_path(a, b, n=160):
    return [
        (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        for t in np.linspace(0.0, 1.0, n)
    ]


def _arc(plan) -> float:
    return float(
        sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(plan, plan[1:]))
    )


def _make_state(cost, path, cfg, tail=()):
    params = cfg.to_params()
    clipped = np.clip(np.nan_to_num(cost, nan=0.0), 0, 100).astype(np.float64)
    obstacle_dist = _obstacle_distance(clipped, RES, params)
    now = time.monotonic()
    slf = SimpleNamespace(
        _plan=None,
        _plan_dirty=True,
        _obstacle_dist=obstacle_dist,
        _path_world=path,
        _route_tail=list(tail),
        _origin=ORIGIN,
        _resolution=RES,
        _cost=clipped,
        _prev_local=None,
        _best_remaining_arc=float("inf"),
        _last_progress_monotonic=now,
        _last_solve_mono=now,
        _costmap_generation=0,
        _blocked_costmap_id=None,
        _horizon_boost=0.0,
        config=cfg,
    )
    return slf, params


# --- route tail: no shrink at intermediate goals -------------------------------


def test_tail_keeps_full_horizon_through_an_intermediate_goal():
    """Robot 1 m short of the leg goal: without a tail the plan pins at the goal
    (~1 m long); with the next leg's markers appended it holds the full horizon."""
    cost = _free_grid(300, 40)  # 30 x 4 m free corridor
    path = _straight_path((1.0, 2.0), (10.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    robot = (9.0, 2.0, 0.0)

    slf, params = _make_state(cost, path, cfg)
    bare = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist
    )
    assert _arc(bare) < 1.5  # pinned at the leg goal

    slf, params = _make_state(cost, path, cfg, tail=[(20.0, 2.0), (28.0, 2.0)])
    extended = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist,
        carrot_extension=WavefrontLocalPlanner._carrot_extension(slf),
    )
    assert _arc(extended) > 0.8 * params.horizon  # holds the horizon through the goal


def test_no_tail_still_shrinks_at_the_true_final_goal():
    """Terminal semantics unchanged: with an empty tail the plan still ends at the
    route's final goal (precise arrival)."""
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (10.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    slf, params = _make_state(cost, path, cfg)
    robot = (9.0, 2.0, 0.0)
    plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist,
        carrot_extension=WavefrontLocalPlanner._carrot_extension(slf) or None,
    )
    end = plan[-1]
    assert math.hypot(end[0] - 10.0, end[1] - 2.0) < 0.2


def test_tail_through_a_wall_is_capped_by_reachability_not_hopped():
    """A tail whose straight segment crosses a wall must not drag the carrot through
    it: the plan extends toward the wall and stops, never past it."""
    cost = _free_grid(300, 40)
    cost[:, int(12.0 / RES):int(14.0 / RES)] = 100  # 2 m thick wall at x=12..14
    path = _straight_path((1.0, 2.0), (10.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=6.0, carrot_lookahead=6.0)
    slf, params = _make_state(cost, path, cfg, tail=[(20.0, 2.0)])
    robot = (9.0, 2.0, 0.0)
    plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist,
        carrot_extension=WavefrontLocalPlanner._carrot_extension(slf),
    )
    assert plan
    assert max(p[0] for p in plan) < 12.0  # never inside or past the wall


# --- carrot gap tolerance -------------------------------------------------------


def test_single_cell_flicker_no_longer_collapses_the_carrot():
    """One unreachable cell on the route (reachability flicker) used to truncate the
    carrot — and the whole plan — right there. With carrot_gap_max it is hopped."""
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    robot = (5.0, 2.0, 0.0)
    # A single lethal cell ON the route 1 m ahead (row 20 = y 2.0, col 60 = x 6.0):
    # the 4 m corridor around it stays free, so the Dijkstra flows around it.
    cost[20, 60] = 100

    hard = WavefrontParams(carrot_lookahead=4.0, horizon=3.0, carrot_gap_max=0.0)
    tolerant = WavefrontParams(carrot_lookahead=4.0, horizon=3.0, carrot_gap_max=1.0)
    short = plan_path(cost, RES, ORIGIN, path, robot, hard)
    long = plan_path(cost, RES, ORIGIN, path, robot, tolerant)
    assert _arc(short) < 1.5  # the old hard break: collapsed at the sliver
    assert _arc(long) > 2.0  # gap hopped; full horizon retained


def test_gap_cap_still_stops_at_a_real_wall():
    """A wall spanning well over carrot_gap_max of route arc must still stop the
    carrot scan — the tolerance must not re-introduce the beeline-through-walls bug."""
    cost = _free_grid(300, 40)
    cost[:, int(8.0 / RES):int(10.0 / RES)] = 100  # 2 m thick wall across the route
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    robot = (5.0, 2.0, 0.0)
    tolerant = WavefrontParams(carrot_lookahead=8.0, horizon=8.0, carrot_gap_max=1.0)
    plan = plan_path(cost, RES, ORIGIN, path, robot, tolerant)
    assert plan
    assert max(p[0] for p in plan) < 8.0  # stopped at the wall


# --- solve adoption gate ---------------------------------------------------------


def _gate(slf, new_plan, robot, reason):
    slf._initial_direction = WavefrontLocalPlanner._initial_direction  # staticmethod
    if not hasattr(slf, "_gate_rejections"):
        slf._gate_rejections = 0
    return WavefrontLocalPlanner._adopt_solve(slf, new_plan, robot, reason)


def test_routine_short_solve_is_rejected_keeping_the_committed_plan():
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    slf, params = _make_state(cost, path, cfg)
    robot = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist
    )
    assert _arc(slf._plan) > 2.0
    collapsed = [(5.0, 2.0, 0.0), (5.4, 2.0, 0.0)]  # a 0.4 m transient solve
    assert _gate(slf, collapsed, robot, "periodic") is False
    assert _gate(slf, collapsed, robot, "horizon") is False


def test_event_replans_always_adopt_even_a_short_solve():
    """A confirmed blockage / new route / drift means the committed plan is bad —
    a short fresh solve is still the best available and must be adopted."""
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    slf, params = _make_state(cost, path, cfg)
    robot = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist
    )
    collapsed = [(5.0, 2.0, 0.0), (5.4, 2.0, 0.0)]
    for reason in ("dirty", "blocked", "deviation", "no_progress", "goal"):
        assert _gate(slf, collapsed, robot, reason) is True


def test_comparable_routine_solve_is_adopted():
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    slf, params = _make_state(cost, path, cfg)
    robot = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist
    )
    fresh = plan_path(
        slf._cost, RES, ORIGIN, path, (5.5, 2.0, 0.0), params, obstacle_dist=slf._obstacle_dist
    )
    assert _gate(slf, fresh, (5.5, 2.0, 0.0), "periodic") is True


# --- periodic replan trigger ------------------------------------------------------


def test_periodic_trigger_fires_after_the_period_and_not_before():
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(
        horizon=3.0, replan_period_s=0.15, no_progress_replan_s=0.0
    )
    slf, params = _make_state(cost, path, cfg)
    robot = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist
    )
    slf._plan_dirty = False
    # The setup solve above can outlast the 0.15 s period on a loaded machine
    # (flaked while a sim run shared the box) — the trigger clock starts NOW.
    slf._last_solve_mono = time.monotonic()
    effective = slf._path_world + WavefrontLocalPlanner._carrot_extension(slf)
    assert (
        WavefrontLocalPlanner._plan_needs_replan(slf, robot, params, effective) is None
    )
    time.sleep(0.2)
    assert (
        WavefrontLocalPlanner._plan_needs_replan(slf, robot, params, effective)
        == "periodic"
    )


# --- tail reversal trim ----------------------------------------------------------


def test_tail_trims_at_a_sharp_reversal():
    """A tail whose next leg REVERSES (the top-of-stairs -> back-down case) must
    not extend the carrot: hooking the effective path back toward the robot made
    the follower cut toward the next leg before reaching the waypoint (recorded
    as repeated double-backs on the last few stairs)."""
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (10.0, 2.0))  # approach heading +x
    cfg = WavefrontLocalPlannerConfig(horizon=3.0, tail_reversal_trim_deg=100.0)
    slf, params = _make_state(cost, path, cfg, tail=[(2.0, 2.0)])  # next leg: straight back
    assert WavefrontLocalPlanner._carrot_extension(slf) == []
    # ... and the plan pins at the leg goal exactly like a terminal one.
    robot = (9.0, 2.0, 0.0)
    plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist,
        carrot_extension=WavefrontLocalPlanner._carrot_extension(slf) or None,
    )
    end = plan[-1]
    assert math.hypot(end[0] - 10.0, end[1] - 2.0) < 0.2


def test_tail_keeps_gentle_corners():
    """A 90-degree next leg is arcable (below the follower's rotate-in-place
    cutoff and below tail_reversal_trim_deg) — the tail must survive so the
    path keeps flowing through the waypoint."""
    cost = _free_grid(300, 300)
    path = _straight_path((1.0, 2.0), (10.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    slf, params = _make_state(cost, path, cfg, tail=[(10.0, 12.0)])  # next leg: +y (90 deg)
    ext = WavefrontLocalPlanner._carrot_extension(slf)
    assert len(ext) > 10  # densified tail survives
    robot = (9.0, 2.0, 0.0)
    plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist,
        carrot_extension=ext,
    )
    assert _arc(plan) > 0.8 * params.horizon  # flows through the corner


def test_tail_trims_at_a_later_reversal():
    """bot -> top -> bot again: the first tail leg continues, the second reverses;
    the extension must keep the first and cut at the second."""
    cost = _free_grid(300, 300)
    path = _straight_path((1.0, 2.0), (10.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0, tail_reversal_trim_deg=100.0)
    slf, _ = _make_state(cost, path, cfg, tail=[(14.0, 2.0), (6.0, 2.0)])
    ext = WavefrontLocalPlanner._carrot_extension(slf)
    assert ext, "first (straight-on) tail leg must survive"
    assert max(p[0] for p in ext) <= 14.0 + 1e-6
    assert all(p[0] >= 9.9 for p in ext), "nothing behind the reversal may remain"


# --- same-goal alternative-route debounce ----------------------------------------


def _feed_route(slf, path):
    """Duck-typed handle_global_path body (the handler is async; drive it sync)."""
    import asyncio

    slf._route_deviation = WavefrontLocalPlanner._route_deviation  # staticmethod
    slf._route_changed = lambda new: WavefrontLocalPlanner._route_changed(slf, new)

    class _Msg:
        def __init__(self, pts):
            class _P:
                def __init__(self, x, y):
                    self.position = SimpleNamespace(x=x, y=y)
            self.poses = [_P(x, y) for x, y in pts]

    asyncio.get_event_loop().run_until_complete(
        WavefrontLocalPlanner.handle_global_path(slf, _Msg(path))
    )


def test_alternative_route_flip_flop_is_debounced():
    """The global planner flip-flopping between two same-goal routes must not drag
    the local planner back and forth (recorded: stairs corridor <-> east terrace
    and wp4 north <-> south, phases of 2-8 s). An alternative only lands after it
    has been published consistently for route_change_persist_s."""
    cost = _free_grid(300, 300)
    route_a = _straight_path((1.0, 2.0), (20.0, 2.0))
    route_b = [(x, y + 6.0) for x, y in route_a[:-1]] + [route_a[-1]]  # same goal, 6 m away
    cfg = WavefrontLocalPlannerConfig()
    slf, _ = _make_state(cost, route_a, cfg)
    slf._pending_route = None
    slf._pending_route_since = 0.0
    slf._plan_dirty = False

    for _ in range(4):  # rapid A/B alternation: B never persists long enough
        _feed_route(slf, route_b)
        assert slf._path_world == route_a, "flip must not land"
        _feed_route(slf, route_a)
    assert not slf._plan_dirty

    # B persists: republished after the persistence window has elapsed -> adopted.
    _feed_route(slf, route_b)
    assert slf._path_world == route_a
    slf._pending_route_since -= cfg.route_change_persist_s + 1.0  # window elapsed
    _feed_route(slf, route_b)
    assert slf._path_world == route_b
    assert slf._plan_dirty


def test_goal_advance_is_never_debounced():
    """A new goal (route leg advance) must land immediately."""
    cost = _free_grid(300, 300)
    route_a = _straight_path((1.0, 2.0), (20.0, 2.0))
    next_leg = _straight_path((1.0, 2.0), (20.0, 25.0))  # different goal
    cfg = WavefrontLocalPlannerConfig()
    slf, _ = _make_state(cost, route_a, cfg)
    slf._pending_route = None
    slf._pending_route_since = 0.0
    slf._plan_dirty = False
    _feed_route(slf, next_leg)
    assert slf._path_world == next_leg
    assert slf._plan_dirty


def test_routine_direction_reversal_is_rejected():
    """Stairs descent (vid28): band-edge flicker made every other solve point back
    UP while the global route ran DOWN; each landed via the 'refine' dirty path and
    the robot turned around mid-staircase. A routine solve contradicting the
    committed plan's direction (>90 deg initial swing) must be rejected."""
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    slf, params = _make_state(cost, path, cfg)
    robot = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist
    )
    # A long solve pointing the opposite way (as long as the committed remainder).
    reversed_plan = [(5.0 - 0.1 * k, 2.0, math.pi) for k in range(40)]
    for reason in ("refine", "periodic", "horizon"):
        assert _gate(slf, reversed_plan, robot, reason) is False
    # The same solve under an EVENT reason (moved goal / blockage) must adopt.
    for reason in ("goal", "blocked", "deviation", "no_progress"):
        assert _gate(slf, reversed_plan, robot, reason) is True


def test_routine_aligned_solve_still_adopts():
    cost = _free_grid(300, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0)
    slf, params = _make_state(cost, path, cfg)
    robot = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, robot, params, obstacle_dist=slf._obstacle_dist
    )
    fresh = plan_path(
        slf._cost, RES, ORIGIN, path, (5.4, 2.0, 0.0), params, obstacle_dist=slf._obstacle_dist
    )
    assert _gate(slf, fresh, (5.4, 2.0, 0.0), "refine") is True
