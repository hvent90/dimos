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

"""Lockstep tests for the module-level rolling-horizon refresh in
``WavefrontLocalPlanner._plan_needs_replan``.

The local planner caches its committed plan and only re-solves the (expensive)
Dijkstra on invalidation; between solves it re-emits the cached plan. The
*rolling-horizon refresh* re-solves when little of the committed plan remains
ahead of the robot, so the follower never runs out of lookahead and stalls
(a ~horizon-periodic stop-go that roughly halves effective speed on long
routes — the wp4 loop). These tests exercise the ACTUAL ``_plan_needs_replan``
method (via a duck-typed self, no framework machinery) rather than calling
``plan_path`` every tick the way ``test_dynamic_obstacles`` does, because the
stop-go lives in the caching loop, not in ``plan_path``.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np

from dimos.navigation.jnav.modules.local_planner.wavefront.local_planner import (
    WavefrontLocalPlanner,
    WavefrontLocalPlannerConfig,
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


def _make_planner_state(cost, path, cfg):
    """A duck-typed stand-in for the module holding exactly the attributes
    ``_plan_needs_replan`` reads. Lets us drive the real method deterministically."""
    params = cfg.to_params()
    clipped = np.clip(np.nan_to_num(cost, nan=0.0), 0, 100).astype(np.float64)
    obstacle_dist = _obstacle_distance(clipped, RES, params)
    slf = SimpleNamespace(
        _plan=None,
        _plan_dirty=True,
        _obstacle_dist=obstacle_dist,
        _path_world=path,
        _route_tail=[],
        _origin=ORIGIN,
        _resolution=RES,
        _cost=clipped,
        _prev_local=None,
        _best_remaining_arc=float("inf"),
        _last_progress_monotonic=0.0,
        _last_solve_mono=0.0,
        _costmap_generation=0,
        _blocked_costmap_id=None,
        _horizon_boost=0.0,
        config=cfg,
    )
    now = __import__("time").monotonic()
    slf._last_progress_monotonic = now
    slf._last_solve_mono = now
    return slf, params, clipped


def _remaining_ahead(plan, robot) -> float:
    """Arc-length of the committed plan still ahead of the robot."""
    pw = np.asarray([(p[0], p[1]) for p in plan], dtype=np.float64)
    d2 = (pw[:, 0] - robot[0]) ** 2 + (pw[:, 1] - robot[1]) ** 2
    i = int(d2.argmin())
    if i >= len(pw) - 1:
        return 0.0
    return float(np.sum(np.hypot(np.diff(pw[i:, 0]), np.diff(pw[i:, 1]))))


def _needs_replan(slf, robot, params) -> bool:
    slf._last_solve_mono = __import__("time").monotonic()  # keep the periodic trigger quiet
    effective = slf._path_world + WavefrontLocalPlanner._carrot_extension(slf)
    return WavefrontLocalPlanner._plan_needs_replan(slf, robot, params, effective) is not None


def test_refresh_fires_exactly_when_plan_is_nearly_consumed_and_on_path():
    """Isolates the fix: in a state where the robot is ON the committed path
    (no drift), the plan is not dirty, the goal is far (plan end != goal), and
    nothing ahead is blocked — so EVERY pre-existing trigger is False — the
    refresh must still fire because the plan is nearly consumed. Without the fix
    the planner would reuse the spent plan and the robot would stall."""
    cost = _free_grid(220, 40)  # 22 x 4 m free corridor
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0, replan_deviation_meters=0.35)
    slf, params, _ = _make_planner_state(cost, path, cfg)

    # Commit a plan solved from x=5.0 (a ~horizon-long slice of the straight route).
    solve_from = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, solve_from, params, obstacle_dist=slf._obstacle_dist
    )
    slf._plan_dirty = False
    assert slf._plan and len(slf._plan) >= 2

    # The committed plan reaches ~x=8 (horizon 3 m). Its end is NOT the goal (x=20).
    plan_end = slf._plan[-1]
    assert math.hypot(plan_end[0] - 20.0, plan_end[1] - 2.0) > 1.0, "plan should be horizon-truncated, not at goal"

    # Robot near the END of the committed plan, still exactly on the path (y=2.0).
    robot = (plan_end[0] - 0.2, 2.0, 0.0)

    # Every PRE-EXISTING trigger is False in this state:
    assert not slf._plan_dirty
    gx, gy = path[-1]
    assert math.hypot(robot[0] - gx, robot[1] - gy) > params.goal_tolerance  # not at goal
    pw = np.asarray([(p[0], p[1]) for p in slf._plan])
    drift = math.sqrt(float(np.min((pw[:, 0] - robot[0]) ** 2 + (pw[:, 1] - robot[1]) ** 2)))
    assert drift <= cfg.replan_deviation_meters  # on-path, no drift trigger
    remaining = _remaining_ahead(slf._plan, robot)
    assert remaining < 1.0  # plan nearly consumed

    # => the refresh (and only the refresh) makes it re-solve:
    assert _needs_replan(slf, robot, params) is True


def test_refresh_does_not_over_trigger_with_plenty_of_plan_ahead():
    """Control: robot at the START of a fresh committed plan (lots of runway,
    on-path) must REUSE the plan, not re-solve — the refresh is bounded."""
    cost = _free_grid(220, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0, replan_deviation_meters=0.35)
    slf, params, _ = _make_planner_state(cost, path, cfg)
    solve_from = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, solve_from, params, obstacle_dist=slf._obstacle_dist
    )
    slf._plan_dirty = False
    robot = (5.1, 2.0, 0.0)  # just started the plan
    assert _remaining_ahead(slf._plan, robot) > 1.0
    assert _needs_replan(slf, robot, params) is False


def test_lockstep_traversal_keeps_continuous_lookahead_no_stall():
    """End-to-end: drive the real caching loop (re-solve only when
    ``_plan_needs_replan`` says so) while advancing the robot along a 16 m
    straight route. The committed plan must ALWAYS keep lookahead ahead of the
    robot (never fully consumed), and the refresh must fire multiple times."""
    cost = _free_grid(220, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(horizon=3.0, replan_deviation_meters=0.35)
    slf, params, _ = _make_planner_state(cost, path, cfg)

    resolves = 0
    min_ahead = float("inf")
    x = 1.0
    step = 0.2  # the follower consumes ~step m of plan per tick
    while x < 17.0:
        robot = (x, 2.0, 0.0)
        if _needs_replan(slf, robot, params):
            slf._plan = plan_path(
                slf._cost, RES, ORIGIN, path, robot, params,
                previous_path=slf._prev_local, obstacle_dist=slf._obstacle_dist,
            )
            slf._plan_dirty = False
            slf._prev_local = [(p[0], p[1]) for p in slf._plan] if len(slf._plan) >= 2 else None
            resolves += 1
        assert slf._plan and len(slf._plan) >= 2
        min_ahead = min(min_ahead, _remaining_ahead(slf._plan, robot))
        x += step

    # The rolling horizon fired repeatedly (not just the initial solve) ...
    assert resolves >= 4, f"refresh should re-solve several times over 16 m, got {resolves}"
    # ... and the robot ALWAYS had meaningful lookahead — never stalled at a spent plan.
    assert min_ahead > 0.5, f"lookahead collapsed to {min_ahead:.2f} m (stall)"


# --- dijkstra_radius (search window) — Corner 2: cheap per-tick solves on a big costmap ----
from dimos.navigation.jnav.modules.local_planner.wavefront.local_planner import (  # noqa: E402
    WavefrontParams,
    _dijkstra_tree,
)


def _reachable_count(free, entry, start, radius_cells):
    _, parent = _dijkstra_tree(free, entry, RES, start, radius_cells)
    return int((parent[:, :, 0] >= 0).sum())


def test_dijkstra_radius_confines_exploration():
    """A radius window must explore far fewer cells than the whole costmap."""
    free = np.ones((200, 200), dtype=bool)  # 20 x 20 m all free
    entry = np.ones((200, 200), dtype=np.float64)
    start = (100, 100)
    full = _reachable_count(free, entry, start, 0)
    windowed = _reachable_count(free, entry, start, 30)  # 3 m window
    assert full >= 200 * 200 - 1  # whole grid reached (root cell has no parent)
    # (2*30+1)^2 = 3721 cells — an order of magnitude less than 40000.
    assert windowed <= (2 * 30 + 1) ** 2
    assert windowed < full / 5


def test_dijkstra_radius_matches_unbounded_when_window_covers_carrot():
    """With the window comfortably larger than the carrot, the emitted local path
    is IDENTICAL to the unbounded search — the bound is a pure compute saving, not
    a behavior change — both in open space and around an in-window obstacle."""
    cost_open = _free_grid(220, 60)
    path = _straight_path((1.0, 3.0), (20.0, 3.0))
    robot = (2.0, 3.0, 0.0)
    p_unbounded = WavefrontParams(carrot_lookahead=4.0, dijkstra_radius=0.0)
    p_windowed = WavefrontParams(carrot_lookahead=4.0, dijkstra_radius=8.0)
    a = plan_path(cost_open, RES, ORIGIN, path, robot, p_unbounded)
    b = plan_path(cost_open, RES, ORIGIN, path, robot, p_windowed)
    assert a and b and len(a) == len(b)
    assert max(math.hypot(pa[0] - pb[0], pa[1] - pb[1]) for pa, pb in zip(a, b)) < 1e-9

    # Obstacle straddling the straight line within the window -> a detour that still fits.
    cost_obs = _free_grid(220, 60)
    cost_obs[int(2.4 / RES):int(3.6 / RES), int(4.4 / RES):int(4.7 / RES)] = 100
    a2 = plan_path(cost_obs, RES, ORIGIN, path, robot, p_unbounded)
    b2 = plan_path(cost_obs, RES, ORIGIN, path, robot, p_windowed)
    assert a2 and b2 and len(a2) == len(b2)
    assert max(math.hypot(pa[0] - pb[0], pa[1] - pb[1]) for pa, pb in zip(a2, b2)) < 1e-9


def test_no_progress_invalidates_the_committed_plan_and_drops_commitment():
    """A stalled robot never trips the deviation check (it is not moving), so a stale
    committed plan re-emits every tick forever, and commitment_weight resurrects its
    shape on any re-solve — a deadlock recorded live as the local path flip-flopping
    between a fresh forward route and a stale backward one for 80+ s in open space.
    After no_progress_replan_s without arc progress, the plan must invalidate AND the
    commitment (previous_path) must be dropped so the fresh solve is unbiased."""
    import time as _time

    cost = _free_grid(220, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(
        horizon=3.0, replan_deviation_meters=0.35, no_progress_replan_s=0.2
    )
    slf, params, _ = _make_planner_state(cost, path, cfg)
    solve_from = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, solve_from, params, obstacle_dist=slf._obstacle_dist
    )
    slf._plan_dirty = False
    slf._prev_local = [(p[0], p[1]) for p in slf._plan]
    robot = (5.0, 2.0, 0.0)  # at the plan start, on-path, goal far, nothing blocked

    # Fresh plan with runway: no trigger fires (this also primes the progress tracker).
    assert _needs_replan(slf, robot, params) is False
    # The robot does not move; within the grace window the plan is still reused.
    assert _needs_replan(slf, robot, params) is False
    # After no_progress_replan_s with zero arc progress the plan must invalidate and
    # the commitment bias must be dropped.
    _time.sleep(0.25)
    assert _needs_replan(slf, robot, params) is True
    assert slf._prev_local is None


def test_progress_keeps_resetting_the_no_progress_timer():
    """Control: a robot advancing along the plan must never trip the no-progress
    invalidation, no matter how much wall time passes between checks."""
    import time as _time

    cost = _free_grid(220, 40)
    path = _straight_path((1.0, 2.0), (20.0, 2.0))
    cfg = WavefrontLocalPlannerConfig(
        horizon=3.0, replan_deviation_meters=0.35, no_progress_replan_s=0.2
    )
    slf, params, _ = _make_planner_state(cost, path, cfg)
    solve_from = (5.0, 2.0, 0.0)
    slf._plan = plan_path(
        slf._cost, RES, ORIGIN, path, solve_from, params, obstacle_dist=slf._obstacle_dist
    )
    slf._plan_dirty = False
    slf._prev_local = [(p[0], p[1]) for p in slf._plan]

    x = 5.0
    assert _needs_replan(slf, (x, 2.0, 0.0), params) is False
    for _ in range(3):
        _time.sleep(0.12)
        x += 0.3  # steady progress along the straight plan
        result = _needs_replan(slf, (x, 2.0, 0.0), params)
        remaining = _remaining_ahead(slf._plan, (x, 2.0, 0.0))
        if remaining < 1.0:
            break  # the rolling-horizon refresh may legitimately fire near the end
        assert result is False, "no-progress invalidation fired despite steady progress"
        assert slf._prev_local is not None
