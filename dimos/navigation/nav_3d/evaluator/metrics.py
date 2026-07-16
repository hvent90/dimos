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

"""Scoring for the nav-3d evaluator: SPL, the path validity gate, references."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.final_map import (
    cylinder_offsets,
    densify,
    key_centers,
    keys_contain,
    offset_keys,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.recording import Trajectory


def path_length(waypoints: NDArray[np.float32]) -> float:
    if len(waypoints) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(waypoints, axis=0), axis=1).sum())


def goal_reached(
    waypoints: NDArray[np.float32], goal: tuple[float, float, float], tolerance: float
) -> bool:
    return bool(np.linalg.norm(waypoints[-1] - np.asarray(goal, dtype=np.float32)) <= tolerance)


@dataclass
class GateResult:
    """Collision check of a path against the final obstacle set."""

    valid: bool
    collision_points: NDArray[np.float32]


def check_path(
    waypoints: NDArray[np.float32],
    obstacle_keys: NDArray[np.int64],
    voxel_size: float,
    robot_radius: float,
    ground_margin: float,
    body_clearance: float,
) -> GateResult:
    """Sweep the robot body along foot-level waypoints against final obstacles.

    The checked volume at each sample is a cylinder from ground_margin above
    the foot (so the supporting floor never counts) up to body_clearance.
    Candidate voxels come from a padded voxelized cylinder and are then
    verified against the exact continuous bounds, so quantization never pulls
    ground voxels into the check.
    """
    samples = densify(waypoints, voxel_size / 2)
    offsets = cylinder_offsets(
        robot_radius + voxel_size,
        ground_margin - voxel_size,
        body_clearance + voxel_size,
        voxel_size,
    )
    keys = offset_keys(samples, offsets, voxel_size)
    candidate = keys_contain(obstacle_keys, keys.ravel()).reshape(keys.shape)
    s_idx, o_idx = np.nonzero(candidate)
    if len(s_idx) == 0:
        return GateResult(valid=True, collision_points=samples[:0])
    delta = key_centers(keys[s_idx, o_idx], voxel_size) - samples[s_idx]
    exact = (
        (np.linalg.norm(delta[:, :2], axis=1) <= robot_radius)
        & (delta[:, 2] >= ground_margin)
        & (delta[:, 2] <= body_clearance)
    )
    colliding = np.unique(s_idx[exact])
    return GateResult(valid=len(colliding) == 0, collision_points=samples[colliding])


@dataclass
class Reference:
    """Demonstrated route between a case's endpoints."""

    length: float
    snapped: bool
    # When the robot stood at the start about to walk the route; inf when
    # the endpoints are off the trajectory or no causal pair exists.
    start_ts: float
    # True when the goal was visited before the chosen start visit, so a
    # planner at start_ts targets a place the robot has already been.
    causal: bool


def reference_length(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    robot_height: float,
    max_snap_m: float = 1.0,
) -> Reference:
    """Shortest walked length the trajectory demonstrates between start and goal.

    The robot usually passes each spot several times, so the reference is the
    minimum route length over every combination of start and goal visits, not
    the route between single nearest poses, which would include any wandering
    in between. Only causal pairs count when one exists: the goal visited
    before the start, so an incremental map at the start time has seen the
    goal and the demonstrated route. When either endpoint is farther than
    max_snap_m from the trajectory, falls back to the straight-line distance.
    """
    foot = trajectory.positions - np.array([0.0, 0.0, robot_height], dtype=np.float32)
    s = np.asarray(start, dtype=np.float32)
    g = np.asarray(goal, dtype=np.float32)
    ds = np.linalg.norm(foot - s, axis=1)
    dg = np.linalg.norm(foot - g, axis=1)
    if ds.min() > max_snap_m or dg.min() > max_snap_m:
        return Reference(float(np.linalg.norm(g - s)), False, float("inf"), False)
    arcs = trajectory.arc_lengths()
    near_s = np.flatnonzero(ds <= max_snap_m)
    near_g = np.flatnonzero(dg <= max_snap_m)
    totals = (
        np.abs(arcs[near_s][:, None] - arcs[near_g][None, :])
        + ds[near_s][:, None]
        + dg[near_g][None, :]
    )
    backward = trajectory.ts[near_g][None, :] <= trajectory.ts[near_s][:, None]
    causal = bool(backward.any())
    if causal:
        totals = np.where(backward, totals, np.inf)
    best = np.unravel_index(totals.argmin(), totals.shape)
    i = int(near_s[best[0]])
    start_ts = float(trajectory.ts[i]) if causal else float("inf")
    return Reference(max(float(totals[best]), 1e-6), True, start_ts, causal)


def spl(success: bool, l_ref: float, p_len: float) -> float:
    if not success:
        return 0.0
    return l_ref / max(p_len, l_ref)


def soft_progress(
    end: NDArray[np.float32] | None,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
) -> float:
    """Fraction of the start-goal distance covered by the path endpoint."""
    d0 = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
    if end is None or d0 < 1e-6:
        return 0.0
    d1 = float(np.linalg.norm(np.asarray(goal, dtype=np.float32) - end))
    return float(np.clip(1.0 - d1 / d0, 0.0, 1.0))


def timing_stats(samples_ms: list[float]) -> dict[str, float]:
    if not samples_ms:
        return {"p50": 0.0, "p95": 0.0, "max": 0.0}
    arr = np.asarray(samples_ms)
    return {
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(arr.max()),
    }
