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

from dimos.navigation.nav_3d.evaluator.golden import (
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
    """Collision check of a path against the golden obstacle set."""

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
    """Sweep the robot body along foot-level waypoints against golden obstacles.

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


def reference_length(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    robot_height: float,
    max_snap_m: float = 1.0,
) -> tuple[float, bool]:
    """Walked-trajectory length between the poses nearest start and goal.

    Returns (length, snapped). When either endpoint is farther than max_snap_m
    from the trajectory, falls back to the straight-line distance.
    """
    foot = trajectory.positions - np.array([0.0, 0.0, robot_height], dtype=np.float32)
    s = np.asarray(start, dtype=np.float32)
    g = np.asarray(goal, dtype=np.float32)
    ds = np.linalg.norm(foot - s, axis=1)
    dg = np.linalg.norm(foot - g, axis=1)
    i, j = int(ds.argmin()), int(dg.argmin())
    if ds[i] > max_snap_m or dg[j] > max_snap_m:
        return float(np.linalg.norm(g - s)), False
    arcs = trajectory.arc_lengths()
    length = abs(float(arcs[j] - arcs[i])) + float(ds[i]) + float(dg[j])
    return max(length, 1e-6), True


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
