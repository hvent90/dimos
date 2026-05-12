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

"""Tests for Relocalize: synthetic ICP recovery."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest
from scipy.spatial.transform import Rotation

from dimos.navigation.relocalize.relocalize import (
    ICPParams,
    load_prior_pcd,
    pose_matrix,
    two_stage_icp,
)


def _synthetic_room(n_points: int = 8000) -> o3d.geometry.PointCloud:
    """Build a structured point cloud: 5 walls of a 10x10x3m room plus a floor.

    Structured walls give ICP enough features to recover translation+yaw; a
    pure floor or single wall would be degenerate.
    """
    rng = np.random.default_rng(seed=42)
    per_face = n_points // 6

    floor_xy = rng.uniform([-5, -5], [5, 5], size=(per_face, 2))
    floor = np.column_stack([floor_xy, np.zeros(per_face)])

    ceil_xy = rng.uniform([-5, -5], [5, 5], size=(per_face, 2))
    ceiling = np.column_stack([ceil_xy, 3.0 * np.ones(per_face)])

    wall_n_x = rng.uniform(-5, 5, size=per_face)
    wall_n_z = rng.uniform(0, 3, size=per_face)
    wall_n = np.column_stack([wall_n_x, 5.0 * np.ones(per_face), wall_n_z])

    wall_s_x = rng.uniform(-5, 5, size=per_face)
    wall_s_z = rng.uniform(0, 3, size=per_face)
    wall_s = np.column_stack([wall_s_x, -5.0 * np.ones(per_face), wall_s_z])

    wall_e_y = rng.uniform(-5, 5, size=per_face)
    wall_e_z = rng.uniform(0, 3, size=per_face)
    wall_e = np.column_stack([5.0 * np.ones(per_face), wall_e_y, wall_e_z])

    wall_w_y = rng.uniform(-5, 5, size=per_face)
    wall_w_z = rng.uniform(0, 3, size=per_face)
    wall_w = np.column_stack([-5.0 * np.ones(per_face), wall_w_y, wall_w_z])

    points = np.concatenate([floor, ceiling, wall_n, wall_s, wall_e, wall_w]).astype(np.float64)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud


def _apply_transform(
    cloud: o3d.geometry.PointCloud, transform: np.ndarray
) -> o3d.geometry.PointCloud:
    out = o3d.geometry.PointCloud(cloud)
    out.transform(transform)
    return out


def test_two_stage_icp_recovers_known_pose():
    """Live scan = prior map shifted by a known transform; ICP must recover it."""
    prior = _synthetic_room()
    params = ICPParams()
    rough_target = prior.voxel_down_sample(params.rough_map_resolution)
    refine_target = prior.voxel_down_sample(params.refine_map_resolution)

    truth = pose_matrix(x=1.2, y=-0.7, z=0.05, roll=0.0, pitch=0.0, yaw=math.radians(8.0))

    # The live scan lives in the local-map frame (= world shifted by truth^-1).
    # ICP's job is to recover `truth` itself, mapping live-scan points into the
    # prior map frame.
    live_scan = _apply_transform(prior, np.linalg.inv(truth))

    # A rough initial guess offset from the truth by ~30cm + 4°.
    rough_guess = pose_matrix(x=0.9, y=-0.4, z=0.0, roll=0.0, pitch=0.0, yaw=math.radians(4.0))

    converged, recovered = two_stage_icp(
        live_scan, rough_target, refine_target, rough_guess, params
    )
    assert converged, "ICP failed to converge on synthetic room with reasonable initial guess"

    trans_err = float(np.linalg.norm(recovered[:3, 3] - truth[:3, 3]))
    rot_delta = Rotation.from_matrix(recovered[:3, :3] @ truth[:3, :3].T)
    yaw_err = abs(rot_delta.as_euler("ZYX")[0])

    assert trans_err < 0.10, f"Translation off by {trans_err:.3f} m"
    assert yaw_err < math.radians(2.0), f"Yaw off by {math.degrees(yaw_err):.3f} deg"


def test_two_stage_icp_rejects_bad_match():
    """Wildly wrong initial guess should fail the score threshold and not converge."""
    prior = _synthetic_room()
    params = ICPParams()
    rough_target = prior.voxel_down_sample(params.rough_map_resolution)
    refine_target = prior.voxel_down_sample(params.refine_map_resolution)

    # Live scan is a totally unrelated cloud — points scattered in a small ball
    # far from the room; should not converge to anything useful.
    rng = np.random.default_rng(seed=0)
    junk = rng.normal(loc=(20.0, 20.0, 20.0), scale=0.5, size=(2000, 3))
    live_scan = o3d.geometry.PointCloud()
    live_scan.points = o3d.utility.Vector3dVector(junk)

    converged, _ = two_stage_icp(live_scan, rough_target, refine_target, np.eye(4), params)
    assert not converged


def test_load_prior_pcd_round_trip(tmp_path: Path):
    """Writing and reading back a PCD yields a non-empty downsampled target."""
    prior = _synthetic_room(n_points=4000)
    pcd_path = tmp_path / "room.pcd"
    o3d.io.write_point_cloud(str(pcd_path), prior)

    rough, refine = load_prior_pcd(pcd_path, ICPParams())
    assert len(rough.points) > 0
    assert len(refine.points) > 0
    # Refine resolution is finer → at least as many points as rough.
    assert len(refine.points) >= len(rough.points)


def test_load_prior_pcd_missing(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        load_prior_pcd(tmp_path / "nope.pcd", ICPParams())
