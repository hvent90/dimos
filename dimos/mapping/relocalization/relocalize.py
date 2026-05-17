#!/usr/bin/env python3
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

"""THIS IS THE FILE YOU EDIT.

Define ``relocalize(global_map, local_map) -> 4x4 numpy array`` that
maps body-frame points in ``local_map`` to world-frame points in
``global_map``. The read-only evaluator (`run.py`) imports this module
and runs your function against 20 cached test frames within a 5-minute
wall-clock budget. Invoke the experiment as:

    uv run dimos/mapping/relocalization/run.py > .../run.log 2>&1

Baseline: FPFH + multi-scale RANSAC + point-to-plane ICP refinement,
with a gravity-prior filter (assumes both clouds are z-up, which they
are here: Go2 body frame has z = vertical, global map is SLAM-built
with z = vertical).

Reference:
  https://www.open3d.org/docs/latest/python_api/open3d.registration.registration_ransac_based_on_feature_matching.html
"""

from __future__ import annotations

import numpy as np
import open3d as o3d

_reg = o3d.pipelines.registration

# ---- Tuning knobs ----------------------------------------------------------
VOXEL_SIZES = [0.3, 0.5, 0.8]     # coarse voxels for FPFH + RANSAC (multi-scale)
RANSAC_RESTARTS = 3                # extra RANSAC runs per scale → more candidates to choose from
RANSAC_ITERS = 500_000             # RANSAC iteration budget per scale
FINE_VOXEL = 0.1                   # voxel for the final ICP refinement
RERANK_DIST = FINE_VOXEL * 1.5     # inlier dist for fine-scale candidate scoring
GRAVITY_TILT_MAX_DEG = 10.0        # reject candidates whose z-axis tilts more than this


def _preprocess(pcd: o3d.geometry.PointCloud, voxel_size: float):
    """Downsample, estimate normals, compute FPFH descriptors."""
    down = pcd.voxel_down_sample(voxel_size)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )
    fpfh = _reg.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100),
    )
    return down, fpfh


def _ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size: float):
    """Open3D feature-matching RANSAC. Returns a RegistrationResult.

    Docs:
      https://www.open3d.org/docs/latest/python_api/open3d.registration.registration_ransac_based_on_feature_matching.html
    """
    dist = voxel_size * 1.5
    return _reg.registration_ransac_based_on_feature_matching(
        src_down,
        tgt_down,
        src_fpfh,
        tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist,
        estimation_method=_reg.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            _reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            _reg.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=_reg.RANSACConvergenceCriteria(RANSAC_ITERS, 0.999),
    )


def _gravity_tilt_deg(T: np.ndarray) -> float:
    """Angle (deg) between the transform's z-axis and world z-up."""
    z_world = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(z_world[2], -1.0, 1.0))))


def relocalize(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
) -> np.ndarray:
    """Estimate the 4x4 transform placing ``local_map`` into ``global_map``.

    Multi-scale × multi-restart FPFH+RANSAC → gravity-filtered, re-ranked by
    fine-scale inlier ratio (not RANSAC's own fitness) → fine ICP. The
    rerank catches z-degenerate and wrong-room busts: at FINE_VOXEL a
    5m-off candidate has ~0 inliers while RANSAC reports it as fit.
    """
    # Fine downsample once — used for both candidate scoring and the final ICP.
    src_fine = local_map.voxel_down_sample(FINE_VOXEL)
    tgt_fine = global_map.voxel_down_sample(FINE_VOXEL)
    src_fine.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=FINE_VOXEL * 2, max_nn=30)
    )
    tgt_fine.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=FINE_VOXEL * 2, max_nn=30)
    )

    candidates: list[np.ndarray] = []  # 4x4 transforms
    for vs in VOXEL_SIZES:
        src_down, src_fpfh = _preprocess(local_map, vs)
        tgt_down, tgt_fpfh = _preprocess(global_map, vs)
        for _ in range(1 + RANSAC_RESTARTS):
            # Successive calls advance Open3D's RNG state (seeded per-frame in
            # run.py), so each restart explores a different sample sequence.
            result = _ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, vs)
            candidates.append(np.asarray(result.transformation))

    # Gravity filter; fall back to all if everything is tilted (degenerate clouds).
    upright = [T for T in candidates if _gravity_tilt_deg(T) <= GRAVITY_TILT_MAX_DEG]
    pool = upright if upright else candidates

    # Re-rank by fine-scale inlier fitness. evaluate_registration runs only
    # correspondence accounting (no refinement) — cheap and honest.
    def fine_fitness(T: np.ndarray) -> float:
        r = _reg.evaluate_registration(src_fine, tgt_fine, RERANK_DIST, T)
        return float(r.fitness)

    best_T = max(pool, key=fine_fitness)

    refined = _reg.registration_icp(
        src_fine,
        tgt_fine,
        FINE_VOXEL * 0.4,
        best_T,
        _reg.TransformationEstimationPointToPlane(),
        _reg.ICPConvergenceCriteria(max_iteration=200),
    )
    return np.asarray(refined.transformation)
