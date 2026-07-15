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

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.cases import load_suite
from dimos.navigation.nav_3d.evaluator.golden import (
    key_centers,
    keys_contain,
    voxel_keys,
    walked_corridor_keys,
)
from dimos.navigation.nav_3d.evaluator.recording import Trajectory

VOXEL = 0.1


def test_voxel_key_roundtrip() -> None:
    pts = np.array([[0.05, 0.05, 0.05], [-3.21, 4.7, -0.09], [80.0, -80.0, 12.3]], dtype=np.float32)
    centers = key_centers(voxel_keys(pts, VOXEL), VOXEL)
    assert np.all(np.abs(centers - pts) <= VOXEL / 2 + 1e-5)


def test_keys_contain() -> None:
    keys = np.sort(voxel_keys(np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float32), VOXEL))
    query = voxel_keys(np.array([[0, 0, 0], [5, 5, 5]], dtype=np.float32), VOXEL)
    assert keys_contain(keys, query).tolist() == [True, False]
    assert keys_contain(np.array([], dtype=np.int64), query).tolist() == [False, False]


def _wall(x: float) -> np.ndarray:
    ys, zs = np.meshgrid(np.arange(-1, 1, VOXEL), np.arange(0.05, 1.5, VOXEL))
    return np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1, dtype=np.float32)


def _gate(waypoints: np.ndarray, obstacles: np.ndarray) -> metrics.GateResult:
    keys = np.unique(voxel_keys(obstacles, VOXEL))
    return metrics.check_path(
        waypoints, keys, VOXEL, robot_radius=0.16, ground_margin=0.25, body_clearance=0.45
    )


def test_gate_blocks_wall_crossing() -> None:
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    result = _gate(path, _wall(0.0))
    assert not result.valid
    assert len(result.collision_points) > 0
    assert np.all(np.abs(result.collision_points[:, 0]) < 0.3)


def test_gate_passes_clear_path() -> None:
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    assert _gate(path, _wall(5.0)).valid


def test_gate_ignores_ground() -> None:
    xs, ys = np.meshgrid(np.arange(-2, 2, VOXEL), np.arange(-2, 2, VOXEL))
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.05)], axis=1, dtype=np.float32)
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    assert _gate(path, floor).valid


def test_gate_tolerates_stair_slope() -> None:
    """Terrain rising at stair slope inside the disc must not trigger the gate."""
    xs, ys = np.meshgrid(np.arange(-1, 2, VOXEL), np.arange(-1, 1, VOXEL))
    slope = np.stack([xs.ravel(), ys.ravel(), xs.ravel() * 0.7 - 0.05], axis=1, dtype=np.float32)
    path = np.stack(
        [np.arange(-0.5, 1.5, 0.1), np.zeros(20), np.arange(-0.5, 1.5, 0.1) * 0.7], axis=1
    ).astype(np.float32)
    assert _gate(path, slope).valid


def test_walked_corridor_exempts_gate() -> None:
    """A wall crossing carved by the walked corridor passes the gate."""
    wall = _wall(0.0)
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    traj = Trajectory(
        ts=np.array([0.0, 1.0]),
        positions=np.array([[-1, 0, 0.3], [1, 0, 0.3]], dtype=np.float32),
    )
    walked = walked_corridor_keys(traj, VOXEL, radius=0.3, z_lo=-0.3, z_hi=0.3)
    obstacles = np.setdiff1d(np.unique(voxel_keys(wall, VOXEL)), walked)
    result = metrics.check_path(
        path, obstacles, VOXEL, robot_radius=0.16, ground_margin=0.25, body_clearance=0.45
    )
    assert result.valid


def test_spl() -> None:
    assert metrics.spl(False, 10.0, 10.0) == 0.0
    assert metrics.spl(True, 10.0, 10.0) == 1.0
    assert metrics.spl(True, 10.0, 20.0) == pytest.approx(0.5)
    assert metrics.spl(True, 10.0, 5.0) == 1.0


def test_reference_length_snaps_to_trajectory() -> None:
    positions = np.stack(
        [np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1
    ).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 10, 101), positions=positions)
    l_ref, snapped = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), robot_height=0.3)
    assert snapped
    assert l_ref == pytest.approx(10.0, abs=0.01)
    l_ref, snapped = metrics.reference_length(traj, (0, 5, 0), (10, 0, 0), robot_height=0.3)
    assert not snapped


def test_path_length_and_goal() -> None:
    path = np.array([[0, 0, 0], [3, 4, 0]], dtype=np.float32)
    assert metrics.path_length(path) == pytest.approx(5.0)
    assert metrics.goal_reached(path, (3, 4, 0.2), tolerance=0.5)
    assert not metrics.goal_reached(path, (3, 4, 1.0), tolerance=0.5)


def test_load_suite(tmp_path) -> None:
    manifest = tmp_path / "demo.yaml"
    manifest.write_text(
        "dataset: demo\n"
        "cases:\n"
        "  - id: a\n"
        "    start: [0, 0, 0]\n"
        "    goal: [1, 2, 3]\n"
        "    weight: 2\n"
        "    tags: [stairs]\n"
    )
    suite = load_suite(manifest)
    assert suite.dataset == "demo"
    assert suite.cases[0].goal == (1.0, 2.0, 3.0)
    assert suite.cases[0].weight == 2.0

    manifest.write_text(
        "dataset: demo\ncases:\n"
        "  - {id: a, start: [0, 0, 0], goal: [1, 2, 3]}\n"
        "  - {id: a, start: [0, 0, 0], goal: [4, 5, 6]}\n"
    )
    with pytest.raises(ValueError, match="duplicate"):
        load_suite(manifest)
