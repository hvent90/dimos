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

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.cases import Case, Suite, load_suite, save_suite
from dimos.navigation.nav_3d.evaluator.generate import (
    Candidate,
    GenerationParams,
    _select_diverse,
    drift_stats,
    generate_cases,
    snap_to_surface,
)
from dimos.navigation.nav_3d.evaluator.golden import (
    GoldenMap,
    key_centers,
    keys_contain,
    voxel_keys,
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


def test_reference_length_uses_shortest_revisit() -> None:
    """An out-and-back trajectory must not inflate the reference with the loop."""
    out = np.stack([np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1)
    detour = np.stack([np.full(101, 10.0), np.linspace(0, 30, 101), np.full(101, 0.3)], axis=1)
    back = detour[::-1]
    positions = np.concatenate([out, detour, back]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 30, len(positions)), positions=positions)
    l_ref, snapped = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), robot_height=0.3)
    assert snapped
    assert l_ref == pytest.approx(10.0, abs=0.2)


def test_path_length_and_goal() -> None:
    path = np.array([[0, 0, 0], [3, 4, 0]], dtype=np.float32)
    assert metrics.path_length(path) == pytest.approx(5.0)
    assert metrics.goal_reached(path, (3, 4, 0.2), tolerance=0.5)
    assert not metrics.goal_reached(path, (3, 4, 1.0), tolerance=0.5)


def test_generate_cases_around_wall() -> None:
    """A U-shaped walk around a wall must yield non-trivial cases spanning it."""
    wall_pts = _wall(10.0)
    wall_keys = np.unique(voxel_keys(wall_pts, VOXEL))
    golden = GoldenMap(
        voxel_size=VOXEL,
        occupied=wall_pts,
        occupied_keys=wall_keys,
        frames=1,
        add_frame_ms={"p50": 0.0, "p95": 0.0, "max": 0.0},
        build_ms=0.0,
    )
    xs, ys = np.meshgrid(np.arange(0, 20, VOXEL), np.arange(-3, 6, VOXEL))
    surface = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1, dtype=np.float32)

    legs = [
        np.stack([np.linspace(2, 8, 40), np.zeros(40)], axis=1),
        np.stack([np.full(40, 8.0), np.linspace(0, 4, 40)], axis=1),
        np.stack([np.linspace(8, 12, 40), np.full(40, 4.0)], axis=1),
        np.stack([np.full(40, 12.0), np.linspace(4, 0, 40)], axis=1),
        np.stack([np.linspace(12, 18, 40), np.zeros(40)], axis=1),
    ]
    xy = np.concatenate(legs)
    positions = np.column_stack([xy, np.full(len(xy), 0.3)]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 60, len(positions)), positions=positions)

    cfg = SimpleNamespace(
        robot_height=0.3,
        voxel_size=VOXEL,
        robot_radius=0.16,
        ground_margin=0.25,
        body_clearance=0.45,
    )
    cases = generate_cases(traj, golden, surface, cfg, GenerationParams(max_cases=10))
    assert cases
    assert len({c.id for c in cases}) == len(cases)
    spans_wall = [c for c in cases if (c.start[0] - 10) * (c.goal[0] - 10) < 0]
    assert spans_wall
    for c in cases:
        assert abs(c.start[2]) < 1e-5 and abs(c.goal[2]) < 1e-5
        assert "flat" in c.tags


def test_select_diverse_backfills_to_min_cases() -> None:
    """Sector caps must not starve a dataset below the case floor."""
    candidates = [
        Candidate(start=(x, 0.0, 0.0), goal=(x, 20.0, 0.0), walked_m=30.0, detour_ratio=1.5, dz=0.0)
        for x in np.arange(0.0, 16.0, 2.0)
    ]
    strict = _select_diverse(candidates, GenerationParams(min_cases=0), max_cases=12)
    assert len(strict) == 4
    backfilled = _select_diverse(candidates, GenerationParams(min_cases=10), max_cases=12)
    assert len(backfilled) == 8
    assert len({(c.start, c.goal) for c in backfilled}) == 8


def test_snap_to_surface() -> None:
    xs, ys = np.meshgrid(np.arange(0, 2, VOXEL), np.arange(0, 2, VOXEL))
    surface = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1, dtype=np.float32)
    snapped = snap_to_surface(np.array([1.0, 1.0, 0.4], dtype=np.float32), surface, 1.0)
    assert snapped is not None
    assert abs(snapped[2]) < 1e-6
    assert np.linalg.norm(snapped[:2] - [1.0, 1.0]) < VOXEL
    assert snap_to_surface(np.array([9.0, 9.0, 0.0], dtype=np.float32), surface, 1.0) is None
    assert snap_to_surface(np.array([1.0, 1.0, 5.0], dtype=np.float32), surface, 1.0) is None


def test_drift_stats_flags_z_mismatch() -> None:
    n = 400
    ts = np.linspace(0, 120, n)
    out = np.stack([np.linspace(0, 20, n // 2), np.zeros(n // 2), np.zeros(n // 2)], axis=1)
    back = np.stack([np.linspace(20, 0, n // 2), np.zeros(n // 2), np.full(n // 2, 0.6)], axis=1)
    drifty = Trajectory(ts=ts, positions=np.concatenate([out, back]).astype(np.float32))
    stats = drift_stats(drifty)
    assert stats.revisit_dz_p95 > 0.3
    assert stats.warnings

    clean = Trajectory(ts=ts, positions=np.concatenate([out, out[::-1]]).astype(np.float32))
    assert not drift_stats(clean).warnings


def test_save_suite_roundtrip(tmp_path) -> None:
    suite = Suite(
        dataset="demo",
        cases=[Case(id="a", start=(0.0, 0.0, 0.0), goal=(1.0, 2.0, 3.0), weight=2.0, tags=["x"])],
        lidar_stream="other_lidar",
    )
    path = save_suite(suite, tmp_path / "demo.yaml")
    loaded = load_suite(path)
    assert loaded.dataset == "demo"
    assert loaded.lidar_stream == "other_lidar"
    assert loaded.odom_stream == "pointlio_odometry"
    assert loaded.cases[0].goal == (1.0, 2.0, 3.0)
    assert loaded.cases[0].tags == ["x"]


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
