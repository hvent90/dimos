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

"""Run case suites through the ray tracer and MLS planner and score them.

Every case is planned twice. The online plan runs on the incremental map:
what the mapper had built by the moment the robot stood at the case's start,
about to walk the demonstrated route back to a goal it already visited. The
final plan runs on the final map: the same pipeline fed the whole recording.
The final map is not ground truth, just the most complete map this pipeline
produces, so a failure on it means the whole pipeline cannot solve the case
even with all the data. The final path is gated against the full final
occupancy; the online path only against the final obstacles the sensor had
returns from by plan time: hitting a wall no lidar return ever came from is
not an error, but hitting one the sensor saw and the mapper dropped is. The
headline score is validity-gated SPL on the incremental map.
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import asdict, dataclass, field
import itertools
import threading
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.final_map import (
    key_centers,
    load_or_build_checkpoints,
    load_or_build_final_map,
)
from dimos.navigation.nav_3d.evaluator.recording import load_trajectory
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Case, Suite
    from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner

logger = setup_logger()

MAX_COLLISIONS_KEPT = 50
# The goal counts as seen when the incremental map has an occupied voxel
# within this distance of it at plan time.
GOAL_SEEN_RADIUS_M = 1.0


@dataclass
class PlanOutcome:
    planned: bool
    reached: bool
    valid: bool
    length: float
    plan_ms: float
    spl: float
    waypoints: list[list[float]]
    collisions: list[list[float]]

    @property
    def success(self) -> bool:
        return self.planned and self.reached and self.valid


@dataclass
class PlannerArtifacts:
    """Graph state of one planner after its map update. Not serialized to JSON."""

    surface_clearance: NDArray[np.float32]
    edges: NDArray[np.float32]


@dataclass
class CaseResult:
    id: str
    dataset: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    weight: float
    tags: list[str]
    l_ref: float
    l_ref_snapped: bool
    plan_ts: float
    online_voxels: int
    map_update_ms: float
    goal_seen: bool
    online: PlanOutcome
    final: PlanOutcome
    soft_progress: float
    # Planner graph on the incremental map, kept only for failed cases.
    online_artifacts: PlannerArtifacts | None = None


@dataclass
class DatasetResult:
    dataset: str
    cases: list[CaseResult]
    final_voxels: int
    map_build_ms: float
    add_frame_ms: dict[str, float]
    frames: int
    final_artifacts: PlannerArtifacts | None = None


@dataclass
class Report:
    score: float
    score_soft: float
    final_score: float
    n_cases: int
    n_success: int
    n_success_final: int
    # The incremental and final runs are independent tests per case; these
    # count the four pass/fail combinations.
    outcome_counts: dict[str, int]
    plan_ms: dict[str, float]
    map_update_ms: dict[str, float]
    datasets: list[DatasetResult]
    config: dict[str, float | int] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        out = asdict(self)
        for dataset in out["datasets"]:
            dataset.pop("final_artifacts")
            for case in dataset["cases"]:
                case.pop("online_artifacts")
        return out


def _run_plan(
    planner: MLSPlanner,
    case: Case,
    l_ref: float,
    obstacle_keys: NDArray[np.int64],
    cfg: EvalConfig,
) -> tuple[PlanOutcome, NDArray[np.float32] | None]:
    t0 = perf_counter()
    waypoints = planner.plan(case.start, case.goal)
    plan_ms = (perf_counter() - t0) * 1000
    if waypoints is None or len(waypoints) == 0:
        return PlanOutcome(False, False, False, 0.0, plan_ms, 0.0, [], []), None

    reached = metrics.goal_reached(waypoints, case.goal, cfg.goal_tolerance)
    gate = metrics.check_path(
        waypoints,
        obstacle_keys,
        cfg.voxel_size,
        cfg.robot_radius,
        cfg.ground_margin,
        cfg.body_clearance,
    )
    length = metrics.path_length(waypoints)
    success = reached and gate.valid
    outcome = PlanOutcome(
        planned=True,
        reached=reached,
        valid=gate.valid,
        length=length,
        plan_ms=plan_ms,
        spl=metrics.spl(success, l_ref, length),
        waypoints=waypoints.tolist(),
        collisions=gate.collision_points[:MAX_COLLISIONS_KEPT].tolist(),
    )
    return outcome, waypoints


def _goal_seen(online_points: NDArray[np.float32], goal: tuple[float, float, float]) -> bool:
    if len(online_points) == 0:
        return False
    d = np.linalg.norm(online_points - np.asarray(goal, dtype=np.float32), axis=1)
    return bool(d.min() <= GOAL_SEEN_RADIUS_M)


def _snapshot(planner: MLSPlanner) -> PlannerArtifacts:
    return PlannerArtifacts(
        surface_clearance=planner.surface_clearance_map(),
        edges=planner.node_edges(),
    )


def run_suite(suite: Suite, cfg: EvalConfig, threads: int = 1) -> DatasetResult:
    db_path = resolve_named_path(suite.dataset, ".db")
    trajectory = load_trajectory(db_path, suite.odom_stream)
    final = load_or_build_final_map(db_path, suite, cfg)
    obstacle_keys = final.occupied_keys

    refs: list[metrics.Reference] = []
    for case in suite.cases:
        ref = metrics.reference_length(trajectory, case.start, case.goal, cfg.robot_height)
        if not ref.snapped:
            logger.warning(
                "%s/%s: start or goal is off the walked trajectory; "
                "using straight-line reference and the full map",
                suite.dataset,
                case.id,
            )
        elif not ref.causal:
            logger.warning(
                "%s/%s: goal is never visited before the start; planning on the full map",
                suite.dataset,
                case.id,
            )
        if case.l_ref is not None:
            ref = metrics.Reference(case.l_ref, ref.snapped, ref.start_ts, ref.causal)
        refs.append(ref)

    start_ts = np.array([r.start_ts for r in refs], dtype=np.float64)
    checkpoints = load_or_build_checkpoints(db_path, suite, cfg, start_ts)
    case_ckpt = np.searchsorted(checkpoints.times, start_ts)

    final_planner = cfg.make_planner()
    final_planner.update_global_map(final.occupied)

    results: list[CaseResult | None] = [None] * len(suite.cases)

    def process_checkpoint(
        k: int,
        keys: NDArray[np.int64],
        online_gate_keys: NDArray[np.int64],
        online_planner: MLSPlanner,
    ) -> None:
        online_points = key_centers(keys, cfg.voxel_size)
        t0 = perf_counter()
        if len(online_points):
            online_planner.update_global_map(online_points)
        map_update_ms = (perf_counter() - t0) * 1000
        for ci in np.flatnonzero(case_ckpt == k):
            case, ref = suite.cases[ci], refs[ci]
            final_out, _ = _run_plan(final_planner, case, ref.length, obstacle_keys, cfg)
            if len(online_points):
                online_out, online_wp = _run_plan(
                    online_planner, case, ref.length, online_gate_keys, cfg
                )
            else:
                online_out = PlanOutcome(False, False, False, 0.0, 0.0, 0.0, [], [])
                online_wp = None
            end = online_wp[-1] if online_wp is not None and len(online_wp) else None
            goal_seen = _goal_seen(online_points, case.goal)
            results[ci] = CaseResult(
                id=case.id,
                dataset=suite.dataset,
                start=case.start,
                goal=case.goal,
                weight=case.weight,
                tags=case.tags,
                l_ref=ref.length,
                l_ref_snapped=ref.snapped,
                plan_ts=float(checkpoints.times[k]),
                online_voxels=len(keys),
                map_update_ms=map_update_ms,
                goal_seen=goal_seen,
                online=online_out,
                final=final_out,
                soft_progress=metrics.soft_progress(end, case.start, case.goal),
                online_artifacts=None
                if online_out.success or not len(online_points)
                else _snapshot(online_planner),
            )

    active = {int(k) for k in case_ckpt}
    tls = threading.local()

    def task(k: int, keys: NDArray[np.int64], gate: NDArray[np.int64]) -> None:
        planner = getattr(tls, "planner", None)
        if planner is None:
            planner = tls.planner = cfg.make_planner()
        try:
            process_checkpoint(k, keys, gate, planner)
        finally:
            in_flight.release()

    def snapshot_stream() -> Iterator[tuple[int, NDArray[np.int64], NDArray[np.int64]]]:
        """Walk the delta chain once. The online gate only holds obstacles the
        sensor had returns from by plan time; obstacles never observed are not
        the planner's fault."""
        gate = np.array([], dtype=np.int64)
        for k, (keys, observed_new) in enumerate(checkpoints.iter_snapshots()):
            fresh = np.intersect1d(obstacle_keys, observed_new, assume_unique=True)
            if len(fresh):
                gate = np.union1d(gate, fresh)
            if k in active:
                yield k, keys, gate

    # The planner releases the GIL and parallelizes updates internally via a
    # shared rayon pool, so a few worker threads interleave the serial parts
    # of checkpoint updates without oversubscribing. The semaphore bounds how
    # many reconstructed snapshots are held in memory at once.
    if threads > 1:
        in_flight = threading.BoundedSemaphore(threads * 2)
        with ThreadPoolExecutor(max_workers=threads) as pool:
            futures = []
            for item in snapshot_stream():
                in_flight.acquire()
                futures.append(pool.submit(task, *item))
            for future in futures:
                future.result()
    else:
        online_planner = cfg.make_planner()
        for k, keys, gate in snapshot_stream():
            process_checkpoint(k, keys, gate, online_planner)

    done = [r for r in results if r is not None]
    if len(done) != len(suite.cases):
        raise RuntimeError(f"{suite.dataset}: {len(suite.cases) - len(done)} cases not planned")
    return DatasetResult(
        dataset=suite.dataset,
        cases=done,
        final_voxels=len(final.occupied),
        map_build_ms=final.build_ms,
        add_frame_ms=final.add_frame_ms,
        frames=final.frames,
        final_artifacts=_snapshot(final_planner),
    )


def evaluate(suites: list[Suite], cfg: EvalConfig | None = None, workers: int = 1) -> Report:
    """Score every suite. workers is total parallelism: datasets spread over
    processes and each dataset's checkpoints over threads."""
    cfg = cfg or EvalConfig()
    if workers > 1 and len(suites) > 1:
        threads = max(1, workers // len(suites))
        with ProcessPoolExecutor(max_workers=min(workers, len(suites))) as pool:
            datasets = list(
                pool.map(run_suite, suites, itertools.repeat(cfg), itertools.repeat(threads))
            )
    else:
        datasets = [run_suite(suite, cfg, threads=workers) for suite in suites]
    cases = [c for d in datasets for c in d.cases]
    if not cases:
        raise ValueError("no cases to evaluate")

    weights = np.array([c.weight for c in cases])
    online_spl = np.array([c.online.spl for c in cases])
    final_spl = np.array([c.final.spl for c in cases])
    soft = np.array([c.soft_progress if not c.online.success else c.online.spl for c in cases])
    outcome_counts = {"both": 0, "final_only": 0, "incremental_only": 0, "neither": 0}
    for c in cases:
        key = {
            (True, True): "both",
            (False, True): "final_only",
            (True, False): "incremental_only",
            (False, False): "neither",
        }[(c.online.success, c.final.success)]
        outcome_counts[key] += 1

    return Report(
        score=float(np.average(online_spl, weights=weights)),
        score_soft=float(np.average(soft, weights=weights)),
        final_score=float(np.average(final_spl, weights=weights)),
        n_cases=len(cases),
        n_success=sum(c.online.success for c in cases),
        n_success_final=sum(c.final.success for c in cases),
        outcome_counts=outcome_counts,
        plan_ms=metrics.timing_stats([c.online.plan_ms for c in cases]),
        map_update_ms=metrics.timing_stats([c.map_update_ms for c in cases]),
        datasets=datasets,
        config=asdict(cfg),
    )
