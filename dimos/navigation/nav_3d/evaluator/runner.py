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

Every case is planned twice: on the golden map (planner ceiling) and on the
map the online mapper built from the recording (end to end). Both paths must
pass the golden collision gate. The headline score is validity-gated SPL on
the online map.
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field
import itertools
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.golden import (
    keys_contain,
    load_or_build_golden,
    voxel_keys,
)
from dimos.navigation.nav_3d.evaluator.recording import load_trajectory
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Case, Suite
    from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner

logger = setup_logger()

MAX_COLLISIONS_KEPT = 50


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
class CaseResult:
    id: str
    dataset: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    weight: float
    tags: list[str]
    l_ref: float
    l_ref_snapped: bool
    online: PlanOutcome
    golden: PlanOutcome
    attribution: str
    soft_progress: float


@dataclass
class PlannerArtifacts:
    """Graph state of one planner after its map update. Not serialized to JSON."""

    surface_clearance: NDArray[np.float32]
    nodes: NDArray[np.float32]
    edges: NDArray[np.float32]


@dataclass
class DatasetResult:
    dataset: str
    cases: list[CaseResult]
    walked_path_valid: bool
    false_obstacle_rate: float
    online_voxels: int
    golden_voxels: int
    map_build_ms: float
    add_frame_ms: dict[str, float]
    frames: int
    online_artifacts: PlannerArtifacts | None = None
    golden_artifacts: PlannerArtifacts | None = None


@dataclass
class Report:
    score: float
    score_soft: float
    planner_score: float
    false_obstacle_rate: float
    n_cases: int
    n_success: int
    attribution_counts: dict[str, int]
    plan_ms: dict[str, float]
    datasets: list[DatasetResult]
    config: dict[str, float | int] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        out = asdict(self)
        for dataset in out["datasets"]:
            dataset.pop("online_artifacts")
            dataset.pop("golden_artifacts")
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


def _attribution(online: PlanOutcome, golden: PlanOutcome) -> str:
    if online.success:
        return "ok"
    if not golden.success:
        return "planner"
    return "mapper"


def run_suite(suite: Suite, cfg: EvalConfig) -> DatasetResult:
    db_path = resolve_named_path(suite.dataset, ".db")
    trajectory = load_trajectory(db_path, suite.odom_stream)
    golden = load_or_build_golden(
        db_path,
        suite,
        cfg,
        corridor_radius=cfg.robot_radius + 0.1,
        corridor_z_lo=-cfg.robot_height,
        corridor_z_hi=-cfg.robot_height + cfg.body_clearance + cfg.voxel_size,
    )
    obstacle_keys = golden.obstacle_keys()

    # Calibration invariant: the physically walked path must pass the gate.
    # A failure here means the gate or corridor geometry is wrong, not the planner.
    foot_path = trajectory.positions - np.array([0.0, 0.0, cfg.robot_height], dtype=np.float32)
    walked_gate = metrics.check_path(
        foot_path,
        obstacle_keys,
        cfg.voxel_size,
        cfg.robot_radius,
        cfg.ground_margin,
        cfg.body_clearance,
    )
    if not walked_gate.valid:
        logger.warning(
            "%s: walked trajectory fails the collision gate at %d samples; "
            "case validity is unreliable",
            suite.dataset,
            len(walked_gate.collision_points),
        )

    # The online map equals the golden map while both use the same mapper
    # config, so reuse it instead of replaying the recording a second time.
    # Tier 2 replay and a separate golden mapper config will change this.
    online_points = golden.occupied
    online_occupied = keys_contain(
        np.sort(voxel_keys(online_points, cfg.voxel_size)), golden.walked_keys
    )
    false_obstacle_rate = float(online_occupied.mean()) if len(golden.walked_keys) else 0.0

    golden_planner = cfg.make_planner()
    golden_planner.update_global_map(golden.occupied)
    online_planner = cfg.make_planner()
    online_planner.update_global_map(online_points)

    def snapshot(planner: MLSPlanner) -> PlannerArtifacts:
        return PlannerArtifacts(
            surface_clearance=planner.surface_clearance_map(),
            nodes=planner.nodes(),
            edges=planner.node_edges(),
        )

    results: list[CaseResult] = []
    for case in suite.cases:
        if case.l_ref is not None:
            l_ref, snapped = case.l_ref, True
        else:
            l_ref, snapped = metrics.reference_length(
                trajectory, case.start, case.goal, cfg.robot_height
            )
            if not snapped:
                logger.warning(
                    "%s/%s: start or goal is off the walked trajectory; "
                    "using straight-line reference",
                    suite.dataset,
                    case.id,
                )
        golden_out, _ = _run_plan(golden_planner, case, l_ref, obstacle_keys, cfg)
        online_out, online_wp = _run_plan(online_planner, case, l_ref, obstacle_keys, cfg)
        end = online_wp[-1] if online_wp is not None and len(online_wp) else None
        results.append(
            CaseResult(
                id=case.id,
                dataset=suite.dataset,
                start=case.start,
                goal=case.goal,
                weight=case.weight,
                tags=case.tags,
                l_ref=l_ref,
                l_ref_snapped=snapped,
                online=online_out,
                golden=golden_out,
                attribution=_attribution(online_out, golden_out),
                soft_progress=metrics.soft_progress(end, case.start, case.goal),
            )
        )

    return DatasetResult(
        dataset=suite.dataset,
        cases=results,
        walked_path_valid=walked_gate.valid,
        false_obstacle_rate=false_obstacle_rate,
        online_voxels=len(online_points),
        golden_voxels=len(golden.occupied),
        map_build_ms=golden.build_ms,
        add_frame_ms=golden.add_frame_ms,
        frames=golden.frames,
        online_artifacts=snapshot(online_planner),
        golden_artifacts=snapshot(golden_planner),
    )


def evaluate(suites: list[Suite], cfg: EvalConfig | None = None, workers: int = 1) -> Report:
    cfg = cfg or EvalConfig()
    if workers > 1 and len(suites) > 1:
        with ProcessPoolExecutor(max_workers=min(workers, len(suites))) as pool:
            datasets = list(pool.map(run_suite, suites, itertools.repeat(cfg)))
    else:
        datasets = [run_suite(suite, cfg) for suite in suites]
    cases = [c for d in datasets for c in d.cases]
    if not cases:
        raise ValueError("no cases to evaluate")

    weights = np.array([c.weight for c in cases])
    online_spl = np.array([c.online.spl for c in cases])
    golden_spl = np.array([c.golden.spl for c in cases])
    soft = np.array([c.soft_progress if not c.online.success else c.online.spl for c in cases])
    attribution_counts: dict[str, int] = {}
    for c in cases:
        attribution_counts[c.attribution] = attribution_counts.get(c.attribution, 0) + 1

    rates = [d.false_obstacle_rate for d in datasets]
    return Report(
        score=float(np.average(online_spl, weights=weights)),
        score_soft=float(np.average(soft, weights=weights)),
        planner_score=float(np.average(golden_spl, weights=weights)),
        false_obstacle_rate=float(np.mean(rates)) if rates else 0.0,
        n_cases=len(cases),
        n_success=sum(c.online.success for c in cases),
        attribution_counts=attribution_counts,
        plan_ms=metrics.timing_stats([c.online.plan_ms for c in cases]),
        datasets=datasets,
        config=asdict(cfg),
    )
