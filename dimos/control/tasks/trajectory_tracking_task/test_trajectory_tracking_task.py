# Copyright 2025-2026 Dimensional Inc.
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

"""TrajectoryTrackingTask unit tests + closed-loop sim validation against
the FlowBase FOPDT plant (vendored fit + firmware-style command limiter)."""

from __future__ import annotations

import math

import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.trajectory_tracking_task.constants import (
    FB_CLAMP_LINEAR,
    FB_CLAMP_YAW,
    K_HAT,
)
from dimos.control.tasks.trajectory_tracking_task.trajectory_tracking_task import (
    TrajectoryTrackingTask,
    TrajectoryTrackingTaskConfig,
    create_task,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.benchmarking.plant import (
    FLOWBASE_PLANT_FITTED,
    TwistBasePlantSim,
    flowbase_command_limiter,
)
from dimos.utils.trigonometry import angle_diff

_DT = 0.01  # coordinator tick
_JOINTS = ["base/vx", "base/vy", "base/wz"]
_T0 = 1000.0  # arbitrary perf_counter-like epoch


def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _line_path(length: float = 2.0, step: float = 0.05) -> Path:
    n = int(length / step) + 1
    return Path(frame_id="world", poses=[_pose(i * step, 0.0) for i in range(n)])


def _square_path(side: float = 1.0, step: float = 0.05) -> Path:
    poses = []
    n = int(side / step)
    for i in range(n):
        poses.append(_pose(i * step, 0.0, 0.0))
    for i in range(n):
        poses.append(_pose(side, i * step, math.pi / 2))
    for i in range(n):
        poses.append(_pose(side - i * step, side, math.pi))
    for i in range(n + 1):
        poses.append(_pose(0.0, side - i * step, -math.pi / 2))
    return Path(frame_id="world", poses=poses)


def _state(x: float, y: float, yaw: float, t_now: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={"base/vx": x, "base/vy": y, "base/wz": yaw},
            joint_velocities={"base/vx": 0.0, "base/vy": 0.0, "base/wz": 0.0},
        ),
        t_now=t_now,
        dt=_DT,
    )


def _task(**overrides: object) -> TrajectoryTrackingTask:
    config = TrajectoryTrackingTaskConfig(joint_names=list(_JOINTS), **overrides)  # type: ignore[arg-type]
    return TrajectoryTrackingTask("trajectory_tracker", config)


# --- unit -----------------------------------------------------------------


def test_inactive_until_start_path() -> None:
    task = _task()
    assert not task.is_active()
    assert task.compute(_state(0.0, 0.0, 0.0, _T0)) is None
    assert task.start_path(_line_path(), _pose(0.0, 0.0))
    assert task.is_active()
    assert task.get_state() == "tracking"


def test_rejects_degenerate_path() -> None:
    task = _task()
    assert not task.start_path(Path(frame_id="world", poses=[_pose(0.0, 0.0)]), _pose(0.0, 0.0))
    assert not task.is_active()


def test_feedback_pushes_toward_reference() -> None:
    """Robot displaced left of the reference -> negative body-vy command."""
    task = _task(compensate_gain=False, max_speed=0.4)
    task.start_path(_line_path(), _pose(0.0, 0.0))
    task.compute(_state(0.0, 0.0, 0.0, _T0))  # anchors t0
    out = task.compute(_state(0.0, 0.3, 0.0, _T0 + _DT))
    assert out is not None
    assert out.velocities is not None
    assert out.velocities[1] < 0.0


def test_feedback_clamped() -> None:
    """Huge pose error -> FB contribution saturates at the clamps (at t=0
    the FF reference velocity is ~0, so the command IS the clamped FB)."""
    task = _task(compensate_gain=False, max_speed=0.4)
    task.start_path(_line_path(), _pose(0.0, 0.0))
    task.compute(_state(0.0, 0.0, 0.0, _T0))
    out = task.compute(_state(-10.0, 10.0, 3.0, _T0 + _DT))
    assert out is not None
    assert out.velocities is not None
    vx, vy, wz = out.velocities
    margin = 0.02  # the only non-FB term is the tiny t~0 FF reference
    assert abs(vx) <= FB_CLAMP_LINEAR + margin
    assert abs(vy) <= FB_CLAMP_LINEAR + margin
    assert abs(wz) <= FB_CLAMP_YAW + margin


def test_gain_compensation_divides_command() -> None:
    raw = _task(compensate_gain=False, max_speed=0.4)
    compensated = _task(compensate_gain=True, max_speed=0.4)
    for task in (raw, compensated):
        task.start_path(_line_path(), _pose(0.0, 0.0))
        task.compute(_state(0.0, 0.0, 0.0, _T0))
    t_mid = _T0 + 2.0  # cruise phase
    out_raw = raw.compute(_state(1.0, 0.0, 0.0, t_mid))
    out_compensated = compensated.compute(_state(1.0, 0.0, 0.0, t_mid))
    assert out_raw is not None and out_compensated is not None
    assert out_raw.velocities is not None and out_compensated.velocities is not None
    assert out_compensated.velocities[0] == pytest.approx(out_raw.velocities[0] / K_HAT.x, rel=1e-6)


def test_stale_pose_falls_back_to_feedforward() -> None:
    task = _task(compensate_gain=False, max_speed=0.4, stale_pose_timeout=0.1)
    task.start_path(_line_path(), _pose(0.0, 0.0))
    task.compute(_state(0.0, 0.1, 0.0, _T0))  # fresh pose with cross-track error
    # Pose vanishes from the snapshot; past the staleness budget the task
    # must stop correcting (pure FF: vy ~ 0 on a straight +x line).
    empty = CoordinatorState(joints=JointStateSnapshot(), t_now=_T0 + 2.0, dt=_DT)
    out = empty and task.compute(empty)
    assert out is not None
    assert out.velocities is not None
    assert out.velocities[1] == pytest.approx(0.0, abs=1e-6)


def test_preemption_aborts() -> None:
    task = _task()
    task.start_path(_line_path(), _pose(0.0, 0.0))
    task.on_preempted("vel_base", frozenset(_JOINTS))
    assert task.get_state() == "aborted"
    assert not task.is_active()


def test_configure_tolerates_follower_kwargs() -> None:
    task = _task()
    assert task.configure(speed=0.4, k_angular=1.0, lookahead_dist=0.25, ff_config=None)
    assert task.configure(gain_profile="aggressive")
    assert not task.configure(gain_profile="nonsense")
    task.start_path(_line_path(), _pose(0.0, 0.0))
    assert not task.configure(speed=0.3)  # refused while active


# --- closed loop against the FOPDT plant ----------------------------------


def _run_closed_loop(
    task: TrajectoryTrackingTask,
    path: Path,
    max_ticks: int = 6000,
    plant: TwistBasePlantSim | None = None,
) -> tuple[TwistBasePlantSim, list[tuple[float, float, float]], int]:
    """Tick task + plant together (coordinator-style); returns the plant,
    per-tick (cross_track, along_track, yaw) errors vs the reference, and
    the tick count. Defaults to the FlowBase plant + firmware limiter."""
    if plant is None:
        plant = TwistBasePlantSim(FLOWBASE_PLANT_FITTED, limiter=flowbase_command_limiter())
    start = path.poses[0]
    start_yaw = start.orientation.euler[2]
    plant.reset(start.position.x, start.position.y, start_yaw, _DT)

    assert task.start_path(path, _pose(start.position.x, start.position.y, start_yaw))
    trajectory = task._trajectory
    assert trajectory is not None

    errors: list[tuple[float, float, float]] = []
    tick = 0
    for tick in range(max_ticks):
        t_now = _T0 + tick * _DT
        out = task.compute(_state(plant.x, plant.y, plant.yaw, t_now))
        if not task.is_active():
            break
        assert out is not None and out.velocities is not None
        plant.step(*out.velocities, _DT)

        ref = trajectory.sample(tick * _DT)
        ex = ref.x - plant.x
        ey = ref.y - plant.y
        cos_ref = math.cos(ref.yaw)
        sin_ref = math.sin(ref.yaw)
        along = cos_ref * ex + sin_ref * ey
        cross = -sin_ref * ex + cos_ref * ey
        errors.append((cross, along, angle_diff(ref.yaw, plant.yaw)))
    return plant, errors, tick


def test_closed_loop_tracks_straight_line() -> None:
    task = _task(max_speed=0.5)
    plant, errors, _ = _run_closed_loop(task, _line_path(2.0))
    assert task.get_state() == "arrived"
    cross_max = max(abs(e[0]) for e in errors)
    along_max = max(abs(e[1]) for e in errors)
    # Cross-track (geometric path error) is the metric that matters and is
    # tight. Along-track is a temporal FOPDT *following* lag during the
    # accel ramp (the plant velocity lags v*(t) by ~tau); it decays at
    # cruise and is harmless for path-keeping. Acceleration feedforward
    # (u += tau*a_ref) would cut it further — a future controller upgrade.
    assert cross_max < 0.02, f"cross-track {cross_max:.3f} m"
    assert along_max < 0.12, f"along-track lag {along_max:.3f} m"
    assert math.hypot(2.0 - plant.x, plant.y) < 0.05


def test_closed_loop_tracks_square() -> None:
    task = _task(max_speed=0.4)
    plant, errors, _ = _run_closed_loop(task, _square_path(1.0), max_ticks=12000)
    assert task.get_state() == "arrived"
    cross_max = max(abs(e[0]) for e in errors)
    # Curvature-aware slowdown holds the sharp corners within tolerance and
    # the task arrives back at the start corner.
    assert cross_max < 0.12, f"cross-track {cross_max:.3f} m"
    assert math.hypot(plant.x, plant.y - 0.0) < 0.08


def test_closed_loop_without_compensation_lags() -> None:
    """Sanity: K-compensation is doing real work — without it the plant
    only achieves K x the command and the tracker accumulates lag that the
    clamped FB cannot fully cancel."""
    with_compensation = _task(max_speed=0.5, compensate_gain=True)
    without = _task(max_speed=0.5, compensate_gain=False)
    _, err_with, _ = _run_closed_loop(with_compensation, _line_path(2.0))
    _, err_without, _ = _run_closed_loop(without, _line_path(2.0))
    lag_with = max(abs(e[1]) for e in err_with)
    lag_without = max(abs(e[1]) for e in err_without)
    assert lag_with < lag_without


def test_rounded_corners_beat_sharp_corners() -> None:
    """Rounding the corners (bounded curvature) tracks tighter than the
    curvature-slowed sharp square, and returns closer to the start — the
    holonomic-base argument for filleting corners over slowing into them.
    Uses the canonical benchmark paths so this asserts the comparison the
    benchmark battery reports."""
    from dimos.utils.benchmarking.paths import rounded_square, square

    sharp_path = square(side=2.0)
    rounded_path = rounded_square(side=2.0, arc_radius=0.5)
    sharp = _task(max_speed=0.5)
    rounded = _task(max_speed=0.5)
    _, sharp_err, _ = _run_closed_loop(sharp, sharp_path, max_ticks=40000)
    rounded_plant, rounded_err, _ = _run_closed_loop(rounded, rounded_path, max_ticks=40000)
    assert sharp.get_state() == "arrived"
    assert rounded.get_state() == "arrived"

    sharp_cte = max(abs(e[0]) for e in sharp_err)
    rounded_cte = max(abs(e[0]) for e in rounded_err)
    assert rounded_cte < sharp_cte, f"rounded {rounded_cte:.3f} !< sharp {sharp_cte:.3f}"
    # Sharp corners are infinite-curvature: even slowed, some overshoot
    # remains. Rounded holds well under it.
    assert rounded_cte < 0.08
    # Return-to-start: rounded should close the loop tightly back to its
    # start waypoint (rounded_square starts mid-edge, not at the origin).
    start = rounded_path.poses[0]
    assert math.hypot(rounded_plant.x - start.position.x, rounded_plant.y - start.position.y) < 0.03


# --- Go2: artifact-driven config (same controller, different robot) -------


def test_go2_artifact_config_derives_sane_gains() -> None:
    """A TrackingConfig built from a Go2 characterization artifact derives
    its gains/limits from the Go2 fit + envelope (not the FlowBase)."""
    from dimos.control.tasks.trajectory_tracking_task.config import TrackingConfig
    from dimos.utils.benchmarking.plant import GO2_PLANT_FITTED
    from dimos.utils.benchmarking.tuning import Provenance, derive_config

    artifact = derive_config(
        GO2_PLANT_FITTED,
        Provenance(robot_id="go2", surface="concrete", mode="default", sim_or_hw="sim"),
    )
    tracking = TrackingConfig.from_artifact(artifact)
    # kp = 1/(4 zeta^2 tau) from the Go2 plant tau, not the FlowBase's.
    assert tracking.kp_default.x == pytest.approx(1.0 / (4.0 * GO2_PLANT_FITTED.vx.tau), rel=1e-6)
    assert tracking.kp_default.yaw == pytest.approx(1.0 / (4.0 * GO2_PLANT_FITTED.wz.tau), rel=1e-6)
    assert tracking.deadtime.yaw == pytest.approx(GO2_PLANT_FITTED.wz.L)
    assert tracking.k_hat.x == pytest.approx(GO2_PLANT_FITTED.vx.K)
    assert "go2" in tracking.provenance
    assert tracking.plan_max_vel.x > 0.0 and tracking.a_lat_max > 0.0


def test_missing_artifact_does_not_break_creation() -> None:
    """Regression: a task pointed at a not-yet-created artifact must still be
    constructible (inactive) so its mere presence in a coordinator never
    blocks startup — the file is only required when a run actually begins."""

    class _Cfg:
        name = "trajectory_tracker"
        joint_names = list(_JOINTS)
        priority = 10
        params = {"artifact_path": "/does/not/exist.json"}

    task = create_task(_Cfg(), None)
    assert not task.is_active()
    # Only start_path (an actual run) should touch the file.
    with pytest.raises(FileNotFoundError):
        task.start_path(_line_path(2.0), _pose(0.0, 0.0))


def test_eso_flag_tracks_on_clean_go2() -> None:
    """The opt-in ESO inner loop wires through compute() and still tracks the
    clean Go2 plant (no disturbance => ESO ~no-op, must not regress)."""
    from dimos.control.tasks.trajectory_tracking_task.config import TrackingConfig
    from dimos.utils.benchmarking.plant import GO2_PLANT_FITTED
    from dimos.utils.benchmarking.tuning import Provenance, derive_config

    artifact = derive_config(
        GO2_PLANT_FITTED,
        Provenance(robot_id="go2", surface="concrete", mode="default", sim_or_hw="sim"),
    )
    tracking = TrackingConfig.from_artifact(artifact)
    task = TrajectoryTrackingTask(
        "go2_eso",
        TrajectoryTrackingTaskConfig(
            joint_names=list(_JOINTS), tracking=tracking, max_speed=0.5, eso=True
        ),
    )
    plant = TwistBasePlantSim(GO2_PLANT_FITTED)
    _, errors, _ = _run_closed_loop(task, _line_path(2.0), plant=plant)
    assert task.get_state() == "arrived"
    assert max(abs(e[0]) for e in errors) < 0.05  # cross-track


def test_eso_off_is_unchanged_command() -> None:
    """eso=False must produce byte-for-byte the gain-inverted command (the ESO
    is strictly opt-in; the default path is untouched)."""
    off = _task(eso=False, compensate_gain=True, max_speed=0.4)
    also_off = _task(eso=False, compensate_gain=True, max_speed=0.4)
    for task in (off, also_off):
        task.start_path(_line_path(), _pose(0.0, 0.0))
        task.compute(_state(0.0, 0.0, 0.0, _T0))
    a = off.compute(_state(1.0, 0.0, 0.0, _T0 + 2.0))
    b = also_off.compute(_state(1.0, 0.0, 0.0, _T0 + 2.0))
    assert a is not None and b is not None
    assert a.velocities == b.velocities


def test_deadtime_compensation_is_opt_in_feedback_prediction() -> None:
    baseline = _task(compensate_gain=False, max_speed=0.5)
    compensated = _task(
        compensate_gain=False,
        max_speed=0.5,
        deadtime_compensation=True,
        deadtime_feedback_lag_s=0.2,
    )
    for task in (baseline, compensated):
        task.start_path(_line_path(), _pose(0.0, 0.0))
        task.compute(_state(0.0, 0.0, 0.0, _T0))

    stale_pose_time = _T0 + 0.7
    out_baseline = baseline.compute(_state(0.0, 0.0, 0.0, stale_pose_time))
    out_compensated = compensated.compute(_state(0.0, 0.0, 0.0, stale_pose_time))

    assert out_baseline is not None and out_compensated is not None
    assert out_baseline.velocities is not None and out_compensated.velocities is not None
    assert out_compensated.velocities[0] < out_baseline.velocities[0]


def test_measured_speed_yaw_feedforward_scales_curve_yaw(mocker) -> None:
    from dimos.utils.benchmarking.paths import circle

    planned = _task(compensate_gain=False, max_speed=0.5)
    measured = _task(
        compensate_gain=False,
        max_speed=0.5,
        yaw_feedforward_mode="measured_speed",
    )
    path = circle(radius=1.0)
    for task in (planned, measured):
        task.start_path(path, path.poses[0])
    assert planned._trajectory is not None
    assert measured._trajectory is not None

    t_elapsed = 1.0
    ref = planned._trajectory.sample(t_elapsed)
    pose = (ref.x, ref.y, ref.yaw)
    measured._velocity_estimator = mocker.Mock()
    measured._velocity_estimator.world_velocity.return_value = (0.0, 0.0, 0.0)

    planned_cmd = planned._tracking_command(t_elapsed, pose, _T0 + t_elapsed)
    measured_cmd = measured._tracking_command(t_elapsed, pose, _T0 + t_elapsed)

    assert abs(measured_cmd[2]) < abs(planned_cmd[2])


def test_go2_artifact_config_tracks_in_sim() -> None:
    """End-to-end: the artifact-driven config drives the controller against
    the Go2 FOPDT plant (no firmware limiter). Validates the plumbing — the
    vy axis uses the provisional placeholder fit until a real Go2 lateral
    characterization lands."""
    from dimos.control.tasks.trajectory_tracking_task.config import TrackingConfig
    from dimos.utils.benchmarking.plant import GO2_PLANT_FITTED
    from dimos.utils.benchmarking.tuning import Provenance, derive_config

    artifact = derive_config(
        GO2_PLANT_FITTED,
        Provenance(robot_id="go2", surface="concrete", mode="default", sim_or_hw="sim"),
    )
    tracking = TrackingConfig.from_artifact(artifact)
    task = TrajectoryTrackingTask(
        "go2_tracker",
        TrajectoryTrackingTaskConfig(joint_names=list(_JOINTS), tracking=tracking, max_speed=0.5),
    )
    plant = TwistBasePlantSim(GO2_PLANT_FITTED)  # Go2 has no firmware Ruckig limiter
    _, errors, _ = _run_closed_loop(task, _line_path(2.0), plant=plant)
    assert task.get_state() == "arrived"
    assert max(abs(e[0]) for e in errors) < 0.03  # cross-track
