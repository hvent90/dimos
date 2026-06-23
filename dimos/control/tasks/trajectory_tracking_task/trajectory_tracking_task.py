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

"""Trajectory-tracking ControlTask for the FlowBase holonomic base.

Control stack (design certified against the 2026-06-09 FOPDT fit):

    waypoint path
      -> TimedTrajectory: time-parameterized reference (trapezoidal,
         accel-limited, 85% planning margins)
      -> feedforward: reference world velocity sampled at t + L per axis
         (dead-time preview), rotated into the body frame
      -> feedback: per-axis P on pose error (world error rotated into the
         body frame by current yaw), clamped so FF carries the trajectory
      -> plant input compensation: u_cmd = u_phys / K_hat (toggleable,
         reuses FeedforwardGainCompensator)
      -> JointCommandOutput (VELOCITY) at the coordinator tick rate

The base is holonomic, so the three axes are decoupled SISO loops. No
integral, no derivative. Gains and limits all trace to ``constants.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Literal

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainCompensator
from dimos.control.tasks.trajectory_tracking_task.config import (
    TrackingConfig,
    tracking_config_from_artifact_path,
)
from dimos.control.tasks.trajectory_tracking_task.constants import FLOWBASE_TRACKING
from dimos.control.tasks.trajectory_tracking_task.deadtime_predictor import (
    DeadtimePosePredictor,
    build_deadtime_predictor,
)
from dimos.control.tasks.trajectory_tracking_task.eso import ESOCompensator, build_eso
from dimos.control.tasks.trajectory_tracking_task.gain_schedule import ScheduledGainCompensator
from dimos.control.tasks.trajectory_tracking_task.velocity_estimator import BodyVelocityEstimator
from dimos.control.tasks.trajectory_tracking_task.trajectory_generator import (
    TimedTrajectory,
    TrajectorySample,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()

TrajectoryTrackingState = Literal["idle", "tracking", "holding", "arrived", "aborted"]

GainProfile = Literal["default", "aggressive"]
YawFeedforwardMode = Literal["planned", "measured_speed"]


@dataclass
class TrajectoryTrackingTaskConfig:
    joint_names: list[str] = field(default_factory=lambda: ["base/vx", "base/vy", "base/wz"])
    priority: int = 20
    # Per-robot gains/limits (plant fit). Defaults to the FlowBase. Either
    # inject a TrackingConfig directly, or set ``artifact_path`` to load one
    # from a characterization JSON (lazily, on the first start_path).
    tracking: TrackingConfig = FLOWBASE_TRACKING
    artifact_path: str | None = None
    # Cruise-speed cap for the trajectory profile; the generator clamps it
    # to the planning margins regardless.
    max_speed: float | None = None
    gain_profile: GainProfile = "default"
    # Plant-gain inversion (u_cmd = u_phys / K_hat). On by default — the
    # base genuinely moves K x the command.
    compensate_gain: bool = True
    # Opt-in ESO/ADRC disturbance-rejection inner loop (see eso.py). OFF =
    # today's behavior exactly (plain gain inversion). When ON, the ESO
    # REPLACES the gain inversion at the velocity-command layer and is fed the
    # smoothed/differentiated measured body velocity. eso_bandwidth is the
    # single precision(>1)-vs-robustness(<1) dial.
    eso: bool = False
    eso_bandwidth: float = 1.0
    # Opt-in feedback dead-time compensation. OFF = measured-pose feedback as
    # today. When ON, the feedback error uses a measured pose advanced by the
    # nominal FOPDT command-history model.
    deadtime_compensation: bool = False
    deadtime_feedback_lag_s: float = 0.0
    deadtime_prediction_blend: float = 1.0
    deadtime_prediction_mode: Literal["full", "yaw_only"] = "full"
    # "planned" tracks the time-parameterized tangent yaw rate. "measured_speed"
    # keeps the geometric curvature but scales yaw FF by measured along-path
    # speed, so yaw does not run ahead when the Go2 falls behind in translation.
    yaw_feedforward_mode: YawFeedforwardMode = "planned"
    heading_mode: Literal["tangent", "fixed"] = "tangent"
    fixed_heading: float = 0.0
    # Arrival tolerances for the hold phase.
    goal_tolerance: float = 0.05
    orientation_tolerance: float = 0.1
    # If the pose in CoordinatorState goes stale for longer than this,
    # fall back to FF-only (no feedback on a frozen error).
    stale_pose_timeout: float = 0.3
    # Throttle the COMMAND-UPDATE rate (Hz): recompute every 1/command_rate_hz s
    # and hold the command in between, while the coordinator keeps ticking at
    # full rate. None = a fresh command every tick. Lets us A/B the command rate
    # against feedback freshness without touching tick_rate.
    command_rate_hz: float | None = None


class TrajectoryTrackingTask(BaseControlTask):
    """FF + per-axis P trajectory tracker (holonomic twist base)."""

    def __init__(self, name: str, config: TrajectoryTrackingTaskConfig) -> None:
        if len(config.joint_names) != 3:
            raise ValueError(
                f"TrajectoryTrackingTask '{name}' needs 3 joints (vx, vy, wz), "
                f"got {len(config.joint_names)}"
            )
        self._name = name
        self._config = config
        self._tracking = config.tracking
        # Artifact (if any) is loaded LAZILY on the first start_path so a
        # missing/not-yet-characterized file never breaks coordinator startup
        # (the task is inactive until a run begins). Mirrors precision_follower.
        self._artifact_path = config.artifact_path
        self._artifact_loaded = False
        self._joint_names_list = list(config.joint_names)
        self._joint_names = frozenset(config.joint_names)
        self._kp = self._tracking.kp(config.gain_profile)
        self._compensator: FeedforwardGainCompensator | ScheduledGainCompensator | None = (
            self._build_compensator()
        )
        # Velocity estimator is needed by ESO and by measured-speed yaw FF.
        self._eso: ESOCompensator | None = None
        self._velocity_estimator: BodyVelocityEstimator | None = None
        self._last_eso_t: float | None = None
        self._rebuild_velocity_estimator()
        self._rebuild_eso()
        self._deadtime_predictor: DeadtimePosePredictor | None = None
        self._rebuild_deadtime_predictor()

        self._state: TrajectoryTrackingState = "idle"
        self._trajectory: TimedTrajectory | None = None
        # t0 anchors at the first compute() after start_path (state.t_now).
        self._t0: float | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._last_pose_t: float | None = None
        # Command-rate throttle (hold the command between recomputes).
        self._command_period = 1.0 / config.command_rate_hz if config.command_rate_hz else 0.0
        self._last_command: JointCommandOutput | None = None
        self._last_command_t: float | None = None

    # ------------------------------------------------------------------
    # ControlTask protocol
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.VELOCITY,
        )

    def is_active(self) -> bool:
        return self._state in ("tracking", "holding")

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self.is_active() or self._trajectory is None:
            return None
        # Command-rate throttle: hold the last command until a full period has
        # elapsed, so the robot is commanded at command_rate_hz while the
        # coordinator still ticks at full rate.
        if (
            self._command_period > 0.0
            and self._last_command is not None
            and self._last_command_t is not None
            and state.t_now - self._last_command_t < self._command_period
        ):
            return self._last_command
        if self._t0 is None:
            self._t0 = state.t_now
        t_elapsed = state.t_now - self._t0

        pose = self._read_pose(state)
        pose_fresh = pose is not None
        if pose is not None:
            self._last_pose = pose
            self._last_pose_t = state.t_now
            # Feed the ESO's body-velocity estimator only on genuinely fresh
            # odom reads (held/duplicate poses are de-duped inside it).
            if self._velocity_estimator is not None:
                self._velocity_estimator.update(state.t_now, pose[0], pose[1], pose[2])
        elif (
            self._last_pose_t is not None
            and state.t_now - self._last_pose_t < self._config.stale_pose_timeout
        ):
            pose = self._last_pose
            pose_fresh = True  # within the staleness budget — still usable

        if self._state == "tracking" and t_elapsed >= self._trajectory.duration:
            self._state = "holding"

        if self._state == "tracking":
            vx, vy, wz = self._tracking_command(
                t_elapsed, pose if pose_fresh else None, state.t_now
            )
        else:
            vx, vy, wz = self._holding_command(pose if pose_fresh else None)
            if self._state == "arrived":
                vx, vy, wz = 0.0, 0.0, 0.0

        if self._eso is not None:
            # ADRC inner loop: makes the proven outer controller see a clean
            # plant by estimating and cancelling the Go2's total disturbance.
            # Replaces the steady-state gain inversion (which it reduces to in
            # the disturbance-free limit). Runs at the command-recompute rate;
            # dt is the time since the last recompute.
            measured = None
            if pose_fresh and pose is not None and self._velocity_estimator is not None:
                measured = self._velocity_estimator.body_velocity(pose[2])
            dt_eso = state.t_now - self._last_eso_t if self._last_eso_t is not None else 0.0
            vx, vy, wz = self._eso.compute((vx, vy, wz), measured, dt_eso)
            self._last_eso_t = state.t_now
        elif self._compensator is not None:
            vx, vy, wz = self._compensator.compute(vx, vy, wz)

        command = JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[vx, vy, wz],
            mode=ControlMode.VELOCITY,
        )
        if self._deadtime_predictor is not None:
            self._deadtime_predictor.record_command(state.t_now, (vx, vy, wz))
        self._last_command = command
        self._last_command_t = state.t_now
        return command

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names and self.is_active():
            logger.warning(f"TrajectoryTrackingTask '{self._name}' preempted by {by_task}")
            self._state = "aborted"

    # ------------------------------------------------------------------
    # Control law
    # ------------------------------------------------------------------

    def _build_compensator(
        self,
    ) -> FeedforwardGainCompensator | ScheduledGainCompensator | None:
        """Pick the gain-inversion block: speed-scheduled when the config
        carries a schedule (nonlinear plant), else constant-K. None if
        compensation is off."""
        if not self._config.compensate_gain:
            return None
        if self._tracking.schedule is not None:
            return ScheduledGainCompensator(
                self._tracking.schedule, self._tracking.ff_output_limit.as_tuple()
            )
        return FeedforwardGainCompensator(self._tracking.feedforward_config())

    def _compensator_name(self) -> str:
        if self._eso is not None:
            return "eso"
        if isinstance(self._compensator, ScheduledGainCompensator):
            return "scheduled-gain"
        if isinstance(self._compensator, FeedforwardGainCompensator):
            return "constant-gain"
        return "none"

    def _rebuild_eso(self) -> None:
        """(Re)build the ESO inner loop + its velocity estimator from the
        current tracking config. No-op block when ``config.eso`` is off, so
        the gain-inversion path above is what runs (today's behavior)."""
        if self._config.eso:
            self._eso = build_eso(self._tracking, bandwidth=self._config.eso_bandwidth)
        else:
            self._eso = None
        self._last_eso_t = None

    def _rebuild_velocity_estimator(self) -> None:
        if self._config.eso or self._config.yaw_feedforward_mode == "measured_speed":
            self._velocity_estimator = BodyVelocityEstimator()
        else:
            self._velocity_estimator = None

    def _rebuild_deadtime_predictor(self) -> None:
        if self._config.deadtime_compensation:
            self._deadtime_predictor = build_deadtime_predictor(
                self._tracking,
                feedback_lag_s=self._config.deadtime_feedback_lag_s,
                blend=self._config.deadtime_prediction_blend,
                mode=self._config.deadtime_prediction_mode,
            )
        else:
            self._deadtime_predictor = None

    def _read_pose(self, state: CoordinatorState) -> tuple[float, float, float] | None:
        # Twist-base ConnectedHardware routes adapter.read_odometry() ->
        # joint positions [x, y, yaw] (same convention as PathFollowerTask).
        positions = state.joints.joint_positions
        x = positions.get(self._joint_names_list[0])
        y = positions.get(self._joint_names_list[1])
        yaw = positions.get(self._joint_names_list[2])
        if x is None or y is None or yaw is None:
            return None
        return float(x), float(y), float(yaw)

    def _feedback(
        self, reference: TrajectorySample, pose: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        x, y, yaw = pose
        ex_world = reference.x - x
        ey_world = reference.y - y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        ex_body = cos_yaw * ex_world + sin_yaw * ey_world
        ey_body = -sin_yaw * ex_world + cos_yaw * ey_world
        e_yaw = angle_diff(reference.yaw, yaw)
        return (
            _clamp(self._kp.x * ex_body, self._tracking.fb_clamp_linear),
            _clamp(self._kp.y * ey_body, self._tracking.fb_clamp_linear),
            _clamp(self._kp.yaw * e_yaw, self._tracking.fb_clamp_yaw),
        )

    def _tracking_command(
        self, t_elapsed: float, pose: tuple[float, float, float] | None, t_now: float
    ) -> tuple[float, float, float]:
        assert self._trajectory is not None
        # Per-axis dead-time preview: each axis sees the reference velocity
        # it should be producing L seconds from now.
        deadtime = self._tracking.deadtime
        ref_x = self._trajectory.sample(t_elapsed + deadtime.x)
        ref_y = self._trajectory.sample(t_elapsed + deadtime.y)
        ref_yaw = self._trajectory.sample(t_elapsed + deadtime.yaw)
        ref_now = self._trajectory.sample(t_elapsed)

        yaw = pose[2] if pose is not None else ref_now.yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        ff_vx = cos_yaw * ref_x.vx_world + sin_yaw * ref_x.vy_world
        ff_vy = -sin_yaw * ref_y.vx_world + cos_yaw * ref_y.vy_world
        ff_wz = ref_yaw.omega
        if self._config.yaw_feedforward_mode == "measured_speed":
            ff_wz = self._measured_speed_yaw_ff(ref_yaw)

        if pose is None:
            # Stale pose: feedforward only — never correct against a frozen error.
            return ff_vx, ff_vy, ff_wz

        feedback_pose = pose
        if self._deadtime_predictor is not None:
            feedback_pose = self._deadtime_predictor.predict(pose, t_now)
        fb_vx, fb_vy, fb_wz = self._feedback(ref_now, feedback_pose)
        return ff_vx + fb_vx, ff_vy + fb_vy, ff_wz + fb_wz

    def _measured_speed_yaw_ff(self, reference: TrajectorySample) -> float:
        planned_speed = math.hypot(reference.vx_world, reference.vy_world)
        if planned_speed < 1e-6 or self._velocity_estimator is None:
            return reference.omega
        measured = self._velocity_estimator.world_velocity()
        if measured is None:
            return reference.omega
        vx_world, vy_world, _ = measured
        tangent_x = reference.vx_world / planned_speed
        tangent_y = reference.vy_world / planned_speed
        measured_path_speed = max(0.0, vx_world * tangent_x + vy_world * tangent_y)
        curvature = reference.omega / planned_speed
        return curvature * measured_path_speed

    def _holding_command(
        self, pose: tuple[float, float, float] | None
    ) -> tuple[float, float, float]:
        assert self._trajectory is not None
        end = self._trajectory.end_sample()
        if pose is None:
            return 0.0, 0.0, 0.0
        if (
            math.hypot(end.x - pose[0], end.y - pose[1]) < self._config.goal_tolerance
            and abs(angle_diff(end.yaw, pose[2])) < self._config.orientation_tolerance
        ):
            self._state = "arrived"
            logger.info(f"TrajectoryTrackingTask '{self._name}' arrived")
            return 0.0, 0.0, 0.0
        return self._feedback(end, pose)

    # ------------------------------------------------------------------
    # Public API (called by runner — typically over RPC from a tool)
    # ------------------------------------------------------------------

    def configure(
        self,
        speed: float | None = None,
        gain_profile: str | None = None,
        compensate_gain: bool | None = None,
        eso: bool | None = None,
        eso_bandwidth: float | None = None,
        deadtime_compensation: bool | None = None,
        deadtime_feedback_lag_s: float | None = None,
        deadtime_prediction_blend: float | None = None,
        deadtime_prediction_mode: str | None = None,
        yaw_feedforward_mode: str | None = None,
        heading_mode: str | None = None,
        fixed_heading: float | None = None,
        command_rate_hz: float | None = None,
        **ignored: Any,
    ) -> bool:
        """Override per-run knobs before start_path. Accepts (and logs)
        unknown kwargs so callers built for PathFollowerTask.configure
        (e.g. the benchmark tool's k_angular / lookahead_dist) work
        unchanged."""
        if self.is_active():
            logger.warning(f"TrajectoryTrackingTask '{self._name}': cannot configure while active")
            return False
        if speed is not None:
            self._config.max_speed = speed
        if gain_profile is not None:
            if gain_profile not in ("default", "aggressive"):
                logger.warning(f"unknown gain_profile {gain_profile!r}")
                return False
            self._config.gain_profile = gain_profile  # type: ignore[assignment]
            self._kp = self._tracking.kp(gain_profile)
        if compensate_gain is not None:
            self._config.compensate_gain = compensate_gain
            self._compensator = self._build_compensator()
        if eso is not None:
            self._config.eso = eso
            self._rebuild_velocity_estimator()
            self._rebuild_eso()
        if eso_bandwidth is not None:
            self._config.eso_bandwidth = eso_bandwidth
            self._rebuild_eso()
        if deadtime_compensation is not None:
            self._config.deadtime_compensation = deadtime_compensation
            self._rebuild_deadtime_predictor()
        if deadtime_feedback_lag_s is not None:
            self._config.deadtime_feedback_lag_s = deadtime_feedback_lag_s
            self._rebuild_deadtime_predictor()
        if deadtime_prediction_blend is not None:
            self._config.deadtime_prediction_blend = deadtime_prediction_blend
            self._rebuild_deadtime_predictor()
        if deadtime_prediction_mode is not None:
            if deadtime_prediction_mode not in ("full", "yaw_only"):
                logger.warning(f"unknown deadtime_prediction_mode {deadtime_prediction_mode!r}")
                return False
            self._config.deadtime_prediction_mode = deadtime_prediction_mode  # type: ignore[assignment]
            self._rebuild_deadtime_predictor()
        if yaw_feedforward_mode is not None:
            if yaw_feedforward_mode not in ("planned", "measured_speed"):
                logger.warning(f"unknown yaw_feedforward_mode {yaw_feedforward_mode!r}")
                return False
            self._config.yaw_feedforward_mode = yaw_feedforward_mode  # type: ignore[assignment]
            self._rebuild_velocity_estimator()
        if heading_mode is not None:
            self._config.heading_mode = heading_mode  # type: ignore[assignment]
        if fixed_heading is not None:
            self._config.fixed_heading = fixed_heading
        if command_rate_hz is not None:
            self._config.command_rate_hz = command_rate_hz
            self._command_period = 1.0 / command_rate_hz if command_rate_hz > 0 else 0.0
        if ignored:
            logger.info(
                f"TrajectoryTrackingTask '{self._name}': ignoring follower-specific "
                f"configure kwargs {sorted(ignored)}"
            )
        return True

    def _ensure_artifact_loaded(self) -> None:
        """Load the characterization artifact on first use and rebuild the
        gains/compensator from it. Deferred from __init__ so startup never
        depends on the file existing."""
        if self._artifact_loaded or not self._artifact_path:
            return
        self._tracking = tracking_config_from_artifact_path(self._artifact_path)
        self._kp = self._tracking.kp(self._config.gain_profile)
        self._compensator = self._build_compensator()
        self._rebuild_eso()
        self._rebuild_deadtime_predictor()
        self._artifact_loaded = True
        logger.info(
            f"TrajectoryTrackingTask '{self._name}' loaded artifact ({self._tracking.provenance})"
        )

    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        if path is None or len(path.poses) < 2:
            logger.warning(f"TrajectoryTrackingTask '{self._name}': invalid path")
            return False
        self._ensure_artifact_loaded()
        start_pose = (
            float(current_odom.position.x),
            float(current_odom.position.y),
            float(current_odom.orientation.euler[2]),
        )
        self._trajectory = TimedTrajectory.from_path(
            path,
            limits=self._tracking.profile_limits,
            max_speed=self._config.max_speed,
            heading_mode=self._config.heading_mode,
            fixed_heading=self._config.fixed_heading,
        )
        if self._compensator is not None:
            self._compensator.reset()
        if self._eso is not None:
            self._eso.reset()
        if self._velocity_estimator is not None:
            self._velocity_estimator.reset()
        if self._deadtime_predictor is not None:
            self._deadtime_predictor.reset(None, start_pose)
        self._last_eso_t = None
        self._t0 = None
        self._last_command = None
        self._last_command_t = None
        self._state = "tracking"
        deadtime_horizon = (
            max(
                self._tracking.deadtime.x,
                self._tracking.deadtime.y,
                self._tracking.deadtime.yaw,
            )
            + self._config.deadtime_feedback_lag_s
        )
        logger.info(
            f"TrajectoryTrackingTask '{self._name}' started: "
            f"{len(path.poses)} poses, {self._trajectory.length:.2f} m, "
            f"{self._trajectory.duration:.2f} s, cruise {self._trajectory.max_speed:.2f} m/s, "
            f"compensator={self._compensator_name()}, "
            f"deadtime_compensation={self._config.deadtime_compensation}, "
            f"deadtime_horizon={deadtime_horizon:.3f}s, "
            f"deadtime_blend={self._config.deadtime_prediction_blend:.2f}, "
            f"deadtime_mode={self._config.deadtime_prediction_mode}, "
            f"yaw_ff={self._config.yaw_feedforward_mode}, "
            f"artifact={self._artifact_path or '<default>'}"
        )
        return True

    def cancel(self) -> bool:
        if not self.is_active():
            return False
        self._state = "aborted"
        return True

    def reset(self) -> bool:
        if self.is_active():
            return False
        self._state = "idle"
        self._trajectory = None
        self._t0 = None
        self._last_pose = None
        self._last_pose_t = None
        self._last_command = None
        self._last_command_t = None
        if self._eso is not None:
            self._eso.reset()
        if self._velocity_estimator is not None:
            self._velocity_estimator.reset()
        if self._deadtime_predictor is not None:
            self._deadtime_predictor.reset()
        self._last_eso_t = None
        return True

    def get_state(self) -> TrajectoryTrackingState:
        return self._state


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class TrajectoryTrackingTaskParams(BaseConfig):
    # Path to a characterization artifact (TuningConfig JSON). When set, the
    # gains/limits are built from it (the Go2 / any-base path); when None the
    # task uses the vendored FlowBase config.
    artifact_path: str | None = None
    max_speed: float | None = None
    gain_profile: GainProfile = "default"
    compensate_gain: bool = True
    # Opt-in ESO/ADRC disturbance-rejection inner loop (OFF = today's behavior).
    eso: bool = False
    eso_bandwidth: float = 1.0
    deadtime_compensation: bool = False
    deadtime_feedback_lag_s: float = 0.0
    deadtime_prediction_blend: float = 1.0
    deadtime_prediction_mode: Literal["full", "yaw_only"] = "full"
    yaw_feedforward_mode: YawFeedforwardMode = "planned"
    heading_mode: Literal["tangent", "fixed"] = "tangent"
    fixed_heading: float = 0.0
    goal_tolerance: float = 0.05
    orientation_tolerance: float = 0.1
    stale_pose_timeout: float = 0.3
    command_rate_hz: float | None = None


def create_task(cfg: Any, hardware: Any) -> TrajectoryTrackingTask:
    params = TrajectoryTrackingTaskParams.model_validate(cfg.params)
    # The artifact is loaded lazily on start_path (see _ensure_artifact_loaded),
    # so a missing file never blocks coordinator startup.
    return TrajectoryTrackingTask(
        cfg.name,
        TrajectoryTrackingTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            artifact_path=params.artifact_path,
            max_speed=params.max_speed,
            gain_profile=params.gain_profile,
            compensate_gain=params.compensate_gain,
            eso=params.eso,
            eso_bandwidth=params.eso_bandwidth,
            deadtime_compensation=params.deadtime_compensation,
            deadtime_feedback_lag_s=params.deadtime_feedback_lag_s,
            deadtime_prediction_blend=params.deadtime_prediction_blend,
            deadtime_prediction_mode=params.deadtime_prediction_mode,
            yaw_feedforward_mode=params.yaw_feedforward_mode,
            heading_mode=params.heading_mode,
            fixed_heading=params.fixed_heading,
            goal_tolerance=params.goal_tolerance,
            orientation_tolerance=params.orientation_tolerance,
            stale_pose_timeout=params.stale_pose_timeout,
            command_rate_hz=params.command_rate_hz,
        ),
    )


__all__ = [
    "TrajectoryTrackingTask",
    "TrajectoryTrackingTaskConfig",
]
