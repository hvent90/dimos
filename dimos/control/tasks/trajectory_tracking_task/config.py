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

"""Robot-agnostic config for the trajectory-tracking controller.

A :class:`TrackingConfig` holds every gain and limit the controller needs,
all derived from a measured FOPDT fit. There are two ways to build one:

* :meth:`TrackingConfig.from_plant_fit` — for a base whose physical limits
  come from a firmware command limiter (``physical = K x command``, then an
  85% planning margin). This is the FlowBase path.
* :meth:`TrackingConfig.from_artifact` — for a base characterized into a
  ``TuningConfig`` artifact whose ``velocity_profile`` section already holds
  the operating envelope. This is the Go2 path: a new characterization
  artifact drops in with no code change.

The control-design choices (damping ratio, planning margin, lateral-accel
fraction, feedback clamps) are shared across robots; only the plant fit and
the envelope are robot-specific.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig
from dimos.control.tasks.trajectory_tracking_task.gain_schedule import GainSchedule
from dimos.utils.benchmarking.plant import TwistBasePlantParams
from dimos.utils.benchmarking.tuning import TuningConfig

# --- shared control-design constants (not robot-specific) ----------------
PLANNING_MARGIN = 0.85
LAT_ACCEL_FRACTION = 0.7
ZETA_DEFAULT = 1.0  # critically damped — no overshoot
ZETA_AGGRESSIVE = 0.7
FB_CLAMP_LINEAR = 0.15  # m/s — feedback contribution clamp
FB_CLAMP_YAW = 0.4  # rad/s
_EPS = 1e-9


@dataclass(frozen=True)
class AxisTriple:
    """One value per twist-base axis (x, y, yaw)."""

    x: float
    y: float
    yaw: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


def kp_for_zeta(tau: float, zeta: float) -> float:
    """P gain for a target damping ratio on a first-order-lag plant.

    From the normalized characteristic equation tau*s^2 + s + kp = 0.
    """
    return 1.0 / (4.0 * zeta * zeta * tau)


def _safe_div(a: float, b: float) -> float:
    return a / b if abs(b) > _EPS else a


@dataclass(frozen=True)
class ProfileLimits:
    """Limits the trajectory generator needs (per-axis vel/acc + lateral)."""

    plan_max_vel: AxisTriple
    plan_max_acc: AxisTriple
    a_lat_max: float


@dataclass(frozen=True)
class TrackingConfig:
    """Everything the trajectory tracker needs, derived from a plant fit."""

    k_hat: AxisTriple  # plant steady-state gain (for u_cmd = u_phys / K_hat)
    deadtime: AxisTriple  # per-axis L (s) — FF reference is previewed by this
    kp_default: AxisTriple  # per-axis P gain at zeta=1.0
    kp_aggressive: AxisTriple  # per-axis P gain at zeta=0.7
    plan_max_vel: AxisTriple
    plan_max_acc: AxisTriple
    a_lat_max: float  # lateral/centripetal accel budget for cornering
    ff_output_limit: AxisTriple  # command-space clamp on the gain-inverted output
    # Optional speed-scheduled gain inversion (nonlinear plants). When set,
    # the controller inverts K(|v|) per axis instead of the constant k_hat.
    schedule: GainSchedule | None = None
    fb_clamp_linear: float = FB_CLAMP_LINEAR
    fb_clamp_yaw: float = FB_CLAMP_YAW
    provenance: str = ""

    @property
    def profile_limits(self) -> ProfileLimits:
        return ProfileLimits(self.plan_max_vel, self.plan_max_acc, self.a_lat_max)

    def kp(self, gain_profile: str) -> AxisTriple:
        return self.kp_aggressive if gain_profile == "aggressive" else self.kp_default

    def feedforward_config(self) -> FeedforwardGainConfig:
        """Gain-inversion config (u_cmd = u_phys / K_hat) for the existing
        FeedforwardGainCompensator, clamped to the command ceiling."""
        return FeedforwardGainConfig(
            K_vx=self.k_hat.x,
            K_vy=self.k_hat.y,
            K_wz=self.k_hat.yaw,
            output_min_vx=-self.ff_output_limit.x,
            output_max_vx=self.ff_output_limit.x,
            output_min_vy=-self.ff_output_limit.y,
            output_max_vy=self.ff_output_limit.y,
            output_min_wz=-self.ff_output_limit.yaw,
            output_max_wz=self.ff_output_limit.yaw,
        )

    @staticmethod
    def from_plant_fit(
        plant: TwistBasePlantParams,
        cmd_max_vel: tuple[float, float, float],
        cmd_max_acc: tuple[float, float, float],
        *,
        provenance: str = "",
    ) -> TrackingConfig:
        """Build from a fit + a firmware command limiter (FlowBase style):
        physical limits = K x command limits, then the 85% planning margin."""
        k_hat = AxisTriple(plant.vx.K, plant.vy.K, plant.wz.K)
        plan_vel = AxisTriple(
            PLANNING_MARGIN * k_hat.x * cmd_max_vel[0],
            PLANNING_MARGIN * k_hat.y * cmd_max_vel[1],
            PLANNING_MARGIN * k_hat.yaw * cmd_max_vel[2],
        )
        plan_acc = AxisTriple(
            PLANNING_MARGIN * k_hat.x * cmd_max_acc[0],
            PLANNING_MARGIN * k_hat.y * cmd_max_acc[1],
            PLANNING_MARGIN * k_hat.yaw * cmd_max_acc[2],
        )
        return TrackingConfig(
            k_hat=k_hat,
            deadtime=AxisTriple(plant.vx.L, plant.vy.L, plant.wz.L),
            kp_default=AxisTriple(
                kp_for_zeta(plant.vx.tau, ZETA_DEFAULT),
                kp_for_zeta(plant.vy.tau, ZETA_DEFAULT),
                kp_for_zeta(plant.wz.tau, ZETA_DEFAULT),
            ),
            kp_aggressive=AxisTriple(
                kp_for_zeta(plant.vx.tau, ZETA_AGGRESSIVE),
                kp_for_zeta(plant.vy.tau, ZETA_AGGRESSIVE),
                kp_for_zeta(plant.wz.tau, ZETA_AGGRESSIVE),
            ),
            plan_max_vel=plan_vel,
            plan_max_acc=plan_acc,
            a_lat_max=LAT_ACCEL_FRACTION * min(plan_acc.x, plan_acc.y),
            ff_output_limit=AxisTriple(*cmd_max_vel),
            provenance=provenance,
        )

    @staticmethod
    def from_artifact(tuning: TuningConfig, schedule: GainSchedule | None = None) -> TrackingConfig:
        """Build from a characterization artifact (Go2 style): gains from the
        plant fit, limits straight from the artifact's velocity-profile
        envelope. The artifact already carries the operating margins, so no
        extra planning margin is applied here. ``schedule`` (optional) enables
        speed-scheduled gain inversion for nonlinear plants."""
        plant = tuning.plant
        vp = tuning.velocity_profile
        k_hat = AxisTriple(plant.vx.K, plant.vy.K, plant.wz.K)
        # The envelope has one linear cap (applies to vx and vy) and one
        # angular cap. Derive a yaw accel from the first-order rise
        # (omega_max / tau_wz), mirroring how DERIVE sets the linear accel.
        yaw_acc = _safe_div(vp.max_angular_speed, plant.wz.tau)
        prov = tuning.provenance
        return TrackingConfig(
            k_hat=k_hat,
            deadtime=AxisTriple(plant.vx.L, plant.vy.L, plant.wz.L),
            kp_default=AxisTriple(
                kp_for_zeta(plant.vx.tau, ZETA_DEFAULT),
                kp_for_zeta(plant.vy.tau, ZETA_DEFAULT),
                kp_for_zeta(plant.wz.tau, ZETA_DEFAULT),
            ),
            kp_aggressive=AxisTriple(
                kp_for_zeta(plant.vx.tau, ZETA_AGGRESSIVE),
                kp_for_zeta(plant.vy.tau, ZETA_AGGRESSIVE),
                kp_for_zeta(plant.wz.tau, ZETA_AGGRESSIVE),
            ),
            plan_max_vel=AxisTriple(vp.max_linear_speed, vp.max_linear_speed, vp.max_angular_speed),
            plan_max_acc=AxisTriple(vp.max_linear_accel, vp.max_linear_accel, yaw_acc),
            a_lat_max=vp.max_centripetal_accel,
            # FF output is a command; the command that reaches the envelope
            # speed is envelope / K.
            ff_output_limit=AxisTriple(
                _safe_div(vp.max_linear_speed, k_hat.x),
                _safe_div(vp.max_linear_speed, k_hat.y),
                _safe_div(vp.max_angular_speed, k_hat.yaw),
            ),
            schedule=schedule,
            provenance=f"{prov.robot_id} {prov.date} {prov.git_sha}",
        )


def tracking_config_from_artifact_path(path: str) -> TrackingConfig:
    """Load a characterization artifact JSON and build a TrackingConfig,
    including a speed-scheduled gain inversion when the artifact carries a
    ``dynamics_by_amplitude`` table (nonlinear plants)."""
    raw = json.loads(Path(path).read_text())
    schedule = GainSchedule.from_dynamics(raw.get("dynamics_by_amplitude"))
    return TrackingConfig.from_artifact(TuningConfig.from_json(path), schedule=schedule)


__all__ = [
    "FB_CLAMP_LINEAR",
    "FB_CLAMP_YAW",
    "LAT_ACCEL_FRACTION",
    "PLANNING_MARGIN",
    "ZETA_AGGRESSIVE",
    "ZETA_DEFAULT",
    "AxisTriple",
    "ProfileLimits",
    "TrackingConfig",
    "kp_for_zeta",
    "tracking_config_from_artifact_path",
]
