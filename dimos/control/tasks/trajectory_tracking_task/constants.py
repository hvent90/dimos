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

"""Single source of truth for the FlowBase trajectory-tracking controller.

Every gain and limit the controller uses traces back here, and everything
here is COMPUTED from the vendored plant fit (``FLOWBASE_PLANT_FITTED``)
plus the firmware command limits — nothing is retyped from the analysis
docs, so a re-characterization only has to update the fit in plant.py.

Closed-loop design (per-axis, FOPDT plant, P feedback over FF):
the normalized characteristic equation is ``tau*s^2 + s + kp = 0``, giving
``kp = 1 / (4 * zeta^2 * tau)`` for damping ratio zeta.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig
from dimos.utils.benchmarking.plant import (
    FLOWBASE_CMD_MAX_ACC,
    FLOWBASE_CMD_MAX_VEL,
    FLOWBASE_PLANT_FITTED,
)

# Provenance — embed in run metadata of every certification/benchmark run.
CHARACTERIZATION_DATE = "2026-06-09"
CHARACTERIZATION_ARTIFACT = (
    "data/characterization/flowbase/flowbase_config_hw_concrete_2026-06-09_704a591f5.json"
)


@dataclass(frozen=True)
class AxisTriple:
    """One value per twist-base axis (x, y, yaw)."""

    x: float
    y: float
    yaw: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


# Plant steady-state gains (actuation-side: robot moves K x the command).
K_HAT = AxisTriple(
    x=FLOWBASE_PLANT_FITTED.vx.K,
    y=FLOWBASE_PLANT_FITTED.vy.K,
    yaw=FLOWBASE_PLANT_FITTED.wz.K,
)

# Per-axis dead time (s) — the FF reference is previewed by this much.
DEADTIME = AxisTriple(
    x=FLOWBASE_PLANT_FITTED.vx.L,
    y=FLOWBASE_PLANT_FITTED.vy.L,
    yaw=FLOWBASE_PLANT_FITTED.wz.L,
)

# Physical limits = K x firmware command limits. Trajectories must stay
# within PLANNING_MARGIN of these so the firmware Ruckig limiter never
# engages and the loop stays linear.
PHYSICAL_MAX_VEL = AxisTriple(
    x=K_HAT.x * FLOWBASE_CMD_MAX_VEL[0],
    y=K_HAT.y * FLOWBASE_CMD_MAX_VEL[1],
    yaw=K_HAT.yaw * FLOWBASE_CMD_MAX_VEL[2],
)
PHYSICAL_MAX_ACC = AxisTriple(
    x=K_HAT.x * FLOWBASE_CMD_MAX_ACC[0],
    y=K_HAT.y * FLOWBASE_CMD_MAX_ACC[1],
    yaw=K_HAT.yaw * FLOWBASE_CMD_MAX_ACC[2],
)

PLANNING_MARGIN = 0.85

PLAN_MAX_VEL = AxisTriple(
    x=PLANNING_MARGIN * PHYSICAL_MAX_VEL.x,
    y=PLANNING_MARGIN * PHYSICAL_MAX_VEL.y,
    yaw=PLANNING_MARGIN * PHYSICAL_MAX_VEL.yaw,
)
PLAN_MAX_ACC = AxisTriple(
    x=PLANNING_MARGIN * PHYSICAL_MAX_ACC.x,
    y=PLANNING_MARGIN * PHYSICAL_MAX_ACC.y,
    yaw=PLANNING_MARGIN * PHYSICAL_MAX_ACC.yaw,
)

# Lateral (centripetal) acceleration budget for cornering. A point following
# a curve of curvature kappa at speed v needs a = v^2 * kappa; the trajectory
# generator caps v <= sqrt(A_LAT_MAX / kappa) so a corner is taken at a speed
# the base can actually hold instead of overshooting. It's drawn from the same
# translational-accel authority as PLAN_MAX_ACC (a fraction, leaving headroom
# for the along-path accel/decel ramps to share the budget). tuning.A_LAT_MAX
# = 1.0 is the repo's ride-quality precedent; this plant-traced value brackets it.
LAT_ACCEL_FRACTION = 0.7
A_LAT_MAX = LAT_ACCEL_FRACTION * min(PLAN_MAX_ACC.x, PLAN_MAX_ACC.y)


def kp_for_zeta(tau: float, zeta: float) -> float:
    """P gain for a target damping ratio on a first-order-lag plant."""
    return 1.0 / (4.0 * zeta * zeta * tau)


def _gains(zeta: float) -> AxisTriple:
    return AxisTriple(
        x=kp_for_zeta(FLOWBASE_PLANT_FITTED.vx.tau, zeta),
        y=kp_for_zeta(FLOWBASE_PLANT_FITTED.vy.tau, zeta),
        yaw=kp_for_zeta(FLOWBASE_PLANT_FITTED.wz.tau, zeta),
    )


ZETA_DEFAULT = 1.0  # critically damped — no overshoot
ZETA_AGGRESSIVE = 0.7

KP_DEFAULT = _gains(ZETA_DEFAULT)  # ~ (0.87, 0.94, 0.41)
KP_AGGRESSIVE = _gains(ZETA_AGGRESSIVE)  # ~ (1.77, 1.91, 0.84)

# Feedback contribution clamps: FF carries the trajectory, FB only corrects
# deviations — bounding FB keeps the total command within limits even with
# large transient pose error.
FB_CLAMP_LINEAR = 0.15  # m/s
FB_CLAMP_YAW = 0.4  # rad/s


def flowbase_feedforward_config() -> FeedforwardGainConfig:
    """Gain-inversion config (u_cmd = u_phys / K_hat) for the existing
    FeedforwardGainCompensator, with output clamps at the firmware
    command ceiling."""
    return FeedforwardGainConfig(
        K_vx=K_HAT.x,
        K_vy=K_HAT.y,
        K_wz=K_HAT.yaw,
        output_min_vx=-FLOWBASE_CMD_MAX_VEL[0],
        output_max_vx=FLOWBASE_CMD_MAX_VEL[0],
        output_min_vy=-FLOWBASE_CMD_MAX_VEL[1],
        output_max_vy=FLOWBASE_CMD_MAX_VEL[1],
        output_min_wz=-FLOWBASE_CMD_MAX_VEL[2],
        output_max_wz=FLOWBASE_CMD_MAX_VEL[2],
    )
