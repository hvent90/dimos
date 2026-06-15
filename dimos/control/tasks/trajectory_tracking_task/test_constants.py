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

"""The constants module is computed (not retyped) from the vendored plant
fit, and the gain-inversion compensation makes the plant behave as K=1."""

from __future__ import annotations

import pytest

from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainCompensator
from dimos.control.tasks.trajectory_tracking_task.constants import (
    K_HAT,
    KP_AGGRESSIVE,
    KP_DEFAULT,
    PHYSICAL_MAX_VEL,
    PLAN_MAX_VEL,
    PLANNING_MARGIN,
    flowbase_feedforward_config,
    kp_for_zeta,
)
from dimos.utils.benchmarking.plant import FLOWBASE_PLANT_FITTED, TwistBasePlantSim

_DT = 0.01
_SETTLE_S = 6.0
_STEADY_STATE_TOLERANCE = 0.02


def test_gains_match_closed_loop_design() -> None:
    # Spot-check against the analysis doc's table (zeta=1.0 / 0.7).
    assert KP_DEFAULT.x == pytest.approx(0.87, abs=0.01)
    assert KP_DEFAULT.y == pytest.approx(0.94, abs=0.01)
    assert KP_DEFAULT.yaw == pytest.approx(0.41, abs=0.01)
    assert KP_AGGRESSIVE.x == pytest.approx(1.77, abs=0.01)
    assert KP_AGGRESSIVE.yaw == pytest.approx(0.84, abs=0.01)
    assert kp_for_zeta(tau=0.25, zeta=1.0) == pytest.approx(1.0)


def test_limits_derive_from_plant_and_margin() -> None:
    assert K_HAT.x == pytest.approx(FLOWBASE_PLANT_FITTED.vx.K)
    assert PHYSICAL_MAX_VEL.x == pytest.approx(0.62, abs=0.01)
    assert PHYSICAL_MAX_VEL.yaw == pytest.approx(8.8, abs=0.1)
    assert PLAN_MAX_VEL.x == pytest.approx(PLANNING_MARGIN * PHYSICAL_MAX_VEL.x)


@pytest.mark.parametrize("channel", ["vx", "vy", "wz"])
def test_compensated_plant_behaves_as_unity_gain(channel: str) -> None:
    """u_cmd = u_phys / K_hat: the compensated closed chain settles at the
    requested velocity; uncompensated it settles at K x."""
    request = {"vx": 0.4, "vy": 0.4, "wz": 0.5}[channel]
    params = getattr(FLOWBASE_PLANT_FITTED, channel)

    compensator = FeedforwardGainCompensator(flowbase_feedforward_config())
    sim = TwistBasePlantSim(FLOWBASE_PLANT_FITTED)
    sim.reset(0.0, 0.0, 0.0, _DT)
    desired = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    desired[channel] = request
    for _ in range(int(_SETTLE_S / _DT)):
        cmd = compensator.compute(desired["vx"], desired["vy"], desired["wz"])
        sim.step(*cmd, _DT)
    assert getattr(sim, channel) == pytest.approx(request, rel=_STEADY_STATE_TOLERANCE)

    sim.reset(0.0, 0.0, 0.0, _DT)
    for _ in range(int(_SETTLE_S / _DT)):
        sim.step(desired["vx"], desired["vy"], desired["wz"], _DT)
    assert getattr(sim, channel) == pytest.approx(params.K * request, rel=_STEADY_STATE_TOLERANCE)
