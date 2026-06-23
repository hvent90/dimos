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

from __future__ import annotations

import pytest

from dimos.control.tasks.trajectory_tracking_task.config import PerAxis
from dimos.control.tasks.trajectory_tracking_task.deadtime_predictor import (
    DeadtimePosePredictor,
    DeadtimePredictorConfig,
)
from dimos.control.tasks.trajectory_tracking_task.gain_schedule import AxisGainCurve, GainSchedule


def _predictor(horizon_s: float = 0.2) -> DeadtimePosePredictor:
    return DeadtimePosePredictor(
        DeadtimePredictorConfig(
            k_hat=PerAxis(1.0, 1.0, 1.0),
            tau=PerAxis(0.05, 0.05, 0.05),
            deadtime=PerAxis(0.0, 0.0, 0.0),
            horizon_s=horizon_s,
            model_dt=0.005,
        )
    )


def test_predict_advances_measured_pose_by_recent_model_delta() -> None:
    predictor = _predictor()
    predictor.reset(0.0, (0.0, 0.0, 0.0))
    predictor.record_command(0.0, (1.0, 0.0, 0.0))

    predicted = predictor.predict((0.0, 0.0, 0.0), 0.5)

    assert predicted[0] == pytest.approx(0.2, abs=0.02)
    assert predicted[1] == pytest.approx(0.0, abs=1e-6)
    assert predicted[2] == pytest.approx(0.0, abs=1e-6)


def test_predict_uses_delta_not_absolute_model_pose() -> None:
    predictor = _predictor()
    predictor.reset(0.0, (10.0, -3.0, 0.0))
    predictor.record_command(0.0, (1.0, 0.0, 0.0))

    predicted = predictor.predict((2.0, 4.0, 0.0), 0.5)

    assert predicted[0] == pytest.approx(2.2, abs=0.02)
    assert predicted[1] == pytest.approx(4.0, abs=1e-6)


def test_zero_horizon_is_noop() -> None:
    predictor = _predictor(horizon_s=0.0)
    predictor.reset(0.0, (0.0, 0.0, 0.0))
    predictor.record_command(0.0, (1.0, 0.0, 0.0))

    assert predictor.predict((2.0, 3.0, 0.4), 1.0) == (2.0, 3.0, 0.4)


def test_schedule_changes_nominal_model_gain() -> None:
    schedule = GainSchedule(
        vx=AxisGainCurve(amp=(0.0, 1.0), gain=(2.0, 2.0), tau=(0.05, 0.05)),
        vy=AxisGainCurve(amp=(0.0, 1.0), gain=(1.0, 1.0), tau=(0.05, 0.05)),
        wz=AxisGainCurve(amp=(0.0, 1.0), gain=(1.0, 1.0), tau=(0.05, 0.05)),
    )
    predictor = DeadtimePosePredictor(
        DeadtimePredictorConfig(
            k_hat=PerAxis(1.0, 1.0, 1.0),
            tau=PerAxis(0.05, 0.05, 0.05),
            deadtime=PerAxis(0.0, 0.0, 0.0),
            horizon_s=0.2,
            schedule=schedule,
            model_dt=0.005,
        )
    )
    predictor.reset(0.0, (0.0, 0.0, 0.0))
    predictor.record_command(0.0, (1.0, 0.0, 0.0))

    predicted = predictor.predict((0.0, 0.0, 0.0), 0.5)

    assert predicted[0] == pytest.approx(0.4, abs=0.03)


def test_blend_and_yaw_only_limit_prediction_delta() -> None:
    predictor = DeadtimePosePredictor(
        DeadtimePredictorConfig(
            k_hat=PerAxis(1.0, 1.0, 1.0),
            tau=PerAxis(0.05, 0.05, 0.05),
            deadtime=PerAxis(0.0, 0.0, 0.0),
            horizon_s=0.2,
            blend=0.5,
            mode="yaw_only",
            model_dt=0.005,
        )
    )
    predictor.reset(0.0, (0.0, 0.0, 0.0))
    predictor.record_command(0.0, (1.0, 0.0, 1.0))

    predicted = predictor.predict((2.0, 3.0, 0.0), 0.5)

    assert predicted[0] == pytest.approx(2.0, abs=1e-6)
    assert predicted[1] == pytest.approx(3.0, abs=1e-6)
    assert predicted[2] == pytest.approx(0.1, abs=0.02)
