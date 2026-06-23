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

"""Tests for the causal body-velocity estimator."""

from __future__ import annotations

import math
import random

import pytest

from dimos.control.tasks.trajectory_tracking_task.velocity_estimator import BodyVelocityEstimator

_DT = 1.0 / 16.0


def test_none_until_enough_samples() -> None:
    est = BodyVelocityEstimator()
    assert est.body_velocity(0.0) is None
    est.update(0.0, 0.0, 0.0, 0.0)
    est.update(_DT, 0.1, 0.0, 0.0)
    assert est.body_velocity(0.0) is None  # < 3 distinct samples
    est.update(2 * _DT, 0.2, 0.0, 0.0)
    assert est.body_velocity(0.0) is not None


def test_constant_world_velocity_aligned_heading() -> None:
    """Robot moving +x in world at 1 m/s, heading 0 -> body vx=1, vy=0."""
    est = BodyVelocityEstimator()
    for k in range(10):
        est.update(k * _DT, 1.0 * k * _DT, 0.0, 0.0)
    vx, vy, wz = est.body_velocity(0.0)
    assert vx == pytest.approx(1.0, abs=1e-3)
    assert vy == pytest.approx(0.0, abs=1e-3)
    assert wz == pytest.approx(0.0, abs=1e-3)


def test_world_velocity_rotated_into_body() -> None:
    """Robot moving +x in WORLD at 1 m/s but heading +90deg: body frame sees
    vx=0, vy=-1 (the world +x is to the robot's right)."""
    est = BodyVelocityEstimator()
    yaw = math.pi / 2
    for k in range(10):
        est.update(k * _DT, 1.0 * k * _DT, 0.0, yaw)
    vx, vy, _ = est.body_velocity(yaw)
    assert vx == pytest.approx(0.0, abs=1e-3)
    assert vy == pytest.approx(-1.0, abs=1e-3)


def test_yaw_rate_recovered_through_wrap() -> None:
    """Constant yaw rate that crosses the +-pi wrap must still differentiate
    to the right omega (the estimator unwraps internally)."""
    est = BodyVelocityEstimator()
    wz_true = 2.0
    yaw = math.pi - 0.1
    for k in range(10):
        y = (yaw + wz_true * k * _DT + math.pi) % (2 * math.pi) - math.pi
        est.update(k * _DT, 0.0, 0.0, y)
    _, _, wz = est.body_velocity(est._yaw[-1])
    assert wz == pytest.approx(wz_true, abs=1e-2)


def test_held_pose_deduplicated() -> None:
    """Repeated identical poses (odom held between updates) must not bias the
    velocity toward zero — they are dropped."""
    est = BodyVelocityEstimator()
    # three real moving samples
    for k in range(4):
        est.update(k * _DT, 1.0 * k * _DT, 0.0, 0.0)
    moving = est.body_velocity(0.0)
    # now feed the SAME last pose several times at advancing timestamps
    last_x = 3 * _DT
    for k in range(4, 10):
        est.update(k * _DT, last_x, 0.0, 0.0)
    held = est.body_velocity(0.0)
    assert moving is not None and held is not None
    # the dedup keeps the last real velocity rather than collapsing to 0
    assert held[0] == pytest.approx(moving[0], abs=1e-6)


def test_smoothing_beats_finite_difference_on_noise() -> None:
    """On a noisy constant-velocity stream, the smoothed estimate is closer to
    truth than a raw two-point finite difference (the reason we smooth)."""
    rng = random.Random(1)
    est = BodyVelocityEstimator(window=7)
    v_true = 1.0
    sigma = 0.02
    errs_est = []
    errs_fd = []
    prev = None
    for k in range(60):
        x = v_true * k * _DT + rng.gauss(0.0, sigma)
        if prev is not None:
            errs_fd.append(abs((x - prev[1]) / (k * _DT - prev[0]) - v_true))
        prev = (k * _DT, x)
        est.update(k * _DT, x, 0.0, 0.0)
        bv = est.body_velocity(0.0)
        if bv is not None:
            errs_est.append(abs(bv[0] - v_true))
    # mean error of the smoothed estimate beats the raw finite difference
    assert sum(errs_est) / len(errs_est) < sum(errs_fd) / len(errs_fd)
