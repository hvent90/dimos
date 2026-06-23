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

"""Unit + closed-loop tests for the model-based ESO/ADRC layer."""

from __future__ import annotations

import math

import pytest

from dimos.control.tasks.trajectory_tracking_task.config import TrackingConfig
from dimos.control.tasks.trajectory_tracking_task.eso import (
    _WO_ABS_CAP,
    AxisESO,
    AxisESOConfig,
    ESOCompensator,
    build_eso,
)
from dimos.utils.benchmarking.plant import GO2_PLANT_FITTED
from dimos.utils.benchmarking.tuning import Provenance, derive_config

_DT = 1.0 / 16.0  # odom-rate tick


class _FirstOrderPlant:
    """Discrete first-order velocity plant ``y_dot = -y/tau + (K_true/tau) u``
    with an optional constant output disturbance ``d`` — i.e. the true steady
    gain is ``K_true`` even though the controller only knows the nominal ``K``."""

    def __init__(self, K_true: float, tau: float, d: float = 0.0) -> None:
        self.K_true = K_true
        self.tau = tau
        self.d = d
        self.y = 0.0

    def step(self, u: float, dt: float, substeps: int = 8) -> float:
        h = dt / substeps
        for _ in range(substeps):
            self.y += h * (-self.y / self.tau + (self.K_true / self.tau) * u + self.d)
        return self.y


def _go2_tracking() -> TrackingConfig:
    artifact = derive_config(
        GO2_PLANT_FITTED,
        Provenance(robot_id="go2", surface="concrete", mode="default", sim_or_hw="sim"),
    )
    return TrackingConfig.from_artifact(artifact)


# --- builder --------------------------------------------------------------


def test_build_eso_shapes_and_caps() -> None:
    tracking = _go2_tracking()
    eso = build_eso(tracking, bandwidth=1.0)
    assert isinstance(eso, ESOCompensator)
    for axis, K, tau in (
        (eso.vx, tracking.k_hat.x, tracking.tau.x),
        (eso.wz, tracking.k_hat.yaw, tracking.tau.yaw),
    ):
        assert axis.cfg.K == pytest.approx(K)
        assert axis.cfg.tau == pytest.approx(tau)
        assert axis.cfg.w_o <= _WO_ABS_CAP + 1e-9
        assert axis.cfg.w_o > 0.0


def test_dial_scales_observer_bandwidth() -> None:
    tracking = _go2_tracking()
    lo = build_eso(tracking, bandwidth=0.5).wz.cfg.w_o
    hi = build_eso(tracking, bandwidth=2.0).wz.cfg.w_o
    assert hi > lo  # the dial opens the observer bandwidth
    assert hi <= _WO_ABS_CAP + 1e-9  # but never past the cap


# --- single-axis behaviour ------------------------------------------------


def _settle(axis: AxisESO, plant: _FirstOrderPlant, r: float, n: int = 400) -> float:
    """Run the axis ESO closed loop against the plant; return final y."""
    for _ in range(n):
        u = axis.compute(r, plant.y, _DT)
        plant.step(u, _DT)
    return plant.y


def test_no_disturbance_reduces_to_gain_inversion() -> None:
    """On the nominal plant (true gain == fit gain, no disturbance) the ESO
    must reproduce the baseline: y -> r and the command -> r/K, disturbance
    estimate -> ~0."""
    K, tau = 2.45, 0.6
    axis = AxisESO(AxisESOConfig(K=K, tau=tau, w_o=1.0, u_limit=2.0))
    plant = _FirstOrderPlant(K_true=K, tau=tau)
    y = _settle(axis, plant, r=1.0)
    assert y == pytest.approx(1.0, abs=0.02)
    assert axis.disturbance == pytest.approx(0.0, abs=0.05)
    assert axis._control(1.0) == pytest.approx(1.0 / K, abs=0.02)


def test_cancels_constant_gain_error() -> None:
    """With a 30% gain error (true gain 0.7K), plain gain inversion would reach
    only ~0.7 r; the ESO's disturbance estimate must recover y -> r."""
    K, tau, r = 2.45, 0.6, 1.0
    axis = AxisESO(AxisESOConfig(K=K, tau=tau, w_o=1.0, u_limit=3.0))
    # plant with 0.7K gain
    plant = _FirstOrderPlant(K_true=0.7 * K, tau=tau)
    y_eso = _settle(axis, plant, r=r, n=800)
    # the gain-inversion baseline (open loop u = r/K) settles at 0.7 r
    y_baseline = 0.7 * r
    assert y_eso == pytest.approx(r, abs=0.05)
    assert abs(y_eso - r) < abs(y_baseline - r)


def test_command_clamped_to_limit() -> None:
    axis = AxisESO(AxisESOConfig(K=2.45, tau=0.6, w_o=1.0, u_limit=0.3))
    # Demand far beyond what the clamp allows; command must saturate.
    for _ in range(50):
        u = axis.compute(10.0, 0.0, _DT)
    assert abs(u) <= 0.3 + 1e-9


def test_predict_only_when_measurement_missing() -> None:
    """A None measurement (no fresh odom) must not raise and must keep
    producing a finite command (predict-only, no correction)."""
    axis = AxisESO(AxisESOConfig(K=2.45, tau=0.6, w_o=1.0, u_limit=2.0))
    axis.compute(1.0, 0.5, _DT)
    for _ in range(20):
        u = axis.compute(1.0, None, _DT)
    assert math.isfinite(u)


def test_reset_clears_state() -> None:
    axis = AxisESO(AxisESOConfig(K=2.45, tau=0.6, w_o=1.0, u_limit=2.0))
    for _ in range(50):
        axis.compute(1.0, 0.3, _DT)
    axis.reset()
    assert axis.disturbance == 0.0
    assert axis.velocity_estimate == 0.0


def test_stable_under_noisy_measurement() -> None:
    """A bounded, zero-mean measurement perturbation must not make the command
    diverge (noise robustness — the known failure mode)."""
    import random

    rng = random.Random(0)
    K, tau = 2.45, 0.6
    axis = AxisESO(AxisESOConfig(K=K, tau=tau, w_o=1.0, u_limit=2.0))
    plant = _FirstOrderPlant(K_true=K, tau=tau)
    umax = 0.0
    for _ in range(2000):
        y_meas = plant.y + rng.uniform(-0.05, 0.05)
        u = axis.compute(1.0, y_meas, _DT)
        plant.step(u, _DT)
        umax = max(umax, abs(u))
    assert umax < 2.0 + 1e-9  # stayed within the clamp, never ran away
    assert plant.y == pytest.approx(1.0, abs=0.1)


# --- 3-axis compensator ---------------------------------------------------


def test_compensator_handles_missing_measurement() -> None:
    eso = build_eso(_go2_tracking(), bandwidth=1.0)
    out = eso.compute((0.5, 0.0, 0.3), None, _DT)
    assert len(out) == 3 and all(math.isfinite(v) for v in out)
    eso.reset()
    assert eso.wz.disturbance == 0.0
