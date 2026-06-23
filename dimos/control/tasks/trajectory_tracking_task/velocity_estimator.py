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

"""Body-frame velocity from a noisy, low-rate odometry pose stream.

The ESO inner loop (:mod:`.eso`) needs the *measured* body velocity, but on a
legged base the only honest feedback is odometry pose at ~16-18 Hz with several
cm of noise — and ``read_velocities()`` returns the last command, not a
measurement. The validated recipe for this base (see the project memory on
legged-base velocity recovery) is **smooth the position, then differentiate** —
never differentiate raw odom and then try to smooth the velocity, which cannot
recover from the noise amplification.

This estimator does that causally so it is deployable in the live tick loop:
it keeps a short rolling window of distinct pose samples, fits a low-order
polynomial in time to each of ``x``, ``y`` and (unwrapped) ``yaw``, and reads
the derivative at the most recent sample. That is a causal Savitzky-Golay
differentiator. Evaluated at the window endpoint with a quadratic (or higher)
fit it has only a small group delay — much less than half the window, which a
*linear* fit would have — so the estimate corresponds to "now" and is rotated
into the body frame by the current yaw. (Do not subtract a half-window lag from
that rotation: with the quadratic fit it over-rotates and injects a phantom
lateral velocity on curves.)

Held/duplicate poses (the adapter returns the last odom between updates, so at
a 100 Hz tick the same pose repeats ~6x) are de-duplicated: feeding repeated
identical samples at distinct timestamps would bias the slope toward zero.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np

from dimos.utils.trigonometry import angle_diff

# Default rolling-window length (samples). At 16 Hz, 5 samples span ~0.25 s;
# the endpoint derivative of the quadratic fit has only a small group delay.
_DEFAULT_WINDOW = 5
_DEFAULT_POLY = 2
_MIN_SAMPLES = 3
# Two samples closer than this in time are treated as one (no new information).
_MIN_SAMPLE_DT = 1e-3
# Pose deltas below these are "the same pose held" (skip — see module docstring).
_POS_EPS = 1e-4  # m
_YAW_EPS = 1e-4  # rad


class BodyVelocityEstimator:
    """Causal smooth-then-differentiate body-velocity estimator.

    Feed it ``update(t, x, y, yaw)`` whenever a pose is read; query
    ``body_velocity(yaw)`` for the latest ``(vx, vy, wz)`` in the body frame,
    or ``None`` until enough distinct samples have accumulated.
    """

    def __init__(
        self, window: int = _DEFAULT_WINDOW, poly: int = _DEFAULT_POLY
    ) -> None:
        if window < 3:
            raise ValueError("window must be >= 3")
        self._window = window
        self._poly = poly
        self._t: deque[float] = deque(maxlen=window)
        self._x: deque[float] = deque(maxlen=window)
        self._y: deque[float] = deque(maxlen=window)
        self._yaw: deque[float] = deque(maxlen=window)  # unwrapped
        self._last_raw_yaw: float | None = None

    def reset(self) -> None:
        self._t.clear()
        self._x.clear()
        self._y.clear()
        self._yaw.clear()
        self._last_raw_yaw = None

    def update(self, t: float, x: float, y: float, yaw: float) -> None:
        """Ingest one pose sample. De-duplicates held/near-identical poses."""
        if self._t:
            if t - self._t[-1] < _MIN_SAMPLE_DT:
                return
            if (
                abs(x - self._x[-1]) < _POS_EPS
                and abs(y - self._y[-1]) < _POS_EPS
                and abs(angle_diff(yaw, self._last_raw_yaw)) < _YAW_EPS  # type: ignore[arg-type]
            ):
                # Same pose, held between odom updates — no new information.
                return
        # Unwrap yaw onto a continuous axis so the polynomial fit (and its
        # derivative) never sees a +-pi jump.
        if self._yaw:
            unwrapped = self._yaw[-1] + angle_diff(yaw, self._last_raw_yaw)  # type: ignore[arg-type]
        else:
            unwrapped = yaw
        self._t.append(t)
        self._x.append(x)
        self._y.append(y)
        self._yaw.append(unwrapped)
        self._last_raw_yaw = yaw

    def world_velocity(self) -> tuple[float, float, float] | None:
        """Latest ``(vx_world, vy_world, yaw_rate)`` or ``None`` if too few
        samples / degenerate time span."""
        n = len(self._t)
        if n < _MIN_SAMPLES:
            return None
        t = np.asarray(self._t, dtype=float)
        t0 = t[-1]
        ts = t - t0  # derivative evaluated at the latest sample (ts = 0)
        if ts[-1] - ts[0] < _MIN_SAMPLE_DT:
            return None
        poly = min(self._poly, n - 1)
        vx = _deriv_at_zero(ts, np.asarray(self._x, dtype=float), poly)
        vy = _deriv_at_zero(ts, np.asarray(self._y, dtype=float), poly)
        wz = _deriv_at_zero(ts, np.asarray(self._yaw, dtype=float), poly)
        return vx, vy, wz

    def body_velocity(self, yaw: float) -> tuple[float, float, float] | None:
        """Latest body-frame ``(vx, vy, wz)`` or ``None`` if not enough samples.

        Rotate the world velocity by the *current* yaw. The derivative is read
        at the most recent sample (``t=0`` after the time shift), and for the
        quadratic (or higher) fit used here that endpoint derivative has
        essentially no group delay, so the velocity estimate corresponds to
        "now" and the current heading is the right rotation. (An earlier
        ``yaw - wz*span/2`` lag correction assumed a half-window delay that only
        a *linear* fit has; with the quadratic fit it over-rotated and injected
        a phantom lateral velocity on curves — the opposite of the intent.)
        """
        w = self.world_velocity()
        if w is None:
            return None
        vxw, vyw, wz = w
        c, s = math.cos(yaw), math.sin(yaw)
        return c * vxw + s * vyw, -s * vxw + c * vyw, wz


def _deriv_at_zero(ts: np.ndarray, values: np.ndarray, poly: int) -> float:
    """Derivative of a degree-``poly`` least-squares fit, evaluated at t=0
    (which the caller has shifted to the most recent sample)."""
    if poly < 1:
        return 0.0
    coeffs = np.polyfit(ts, values, poly)  # highest power first
    # d/dt of (... + c1*t + c0) at t=0 is the linear coefficient.
    return float(coeffs[-2])


__all__ = ["BodyVelocityEstimator"]
