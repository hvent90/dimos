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

"""Active Disturbance Rejection (ESO/ADRC) layer for the velocity command.

The trajectory tracker works on a clean wheeled base (FlowBase) and fails on
curves at speed on a Go2. The control *math* is the same, so the fault is the
Go2 velocity-command plant: a first-order-plus-dead-time (FOPDT) response with
a varying gain, plus nonlinearity and gait stochasticity from the black-box
onboard locomotion controller. Tuning the outer gains cannot escape this — the
fix has to be structural.

This module adds a per-axis **Extended State Observer / ADRC** inner loop that
sits exactly where the steady-state gain inversion sits today
(:class:`~dimos.control.tasks.feedforward_gain_compensator.FeedforwardGainCompensator`).
It is opt-in: off, the tracker is byte-for-byte unchanged.

The idea (Han; Gao's bandwidth parameterization; Feng & Guo): we already KNOW
the nominal first-order model (the FOPDT fit gives K and tau), so we use a
**model-based ESO** that estimates only the *extra* total disturbance ``d`` —
everything the nominal model misses: the high-speed gain sag, the deadtime
residual, the gait stochasticity, slow drift. Per velocity channel,

    y_dot = -y/tau + (K/tau) * u + d

with ``y`` the measured body velocity, ``u`` the command, and ``d`` the lumped
extra disturbance. The ESO estimates ``y`` (``z1``) and ``d`` (``z2``) from the
command and the (noisy) measured velocity, and the control law cancels ``d`` on
top of the nominal gain inversion:

    u = (r - tau * z2) / K

where ``r`` is the velocity the outer FF+P controller asked for. Read it as
**today's gain inversion ``r/K`` plus a disturbance correction ``-tau*z2/K``**.
Two consequences that the alternative (lumping ``-y/tau`` into the disturbance)
does NOT give, and which matter at 16-18 Hz:

* When ``d = 0`` the law is exactly ``u = r/K`` — byte-for-byte the baseline.
  On curve entry ``z2`` starts at 0, so the command starts identical to the
  baseline and only deviates as the real disturbance reveals itself. No added
  inner-loop lag (the failure mode of the lumped form, which is forced to run a
  velocity-feedback loop too slow for the odometry and spirals on curves).
* The control law uses only ``z2`` (the disturbance integral, naturally
  low-pass), not ``z1``, so odometry-differentiation noise is not amplified
  straight into the command.

The one knob is the observer bandwidth ``w_o`` (the disturbance-tracking rate).
The binding limit on it is NOT the deadtime but the NOISE: the Go2's dominant
disturbance (its mis-calibrated gain) is essentially DC, captured even at low
``w_o``, while a high ``w_o`` chases the broadband gait/odom noise and injects
it onto otherwise-clean axes (it destabilises straights). A sim dial sweep
puts the clean net-win region near ``w_o ~= 0.25 rad/s`` for 16-18 Hz odom, with
a noise cliff above ~2 rad/s — so ``w_o`` is set low, not at the (much looser)
deadtime margin.
The inner reference-tracking rate is pinned to the plant's own ``1/tau`` — the
odom feedback cannot support faster, and faster is what chatters. ``bandwidth``
is the single precision(up)-vs-robustness(down) dial: ``1.0`` is the calibrated
default, ``>1`` is more aggressive (toward the cliff), ``<1`` more conservative.

The observer integrates with internal sub-steps so it stays stable for any
timestep — the command loop may run as slow as ~6 Hz (the command-rate
throttle) with gaps the size of several odom periods.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dimos.control.tasks.trajectory_tracking_task.config import TrackingConfig

# --- ESO tuning rule (all overridable via build_eso) ---------------------
# Observer bandwidth at dial 1.0 (rad/s). Calibrated to the 16-18 Hz noisy odom
# by the sim dial sweep (eso_sim_ab --dial-grid): ~0.25 rad/s is the clean
# net-win point — every benchmark path/speed improves and none regress. It is
# deliberately low: the Go2's dominant disturbance (its DC gain error) is
# captured at this bandwidth, while staying well clear of the ~2 rad/s noise
# cliff where the observer starts chasing gait/odom noise onto straights. The
# dial scales this linearly (so dial 2-3 is still safe; the cap binds beyond).
_WO_BASE = 0.25
# Hard ceiling on w_o (rad/s) regardless of dial: above ~2 rad/s the observer
# starts chasing gait/odom noise and destabilises low-speed straights (the
# noise cliff seen in the sweep). Also bounded by the inner-loop effective
# delay (plant deadtime + velocity-estimator group delay): w_o * L_eff < margin.
_WO_ABS_CAP = 2.5
_WO_DELAY_MARGIN = 0.6
# The velocity estimator (quadratic-fit endpoint derivative) adds only a small
# group delay; fold a conservative estimate into L_eff for the delay cap so the
# loop never out-runs its own feedback. (This cap is not binding at the default
# low w_o; it matters only when the dial is pushed high.)
_DEFAULT_MEAS_DELAY = 0.05
_L_EFF_FLOOR = 0.08
# Lower floor so a small dial still tracks the slow disturbance without freezing.
_WO_FLOOR = 0.1
# Floors so the derived rates never blow up on a degenerate fit.
_TAU_FLOOR = 0.05
_K_FLOOR = 0.1
# Sub-step the observer integration below this dt so Euler stays stable and
# accurate even when the command loop runs slow (~6 Hz) with held commands.
_OBS_SUBSTEP = 0.02


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


@dataclass(frozen=True)
class AxisESOConfig:
    """Per-axis model-based ESO parameters.

    ``K`` and ``tau`` are the nominal FOPDT gain / time constant (the model the
    observer knows), ``w_o`` the observer (disturbance-tracking) bandwidth
    [rad/s], ``u_limit`` the symmetric command clamp (the same ceiling the
    gain-inversion compensator uses).
    """

    K: float
    tau: float
    w_o: float
    u_limit: float


class AxisESO:
    """Model-based ESO + disturbance-cancelling control for one velocity axis.

    The observer knows the nominal first-order model and estimates the velocity
    ``z1`` and the *extra* disturbance ``z2``::

        z1_dot = -z1/tau + (K/tau) u + z2 + l1 (y - z1)
        z2_dot = l2 (y - z1)

    with error poles placed at ``-w_o`` (double): ``l1 = 2 w_o - 1/tau``,
    ``l2 = w_o^2``. The control cancels the disturbance on top of the nominal
    gain inversion::

        u = (r - tau * z2) / K

    so ``z2 = 0`` reproduces the baseline ``r/K`` exactly, and only ``z2`` (the
    low-pass disturbance integral) — never the noisy velocity estimate —
    reaches the command. The integration is sub-stepped for stability at any
    ``dt``; the applied (clamped) command feeds the next prediction, giving
    built-in anti-windup.
    """

    def __init__(self, cfg: AxisESOConfig) -> None:
        self.cfg = cfg
        self._inv_tau = 1.0 / cfg.tau
        self._l1 = 2.0 * cfg.w_o - self._inv_tau
        self._l2 = cfg.w_o * cfg.w_o
        self.reset()

    def reset(self) -> None:
        self._z1 = 0.0
        self._z2 = 0.0
        self._u = 0.0  # command applied over the previous interval
        self._init = False

    @property
    def disturbance(self) -> float:
        """Latest extra-disturbance estimate (diagnostics/plots)."""
        return self._z2

    @property
    def velocity_estimate(self) -> float:
        return self._z1

    def compute(self, r: float, y: float | None, dt: float) -> float:
        """One inner-loop step.

        ``r`` is the desired body velocity from the outer controller (physical
        units), ``y`` the measured body velocity (physical units) or ``None``
        when no fresh estimate is available (predict-only, no correction),
        ``dt`` the time since the previous call. Returns the command ``u``.
        """
        # A non-finite measurement is treated as "no measurement" (predict-only)
        # so one bad odom sample can never poison the estimator state.
        if y is not None and not math.isfinite(y):
            y = None
        if not self._init:
            self._z1 = y if y is not None else r
            self._z2 = 0.0
            self._init = True
            self._u = self._control(r)
            return self._u

        if dt > 0.0:
            self._observe(y, dt)
        self._u = self._control(r)
        return self._u

    def _observe(self, y: float | None, dt: float) -> None:
        cfg = self.cfg
        n = max(1, math.ceil(dt / _OBS_SUBSTEP))
        h = dt / n
        drive = cfg.K * self._inv_tau * self._u  # (K/tau) u, held over the step
        for _ in range(n):
            innov = (y - self._z1) if y is not None else 0.0
            z1_dot = -self._inv_tau * self._z1 + drive + self._z2 + self._l1 * innov
            z2_dot = self._l2 * innov
            self._z1 += h * z1_dot
            self._z2 += h * z2_dot

    def _control(self, r: float) -> float:
        # Nominal gain inversion r/K minus the disturbance correction tau*z2/K.
        u = (r - self.cfg.tau * self._z2) / self.cfg.K
        if not math.isfinite(u):
            return 0.0  # never let a non-finite estimate become a clamp-rail command
        return _clamp(u, self.cfg.u_limit)


class ESOCompensator:
    """Per-axis ESO/ADRC inner loop for the (vx, vy, wz) twist base.

    Drop-in sibling of
    :class:`~dimos.control.tasks.feedforward_gain_compensator.FeedforwardGainCompensator`,
    but stateful and feedback-driven: it needs the measured body velocity and
    the timestep, so its ``compute`` signature carries them. The task selects
    this block instead of the gain inversion when the ESO is enabled.
    """

    def __init__(self, vx: AxisESO, vy: AxisESO, wz: AxisESO) -> None:
        self.vx = vx
        self.vy = vy
        self.wz = wz

    def compute(
        self,
        desired: tuple[float, float, float],
        measured: tuple[float, float, float] | None,
        dt: float,
    ) -> tuple[float, float, float]:
        """Disturbance-rejected command for ``desired`` body velocity.

        ``measured`` is the estimated body velocity ``(vx, vy, wz)`` or ``None``
        (predict-only this step). ``dt`` is the time since the previous call.
        """
        mx, my, mz = measured if measured is not None else (None, None, None)
        return (
            self.vx.compute(desired[0], mx, dt),
            self.vy.compute(desired[1], my, dt),
            self.wz.compute(desired[2], mz, dt),
        )

    def reset(self) -> None:
        self.vx.reset()
        self.vy.reset()
        self.wz.reset()


def _axis_config(
    K: float, tau: float, L: float, u_limit: float, bandwidth: float, meas_delay: float
) -> AxisESOConfig:
    tau = max(tau, _TAU_FLOOR)
    K = K if abs(K) >= _K_FLOOR else math.copysign(_K_FLOOR, K) if K else _K_FLOOR
    # Observer bandwidth: the noise-calibrated base, scaled by the dial, then
    # hard-capped (noise cliff) and delay-capped, then floored.
    l_eff = max(L + meas_delay, _L_EFF_FLOOR)
    w_o = bandwidth * _WO_BASE
    w_o = min(w_o, _WO_ABS_CAP, _WO_DELAY_MARGIN / l_eff)
    w_o = max(w_o, _WO_FLOOR)
    return AxisESOConfig(K=K, tau=tau, w_o=w_o, u_limit=u_limit)


def build_eso(
    tracking: TrackingConfig,
    *,
    bandwidth: float = 1.0,
    meas_delay: float = _DEFAULT_MEAS_DELAY,
) -> ESOCompensator:
    """Build an :class:`ESOCompensator` from the FOPDT fit in ``tracking``.

    ``bandwidth`` is the single precision(>1)-vs-robustness(<1) dial: per axis
    the observer bandwidth is ``w_o = bandwidth * 0.25 rad/s``, hard-capped at
    2.5 rad/s (the noise cliff) and at ``0.6 / (L + meas_delay)`` (the
    inner-loop delay margin). The nominal model uses the fit ``K`` and ``tau``
    directly. ``meas_delay`` (s) is the velocity-estimator group delay.
    """
    limit = tracking.ff_output_limit
    return ESOCompensator(
        vx=AxisESO(
            _axis_config(
                tracking.k_hat.x, tracking.tau.x, tracking.deadtime.x, limit.x, bandwidth, meas_delay
            )
        ),
        vy=AxisESO(
            _axis_config(
                tracking.k_hat.y, tracking.tau.y, tracking.deadtime.y, limit.y, bandwidth, meas_delay
            )
        ),
        wz=AxisESO(
            _axis_config(
                tracking.k_hat.yaw,
                tracking.tau.yaw,
                tracking.deadtime.yaw,
                limit.yaw,
                bandwidth,
                meas_delay,
            )
        ),
    )


__all__ = [
    "AxisESO",
    "AxisESOConfig",
    "ESOCompensator",
    "build_eso",
]
