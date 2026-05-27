# Copyright 2026 Dimensional Inc.
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

"""Reference governor: model-based per-waypoint velocity profile.

Wraps the existing curvature MVC + forward/backward accel passes from
:class:`dimos.control.tasks.velocity_profiler.VelocityProfiler` with one
NEW constraint — a precision cap derived from the FOPDT plant floor:

    v <= e_max / max(tau_vx + L_vx, tau_wz + L_wz)

(the empirical straight-line CTE floor is ~(tau+L)*v below ω-saturation;
on curved paths the wz-channel lag dominates instead, so the constraint
takes the worse channel — see PrecisionMVC docstring). The artifact's
v_max / ω_max / a_lat caps are unchanged and remain in the constraint
set; the governor composes ``min(all_constraints)`` per waypoint and
runs the existing accel passes.

An optional closed-loop alpha-feedback variant (config flag ``closed_loop``)
observes per-tick CTE and multiplicatively scales the open-loop output
by alpha ∈ [alpha_min, 1.0]. Shipped opt-in; empirically does NOT converge on
continuously-curving paths (see ``tuning_README.md`` "Closed-loop
alpha-feedback variant — NEGATIVE result" for the data and the structural
follow-up options).

The governor is a dimos Module (not a ControlTask) — the coordinator's
tick loop has no upstream-non-actuating slot. It exposes the
:class:`PathSpeedCapProtocol` method shape (``for_path``,
``speed_limit_at``, ``cap``) so it can be injected as the baseline
follower's ``_profile_cap`` (the existing per-tick consumption seam).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import threading
from typing import Any, Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.control.tasks.velocity_profiler import VelocityProfiler
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.benchmarking.scoring import nearest_segment
from dimos.utils.benchmarking.tuning import PlantModelDC, TuningConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ---------------------------------------------------------------------------
# PathSpeedCap method contract — the consumption seam in BaselinePathFollowerTask.
# ---------------------------------------------------------------------------


@runtime_checkable
class PathSpeedCapProtocol(Protocol):
    """Duck-type contract for objects that can be installed as the
    follower's ``_profile_cap``. Mirrors the shape of
    :class:`dimos.utils.benchmarking.velocity_profile.PathSpeedCap`.
    """

    def for_path(self, path: Path) -> None: ...

    def speed_limit_at(self, x: float, y: float) -> float: ...

    def cap(
        self, x: float, y: float, vx: float, vy: float, wz: float
    ) -> tuple[float, float, float]: ...


# ---------------------------------------------------------------------------
# Velocity constraint generators — per-waypoint pure-function upper bounds.
#
# Architecturally the solver applies min() over the constraint set then runs
# the existing accel passes. To add a new constraint: drop in one more class,
# include it in the governor's constraint list. No solver change needed.
# ---------------------------------------------------------------------------


@dataclass
class ConstraintContext:
    """Path-derived context passed to each constraint's upper_bound.

    Bundling curvatures (computed once per path) avoids each curvature-
    dependent constraint recomputing the same numbers.
    """

    path: Path
    curvatures: NDArray[np.float64]  # length len(path.poses); abs curvature in 1/m
    plant: PlantModelDC


class VelocityConstraint(Protocol):
    name: str

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float: ...


_INF = float("inf")
_KAPPA_EPS = 1e-6


@dataclass
class GeometricMVC:
    """Static linear-speed cap (artifact's ``velocity_profile.max_linear_speed``)."""

    v_max: float
    name: str = "geometric"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        return float(self.v_max)


@dataclass
class SaturationMVC:
    """Turn-rate saturation: v <= omega_max / |kappa|. HARD cap — above
    this the controller can't track the geometry at all."""

    omega_max: float
    name: str = "saturation"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        kappa = float(ctx.curvatures[s_idx])
        if kappa < _KAPPA_EPS:
            return _INF
        return self.omega_max / kappa


@dataclass
class LateralMVC:
    """Lateral-comfort cap: v <= sqrt(a_lat_max / |kappa|)."""

    a_lat_max: float
    name: str = "lateral"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        kappa = float(ctx.curvatures[s_idx])
        if kappa < _KAPPA_EPS:
            return _INF
        return float(np.sqrt(self.a_lat_max / kappa))


@dataclass
class PrecisionMVC:
    """Precision cap derived from the FOPDT plant CTE floor:

        v <= e_max / max(tau_vx + L_vx, tau_wz + L_wz)

    The straight-line characterization fits only `(tau_vx + L_vx) * v` as
    the CTE floor; that holds on straight segments. On curved segments
    the heading-tracking lag (a function of `tau_wz + L_wz`) dominates
    instead. Empirically (slalom/smooth_corner/figure_eight sim runs)
    using only the vx channel under-predicts CTE on curved paths by ~2x;
    taking max(vx_chan, wz_chan) halves the residual at the cost of a
    proportionally lower v.

    e_max read via a callable so the governor can hot-update the runtime
    corridor half-width without rebuilding the constraint. The bound is
    constant across waypoints (κ-independent); the min() in the solver
    handles composition with the κ-dependent caps.
    """

    e_max_provider: Callable[[], float]
    min_e_max: float = 0.005  # 5 mm floor: keeps v from collapsing to 0
    name: str = "precision"

    def upper_bound(self, ctx: ConstraintContext, s_idx: int) -> float:
        e_max = max(float(self.e_max_provider()), self.min_e_max)
        tau_plus_L = max(
            float(ctx.plant.vx.tau + ctx.plant.vx.L),
            float(ctx.plant.wz.tau + ctx.plant.wz.L),
        )
        if tau_plus_L < 1e-9:
            return _INF
        return e_max / tau_plus_L


# ---------------------------------------------------------------------------
# Solver: compose constraints, then reuse existing accel passes.
# ---------------------------------------------------------------------------


def _path_pts(path: Path) -> NDArray[np.float64]:
    return np.array([[p.position.x, p.position.y] for p in path.poses], dtype=float)


def solve_profile(
    path: Path,
    plant: PlantModelDC,
    constraints: Sequence[VelocityConstraint],
    *,
    accel_max: float,
    decel_max: float,
    min_speed: float,
    curvatures: NDArray[np.float64] | None = None,
    pts: NDArray[np.float64] | None = None,
) -> NDArray[np.float64]:
    """Per-waypoint MVC = min(constraints), then forward/backward accel
    passes via the existing :class:`VelocityProfiler` helpers. ``pts``
    and ``curvatures`` can be passed in to avoid recomputing on hot
    paths (e_max updates with a fixed path)."""
    n = len(path.poses)
    if n < 2:
        return np.array([min_speed], dtype=float)

    if pts is None:
        pts = _path_pts(path)
    if curvatures is None:
        # _compute_curvatures depends only on pts (it's stateless w.r.t. profiler config).
        curvatures = VelocityProfiler()._compute_curvatures(pts)

    ctx = ConstraintContext(path=path, curvatures=curvatures, plant=plant)
    mvc = np.empty(n, dtype=float)
    for i in range(n):
        mvc[i] = min(c.upper_bound(ctx, i) for c in constraints)

    # Reuse the forward/backward accel passes verbatim.
    profiler = VelocityProfiler(
        max_linear_accel=accel_max,
        max_linear_decel=decel_max,
    )
    v = profiler._acceleration_pass(pts, mvc, forward=True)
    v = profiler._acceleration_pass(pts, v, forward=False)
    return np.maximum(v, min_speed)


# ---------------------------------------------------------------------------
# Module: lifecycle, In streams, atomic-snapshot state.
# ---------------------------------------------------------------------------


class ReferenceGovernorConfig(ModuleConfig):
    plant_artifact_path: str
    e_max_default: float = 0.1
    min_e_max: float = 0.005
    lookahead_pts: int = 8

    # --- Closed-loop alpha-feedback (OPT-IN; default off = open-loop) -------
    # Background: the open-loop precision bound v ≤ e_max / max(τ+L per
    # channel) under-promises CTE on continuously-curving paths by ~2×
    # because the FOPDT lag model doesn't see the controller's
    # heading-chase dynamics. Closing the loop on measured CTE corrects
    # this via a multiplicative scaling factor alpha ∈ [alpha_min, 1.0] applied
    # to the open-loop profile output.
    closed_loop: bool = False
    kp_alpha: float = 4.0  # P gain on (cte - e_max) → -Δalpha
    ki_alpha: float = 0.5  # I gain (1/s); produces zero steady-state error
    alpha_min: float = 0.2  # floor: never starve v below 20% of open-loop
    max_integral: float = 0.5  # anti-windup clamp on the integral state
    cte_ema_alpha: float = 0.3  # EMA factor on raw CTE; 0.3 ≈ 3-tick window @ 10Hz
    tick_dt_s: float = 0.05  # nominal tick period for integrating error


class ReferenceGovernor(Module):
    """Per-waypoint velocity-profile producer for the baseline path
    follower. Consumes (path, e_max). Outputs are read via the
    :class:`PathSpeedCapProtocol` methods on each follower tick.
    """

    config: ReferenceGovernorConfig

    path: In[Path]
    corridor_half_width: In[float]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        cfg = self.config

        artifact = TuningConfig.from_json(cfg.plant_artifact_path)
        self._tuning_config = artifact
        self._plant = artifact.plant
        vp = artifact.velocity_profile
        self._accel_max = vp.max_linear_accel
        self._decel_max = vp.max_linear_decel
        self._min_speed = vp.min_speed
        self._lookahead_pts = cfg.lookahead_pts

        # State (all guarded by _state_lock).
        self._state_lock = threading.Lock()
        self._path: Path | None = None
        self._pts: NDArray[np.float64] | None = None
        self._curvatures: NDArray[np.float64] | None = None
        self._profile: NDArray[np.float64] | None = None
        if cfg.e_max_default <= 0:
            raise ValueError(f"e_max_default must be > 0, got {cfg.e_max_default}")
        self._e_max: float = max(cfg.e_max_default, cfg.min_e_max)

        # Constraints — PrecisionMVC reads _e_max via _current_e_max
        # under the lock so the constraint always sees a consistent value
        # even while recompute is running on another thread.
        self._constraints: list[VelocityConstraint] = [
            GeometricMVC(v_max=vp.max_linear_speed),
            SaturationMVC(omega_max=vp.max_angular_speed),
            LateralMVC(a_lat_max=vp.max_centripetal_accel),
            PrecisionMVC(e_max_provider=self._current_e_max, min_e_max=cfg.min_e_max),
        ]

        # Closed-loop alpha-feedback state (also guarded by _state_lock).
        # When closed_loop=False these stay at their initial values and
        # speed_limit_at() short-circuits the feedback path entirely.
        self._alpha: float = 1.0
        self._alpha_integral: float = 0.0
        self._cte_filtered: float = 0.0

    # ----- lifecycle ------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        self.register_disposable(Disposable(self.corridor_half_width.subscribe(self._on_e_max)))

    @rpc
    def stop(self) -> None:
        super().stop()

    # ----- imperative API (also RPC-callable) -----------------------------

    @rpc
    def set_path(self, path: Path) -> None:
        """Install a new path and compute its profile."""
        self._on_path(path)

    @rpc
    def update_e_max(self, value: float) -> None:
        """Update the corridor half-width and recompute (if a path is set)."""
        self._on_e_max(value)

    # ----- PathSpeedCap interface (duck-typed; called per-tick from the
    # follower's compute() on the coordinator tick thread) ----------------

    def for_path(self, path: Path) -> None:
        """Alias of set_path. Lets the governor drop into the follower's
        existing PathSpeedCap injection seam (the follower calls
        ``cap.for_path(path)`` inside ``start_path``)."""
        self._on_path(path)

    def speed_limit_at(self, x: float, y: float) -> float:
        # Atomic snapshot under the lock; lock-free work afterwards.
        with self._state_lock:
            profile = self._profile
            pts = self._pts
            min_speed = self._min_speed
            alpha = self._alpha
        if profile is None or pts is None or len(profile) == 0:
            # No path installed yet — fall back to the artifact's v_max
            # so the follower's cap is a no-op until set_path is called.
            return float(self._tuning_config.velocity_profile.max_linear_speed)

        # Closed-loop alpha-feedback: measure CTE from current pose, update alpha
        # via PI law, write back atomically. Open-loop short-circuit keeps
        # alpha at its initial 1.0 so this becomes a no-op multiplier.
        if self.config.closed_loop:
            cte_raw = self._measure_cte(x, y, pts)
            alpha, integral, filtered = self._update_alpha(cte_raw)
            with self._state_lock:
                self._alpha = alpha
                self._alpha_integral = integral
                self._cte_filtered = filtered

        i = int(np.argmin(np.sum((pts - np.array([x, y])) ** 2, axis=1)))
        j = min(len(profile), i + max(1, self._lookahead_pts))
        return float(max(alpha * float(np.min(profile[i:j])), min_speed))

    def cap(
        self, x: float, y: float, vx: float, vy: float, wz: float
    ) -> tuple[float, float, float]:
        vlim = self.speed_limit_at(x, y)
        s = abs(vx)
        if s <= vlim or s < 1e-9:
            return vx, vy, wz
        k = vlim / s
        return vx * k, vy * k, wz * k

    def speed_at(self, s_idx: int, fallback: float | None = None) -> float:
        """Direct index-based lookup (debug/test seam)."""
        with self._state_lock:
            profile = self._profile
        if profile is None or len(profile) == 0:
            if fallback is None:
                raise RuntimeError("ReferenceGovernor.speed_at: no path installed")
            return float(fallback)
        idx = max(0, min(s_idx, len(profile) - 1))
        return float(profile[idx])

    # ----- internals ------------------------------------------------------

    def _current_e_max(self) -> float:
        # No lock here — the constraint is called from within a recompute
        # that already holds the snapshot of e_max it was triggered with.
        # (Holding the lock here would self-deadlock.)
        return self._e_max

    # ----- closed-loop alpha-feedback helpers ---------------------------------

    def _measure_cte(self, x: float, y: float, pts: NDArray[np.float64]) -> float:
        """Perpendicular distance from (x, y) to the nearest path segment.
        Thin wrapper over scoring.nearest_segment so the governor's
        closed-loop math is decoupled from the scoring module's call shape."""
        _, perp_dist, _ = nearest_segment(np.array([x, y], dtype=float), pts)
        return float(perp_dist)

    def _update_alpha(self, cte_raw: float) -> tuple[float, float, float]:
        """PI feedback on (cte - e_max) with anti-windup + EMA pre-filter.

        Pure function of inputs + the current alpha/integral/filtered_cte
        snapshot — caller writes back under _state_lock. Returns
        (new_alpha, new_integral, new_filtered_cte).

        Convention: positive error (cte > e_max) shrinks alpha toward
        alpha_min; negative error (cte < e_max) grows alpha back toward 1.0.
        The 1-sided clamp at alpha=1.0 means the closed-loop never asks for v
        ABOVE the open-loop bound — only at-or-below.
        """
        cfg = self.config

        # Snapshot current state (atomic; cheap).
        with self._state_lock:
            integral = self._alpha_integral
            filtered_prev = self._cte_filtered

        # 1. EMA-smooth the raw CTE to reject single-tick noise spikes.
        ema = float(cfg.cte_ema_alpha)
        cte = ema * float(cte_raw) + (1.0 - ema) * filtered_prev

        # 2. Error in metres; positive means we're outside the corridor.
        error = cte - self._e_max
        dt = float(cfg.tick_dt_s)

        # 3. Tentative integral update + clamp (anti-windup).
        integral_candidate = integral + error * dt
        max_int = float(cfg.max_integral)
        integral_candidate = max(-max_int, min(max_int, integral_candidate))

        # 4. PI output; clamp alpha into [alpha_min, 1.0].
        alpha_min = float(cfg.alpha_min)
        kp = float(cfg.kp_alpha)
        ki = float(cfg.ki_alpha)
        alpha_raw = 1.0 - kp * error - ki * integral_candidate
        alpha_new = max(alpha_min, min(1.0, alpha_raw))

        # 5. Anti-windup: if alpha saturated against a clamp this tick AND
        # the integral move is in the saturating direction, freeze the
        # integral. Prevents the integral from winding up while alpha can't
        # respond.
        if alpha_new == alpha_min and alpha_raw < alpha_min:
            # alpha pegged low; integral would push alpha even lower → freeze.
            integral_new = integral
        elif alpha_new == 1.0 and alpha_raw > 1.0:
            integral_new = integral
        else:
            integral_new = integral_candidate

        return alpha_new, integral_new, cte

    def _on_path(self, path: Path) -> None:
        if path is None or len(path.poses) < 2:
            logger.warning(
                "ReferenceGovernor: ignored invalid path "
                f"(poses={0 if path is None else len(path.poses)})"
            )
            return
        # Compute pts + curvatures outside the lock (pure numpy).
        pts = _path_pts(path)
        curvatures = VelocityProfiler()._compute_curvatures(pts)
        # Lock briefly to snapshot e_max for the recompute.
        with self._state_lock:
            e_max_snapshot = self._e_max
        profile = self._compute_profile(path, pts, curvatures, e_max_snapshot)
        with self._state_lock:
            self._path = path
            self._pts = pts
            self._curvatures = curvatures
            self._profile = profile
            # Reset closed-loop state on every new path so feedback from
            # the prior path doesn't carry over (different geometry, fresh
            # convergence).
            self._alpha = 1.0
            self._alpha_integral = 0.0
            self._cte_filtered = 0.0
        self._log_provenance("path", profile, e_max_snapshot)

    def _on_e_max(self, value: float) -> None:
        if not np.isfinite(value) or value <= 0:
            logger.warning(f"ReferenceGovernor: ignored non-positive e_max={value}")
            return
        clamped = max(float(value), self.config.min_e_max)
        # Need pts/curvatures/path under the lock; recompute outside.
        with self._state_lock:
            path = self._path
            pts = self._pts
            curvatures = self._curvatures
        if path is None or pts is None or curvatures is None:
            # No path yet — just stash the value; recompute will pick it up
            # when set_path is called.
            with self._state_lock:
                self._e_max = clamped
            return
        # Set e_max BEFORE solving so PrecisionMVC reads the new value.
        with self._state_lock:
            self._e_max = clamped
        profile = self._compute_profile(path, pts, curvatures, clamped)
        with self._state_lock:
            self._profile = profile
        self._log_provenance("e_max", profile, clamped)

    def _compute_profile(
        self,
        path: Path,
        pts: NDArray[np.float64],
        curvatures: NDArray[np.float64],
        e_max: float,  # unused directly; PrecisionMVC reads via provider
    ) -> NDArray[np.float64]:
        del e_max  # documented above; kept for log signature parity
        return solve_profile(
            path,
            self._plant,
            self._constraints,
            accel_max=self._accel_max,
            decel_max=self._decel_max,
            min_speed=self._min_speed,
            curvatures=curvatures,
            pts=pts,
        )

    def _log_provenance(
        self,
        trigger: str,
        profile: NDArray[np.float64],
        e_max: float,
    ) -> None:
        # Which constraint binds at each waypoint? (Debug surface — count by name.)
        with self._state_lock:
            curvatures = self._curvatures
            path = self._path
        if curvatures is None or path is None:
            return
        ctx = ConstraintContext(path=path, curvatures=curvatures, plant=self._plant)
        n = len(profile)
        binding_counts: dict[str, int] = {c.name: 0 for c in self._constraints}
        for i in range(n):
            bounds = [(c.name, c.upper_bound(ctx, i)) for c in self._constraints]
            binding = min(bounds, key=lambda t: t[1])[0]
            binding_counts[binding] = binding_counts.get(binding, 0) + 1
        logger.info(
            "ReferenceGovernor recomputed",
            trigger=trigger,
            e_max=round(e_max, 4),
            v_min=round(float(np.min(profile)), 3),
            v_max=round(float(np.max(profile)), 3),
            n_waypoints=n,
            binding=binding_counts,
        )


__all__ = [
    "ConstraintContext",
    "GeometricMVC",
    "LateralMVC",
    "PathSpeedCapProtocol",
    "PrecisionMVC",
    "ReferenceGovernor",
    "ReferenceGovernorConfig",
    "SaturationMVC",
    "VelocityConstraint",
    "solve_profile",
]
