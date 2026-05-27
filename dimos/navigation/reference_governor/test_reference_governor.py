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

from __future__ import annotations

from collections.abc import Generator
import math
from pathlib import Path as FsPath
import threading
import time

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.reference_governor.reference_governor import (
    ConstraintContext,
    GeometricMVC,
    LateralMVC,
    PrecisionMVC,
    ReferenceGovernor,
    SaturationMVC,
    solve_profile,
)
from dimos.utils.benchmarking.paths import circle, single_corner, straight_line
from dimos.utils.benchmarking.tuning import (
    FeedforwardDC,
    FopdtChannelDC,
    PlantModelDC,
    Provenance,
    RecommendedControllerDC,
    TuningConfig,
    VelocityProfileDC,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _plant() -> PlantModelDC:
    """Go2-class FOPDT plant.
    vx: tau+L = 0.46s; wz: tau+L = 0.65s.
    PrecisionMVC takes max() across channels, so 0.65 is the divisor."""
    return PlantModelDC(
        vx=FopdtChannelDC(K=0.922, tau=0.4, L=0.06),
        vy=FopdtChannelDC(K=1.0, tau=0.4, L=0.06),
        wz=FopdtChannelDC(K=2.45, tau=0.6, L=0.05),
    )


def _ctx(path: Path) -> ConstraintContext:
    from dimos.control.tasks.velocity_profiler import VelocityProfiler

    pts = np.array([[p.position.x, p.position.y] for p in path.poses], dtype=float)
    curvatures = VelocityProfiler()._compute_curvatures(pts)
    return ConstraintContext(path=path, curvatures=curvatures, plant=_plant())


@pytest.fixture()
def artifact_path(tmp_path: FsPath) -> FsPath:
    """Write a minimal valid TuningConfig artifact to tmp_path."""
    cfg = TuningConfig(
        provenance=Provenance(
            robot_id="go2-test",
            surface="test",
            mode="default",
            date="2026-05-25",
            git_sha="testtest",
            sim_or_hw="hw",
        ),
        plant=_plant(),
        feedforward=FeedforwardDC(K_vx=1.085, K_vy=1.0, K_wz=0.408),
        velocity_profile=VelocityProfileDC(
            max_linear_speed=1.0,
            max_angular_speed=1.275,
            max_centripetal_accel=1.0,
            max_linear_accel=2.5,
            max_linear_decel=5.0,
        ),
        recommended_controller=RecommendedControllerDC(),
        caveats=[],
    )
    p = tmp_path / "test_config.json"
    cfg.to_json(p)
    return p


@pytest.fixture()
def governor(artifact_path: FsPath) -> Generator[ReferenceGovernor, None, None]:
    gov = ReferenceGovernor(
        plant_artifact_path=str(artifact_path),
        e_max_default=0.05,
    )
    try:
        yield gov
    finally:
        gov._close_module()


# ---------------------------------------------------------------------------
# Constraint generators — pure-function tests
# ---------------------------------------------------------------------------


class TestGeometricMVC:
    def test_returns_v_max_everywhere(self) -> None:
        ctx = _ctx(straight_line(length=2.0))
        c = GeometricMVC(v_max=0.7)
        assert c.upper_bound(ctx, 0) == pytest.approx(0.7)
        assert c.upper_bound(ctx, len(ctx.curvatures) // 2) == pytest.approx(0.7)
        assert c.upper_bound(ctx, len(ctx.curvatures) - 1) == pytest.approx(0.7)


class TestSaturationMVC:
    def test_infinite_at_zero_curvature(self) -> None:
        ctx = _ctx(straight_line(length=2.0))
        c = SaturationMVC(omega_max=1.5)
        # All curvatures should be ~0 on a straight line.
        for i in range(len(ctx.curvatures)):
            assert c.upper_bound(ctx, i) == float("inf")

    def test_omega_over_kappa_on_circle(self) -> None:
        # Circle of radius R: the existing _compute_curvatures uses a
        # (d1+d2) arc-length denominator (2-step), so reported kappa is
        # ~1/(2R) = 0.5 for R=1 — half the textbook 1/R. The whole
        # tuning pipeline is calibrated against this convention; our
        # constraints follow it consistently.
        ctx = _ctx(circle(radius=1.0, n_points=200))
        c = SaturationMVC(omega_max=1.275)
        v = c.upper_bound(ctx, len(ctx.curvatures) // 2)
        # omega/kappa = 1.275 / 0.5 = 2.55
        assert v == pytest.approx(2.55, rel=0.05)


class TestLateralMVC:
    def test_infinite_at_zero_curvature(self) -> None:
        ctx = _ctx(straight_line(length=2.0))
        c = LateralMVC(a_lat_max=1.0)
        for i in range(len(ctx.curvatures)):
            assert c.upper_bound(ctx, i) == float("inf")

    def test_sqrt_a_over_kappa_on_circle(self) -> None:
        # Same kappa convention as SaturationMVC: kappa ~ 1/(2R) = 0.5 for R=1.
        ctx = _ctx(circle(radius=1.0, n_points=200))
        c = LateralMVC(a_lat_max=1.0)
        v = c.upper_bound(ctx, len(ctx.curvatures) // 2)
        # sqrt(a_lat / kappa) = sqrt(1.0 / 0.5) = sqrt(2) ≈ 1.414
        assert v == pytest.approx(math.sqrt(2.0), rel=0.05)


class TestPrecisionMVC:
    def test_constant_across_waypoints(self) -> None:
        ctx = _ctx(single_corner(leg_length=2.0, angle_deg=90.0))
        c = PrecisionMVC(e_max_provider=lambda: 0.05)
        # max(vx tau+L, wz tau+L) = max(0.46, 0.65) = 0.65
        # → 0.05/0.65 ≈ 0.0769
        expected = 0.05 / 0.65
        for i in (0, len(ctx.curvatures) // 2, len(ctx.curvatures) - 1):
            assert c.upper_bound(ctx, i) == pytest.approx(expected, rel=1e-6)

    def test_min_floor_when_e_max_is_tiny(self) -> None:
        ctx = _ctx(straight_line(length=1.0))
        c = PrecisionMVC(e_max_provider=lambda: 0.0001, min_e_max=0.005)
        # 0.005/0.65 ≈ 0.0077 — the e_max floor wins.
        assert c.upper_bound(ctx, 0) == pytest.approx(0.005 / 0.65, rel=1e-6)

    def test_reads_provider_at_call_time(self) -> None:
        ctx = _ctx(straight_line(length=1.0))
        state = {"e": 0.05}
        c = PrecisionMVC(e_max_provider=lambda: state["e"])
        v0 = c.upper_bound(ctx, 0)
        state["e"] = 0.10
        v1 = c.upper_bound(ctx, 0)
        assert v1 == pytest.approx(v0 * 2.0, rel=1e-6)


# ---------------------------------------------------------------------------
# Solver
# ---------------------------------------------------------------------------


class TestSolveProfile:
    def test_straight_line_clamped_by_precision(self) -> None:
        path = straight_line(length=2.0)
        plant = _plant()
        constraints = [
            GeometricMVC(v_max=1.0),
            SaturationMVC(omega_max=1.275),
            LateralMVC(a_lat_max=1.0),
            PrecisionMVC(e_max_provider=lambda: 0.05),
        ]
        v = solve_profile(
            path,
            plant,
            constraints,
            accel_max=2.5,
            decel_max=5.0,
            min_speed=0.05,
        )
        # On a straight line, only GeometricMVC and PrecisionMVC are
        # finite; precision is tighter (0.109 < 1.0).
        assert np.all(v == pytest.approx(0.05 / 0.65, rel=1e-4))

    def test_short_path_returns_min_speed(self) -> None:
        # Path with a single pose — degenerate, solver returns [min_speed].
        path = Path(
            poses=[
                PoseStamped(
                    position=Vector3(0.0, 0.0, 0.0),
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
                )
            ]
        )
        plant = _plant()
        constraints: list = [GeometricMVC(v_max=1.0)]
        v = solve_profile(
            path,
            plant,
            constraints,
            accel_max=2.5,
            decel_max=5.0,
            min_speed=0.05,
        )
        assert len(v) == 1
        assert v[0] == pytest.approx(0.05)

    def test_corner_slows_at_curvature(self) -> None:
        path = single_corner(leg_length=2.0, angle_deg=90.0)
        plant = _plant()
        constraints = [
            GeometricMVC(v_max=1.0),
            SaturationMVC(omega_max=1.275),
            LateralMVC(a_lat_max=1.0),
            PrecisionMVC(e_max_provider=lambda: 0.5),  # loose enough not to bind on legs
        ]
        v = solve_profile(
            path,
            plant,
            constraints,
            accel_max=2.5,
            decel_max=5.0,
            min_speed=0.05,
        )
        # Far from corner v should ~ geometric cap, at corner v should drop.
        assert v.min() < v.max(), "expected corner to slow profile"

    def test_min_speed_floor(self) -> None:
        # Sharp corner + impossibly tight e_max should still floor at min_speed.
        path = single_corner(leg_length=1.0, angle_deg=170.0)
        constraints = [
            GeometricMVC(v_max=1.0),
            SaturationMVC(omega_max=1.275),
            LateralMVC(a_lat_max=1.0),
            PrecisionMVC(e_max_provider=lambda: 0.001, min_e_max=0.001),
        ]
        v = solve_profile(
            path,
            _plant(),
            constraints,
            accel_max=2.5,
            decel_max=5.0,
            min_speed=0.05,
        )
        assert np.all(v >= 0.05 - 1e-9)


# ---------------------------------------------------------------------------
# ReferenceGovernor — Module integration
# ---------------------------------------------------------------------------


class TestReferenceGovernor:
    def test_no_path_returns_v_max_fallback(self, governor: ReferenceGovernor) -> None:
        # Before any path is set, speed_limit_at falls back to artifact v_max.
        assert governor.speed_limit_at(0.0, 0.0) == pytest.approx(1.0)

    def test_set_path_then_speed_limit_drops_to_precision(
        self, governor: ReferenceGovernor
    ) -> None:
        governor.set_path(straight_line(length=2.0))
        # e_max_default=0.05 → 0.05/0.46 ≈ 0.10870
        v = governor.speed_limit_at(1.0, 0.0)
        assert v == pytest.approx(0.05 / 0.65, rel=1e-3)

    def test_update_e_max_recomputes(self, governor: ReferenceGovernor) -> None:
        governor.set_path(straight_line(length=2.0))
        v0 = governor.speed_limit_at(1.0, 0.0)
        governor.update_e_max(0.20)
        v1 = governor.speed_limit_at(1.0, 0.0)
        # e_max quadrupled → v_cap should quadruple (still below geometric cap of 1.0)
        assert v1 == pytest.approx(min(4 * v0, 1.0), rel=1e-3)

    def test_update_e_max_clamps_below_floor(self, governor: ReferenceGovernor) -> None:
        # Two floors stack: min_e_max (governor, 5 mm) AND the artifact's
        # velocity_profile.min_speed (solver, 0.05 m/s here). For a Go2
        # plant with max(vx,wz)tau+L=0.65s, 0.005/0.65 ≈ 0.008 m/s — below the
        # solver's min_speed, so the output floors at min_speed.
        governor.set_path(straight_line(length=2.0))
        governor.update_e_max(0.0001)  # below min_e_max=0.005
        v = governor.speed_limit_at(1.0, 0.0)
        assert v == pytest.approx(0.05, rel=1e-6)  # solver min_speed floor
        # Sanity: not zero, not NaN.
        assert v > 0.0

    def test_update_e_max_rejects_non_positive(self, governor: ReferenceGovernor) -> None:
        governor.set_path(straight_line(length=2.0))
        before = governor.speed_limit_at(1.0, 0.0)
        governor.update_e_max(-1.0)
        governor.update_e_max(0.0)
        governor.update_e_max(float("nan"))
        after = governor.speed_limit_at(1.0, 0.0)
        assert before == pytest.approx(after, rel=1e-6)

    def test_cap_preserves_direction_when_under_limit(self, governor: ReferenceGovernor) -> None:
        governor.set_path(straight_line(length=2.0))
        # Commanded vx already below vlim → pass-through.
        vlim = governor.speed_limit_at(1.0, 0.0)
        vx, vy, wz = governor.cap(1.0, 0.0, vlim * 0.5, 0.0, 0.0)
        assert vx == pytest.approx(vlim * 0.5)
        assert vy == pytest.approx(0.0)
        assert wz == pytest.approx(0.0)

    def test_cap_scales_overcommanded_geometry_preserved(self, governor: ReferenceGovernor) -> None:
        governor.set_path(straight_line(length=2.0))
        vlim = governor.speed_limit_at(1.0, 0.0)
        # Command 2x vlim with some wz → output should be vlim with wz scaled
        # by the same factor so turn radius is preserved.
        out_vx, out_vy, out_wz = governor.cap(1.0, 0.0, 2 * vlim, 0.0, 1.0)
        assert out_vx == pytest.approx(vlim, rel=1e-3)
        assert out_wz == pytest.approx(0.5, rel=1e-3)  # scaled by vlim / (2*vlim) = 0.5

    def test_speed_at_index_lookup(self, governor: ReferenceGovernor) -> None:
        path = straight_line(length=2.0)
        governor.set_path(path)
        v_mid = governor.speed_at(len(path.poses) // 2)
        assert v_mid == pytest.approx(0.05 / 0.65, rel=1e-3)

    def test_speed_at_fallback_when_no_path(self, governor: ReferenceGovernor) -> None:
        assert governor.speed_at(5, fallback=0.42) == pytest.approx(0.42)
        with pytest.raises(RuntimeError):
            governor.speed_at(0)

    def test_invalid_path_ignored(self, governor: ReferenceGovernor) -> None:
        # Path with 0 or 1 poses is rejected; pre-state preserved.
        before = governor.speed_limit_at(1.0, 0.0)
        governor.set_path(Path(poses=[]))
        governor.set_path(Path(poses=[straight_line(length=0.1).poses[0]]))
        after = governor.speed_limit_at(1.0, 0.0)
        assert before == pytest.approx(after, rel=1e-6)


class TestClosedLoopAlpha:
    """alpha-feedback unit tests. The PI law is exercised by directly poking
    ``_update_alpha`` (pure function) and by driving ``speed_limit_at``
    with synthetic poses that land at known perpendicular distances from
    a straight reference path."""

    @pytest.fixture()
    def closed_loop_governor(
        self, artifact_path: FsPath
    ) -> Generator[ReferenceGovernor, None, None]:
        gov = ReferenceGovernor(
            plant_artifact_path=str(artifact_path),
            e_max_default=0.05,
            closed_loop=True,
            # Tighter gains than defaults so unit tests converge in
            # 10s-100s of synthetic ticks rather than seconds.
            kp_alpha=4.0,
            ki_alpha=0.5,
            alpha_min=0.2,
            max_integral=0.5,
            cte_ema_alpha=1.0,  # disable filter to make tests deterministic
            tick_dt_s=0.05,
        )
        try:
            yield gov
        finally:
            gov._close_module()

    def test_closed_loop_off_alpha_stays_one(self, governor: ReferenceGovernor) -> None:
        # Sanity: with closed_loop=False, speed_limit_at never touches alpha.
        governor.set_path(straight_line(length=2.0))
        # Query at a series of poses — including off-path ones that would
        # produce nonzero CTE if measured.
        v_a = governor.speed_limit_at(1.0, 0.0)
        v_b = governor.speed_limit_at(1.0, 0.2)  # 20cm off path
        v_c = governor.speed_limit_at(1.0, -0.5)
        # All identical (open-loop is pose-independent past nearest-idx).
        assert v_a == pytest.approx(v_b, rel=1e-6)
        assert v_b == pytest.approx(v_c, rel=1e-6)
        # And alpha never moved.
        assert governor._alpha == pytest.approx(1.0)

    def test_alpha_decreases_on_excess_cte(self, closed_loop_governor: ReferenceGovernor) -> None:
        gov = closed_loop_governor
        gov.set_path(straight_line(length=2.0))
        # Feed cte = 2 * e_max = 0.10m for 5 ticks.
        prev_alpha = 1.0
        for _ in range(5):
            alpha, integral, filtered = gov._update_alpha(cte_raw=0.10)
            with gov._state_lock:
                gov._alpha = alpha
                gov._alpha_integral = integral
                gov._cte_filtered = filtered
            assert alpha <= prev_alpha + 1e-9  # monotonic decrease
            prev_alpha = alpha
        assert gov._alpha < 1.0
        assert gov._alpha >= gov.config.alpha_min

    def test_alpha_recovers_when_cte_low(self, closed_loop_governor: ReferenceGovernor) -> None:
        gov = closed_loop_governor
        gov.set_path(straight_line(length=2.0))
        # Drive alpha down with high CTE.
        for _ in range(10):
            a, i, f = gov._update_alpha(cte_raw=0.15)
            with gov._state_lock:
                gov._alpha = a
                gov._alpha_integral = i
                gov._cte_filtered = f
        depressed = gov._alpha
        assert depressed < 0.9, f"setup failed: alpha={depressed}"
        # Now feed cte well below e_max → alpha should rise back.
        for _ in range(20):
            a, i, f = gov._update_alpha(cte_raw=0.01)
            with gov._state_lock:
                gov._alpha = a
                gov._alpha_integral = i
                gov._cte_filtered = f
        assert gov._alpha > depressed

    def test_anti_windup_at_saturation(self, closed_loop_governor: ReferenceGovernor) -> None:
        gov = closed_loop_governor
        gov.set_path(straight_line(length=2.0))
        # Feed a catastrophic CTE (10x e_max) for many ticks. alpha will peg
        # at alpha_min; the integral must NOT keep accumulating past the
        # max_integral clamp.
        for _ in range(500):
            a, i, f = gov._update_alpha(cte_raw=0.50)
            with gov._state_lock:
                gov._alpha = a
                gov._alpha_integral = i
                gov._cte_filtered = f
        assert gov._alpha == pytest.approx(gov.config.alpha_min)
        assert abs(gov._alpha_integral) <= gov.config.max_integral + 1e-9

    def test_alpha_resets_on_set_path(self, closed_loop_governor: ReferenceGovernor) -> None:
        gov = closed_loop_governor
        gov.set_path(straight_line(length=2.0))
        # Drive alpha down.
        for _ in range(20):
            a, i, f = gov._update_alpha(cte_raw=0.20)
            with gov._state_lock:
                gov._alpha = a
                gov._alpha_integral = i
                gov._cte_filtered = f
        assert gov._alpha < 1.0
        # New path → fresh alpha, integral, filter.
        gov.set_path(circle(radius=1.0, n_points=120))
        assert gov._alpha == pytest.approx(1.0)
        assert gov._alpha_integral == pytest.approx(0.0)
        assert gov._cte_filtered == pytest.approx(0.0)

    def test_speed_limit_at_uses_alpha(self, closed_loop_governor: ReferenceGovernor) -> None:
        gov = closed_loop_governor
        gov.set_path(straight_line(length=2.0))
        # On-path query (alpha==1.0 at start; alpha update pushes back to 1.0).
        v_on_path = gov.speed_limit_at(1.0, 0.0)
        # Set both alpha=0.5 AND a consistent integral state so the alpha update
        # at error=0 keeps alpha at 0.5 (the integral cancels the +Δalpha the P
        # term would otherwise inject toward 1.0). With kp=4, ki=0.5,
        # error=0 ⟹ alpha_raw = 1.0 - 0 - 0.5·I = 0.5 ⇒ I = 1.0. Then
        # max_integral=0.5 clamps to 0.5, giving alpha_raw = 0.75 → alpha=0.75.
        # Just assert that v scales with alpha after one tick (not exactly
        # 0.5x, but strictly less than the on-path alpha=1.0 case).
        with gov._state_lock:
            gov._alpha = 0.5
            gov._alpha_integral = 1.0  # gets clamped to max_integral=0.5
        v_with_alpha_low = gov.speed_limit_at(1.0, 0.0)
        assert v_with_alpha_low < v_on_path

    def test_ema_filter_dampens_spikes(self, artifact_path: FsPath) -> None:
        # Use the public ctor with EMA enabled (cte_ema_alpha < 1).
        gov = ReferenceGovernor(
            plant_artifact_path=str(artifact_path),
            e_max_default=0.05,
            closed_loop=True,
            cte_ema_alpha=0.2,  # heavy smoothing
        )
        try:
            gov.set_path(straight_line(length=2.0))
            # Single CTE spike at 0.3m followed by zeros.
            inputs = [0.30] + [0.0] * 10
            filtered_history = []
            for cte_raw in inputs:
                a, i, f = gov._update_alpha(cte_raw=cte_raw)
                with gov._state_lock:
                    gov._alpha = a
                    gov._alpha_integral = i
                    gov._cte_filtered = f
                filtered_history.append(f)
            # First filtered value < raw spike (low-pass).
            assert filtered_history[0] < 0.30
            # Filter decays back toward zero (subsequent zeros).
            assert filtered_history[-1] < filtered_history[0]
        finally:
            gov._close_module()


class TestConcurrentUpdates:
    """Atomic-snapshot test: hammer update_e_max from one thread while
    cap() reads from another. Should never crash or return torn state.
    """

    def test_concurrent_update_and_read(self, governor: ReferenceGovernor) -> None:
        governor.set_path(circle(radius=1.0, n_points=200))
        stop = threading.Event()
        errors: list[BaseException] = []

        def writer() -> None:
            try:
                e_values = [0.02, 0.05, 0.10, 0.20, 0.50]
                i = 0
                while not stop.is_set():
                    governor.update_e_max(e_values[i % len(e_values)])
                    i += 1
            except BaseException as e:
                errors.append(e)

        def reader() -> None:
            try:
                while not stop.is_set():
                    out = governor.cap(0.5, 0.5, 1.0, 0.0, 0.5)
                    # All outputs must be finite and bounded.
                    assert all(math.isfinite(v) for v in out)
                    assert abs(out[0]) <= 1.0 + 1e-6
            except BaseException as e:
                errors.append(e)

        threads = [threading.Thread(target=writer), threading.Thread(target=reader)]
        for t in threads:
            t.start()
        time.sleep(0.5)
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
        assert not errors, f"Concurrent execution raised: {errors}"
