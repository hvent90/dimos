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

"""Speed-scheduled gain inversion: the per-amplitude table is sanitized into
a bounded K(|v|), and on a nonlinear plant the scheduled inversion hits the
target velocity where a single constant K over/under-shoots."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.control.tasks.trajectory_tracking_task.gain_schedule import (
    GainSchedule,
    ScheduledGainCompensator,
)


def _dynamics(channel_rows: dict[str, list[tuple[float, float, float]]]) -> dict:
    """Build a dynamics_by_amplitude dict from (amp, K, tau) tuples."""
    return {
        ax: [{"amp": a, "K": k, "tau": t} for a, k, t in rows] for ax, rows in channel_rows.items()
    }


def test_from_dynamics_drops_degenerate_and_floors_gain() -> None:
    # vy's amp=0.2 point collapsed at the motion floor (K~0) — must be dropped,
    # and the interpolated gain never drops below the floor (so 1/K is bounded).
    dyn = _dynamics(
        {
            "vx": [(0.5, 1.1, 0.35), (1.0, 1.0, 0.5)],
            "vy": [(0.2, 0.01, 0.001), (0.5, 0.8, 0.5), (1.0, 0.9, 0.6)],
            "wz": [(0.5, 2.5, 0.36), (1.0, 2.7, 0.38)],
        }
    )
    sched = GainSchedule.from_dynamics(dyn)
    assert sched is not None
    assert 0.2 not in sched.vy.amp  # degenerate floor point dropped
    assert sched.vy.k_at(0.0) >= 0.2  # gain floored, 1/K bounded
    assert sched.wz.k_at(0.5) == pytest.approx(2.5)


def test_from_dynamics_none_when_missing() -> None:
    assert GainSchedule.from_dynamics(None) is None
    assert GainSchedule.from_dynamics({}) is None


def test_scheduled_inversion_hits_target_where_constant_overshoots() -> None:
    """Nonlinear plant: K is high at low command, low at high command (like
    the Go2). A constant K picked at the high-amplitude end over-commands at
    low speed; the schedule inverts the local gain and lands on target."""
    # True plant gain K_plant(amp): 2.0 at amp 0.1 -> 1.0 at amp 1.0.
    amps = np.array([0.1, 0.5, 1.0])
    gains = np.array([2.0, 1.4, 1.0])

    def k_plant(cmd: float) -> float:
        return float(np.interp(abs(cmd), amps, gains))

    dyn = _dynamics(
        {
            ax: [(float(a), float(g), 0.3) for a, g in zip(amps, gains, strict=True)]
            for ax in ("vx", "vy", "wz")
        }
    )
    sched = GainSchedule.from_dynamics(dyn)
    assert sched is not None
    scheduled = ScheduledGainCompensator(sched, output_limit=(5.0, 5.0, 5.0))

    desired = 0.3  # target body velocity
    k_const = 1.0  # canonical fit landed at the high-amplitude end

    # open-loop feedforward: achieved = K_plant(cmd) * cmd
    cmd_sched = scheduled.compute(desired, 0.0, 0.0)[0]
    achieved_sched = k_plant(cmd_sched) * cmd_sched

    cmd_const = desired / k_const
    achieved_const = k_plant(cmd_const) * cmd_const

    # Scheduled lands within ~15% of target; constant overshoots badly.
    assert abs(achieved_sched - desired) / desired < 0.15
    assert achieved_const > 1.5 * desired
    assert abs(achieved_sched - desired) < abs(achieved_const - desired)


def test_invert_solves_fixed_point_on_nonmonotonic_gain() -> None:
    """The Go2 wz gain rises then falls, so K at the desired speed differs from
    K at the resulting command. The fixed-point inversion lands on target
    across the range where a one-shot lookup would not."""
    amps = np.array([0.2, 0.5, 1.0, 2.0])
    gains = np.array([1.7, 2.5, 2.7, 1.5])  # rise then fall

    def k_plant(cmd: float) -> float:
        return float(np.interp(abs(cmd), amps, gains))

    dyn = _dynamics(
        {
            ax: [(float(a), float(g), 0.3) for a, g in zip(amps, gains, strict=True)]
            for ax in ("vx", "vy", "wz")
        }
    )
    sched = GainSchedule.from_dynamics(dyn)
    assert sched is not None
    for desired in (0.3, 0.6, 1.0, 1.5):
        cmd = sched.wz.invert(desired)
        achieved = k_plant(cmd) * cmd
        assert abs(achieved - desired) / desired < 0.05, f"desired {desired} -> {achieved}"
