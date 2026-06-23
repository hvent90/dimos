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

"""Honesty gate for the ESO sim A/B (fixed-time window, single-variable).

History: an earlier version of these tests asserted large curve "wins". An
adversarial review showed those were a metric confound (the ESO arm orbited a
closed path longer and padded its cross-track RMS). With the fixed-time-window
metric the ESO is roughly NEUTRAL for path tracking, so these tests now assert
the claims that actually hold: it is a no-op on the clean FlowBase, it never
catastrophically regresses the hard Go2, and the A/B is deterministic. The
velocity-level disturbance cancellation (the thing the ESO genuinely does) is
asserted in test_eso.py, not here.
"""

from __future__ import annotations

import pytest

from dimos.utils.benchmarking.eso_sim_ab import _plants, _paths, _run_seed, run_arm


def _ab(plant: str, path: str, speed: float, dial: float = 1.0):
    tracking, params, pcfg = _plants()[plant]
    pobj = _paths()[path]
    seed = _run_seed(7, plant, path, speed)
    base = run_arm(tracking, params, pcfg, pobj, speed, eso=False, dial=dial, tick_hz=16.0, seed=seed)
    eso = run_arm(tracking, params, pcfg, pobj, speed, eso=True, dial=dial, tick_hz=16.0, seed=seed)
    return base, eso


@pytest.mark.parametrize("path,speed", [("straight", 1.0), ("circle_r1.0", 1.0), ("smooth_corner", 1.0)])
def test_eso_is_noop_on_flowbase(path: str, speed: float) -> None:
    """The clean FlowBase plant is the control: the ESO must barely move the
    needle (no regression on an already-good loop)."""
    base, eso = _ab("flowbase", path, speed)
    assert eso.cte_rms == pytest.approx(base.cte_rms, abs=0.01)


@pytest.mark.parametrize("path,speed", [("straight", 1.0), ("circle_r1.0", 1.0), ("smooth_corner", 1.0)])
def test_eso_does_not_blow_up_go2(path: str, speed: float) -> None:
    """On the hard Go2 the ESO must not catastrophically regress or run away
    (guards the NaN/clamp-rail failure mode and gross instability)."""
    base, eso = _ab("go2-hard", path, speed)
    assert eso.cte_rms < 1.6 * base.cte_rms + 0.03
    assert eso.cross_traj_rms < 1.6 * base.cross_traj_rms + 0.03


def test_ab_is_deterministic() -> None:
    """Same seed => identical result across runs (no Python-hash randomness)."""
    a = _ab("go2-hard", "circle_r1.0", 0.7)
    b = _ab("go2-hard", "circle_r1.0", 0.7)
    assert a[0].cte_rms == pytest.approx(b[0].cte_rms, abs=1e-9)
    assert a[1].cte_rms == pytest.approx(b[1].cte_rms, abs=1e-9)
