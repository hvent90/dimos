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

"""Fast, hardware-free tests for the sim ground-truth harness: the recording it
emits round-trips through the memory2 store in the recorder's format, the odom
sampling rate is honored, generation is deterministic per seed, and the injected
ground truth is reported faithfully."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from dimos.control.components import make_twist_base_joints
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.utils.benchmarking.plant import FopdtChannelParams, TwistBasePlantParams
from dimos.utils.characterization.sim_ground_truth import (
    MeasurementModel,
    StepSegment,
    effective_deadtime_s,
    go2_validation_grid,
    synthesize_recording,
)

_ODOM_RATE = 18.0


def _plant(tau: float = 0.2, dead_time: float = 0.15) -> TwistBasePlantParams:
    ch = FopdtChannelParams(K=0.9, tau=tau, L=dead_time)
    return TwistBasePlantParams(vx=ch, vy=ch, wz=FopdtChannelParams(K=2.4, tau=tau, L=dead_time))


def _short_segments() -> list[StepSegment]:
    # ~4.5 s total: keeps the test fast while exercising all three axes.
    return [
        StepSegment("vx", 0.4, hold_s=1.0, settle_s=0.5),
        StepSegment("wz", 0.6, hold_s=1.0, settle_s=0.5),
    ]


def _read(db_path: Path, name: str, payload_type: type) -> list:
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        observations = list(store.stream(name, payload_type))
        for obs in observations:
            _ = obs.data  # materialize the lazy blob while the connection is open
        return observations
    finally:
        store.stop()


def test_round_trips_in_recorder_format(tmp_path: Path) -> None:
    rec = synthesize_recording(
        _plant(), db_path=tmp_path / "sim.db", segments=_short_segments(), odom_rate_hz=_ODOM_RATE
    )
    assert rec.db_path.exists()

    cmd = _read(rec.db_path, "cmd_vel", Twist)
    odom = _read(rec.db_path, "odom", PoseStamped)
    joint = _read(rec.db_path, "joint_state", JointState)
    gate = _read(rec.db_path, "gate", Int8)

    assert len(cmd) > 0 and len(odom) > 0
    assert len(joint) == len(odom)  # both carry the same decimated pose
    assert len(gate) == len(_short_segments())  # one advance marker per segment
    assert isinstance(odom[0].data, PoseStamped)
    # joint_state must match what the old fitter reads: go2/{vx,vy,wz}, pos=[x,y,yaw].
    assert joint[0].data.name == make_twist_base_joints("go2")
    assert len(joint[0].data.position) == 3
    # odom and joint_state agree on the pose at matching samples.
    assert joint[0].data.position[0] == pytest.approx(odom[0].data.x)
    assert joint[0].data.position[2] == pytest.approx(odom[0].data.yaw)


def test_odom_sampled_at_requested_rate(tmp_path: Path) -> None:
    rec = synthesize_recording(
        _plant(), db_path=tmp_path / "sim.db", segments=_short_segments(), odom_rate_hz=_ODOM_RATE
    )
    ts = np.array([obs.ts for obs in _read(rec.db_path, "odom", PoseStamped)])
    median_dt = float(np.median(np.diff(ts)))
    assert median_dt == pytest.approx(1.0 / _ODOM_RATE, rel=0.02)


def test_reports_injected_ground_truth(tmp_path: Path) -> None:
    plant = _plant(tau=0.3, dead_time=0.1)
    rec = synthesize_recording(
        plant, db_path=tmp_path / "sim.db", segments=_short_segments(), robot_id="go2_u07", seed=5
    )
    sidecar = json.loads(rec.sidecar_path.read_text())

    assert sidecar["robot_id"] == "go2_u07"
    assert sidecar["seed"] == 5
    assert sidecar["plant"]["vx"]["tau"] == pytest.approx(0.3)
    assert sidecar["plant"]["wz"]["K"] == pytest.approx(2.4)
    # Reported L is the realized (discretized) dead time, not the nominal request.
    assert sidecar["effective_l_s"]["vx"] == pytest.approx(
        effective_deadtime_s(plant.vx, sidecar["sim_dt_s"])
    )


def test_same_seed_is_reproducible(tmp_path: Path) -> None:
    def gen(name: str, seed: int) -> np.ndarray:
        rec = synthesize_recording(
            _plant(), db_path=tmp_path / name, segments=_short_segments(), seed=seed
        )
        return np.array([obs.data.x for obs in _read(rec.db_path, "odom", PoseStamped)])

    assert np.array_equal(gen("a.db", 0), gen("b.db", 0))
    assert not np.array_equal(gen("c.db", 0), gen("d.db", 1))


def test_drift_dominates_a_zero_command_recording(tmp_path: Path) -> None:
    # No commanded motion + no noise: the only displacement is the injected drift,
    # so net travel must equal drift_total_m (the nuisance Phase 2 fits out).
    drift_total = 0.07
    measurement = MeasurementModel(
        pos_noise_std_m=0.0, yaw_noise_std_rad=0.0, drift_total_m=drift_total
    )
    rec = synthesize_recording(
        _plant(),
        db_path=tmp_path / "drift.db",
        segments=[StepSegment("vx", 0.0, hold_s=3.0, settle_s=0.0)],
        measurement=measurement,
    )
    odom = _read(rec.db_path, "odom", PoseStamped)
    net = float(np.hypot(odom[-1].data.x - odom[0].data.x, odom[-1].data.y - odom[0].data.y))
    assert net == pytest.approx(drift_total, rel=0.05)


def test_validation_grid_brackets_both_regimes() -> None:
    grid = go2_validation_grid()
    assert len(grid) == 20  # 5 taus x 4 Ls
    taus = {plant.vx.tau for _, plant in grid}
    deadtimes = {plant.vx.L for _, plant in grid}
    assert min(taus) <= 0.15 and max(taus) >= 0.6  # fast regime .. vendored regime
    assert min(deadtimes) <= 0.05 and max(deadtimes) >= 0.20
