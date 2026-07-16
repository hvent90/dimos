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

import json

import pytest

from dimos.sim2.ipc.abi import make_channel_descriptor
from dimos.sim2.ipc.registry import SimRegistry, shared_memory_name
from dimos.sim2.spec import ControlInterface


def _descriptor(generation: str):
    return make_channel_descriptor(
        sim_id="main",
        robot_id="arm",
        generation=generation,
        shm_name=shared_memory_name("run", "main", "arm", generation),
        control_interface=ControlInterface.MANIPULATOR,
        dof=7,
        physics_dt=0.002,
        control_decimation=10,
    )


def test_registry_round_trips_descriptor_and_removes_owner_generation(tmp_path) -> None:
    registry = SimRegistry(run_id="run", root=tmp_path)
    descriptor = _descriptor("generation-1")

    path = registry.publish("main", "generation-1", {"arm": descriptor})
    resolved = registry.resolve("main", "arm")
    registry.remove("main", "other-generation")

    assert resolved == descriptor
    assert path.exists()

    registry.remove("main", "generation-1")
    assert not path.exists()


def test_registry_rejects_stale_channel_generation(tmp_path) -> None:
    registry = SimRegistry(run_id="run", root=tmp_path)
    descriptor = _descriptor("old")
    path = registry.publish("main", "new", {"arm": descriptor})

    with pytest.raises(ValueError, match="stale channel generation"):
        registry.resolve("main", "arm")

    raw = json.loads(path.read_text())
    assert raw["generation"] == "new"


def test_registry_reports_missing_robot(tmp_path) -> None:
    registry = SimRegistry(run_id="run", root=tmp_path)
    descriptor = _descriptor("generation")
    registry.publish("main", "generation", {"arm": descriptor})

    with pytest.raises(KeyError, match="robot 'base'"):
        registry.resolve("main", "base")
