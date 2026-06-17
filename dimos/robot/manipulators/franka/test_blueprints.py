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

"""Tests for Franka Panda blueprints."""

from __future__ import annotations

from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.franka.blueprints import panda_coordinator


def test_panda_coordinator_is_registered() -> None:
    assert (
        all_blueprints["panda-coordinator"]
        == "dimos.robot.manipulators.franka.blueprints:panda_coordinator"
    )


def test_panda_coordinator_accepts_vamp_cli_override_shape() -> None:
    config = panda_coordinator.config()(
        manipulationmodule={
            "world": {
                "backend": "vamp",
                "artifact": {"mode": "official", "robot": "panda"},
            },
            "planner": {
                "backend": "vamp",
                "algorithm": "rrtc",
                "simplify": "true",
                "validate_path": "true",
            },
        }
    )

    assert config.manipulationmodule is not None
    assert config.manipulationmodule.world.backend == "vamp"
    assert config.manipulationmodule.world.artifact.robot == "panda"
    assert config.manipulationmodule.planner.backend == "vamp"
    assert config.manipulationmodule.planner.algorithm == "rrtc"
    assert config.manipulationmodule.planner.simplify is True
    assert config.manipulationmodule.planner.validate_path is True
