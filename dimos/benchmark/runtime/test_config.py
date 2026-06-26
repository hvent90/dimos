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

"""Tests for benchmark runtime config resolution."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
sys.path.insert(0, str(PROTOCOL_SRC))

from dimos_runtime_protocol import (
    MotorDescription,
    ProtocolVersion,
    RobotMotorSurface,
    RuntimeDescription,
)

from dimos.benchmark.runtime.config import (
    BenchmarkEpisodeConfig,
    resolve_runtime_plan,
)


def test_resolve_runtime_plan_rejects_incompatible_protocol() -> None:
    description = _description(protocol=ProtocolVersion(version="1.0", min_compatible="1.0"))

    with pytest.raises(ValueError, match="incompatible sidecar protocol"):
        resolve_runtime_plan(BenchmarkEpisodeConfig(), description)


def test_resolve_runtime_plan_rejects_robot_profile_mismatch() -> None:
    description = _description(robot_id="otherbot")

    with pytest.raises(ValueError, match="sidecar did not report robot surface"):
        resolve_runtime_plan(BenchmarkEpisodeConfig(), description)


def _description(
    *,
    robot_id: str = "fakebot",
    protocol: ProtocolVersion | None = None,
) -> RuntimeDescription:
    return RuntimeDescription(
        runtime_id="fake-runtime",
        backend="fake",
        protocol=protocol or ProtocolVersion(),
        robot_surfaces=[
            RobotMotorSurface(
                robot_id=robot_id,
                motors=[
                    MotorDescription(name=f"{robot_id}/joint{i + 1}", index=i) for i in range(3)
                ],
            )
        ],
        control_step_hz=100,
    )
