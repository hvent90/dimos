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

"""Franka Panda robot configurations."""

from __future__ import annotations

from typing import Any

from dimos.robot.config import RobotConfig
from dimos.utils.data import LfsPath

# LFS-backed: data/.lfs/franka_description.tar.gz extracts to data/franka_description/.
_FRANKA_DESCRIPTION_PKG = LfsPath("franka_description")

# Keep URDF/SRDF paths explicit so VAMP tests/benchmarks can validate that the
# DimOS robot model matches the official VAMP Panda artifact joint order.
FRANKA_PANDA_MODEL = _FRANKA_DESCRIPTION_PKG / "urdf/panda.urdf.xacro"
FRANKA_PANDA_FK_MODEL = _FRANKA_DESCRIPTION_PKG / "urdf/panda.urdf"
FRANKA_PANDA_SRDF = _FRANKA_DESCRIPTION_PKG / "srdf/panda.srdf"

FRANKA_PANDA_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
FRANKA_PANDA_HOME_JOINTS = [0.0, -0.7853981634, 0.0, -2.35619449, 0.0, 1.5707963268, 0.7853981634]
FRANKA_PANDA_JOINT_LIMITS_LOWER = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
FRANKA_PANDA_JOINT_LIMITS_UPPER = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]


def franka_panda(
    name: str = "panda",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    **overrides: Any,
) -> RobotConfig:
    """Franka Panda config for mock-control planning tests and benchmarks."""
    defaults: dict[str, Any] = {
        "name": name,
        "model_path": FRANKA_PANDA_MODEL,
        "end_effector_link": "panda_hand",
        "adapter_type": adapter_type,
        "address": address,
        "joint_names": FRANKA_PANDA_JOINT_NAMES,
        "base_link": "panda_link0",
        "home_joints": FRANKA_PANDA_HOME_JOINTS,
        "joint_limits_lower": FRANKA_PANDA_JOINT_LIMITS_LOWER,
        "joint_limits_upper": FRANKA_PANDA_JOINT_LIMITS_UPPER,
        "package_paths": {
            "franka_description": _FRANKA_DESCRIPTION_PKG,
            "moveit_resources_panda_description": _FRANKA_DESCRIPTION_PKG,
        },
        "auto_convert_meshes": True,
        "max_velocity": 1.0,
        "max_acceleration": 2.0,
        "adapter_kwargs": {"srdf_path": FRANKA_PANDA_SRDF},
    }
    if "adapter_kwargs" in overrides:
        defaults["adapter_kwargs"] = {
            **defaults["adapter_kwargs"],
            **overrides.pop("adapter_kwargs"),
        }
    defaults.update(overrides)
    return RobotConfig(**defaults)


__all__ = [
    "FRANKA_PANDA_FK_MODEL",
    "FRANKA_PANDA_HOME_JOINTS",
    "FRANKA_PANDA_JOINT_LIMITS_LOWER",
    "FRANKA_PANDA_JOINT_LIMITS_UPPER",
    "FRANKA_PANDA_JOINT_NAMES",
    "FRANKA_PANDA_MODEL",
    "FRANKA_PANDA_SRDF",
    "franka_panda",
]
