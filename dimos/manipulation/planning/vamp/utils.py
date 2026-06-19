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

"""Utilities for adapting VAMP binding values to DimOS models."""

from __future__ import annotations

from typing import TypeGuard

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.planning.vamp.protocols import VampPathProtocol
from dimos.msgs.sensor_msgs.JointState import JointState


def path_to_joint_states(path_source: object, joint_names: list[str]) -> list[JointState]:
    """Convert a VAMP path object or numeric waypoint array into joint states."""
    path_array = path_to_array(path_source)
    return [JointState(name=joint_names, position=row.astype(float).tolist()) for row in path_array]


def path_to_array(path_source: object) -> NDArray[np.float64]:
    """Convert a VAMP path object or sequence into a float waypoint array."""
    if _has_numpy(path_source):
        return np.asarray(path_source.numpy(), dtype=np.float64)
    return np.asarray(path_source, dtype=np.float64)


def _has_numpy(path_source: object) -> TypeGuard[VampPathProtocol]:
    return hasattr(path_source, "numpy")
