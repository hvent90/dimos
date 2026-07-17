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

"""Piper-to-LeRobot conversion configuration.

The Piper collection records RGB frames on ``color_image`` and the measured
arm-plus-gripper vector on ``coordinator_joint_state.position``. This setup
resamples both streams on the RGB timeline at 30 Hz and shifts the action by
one frame, producing the current joint state as observation and the next
absolute joint state as action. SQLite retains native timestamps; the
nearest-neighbor tolerance is configurable for camera/teleop timing.
"""

from __future__ import annotations

from pathlib import Path

from dimos.learning.dataprep.core import (
    DataPrepConfig,
    OutputConfig,
    StreamField,
    SyncConfig,
)

PIPER_RATE_HZ = 30.0
PIPER_SYNC_TOLERANCE_MS = 50.0


def piper_lerobot_config(
    source: str | Path,
    output: str | Path,
    *,
    tolerance_ms: float = PIPER_SYNC_TOLERANCE_MS,
) -> DataPrepConfig:
    """Return the standard Piper recording-to-LeRobot conversion setup."""
    joint_state = StreamField(stream="coordinator_joint_state", field="position")
    return DataPrepConfig(
        source=str(source),
        observation={
            "image": StreamField(stream="color_image"),
            "joints": joint_state,
        },
        action={"action": joint_state},
        sync=SyncConfig(
            anchor="image",
            rate_hz=PIPER_RATE_HZ,
            tolerance_ms=tolerance_ms,
            action_shift=1,
        ),
        output=OutputConfig(format="lerobot", path=Path(output)),
    )
