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

"""Adapter: pimsim ``/odom`` (PoseStamped) -> nav stack ``/odometry`` (Odometry).

The browser publishes the integrated base pose as ``geometry_msgs/PoseStamped``
on ``/odom``. The nav stack (TerrainAnalysis, planners, PGO) consumes
``nav_msgs/Odometry`` on ``/odometry``. This module bridges the two so a
``create_nav_stack`` blueprint can autoconnect onto the babylon sim.
"""

from __future__ import annotations

from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry

DEFAULT_WORLD_FRAME = "map"
DEFAULT_CHILD_FRAME = "base_link"


class PoseStampedToOdometry(Module):
    """Republish a ``PoseStamped`` stream as ``nav_msgs/Odometry``."""

    pose: In[PoseStamped]
    odometry: Out[Odometry]

    def __init__(
        self,
        *,
        world_frame: str = DEFAULT_WORLD_FRAME,
        child_frame: str = DEFAULT_CHILD_FRAME,
    ) -> None:
        super().__init__()
        self._world_frame = world_frame
        self._child_frame = child_frame

    async def handle_pose(self, value: PoseStamped) -> None:
        pose = Pose()
        pose.position = value.position
        pose.orientation = value.orientation
        self.odometry.publish(
            Odometry(
                pose=pose,
                frame_id=value.frame_id or self._world_frame,
                child_frame_id=self._child_frame,
                ts=value.ts,
            )
        )
