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

# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").

from __future__ import annotations

import math

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry


class OdomBodyFrameConfig(ModuleConfig):
    # base_link from sensor mount rotation, xyzw.
    mount_rotation: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    # base_link position in the sensor frame (xyz). Shifts the reported pose from the
    # sensor mount (e.g. lidar on the head) back to the robot body center, so the local
    # planner's footprint is centered on the body instead of the sensor.
    mount_translation: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    body_frame_id: str = "base_link"
    # Also broadcast a gravity-leveled (yaw-only) tf at the body center under this
    # name. Anchor the footprint viz box to it so the box is always horizontal --
    # independent of the robot's live body pitch/roll -- and sits exactly where the
    # local planner's footprint is (the body center), not out at the head sensor.
    footprint_frame_id: str = "base_footprint"


class OdomBodyFrame(Module):
    """Re-express tilted-sensor LIO odometry in the level robot body frame.

    Composes out the fixed mount rotation from the orientation, and offsets the
    position from the sensor mount to the body center by ``mount_translation``
    (rotated into the world frame by the sensor orientation). Twist passes through.
    """

    config: OdomBodyFrameConfig

    odometry: In[Odometry]
    body_odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        super().start()
        self._mount_inv = Quaternion(*self.config.mount_rotation).inverse()
        self._mount_t = Vector3(*self.config.mount_translation)
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        leveled = msg.orientation * self._mount_inv
        off = msg.orientation.rotate_vector(self._mount_t)
        body_pos = Vector3(
            msg.position.x + off.x,
            msg.position.y + off.y,
            msg.position.z + off.z,
        )
        self.body_odometry.publish(
            Odometry(
                ts=msg.ts,
                frame_id=msg.frame_id,
                child_frame_id=self.config.body_frame_id,
                pose=Pose(body_pos, leveled),
                twist=msg.twist,
            )
        )
        # Gravity-leveled (yaw-only) frame at the body center for the footprint
        # viz box. Yaw comes from the leveled forward vector, so the frame -- and
        # thus the box -- stays horizontal no matter how the robot body pitches.
        fwd = leveled.rotate_vector(Vector3(1.0, 0.0, 0.0))
        yaw = math.atan2(fwd.y, fwd.x)
        self.tf.publish(
            Transform(
                translation=body_pos,
                rotation=Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
                frame_id=msg.frame_id,
                child_frame_id=self.config.footprint_frame_id,
                ts=msg.ts,
            )
        )
