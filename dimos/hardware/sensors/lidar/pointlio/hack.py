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

"""Fake a normally-mounted mid-360 from the physically-tilted one.

The sensor is bolted to the robot at an angle (``rotated_urdf``), but the rest of
the stack is wired as if it sat in the nominal mount (``normal_urdf``). This
module rewrites both the cloud and the odometry so nothing downstream can tell
the sensor is tilted.

The correction is a single rigid transform ``C`` derived from the two URDFs:
``C = inv(normal_base_to_sensor) @ rotated_base_to_sensor`` evaluated at the
shared ``sensor_frame``.

* cloud: each point is moved ``p' = C @ p`` (rotated sensor frame -> normal one).
* odometry: the body pose is conjugated ``T' = C @ T @ inv(C)`` and the twist
  rotated by ``C``'s rotation. That is exactly the odometry a normally-mounted
  sensor would have produced, so a forward walk stays forward instead of drifting
  sideways the way a bare body-frame relabel would.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.urdf_loader import UrdfLoader
from dimos.spec import perception


class PointLioHackConfig(ModuleConfig):
    rotated_urdf: Path
    normal_urdf: Path
    sensor_frame: str = "mid360_link"


def _transform_to_matrix(transform: Transform) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = transform.rotation.to_rotation_matrix()
    matrix[:3, 3] = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    return matrix


def _base_to_frame_matrix(loader: UrdfLoader, leaf_frame: str) -> np.ndarray:
    """Compose the fixed-joint chain from the model root down to ``leaf_frame``."""
    static_transforms = loader.static_transforms
    chain: list[Transform] = []
    frame = leaf_frame
    while frame in static_transforms:
        transform = static_transforms[frame]
        chain.append(transform)
        frame = transform.frame_id
    matrix = np.eye(4)
    for transform in reversed(chain):
        matrix = matrix @ _transform_to_matrix(transform)
    return matrix


def _vector_to_numpy(vector: Vector3) -> np.ndarray:
    return np.array([vector.x, vector.y, vector.z])


class PointLioHack(Module, perception.Lidar, perception.Odometry):
    config: PointLioHackConfig

    rotated_lidar: In[PointCloud2]
    lidar: Out[PointCloud2]
    rotated_odometry: In[Odometry]
    odometry: Out[Odometry]

    async def main(self) -> AsyncIterator[None]:
        rotated = _base_to_frame_matrix(
            UrdfLoader(name="rotated", model_path=self.config.rotated_urdf),
            self.config.sensor_frame,
        )
        normal = _base_to_frame_matrix(
            UrdfLoader(name="normal", model_path=self.config.normal_urdf),
            self.config.sensor_frame,
        )
        self._correction = np.linalg.inv(normal) @ rotated
        self._correction_inv = np.linalg.inv(self._correction)
        self._rotation = self._correction[:3, :3].astype(np.float32)
        self._translation = self._correction[:3, 3].astype(np.float32)
        yield

    async def handle_rotated_lidar(self, value: PointCloud2) -> None:
        points = value.points_f32() @ self._rotation.T + self._translation
        self.lidar.publish(
            PointCloud2.from_numpy(
                points=points,
                frame_id=value.frame_id,
                timestamp=value.ts,
                intensities=value.intensities_f32(),
            )
        )

    async def handle_rotated_odometry(self, value: Odometry) -> None:
        pose = np.eye(4)
        pose[:3, :3] = value.orientation.to_rotation_matrix()
        pose[:3, 3] = _vector_to_numpy(value.position)
        faked = self._correction @ pose @ self._correction_inv

        rotation = self._correction[:3, :3]
        linear = rotation @ _vector_to_numpy(value.linear_velocity)
        angular = rotation @ _vector_to_numpy(value.angular_velocity)

        self.odometry.publish(
            Odometry(
                ts=value.ts,
                frame_id=value.frame_id,
                child_frame_id=value.child_frame_id,
                pose=Pose(
                    Vector3(*(float(component) for component in faked[:3, 3])),
                    Quaternion.from_rotation_matrix(faked[:3, :3]),
                ),
                twist=Twist(
                    Vector3(*(float(component) for component in linear)),
                    Vector3(*(float(component) for component in angular)),
                ),
            )
        )


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    PointLioHack()
