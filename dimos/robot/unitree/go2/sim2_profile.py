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

"""Go2's backend-neutral sim2 control profile."""

from dimos.control.components import make_twist_base_joints
from dimos.sim2.spec import (
    ControlInterface,
    RaycastLidarSpec,
    SensorImplementation,
    SimRobotSpec,
    SpawnPose,
)


def go2_kinematic_sim2_robot(
    robot_id: str = "go2",
    *,
    portable_lidar: bool = False,
) -> SimRobotSpec:
    sensors = (
        (
            RaycastLidarSpec(
                sensor_id="lidar",
                frame_id="world",
                implementation=SensorImplementation.PORTABLE,
                width=720,
                height=16,
                rate_hz=10.0,
                min_range=0.1,
                max_range=10.0,
                voxel_size=0.03,
            ),
        )
        if portable_lidar
        else ()
    )
    return SimRobotSpec(
        robot_id=robot_id,
        control_interface=ControlInterface.TWIST_BASE,
        dof=3,
        joint_names=tuple(make_twist_base_joints(robot_id)),
        spawn=SpawnPose(position=(0.0, 0.0, 0.3)),
        sensors=sensors,
    )
