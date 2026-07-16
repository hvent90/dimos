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

"""G1 model binding for the GR00T whole-body sim2 stack."""

from pathlib import Path

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_joints
from dimos.robot.unitree.g1.config import G1
from dimos.sim2.spec import (
    ControlInterface,
    RaycastLidarSpec,
    SimRobotSpec,
    SpawnPose,
)
from dimos.simulation.engines.robot_sim_binding import (
    RobotSimSpec,
    mjcf_joint_names_from_hardware,
)
from dimos.utils.data import LfsPath

G1_GROOT_SIM2_LEGACY_MJCF = LfsPath("mujoco_sim/g1_gear_wbc.xml")
G1_GROOT_SIM2_ROBOT_MJCF = Path(__file__).resolve().parent / "assets" / "g1_29dof.xml"
G1_GROOT_SIM2_MESHDIR = LfsPath("g1_urdf/meshes")


def g1_groot_sim2_robot(robot_id: str = "g1") -> SimRobotSpec:
    joints = tuple(g1_joints)
    return SimRobotSpec(
        robot_id=robot_id,
        control_interface=ControlInterface.WHOLE_BODY,
        dof=len(joints),
        joint_names=joints,
        model_path=G1_GROOT_SIM2_ROBOT_MJCF,
        spawn=SpawnPose(position=(0.0, 0.0, 0.793)),
        sensors=(
            RaycastLidarSpec(
                sensor_id="lidar",
                frame_id="world",
                camera_names=(
                    "lidar_front_camera",
                    "lidar_left_camera",
                    "lidar_right_camera",
                ),
                width=64,
                height=32,
                rate_hz=1.0,
                min_range=0.2,
                max_range=3.0,
                max_height=1.2,
                geom_groups=(2, 3),
                robot_exclusion_radius=G1.width_clearance or 0.0,
                voxel_size=0.005,
            ),
        ),
        backend_options={
            "mujoco_meshdir": G1_GROOT_SIM2_MESHDIR,
            "mujoco_spec": RobotSimSpec(
                robot_id=robot_id,
                hardware_joints=joints,
                root_body_names=("pelvis",),
                root_joint_names=("floating_base_joint",),
                require_floating_base=True,
                model_joint_names=mjcf_joint_names_from_hardware(joints),
                imu_gyro_names=(
                    "imu-pelvis-angular-velocity",
                    "imu-torso-angular-velocity",
                    "imu-angular-velocity",
                    "gyro_pelvis",
                    "imu_gyro",
                ),
                imu_accel_names=(
                    "imu-pelvis-linear-acceleration",
                    "imu-torso-linear-acceleration",
                    "imu-linear-acceleration",
                    "accelerometer_pelvis",
                    "imu_accel",
                ),
                require_imu=True,
            ),
        },
    )
