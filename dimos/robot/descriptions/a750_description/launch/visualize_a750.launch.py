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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_state_publisher_spawner(context: LaunchContext, hwrev):
    hwrev_str = context.perform_substitution(hwrev)

    package_path = get_package_share_directory("a750_description")
    robot_description_path = os.path.join(package_path, "urdf", f"a750_rev{hwrev_str}.urdf")

    with open(robot_description_path) as f:
        robot_description = f.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def rviz_spawner(context: LaunchContext):
    # rviz_config_file = "bimanual.rviz" if bimanual_str.lower() == "true" else "arm_only.rviz"
    rviz_config_file = "arm.rviz"
    rviz_config_path = os.path.join(
        get_package_share_directory("a750_description"), "rviz", rviz_config_file
    )

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_config_path],
            output="screen",
        ),
    ]


def generate_launch_description():
    hwrev_arg = DeclareLaunchArgument(
        "hwrev", default_value="1", description="Hardware revision (e.g., 1)"
    )

    hwrev_type = LaunchConfiguration("hwrev")

    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner, args=[hwrev_type]
    )

    rviz_loader = OpaqueFunction(function=rviz_spawner, args=[])

    return LaunchDescription(
        [
            hwrev_arg,
            robot_state_publisher_loader,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            rviz_loader,
        ]
    )
