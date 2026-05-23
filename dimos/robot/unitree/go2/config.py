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

from importlib import resources
from pathlib import Path

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.config import RobotConfig

_FRONT_CAMERA_720_YAML = resources.files("dimos.robot.unitree.go2").joinpath(
    "front_camera_720.yaml"
)


def camera_info_static() -> CameraInfo:
    with resources.as_file(_FRONT_CAMERA_720_YAML) as yaml_path:
        return CameraInfo.from_yaml(str(yaml_path))


Go2Config = RobotConfig(
    name="unitree_go2",
    model_path=Path(__file__).parent / "go2.urdf",
)
