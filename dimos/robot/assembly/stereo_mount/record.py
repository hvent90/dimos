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

"""Record the stereo_mount rig (ZED eyes + Mid-360) into a memory2 db.

A ``Recorder`` that records its In ports under their own names — wire the
SDK-free ZED module's ``color_image_left`` / ``color_image_right`` straight in
(names already match) and remap the Mid-360's ``lidar`` / ``imu`` to
``livox_lidar`` / ``livox_imu``. Raw streams only; the mount geometry lands in
the recording via the tf stream published by
:class:`~dimos.robot.assembly.stereo_mount.assembly.StereoMountStaticTf`.
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder, RecorderConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class StereoMountRecorderConfig(RecorderConfig):
    # Append into a populated db (keep other streams); replace only our own.
    on_existing: OnExisting = OnExisting.APPEND


class StereoMountRecorder(Recorder):
    config: StereoMountRecorderConfig

    color_image_left: In[Image]
    color_image_right: In[Image]
    livox_lidar: In[PointCloud2]
    livox_imu: In[Imu]
