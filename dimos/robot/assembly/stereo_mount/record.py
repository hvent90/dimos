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

"""Record the stereo_mount rig (ZED eyes + Point-LIO odom/lidar) into a memory2 db.

Extends :class:`~dimos.hardware.sensors.lidar.pointlio.recorder.PointlioRecorder`
(``pointlio_odometry`` / ``pointlio_lidar`` with odometry poses baked in) with the
SDK-free ZED module's ``color_image_left`` / ``color_image_right`` — names already
match, so autoconnect wires them straight in. Point-LIO publishes the moving
``world -> lidar_link`` edge onto tf and the rig's static urdf frames tie the
cameras into that tree, so every stream lands world-anchored.
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.msgs.sensor_msgs.Image import Image


class StereoMountRecorder(PointlioRecorder):
    # pointlio_odometry / pointlio_lidar are inherited from PointlioRecorder.
    color_image_left: In[Image]
    color_image_right: In[Image]
