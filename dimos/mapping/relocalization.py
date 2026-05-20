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

import threading, time, collections
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos.utils.data import get_data

from dimos.mapping.relocalize import relocalize as _relocalize

logger = setup_logger()

FRAME_MAP = "map"
FRAME_WORLD = "world"

DEFAULT_Z_OFFSET = 2.0      # before the first relocalize() converges, offset map this much in z
ACCUM_MAX = 30              # accumulated lidars scans to feed into relocalize() call
PUBLISH_INTERVAL = 2.0      # for loaded_map + TF
RELOC_INTERVAL = 2.0
MIN_LOCAL_POINTS = 50000


class RelocalizationModule(Module):
    lidar: In[PointCloud2]
    loaded_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._map_data: PointCloud2 | None = None
        self._running = False
        self._publish_thread: threading.Thread | None = None
        self._reloc_thread: threading.Thread | None = None

        self._accum: collections.deque[PointCloud2] = collections.deque(maxlen=ACCUM_MAX)
        self._accum_lock = threading.Lock()

        self._scan_frame_id: str = FRAME_WORLD

        self._tf_lock = threading.Lock()
        self._world_to_map: Transform = Transform(
            translation=Vector3(0.0, 0.0, DEFAULT_Z_OFFSET),
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        )

    @rpc
    def start(self):
        super().start()

        self._map_data = PointCloud2.lcm_decode(
            get_data("go2_hongkong_office_twopass_map.pc2.lcm").read_bytes()
        )
        self._map_data.frame_id = FRAME_MAP
        self._running = True
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))

        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()
        self._reloc_thread = threading.Thread(target=self._reloc_loop, daemon=True)
        self._reloc_thread.start()

        logger.info(
            f"Relocalization module started: "
            f"loaded_map.frame_id={self._map_data.frame_id!r}  "
            f"placeholder TF {FRAME_WORLD!r} -> {FRAME_MAP!r}  "
            f"z_offset={DEFAULT_Z_OFFSET}"
        )

    @rpc
    def stop(self) -> None:
        self._running = False
        for t in (self._publish_thread, self._reloc_thread):
            if t is not None:
                t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_lidar(self, msg: PointCloud2) -> None:
        with self._accum_lock:
            self._accum.append(msg)

    def _reloc_loop(self) -> None:
        while self._running:
            if self._map_data is None:
                continue

            with self._accum_lock:
                scans = list(self._accum)
            if not scans:
                continue

            chunks = [s.points_f32() for s in scans if len(s) > 0]
            if not chunks:
                continue
            local_pts = np.concatenate(chunks)
            if len(local_pts) < MIN_LOCAL_POINTS:
                continue

            local_map = PointCloud2.from_numpy(local_pts)

            t0 = time.monotonic()
            try:
                T = _relocalize(self._map_data.pointcloud, local_map.pointcloud)
            except Exception:
                logger.exception("relocalize() failed")
                continue
            dt = time.monotonic() - t0

            # relocalize(scan, map) returns T such that scan_in_map_frame = T(scan_raw).
            # We are publishing a TF for map_in_scan_frame, notice that the base frame is `world`
            # so inverse the transform T here to get map_in_scan_frame

            T_inv = np.linalg.inv(T)
            new_tf = Transform(
                translation=Vector3(*T_inv[:3, 3]),
                rotation=Quaternion.from_rotation_matrix(T_inv[:3, :3]),
                frame_id=self._scan_frame_id,
                child_frame_id=FRAME_MAP,
            )
            with self._tf_lock:
                self._world_to_map = new_tf

            logger.info(
                f"relocalize: time_cost={dt:.1f}s n_pts={len(local_pts)} "
                f"reloc_t={T[:3, 3].round(3).tolist()} "
                f"TF {self._scan_frame_id!r} -> {FRAME_MAP!r} "
                f"published_t={T_inv[:3, 3].round(3).tolist()} "
            )

            time.sleep(RELOC_INTERVAL)

    def _publish_loop(self) -> None:
        while self._running:
            if self._map_data is None:
                continue
            self.loaded_map.publish(self._map_data)

            with self._tf_lock:
                tf = self._world_to_map
            self.tf.publish(tf)

            time.sleep(PUBLISH_INTERVAL)


# class GlobalLookupModule:
#     loaded_map: In[PointCloud2]

#     object_locations: {
#         "self_charging_dock": PoseStamped(frame_id="map", pose=Pose(10, 0, 0)),
#         "plant": PoseStamped(frame_id="map", pose=Pose(10, 10, 0)),
#     }

#     def start(self):
#         super().start()
#         self._map = None
#         self.loaded_map.subscribe(self._on_map)

#     def _on_map(self, msg: PointCloud2):
#         self._map = msg

#     # gives you relative pose of object in base_link frame, or None if not found
#     def lookup(self, query: str) -> Transform | None:
#         if not self._map:
#             # no relocalization until we have a map
#             return None

#         return Transform.from_pose(self.object_locations[query], frame_id="base_link")
