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

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import numpy as np

from dimos.simulation.engines.mujoco_engine import CameraFrame
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig


class _FakeData:
    qpos = np.array([0.0, 0.0, 0.75, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    sensordata = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0], dtype=np.float64)


class _FakeEngine:
    data = _FakeData()
    joint_names = ["joint_a", "joint_b"]


def test_ready_signal_happens_after_joint_state_and_imu_write() -> None:
    events: list[str] = []
    module = object.__new__(MujocoSimModule)
    module._shm_ready_signaled = False
    module._root_base_qpos_adr = 0
    module._imu_quat_slice = None
    module._imu_base_qpos_slice = slice(3, 7)
    module._imu_gyro_slice = slice(0, 3)
    module._imu_accel_slice = slice(3, 6)
    module.odom = MagicMock()
    module.imu = MagicMock()

    class _FakeHooks:
        def post_step(self, engine: Any) -> None:
            assert engine is _FakeEngine
            events.append("joint_state")

    class _FakeShm:
        def write_imu(self, **_: Any) -> None:
            events.append("imu")

        def signal_ready(self, *, num_joints: int) -> None:
            assert num_joints == 2
            events.append("ready")

    module._sim_hooks = _FakeHooks()
    module._shm = _FakeShm()

    module._publish_shm_and_lcm(_FakeEngine)

    assert events == ["joint_state", "imu", "ready"]


def test_camera_tf_is_published_relative_to_configured_base_frame() -> None:
    module = object.__new__(MujocoSimModule)
    module.config = MujocoSimModuleConfig(base_frame_id="link7")

    class _FakeEngine:
        def get_body_pose(self, body_name: str):  # type: ignore[no-untyped-def]
            assert body_name == "link7"
            return np.array([1.0, 2.0, 2.0]), np.eye(3)

    class _FakeTf:
        transforms = ()

        def publish(self, *transforms: Any) -> None:
            self.transforms = transforms

    fake_tf = _FakeTf()
    module._engine = _FakeEngine()
    module._tf = fake_tf
    frame = CameraFrame(
        rgb=np.zeros((1, 1, 3), dtype=np.uint8),
        depth=np.ones((1, 1), dtype=np.float32),
        cam_pos=np.array([1.0, 2.0, 3.0]),
        cam_mat=np.eye(3),
        fovy=60.0,
        timestamp=1.0,
    )

    module._publish_tf(10.0, frame)

    color_tf, depth_tf, camera_link_tf = fake_tf.transforms
    assert color_tf.frame_id == "link7"
    assert color_tf.child_frame_id == "wrist_camera_color_optical_frame"
    assert np.allclose(color_tf.translation.to_numpy(), [0.0, 0.0, 1.0])
    assert depth_tf.frame_id == "link7"
    assert depth_tf.child_frame_id == "wrist_camera_depth_optical_frame"
    assert camera_link_tf.frame_id == "link7"
    assert camera_link_tf.child_frame_id == "wrist_camera_link"
    assert np.allclose(camera_link_tf.translation.to_numpy(), [0.0, 0.0, 1.0])
