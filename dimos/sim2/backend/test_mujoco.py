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

from pathlib import Path

import numpy as np
import pytest

from dimos.sim2.backend.mujoco import MujocoBackend
from dimos.sim2.control.adapters.manipulator.adapter import SimManipulatorAdapter
from dimos.sim2.ipc.registry import SimRegistry
from dimos.sim2.runtime import SimRuntime
from dimos.sim2.spec import (
    ControlInterface,
    ExecutionConfig,
    RaycastLidarSpec,
    SimConfig,
    SimRobotSpec,
    SpawnPose,
    WorldSpec,
)
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.simulation.scene_assets.spec import SceneMeshAlignment, ScenePackage

pytestmark = pytest.mark.mujoco

_ARM_XML = """
<mujoco model="sim2_test_arm">
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="arm_link" pos="0 0 0">
      <joint name="joint1" type="hinge" axis="0 0 1" damping="0.1"/>
      <geom type="capsule" size="0.02 0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="joint1_actuator" joint="joint1" kp="20"/>
  </actuator>
</mujoco>
"""

_WHOLE_BODY_XML = """
<mujoco model="sim2_test_whole_body">
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="link" pos="0 0 0">
      <joint name="joint1" type="hinge" axis="0 0 1"/>
      <geom type="capsule" size="0.02 0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="joint1_actuator" joint="joint1"/>
  </actuator>
</mujoco>
"""

_LIDAR_XML = """
<mujoco model="sim2_test_lidar">
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05" group="2"/>
    <body name="sensor_link" pos="0 0 1">
      <joint name="joint1" type="hinge" axis="0 0 1"/>
      <geom type="sphere" size="0.02" mass="1"/>
      <camera name="lidar_camera" xyaxes="1 0 0 0 1 0" fovy="90"/>
    </body>
  </worldbody>
  <actuator><position name="joint1_actuator" joint="joint1" kp="20"/></actuator>
</mujoco>
"""


def test_mujoco_backend_runs_through_generic_manipulator_adapter(tmp_path: Path) -> None:
    model_path = tmp_path / "arm.xml"
    model_path.write_text(_ARM_XML)
    registry = SimRegistry(run_id="mujoco-backend", root=tmp_path / "registry")
    robot = SimRobotSpec(
        robot_id="arm",
        control_interface=ControlInterface.MANIPULATOR,
        dof=1,
        joint_names=("arm/joint1",),
        model_path=model_path,
        backend_options={
            "mujoco_spec": RobotSimSpec(
                robot_id="arm",
                hardware_joints=("arm/joint1",),
                model_joint_names=("joint1",),
            )
        },
    )
    runtime = SimRuntime(
        SimConfig(
            sim_id="main",
            backend=MujocoBackend(),
            robots=(robot,),
            execution=ExecutionConfig(
                physics_dt=0.002,
                control_decimation=10,
                autostart=False,
            ),
        ),
        registry=registry,
    )
    runtime.start()
    adapter = SimManipulatorAdapter(dof=1, hardware_id="arm", registry=registry)
    try:
        assert adapter.connect()
        assert adapter.write_joint_positions([0.5])

        runtime.step(10)

        assert adapter.read_joint_positions()[0] > 0.0
        observation = runtime.channels["arm"].read_observation()
        assert observation is not None
        assert observation.metadata.applied_action_sequence == 1
        assert observation.metadata.physics_tick == 100
    finally:
        adapter.disconnect()
        runtime.close()


def test_whole_body_pd_is_recomputed_on_every_physics_step(tmp_path: Path) -> None:
    model_path = tmp_path / "whole_body.xml"
    model_path.write_text(_WHOLE_BODY_XML)
    robot = SimRobotSpec(
        robot_id="robot",
        control_interface=ControlInterface.WHOLE_BODY,
        dof=1,
        joint_names=("robot/joint1",),
        model_path=model_path,
        backend_options={
            "mujoco_spec": RobotSimSpec(
                robot_id="robot",
                hardware_joints=("robot/joint1",),
                model_joint_names=("joint1",),
            )
        },
    )
    backend = MujocoBackend()
    handle = backend.load(WorldSpec(), (robot,), physics_dt=0.002)["robot"]
    try:
        backend.apply_action(
            handle,
            {
                "enabled": np.array([1], dtype=np.uint8),
                "position": np.array([1.0]),
                "velocity": np.array([0.0]),
                "kp": np.array([10.0]),
                "kd": np.array([0.0]),
                "effort": np.array([0.0]),
            },
        )
        assert backend._data is not None
        assert backend._data.ctrl[0] == pytest.approx(10.0)

        backend._data.qpos[0] = 0.5
        backend.step(0.002)

        assert backend._data.ctrl[0] == pytest.approx(5.0)
    finally:
        backend.close()


def test_scene_package_composes_robot_and_tracks_cooked_entities(tmp_path: Path) -> None:
    scene_path = tmp_path / "scene.xml"
    scene_path.write_text(
        """
        <mujoco model="scene">
          <worldbody><geom name="floor" type="plane" size="0 0 0.05"/></worldbody>
        </mujoco>
        """
    )
    robot_path = tmp_path / "arm.xml"
    robot_path.write_text(_ARM_XML)
    scene = ScenePackage(
        package_dir=tmp_path,
        source_path=scene_path,
        alignment=SceneMeshAlignment(),
        mujoco_scene_path=scene_path,
        entities=[
            {
                "id": "box",
                "spawn": "initial",
                "initial_pose": {"x": 1.0, "y": 2.0, "z": 0.5, "qw": 1.0},
                "descriptor": {
                    "entity_id": "box",
                    "kind": "static",
                    "shape_hint": "box",
                    "extents": [0.2, 0.2, 0.2],
                },
            }
        ],
    )
    robot = SimRobotSpec(
        robot_id="arm",
        control_interface=ControlInterface.MANIPULATOR,
        dof=1,
        joint_names=("arm/joint1",),
        model_path=robot_path,
        backend_options={
            "mujoco_spec": RobotSimSpec(
                robot_id="arm",
                hardware_joints=("arm/joint1",),
                model_joint_names=("joint1",),
            )
        },
    )
    backend = MujocoBackend()
    try:
        backend.load(WorldSpec(scene=scene, revision="test-scene"), (robot,), 0.002)
        states = {state.entity_id: state for state in backend.entity_states()}
        assert states["box"].position == pytest.approx((1.0, 2.0, 0.5))
        assert "arm" in states
    finally:
        backend.close()


def test_native_lidar_is_sampled_from_authoritative_mujoco_state(tmp_path: Path) -> None:
    model_path = tmp_path / "lidar.xml"
    model_path.write_text(_LIDAR_XML)
    robot = SimRobotSpec(
        robot_id="robot",
        control_interface=ControlInterface.MANIPULATOR,
        dof=1,
        joint_names=("robot/joint1",),
        model_path=model_path,
        sensors=(
            RaycastLidarSpec(
                camera_names=("lidar_camera",),
                width=4,
                height=3,
                rate_hz=2.0,
                min_range=0.1,
                max_range=2.0,
                geom_groups=(2,),
            ),
        ),
        backend_options={
            "mujoco_spec": RobotSimSpec(
                robot_id="robot",
                hardware_joints=("robot/joint1",),
                model_joint_names=("joint1",),
            )
        },
    )
    backend = MujocoBackend()
    try:
        backend.load(WorldSpec(), (robot,), 0.002)
        samples = backend.sensor_samples(0.0)
        assert len(samples) == 1
        assert samples[0].sensor_id == "lidar"
        assert samples[0].payload["points"].shape[1] == 3
        assert len(samples[0].payload["points"]) > 0
        assert backend.sensor_samples(0.1) == ()
        backend.reset()
        assert len(backend.sensor_samples(0.0)) == 1
    finally:
        backend.close()


def test_multi_robot_composition_prefixes_control_channels(tmp_path: Path) -> None:
    model_path = tmp_path / "arm.xml"
    model_path.write_text(_ARM_XML)

    def robot(robot_id: str, x: float) -> SimRobotSpec:
        return SimRobotSpec(
            robot_id=robot_id,
            control_interface=ControlInterface.MANIPULATOR,
            dof=1,
            joint_names=(f"{robot_id}/joint1",),
            model_path=model_path,
            spawn=SpawnPose(position=(x, 0.0, 0.0)),
            backend_options={
                "mujoco_spec": RobotSimSpec(
                    robot_id=robot_id,
                    hardware_joints=(f"{robot_id}/joint1",),
                    model_joint_names=("joint1",),
                )
            },
        )

    backend = MujocoBackend()
    first = robot("first", -1.0)
    second = robot("second", 1.0)
    try:
        handles = backend.load(WorldSpec(), (first, second), physics_dt=0.002)
        for robot_id, target in (("first", 0.5), ("second", -0.5)):
            backend.apply_action(
                handles[robot_id],
                {
                    "command_mode": np.array([0], dtype=np.int32),
                    "enabled": np.array([1], dtype=np.uint8),
                    "position": np.array([target]),
                    "velocity": np.zeros(1),
                    "effort": np.zeros(1),
                    "velocity_scale": np.array([1.0]),
                    "gripper": np.array([0.0]),
                },
            )
        for _ in range(50):
            backend.step(0.002)

        first_observation = backend.observe(handles["first"])
        second_observation = backend.observe(handles["second"])
        states = {state.entity_id: state for state in backend.entity_states()}
        assert first_observation["position"][0] > 0.0
        assert second_observation["position"][0] < 0.0
        assert handles["first"].backend_data.binding.actuator_ids != (
            handles["second"].backend_data.binding.actuator_ids
        )
        assert states["first"].position == pytest.approx((-1.0, 0.0, 0.0))
        assert states["second"].position == pytest.approx((1.0, 0.0, 0.0))
    finally:
        backend.close()
