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

mujoco = pytest.importorskip("mujoco")

from dimos.simulation.sim_module import (
    MujocoSimModule,
    MujocoSimModuleConfig,
    _find_sensor_slice,
)

pytestmark = pytest.mark.mujoco


def _write_scene_xml(path: Path) -> None:
    path.write_text(
        """
<mujoco model="scene">
  <option timestep="0.02"/>
  <worldbody>
    <geom name="static_scene_box" type="box" pos="2 0 0.5" size="0.2 0.2 0.5"/>
  </worldbody>
</mujoco>
""".strip()
    )


def _write_robot_xml(path: Path) -> None:
    path.write_text(
        """
<mujoco model="robot">
  <option timestep="0.005"/>
  <worldbody>
    <body name="pelvis" pos="0 0 0">
      <freejoint name="floating_base_joint"/>
      <geom name="body" type="sphere" size="0.05" mass="1.0"/>
      <site name="imu_site" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <gyro name="body-angular-velocity" site="imu_site"/>
    <velocimeter name="body-linear-vel" site="imu_site"/>
  </sensor>
</mujoco>
""".strip()
    )


def _entity(entity_id: str) -> dict[str, object]:
    return {
        "id": entity_id,
        "spawn": "initial",
        "descriptor": {
            "entity_id": entity_id,
            "kind": "dynamic",
            "mass": 1.0,
            "shape_hint": "box",
            "extents": [0.2, 0.2, 0.2],
        },
        "initial_pose": {
            "x": 1.0,
            "y": 0.0,
            "z": 0.1,
            "qw": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
        },
    }


def test_compose_model_keeps_robot_contract_before_scene_entities(tmp_path: Path) -> None:
    scene_xml = tmp_path / "scene.xml"
    robot_xml = tmp_path / "robot.xml"
    _write_scene_xml(scene_xml)
    _write_robot_xml(robot_xml)

    module = object.__new__(MujocoSimModule)
    module.config = MujocoSimModuleConfig(
        scene_xml=scene_xml,
        robot_mjcf=robot_xml,
        scene_entities=[_entity("chair_000")],
        support_floor=True,
        spawn_xy=(0.25, -0.5),
        spawn_z=0.8,
    )

    model = MujocoSimModule._compose_model(module)

    assert model.opt.timestep == pytest.approx(0.005)

    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "locomotion_support_floor")
    assert floor_id >= 0
    assert int(model.geom_type[floor_id]) == int(mujoco.mjtGeom.mjGEOM_PLANE)
    assert int(model.geom_group[floor_id]) == 2
    np.testing.assert_allclose(model.geom_pos[floor_id], [0.0, 0.0, 0.0])
    assert model.geom_rgba[floor_id][3] == pytest.approx(0.0)

    free_joints = []
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE):
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        free_joints.append((name or "", int(model.jnt_qposadr[joint_id])))

    robot_free = next(item for item in free_joints if item[0].endswith("floating_base_joint"))
    entity_free = next(item for item in free_joints if item[0] == "entity:chair_000:free")
    assert robot_free[1] == 0
    assert entity_free[1] > robot_free[1]


def test_composed_robot_sensor_lookup_handles_attached_names(tmp_path: Path) -> None:
    robot_xml = tmp_path / "robot.xml"
    _write_robot_xml(robot_xml)

    module = object.__new__(MujocoSimModule)
    module.config = MujocoSimModuleConfig(robot_mjcf=robot_xml)

    model = MujocoSimModule._compose_model(module)

    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "/body-angular-velocity") >= 0
    assert _find_sensor_slice(model, "body-angular-velocity", dim=3) == slice(0, 3)
    assert _find_sensor_slice(model, "body-linear-vel", dim=3) == slice(3, 6)
