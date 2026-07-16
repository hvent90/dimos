import math

import pytest

mujoco = pytest.importorskip("mujoco")

pytestmark = pytest.mark.mujoco

from dimos.robot.manipulators.a1z.simulation import A1Z_SCENE_PATH


def test_a1z_scene_compiles_and_has_stable_contract() -> None:
    model = mujoco.MjModel.from_xml_path(str(A1Z_SCENE_PATH))
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    assert names[:6] == [f"arm_joint{i}" for i in range(1, 7)]
    assert names[6:8] == ["gripper_finger_left_joint", "gripper_finger_rIght_joint"]
    assert model.jnt_range[6].tolist() == pytest.approx([0.0, 0.015])
    axis_norm = math.hypot(0.811, 0.584)
    assert model.jnt_axis[6].tolist() == pytest.approx([0.0, -0.811 / axis_norm, 0.584 / axis_norm])
    assert model.jnt_axis[7].tolist() == pytest.approx([0.0, 0.811 / axis_norm, -0.584 / axis_norm])
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    assert actuator_names == [f"arm_joint{i}_motor" for i in range(1, 7)] + ["gripper_motor"]
    assert model.nu == 7
    assert model.actuator_gainprm[:6, 0].tolist() == pytest.approx([500.0] * 6)
    assert model.actuator_biasprm[:6, 2].tolist() == pytest.approx([-45.0] * 6)
    assert model.actuator_gainprm[6, 0] == pytest.approx(10.0)
    assert model.actuator_biasprm[6, 2] == pytest.approx(-5.0)
    assert model.eq_obj1id[0] == 7
    assert model.eq_obj2id[0] == 6
    assert model.eq_data[0, :5].tolist() == pytest.approx([0.0, 1.0, 0.0, 0.0, 0.0])
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_camera") >= 0
    assert model.vis.global_.offwidth == 640
    assert model.vis.global_.offheight == 480
    fps_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, "wrist_camera_fps")
    assert model.numeric_data[model.numeric_adr[fps_id]] == pytest.approx(30.0)
    data = mujoco.MjData(model)
    for value in (0.0, 0.0075, 0.015):
        data.ctrl[6] = value
        for _ in range(20):
            mujoco.mj_step(model, data)
        assert data.qpos[6] == pytest.approx(data.qpos[7], abs=2e-3)


def test_a1z_arm_position_hold_and_small_command() -> None:
    model = mujoco.MjModel.from_xml_path(str(A1Z_SCENE_PATH))
    data = mujoco.MjData(model)
    home = [0.0, 0.7, -1.2, 0.5, 0.0, 0.0]
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:6] = home

    for _ in range(500):
        mujoco.mj_step(model, data)

    home_drift = max(abs(float(data.qpos[i]) - home[i]) for i in range(6))
    assert home_drift < 0.02

    commanded = home[1] + 0.1
    data.ctrl[1] = commanded
    for _ in range(500):
        mujoco.mj_step(model, data)

    assert abs(float(data.qpos[1]) - commanded) < 0.01
