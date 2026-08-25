from pathlib import Path

import mujoco

from embodied_swarm.core.frames import FRAME_CHAIN
from embodied_swarm.hal.mujoco_robot_adapter import MujocoRobotAdapter


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "mjcf"
    / "mobile_manipulator.xml"
)


def test_w02_model_dimensions_and_named_interfaces():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    assert (model.nq, model.nv, model.nu) == (10, 10, 7)

    joint_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    }
    body_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index)
        for index in range(model.nbody)
    }
    site_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, index)
        for index in range(model.nsite)
    }
    actuator_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        for index in range(model.nu)
    }

    assert {"base_x", "base_y", "base_yaw"}.issubset(joint_names)
    assert set(MujocoRobotAdapter.ARM_JOINT_NAMES).issubset(joint_names)
    assert set(FRAME_CHAIN[1:-1]).issubset(body_names)
    assert "end_effector" in site_names
    assert set(MujocoRobotAdapter.ARM_ACTUATOR_NAMES) == actuator_names
