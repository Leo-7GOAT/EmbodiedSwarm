from pathlib import Path

import numpy as np
import pytest

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.hal.mock_hardware_adapter import MockHardwareAdapter
from embodied_swarm.hal.mujoco_robot_adapter import MujocoRobotAdapter
from embodied_swarm.hal.robot_adapter import RobotAdapter


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "mjcf"
    / "mobile_manipulator.xml"
)


def run_same_upper_level_program(adapter: RobotAdapter):
    try:
        adapter.reset()
        adapter.send_command(
            RobotCommand(
                base_twist=[0.5, 0.0, 0.2],
                joint_position_targets=[0.1] * 7,
                gripper_width_target=0.03,
            )
        )
        return adapter.step(0.2)
    finally:
        adapter.shutdown()


@pytest.mark.parametrize(
    "adapter_factory",
    [MockHardwareAdapter, lambda: MujocoRobotAdapter(MODEL_PATH)],
    ids=["mock", "mujoco"],
)
def test_adapter_switch_contract(adapter_factory):
    adapter = adapter_factory()
    state = run_same_upper_level_program(adapter)
    assert state.base_pose.shape == (3,)
    assert state.base_twist.shape == (3,)
    assert state.joint_positions.shape == (7,)
    assert state.joint_velocities.shape == (7,)
    assert state.gripper_width == pytest.approx(0.03)
    assert np.all(np.isfinite(state.base_pose))
    assert np.all(np.isfinite(state.joint_positions))
