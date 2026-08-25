import numpy as np
import pytest

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.hal.mock_hardware_adapter import (
    MockHardwareAdapter,
)


def test_default_state():
    robot = MockHardwareAdapter()

    state = robot.read_state()

    assert state.timestamp == pytest.approx(0.0)
    assert np.allclose(state.base_pose, np.zeros(3))
    assert np.allclose(state.joint_positions, np.zeros(7))


def test_forward_motion():
    robot = MockHardwareAdapter()

    robot.send_command(
        RobotCommand(
            base_twist=[1.0, 0.0, 0.0]
        )
    )

    state = robot.step(1.0)

    assert state.base_pose[0] == pytest.approx(1.0)
    assert state.base_pose[1] == pytest.approx(0.0)
    assert state.base_pose[2] == pytest.approx(0.0)


def test_motion_at_90_degrees():
    robot = MockHardwareAdapter()

    robot._state.base_pose = np.array(
        [0.0, 0.0, np.pi / 2]
    )

    robot.send_command(
        RobotCommand(
            base_twist=[0.5, 0.0, 0.0]
        )
    )

    state = robot.step(1.0)

    assert state.base_pose[0] == pytest.approx(
        0.0,
        abs=1e-9,
    )

    assert state.base_pose[1] == pytest.approx(0.5)


def test_robot_rotates():
    robot = MockHardwareAdapter()

    robot.send_command(
        RobotCommand(
            base_twist=[0.0, 0.0, 0.2]
        )
    )

    state = robot.step(2.0)

    assert state.base_pose[2] == pytest.approx(0.4)


def test_arm_moves_toward_target():
    robot = MockHardwareAdapter()

    target = np.zeros(7)
    target[1] = 0.5

    robot.send_command(
        RobotCommand(
            joint_position_targets=target
        )
    )

    before = robot.read_state()

    after = robot.step(0.1)

    assert after.joint_positions[1] > before.joint_positions[1]

    assert (
        after.joint_positions[1]
        < target[1]
    )


def test_none_arm_command_holds_position():
    robot = MockHardwareAdapter()

    robot._state.joint_positions[2] = 0.4

    robot.send_command(
        RobotCommand(
            base_twist=[0.2, 0.0, 0.0]
        )
    )

    state = robot.step(0.1)

    assert state.joint_positions[2] == pytest.approx(0.4)
    assert np.allclose(
        state.joint_velocities,
        np.zeros(7),
    )


def test_gripper_command():
    robot = MockHardwareAdapter()

    robot.send_command(
        RobotCommand(
            gripper_width_target=0.04
        )
    )

    state = robot.step(0.1)

    assert state.gripper_width == pytest.approx(0.04)


def test_rejects_lateral_velocity():
    robot = MockHardwareAdapter()

    with pytest.raises(ValueError):
        robot.send_command(
            RobotCommand(
                base_twist=[0.0, 0.2, 0.0]
            )
        )


def test_reset():
    robot = MockHardwareAdapter()

    robot.send_command(
        RobotCommand(
            base_twist=[1.0, 0.0, 0.0]
        )
    )

    robot.step(1.0)

    state = robot.reset()

    assert state.timestamp == pytest.approx(0.0)
    assert np.allclose(state.base_pose, np.zeros(3))


def test_shutdown_blocks_step():
    robot = MockHardwareAdapter()

    robot.shutdown()

    with pytest.raises(RuntimeError):
        robot.step(0.1)


def test_read_state_returns_snapshot():
    robot = MockHardwareAdapter()

    state = robot.read_state()

    state.base_pose[0] = 999.0

    internal = robot.read_state()

    assert internal.base_pose[0] == pytest.approx(0.0)
