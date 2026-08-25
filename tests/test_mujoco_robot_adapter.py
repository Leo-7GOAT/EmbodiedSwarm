from pathlib import Path

import numpy as np
import pytest

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.hal.mujoco_robot_adapter import (
    MujocoRobotAdapter,
)


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "mjcf"
    / "mobile_manipulator.xml"
)


@pytest.fixture
def adapter():
    robot = MujocoRobotAdapter(MODEL_PATH)

    yield robot

    robot.shutdown()


def test_initial_state(adapter):
    state = adapter.read_state()

    assert isinstance(state, RobotState)

    assert state.timestamp == pytest.approx(0.0)

    assert np.allclose(
        state.base_pose,
        np.zeros(3),
    )

    assert np.allclose(
        state.base_twist,
        np.zeros(3),
    )


def test_forward_motion(adapter):
    adapter.send_command(
        RobotCommand(
            base_twist=[
                1.0,
                0.0,
                0.0,
            ]
        )
    )

    state = adapter.step(1.0)

    assert state.base_pose[0] == pytest.approx(
        1.0,
        abs=1e-6,
    )

    assert state.base_pose[1] == pytest.approx(
        0.0,
        abs=1e-6,
    )

    assert state.base_pose[2] == pytest.approx(
        0.0,
        abs=1e-6,
    )


def test_rotation(adapter):
    adapter.send_command(
        RobotCommand(
            base_twist=[
                0.0,
                0.0,
                np.pi / 2,
            ]
        )
    )

    state = adapter.step(1.0)

    assert state.base_pose[2] == pytest.approx(
        np.pi / 2,
        abs=1e-6,
    )


def test_body_velocity_at_90_degrees(adapter):
    # Put robot at yaw = 90 degrees.
    adapter._data.qpos[
        adapter._base_yaw_qpos_addr
    ] = np.pi / 2

    adapter._data.qpos[
        adapter._base_x_qpos_addr
    ] = 0.0

    adapter._data.qpos[
        adapter._base_y_qpos_addr
    ] = 0.0

    adapter.send_command(
        RobotCommand(
            base_twist=[
                1.0,
                0.0,
                0.0,
            ]
        )
    )

    state = adapter.step(1.0)

    # Robot's forward direction now points along world +Y.
    assert state.base_pose[0] == pytest.approx(
        0.0,
        abs=1e-6,
    )

    assert state.base_pose[1] == pytest.approx(
        1.0,
        abs=1e-6,
    )

    # But RobotState still reports body-frame forward velocity.
    assert state.base_twist[0] == pytest.approx(
        1.0,
        abs=1e-6,
    )

    assert state.base_twist[1] == pytest.approx(
        0.0,
        abs=1e-6,
    )


def test_rejects_lateral_velocity(adapter):
    with pytest.raises(ValueError):
        adapter.send_command(
            RobotCommand(
                base_twist=[
                    0.0,
                    0.5,
                    0.0,
                ]
            )
        )


def test_reset(adapter):
    adapter.send_command(
        RobotCommand(
            base_twist=[
                1.0,
                0.0,
                0.0,
            ]
        )
    )

    adapter.step(1.0)

    state = adapter.reset()

    assert state.timestamp == pytest.approx(0.0)

    assert np.allclose(
        state.base_pose,
        np.zeros(3),
    )


def test_shutdown_blocks_step():
    adapter = MujocoRobotAdapter(MODEL_PATH)

    adapter.shutdown()

    with pytest.raises(RuntimeError):
        adapter.step(0.1)


def test_shutdown_blocks_command():
    adapter = MujocoRobotAdapter(MODEL_PATH)

    adapter.shutdown()

    with pytest.raises(RuntimeError):
        adapter.send_command(
            RobotCommand()
        )