import numpy as np
import pytest

from embodied_swarm.core.robot_command import RobotCommand


def test_default_command():
    command = RobotCommand()

    assert command.timestamp == pytest.approx(0.0)

    assert isinstance(command.base_twist, np.ndarray)
    assert command.base_twist.shape == (3,)
    assert np.allclose(command.base_twist, np.zeros(3))

    assert command.joint_position_targets is None
    assert command.gripper_width_target is None


def test_accepts_python_lists():
    command = RobotCommand(
        timestamp=1.0,
        base_twist=[0.5, 0.0, 0.2],
        joint_position_targets=[0.1] * 7,
        gripper_width_target=0.04,
    )

    assert isinstance(command.base_twist, np.ndarray)
    assert isinstance(command.joint_position_targets, np.ndarray)

    assert command.base_twist.shape == (3,)
    assert command.joint_position_targets.shape == (7,)

    assert command.gripper_width_target == pytest.approx(0.04)


@pytest.mark.parametrize(
    "base_twist",
    [
        [0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [[0.0], [0.0], [0.0]],
    ],
)
def test_rejects_wrong_base_twist_shape(base_twist):
    with pytest.raises(ValueError):
        RobotCommand(
            base_twist=base_twist,
        )


@pytest.mark.parametrize(
    "joint_targets",
    [
        [0.0] * 6,
        [0.0] * 8,
        [[0.0]] * 7,
    ],
)
def test_rejects_wrong_joint_target_shape(joint_targets):
    with pytest.raises(ValueError):
        RobotCommand(
            joint_position_targets=joint_targets,
        )


@pytest.mark.parametrize(
    "base_twist",
    [
        [np.nan, 0.0, 0.0],
        [np.inf, 0.0, 0.0],
        [0.0, -np.inf, 0.0],
    ],
)
def test_rejects_non_finite_base_twist(base_twist):
    with pytest.raises(ValueError):
        RobotCommand(
            base_twist=base_twist,
        )


@pytest.mark.parametrize(
    "joint_targets",
    [
        [0.0] * 6 + [np.nan],
        [0.0] * 6 + [np.inf],
    ],
)
def test_rejects_non_finite_joint_targets(joint_targets):
    with pytest.raises(ValueError):
        RobotCommand(
            joint_position_targets=joint_targets,
        )


def test_rejects_negative_gripper_width_target():
    with pytest.raises(ValueError):
        RobotCommand(
            gripper_width_target=-0.01,
        )


@pytest.mark.parametrize(
    "bad_value",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_rejects_non_finite_gripper_width_target(bad_value):
    with pytest.raises(ValueError):
        RobotCommand(
            gripper_width_target=bad_value,
        )


@pytest.mark.parametrize(
    "bad_timestamp",
    [
        np.nan,
        np.inf,
        -np.inf,
    ],
)
def test_rejects_non_finite_timestamp(bad_timestamp):
    with pytest.raises(ValueError):
        RobotCommand(
            timestamp=bad_timestamp,
        )


def test_command_owns_base_twist_array():
    base_twist = np.array([0.5, 0.0, 0.2])

    command = RobotCommand(
        base_twist=base_twist,
    )

    base_twist[0] = 999.0

    assert command.base_twist[0] == pytest.approx(0.5)


def test_command_owns_joint_target_array():
    joint_targets = np.array([0.1] * 7)

    command = RobotCommand(
        joint_position_targets=joint_targets,
    )

    joint_targets[0] = 999.0

    assert command.joint_position_targets[0] == pytest.approx(0.1)


def test_none_means_no_arm_or_gripper_command():
    command = RobotCommand(
        base_twist=[0.3, 0.0, 0.1],
    )

    assert command.joint_position_targets is None
    assert command.gripper_width_target is None