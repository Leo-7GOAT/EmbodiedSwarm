import numpy as np
import pytest

from embodied_swarm.core.robot_state import RobotState


def test_default_state():
    state = RobotState(timestamp=0.0)

    assert state.base_pose.shape == (3,)
    assert state.base_twist.shape == (3,)
    assert state.joint_positions.shape == (7,)
    assert state.joint_velocities.shape == (7,)
    assert state.gripper_width == pytest.approx(0.0)


def test_accepts_python_lists():
    state = RobotState(
        timestamp=1.0,
        base_pose=[1.0, 2.0, 0.3],
        base_twist=[0.5, 0.0, 0.1],
        joint_positions=[0.0] * 7,
        joint_velocities=[0.0] * 7,
    )

    assert isinstance(state.base_pose, np.ndarray)
    assert isinstance(state.base_twist, np.ndarray)
    assert isinstance(state.joint_positions, np.ndarray)
    assert isinstance(state.joint_velocities, np.ndarray)


@pytest.mark.parametrize(
    "field_name,value",
    [
        ("base_pose", [0.0, 0.0]),
        ("base_twist", [0.0, 0.0]),
        ("joint_positions", [0.0] * 6),
        ("joint_velocities", [0.0] * 8),
    ],
)
def test_rejects_wrong_vector_shapes(field_name, value):
    kwargs = {
        "timestamp": 0.0,
        field_name: value,
    }

    with pytest.raises(ValueError):
        RobotState(**kwargs)


@pytest.mark.parametrize(
    "field_name,value",
    [
        ("base_pose", [0.0, np.nan, 0.0]),
        ("base_twist", [np.inf, 0.0, 0.0]),
        ("joint_positions", [0.0] * 6 + [np.nan]),
        ("joint_velocities", [0.0] * 6 + [np.inf]),
    ],
)
def test_rejects_non_finite_vectors(field_name, value):
    kwargs = {
        "timestamp": 0.0,
        field_name: value,
    }

    with pytest.raises(ValueError):
        RobotState(**kwargs)


def test_rejects_negative_gripper_width():
    with pytest.raises(ValueError):
        RobotState(
            timestamp=0.0,
            gripper_width=-0.01,
        )


def test_rejects_non_finite_gripper_width():
    with pytest.raises(ValueError):
        RobotState(
            timestamp=0.0,
            gripper_width=np.nan,
        )


def test_state_owns_array_data():
    pose = np.array([1.0, 2.0, 0.3])

    state = RobotState(
        timestamp=0.0,
        base_pose=pose,
    )

    pose[0] = 999.0

    assert state.base_pose[0] == pytest.approx(1.0)