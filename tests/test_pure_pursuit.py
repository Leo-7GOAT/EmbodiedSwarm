import math

import pytest

from embodied_swarm.control.pure_pursuit import (
    find_lookahead_point,
    pure_pursuit_control,
    pure_pursuit_curvature,
    world_to_body,
)


def test_world_to_body_identity_heading():
    result = world_to_body(
        (0.0, 0.0, 0.0),
        (2.0, 1.0),
    )

    assert result[0] == pytest.approx(2.0)
    assert result[1] == pytest.approx(1.0)


def test_world_to_body_rotated_heading():
    result = world_to_body(
        (0.0, 0.0, math.pi / 2.0),
        (0.0, 2.0),
    )

    assert result[0] == pytest.approx(2.0)
    assert result[1] == pytest.approx(
        0.0,
        abs=1e-12,
    )


def test_curvature_left():
    curvature = pure_pursuit_curvature(
        (2.0, 1.0)
    )

    assert curvature == pytest.approx(
        0.4
    )


def test_curvature_right():
    curvature = pure_pursuit_curvature(
        (2.0, -1.0)
    )

    assert curvature == pytest.approx(
        -0.4
    )


def test_curvature_target_at_robot():
    curvature = pure_pursuit_curvature(
        (0.0, 0.0)
    )

    assert curvature == 0.0


def test_find_lookahead_point():
    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
        (4.0, 0.0),
    ]

    target = find_lookahead_point(
        robot_xy=(0.2, 0.0),
        path=path,
        lookahead_distance=1.5,
    )

    assert target == (
        2.0,
        0.0,
    )


def test_control_straight():
    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
    ]

    v, omega = pure_pursuit_control(
        robot_pose=(0.0, 0.0, 0.0),
        path=path,
        lookahead_distance=1.5,
        linear_speed=1.0,
    )

    assert v == pytest.approx(1.0)
    assert omega == pytest.approx(0.0)


def test_control_turns_left():
    path = [
        (0.0, 0.0),
        (1.0, 1.0),
        (2.0, 1.0),
        (3.0, 1.0),
    ]

    _, omega = pure_pursuit_control(
        robot_pose=(0.0, 0.0, 0.0),
        path=path,
        lookahead_distance=1.0,
        linear_speed=1.0,
    )

    assert omega > 0.0


def test_control_turns_right():
    path = [
        (0.0, 0.0),
        (1.0, -1.0),
        (2.0, -1.0),
        (3.0, -1.0),
    ]

    _, omega = pure_pursuit_control(
        robot_pose=(0.0, 0.0, 0.0),
        path=path,
        lookahead_distance=1.0,
        linear_speed=1.0,
    )

    assert omega < 0.0


def test_control_stops_at_goal():
    path = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
    ]

    v, omega = pure_pursuit_control(
        robot_pose=(1.95, 0.0, 0.0),
        path=path,
        lookahead_distance=1.0,
        linear_speed=1.0,
        goal_tolerance=0.1,
    )

    assert v == 0.0
    assert omega == 0.0