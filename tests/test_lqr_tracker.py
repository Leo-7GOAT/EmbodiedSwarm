import math

import numpy as np
import pytest

from embodied_swarm.control.lqr_tracker import (
    build_error_dynamics,
    compute_lqr_gain,
    compute_tracking_error,
    lqr_control,
    normalize_angle,
    solve_discrete_riccati,
)


def test_tracking_error_uses_reference_frame_sign_convention() -> None:
    lateral, yaw = compute_tracking_error(
        pose=(0.0, 1.0, math.radians(10.0)),
        reference_pos=(0.0, 0.0, 0.0),
    )
    assert lateral == pytest.approx(1.0)
    assert yaw == pytest.approx(math.radians(10.0))

    lateral, _ = compute_tracking_error(
        pose=(0.0, 1.0, math.pi / 2.0),
        reference_pos=(0.0, 0.0, math.pi / 2.0),
    )
    assert lateral == pytest.approx(0.0, abs=1e-12)


def test_tracking_yaw_error_wraps_at_pi() -> None:
    _, yaw = compute_tracking_error(
        pose=(0.0, 0.0, -math.pi + 0.1),
        reference_pos=(0.0, 0.0, math.pi - 0.1),
    )
    assert yaw == pytest.approx(0.2)
    assert normalize_angle(3.0 * math.pi / 2.0) == pytest.approx(-math.pi / 2.0)


def test_error_dynamics_match_discrete_lateral_model() -> None:
    A, B = build_error_dynamics(linear_velocity=2.0, dt=0.1)

    np.testing.assert_allclose(A, [[1.0, 0.2], [0.0, 1.0]])
    np.testing.assert_allclose(B, [[0.0], [0.1]])

    with pytest.raises(ValueError, match="dt must be positive"):
        build_error_dynamics(linear_velocity=1.0, dt=0.0)


def test_riccati_solution_satisfies_residual_and_stabilizes_system() -> None:
    A, B = build_error_dynamics(linear_velocity=1.0, dt=0.1)
    Q = np.diag([2.0, 1.0])
    R = np.array([[0.5]])

    P = solve_discrete_riccati(A, B, Q, R)
    K = compute_lqr_gain(A, B, Q, R)
    residual = Q + A.T @ P @ A - A.T @ P @ B @ K - P

    np.testing.assert_allclose(residual, np.zeros((2, 2)), atol=1e-7)
    assert np.max(np.abs(np.linalg.eigvals(A - B @ K))) < 1.0


def test_lqr_control_has_correct_sign_and_saturates() -> None:
    K = np.array([[2.0, 1.0]])

    assert lqr_control((0.0, 1.0, 0.0), (0.0, 0.0, 0.0), K) < 0.0
    assert lqr_control((0.0, -1.0, 0.0), (0.0, 0.0, 0.0), K) > 0.0
    assert lqr_control(
        (0.0, 10.0, 0.0),
        (0.0, 0.0, 0.0),
        K,
        max_angular_velocity=0.3,
    ) == pytest.approx(-0.3)

    with pytest.raises(ValueError, match="max_angular_velocity"):
        lqr_control(
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
            K,
            max_angular_velocity=0.0,
        )


def test_lqr_closed_loop_converges_from_lateral_offset() -> None:
    dt = 0.1
    speed = 1.0
    A, B = build_error_dynamics(speed, dt)
    K = compute_lqr_gain(A, B, np.diag([2.0, 1.0]), np.array([[0.5]]))
    pose = (0.0, 1.0, 0.0)
    reference = (0.0, 0.0, 0.0)

    for _ in range(100):
        omega = lqr_control(pose, reference, K, max_angular_velocity=1.5)
        x, y, yaw = pose
        pose = (
            x + speed * math.cos(yaw) * dt,
            y + speed * math.sin(yaw) * dt,
            normalize_angle(yaw + omega * dt),
        )

    lateral, yaw = compute_tracking_error(pose, reference)
    assert abs(lateral) < 0.01
    assert abs(yaw) < math.radians(0.1)
