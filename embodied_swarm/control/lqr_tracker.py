import math

import numpy as np


def compute_tracking_error(
        pose: tuple[float, float, float],
        reference_pos: tuple[float, float, float],
) -> tuple[float, float]:

    dx = pose[0] - reference_pos[0]
    dy = pose[1] - reference_pos[1]

    lateral_error = (
            -math.sin(reference_pos[2]) * dx
            + math.cos(reference_pos[2]) * dy)

    yaw_error = normalize_angle(pose[2] - reference_pos[2])

    error = lateral_error, yaw_error

    return error

def normalize_angle(angle: float) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )

def build_error_dynamics(
        linear_velocity: float,
        dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    A = np.array([
        [1, linear_velocity * dt],
        [0, 1]
    ])

    B = np.array([
        [0],
        [dt]
    ])
    return A, B

def solve_discrete_riccati(
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        max_iterations: int = 1000,
        tolerance: float = 1e-9,
) -> np.ndarray:
    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    if tolerance <= 0.0:
        raise ValueError("tolerance must be positive")

    P = Q.copy()

    for _ in range(max_iterations):
        S = R + B.T @ P @ B

        K = np.linalg.solve(S, B.T @ P @ A)

        P_new = (Q + A.T @ P @ A - A.T @ P @ B @ K)

        if np.max(np.abs(P_new - P)) < tolerance:
            return P_new
        P = P_new

    raise RuntimeError(
        "discrete Riccati iteration did not converge"
    )

def compute_lqr_gain(
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
) -> np.ndarray:
    P = solve_discrete_riccati(A, B, Q, R)
    S = R + B.T @ P @ B

    K = np.linalg.solve(S, B.T @ P @ A)
    return K

def lqr_control(
        pose: tuple[float, float, float],
        reference_pose: tuple[float, float, float],
        K: np.ndarray,
        max_angular_velocity: float = 1.5,
) -> float:
    if max_angular_velocity <= 0.0:
        raise ValueError(
            "max_angular_velocity must be positive"
        )

    lateral_error, yaw_error = compute_tracking_error(
        pose,
        reference_pose,
    )

    error = np.array([
        [lateral_error],
        [yaw_error]
    ])

    angular_velocity = float(
        (-K @ error)[0, 0]
    )
    angular_velocity = float(
        np.clip(
            angular_velocity,
            -max_angular_velocity,
            max_angular_velocity,
        )
    )
    return angular_velocity
