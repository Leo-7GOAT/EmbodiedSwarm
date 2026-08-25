import math

import numpy as np


WORLD_FRAME = "world"
BASE_FRAME = "base_link"
ARM_BASE_FRAME = "panda_link0"
ARM_LINK_FRAMES = tuple(f"panda_link{index}" for index in range(1, 8))
HAND_FRAME = "panda_hand"
END_EFFECTOR_FRAME = "end_effector"

FRAME_CHAIN = (
    WORLD_FRAME,
    BASE_FRAME,
    ARM_BASE_FRAME,
    *ARM_LINK_FRAMES,
    HAND_FRAME,
    END_EFFECTOR_FRAME,
)


def _finite_planar_velocity(
    first: float,
    second: float,
    yaw: float,
) -> tuple[float, float, float]:
    values = np.asarray([first, second, yaw], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("planar velocity and yaw must be finite")
    return float(values[0]), float(values[1]), float(values[2])


def body_to_world_velocity(
    vx_body: float,
    vy_body: float,
    yaw: float,
) -> tuple[float, float]:
    """Rotate a planar body-frame velocity into the world frame."""
    vx_body, vy_body, yaw = _finite_planar_velocity(vx_body, vy_body, yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * vx_body - sin_yaw * vy_body,
        sin_yaw * vx_body + cos_yaw * vy_body,
    )


def world_to_body_velocity(
    x_dot_world: float,
    y_dot_world: float,
    yaw: float,
) -> tuple[float, float]:
    """Rotate a planar world-frame velocity into the body frame."""
    x_dot_world, y_dot_world, yaw = _finite_planar_velocity(
        x_dot_world, y_dot_world, yaw
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x_dot_world + sin_yaw * y_dot_world,
        -sin_yaw * x_dot_world + cos_yaw * y_dot_world,
    )
