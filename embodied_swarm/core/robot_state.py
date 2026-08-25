from dataclasses import dataclass, field

import numpy as np


@dataclass
class RobotState:
    """
    Unified robot state for the mobile manipulator.

    Coordinate / state convention:
        base_pose:
            [x, y, yaw]
            x, y: meter
            yaw: rad

        base_twist:
            [vx, vy, wz]
            vx, vy: m/s
            wz: rad/s

        joint_positions:
            [q1, ..., q7]
            unit: rad

        joint_velocities:
            [dq1, ..., dq7]
            unit: rad/s

        gripper_width:
            distance between gripper fingers
            unit: meter
    """

    # Timestamp of this state [s]
    timestamp: float = 0.0

    # Mobile base pose: [x, y, yaw]
    base_pose: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=float)
    )

    # Mobile base velocity: [vx, vy, wz]
    base_twist: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=float)
    )

    # Franka Panda joint positions: [q1, ..., q7]
    joint_positions: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=float)
    )

    # Franka Panda joint velocities: [dq1, ..., dq7]
    joint_velocities: np.ndarray = field(
        default_factory=lambda: np.zeros(7, dtype=float)
    )

    # Gripper opening width [m]
    gripper_width: float = 0.0

    def __post_init__(self) -> None:
        """
        Normalize external inputs and validate state data.

        Reasonable external representations such as Python lists are accepted,
        but all internal vector states are stored as independent float
        NumPy arrays.
        """

        # ------------------------------------------------------------------
        # Timestamp
        # ------------------------------------------------------------------
        self.timestamp = float(self.timestamp)

        if not np.isfinite(self.timestamp):
            raise ValueError(
                "timestamp must be a finite number"
            )

        # ------------------------------------------------------------------
        # Base pose
        # [x, y, yaw]
        # ------------------------------------------------------------------
        self.base_pose = np.array(
            self.base_pose,
            dtype=float,
            copy=True,
        )

        if self.base_pose.shape != (3,):
            raise ValueError(
                "base_pose must have shape (3,) [x, y, yaw]"
            )

        if not np.all(np.isfinite(self.base_pose)):
            raise ValueError(
                "base_pose must contain only finite values"
            )

        # ------------------------------------------------------------------
        # Base twist
        # [vx, vy, wz]
        # ------------------------------------------------------------------
        self.base_twist = np.array(
            self.base_twist,
            dtype=float,
            copy=True,
        )

        if self.base_twist.shape != (3,):
            raise ValueError(
                "base_twist must have shape (3,) [vx, vy, wz]"
            )

        if not np.all(np.isfinite(self.base_twist)):
            raise ValueError(
                "base_twist must contain only finite values"
            )

        # ------------------------------------------------------------------
        # Arm joint positions
        # [q1, ..., q7]
        # ------------------------------------------------------------------
        self.joint_positions = np.array(
            self.joint_positions,
            dtype=float,
            copy=True,
        )

        if self.joint_positions.shape != (7,):
            raise ValueError(
                "joint_positions must have shape (7,)"
            )

        if not np.all(np.isfinite(self.joint_positions)):
            raise ValueError(
                "joint_positions must contain only finite values"
            )

        # ------------------------------------------------------------------
        # Arm joint velocities
        # [dq1, ..., dq7]
        # ------------------------------------------------------------------
        self.joint_velocities = np.array(
            self.joint_velocities,
            dtype=float,
            copy=True,
        )

        if self.joint_velocities.shape != (7,):
            raise ValueError(
                "joint_velocities must have shape (7,)"
            )

        if not np.all(np.isfinite(self.joint_velocities)):
            raise ValueError(
                "joint_velocities must contain only finite values"
            )

        # ------------------------------------------------------------------
        # Gripper
        # ------------------------------------------------------------------
        self.gripper_width = float(self.gripper_width)

        if not np.isfinite(self.gripper_width):
            raise ValueError(
                "gripper_width must be a finite number"
            )

        if self.gripper_width < 0.0:
            raise ValueError(
                "gripper_width must be non-negative"
            )