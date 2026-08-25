from dataclasses import dataclass, field

import numpy as np


@dataclass
class RobotCommand:
    """
    Unified command interface for the mobile manipulator.

    Command convention:
        base_twist:
            [vx, vy, wz]

            vx, vy: m/s
            wz: rad/s

        joint_position_targets:
            [q1_target, ..., q7_target]

            unit: rad

            None means:
                no new arm position command is specified.

        gripper_width_target:
            target opening distance between gripper fingers

            unit: meter

            None means:
                no new gripper command is specified.
    """

    # Timestamp of this command [s]
    timestamp: float = 0.0

    # Mobile base velocity command:
    # [vx, vy, wz]
    base_twist: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=float)
    )

    # Panda 7-DoF joint position target:
    # [q1_target, ..., q7_target]
    #
    # None means "do not issue a new arm position command".
    joint_position_targets: np.ndarray | None = None

    # Target gripper opening width [m]
    #
    # None means "do not issue a new gripper command".
    gripper_width_target: float | None = None

    def __post_init__(self) -> None:
        """
        Normalize external inputs and validate command data.
        """

        # --------------------------------------------------------------
        # Timestamp
        # --------------------------------------------------------------
        self.timestamp = float(self.timestamp)

        if not np.isfinite(self.timestamp):
            raise ValueError(
                "timestamp must be a finite number"
            )

        # --------------------------------------------------------------
        # Mobile base velocity command
        # [vx, vy, wz]
        # --------------------------------------------------------------
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

        # --------------------------------------------------------------
        # Arm joint position targets
        # [q1_target, ..., q7_target]
        # --------------------------------------------------------------
        if self.joint_position_targets is not None:
            self.joint_position_targets = np.array(
                self.joint_position_targets,
                dtype=float,
                copy=True,
            )

            if self.joint_position_targets.shape != (7,):
                raise ValueError(
                    "joint_position_targets must have shape (7,)"
                )

            if not np.all(
                np.isfinite(self.joint_position_targets)
            ):
                raise ValueError(
                    "joint_position_targets must contain "
                    "only finite values"
                )

        # --------------------------------------------------------------
        # Gripper target
        # --------------------------------------------------------------
        if self.gripper_width_target is not None:
            self.gripper_width_target = float(
                self.gripper_width_target
            )

            if not np.isfinite(
                self.gripper_width_target
            ):
                raise ValueError(
                    "gripper_width_target must be "
                    "a finite number"
                )

            if self.gripper_width_target < 0.0:
                raise ValueError(
                    "gripper_width_target must be "
                    "non-negative"
                )