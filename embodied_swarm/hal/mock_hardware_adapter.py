import math

import numpy as np

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.hal.robot_adapter import RobotAdapter


class MockHardwareAdapter(RobotAdapter):
    """
    Lightweight deterministic robot backend.

    It simulates:
    - differential-drive mobile base
    - simple Panda joint position servo
    - ideal gripper

    No MuJoCo or real hardware is involved.
    """

    def __init__(self) -> None:
        self._state = RobotState()
        self._command = RobotCommand()
        self._running = True

        # Simple proportional gain for arm position servo
        self._arm_position_gain = 4.0

    def reset(self) -> RobotState:
        """
        Reset the mock robot to its default state.
        """
        self._state = RobotState()
        self._command = RobotCommand()
        self._running = True

        return self.read_state()

    def read_state(self) -> RobotState:
        """
        Return a snapshot of the current robot state.

        A new RobotState object is constructed so external code
        cannot modify the internal state directly.
        """
        return RobotState(
            timestamp=self._state.timestamp,
            base_pose=self._state.base_pose,
            base_twist=self._state.base_twist,
            joint_positions=self._state.joint_positions,
            joint_velocities=self._state.joint_velocities,
            gripper_width=self._state.gripper_width,
        )

    def send_command(self, command: RobotCommand) -> None:
        """
        Store the latest command.

        The command does not immediately move the robot.
        Motion occurs when step(dt) is called.
        """
        if not self._running:
            raise RuntimeError(
                "cannot send command: adapter is shut down"
            )

        if not np.isclose(command.base_twist[1], 0.0):
            raise ValueError(
                "differential-drive mock does not support lateral velocity vy"
            )

        # Store an independent command snapshot
        self._command = RobotCommand(
            timestamp=command.timestamp,
            base_twist=command.base_twist,
            joint_position_targets=command.joint_position_targets,
            gripper_width_target=command.gripper_width_target,
        )

    def step(self, dt: float) -> RobotState:
        """
        Advance the mock robot by dt seconds.
        """
        if not self._running:
            raise RuntimeError(
                "cannot step: adapter is shut down"
            )

        dt = float(dt)

        if not np.isfinite(dt):
            raise ValueError("dt must be finite")

        if dt <= 0.0:
            raise ValueError("dt must be positive")

        # ==============================================================
        # 1. Mobile base
        # ==============================================================

        x, y, yaw = self._state.base_pose
        vx, vy, wz = self._command.base_twist

        # Differential-drive kinematics:
        #
        # x_dot   = vx * cos(yaw)
        # y_dot   = vx * sin(yaw)
        # yaw_dot = wz
        #
        # Euler integration:
        #
        # x(k+1) = x(k) + x_dot * dt

        x_next = x + vx * math.cos(yaw) * dt
        y_next = y + vx * math.sin(yaw) * dt
        yaw_next = yaw + wz * dt

        self._state.base_pose = np.array(
            [
                x_next,
                y_next,
                yaw_next,
            ],
            dtype=float,
        )

        self._state.base_twist = np.array(
            [
                vx,
                0.0,
                wz,
            ],
            dtype=float,
        )

        # ==============================================================
        # 2. Panda arm
        # ==============================================================

        if self._command.joint_position_targets is None:
            # No new arm command:
            # hold current joint positions.
            self._state.joint_velocities = np.zeros(
                7,
                dtype=float,
            )

        else:
            q = self._state.joint_positions
            q_target = self._command.joint_position_targets

            # Position error
            error = q_target - q

            # Simple first-order P servo:
            #
            # dq = K * (q_target - q)
            #
            # To prevent the simple mock from overshooting when dt
            # becomes large, limit the interpolation factor to 1.
            alpha = min(
                self._arm_position_gain * dt,
                1.0,
            )

            q_next = q + alpha * error

            # Estimate joint velocity over this time step
            dq = (q_next - q) / dt

            self._state.joint_positions = q_next
            self._state.joint_velocities = dq

        # ==============================================================
        # 3. Gripper
        # ==============================================================

        if self._command.gripper_width_target is not None:
            # W02 simplification:
            # ideal gripper reaches target instantly.
            self._state.gripper_width = (
                self._command.gripper_width_target
            )

        # ==============================================================
        # 4. Time
        # ==============================================================

        self._state.timestamp += dt

        return self.read_state()

    def shutdown(self) -> None:
        """
        Stop the backend.

        Future step() and send_command() calls will be rejected.
        """
        self._running = False

        # Store a safe zero-base command.
        self._command = RobotCommand(
            timestamp=self._state.timestamp,
            base_twist=np.zeros(3, dtype=float),
        )
