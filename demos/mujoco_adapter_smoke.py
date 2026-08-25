from pathlib import Path

import numpy as np

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.hal.mujoco_robot_adapter import (
    MujocoRobotAdapter,
)


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "mjcf"
    / "mobile_manipulator.xml"
)


def main() -> None:

    adapter = MujocoRobotAdapter(
        MODEL_PATH
    )

    try:

        # ==========================================================
        # Initial state
        # ==========================================================

        print()
        print("=== Initial State ===")

        state = adapter.read_state()

        print(
            "time:",
            state.timestamp,
        )

        print(
            "pose:",
            state.base_pose,
        )

        print(
            "twist:",
            state.base_twist,
        )

        print(
            "arm:",
            state.joint_positions,
        )

        # ==========================================================
        # Forward
        # ==========================================================

        print()
        print(
            "=== Forward 1 m/s for 1 second ==="
        )

        adapter.send_command(
            RobotCommand(
                base_twist=[
                    1.0,
                    0.0,
                    0.0,
                ]
            )
        )

        state = adapter.step(
            1.0
        )

        print(
            "time:",
            state.timestamp,
        )

        print(
            "pose:",
            state.base_pose,
        )

        print(
            "twist:",
            state.base_twist,
        )

        # ==========================================================
        # Rotate
        # ==========================================================

        print()
        print("=== Rotate 90 deg ===")

        adapter.send_command(
            RobotCommand(
                base_twist=[
                    0.0,
                    0.0,
                    np.pi / 2,
                ]
            )
        )

        state = adapter.step(
            1.0
        )

        print(
            "time:",
            state.timestamp,
        )

        print(
            "pose:",
            state.base_pose,
        )

        print(
            "twist:",
            state.base_twist,
        )

        # ==========================================================
        # Panda
        # ==========================================================

        print()
        print("=== Panda Joint Command ===")

        target = np.zeros(
            7,
            dtype=float,
        )

        target[1] = 0.4
        target[3] = -0.3

        adapter.send_command(
            RobotCommand(
                base_twist=[
                    0.0,
                    0.0,
                    0.0,
                ],
                joint_position_targets=target,
            )
        )

        state = adapter.step(
            1.0
        )

        print(
            "target:",
            target,
        )

        print(
            "joint positions:",
            state.joint_positions,
        )

        print(
            "joint velocities:",
            state.joint_velocities,
        )

        print()
        print(
            "MUJOCO ADAPTER SMOKE PASS"
        )

    finally:

        # Only shut down after ALL tests finish.
        adapter.shutdown()


if __name__ == "__main__":
    main()