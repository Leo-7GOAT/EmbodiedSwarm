from pathlib import Path

import numpy as np

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.hal.mock_hardware_adapter import (
    MockHardwareAdapter,
)
from embodied_swarm.hal.mujoco_robot_adapter import (
    MujocoRobotAdapter,
)
from embodied_swarm.hal.robot_adapter import (
    RobotAdapter,
)


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "mjcf"
    / "mobile_manipulator.xml"
)


def run_backend(
    name: str,
    adapter: RobotAdapter,
) -> None:

    print()
    print("=" * 60)
    print(name)
    print("=" * 60)

    try:

        state = adapter.reset()

        print(
            "initial pose:",
            state.base_pose,
        )

        print(
            "initial arm:",
            state.joint_positions,
        )

        # ----------------------------------------------------------
        # Same command for BOTH backends.
        # ----------------------------------------------------------

        arm_target = np.zeros(
            7,
            dtype=float,
        )

        arm_target[1] = 0.2

        command = RobotCommand(
            base_twist=[
                0.5,
                0.0,
                0.2,
            ],
            joint_position_targets=arm_target,
            gripper_width_target=0.03,
        )

        adapter.send_command(
            command
        )

        state = adapter.step(
            0.2
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

        # ----------------------------------------------------------
        # Contract checks
        # ----------------------------------------------------------

        assert state.base_pose.shape == (
            3,
        )

        assert state.base_twist.shape == (
            3,
        )

        assert state.joint_positions.shape == (
            7,
        )

        assert state.joint_velocities.shape == (
            7,
        )

        assert state.gripper_width == 0.03

        assert np.all(
            np.isfinite(
                state.base_pose
            )
        )

        assert np.all(
            np.isfinite(
                state.joint_positions
            )
        )

        print(
            f"PASS: {name}"
        )

    finally:

        adapter.shutdown()


def main() -> None:

    run_backend(
        "MockHardwareAdapter",
        MockHardwareAdapter(),
    )

    run_backend(
        "MujocoRobotAdapter",
        MujocoRobotAdapter(
            MODEL_PATH
        ),
    )

    print()
    print("=" * 60)
    print(
        "ADAPTER SWITCH CONTRACT PASS"
    )
    print("=" * 60)


if __name__ == "__main__":
    main()
