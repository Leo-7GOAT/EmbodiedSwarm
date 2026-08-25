from pathlib import Path

import mujoco
import numpy as np

from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.frames import (
    body_to_world_velocity,
    world_to_body_velocity,
)
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.hal.robot_adapter import RobotAdapter


class MujocoRobotAdapter(RobotAdapter):
    """
    MuJoCo backend for EmbodiedSwarm W02.

    Base:
        base_x
        base_y
        base_yaw

    Arm:
        panda_joint1 ... panda_joint7

    Command convention:
        base_twist = [vx_body, vy_body, wz]

    State convention:
        base_pose = [x_world, y_world, yaw]
        base_twist = [vx_body, vy_body, wz]
    """

    ARM_JOINT_NAMES = (
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    )

    ARM_ACTUATOR_NAMES = (
        "panda_joint1_position",
        "panda_joint2_position",
        "panda_joint3_position",
        "panda_joint4_position",
        "panda_joint5_position",
        "panda_joint6_position",
        "panda_joint7_position",
    )

    def __init__(
        self,
        model_path: str | Path,
    ) -> None:

        self._model_path = Path(model_path)

        if not self._model_path.exists():
            raise FileNotFoundError(
                f"MuJoCo model not found: {self._model_path}"
            )

        # Python reads the Unicode Windows path.
        xml_text = self._model_path.read_text(
            encoding="utf-8"
        )

        self._model = mujoco.MjModel.from_xml_string(
            xml_text
        )

        self._data = mujoco.MjData(
            self._model
        )

        self._running = True
        self._command = RobotCommand()
        self._gripper_width = 0.0
        self._gripper_target: float | None = None

        # ----------------------------------------------------------
        # Base addresses
        # ----------------------------------------------------------

        self._base_x_qpos_addr = self._get_qpos_addr(
            "base_x"
        )
        self._base_y_qpos_addr = self._get_qpos_addr(
            "base_y"
        )
        self._base_yaw_qpos_addr = self._get_qpos_addr(
            "base_yaw"
        )

        self._base_x_dof_addr = self._get_dof_addr(
            "base_x"
        )
        self._base_y_dof_addr = self._get_dof_addr(
            "base_y"
        )
        self._base_yaw_dof_addr = self._get_dof_addr(
            "base_yaw"
        )

        # ----------------------------------------------------------
        # Arm addresses
        # ----------------------------------------------------------

        self._arm_qpos_addrs = np.array(
            [
                self._get_qpos_addr(name)
                for name in self.ARM_JOINT_NAMES
            ],
            dtype=int,
        )

        self._arm_dof_addrs = np.array(
            [
                self._get_dof_addr(name)
                for name in self.ARM_JOINT_NAMES
            ],
            dtype=int,
        )

        self._arm_actuator_ids = np.array(
            [
                self._get_actuator_id(name)
                for name in self.ARM_ACTUATOR_NAMES
            ],
            dtype=int,
        )

        # Hold initial arm configuration.
        self._arm_target = self._data.qpos[
            self._arm_qpos_addrs
        ].copy()

        self._apply_arm_command()

        mujoco.mj_forward(
            self._model,
            self._data,
        )

    # ==============================================================
    # Lookup helpers
    # ==============================================================

    def _get_joint_id(
        self,
        joint_name: str,
    ) -> int:

        joint_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )

        if joint_id < 0:
            raise ValueError(
                f"joint not found: {joint_name}"
            )

        return int(joint_id)

    def _get_qpos_addr(
        self,
        joint_name: str,
    ) -> int:

        joint_id = self._get_joint_id(
            joint_name
        )

        return int(
            self._model.jnt_qposadr[
                joint_id
            ]
        )

    def _get_dof_addr(
        self,
        joint_name: str,
    ) -> int:

        joint_id = self._get_joint_id(
            joint_name
        )

        return int(
            self._model.jnt_dofadr[
                joint_id
            ]
        )

    def _get_actuator_id(
        self,
        actuator_name: str,
    ) -> int:

        actuator_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            actuator_name,
        )

        if actuator_id < 0:
            raise ValueError(
                f"actuator not found: {actuator_name}"
            )

        return int(actuator_id)

    # ==============================================================
    # Internal command application
    # ==============================================================

    def _apply_arm_command(self) -> None:

        self._data.ctrl[
            self._arm_actuator_ids
        ] = self._arm_target

    # ==============================================================
    # RobotAdapter interface
    # ==============================================================

    def reset(self) -> RobotState:

        mujoco.mj_resetData(
            self._model,
            self._data,
        )

        self._command = RobotCommand()
        self._gripper_width = 0.0
        self._gripper_target = None

        self._running = True

        self._arm_target = self._data.qpos[
            self._arm_qpos_addrs
        ].copy()

        self._apply_arm_command()

        mujoco.mj_forward(
            self._model,
            self._data,
        )

        return self.read_state()

    def read_state(self) -> RobotState:

        # ----------------------------------------------------------
        # Base pose
        # ----------------------------------------------------------

        x = float(
            self._data.qpos[
                self._base_x_qpos_addr
            ]
        )

        y = float(
            self._data.qpos[
                self._base_y_qpos_addr
            ]
        )

        yaw = float(
            self._data.qpos[
                self._base_yaw_qpos_addr
            ]
        )

        # ----------------------------------------------------------
        # Base velocity in MuJoCo world coordinates
        # ----------------------------------------------------------

        x_dot_world = float(
            self._data.qvel[
                self._base_x_dof_addr
            ]
        )

        y_dot_world = float(
            self._data.qvel[
                self._base_y_dof_addr
            ]
        )

        wz = float(
            self._data.qvel[
                self._base_yaw_dof_addr
            ]
        )

        vx_body, vy_body = (
            world_to_body_velocity(
                x_dot_world,
                y_dot_world,
                yaw,
            )
        )

        # ----------------------------------------------------------
        # Panda
        # ----------------------------------------------------------

        joint_positions = self._data.qpos[
            self._arm_qpos_addrs
        ].copy()

        joint_velocities = self._data.qvel[
            self._arm_dof_addrs
        ].copy()

        return RobotState(
            timestamp=float(
                self._data.time
            ),

            base_pose=np.array(
                [
                    x,
                    y,
                    yaw,
                ],
                dtype=float,
            ),

            base_twist=np.array(
                [
                    vx_body,
                    vy_body,
                    wz,
                ],
                dtype=float,
            ),

            joint_positions=joint_positions,

            joint_velocities=joint_velocities,

            gripper_width=self._gripper_width,
        )

    def send_command(
        self,
        command: RobotCommand,
    ) -> None:

        if not self._running:
            raise RuntimeError(
                "cannot send command: "
                "MuJoCo adapter is shut down"
            )

        # Current base is differential-drive.
        if not np.isclose(
            command.base_twist[1],
            0.0,
        ):
            raise ValueError(
                "differential-drive base does not "
                "support lateral velocity vy"
            )

        if command.joint_position_targets is not None:
            self._arm_target = (
                command.joint_position_targets.copy()
            )

        if command.gripper_width_target is not None:
            self._gripper_target = command.gripper_width_target

        self._command = RobotCommand(
            timestamp=command.timestamp,
            base_twist=command.base_twist,
            joint_position_targets=(
                command.joint_position_targets
            ),
            gripper_width_target=command.gripper_width_target,
        )

    def step(
        self,
        dt: float,
    ) -> RobotState:

        if not self._running:
            raise RuntimeError(
                "cannot step: MuJoCo adapter is shut down"
            )

        dt = float(dt)

        if not np.isfinite(dt):
            raise ValueError(
                "dt must be finite"
            )

        if dt <= 0.0:
            raise ValueError(
                "dt must be positive"
            )

        sim_dt = float(
            self._model.opt.timestep
        )

        num_steps = int(
            round(dt / sim_dt)
        )

        if (
            num_steps < 1
            or not np.isclose(
                num_steps * sim_dt,
                dt,
                atol=1e-9,
                rtol=0.0,
            )
        ):
            raise ValueError(
                f"dt={dt} must be a positive multiple "
                f"of MuJoCo timestep={sim_dt}"
            )

        for _ in range(num_steps):

            # ------------------------------------------------------
            # Remember deterministic base result for this substep.
            # ------------------------------------------------------

            x = float(
                self._data.qpos[
                    self._base_x_qpos_addr
                ]
            )

            y = float(
                self._data.qpos[
                    self._base_y_qpos_addr
                ]
            )

            yaw = float(
                self._data.qpos[
                    self._base_yaw_qpos_addr
                ]
            )

            vx_body, vy_body, wz = (
                self._command.base_twist
            )

            x_dot_world, y_dot_world = (
                body_to_world_velocity(
                    vx_body,
                    vy_body,
                    yaw,
                )
            )

            expected_x = (
                x
                + x_dot_world * sim_dt
            )

            expected_y = (
                y
                + y_dot_world * sim_dt
            )

            expected_yaw = (
                yaw
                + wz * sim_dt
            )

            # Give MuJoCo the base velocity for this step.
            self._data.qvel[
                self._base_x_dof_addr
            ] = x_dot_world

            self._data.qvel[
                self._base_y_dof_addr
            ] = y_dot_world

            self._data.qvel[
                self._base_yaw_dof_addr
            ] = wz

            # Arm goes through actual MuJoCo actuator dynamics.
            self._apply_arm_command()

            mujoco.mj_step(
                self._model,
                self._data,
            )

            # W02 base is intentionally kinematic.
            # Remove arm-reaction disturbance from planar pose.
            self._data.qpos[
                self._base_x_qpos_addr
            ] = expected_x

            self._data.qpos[
                self._base_y_qpos_addr
            ] = expected_y

            self._data.qpos[
                self._base_yaw_qpos_addr
            ] = expected_yaw

            # qvel represents the instantaneous velocity at the new pose.
            # Rotate the persistent body command using the updated yaw so a
            # readback transform does not invent lateral body velocity.
            x_dot_world_next, y_dot_world_next = body_to_world_velocity(
                vx_body,
                vy_body,
                expected_yaw,
            )

            self._data.qvel[
                self._base_x_dof_addr
            ] = x_dot_world_next

            self._data.qvel[
                self._base_y_dof_addr
            ] = y_dot_world_next

            self._data.qvel[
                self._base_yaw_dof_addr
            ] = wz

            mujoco.mj_forward(
                self._model,
                self._data,
            )

        # W02 exposes an ideal logical gripper channel. The MJCF intentionally
        # has no finger joints or contact physics yet.
        if self._gripper_target is not None:
            self._gripper_width = self._gripper_target

        return self.read_state()

    def shutdown(self) -> None:

        if not self._running:
            return

        self._data.qvel[
            self._base_x_dof_addr
        ] = 0.0

        self._data.qvel[
            self._base_y_dof_addr
        ] = 0.0

        self._data.qvel[
            self._base_yaw_dof_addr
        ] = 0.0

        # Hold current arm pose safely.
        self._arm_target = self._data.qpos[
            self._arm_qpos_addrs
        ].copy()

        self._apply_arm_command()

        self._command = RobotCommand(
            timestamp=float(
                self._data.time
            )
        )

        mujoco.mj_forward(
            self._model,
            self._data,
        )

        self._running = False
