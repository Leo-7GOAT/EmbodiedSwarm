"""Force-actuated planar MuJoCo backend; no state overwrites during stepping.

Planar x/y/yaw constraints represent an ideal ground constraint. This is a
physical contact model with generalized velocity servos, not wheel-slip or
differential-drive drivetrain validation. State/command units match RobotAdapter.
"""
from pathlib import Path
import mujoco
import numpy as np
from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.core.frames import body_to_world_velocity, world_to_body_velocity
from embodied_swarm.hal.robot_adapter import RobotAdapter
from embodied_swarm.assets import asset_path


class PhysicalMujocoAdapter(RobotAdapter):
    def __init__(self, model_path=None, command_timeout=0.25):
        path = Path(model_path) if model_path else asset_path("mjcf/physical_base.xml")
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        if not np.isfinite(command_timeout) or command_timeout <= 0:
            raise ValueError("command_timeout must be positive and finite")
        self.timeout = command_timeout
        self.running = True
        self.command = RobotCommand()
        self.interventions = 0
        mujoco.mj_forward(self.model, self.data)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.running = True
        self.command = RobotCommand()
        self.interventions = 0
        mujoco.mj_forward(self.model, self.data)
        return self.read_state()

    def read_state(self):
        vx, vy = world_to_body_velocity(*self.data.qvel[:2], self.data.qpos[2])
        return RobotState(timestamp=self.data.time, base_pose=self.data.qpos[:3],
                          base_twist=[vx, vy, self.data.qvel[2]])

    def send_command(self, command):
        if not self.running:
            raise RuntimeError("adapter is shut down")
        if abs(command.base_twist[1]) > 1e-9:
            raise ValueError("lateral base command unsupported")
        if command.joint_position_targets is not None or command.gripper_width_target is not None:
            raise ValueError("physical base backend has no arm; use PandaSimulation")
        self.command = RobotCommand(timestamp=command.timestamp, base_twist=command.base_twist)

    def step(self, dt):
        if not self.running:
            raise RuntimeError("adapter is shut down")
        n = round(float(dt) / self.model.opt.timestep) if np.isfinite(dt) else 0
        if n < 1 or not np.isclose(n*self.model.opt.timestep, dt, atol=1e-9, rtol=0):
            raise ValueError("dt must be a positive multiple of simulation timestep")
        for _ in range(n):
            age = self.data.time - self.command.timestamp
            twist = self.command.base_twist if -1e-9 <= age <= self.timeout else np.zeros(3)
            if not -1e-9 <= age <= self.timeout:
                self.interventions += 1
            x, y = body_to_world_velocity(*twist[:2], self.data.qpos[2])
            self.data.ctrl[:] = [x, y, twist[2]]
            mujoco.mj_step(self.model, self.data)
        return self.read_state()

    def shutdown(self):
        self.data.ctrl[:] = 0
        self.command = RobotCommand(timestamp=self.data.time)
        self.running = False
