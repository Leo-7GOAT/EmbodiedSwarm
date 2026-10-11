"""One MuJoCo world for planar force-actuated navigation and Panda contact tasks."""
import numpy as np
import mujoco
from embodied_swarm.assets import asset_path
from embodied_swarm.hal.panda_simulation import PandaSimulation
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.core.frames import body_to_world_velocity, world_to_body_velocity
from embodied_swarm.models.kinematics import rotation


class MobilePandaSimulation(PandaSimulation):
    def __init__(self):
        super().__init__(asset_path("mjcf/mobile_panda.xml"))
        self.base_qpos = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in ("base_x","base_y","base_yaw")])
        self.base_dofs = np.array([self.model.jnt_dofadr[self.model.joint(n).id] for n in ("base_x","base_y","base_yaw")])
        self.base_actuators = np.array([self.model.actuator(n+"_velocity").id for n in ("base_x","base_y","base_yaw")])
        self.base_twist = np.zeros(3)

    def read_state(self):
        pose = self.data.qpos[self.base_qpos]
        velocity = self.data.qvel[self.base_dofs]
        vx,vy = world_to_body_velocity(*velocity[:2],pose[2])
        return RobotState(timestamp=self.data.time,base_pose=pose,base_twist=[vx,vy,velocity[2]],
                          joint_positions=self.joint_positions(),joint_velocities=self.data.qvel[self.arm_dofs],
                          gripper_width=float(np.sum(self.data.qpos[self.finger_qpos])))

    def step(self,q,gripper_width,stage,frame_callback=None):
        yaw = self.data.qpos[self.base_qpos[2]]
        vx,vy = body_to_world_velocity(*self.base_twist[:2],yaw)
        self.data.ctrl[self.base_actuators] = [vx,vy,self.base_twist[2]]
        super().step(q,gripper_width,stage,frame_callback)

    def cartesian_target(self,position,rotation_matrix=None):
        base = self.data.qpos[self.base_qpos]
        R = rotation([0,0,1],base[2])
        position = R.T@(np.asarray(position)-np.r_[base[:2],0.])
        rot = np.diag([1.,-1.,-1.]) if rotation_matrix is None else rotation_matrix
        return super().cartesian_target(position,R.T@rot)
