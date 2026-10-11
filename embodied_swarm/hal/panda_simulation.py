"""Menagerie Franka Panda physical contact simulation, SI units."""
from pathlib import Path
import mujoco
import numpy as np
from embodied_swarm.models.kinematics import SerialKinematics
from embodied_swarm.assets import asset_path
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.safety import SafetySupervisor,SafetyLimits
from embodied_swarm.core.frames import body_to_world_velocity


class PandaSimulation:
    HOME = np.array([0., 0., 0., -1.57079, 0., 1.57079, -0.7853])

    def __init__(self, model_path=None):
        path = Path(model_path) if model_path else asset_path("mjcf/panda_pick_place.xml")
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.kinematics = SerialKinematics(self.model)
        self.arm_qpos = self.kinematics.qpos
        self.arm_dofs = self.kinematics.dofs
        self.arm_actuators = np.array([self.model.actuator(f"actuator{i}").id for i in range(1,8)])
        self.gripper_actuator = self.model.actuator("actuator8").id
        self.finger_qpos = [self.model.jnt_qposadr[self.model.joint(f"finger_joint{i}").id] for i in (1,2)]
        self.data.qpos[self.arm_qpos] = self.HOME
        self.data.qpos[self.finger_qpos] = .04
        self.data.ctrl[self.arm_actuators] = self.HOME
        self.data.ctrl[self.gripper_actuator] = 255
        self.object_id = self.model.body("object").id
        self.object_geom = self.model.geom("object_geom").id
        self.records = []
        self.control_ns = []
        self.joint_errors = []
        self.max_contact_force = 0.
        self.safety = SafetySupervisor(SafetyLimits(control_period=float(self.model.opt.timestep)),
                                       joint_limits=self.kinematics.limits)
        self.deadline_sim = None
        mujoco.mj_forward(self.model, self.data)

    def joint_positions(self):
        return self.data.qpos[self.arm_qpos].copy()

    def read_state(self):
        return RobotState(timestamp=self.data.time,joint_positions=self.joint_positions(),
                          joint_velocities=self.data.qvel[self.arm_dofs],
                          gripper_width=float(np.sum(self.data.qpos[self.finger_qpos])))

    def halt(self,reason="software_estop"):
        self.safety.emergency_stop(reason)
        self.deadline_sim = None
        if hasattr(self,"base_twist"):
            self.base_twist[:] = 0
            self.data.ctrl[self.base_actuators] = 0
        self.data.ctrl[self.arm_actuators] = self.joint_positions()
        self.data.qfrc_applied[self.arm_dofs] = self.data.qfrc_bias[self.arm_dofs]

    def object_position(self):
        return self.data.xpos[self.object_id].copy()

    def contact_info(self):
        finger_contacts, force = 0, 0.
        for c in range(self.data.ncon):
            contact = self.data.contact[c]
            if self.object_geom not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == self.object_geom else contact.geom1
            name = self.model.body(int(self.model.geom_bodyid[other])).name
            if "finger" in name:
                finger_contacts += 1
                wrench = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, c, wrench)
                force += abs(wrench[0])
        self.max_contact_force = max(self.max_contact_force, force)
        return finger_contacts, force

    def step(self, q, gripper_width, stage, frame_callback=None):
        from time import perf_counter_ns
        before = perf_counter_ns()
        if self.deadline_sim is not None and self.data.time >= self.deadline_sim:
            self.halt("task_timeout")
            raise RuntimeError("simulation task deadline expired")
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all() or np.any(q < self.kinematics.limits[:, 0]) or np.any(q > self.kinematics.limits[:, 1]):
            raise ValueError("invalid or out-of-limit Panda joint target")
        if not 0 <= gripper_width <= .08:
            raise ValueError("gripper width must be within [0, .08] m")
        state = self.read_state()
        command = RobotCommand(timestamp=self.data.time,base_twist=getattr(self,"base_twist",np.zeros(3)),
                               joint_position_targets=q,gripper_width_target=gripper_width)
        safe = self.safety.filter(state,command,self.data.time)
        self.data.ctrl[self.arm_actuators] = safe.joint_position_targets
        if safe.gripper_width_target is not None:
            self.data.ctrl[self.gripper_actuator] = safe.gripper_width_target/.08*255
        if hasattr(self,"base_twist"):
            self.base_twist = safe.base_twist.copy()
            vx,vy = body_to_world_velocity(*safe.base_twist[:2],state.base_pose[2])
            self.data.ctrl[self.base_actuators] = [vx,vy,safe.base_twist[2]]
        # Arm-only model bias compensation; contacts/object remain physical.
        self.data.qfrc_applied[self.arm_dofs] = self.data.qfrc_bias[self.arm_dofs]
        mujoco.mj_step(self.model, self.data)
        self.control_ns.append(perf_counter_ns()-before)
        self.joint_errors.append((self.joint_positions()-safe.joint_position_targets).copy())
        if len(self.control_ns) % 25 == 0:
            contacts, force = self.contact_info()
            self.records.append(dict(time=float(self.data.time), stage=stage,
                                     q=self.joint_positions().tolist(), ee=self.data.site_xpos[self.kinematics.site].tolist(),
                                     object=self.object_position().tolist(), finger_contacts=contacts, contact_force=force))
            if frame_callback:
                frame_callback(self)

    def move(self, target, width=.08, duration=2., stage="MOVE", frame_callback=None):
        target = np.asarray(target,dtype=float)
        if target.shape != (7,) or not np.isfinite(target).all() or not np.isfinite(duration) or duration <= 0:
            raise ValueError("finite seven-joint target and positive duration required")
        n = round(duration/self.model.opt.timestep)
        if n < 1 or not np.isclose(n*self.model.opt.timestep,duration,atol=1e-9,rtol=0):
            raise ValueError("duration must be a positive multiple of simulation timestep")
        start = self.joint_positions()
        for i in range(n):
            s = min(1., (i+1)/(n*.7))
            s = s*s*(3-2*s)
            self.step(start + s*(target-start), width, stage, frame_callback)

    def cartesian_target(self, position, rotation=None):
        rotation = np.diag([1., -1., -1.]) if rotation is None else rotation
        result = self.kinematics.ik(position, self.joint_positions(), rotation)
        if not result.success:
            raise RuntimeError(f"IK failed: {result.position_error:.4f} m")
        return result.q
