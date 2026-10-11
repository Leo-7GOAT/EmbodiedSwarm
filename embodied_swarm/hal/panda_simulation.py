"""Menagerie Franka Panda physical contact simulation, SI units."""
from pathlib import Path
import mujoco
import numpy as np
from embodied_swarm.models.kinematics import SerialKinematics
from embodied_swarm.assets import asset_path


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
        mujoco.mj_forward(self.model, self.data)

    def joint_positions(self):
        return self.data.qpos[self.arm_qpos].copy()

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
        q = np.asarray(q, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all() or np.any(q < self.kinematics.limits[:, 0]) or np.any(q > self.kinematics.limits[:, 1]):
            raise ValueError("invalid or out-of-limit Panda joint target")
        if not 0 <= gripper_width <= .08:
            raise ValueError("gripper width must be within [0, .08] m")
        self.data.ctrl[self.arm_actuators] = q
        self.data.ctrl[self.gripper_actuator] = gripper_width/.08*255
        mujoco.mj_step(self.model, self.data)
        self.control_ns.append(perf_counter_ns()-before)
        self.joint_errors.append((self.joint_positions()-q).copy())
        if len(self.control_ns) % 25 == 0:
            contacts, force = self.contact_info()
            self.records.append(dict(time=float(self.data.time), stage=stage,
                                     q=self.joint_positions().tolist(), ee=self.data.site_xpos[self.kinematics.site].tolist(),
                                     object=self.object_position().tolist(), finger_contacts=contacts, contact_force=force))
            if frame_callback:
                frame_callback(self)

    def move(self, target, width=.08, duration=2., stage="MOVE", frame_callback=None):
        target = np.asarray(target)
        start = self.joint_positions()
        n = int(duration/self.model.opt.timestep)
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
