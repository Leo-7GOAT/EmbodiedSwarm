"""Small physical contact environment for a bounded residual position policy.

The classical controller is full 6D Cartesian torque impedance. Learning adds
at most 3 cm to its vertical target, never replaces joint limits or saturation.
"""
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_swarm.hal.panda_simulation import PandaSimulation
from embodied_swarm.control.impedance import cartesian_impedance_torque


class ResidualContactEnv:
    def __init__(self,seed=7,stiffness=300.):
        if not np.isfinite(stiffness) or stiffness<=0:
            raise ValueError("stiffness must be positive and finite")
        self.sim = PandaSimulation()
        self.rng = np.random.default_rng(seed)
        self.stiffness = stiffness
        self.ids = self.sim.arm_actuators
        self.limits = np.array([87.,87.,87.,87.,12.,12.,12.])
        self.initial_q = self.sim.cartesian_target([.5,.2,.04])
        m = self.sim.model
        m.actuator_gainprm[self.ids,:] = 0
        m.actuator_gainprm[self.ids,0] = 1
        m.actuator_biasprm[self.ids,:] = 0
        m.actuator_ctrlrange[self.ids,:] = np.column_stack([-self.limits,self.limits])
        m.actuator_forcerange[self.ids,:] = np.column_stack([-self.limits,self.limits])
        self.original_damping = m.dof_damping[self.sim.arm_dofs].copy()
        self.force = 0.
        self.steps = 0
        self.target_z = .04

    def reset(self):
        s = self.sim
        mujoco.mj_resetData(s.model,s.data)
        s.model.dof_damping[s.arm_dofs] = self.original_damping*self.rng.uniform(.8,1.2)
        s.data.qpos[s.arm_qpos] = self.initial_q
        s.data.qpos[s.finger_qpos] = .04
        s.data.ctrl[s.gripper_actuator] = 255
        mujoco.mj_forward(s.model,s.data)
        self.force,self.steps = 0.,0
        self.target_z = .04
        # Establish contact through the same physical torque loop; initialization
        # never teleports to a penetrated posture or bypasses contact transients.
        for _ in range(140):
            self.step(0.)
        self.steps=0
        return self.observe()

    def observe(self):
        s = self.sim
        return np.array([(self.force-2.)/8.,(s.data.site_xpos[s.kinematics.site,2]-.01)/.05,
                         s.data.qvel[s.arm_dofs[1]]],dtype=np.float32)

    def step(self,raw_action):
        if not np.isfinite(raw_action):
            raise ValueError("residual action must be finite")
        s = self.sim
        residual = .03*np.tanh(float(raw_action))
        desired_z = -.02+residual
        force_samples = []
        for _ in range(10):
            self.target_z += np.clip(desired_z-self.target_z,-.00008,.00008)
            goal=np.array([.5,.2,self.target_z])
            site=s.kinematics.site
            R=s.data.site_xmat[site].reshape(3,3)
            jp,jr=np.zeros((3,s.model.nv)),np.zeros((3,s.model.nv))
            mujoco.mj_jacSite(s.model,s.data,jp,jr,site)
            J=np.vstack([jp[:,s.arm_dofs],jr[:,s.arm_dofs]])
            e=np.r_[goal-s.data.site_xpos[site],Rotation.from_matrix(np.diag([1.,-1.,-1.])@R.T).as_rotvec()]
            tau=cartesian_impedance_torque(J,e,s.data.qvel[s.arm_dofs],s.data.qfrc_bias[s.arm_dofs],
                                           self.stiffness,2*np.sqrt(self.stiffness),self.limits)
            s.data.ctrl[self.ids]=tau
            mujoco.mj_step(s.model,s.data)
            force=0.
            for c in range(s.data.ncon):
                con=s.data.contact[c]
                if s.model.geom("floor").id not in (con.geom1,con.geom2):
                    continue
                names=[s.model.body(int(s.model.geom_bodyid[g])).name for g in (con.geom1,con.geom2)]
                if any("finger" in n for n in names):
                    wrench=np.zeros(6)
                    mujoco.mj_contactForce(s.model,s.data,c,wrench)
                    force+=abs(wrench[0])
            force_samples.append(force)
        self.force=float(np.mean(force_samples))
        self.steps+=1
        unsafe=not np.isfinite(s.data.qpos).all() or self.force>25.
        reward=-((self.force-2.)/8.)**2-.002*(residual/.03)**2
        if unsafe:
            reward-=10.
        return self.observe(),reward,self.steps>=64 or unsafe,{"force_N":self.force,"residual_m":residual,"unsafe":unsafe}
