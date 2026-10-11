"""Torque impedance and optional force admittance against the physical floor."""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_swarm.hal.panda_simulation import PandaSimulation
from embodied_swarm.control.impedance import cartesian_impedance_torque


def run(seed=7, stiffness=300., admittance=False):
    sim = PandaSimulation()
    sim.move(sim.cartesian_target([.5,.2,.12]),.08,2.,"APPROACH")
    ids = sim.arm_actuators
    limits = np.array([87.,87.,87.,87.,12.,12.,12.])
    # Reconfigure real MuJoCo affine position actuators as torque motors.
    sim.model.actuator_gainprm[ids,:] = 0
    sim.model.actuator_gainprm[ids,0] = 1
    sim.model.actuator_biasprm[ids,:] = 0
    sim.model.actuator_ctrlrange[ids,:] = np.column_stack([-limits,limits])
    sim.model.actuator_forcerange[ids,:] = np.column_stack([-limits,limits])
    rng = np.random.default_rng(seed)
    sim.model.dof_damping[sim.arm_dofs] *= rng.uniform(.8,1.2)
    goal = np.array([.5,.2,-.02])
    target_rotation = np.diag([1.,-1.,-1.])
    site = sim.kinematics.site
    records, previous_force = [],0.
    for i in range(3000):
        p = sim.data.site_xpos[site]
        R = sim.data.site_xmat[site].reshape(3,3)
        # Ramp target to avoid an impulsive initial contact.
        target = goal.copy()
        target[2] = .12 + min(1.,i/700.)*(goal[2]-.12)
        if admittance and i > 700:
            goal[2] = np.clip(goal[2]+.00001*(previous_force-2.),-.05,.04)
        jp,jr = np.zeros((3,sim.model.nv)),np.zeros((3,sim.model.nv))
        mujoco.mj_jacSite(sim.model,sim.data,jp,jr,site)
        J = np.vstack([jp[:,sim.arm_dofs],jr[:,sim.arm_dofs]])
        error = np.r_[target-p,Rotation.from_matrix(target_rotation@R.T).as_rotvec()]
        tau = cartesian_impedance_torque(J,error,sim.data.qvel[sim.arm_dofs],
                                        sim.data.qfrc_bias[sim.arm_dofs],stiffness,2*np.sqrt(stiffness),limits)
        sim.data.ctrl[ids] = tau
        sim.data.qfrc_applied[:] = 0
        if 1600 <= i < 1650:
            sim.data.qfrc_applied[sim.arm_dofs[1]] = 2.  # real torque disturbance
        mujoco.mj_step(sim.model,sim.data)
        force,contacts = 0.,0
        for c in range(sim.data.ncon):
            con = sim.data.contact[c]
            names = [sim.model.body(int(sim.model.geom_bodyid[g])).name for g in (con.geom1,con.geom2)]
            if any("finger" in n for n in names) and sim.model.geom("floor").id in (con.geom1,con.geom2):
                wrench = np.zeros(6)
                mujoco.mj_contactForce(sim.model,sim.data,c,wrench)
                force += abs(wrench[0]);contacts += 1
        previous_force = force
        if i % 10 == 0:
            records.append(dict(time=float(sim.data.time),position=sim.data.site_xpos[site].tolist(),
                                target=target.tolist(),force_N=force,contacts=contacts,
                                torque=tau.tolist(),disturbed=1600<=i<1650))
    steady = records[-80:]
    forces = np.array([r["force_N"] for r in steady])
    report = dict(status="PASS_REAL_SIM" if np.isfinite(sim.data.qpos).all() and np.count_nonzero(forces) > 40 else "FAIL",
                  mujoco=mujoco.__version__,seed=seed,stiffness=stiffness,admittance=admittance,
                  steady_contact_force_mean_N=float(np.mean(forces)),steady_contact_force_std_N=float(np.std(forces)),
                  force_target_N=2. if admittance else None,force_rmse_N=float(np.sqrt(np.mean((forces-2.)**2))),
                  steady_position_error_m=float(np.sqrt(np.mean([np.sum((np.array(r["position"])-r["target"])**2) for r in steady]))),
                  disturbance_joint2_torque_Nm=2.,samples=len(records),simulation_time_s=sim.data.time)
    out = Path(f"artifacts/physical/impedance_{'admittance' if admittance else 'fixed'}_{seed}")
    out.parent.mkdir(parents=True,exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(report,indent=2))
    out.with_suffix(".jsonl").write_text("".join(json.dumps(r)+"\n" for r in records))
    print(json.dumps(report,indent=2))
    return report


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--seed",type=int,default=7)
    parser.add_argument("--admittance",action="store_true")
    args=parser.parse_args()
    report=run(args.seed,admittance=args.admittance)
    raise SystemExit(0 if report["status"]=="PASS_REAL_SIM" else 1)
