"""NAV -> online GT localization -> contact grasp -> move -> place in one world."""
import argparse
import numpy as np
from embodied_swarm.hal.mobile_panda import MobilePandaSimulation
from embodied_swarm.core.safety import SafetySupervisor
from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.control.pure_pursuit import pure_pursuit_control
from demos.physical_pick_place import run


def navigate(sim,capture):
    path = [(0.,0.),(.1,0.),(.2,0.),(.3,0.),(.4,0.),(.5,0.)]
    safety = SafetySupervisor()
    for _ in range(500):
        state = sim.read_state()
        v,w = pure_pursuit_control(tuple(state.base_pose),path,.15,.18,.025)
        command = safety.filter(state,RobotCommand(timestamp=state.timestamp,base_twist=[v,0.,w]),state.timestamp)
        sim.base_twist = command.base_twist
        for _ in range(10):
            sim.step(sim.HOME,.08,"NAVIGATE",capture)
        if np.linalg.norm(state.base_pose[:2]-np.array(path[-1])) < .025:
            sim.base_twist[:] = 0
            sim.move(sim.joint_positions(),.08,.5,"NAV_SETTLE",capture)
            return np.linalg.norm(sim.read_state().base_pose[:2]-np.array(path[-1])) < .03
    sim.base_twist[:] = 0
    return False


def execute(seed=7,render=False,grasp_fault=False):
    suffix = "_fault" if grasp_fault else ""
    return run(seed,render,f"artifacts/physical/mobile_task{suffix}_seed_{seed}",
               MobilePandaSimulation(),navigate,[.95,.22,.025],grasp_fault)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",type=int,default=7)
    parser.add_argument("--render",action="store_true")
    parser.add_argument("--grasp-fault",action="store_true")
    args = parser.parse_args()
    report = execute(args.seed,args.render,args.grasp_fault)
    raise SystemExit(0 if report["status"] == "PASS_REAL_SIM" else 1)
