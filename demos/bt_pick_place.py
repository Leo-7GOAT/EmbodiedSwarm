"""Tickable BT executes physical contact failure/recovery and safe interruption."""
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault("MUJOCO_GL","egl")
import mujoco
import numpy as np
from embodied_swarm.hal.panda_simulation import PandaSimulation
from embodied_swarm.task.bt import Action,BehaviorTree,Condition,Fallback,Sequence,Status


def build_tree(sim,mode,capture=None,timeout_s=80.):
    def localize(label):
        def observe(context,memory):
            position=sim.object_position()
            context.blackboard["cube"]=position.tolist()
            context.blackboard.setdefault("observations",[]).append(dict(time=float(sim.data.time),position=position.tolist(),label=label))
            return Status.SUCCESS if np.isfinite(position).all() and position[2]<.05 else Status.FAILURE
        return Action(label,observe)

    def motion(name,position=None,width=.08,duration=2.,joint_hold=False):
        def advance(context,memory):
            if not memory:
                target=sim.joint_positions() if joint_hold else sim.cartesian_target(position(context.blackboard))
                memory.update(start=sim.joint_positions().tolist(),target=target.tolist(),step=0,
                              steps=round(duration/sim.model.opt.timestep))
                if name.startswith("CLOSE"):
                    context.blackboard["grasp_attempts"]=context.blackboard.get("grasp_attempts",0)+1
            command_width=width(context.blackboard) if callable(width) else width
            start=np.array(memory["start"]);target=np.array(memory["target"])
            for _ in range(min(5,memory["steps"]-memory["step"])):
                memory["step"]+=1
                fraction=min(1.,memory["step"]/(memory["steps"]*.7))
                blend=fraction*fraction*(3-2*fraction)
                sim.step(start+blend*(target-start),command_width,name,capture)
            if memory["step"]<memory["steps"]:
                return Status.RUNNING
            if not joint_hold:
                position_error=float(np.linalg.norm(sim.data.site_xpos[sim.kinematics.site]-position(context.blackboard)))
                memory["position_error_m"]=position_error
                if position_error>=.012:
                    return Status.FAILURE
            return Status.SUCCESS
        return Action(name,advance)

    def contacts(label):
        def verify(context):
            count,force=sim.contact_info()
            context.blackboard.setdefault("grasp_checks",[]).append(dict(time=float(sim.data.time),contacts=count,force_N=force))
            return count>=2
        return Condition(label,verify)

    cube=lambda board:np.array(board["cube"])
    target=np.array([.45,.22,.025])
    first_width=.08 if mode in ("recover","exhaust","resume") else 0.
    second_width=.08 if mode=="exhaust" else 0.
    grasp=Fallback("GRASP_WITH_RECOVERY",[
        Sequence("FIRST_ATTEMPT",[motion("CLOSE_FIRST",width=first_width,duration=1.,joint_hold=True),contacts("CHECK_FIRST_CONTACT")]),
        Sequence("RECOVERY_AND_SECOND_ATTEMPT",[
            localize("FRESH_FAILURE_LOCALIZATION"),
            motion("RECOVERY_RETREAT",lambda board:cube(board)+[0,0,.12]),
            localize("FRESH_RETRY_LOCALIZATION"),
            motion("RETRY_DESCEND",cube),
            motion("CLOSE_RETRY",width=second_width,duration=1.,joint_hold=True),contacts("CHECK_RETRY_CONTACT")])])
    root=Sequence("PICK_PLACE",[
        localize("INITIAL_LOCALIZATION"),motion("APPROACH",lambda board:cube(board)+[0,0,.15]),motion("DESCEND",cube),grasp,
        motion("LIFT",lambda board:cube(board)+[0,0,.18],width=0.),
        Condition("VERIFY_LIFT",lambda context:bool(sim.object_position()[2]>.14)),
        motion("TRANSPORT",lambda board:target+[0,0,.18],width=0.),
        motion("PLACE",lambda board:target,width=0.),
        motion("RELEASE",width=.08,duration=1.,joint_hold=True),
        motion("RETREAT",lambda board:target+[0,0,.15]),
        Condition("VERIFY_PLACE",lambda context:bool(np.linalg.norm(sim.object_position()[:2]-target[:2])<.04 and sim.object_position()[2]<.04))])
    return BehaviorTree(root,timeout_s=timeout_s,clock=lambda:float(sim.data.time),abort=sim.halt),target


def run(seed=7,mode="recover",output_root="artifacts/bt",render=False):
    if mode not in ("nominal","recover","exhaust","abort","deadline","resume"):
        raise ValueError("unknown BT scenario")
    out=Path(output_root);out.mkdir(parents=True,exist_ok=True)
    sim=PandaSimulation()
    addr=sim.model.jnt_qposadr[sim.model.joint("object_free").id]
    sim.data.qpos[addr:addr+2]+=np.random.default_rng(seed).uniform(-.015,.015,2)  # initialization only
    mujoco.mj_forward(sim.model,sim.data)
    renderer=mujoco.Renderer(sim.model,height=360,width=480) if render else None
    frames=[];camera=mujoco.MjvCamera();camera.lookat[:]=[.4,.1,.3];camera.distance=1.8;camera.azimuth=125;camera.elevation=-25
    def capture(world):
        if renderer:
            renderer.update_scene(world.data,camera);frames.append(renderer.render().copy())
    timeout=3. if mode=="deadline" else 80.
    tree,target=build_tree(sim,mode,capture,timeout)
    sim.deadline_sim=tree.deadline
    resumed=False;checkpoint=None
    tick_times=[]
    from time import perf_counter_ns
    while tree.status==Status.RUNNING:
        if mode=="abort" and sim.data.time>=3.:
            tree.request_abort("operator_abort")
        if mode=="resume" and not resumed and sim.data.time>=2.5:
            checkpoint=tree.snapshot()
            (out/f"resume_seed_{seed}_checkpoint.json").write_text(json.dumps(checkpoint,indent=2,allow_nan=False))
            # Rebuild code and restore persisted cursors against the same live
            # physical world and clock. No simulator state is changed.
            tree,target=build_tree(sim,mode,capture,timeout)
            tree.restore(json.loads((out/f"resume_seed_{seed}_checkpoint.json").read_text()))
            resumed=True
        before=perf_counter_ns();tree.tick();tick_times.append((perf_counter_ns()-before)/1000)
    terminal_time=float(sim.data.time);hold_start=sim.joint_positions()
    if tree.status!=Status.SUCCESS:
        # Halt already latched. Let inertia settle with the same physical gate.
        for _ in range(250):
            sim.step(hold_start,.08,"ABORT_SETTLE",capture)
    final_position=sim.object_position()
    place_error=float(np.linalg.norm(final_position[:2]-target[:2]))
    actual_hold_drift=float(np.max(np.abs(sim.joint_positions()-hold_start)))
    checks=tree.blackboard.get("grasp_checks",[])
    recovery=(tree.nodes.get("RECOVERY_AND_SECOND_ATTEMPT",{}).get("status")=="SUCCESS")
    physics_success=tree.status==Status.SUCCESS and place_error<.04 and final_position[2]<.04 and max(r["object"][2] for r in sim.records)>.14
    expected_abort=(mode in ("abort","deadline") and tree.status==Status.ABORTED and tree.abort_reason==("operator_abort" if mode=="abort" else "deadline"))
    expected_failure=mode=="exhaust" and tree.status==Status.FAILURE and tree.abort_reason=="tree_failure"
    safe_stop=(sim.safety.latched_reason is not None and actual_hold_drift<.02)
    accepted=physics_success if mode in ("nominal","recover","resume") else (expected_abort or expected_failure) and safe_stop
    if mode in ("recover","resume"):
        accepted=accepted and recovery and len(checks)==2 and checks[0]["contacts"]<2 and checks[1]["contacts"]>=2
    report=dict(status="PASS_REAL_SIM" if accepted else "FAIL",seed=seed,mode=mode,mujoco=mujoco.__version__,
                tree_status=tree.status.value,abort_reason=tree.abort_reason,ticks=tree.ticks,terminal_time_s=terminal_time,
                simulation_time_s=float(sim.data.time),object_final=final_position.tolist(),target=target.tolist(),place_error_m=place_error,
                max_object_height_m=max(r["object"][2] for r in sim.records),grasp_checks=checks,recovery_success=recovery,
                observations=tree.blackboard.get("observations",[]),checkpoint_restored=resumed,
                hold_drift_rad=actual_hold_drift,safety_latched_reason=sim.safety.latched_reason,
                joint_tracking_rmse_rad=float(np.sqrt(np.mean(np.square(sim.joint_errors)))),
                tick_runtime_us=dict(zip(["p50","p95","p99"],np.percentile(tick_times,[50,95,99]))),
                localization_provenance="Online MuJoCo object pose, not perception; no object pose overwrites after initialization",
                acceptance=dict(place_error_m=.04,lift_height_m=.14,cartesian_motion_error_m=.012,abort_hold_drift_rad=.02))
    stem=out/f"{mode}_seed_{seed}"
    stem.with_suffix(".json").write_text(json.dumps(report,indent=2,allow_nan=False))
    (out/f"{mode}_seed_{seed}_tree.json").write_text(json.dumps(tree.snapshot(),indent=2,allow_nan=False))
    stem.with_suffix(".jsonl").write_text("".join(json.dumps(row,allow_nan=False)+"\n" for row in sim.records))
    if renderer:
        from PIL import Image
        images=[Image.fromarray(frame) for frame in frames[::2]]
        images[0].save(stem.with_suffix(".gif"),save_all=True,append_images=images[1:],duration=100,loop=0)
        images[len(images)//2].save(stem.with_suffix(".png"));renderer.close()
    print(json.dumps(report,indent=2,allow_nan=False));return report


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--seed",type=int,default=7)
    parser.add_argument("--mode",choices=["nominal","recover","exhaust","abort","deadline","resume"],default="recover")
    parser.add_argument("--render",action="store_true");args=parser.parse_args()
    result=run(args.seed,args.mode,render=args.render)
    raise SystemExit(0 if result["status"]=="PASS_REAL_SIM" else 1)
