"""Real MuJoCo Panda contact pick/place; no welds, teleportation or grasp flags."""
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from dataclasses import asdict
from embodied_swarm.hal.panda_simulation import PandaSimulation
from embodied_swarm.core.task import TaskExecutor


def run(seed=7, render=False, output=None, simulation=None, navigation=None, target_position=None, grasp_fault=False):
    sim = simulation or PandaSimulation()
    sim.deadline_sim = sim.data.time+80.
    rng = np.random.default_rng(seed)
    obj_addr = sim.model.jnt_qposadr[sim.model.joint("object_free").id]
    sim.data.qpos[obj_addr:obj_addr+2] += rng.uniform(-.015, .015, 2)  # reset-only object randomization
    mujoco.mj_forward(sim.model, sim.data)
    frames = []
    renderer = mujoco.Renderer(sim.model, height=360, width=480) if render else None
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [.4, .1, .3]
    cam.distance, cam.azimuth, cam.elevation = 1.8, 125, -25
    def capture(s):
        if renderer:
            renderer.update_scene(s.data, cam)
            frames.append(renderer.render().copy())
    cube = sim.object_position()
    target = np.array([.45, .22, .025] if target_position is None else target_position)
    grasp_attempts = 0
    def move(pos, width, stage, duration=2.):
        sim.move(sim.cartesian_target(pos), width, duration, stage, capture)
        return np.linalg.norm(sim.data.site_xpos[sim.kinematics.site]-pos) < .012
    def grasp():
        nonlocal grasp_attempts
        grasp_attempts += 1
        # Failure injection: physically keep the fingers open on attempt one.
        width = .08 if grasp_fault and grasp_attempts == 1 else 0.
        sim.move(sim.joint_positions(), width, 1., "GRASP", capture)
        contacts, _ = sim.contact_info()
        return contacts >= 2
    def lift():
        move(cube+[0, 0, .18], 0., "LIFT")
        return sim.object_position()[2] > .14
    def release():
        sim.move(sim.joint_positions(), .08, 1., "RELEASE", capture)
        move(target+[0, 0, .15], .08, "RETREAT")
        return True
    def localize():
        cube[:] = sim.object_position()  # explicitly labeled online simulator ground truth
        return np.isfinite(cube).all() and cube[2] < .05
    stages = [("LOCALIZE_OBJECT", localize),
              ("PLAN_GRASP", lambda: move(cube+[0, 0, .15], .08, "APPROACH")),
              ("DESCEND", lambda: move(cube, .08, "DESCEND")),
              ("GRASP", grasp), ("LIFT", lift),
              ("MOVE", lambda: move(target+[0, 0, .18], 0., "MOVE")),
              ("PLACE", lambda: move(target, 0., "PLACE")),
              ("RELEASE", release),
              ("VERIFY", lambda: np.linalg.norm(sim.object_position()[:2]-target[:2]) < .04 and sim.object_position()[2] < .04)]
    if navigation is not None:
        stages.insert(0,("NAVIGATE",lambda:navigation(sim,capture)))
    def recovery(name):
        if name in ("GRASP", "DESCEND"):
            cube[:] = sim.object_position()  # fresh ground-truth localization after failure
            move(cube+[0, 0, .12], .08, "RECOVERY")
            move(cube, .08, "RETRY_DESCEND")
        else:
            sim.move(sim.joint_positions(), .08, .5, "ABORT_HOLD", capture)
    def abort(reason):
        sim.halt(reason)
    result = TaskExecutor(stages, recovery, timeout_s=100., abort=abort).run()
    report = {"status": "PASS_REAL_SIM" if result.success else "FAIL", "seed": seed,
              "mujoco": mujoco.__version__, "task": asdict(result),
              "object_final": sim.object_position().tolist(), "target": target.tolist(),
              "max_object_height_m": max(r["object"][2] for r in sim.records),
              "max_finger_contact_force_N": sim.max_contact_force,
              "joint_tracking_rmse_rad": float(np.sqrt(np.mean(np.square(sim.joint_errors)))),
              "control_runtime_us": dict(zip(["p50", "p95", "p99"], np.percentile(sim.control_ns, [50, 95, 99])/1000)),
              "simulation_time_s": sim.data.time}
    report["grasp_fault_injected"] = grasp_fault
    report["safety_interventions"] = sim.safety.interventions
    report["safety_latched_reason"] = sim.safety.latched_reason
    if hasattr(sim,"base_qpos"):
        report["base_final"] = sim.read_state().base_pose.tolist()
    output = Path(output or f"artifacts/physical/pick_place_seed_{seed}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".json").write_text(json.dumps(report, indent=2))
    output.with_suffix(".jsonl").write_text("".join(json.dumps(r)+"\n" for r in sim.records))
    if renderer:
        from PIL import Image
        images = [Image.fromarray(f) for f in frames[::2]]
        images[0].save(output.with_suffix(".gif"), save_all=True, append_images=images[1:], duration=100, loop=0)
        images[len(images)//2].save(output.with_suffix(".png"))
        renderer.close()
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    report = run(args.seed, args.render)
    raise SystemExit(0 if report["status"] == "PASS_REAL_SIM" else 1)
