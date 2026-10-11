"""A* -> smoothing -> pure pursuit -> force servo -> MuJoCo contact dynamics."""
import argparse
import json
import os
from pathlib import Path
from time import perf_counter_ns
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from embodied_swarm.hal.physical_mujoco_adapter import PhysicalMujocoAdapter
from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.safety import SafetySupervisor
from embodied_swarm.navigation.occupancy_grid import OccupancyGrid
from embodied_swarm.navigation.astar import astar
from embodied_swarm.navigation.path_smoothing import smooth_path
from embodied_swarm.control.pure_pursuit import pure_pursuit_control


def run(seed=7, scenario="obstacle", render=False):
    rng = np.random.default_rng(seed)
    sim = PhysicalMujocoAdapter()
    sim.model.actuator_gainprm[:3, 0] *= rng.uniform(.8, 1.2)
    # Velocity actuators have affine bias: vary both proportional terms together.
    sim.model.actuator_biasprm[:3, 2] = -sim.model.actuator_gainprm[:3, 0]
    obstacle = sim.model.geom("obstacle").id
    if scenario == "straight":
        sim.model.geom_pos[obstacle, :2] = [5., 5.]
    grid_data = np.zeros((50, 50), dtype=int)
    grid = OccupancyGrid(grid_data, .1, -1., -1.)
    if scenario != "straight":
        # Minkowski inflation includes footprint and controller clearance.
        for row in range(50):
            for col in range(50):
                x, y = grid.grid_to_world(row, col)
                if abs(x-1.5) <= .55 and abs(y-.6) <= .85:
                    grid_data[row, col] = 1
    goal = np.array([2.8, 0. if scenario == "straight" else 1.5])
    before = perf_counter_ns()
    raw = astar(grid, grid.world_to_grid(0, 0), grid.world_to_grid(*goal))
    if not raw:
        raise RuntimeError("planning failed")
    smoothed = smooth_path(grid, [(float(r), float(c)) for r, c in raw])
    path = np.array([[grid.origin_x+(c+.5)*grid.resolution,
                      grid.origin_y+(r+.5)*grid.resolution] for r, c in smoothed])
    path[0] = [0., 0.]
    path[-1] = goal
    planning_ms = (perf_counter_ns()-before)/1e6
    safety = SafetySupervisor()
    records, runtimes, frames, errors = [], [], [], []
    renderer = mujoco.Renderer(sim.model, height=360, width=480) if render else None
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = [1.4, .5, 0.], 5., 100, -65
    collisions, fault_injected, recovery_count = 0, False, 0
    pending = []
    for step in range(1800):
        state = sim.read_state()
        before = perf_counter_ns()
        v, w = pure_pursuit_control(tuple(state.base_pose), path.tolist(), .25, .35, .08)
        command = RobotCommand(timestamp=state.timestamp, base_twist=[v, 0., w])
        if scenario == "fault" and step == 100:
            command.timestamp -= 1.
            fault_injected = True
        safe = safety.filter(state, command, state.timestamp)
        # Base-only adapter intentionally rejects arm channels.
        safe = RobotCommand(timestamp=safe.timestamp, base_twist=safe.base_twist)
        pending.append(safe)
        applied = pending.pop(0) if scenario != "delay" or len(pending) > 5 else RobotCommand(timestamp=state.timestamp)
        sim.send_command(applied)
        runtimes.append((perf_counter_ns()-before)/1000)
        state = sim.step(.02)
        if safety.latched_reason and np.linalg.norm(state.base_twist) < .01:
            if safety.reset(state, state.timestamp):
                recovery_count += 1
        contacts = 0
        for c in range(sim.data.ncon):
            contact = sim.data.contact[c]
            if obstacle in (contact.geom1, contact.geom2):
                contacts += 1
        collisions += bool(contacts)
        # Point-to-segment cross-track error (not nearest sampled waypoint).
        a, b = path[:-1], path[1:]
        t = np.clip(np.sum((state.base_pose[:2]-a)*(b-a), axis=1)/np.maximum(1e-12,np.sum((b-a)**2, axis=1)), 0., 1.)
        error = np.min(np.linalg.norm(state.base_pose[:2]-(a+t[:,None]*(b-a)), axis=1))
        errors.append(error)
        records.append(dict(time=state.timestamp, pose=state.base_pose.tolist(), twist=state.base_twist.tolist(),
                            command=applied.base_twist.tolist(), error_m=float(error), contacts=contacts))
        if renderer and step % 5 == 0:
            renderer.update_scene(sim.data, cam)
            frames.append(renderer.render().copy())
        if np.linalg.norm(state.base_pose[:2]-goal) < .085:
            break
    final_error = float(np.linalg.norm(state.base_pose[:2]-goal))
    report = dict(status="PASS_REAL_SIM" if final_error < .085 and collisions == 0 else "FAIL",
                  seed=seed, scenario=scenario, mujoco=mujoco.__version__, planning_ms=planning_ms,
                  tracking_rmse_m=float(np.sqrt(np.mean(np.square(errors)))), final_error_m=final_error,
                  obstacle_contact_steps=collisions, steps=len(records), simulation_time_s=state.timestamp,
                  recovery_count=recovery_count, safety_interventions=safety.interventions,
                  fault_injected=fault_injected,
                  control_runtime_us=dict(zip(["p50", "p95", "p99"],np.percentile(runtimes,[50,95,99]))))
    output = Path(f"artifacts/physical/navigation_{scenario}_seed_{seed}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".json").write_text(json.dumps(report, indent=2))
    output.with_suffix(".jsonl").write_text("".join(json.dumps(r)+"\n" for r in records))
    if renderer:
        from PIL import Image
        images = [Image.fromarray(f) for f in frames]
        images[0].save(output.with_suffix(".gif"), save_all=True, append_images=images[1:], duration=100, loop=0)
        images[len(images)//2].save(output.with_suffix(".png"))
        renderer.close()
    sim.shutdown()
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--scenario", choices=["straight","obstacle","delay","fault"], default="obstacle")
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    result = run(args.seed, args.scenario, args.render)
    raise SystemExit(0 if result["status"] == "PASS_REAL_SIM" else 1)
