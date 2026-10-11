"""Reproduce multi-seed real simulation metrics and figures; no inferred values."""
import json
import platform
from pathlib import Path
from time import perf_counter_ns
import resource
import mujoco
import numpy as np
from demos.physical_navigation import run as navigate
from demos.physical_pick_place import run as manipulate
from demos.mobile_manipulation import execute as mobile
from demos.impedance_contact import run as impedance
from embodied_swarm.models.kinematics import SerialKinematics
from embodied_swarm.assets import asset_path


def main():
    seeds = [7,17,27,37,47]
    nav = [navigate(seed,scenario) for scenario in ["straight","obstacle","delay","fault"] for seed in seeds]
    arm = [manipulate(seed) for seed in seeds]
    tasks = [mobile(seed) for seed in seeds]
    contact = [impedance(seed,admittance=adm) for adm in (False,True) for seed in seeds]
    k = SerialKinematics(mujoco.MjModel.from_xml_path(str(asset_path("mjcf/panda_pick_place.xml"))))
    rng = np.random.default_rng(7)
    ik = []
    for _ in range(100):
        q = np.array([0,0,0,-1.57079,0,1.57079,-.7853])+rng.uniform(-.3,.3,7)
        T = k.fk(q)
        before = perf_counter_ns()
        result = k.ik(T[:3,3],np.array([0,0,0,-1.57079,0,1.57079,-.7853]),T[:3,:3])
        ik.append(dict(success=result.success,error_m=result.position_error,runtime_ms=(perf_counter_ns()-before)/1e6))
    summary = dict(status="PASS_REAL_SIM" if all(r["status"]=="PASS_REAL_SIM" for r in nav+arm+tasks+contact) else "FAIL",
                   environment=dict(platform=platform.platform(),python=platform.python_version(),mujoco=mujoco.__version__,
                                    numpy=np.__version__,cpu=platform.processor(),memory_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
                   seeds=seeds,navigation=nav,manipulation=arm,mobile_tasks=tasks,impedance=contact,
                   ik=dict(samples=len(ik),success_rate=float(np.mean([r["success"] for r in ik])),
                           error_max_m=max(r["error_m"] for r in ik),
                           runtime_ms=dict(zip(["p50","p95","p99"],np.percentile([r["runtime_ms"] for r in ik],[50,95,99])))))
    Path("artifacts/physical/benchmark.json").write_text(json.dumps(summary,indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    scenarios=["straight","obstacle","delay","fault"]
    axes[0].bar(scenarios,[np.mean([r["tracking_rmse_m"] for r in nav if r["scenario"]==s]) for s in scenarios])
    axes[0].set_ylabel("Physical navigation CTE RMSE (m)")
    axes[1].bar(["arm","NAV+arm"],[np.mean([r["joint_tracking_rmse_rad"] for r in arm]),np.mean([r["joint_tracking_rmse_rad"] for r in tasks])])
    axes[1].set_ylabel("Joint tracking RMSE (rad)")
    axes[2].bar(["fixed impedance","admittance"],[np.mean([r["force_rmse_N"] for r in contact if r["admittance"]==a]) for a in (False,True)])
    axes[2].set_ylabel("Contact force RMSE vs 2 N (N)")
    fig.tight_layout();fig.savefig("artifacts/physical/benchmark.png",dpi=150);plt.close(fig)
    navlog=[json.loads(x) for x in Path("artifacts/physical/navigation_obstacle_seed_7.jsonl").read_text().splitlines()]
    armlog=[json.loads(x) for x in Path("artifacts/physical/mobile_task_seed_7.jsonl").read_text().splitlines()]
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    pos=np.array([r["pose"] for r in navlog])
    axes[0].plot(pos[:,0],pos[:,1]);axes[0].add_patch(plt.Rectangle((1.3,.1),.4,1.,color="red",alpha=.5))
    axes[0].set_aspect("equal");axes[0].set_title("Actual MuJoCo base trajectory");axes[0].set_xlabel("x (m)");axes[0].set_ylabel("y (m)")
    axes[1].plot([r["time"] for r in armlog],[r["object"][2] for r in armlog]);axes[1].set_title("Physical object lift and release")
    axes[1].set_xlabel("simulation time (s)");axes[1].set_ylabel("object z (m)")
    fig.tight_layout();fig.savefig("artifacts/physical/trajectories.png",dpi=150);plt.close(fig)
    print(json.dumps({"status":summary["status"],"navigation":len(nav),"arm":len(arm),"mobile":len(tasks),"impedance":len(contact),"ik":summary["ik"]},indent=2))


if __name__ == "__main__":
    main()
