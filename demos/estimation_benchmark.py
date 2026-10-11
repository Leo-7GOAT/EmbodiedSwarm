"""Five seeds, strict unchanged navigation gates, sensor-only feedback."""
import json
import platform
from pathlib import Path
import numpy as np
import mujoco
from demos.estimation_closed_loop import run


def main():
    seeds=[7,17,27,37,47]
    results=[run(seed,scenario) for scenario in ["nominal","delay","dropout","anomalies","outage"] for seed in seeds]
    out=Path("artifacts/estimation")
    report=dict(status="PASS_REAL_SIM" if all(r["status"]=="PASS_REAL_SIM" for r in results) else "FAIL",
                python=platform.python_version(),platform=platform.platform(),mujoco=mujoco.__version__,numpy=np.__version__,
                seeds=seeds,runs=len(results),results=results,
                boundary="20 navigation runs + 5 deliberate safe aborts, not 25 goal successes; virtual encoders and simulated IMU only")
    (out/"benchmark.json").write_text(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    scenarios=["nominal","delay","dropout","anomalies"]
    axes[0].bar(scenarios,[np.mean([r["position_estimation_rmse_m"] for r in results if r["scenario"]==s]) for s in scenarios]);axes[0].set_ylabel("Pose estimate position RMSE (m)")
    axes[1].bar(scenarios,[np.mean([r["path_tracking_rmse_m"] for r in results if r["scenario"]==s]) for s in scenarios]);axes[1].axhline(.08,color="red",linestyle="--",label="unchanged gate");axes[1].set_ylabel("Physical path tracking RMSE (m)");axes[1].legend()
    records=[json.loads(line) for line in (out/"delay_seed_7.jsonl").read_text().splitlines()]
    gt=np.array([r["truth"] for r in records]);est=np.array([r["estimate"] for r in records])
    axes[2].plot(gt[:,0],gt[:,1],label="engine GT, evaluation only");axes[2].plot(est[:,0],est[:,1],"--",label="EKF control feedback");axes[2].set_aspect("equal");axes[2].legend(fontsize=7);axes[2].set_xlabel("x (m)");axes[2].set_ylabel("y (m)")
    fig.tight_layout();fig.savefig(out/"benchmark.png",dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    log=[json.loads(line) for line in (out/"nominal_seed_7_step_response.jsonl").read_text().splitlines()]
    t=np.array([r["time"] for r in log]);v=np.array([r["encoder_speed_m_s"] for r in log]);ident=next(r["actuator_identification"] for r in results if r["scenario"]=="nominal" and r["seed"]==7)
    axes[0].plot(t,v,label="actual physical response, virtual encoder");axes[0].plot(t,ident["steady_speed_m_s"]*(1-np.exp(-t/ident["time_constant_s"])),label="identified first-order model");axes[0].set_xlabel("step time (s)");axes[0].set_ylabel("speed (m/s)");axes[0].legend(fontsize=7)
    outage=[json.loads(line) for line in (out/"outage_seed_7.jsonl").read_text().splitlines()]
    axes[1].plot([r["time"] for r in outage],[np.linalg.norm(r["truth_world_velocity"][:2]) for r in outage],label="actual speed");axes[1].plot([r["time"] for r in outage],[r["measurement_age_s"] for r in outage],label="sensor age");axes[1].set_xlabel("simulation time (s)");axes[1].legend()
    fig.tight_layout();fig.savefig(out/"identification_and_stop.png",dpi=150);plt.close(fig)
    print(json.dumps(dict(status=report["status"],runs=len(results)),indent=2))
    return report


if __name__=="__main__":
    result=main();raise SystemExit(0 if result["status"]=="PASS_REAL_SIM" else 1)
