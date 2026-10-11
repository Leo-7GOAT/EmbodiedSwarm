"""Multi-seed actual BT/contact/recovery/persistence/stop acceptance."""
import json
import argparse
from pathlib import Path
import platform
import mujoco
import numpy as np
from demos.bt_pick_place import run


def main(render=True):
    out=Path("artifacts/bt");out.mkdir(parents=True,exist_ok=True)
    modes=["nominal","recover","resume","abort","deadline","exhaust"]
    seeds=[7,17,27,37,47]
    results=[run(seed,mode,render=(render and mode=="recover" and seed==7)) for mode in modes for seed in seeds]
    report=dict(status="PASS_REAL_SIM" if all(r["status"]=="PASS_REAL_SIM" for r in results) else "FAIL",
                runs=len(results),seeds=seeds,modes=modes,python=platform.python_version(),platform=platform.platform(),
                mujoco=mujoco.__version__,results=results,
                boundary="15 completed place tasks, 10 deliberate safe aborts, 5 deliberate exhausted-grasp safe failures; not 30 task successes")
    (out/"benchmark.json").write_text(json.dumps(report,indent=2,allow_nan=False))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(12,4))
    success=["nominal","recover","resume"]
    axes[0].bar(success,[max(r["place_error_m"] for r in results if r["mode"]==mode) for mode in success])
    axes[0].axhline(.04,color="red",linestyle="--",label="unchanged place gate");axes[0].set_ylabel("Worst final physical place error (m)");axes[0].legend(fontsize=7)
    stops=["abort","deadline","exhaust"]
    axes[1].bar(stops,[max(r["hold_drift_rad"] for r in results if r["mode"]==mode) for mode in stops])
    axes[1].axhline(.02,color="red",linestyle="--",label="unchanged safe hold gate");axes[1].set_ylabel("Worst actual arm stop drift (rad)");axes[1].legend(fontsize=7)
    log=[json.loads(line) for line in (out/"recover_seed_7.jsonl").read_text().splitlines()]
    axes[2].plot([row["time"] for row in log],[row["object"][2] for row in log],label="actual free object height")
    axes[2].axhline(.14,color="red",linestyle="--",label="unchanged lift gate")
    checks=next(r["grasp_checks"] for r in results if r["mode"]=="recover" and r["seed"]==7)
    for i,check in enumerate(checks):
        axes[2].axvline(check["time"],linestyle=":",label=f"grasp {i+1}: {check['contacts']} contacts")
    axes[2].set_xlabel("simulation time (s)");axes[2].set_ylabel("object height (m)");axes[2].legend(fontsize=7)
    fig.tight_layout();fig.savefig(out/"benchmark.png",dpi=150);plt.close(fig)
    print(json.dumps(dict(status=report["status"],runs=len(results)),indent=2))
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--no-render",action="store_true");args=parser.parse_args()
    result=main(not args.no_render);raise SystemExit(0 if result["status"]=="PASS_REAL_SIM" else 1)
