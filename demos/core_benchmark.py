"""Compare in-process FK+Jacobian runtime; CLI startup is excluded."""
import json
import subprocess
from pathlib import Path
from time import perf_counter_ns
import numpy as np
import mujoco
from embodied_swarm.models.kinematics import SerialKinematics
from embodied_swarm.assets import asset_path


def main():
    k=SerialKinematics(mujoco.MjModel.from_xml_path(str(asset_path("mjcf/panda_pick_place.xml"))))
    q=np.array([0.,0.,0.,-1.57079,0.,1.57079,-.7853])
    runtimes=[]
    for i in range(10000):
        q[0]=i*.00001
        start=perf_counter_ns();k.fk(q);k.jacobian(q);runtimes.append((perf_counter_ns()-start)/1000)
    cpp=json.loads(subprocess.run(["cpp/build/core_cli","--benchmark"],check=True,text=True,capture_output=True).stdout)
    report=dict(status="PASS_UNIT",python_samples=len(runtimes),python_fk_jacobian_us=dict(zip(["p50","p95","p99"],np.percentile(runtimes,[50,95,99]))),cpp=cpp,
                method="Same WSL host; 10000 timed calls, no process startup, non-real-time; C++ release and Python NumPy")
    Path("artifacts/physical/core_benchmark.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=="__main__":main()
