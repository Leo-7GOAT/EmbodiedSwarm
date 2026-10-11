from pathlib import Path
import subprocess
import numpy as np
import mujoco
import pytest
from embodied_swarm.models.kinematics import SerialKinematics


def test_cpp_panda_numeric_parity():
    root = Path(__file__).parents[1]
    cli = root/"cpp/build/core_cli"
    if not cli.exists():
        pytest.skip("build cpp/core_cli first; CI builds before parity validation")
    k = SerialKinematics(mujoco.MjModel.from_xml_path(str(root/"assets/mjcf/panda_pick_place.xml")))
    rng = np.random.default_rng(7)
    for _ in range(30):
        q = rng.uniform(k.limits[:,0],k.limits[:,1])
        result = subprocess.run([str(cli),*[str(x) for x in q]],check=True,text=True,capture_output=True)
        values = np.fromstring(result.stdout,sep=" ")
        T = k.fk(q)
        expected = np.r_[T[:3,3],T[:3,:3].ravel(),k.jacobian(q).ravel()]
        assert values.shape == expected.shape
        assert np.max(np.abs(values-expected)) < 1e-12
