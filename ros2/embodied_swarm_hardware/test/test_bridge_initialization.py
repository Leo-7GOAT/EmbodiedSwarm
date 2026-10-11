"""Real MuJoCo regression for the cloud START_STATE_INVALID startup race."""
import importlib.util
from pathlib import Path
import mujoco
import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parent.parent
spec = importlib.util.spec_from_file_location('mujoco_bridge', PACKAGE / 'scripts/mujoco_bridge.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def test_no_keyframe_uses_shared_contract_before_physics():
    model = mujoco.MjModel.from_xml_path(str(REPO / 'assets/mjcf/panda_pick_place.xml'))
    # The generated scene has no home keyframe; zero joint4 is invalid.
    assert model.nkey == 0
    data = mujoco.MjData(model)
    jids = [model.joint(f'joint{i}').id for i in range(1, 8)]
    qa, va = model.jnt_qposadr[jids], model.jnt_dofadr[jids]
    aids = [model.actuator(f'actuator{i}').id for i in range(1, 8)]
    assert data.qpos[qa[3]] > model.jnt_range[jids[3], 1]
    expected = bridge.arm_initial_positions(PACKAGE / 'config/panda.urdf.xacro')
    bridge.initialize_arm(model, data, jids, aids, expected)
    assert data.qpos[qa] == pytest.approx(expected, abs=1e-12)
    assert data.ctrl[aids] == pytest.approx(expected, abs=1e-12)
    for _ in range(500):
        data.qfrc_applied[va] = data.qfrc_bias[va]
        mujoco.mj_step(model, data)
    # Actual physics must retain the valid startup hold, not drift into the
    # self-colliding zero-default pose seen in the failed cloud artifact.
    assert data.time > .9
    assert data.qpos[qa] == pytest.approx(expected, abs=.005)


@pytest.mark.parametrize('fault', ['nan', 'bounds', 'shape'])
def test_invalid_initial_condition_rejected_without_mutation(fault):
    model = mujoco.MjModel.from_xml_path(str(REPO / 'assets/mjcf/panda_pick_place.xml'))
    data = mujoco.MjData(model)
    jids = [model.joint(f'joint{i}').id for i in range(1, 8)]
    aids = [model.actuator(f'actuator{i}').id for i in range(1, 8)]
    initial = bridge.arm_initial_positions(PACKAGE / 'config/panda.urdf.xacro')
    if fault == 'nan': initial[0] = np.nan
    if fault == 'bounds': initial[3] = 0
    if fault == 'shape': initial = initial[:6]
    before_q, before_ctrl = data.qpos.copy(), data.ctrl.copy()
    with pytest.raises(ValueError, match='initial arm'):
        bridge.initialize_arm(model, data, jids, aids, initial)
    assert np.array_equal(data.qpos, before_q)
    assert np.array_equal(data.ctrl, before_ctrl)
