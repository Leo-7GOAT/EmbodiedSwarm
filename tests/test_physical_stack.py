from pathlib import Path
import numpy as np
import pytest
import mujoco
from embodied_swarm.models.kinematics import SerialKinematics
from embodied_swarm.hal.panda_simulation import PandaSimulation
from embodied_swarm.hal.physical_mujoco_adapter import PhysicalMujocoAdapter
from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.robot_state import RobotState
from embodied_swarm.core.safety import SafetySupervisor
from embodied_swarm.core.safety import SafetyLimits
from embodied_swarm.core.task import TaskExecutor
from embodied_swarm.control.impedance import cartesian_impedance_torque


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(Path(__file__).parents[1]/"assets/mjcf/panda_pick_place.xml"))


@pytest.mark.parametrize("seed", range(8))
def test_fk_jacobian_against_mujoco_and_finite_difference(model, seed):
    k = SerialKinematics(model)
    rng = np.random.default_rng(seed)
    q = rng.uniform(k.limits[:, 0]+.05, k.limits[:, 1]-.05)
    d = mujoco.MjData(model)
    d.qpos[:7] = q
    mujoco.mj_forward(model, d)
    T = k.fk(q)
    assert np.max(np.abs(T[:3, 3]-d.site_xpos[k.site])) < 1e-12
    assert np.max(np.abs(T[:3,:3]-d.site_xmat[k.site].reshape(3,3))) < 1e-12
    jp, jr = np.zeros((3,model.nv)), np.zeros((3,model.nv))
    mujoco.mj_jacSite(model,d,jp,jr,k.site)
    assert np.max(np.abs(k.jacobian(q)-np.vstack([jp[:,:7],jr[:,:7]]))) < 1e-12
    eps = 1e-6
    numeric = np.column_stack([(k.fk(q+np.eye(7)[i]*eps)[:3,3]-k.fk(q-np.eye(7)[i]*eps)[:3,3])/(2*eps) for i in range(7)])
    assert np.max(np.abs(numeric-k.jacobian(q)[:3])) < 1e-8


@pytest.mark.parametrize("seed", range(8))
def test_ik_reachable_pose(model, seed):
    k = SerialKinematics(model)
    q = PandaSimulation.HOME+np.random.default_rng(seed).uniform(-.15,.15,7)
    T = k.fk(q)
    result = k.ik(T[:3,3], PandaSimulation.HOME, T[:3,:3])
    assert result.success
    assert result.position_error < 2e-4
    assert np.all(result.q >= k.limits[:,0]) and np.all(result.q <= k.limits[:,1])


def test_ik_unreachable_singular_and_invalid(model):
    k = SerialKinematics(model)
    result = k.ik([50.,0.,0.],np.zeros(7),max_iterations=30)
    assert not result.success and np.isfinite(result.q).all()
    assert np.all(result.q >= k.limits[:,0]) and np.all(result.q <= k.limits[:,1])
    assert k.manipulability(PandaSimulation.HOME) >= 0
    with pytest.raises(ValueError):
        k.ik([np.nan,0,0],np.zeros(7))
    with pytest.raises(ValueError):
        k.ik([0,0,0],np.zeros(7),np.zeros((3,3)))
    with pytest.raises(ValueError):
        k.fk(np.zeros(6))


def test_physical_base_inertia_timeout_and_collision():
    s = PhysicalMujocoAdapter()
    s.send_command(RobotCommand(base_twist=[.35,0,0]))
    state = s.step(.02)
    assert 0 < state.base_twist[0] < .35  # actual finite acceleration
    assert state.base_pose[0] < .35*.02  # excludes ideal kinematic overwrite
    s.step(.5)
    assert s.interventions > 0
    # New reset-only pose fixture aimed at actual collider.
    s.reset()
    s.data.qpos[:2] = [1., .6]
    mujoco.mj_forward(s.model,s.data)
    for _ in range(100):
        s.send_command(RobotCommand(timestamp=s.data.time,base_twist=[.5,0,0]))
        s.step(.02)
    assert s.data.qpos[0] < 1.15
    assert s.data.ncon > 0
    with pytest.raises(ValueError):
        s.step(.003)
    with pytest.raises(ValueError):
        s.send_command(RobotCommand(joint_position_targets=np.zeros(7)))
    s.shutdown()
    with pytest.raises(RuntimeError):
        s.step(.02)


def test_safety_latch_manual_geofence_and_reset():
    s = SafetySupervisor()
    state = RobotState(timestamp=1.)
    out = s.filter(state,RobotCommand(timestamp=1.,base_twist=[5.,0.,5.]),1.)
    assert out.base_twist[0] == .6 and out.base_twist[2] == 1.2
    out = s.filter(state,RobotCommand(timestamp=0.,base_twist=[.2,0,0]),1.)
    assert not out.base_twist.any() and s.latched_reason == "command_timeout"
    assert not s.reset(RobotState(timestamp=1.,base_twist=[.2,0,0]),1.)
    assert s.reset(state,1.)
    s.manual_override = True
    assert not s.filter(state,RobotCommand(timestamp=1.),1.).base_twist.any()
    s.manual_override = False
    assert s.reset(state,1.)
    s.filter(RobotState(timestamp=1.,base_pose=[5,0,0]),RobotCommand(timestamp=1.),1.)
    assert s.latched_reason == "geofence"


def test_safety_invalid_limits_norm_and_joint_slew():
    for limits in (SafetyLimits(max_speed=np.nan),SafetyLimits(timeout=np.inf),
                   SafetyLimits(geofence=(1,0,0,1)),SafetyLimits(geofence=(0,1,np.nan,1))):
        with pytest.raises(ValueError):
            SafetySupervisor(limits)
    limits = np.tile([-1.,1.],(7,1))
    s = SafetySupervisor(joint_limits=limits)
    state = RobotState()
    out = s.filter(state,RobotCommand(base_twist=[.6,.6,0],joint_position_targets=np.full(7,.5)),0.)
    assert np.linalg.norm(out.base_twist[:2]) <= .6+1e-12
    assert np.max(out.joint_position_targets) <= .03+1e-12
    out = s.filter(state,RobotCommand(joint_position_targets=np.full(7,2.)),0.)
    assert s.latched_reason == "joint_command_limit" and not out.base_twist.any()


def test_task_recovery_abort_deadline():
    calls = []
    def action():
        calls.append(1)
        return len(calls) == 2
    r = TaskExecutor([("GRASP",action)],lambda name: calls.append(0)).run()
    # Recovery adds a list item: use independent retry counter instead below.
    assert not r.success and r.recoveries == 1
    attempts = [False, True]
    r = TaskExecutor([("GRASP",lambda: attempts.pop(0))],lambda name: None).run()
    assert r.success and r.recoveries == 1 and r.completed == ["GRASP"]
    times = iter([0.,2.,2.])
    r = TaskExecutor([("MOVE",lambda: True)],lambda name: None,timeout_s=1,clock=lambda:next(times)).run()
    assert not r.success and r.failures[0]["reason"] == "timeout"
    aborts = []
    times = iter([0.,0.,2.,2.])
    r = TaskExecutor([("LAST",lambda: True)],lambda name: None,timeout_s=1,clock=lambda:next(times),abort=aborts.append).run()
    assert not r.success and aborts == ["timeout"]
    def failed_recovery(name):
        raise RuntimeError("recovery collision")
    r = TaskExecutor([("GRASP",lambda: False)],failed_recovery,abort=aborts.append).run()
    assert not r.success and "recovery_failed" in r.failures[-1]["reason"]
    for kwargs in ({"max_retries":1.5},{"timeout_s":np.nan},{"timeout_s":np.inf}):
        with pytest.raises(ValueError):
            TaskExecutor([],lambda name: None,**kwargs)


def test_impedance_damping_bias_saturation():
    J = np.eye(3)
    assert np.allclose(cartesian_impedance_torque(J,[0,0,0],[0,0,0],[1,2,3]),[1,2,3])
    tau = cartesian_impedance_torque(J,[0,0,0],[1,1,1],[0,0,0])
    assert tau@np.ones(3) < 0  # dissipative damping
    assert np.max(np.abs(cartesian_impedance_torque(J,[1,1,1],[0,0,0],[0,0,0],torque_limits=np.ones(3)))) <= 1
    with pytest.raises(ValueError):
        cartesian_impedance_torque(J,[1,1],[0,0,0],[0,0,0])


@pytest.mark.parametrize("seed",[7,17,27])
def test_real_contact_pick_place(seed):
    from demos.physical_pick_place import run
    report = run(seed)
    assert report["status"] == "PASS_REAL_SIM"
    assert report["max_object_height_m"] > .14
    assert report["max_finger_contact_force_N"] > .1
    assert report["object_final"][2] < .04


@pytest.mark.parametrize("scenario",["straight","obstacle","delay","fault"])
def test_real_navigation_scenarios(scenario):
    from demos.physical_navigation import run
    report = run(7,scenario)
    assert report["status"] == "PASS_REAL_SIM"
    assert report["tracking_rmse_m"] < .08
    if scenario == "fault":
        assert report["recovery_count"] == 1 and report["fault_injected"]


@pytest.mark.parametrize("seed",[7,17,27])
def test_same_world_mobile_navigation_grasp_place(seed):
    from demos.mobile_manipulation import execute
    report = execute(seed)
    assert report["status"] == "PASS_REAL_SIM"
    assert report["task"]["completed"][0] == "NAVIGATE"
    assert report["base_final"][0] > .45
    assert report["max_object_height_m"] > .14


def test_physical_impedance_contact_and_admittance():
    from demos.impedance_contact import run
    baseline = run(7)
    controlled = run(7,admittance=True)
    assert baseline["status"] == controlled["status"] == "PASS_REAL_SIM"
    assert controlled["force_rmse_N"] < .2
    assert controlled["force_rmse_N"] < baseline["force_rmse_N"]*.1


def test_physical_grasp_failure_recovery_same_world():
    from demos.mobile_manipulation import execute
    report = execute(7,grasp_fault=True)
    assert report["status"] == "PASS_REAL_SIM"
    assert report["task"]["recoveries"] == 1
    assert report["task"]["failures"][0]["stage"] == "GRASP"
