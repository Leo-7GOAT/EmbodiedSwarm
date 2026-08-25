# EmbodiedSwarm

EmbodiedSwarm is a mobile-manipulation planning and control portfolio project.
The W01/W02 foundation targets a differential-drive base, a Panda-compatible
7DoF arm, and a two-finger-gripper command boundary. MuJoCo headless physics is
the first executable backend; ROS2, MoveIt2, and real hardware remain later
integration stages.

## W01/W02 capabilities

- Validated, owned-snapshot `RobotState` and `RobotCommand` NumPy models.
- Explicit SI units and `None` meaning “no new command on this channel.”
- One abstract `RobotAdapter`: `reset`, `read_state`, `send_command`, `step`,
  and `shutdown`.
- Deterministic differential-drive `MockHardwareAdapter` with Euler base
  integration, a simple 7DoF position servo, and logical gripper behavior.
- `MujocoRobotAdapter` with correct body/world planar-velocity transforms,
  seven named position actuators, state readback, and logical gripper state.
- Panda-compatible W02 MJCF skeleton with `nq=10`, `nv=10`, and `nu=7`.
- One upper-level adapter-switch program that runs unchanged on both backends.
- A reviewable ros2_control `SystemInterface` skeleton for later Linux/ROS2
  integration.

The MJCF is intentionally a Panda-compatible kinematic/dynamic skeleton, not a
claim of official Franka meshes, inertias, transmissions, or gripper contacts.

## Architecture

```text
RobotState
    |
future Task / Planning / Control
    |
RobotCommand
    |
RobotAdapter
   |-----------------------|
MockHardwareAdapter   MujocoRobotAdapter
   |                       |
deterministic mock     headless MuJoCo physics
```

Algorithm layers consume only project-owned state, command, and adapter types.
Backend APIs remain below the Robot HAL boundary.

Frame order is:

```text
world -> base_link -> panda_link0 -> panda_link1 -> ... -> panda_link7
      -> panda_hand -> end_effector
```

Base pose is `[x_world, y_world, yaw]`. `RobotCommand.base_twist` and
`RobotState.base_twist` are body-frame `[vx, vy, wz]` in m/s, m/s, and rad/s.
Arm positions/velocities are rad and rad/s; gripper width is m; timestamps are s.

## Install, test, and run

Python 3.10 or newer is supported.

```bash
python -m pip install -e ".[dev]"
python -m pytest tests -v
python -m demos.adapter_switch_smoke
python -m demos.mujoco_physics_smoke
python -m demos.mujoco_smoke --duration 5
```

The switch smoke must print:

```text
PASS: MockHardwareAdapter
PASS: MujocoRobotAdapter
ADAPTER SWITCH CONTRACT PASS
```

MuJoCo headless physics is the portable W02 acceptance path. The finite-duration
interactive GLFW smoke using the actual mobile-manipulator MJCF was also
validated on the current local Windows environment. Viewer behavior remains
graphics-driver dependent; headless tests remain authoritative on machines
where GLFW rendering is unavailable or corrupted.

## ROS2 skeleton status

`ros2/embodied_swarm_hardware` documents and scaffolds a future
`hardware_interface::SystemInterface`. It is skeleton only and was not built on
the current Windows environment. Its transport hooks deliberately report
unconfigured until a real base/Franka driver is supplied; no hardware support
is claimed.

## Acceptance boundary

W03 will introduce a classical navigation baseline. A*/Dijkstra, Hybrid A*,
MoveIt2/OMPL, FK/IK/Jacobians, grasp planning, BT/TAMP, ROS2 runtime integration,
real robot control, impedance control, and multi-robot coordination are not
implemented or claimed in W01/W02.
