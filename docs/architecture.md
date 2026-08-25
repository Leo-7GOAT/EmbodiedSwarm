# W01/W02 architecture and interface contract

## Layer boundary

```text
Task (future)
  -> Planning (future)
  -> Control (future)
  -> RobotCommand
  -> RobotAdapter
  -> MockHardware / MuJoCo / future ROS2 hardware
```

`RobotState`, `RobotCommand`, and `RobotAdapter` are the project-owned boundary.
No task, planning, or control code should import MuJoCo or a future driver API.

## State and command

`RobotState` contains timestamp, world-frame planar base pose, body-frame base
twist, seven arm joint positions/velocities, and gripper width. Construction
accepts list-like inputs, validates exact shapes and finite values, and copies
all arrays. Adapter `read_state()` implementations create independent snapshots.

`RobotCommand` contains a body-frame base twist plus optional seven-joint
position and gripper-width targets. `None` means that no new command is provided
for that channel; zero is an actual command. Adapters copy incoming commands.

## Differential-drive base

Both W02 backends reject nonzero lateral body velocity `vy`. The mock applies
explicit Euler integration:

```text
x_dot_world = vx_body cos(yaw)
y_dot_world = vx_body sin(yaw)
yaw_dot     = wz
```

MuJoCo stores `base_x/base_y` generalized velocities in World, so the MuJoCo
adapter rotates commanded body velocity into World before stepping and rotates
readback velocity back into Body. The pure transforms live in `core/frames.py`.

## Arm and gripper

The MJCF exposes `panda_joint1..7` and matching
`panda_joint1_position..7_position` actuators. Joint targets flow through
`data.ctrl`; `qpos/qvel` flow back into `RobotState`.

W02 gripper behavior is logical only: both adapters preserve the command/state
contract, but the MJCF contains no finger joints or contact physics. This is
explicitly not a grasp-physics claim.

## Frames

Code and MJCF use the same chain:

```text
world -> base_link -> panda_link0 -> panda_link1 -> ... -> panda_link7
      -> panda_hand -> end_effector
```

## Backend switch

`demos/adapter_switch_smoke.py` passes each backend to the same function typed
against `RobotAdapter`. Tests repeat this contract and verify finite state,
array shapes, and all command channels. Simulator-specific logic remains below
the adapter boundary.

## Later scope

Navigation, manipulation planning, MoveIt2, a live ROS2 transport, official
Franka dynamics, real wheel/contact dynamics, calibration, safety supervision,
and hardware deployment are later milestones, not W02 claims.
