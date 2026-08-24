# Architecture

EmbodiedSwarm is planned around a mobile manipulator: a mobile base and a
6DoF or 7DoF robotic arm. The first target platform is a differential-drive
base with a Franka Panda 7DoF arm.

W01 defines the boundaries below. It does not implement the robot stack.

## System layers

```text
Task
  |
  v
Planning
  |
  v
Control
  |
  v
Robot HAL
  |
  v
Simulator / Hardware
```

Task definitions describe the intended outcome. Planning will later produce
base and arm motion. Control will translate planned motion into commands.

The Robot HAL will keep simulator and hardware details outside the planning
and control layers. MuJoCo is the first simulator target. ROS2, MoveIt2, and
ros2_control are later integration targets.

## State and command boundary

`RobotState` and `RobotCommand` are names for future shared interfaces. They
are architecture notes in W01, not implemented types.

The state boundary is intended to cover the mobile base, arm joints, and end
effector without tying upstream modules to one backend.

## Coordinate hierarchy

```text
world
  |
  v
base
  |
  v
arm
  |
  v
end_effector
```

This hierarchy records the frames that later model and integration work must
connect. Transform algorithms and a TF tree are outside the W01 scope.

