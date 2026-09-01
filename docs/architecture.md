# EmbodiedSwarm architecture and interface contracts

## Layer boundary

```text
occupancy map / task state
    -> grid or continuous-pose planning
    -> remaining-path validation and replanning
    -> path tracking control
    -> RobotCommand
    -> RobotAdapter
    -> mock hardware / MuJoCo / future ROS2 hardware
```

`RobotState`, `RobotCommand`, and `RobotAdapter` are the project-owned boundary.
Planning and control code does not import MuJoCo or a future driver API.

## State and command

`RobotState` contains timestamp, World-frame planar base pose, Body-frame base
twist, seven arm joint positions/velocities, and gripper width. Construction
validates exact shapes and finite values and owns copies of all arrays.

`RobotCommand` contains a Body-frame base twist plus optional seven-joint
position and gripper-width targets. `None` means no new command on that channel;
zero remains a real command.

## Navigation stack

`OccupancyGrid` owns resolution and World/grid conversion. Dijkstra and A*
provide four-connected grid baselines. Bresenham line-of-sight supports path
simplification; iterative smoothing accepts an update only when both adjacent
continuous segments remain collision-free.

Hybrid A* searches a discretized `(row, column, yaw_bin)` key while carrying a
continuous `(x, y, yaw)` pose. Each expansion simulates forward unicycle
primitives at negative, zero, or positive angular rate. Every integration sample
is checked against the occupancy map; reconstruction concatenates the accepted
continuous primitive trajectories.

`replan_if_needed()` validates the remaining path after a map update. If
invalid, it calls Hybrid A* from the current robot pose. This is reactive map
replanning, not predicted dynamic-obstacle or spatiotemporal planning.

## Control stack

Pure Pursuit selects a lookahead point, transforms it into Body coordinates,
and commands `omega = v * curvature`.

The LQR baseline uses `e = [e_y, e_yaw]^T`, a discrete constant-speed error
model, iterative discrete Riccati solution, `u = -K e`, and angular-velocity
saturation. `Q` and `R` expose tracking-error versus control-effort tuning.

## Differential-drive and MuJoCo backends

Both backends reject nonzero lateral Body velocity. The mock integrates:

```text
x_dot_world = vx_body cos(yaw)
y_dot_world = vx_body sin(yaw)
yaw_dot     = wz
```

MuJoCo stores base x/y generalized velocities in World coordinates, so the
adapter rotates commanded Body velocity into World before stepping and rotates
readback velocity back into Body. Pure transforms live in `core/frames.py`.

The MJCF exposes seven Panda-compatible joints and position actuators. Gripper
behavior is logical only; the model contains no finger contact physics.

## Safety boundary and limitations

- The current Hybrid A* occupancy check treats the robot as a point; the W04
  integration demo inflates injected obstacles to provide tracking clearance.
- Pure Pursuit and LQR are kinematic baselines without actuator delay, wheel
  slip, state-estimation noise, or formal safety guarantees.
- Hybrid A* is forward-only with three steering rates and no analytic expansion.
- ROS2 hardware transport, official Franka dynamics, calibration, safety
  supervision, real robot control, and multi-robot coordination are not
  implemented or claimed.
