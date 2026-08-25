# embodied_swarm_hardware (skeleton only)

This directory is a truthful W02 ros2_control design skeleton. It was not built
or run on the current Windows environment and does not claim a connected robot.
The C++ transport hooks deliberately return failure until a real backend is
wired on Linux with ROS2 and ros2_control.

## Interface mapping

The planned `SystemInterface` exports:

- seven arm joint position command interfaces;
- seven arm joint position and velocity state interfaces;
- `base_vx/velocity` and `base_wz/velocity` command/state interfaces.

The base is differential drive, so no `base_vy` command interface is exported.
A controller writes body-frame `vx` and `wz`; the hardware plugin maps them to
`RobotCommand.base_twist = [vx, 0, wz]`.

`read()` maps a transport snapshot as follows:

```text
backend RobotStateBuffer.base_pose       -> diagnostic/odometry owner
backend RobotStateBuffer.base_twist[0,2] -> base_vx/base_wz state
backend RobotStateBuffer.q[0..6]         -> joint position state
backend RobotStateBuffer.dq[0..6]        -> joint velocity state
```

`write()` maps ros2_control command storage as follows:

```text
base_vx/base_wz command -> RobotCommandBuffer.base_twist [vx, 0, wz]
joint position command  -> RobotCommandBuffer.joint_position_targets
```

## Real-hardware integration path

1. Add a transport implementation behind `read_backend()`/`write_backend()`.
2. For a real base, convert body twist to the vendor motor/CAN protocol and
   return timestamped odometry/encoder state.
3. For Franka, connect the supported driver/controller path and preserve the
   seven joint position/state interface contract.
4. Declare matching joints/interfaces in URDF, load the plugin, then validate
   limits, timeouts, E-stop/manual mode, and read/write failure propagation.
5. Build and test on the target Ubuntu/ROS2 environment before changing this
   document from skeleton status.

No socket, CAN ID, vendor SDK, safety behavior, or successful build is invented
here. Those require the selected physical platform and are later acceptance
gates.
