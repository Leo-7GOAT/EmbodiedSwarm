# embodied_swarm_hardware

This ROS2 Humble package was built and executed in Ubuntu 22.04 on WSL2.
The seven-joint C++17 SystemInterface supports two explicitly named backends:
`mock` (rate-limited software hardware) and `mujoco` (real ROS topic transport
into the MuJoCo Panda physics bridge). Physical deployment is PENDING_HARDWARE.

See [actual commands, measurements and limitations](../../docs/ROS2_ACCEPTANCE.md).
The MoveIt model is the official moveit_resources Panda URDF/SRDF and OMPL uses
RRTConnect; no handwritten planner is presented as MoveIt.

From the repository root in WSL:

```bash
bash ros2/verify.sh
```

The script builds the C++ plugin, runs two GoogleTest cases, then exercises real
MoveGroup/ExecuteTrajectory/FollowJointTrajectory actions, JointState, TF2,
software estop/reset services, stale-state fault/recovery, and rosbag replay.
`ros2/evidence` contains raw launch logs, bags, environment and measured JSON.

Interface units are radians and radians/second, `panda_joint1` through
`panda_joint7`. `/mujoco/joint_commands` and `/mujoco/joint_states` use
sensor_msgs/JointState, with wall-clock ROS stamps. The bridge maps these
names to Menagerie `joint1` through `joint7`; commands only write actuator
`ctrl`, states come from `mj_step`. Arm gravity/Coriolis feed-forward uses
MuJoCo bias forces. It never rewrites moving qpos.

The ROS arm acceptance uses a mock hand; physical grasp/place is separately
accepted by the project MuJoCo pick-place suite. The base is also accepted
separately by the project's physics navigation suite. This plugin exports
seven arm position command interfaces and fourteen position/velocity state
interfaces; it does not export fictitious base channels or claim a vendor
motor/CAN driver. A physical vendor transport, encoder/IMU synchronization,
independent hardware emergency stop and target-platform timing remain pending.
