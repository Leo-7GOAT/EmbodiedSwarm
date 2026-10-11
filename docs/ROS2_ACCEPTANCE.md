# ROS2 / MoveIt2 / ros2_control acceptance

Validated on 2026-10-11 (Asia/Shanghai). **PASS_REAL_SIM** for the ROS arm trajectory pipeline, **PASS_MOCK** for the separately selected mock arm/hand, **PASS_UNIT** for C++ and model contracts, **PENDING_HARDWARE** for the physical robot and target-compute HIL.

中文摘要：已实际编译并运行 Humble、MoveIt2、OMPL RRTConnect、controller_manager、JointTrajectoryController、TF2 和 MuJoCo 关节物理闭环，实际记录并回放 rosbag。ROS 手指仍为显式 Mock；项目的物理抓放及同场景移动抓放由独立 MuJoCo 实验验收，不能将这里的单机械臂 ROS 验收扩展为全部移动操作系统或真机验收。

## Implemented and executed pipeline

```text
MoveGroup action / OMPL RRTConnect
    -> time-parameterized RobotTrajectory
    -> ExecuteTrajectory action
    -> JointTrajectoryController / FollowJointTrajectory
    -> C++17 EmbodiedSwarmSystem ros2_control SystemInterface
    -> /mujoco/joint_commands (real ROS DDS)
    -> Menagerie Panda position actuator ctrl + arm bias-force feed-forward
    -> MuJoCo mj_step physics
    -> /mujoco/joint_states (measured qpos/qvel)
    -> joint_state_broadcaster / JointState / TF2 / MoveIt state monitor
```

The model uses the [official MoveIt Panda description and semantic configuration](https://github.com/moveit/moveit_resources). MuJoCo uses the project's licensed [Menagerie Panda asset](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda), loaded from `assets/mjcf/panda_pick_place.xml`. The bridge maps ROS `panda_joint1..7` to model `joint1..7`. SI angular units are radians/radians per second. It initializes a model home keyframe once, then writes only actuator `ctrl` and arm feed-forward forces; moving state is never synthesized by assigning qpos. The object's free-body physics is not gravity-compensated.

`ros2/embodied_swarm_hardware` now contains a working plugin, package manifest, URDF/Xacro, controller YAML, launch, physics bridge, ROS action client, and GoogleTest. The old transport hooks that always returned false were replaced. Base navigation is accepted separately; this plugin exports seven arm position commands and fourteen position/velocity states, without fictitious base motor interfaces.

## Reproduce in Ubuntu 22.04 / WSL2

Dependencies on the tested host: ROS Humble under `/opt/ros/humble`; Python 3.10.12; MuJoCo 3.12.0; colcon; CMake; GCC. Tested versions include MoveIt core/OMPL 2.5.9, controller_manager 2.54.0, JointTrajectoryController 2.53.3. CPU is AMD Ryzen 5 9600X, 12 logical CPUs; WSL2 Linux 6.18.40.1. Raw versions, CPU and memory information are in `ros2/evidence/environment.txt`.

```bash
# After configuring the official ROS2 Humble apt repository:
sudo apt-get install -y ros-humble-ros-base ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-moveit-resources-panda-description \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-ament-cmake-gtest python3-colcon-common-extensions python3-numpy
python3 -m pip install mujoco==3.12.0

# Run from the repository root in Ubuntu/WSL, not PowerShell:
bash ros2/verify.sh

# Individual entry points:
bash ros2/build.sh
bash ros2/accept.sh --backend mock
bash ros2/accept.sh --backend mujoco
```

The builder creates `$HOME/embodied_acceptance_ws`; override with `EMBODIED_ROS_WS`. It refuses to overwrite an unrelated package. `verify.sh` builds, runs GoogleTest, saves versions/resources, runs both backends, records topics, and replays bags into an actual rclpy subscriber. It uses localhost ROS domain 73 and domain 75 for C++ tests. All spawned processes are tracked by their process groups and stopped at completion, without killing unrelated ROS nodes.

Manual launch after sourcing the installation:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/embodied_acceptance_ws/install/setup.bash"
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
ros2 launch embodied_swarm_hardware bringup.launch.py backend:=mock
# For physics: launch backend:=mujoco and, in a second sourced terminal:
ros2 run embodied_swarm_hardware mujoco_bridge.py \
  --model "$PWD/assets/mjcf/panda_pick_place.xml"
```

The MuJoCo backend intentionally requires a fresh measured-state reset after startup discovery or a latched fault. Use `/hardware/reset_safety` only after checking state freshness and fault cause.

## Real interfaces and failure tests

| Interface | Contract / executed check |
|---|---|
| `/joint_states` | Seven arm joints plus explicit mock-hand states; actual joint_state_broadcaster |
| `/mujoco/joint_commands` | sensor_msgs/JointState position commands, ordered by joint name, ROS wall stamps |
| `/mujoco/joint_states` | Actual physics positions, velocities and actuator forces |
| `/tf`, `/tf_static` | `world -> panda_link0` and URDF joint transforms; panda_link0 to panda_link8 lookup tested |
| `/move_action` | MoveGroup action; `ompl` pipeline, `RRTConnectkConfigDefault`, positive success error code |
| `/execute_trajectory` | ExecuteTrajectory action executing the planned RobotTrajectory |
| `/panda_arm_controller/follow_joint_trajectory` | Controller action; invalid joint set rejected, estop trajectory fails |
| `/hardware/software_estop` | SetBool; software hold latch; setting false cannot clear it |
| `/hardware/reset_safety` | Trigger; stale MuJoCo state rejects reset, fresh state permits explicit hold-position reset |
| `/mujoco/pause_state` | SetBool fault injection; physics continues, state output pauses |
| `/hardware/control_period` | Measured controller_manager update period in seconds, distinct from subscriber interarrival |

Finite joint limits and complete measured state are enforced. Out-of-bounds/nonfinite commands latch hold. A state age over 250 ms latches hold; future stamps over 50 ms and old stamps are rejected. The physics bridge also holds after its own command timeout. `software_estop` is software-only: no independent physical emergency stop has been validated.

Two GoogleTest cases passed: malformed configuration rejection (count, NaN, inverted bounds, missing parameter, unsupported backend), and mock rate-limit plus invalid-command latch (2 rad/s at a 10 ms period, no automatic recovery). CTest executes one binary containing those two cases; colcon's result summary counts an additional wrapper result. Two Python XML/model contract tests also passed. Full project Python/C++/physics numbers are reported separately in the final delivery report.

## Latest measured evidence

Sources of every number: `ros2/evidence/mock/acceptance.json`, `ros2/evidence/mujoco/acceptance.json`, and adjacent replay JSON/logs/bag metadata. These are single ROS arm trajectory runs, not a multi-seed motion-planning success-rate benchmark. OMPL sampling was not given a fixed RNG seed; the target joint displacement and safety assertions are fixed.

| Measurement | Mock ros2_control arm | Real MuJoCo arm |
|---|---:|---:|
| Planning / execution error codes | 1 / 1 | 1 / 1 |
| RRTConnect trajectory points | 12 | 12 |
| Planning action wall time | 52.214 ms | 51.511 ms |
| Trajectory execution wall time | 1.126 s | 1.126 s |
| Maximum final joint error | 0.00097946 rad | 0.00095041 rad |
| Nominal latency P50 / P95 / P99 | 0.659 / 0.916 / 2.207 ms | 0.694 / 1.054 / 3.267 ms |
| Nominal samples | 480 | 476 |
| Whole-run latency including faults P50 / P95 / P99 | 0.662 / 0.948 / 6.994 ms | 0.756 / 1.238 / 12.308 ms |
| Whole-run samples | 582 | 642 |
| Actual controller period P50 / P95 / P99 | 9.981 / 10.499 / 10.534 ms | 10.028 / 10.429 / 10.525 ms |
| Actual controller period jitter standard deviation | 0.248 ms | 0.258 ms |
| JointState subscriber interarrival jitter standard deviation | 1.973 ms | 2.474 ms |
| Estop trajectory error | -4, PATH_TOLERANCE_VIOLATED | -4, PATH_TOLERANCE_VIOLATED |
| Pause-state fault / stale reset refusal / recovery | Not applicable to mock | All passed |
| ROS broadcast vs physics joint maximum difference | Not applicable | 7.87e-14 rad |
| JointState messages received through real rosbag replay | 678 | 702 |

Latency is ROS wall publication-stamp to observer callback on the same WSL host, including DDS/executor scheduling. It is not planner-to-actuator latency. Nominal samples stop before fault injection; whole-run samples include failure actions, services and delayed observer processing. Percentiles use NumPy's default linear interpolation. Control jitter uses the actual controller_manager `period` argument; subscriber jitter is listed separately. An earlier instrumented run had fault-stage P99 around 267 ms, demonstrating that observation backlog can be much larger than the control period. No hard real-time or target-hardware latency claim follows from these measurements.

GNU `time -v` resource logs record mock 18.05 s / 49% job CPU / 72,252 KiB maximum RSS and MuJoCo 22.95 s / 61% job CPU / 261,396 KiB maximum RSS. These include launch, action checks, recording, teardown and replay; CPU is the Linux job metric and maximum RSS is a maximum process measurement rather than a sum of the entire ROS process tree.

## Failures resolved and remaining boundary

The initial physics bridge actuator lookup mistakenly treated a tendon actuator sharing numeric transmission ID zero as an arm joint actuator. Filtering by MuJoCo joint transmission type fixed the mapping. Initial DDS discovery correctly latched the state watchdog; an explicit fresh-state reset was added before execution. A measured-position hold without bias-force feed-forward slowly followed gravity sag; the real arm bridge now supplies gravity/Coriolis compensation while retaining physical joint integration and actuator limits. Original replay received zero messages because the observer had a different ROS domain; its domain is now explicitly aligned before rclpy initialization. Subsequent real replay counts appear above.

The URDF collision model and MoveIt action execution are validated; online occupancy/perception updates are **PARTIAL** because no octomap sensor plugin is configured. The ROS hand is **PASS_MOCK** and the ROS base-controller route is **PARTIAL**; physical grasp/place and same-world mobile manipulation are separate genuine MuJoCo experiments. Hardware transport, encoder/IMU synchronization, independent hardware estop, calibration, target Raspberry Pi timing and HIL are **PENDING_HARDWARE / HIL-ready**, without a physical-validation claim.
