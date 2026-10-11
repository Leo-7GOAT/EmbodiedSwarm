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

The model uses the [official MoveIt Panda description and semantic configuration](https://github.com/moveit/moveit_resources). MuJoCo uses the project's licensed [Menagerie Panda asset](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda), loaded from `assets/mjcf/panda_pick_place.xml`. The bridge maps ROS `panda_joint1..7` to model `joint1..7`. SI angular units are radians/radians per second. Before the first physics step, it reads the same URDF/Xacro initial joint configuration as ros2_control and sets physical qpos, zero velocity and position-actuator ctrl once. This is a documented reset initial condition; all moving state subsequently comes from actuator-driven `mj_step`. The object's free-body physics is not gravity-compensated.

`ros2/embodied_swarm_hardware` now contains a working plugin, package manifest, URDF/Xacro, controller YAML, launch, physics bridge, ROS action client, and GoogleTest. The old transport hooks that always returned false were replaced. Base navigation is accepted separately; this plugin exports seven arm position commands and fourteen position/velocity states, without fictitious base motor interfaces.

## Reproduce in Ubuntu 22.04 / WSL2

Dependencies on the tested host: ROS Humble under `/opt/ros/humble`; Python 3.10.12; MuJoCo 3.12.0; colcon; CMake; GCC. Tested versions include MoveIt core/OMPL 2.5.9, controller_manager 2.54.0, JointTrajectoryController 2.53.3. CPU is AMD Ryzen 5 9600X, 12 logical CPUs; WSL2 Linux 6.18.40.1. Raw versions, CPU and memory information are in `ros2/evidence/environment.txt`.

```bash
# After configuring the official ROS2 Humble apt repository:
sudo apt-get install -y ros-humble-ros-base ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-moveit-resources-panda-description \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
  python3-colcon-common-extensions python3-numpy python3-pytest time
python3 -m pip install mujoco==3.12.0

# Run from the repository root in Ubuntu/WSL, not PowerShell:
bash ros2/verify.sh

# Individual entry points:
bash ros2/build.sh
bash ros2/accept.sh --backend mock
bash ros2/accept.sh --backend mujoco
```

The builder creates `$HOME/embodied_acceptance_ws`; override with `EMBODIED_ROS_WS`. It refuses to overwrite an unrelated package. `verify.sh` builds, runs GoogleTest and real-physics initialization pytest, saves versions/resources, runs both backends, records topics, and replays bags into an actual rclpy subscriber. It uses localhost ROS domain 73 and domain 75 for C++ tests. All spawned processes, including the acceptance action client, are tracked by their process groups and stopped at completion or timeout, without killing unrelated ROS nodes. Startup uses measured conditions instead of fixed sleep durations. Result JSONs are removed at the start of a new run and `run_status.json` records RUNNING/PASS/FAIL, so a failed CI artifact cannot reuse a checked-out old PASS JSON.

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
| `/controller_manager/list_controllers` | Both broadcaster and arm controller must actually reach active before planning; bounded condition wait |
| `/check_state_validity` | MoveIt validates the fresh measured complete start state, retaining self-collision checks |

Finite joint limits and complete measured state are enforced. Out-of-bounds/nonfinite commands latch hold. A state age over 250 ms latches hold; future stamps over 50 ms and old stamps are rejected. The physics bridge also holds after its own command timeout. `software_estop` is software-only: no independent physical emergency stop has been validated.

Three GoogleTest cases passed: malformed configuration rejection; mock rate-limit plus invalid-command latch (2 rad/s at a 10 ms period, no automatic recovery); and actual DDS physics-state/reset ordering (first-packet absence and stale packet reject reset, fresh packet permits explicit reset, timeout rejects again). Four pytest cases use actual MuJoCo: the no-keyframe project scene matches the shared initial contract after 500 physics steps, and NaN/out-of-bounds/wrong-shape initial conditions are rejected without mutation. CTest runs two test targets containing seven actual cases; colcon reports nine results including wrappers. The separate two Python XML/model contract tests and full project Python/C++/physics numbers have different denominators.

## Latest measured evidence

Sources of every number: `ros2/evidence/mock/acceptance.json`, `ros2/evidence/mujoco/acceptance.json`, and adjacent replay JSON/logs/bag metadata. These are single ROS arm trajectory runs, not a multi-seed motion-planning success-rate benchmark. OMPL sampling was not given a fixed RNG seed; the target joint displacement and safety assertions are fixed.

| Measurement | Mock ros2_control arm | Real MuJoCo arm |
|---|---:|---:|
| Planning / execution error codes | 1 / 1 | 1 / 1 |
| RRTConnect trajectory points | 12 | 12 |
| Startup home contract max error | 0 rad | 1.572e-15 rad |
| Startup hardware reset attempts / actual refusals | 1 / 0 | 6 / 5, each refusal Fresh state required |
| Measured MoveIt start-state validity | true | true |
| Planning action wall time | 29.419 ms | 22.396 ms |
| Trajectory execution wall time | 1.097 s | 1.127 s |
| Maximum final joint error | 0.00084362 rad | 0.00089210 rad |
| Nominal latency P50 / P95 / P99 | 0.447 / 0.749 / 4.562 ms | 0.515 / 0.850 / 1.581 ms |
| Nominal samples | 164 | 166 |
| Startup samples excluded from nominal | 17 | 102 |
| Whole-run latency including startup/faults P50 / P95 / P99 | 0.476 / 1.152 / 28.638 ms | 0.521 / 0.963 / 11.133 ms |
| Whole-run samples | 279 | 434 |
| Actual controller period P50 / P95 / P99 | 10.001 / 10.089 / 10.120 ms | 9.919 / 10.516 / 10.569 ms |
| Actual controller period jitter standard deviation | 0.050 ms | 0.270 ms |
| JointState subscriber interarrival jitter standard deviation | 3.764 ms | 2.110 ms |
| Estop trajectory error | -4, PATH_TOLERANCE_VIOLATED | -4, PATH_TOLERANCE_VIOLATED |
| Pause-state fault / stale reset refusal / recovery | Not applicable to mock | All passed |
| ROS broadcast vs physics joint maximum difference | Not applicable | 1.75e-15 rad |
| JointState messages received through real rosbag replay | 309 | 457 |

Latency is ROS wall publication-stamp to observer callback on the same WSL host, including DDS/executor scheduling. It is not planner-to-actuator latency. Nominal samples begin after complete fresh state, active-controller, TF, home-contract and MoveIt state-validity checks and include planning/execution plus 0.5 s settled observation; they stop before fault injection. Whole-run samples retain startup work, failure actions, services and delayed observer processing. Percentiles use NumPy's default linear interpolation. Control jitter uses the actual controller_manager `period` argument; subscriber jitter is listed separately.

An independent immediate-start repeat is retained under `ros2/evidence/startup_handshake_repeat_mujoco`: all action/fault/valid-start/replay checks passed; initial reset was refused once before the fresh physics packet, then explicitly succeeded. Max joint error is 0.00099465 rad; nominal 168 samples have P50/P95/P99 0.444/0.698/5.959 ms, controller-period jitter 0.271 ms and replay 377 messages. The earlier `startup_repeat_mujoco` remains a checkpoint with 164 nominal samples, P99 11.225 ms and 367 replay messages. Earlier pre-fix evidence had 476 nominal samples and 0.694/1.054/3.267 ms; those samples used a different startup window and are historical, not the current table. Another earlier instrumented fault stage had P99 around 267 ms. No measured lower percentile is a bound; no hard real-time or target-hardware latency claim follows.

GNU `time -v` resource logs for the latest complete verify record mock 6.76 s / 69% job CPU / 96,428 KiB maximum RSS and MuJoCo 9.30 s / 76% job CPU / 264,860 KiB maximum RSS. These include launch, action checks, recording, teardown and replay; CPU is the Linux job metric and maximum RSS is a maximum process measurement rather than a sum of the entire ROS process tree.

## Failures resolved and remaining boundary

Cloud run [38147716851](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38147716851) at `b1c13cc` failed with MoveIt `START_STATE_INVALID` (-26). This was a real self-collision between panda_hand and panda_link5, not a missing planner or an assumed discovery delay. The generated project MJCF has no home keyframe; the old bridge used zero qpos/ctrl, then controller activation captured a near-zero measured hold. The failed bag records joint4=-0.069837 and joint6=0.198276 instead of the URDF home -2.356194/1.570796 rad. Compact original logs, measured bag-state extraction and source provenance are preserved in `ros2/evidence/ci_failure_38147716851`.

The fix initializes physical state and actuator targets once from the same URDF contract, verifies fresh complete state/home alignment and invokes real MoveIt state validity before using that measured start state. Removing fixed startup sleeps also exposed broadcaster-before-arm activation; the client now polls controller lifecycle states with a deadline rather than asserting one early snapshot. Both native backends and a second MuJoCo cold start passed with unchanged collision checking, safety TTLs and physical-error thresholds. These local repeats fix and exercise the cause; latest remote CI acceptance is reported separately by the coordinator.

Subsequent cloud runs [38148702669](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38148702669) and [38148952528](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38148952528) stopped before initialization-test collection: pytest 9 removed the `path` hook argument used by the globally installed Humble `launch_testing` plugin. The plain MuJoCo test target now disables optional plugin autoload for that target only. All four physical assertions remain active; native colcon and an independent pytest 9.1.1 environment both passed them. No tolerance was relaxed and no test skipped.

The initial physics bridge actuator lookup mistakenly treated a tendon actuator sharing numeric transmission ID zero as an arm joint actuator. Filtering by MuJoCo joint transmission type fixed the mapping. Initial DDS discovery correctly latched the state watchdog; an explicit fresh-state reset was added before execution. A measured-position hold without bias-force feed-forward slowly followed gravity sag; the real arm bridge now supplies gravity/Coriolis compensation while retaining physical joint integration and actuator limits. Original replay received zero messages because the observer had a different ROS domain; its domain is now explicitly aligned before rclpy initialization. Subsequent real replay counts appear above.

The URDF collision model and MoveIt action execution are validated; online occupancy/perception updates are **PARTIAL** because no octomap sensor plugin is configured. The ROS hand is **PASS_MOCK** and the ROS base-controller route is **PARTIAL**; physical grasp/place and same-world mobile manipulation are separate genuine MuJoCo experiments. Hardware transport, encoder/IMU synchronization, independent hardware estop, calibration, target Raspberry Pi timing and HIL are **PENDING_HARDWARE / HIL-ready**, without a physical-validation claim.


## Fresh physics startup handshake and current CI

Cloud run [38149348217](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38149348217) exposed a separate DDS startup race: the broadcaster emitted its initial JointState 107.8 ms before the first physical JointState. Observer freshness therefore did not prove that the hardware plugin had received a physical packet. Compact raw logs, first-message extraction, source provenance and the refused reset are in `ros2/evidence/ci_failure_38149348217`. Old source sometimes passed; rerunning it did not resolve the cause.

At source `5861008419d985a0c3d85f4a4c91e52d1023dbfe`, MuJoCo activation explicitly latches hold and clears received-state readiness. Only a fresh physical packet plus explicit reset permits ACTIVE. Initial acceptance uses a bounded service handshake with actual refusals recorded; the later stale-reset refusal and recovery fault assertions remain unchanged. The complete local run recorded five refusals before success, and the independent repeat recorded one. Three C++ cases plus four physical Python cases pass. [Cloud ROS acceptance](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38150181527) and [software/MuJoCo matrix](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38150181642) both actually passed at that source.
