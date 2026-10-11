# Original-scope acceptance review — 2026-10-11

This is a read-only source/evidence audit against the user's original pasted brief, not a replacement scope. Only this review file was written. The current software stage is reviewable, but it does **not** satisfy the original final stopping condition: outstanding software items remain alongside PENDING_HARDWARE.

A PASS accepts only the stated experiment/component and its stated envelope. PASS_UNIT, PASS_MOCK and PASS_REAL_SIM are not interchangeable. PARTIAL means implementation or evidence is incomplete; it is not an environmental BLOCKED. No current missing simulator/component is classified BLOCKED: both native simulator and ROS environments actually ran. Physical vehicle/robot, manufacturer protocol validation, Raspberry Pi HIL and physical E-stop are PENDING_HARDWARE.

Counts below use named acceptance checks as the denominator, splitting compound requirements where their statuses differ. They are not percentages of engineering effort. Repeated requirements in different original sections are intentionally retained. The 16 week rows and the final §12 gates are separate denominators, not added to the detailed ledger. Historical W01–W05 assignments that the brief does not define precisely are inferred from repository history; an inferred mapping is not a new user requirement. Optional Falcon IPPO/MAPPO is not in the mandatory denominator and remains unimplemented.

## Material findings and remaining software

1. Real MuJoCo physics is accepted. The mature licensed Panda uses actuator commands and mj_step; the object remains a free body with physical finger contact, lift and release. The physical base uses planar generalized-force/velocity servos with inertia and force bounds. It is not a wheel/slip or identified manufacturer drivetrain. Legacy overwritten-coordinate demonstrations are explicitly not used as physical acceptance evidence.
2. Current arm safety supersedes the earlier bypass finding. PandaSimulation.step now applies the shared supervisor every physics step. halt latches stop, holds measured joints and zeros mobile commands. The actual regression test verifies less than 0.02 rad motion during a stop and a 20 ms simulation deadline interrupt within 24 ms; ctrl equals measured joints on timeout. [Source](../embodied_swarm/hal/panda_simulation.py) and [actual test](../tests/test_physical_stack.py) document this. The task also supplies an 80 s simulation deadline and abort callback. General TaskExecutor still cannot preempt an arbitrary blocking third-party callback; only the implemented physics-step path supplies that guarantee. Do not call it a general asynchronous BT engine.
3. The sequential TaskExecutor is a useful bounded task framework with result, failure, retry, recovery, timeout and abort. It does not implement a general behavior tree, persistent resumable Task State or symbolic task-and-motion search. Five same-world mobile tasks and actual failed-grasp recovery pass, but navigation is a fixed straight approach, localization is simulator truth and random object offsets are ±15 mm. Changed-obstacle long tasks, generalized layouts and distributed ROS navigation/manipulation remain PARTIAL.
4. Native ROS2/ros2_control/MoveIt2/OMPL is genuine. Seven arm joint names, positions and limits agree with Panda/Menagerie, actual RRTConnect plans execute in MuJoCo, software-stop/stale-state/invalid-joint faults are exercised, and real bag data replay. The ROS hand is GenericSystem mock; it does not drive the physical fingers. The separate physical mobile task does not use MoveIt planning-scene object/base synchronization or ROS task orchestration. Do not merge the two passing experiments into a claim of ROS-controlled mobile Pick-and-Place with object collision safety.
5. FK/Jacobian/IK cross-validation, actual C++ kernels and performance are accepted. The C++ library is not used as the live physics or ROS-arm controller hot path. FK/Jacobian parity covers 30 configurations; controller parity should not be inferred from it. At 10,000 matched FK+Jacobian calls, P50 Python/C++ is 363.675/0.758 µs; this benchmark is for that kernel, not the entire robot loop.
6. W09 still lacks an independent simulated sensor/encoder/IMU state-estimator experiment and measured robot calibration/identification. Current states are physics ground truth, and prepared hardware tools are not fitted robot payload, friction, joint, sensor/extrinsic or delay calibration.
7. P1 residual PPO is actual training (7,680 control steps, one training seed, real weights/reward curve), with three held-out seeds and stiffness OOD. Accept only the bounded vertical contact residual study. PPO force RMSE about 0.49 N and classical admittance 0.03 N use different evaluation windows; do not imply a general RL grasp/swarm result or an apples-to-apples global ranking. More training seeds would strengthen the result but do not invalidate the minimum experiment already run.
8. Release stage/selection/rollback tools and first-week commissioning guide exist. A joined running-process safe-stop plus deployment rollback remains unaccepted. Physical hardware, force-sensor calibration, target Pi timing and physical E-stop are PENDING_HARDWARE.

## Current evidence and claim boundaries

- Final independent host regression: [220 passed in 20.69 s](../artifacts/final_regression_profile.stdout.txt); an isolated staged release separately passed 220 tests in 12.15 s. These are observed runs, not anticipated counts.
- C++: four release GoogleTests and four ASan/UBSan passes, 30 FK/Jacobian parity configurations.
- [Physical benchmark](../artifacts/physical/benchmark.json): 40 runs (20 navigation, five fixed-arm, five same-world mobile, ten force-contact), five seeds; 100/100 reachable IK targets. These are a bounded benchmark set, not general task reliability estimates.
- Latest native MuJoCo ROS execution: 12 trajectory points, max error 0.000950414 rad; nominal latency 476 samples, P50/P95/P99 0.693767/1.054192/3.267442 ms; controller-period jitter standard deviation 0.257761 ms. Whole-run fault samples and replay counts are separately retained in [ROS report](ROS2_ACCEPTANCE.md).
- README now uses current ROS numbers and states the physical versus ROS task boundary. The architecture arrow through Safety should be read as the intended stable contract; separate torque-contact research and every future adapter are not automatically covered by the accepted Panda position-command supervisor. Final remote CI status and publication SHA remain coordinator-owned evidence.

## Counts with explicit denominators

Detailed ledger: **140/155 checks have passing evidence**, 15 PARTIAL, 0 current FAIL, 0 BLOCKED. Passing evidence by kind: 59 PASS_UNIT, 78 PASS_REAL_SIM, 3 PASS_MOCK. Historic failed experiment rows remain in raw logs and the findings above; a corrected current check is not counted as a current FAIL.

| Original section/check group | Denominator | PASS checks | PARTIAL checks |
| --- | --- | --- | --- |
| §5.1 Navigation | 11 | 10 | 1 |
| §5.2 Arm kinematics | 10 | 10 | 0 |
| §5.3 Genuine MuJoCo | 13 | 13 | 0 |
| §5.4 ROS2/MoveIt2/ros2_control | 18 | 18 | 0 |
| §5.5 Task/TAMP | 16 | 14 | 2 |
| §5.6 Control/learning | 7 | 6 | 1 |
| §3/W08 C++ core engineering | 11 | 11 | 0 |
| §6 Stable architecture | 6 | 5 | 1 |
| §7 Hardware preparation | 16 | 15 | 1 |
| §8 Test classes | 6 | 5 | 1 |
| §8 Robot benchmark metrics | 8 | 8 | 0 |
| §8 Engineering metrics | 7 | 7 | 0 |
| §9 Portfolio | 13 | 13 | 0 |
| §10–13 Delivery | 10 | 5 | 5 |
| Additional W09 estimation/calibration | 3 | 0 | 3 |

The requested week matrix has **12/16 PASS and 4/16 PARTIAL** under the stated week-level scope. This count is not a claim that 12/16 of the entire project is complete.

| Week | Current status | Acceptance boundary |
| --- | --- | --- |
| W01 | PASS_UNIT | SI state/frames/HAL; historical exactweekallocation inferred |
| W02 | PASS_REAL_SIM | Actual physicalbackend separatefromlegacymock |
| W03 | PASS_UNIT | Gridnavigation, physicalA*integration |
| W04 | PASS_UNIT | HybridA*/LQR/replanningtests; physicalbenchA*/pursuit |
| W05 | PASS_REAL_SIM | NativeROS2controller_manager/state/action/safety/recordreplay; mockdistinct |
| W06 | PASS_UNIT | FK/Jac cross-validation andHIL-readyinterface; physicalHILpending |
| W07 | PASS_REAL_SIM | ActualMoveItRRTConnect andDLSIK |
| W08 | PASS_UNIT | ActualC++library4release+4sanitize, parityandperf; notliveoffloadclaim |
| W09 | PARTIAL | TrueMuJoCoPASS; estimation/calibrationnotaccepted |
| W10 | PASS_REAL_SIM | Safetyperphysicalstep/armstop/deadline plusROSfaults; hardwareestoppending |
| W11 | PARTIAL | Sequentialtaskframeworkaccepted; generalBT/symbolicTAMPmissing |
| W12 | PARTIAL | Sameworldfixedapproach/recoveryaccepted; changedobstacleflexibletask/distributedROSnotaccepted |
| W16 | PASS_REAL_SIM | RealPanda contact torqueimpedance/admittance/disturbance |
| W17 | PASS_REAL_SIM | ActualminimalresidualPPO7680steps; one training seed |
| W18 | PASS_REAL_SIM | Boundedheldoutseed/stiffnessOOD; nogeneralizationclaim |
| W19 | PARTIAL | Localengineeringaccepted; finalremoteCI/push/targetdeploymentclosurepending |

## Original §12 minimum visible gates

These are exactly the original 11 project gates. A small set of visible gates can pass while detailed §3–§9 software remains open. Count: **9/11 PASS**, 2 PARTIAL.

| Original gate | Status | Evidence/boundary |
| --- | --- | --- |
| C++ compiles/unit tests | PASS_UNIT | Actual4release+4sanitize |
| MuJoCo actual physical loop | PASS_REAL_SIM | Forceactuators/contact |
| FK/IK/Jacobian unit tests | PASS_UNIT | Cross-validation100IK/30C++parity |
| ROS2/ros2_control | PASS_REAL_SIM | Nativecontrollers/plugin |
| MoveIt2/OMPL | PASS_REAL_SIM | ActualRRTConnect/actionexecution |
| BT/TAMP | PARTIAL | Sequentialframeworkvalid; generalBT/taskmotionsolvernotimplemented |
| Navigation/manipulation task | PASS_REAL_SIM | One-worldfixedstraightapproach5seeds; broaderW12partial |
| Safety/failure recovery | PASS_REAL_SIM | Per-stepPandasupervisor/deadline plusactualfailedgraspandROSfaults |
| Benchmark/logs | PASS_REAL_SIM | 40physicalrunsandrawdata |
| Complete README | PASS_UNIT | Currenthomepage/reports; finalrootpublicationcheckseparate |
| Hardware Adapter Ready | PARTIAL | Contract/toolsready; runningprocessstop+rollbackjoinnotaccepted; physicalpending |

## Detailed acceptance ledger

Each row is the stated check, status and actual evidence/boundary. Passing a unit check does not upgrade it to physical or target-platform validation.

### §5.1 Navigation

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Dijkstra | PASS_UNIT | Inherited grid planner tests |
| A* | PASS_REAL_SIM | Force-actuated obstacle navigation |
| Hybrid A* | PASS_UNIT | Forward-only unit/demo coverage; not used in physical benchmark |
| Path smoothing | PASS_UNIT | Collision-aware legacy planner tests |
| Pure pursuit | PASS_REAL_SIM | Actual physics navigation20runs |
| LQR tracking | PASS_UNIT | Unit/demo coverage |
| Obstacle map | PASS_REAL_SIM | Inflated static physical scene |
| Replanning | PARTIAL | Legacy reactive demo passes; changing-obstacle physical task not accepted |
| Robot state/command interfaces | PASS_UNIT | Stable SI RobotAdapter |
| Multiple physical scenarios | PASS_REAL_SIM | 20runs:straight/obstacle/delay/fault×5seeds |
| Trajectory error and planning runtime | PASS_REAL_SIM | Measured per-run logs/summary |

### §5.2 Arm kinematics

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Forward kinematics | PASS_REAL_SIM | Independent NumPy vs mj_forward |
| Inverse kinematics | PASS_REAL_SIM | 100 reachable target cases |
| Jacobian | PASS_REAL_SIM | Independent geometric matrix vs mj_jacSite |
| Damped least squares IK | PASS_REAL_SIM | Adaptive damping and actual Panda target execution |
| Joint limits | PASS_UNIT | Model bounds/unit cases |
| Singularity handling | PASS_UNIT | Damping/unreachable tests |
| Manipulability | PASS_UNIT | Singular-value method/tests |
| Joint/Cartesian tracking | PASS_REAL_SIM | Contact/trajectory tracking |
| Mature standard model reuse | PASS_REAL_SIM | Pinned licensed Menagerie Panda; MoveIt Panda URDF |
| Cross-validation against existing math library/model | PASS_REAL_SIM | MuJoCo reference, finite differences and C++ |

### §5.3 Genuine MuJoCo

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| MJCF model load | PASS_REAL_SIM | MuJoCo3.12 actual model16/19qpos |
| Joint state read | PASS_REAL_SIM | mjData actualqpos/qvel |
| Control send | PASS_REAL_SIM | ctrl position/torque actuators |
| Simulation advance | PASS_REAL_SIM | mj_step; no acceptance trajectory overwrite |
| Actual contact physics | PASS_REAL_SIM | Finger/freeobject/floor contact and forces |
| Mobile base motion | PASS_REAL_SIM | Force-limited planar servos, inertia; not wheel/slip drivetrain |
| Arm joint control | PASS_REAL_SIM | Seven Panda physical actuators |
| Grasp/release | PASS_REAL_SIM | Physical finger contact, lift and release |
| Control-error recording | PASS_REAL_SIM | Per-run joint/path errors |
| Disturbance/collision tests | PASS_REAL_SIM | 2Nm disturbance/contact and obstacle checks |
| Video/GIF export | PASS_REAL_SIM | EGL actual physics frames |
| Complete physical mobile loop | PASS_REAL_SIM | 20navigationruns |
| Physical pick/place chain | PASS_REAL_SIM | 5fixedarm+5sameworldmobiletasks |

### §5.4 ROS2/MoveIt2/ros2_control

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| ROS2 package | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| Topic | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| Service | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| Action | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| TF2 | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| URDF/Xacro | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| JointState | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| Controller Manager | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| ros2_control | PASS_REAL_SIM | Actual WSL Humble build/launch acceptance; docs/ROS2_ACCEPTANCE.md |
| Mock Hardware Interface | PASS_MOCK | Compiled SystemInterface mock backend and actual ROS controllers |
| MoveIt2/OMPL | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |
| RRTConnect | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |
| Trajectory execution and abnormal handling | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |
| rosbag Record/Replay | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |
| Launch | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |
| Parameter configuration | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |
| Explicit MuJoCo bridge | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |
| Native Ubuntu/WSL compilation/execution | PASS_REAL_SIM | Actual native nodes/actions/physics bridge evidence; no Python pubsub mock |

### §5.5 Task/TAMP

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| BT or task planning framework | PARTIAL | Sequential TaskExecutor valid bounded framework; no general BT/symbolic TAMP motion/task search |
| Required navigation/localize/plan/grasp/move/place/verify chain | PASS_REAL_SIM | Physical same-world stages; ground-truth localization |
| Task State | PASS_UNIT | TaskResult completed/failures/currentstage logs; no persistent resumable task state |
| Failure detection | PASS_REAL_SIM | Open-finger failed-grasp physical criterion |
| Retry | PASS_REAL_SIM | Actual second attempt |
| Timeout | PASS_REAL_SIM | Panda per-mj_step sim deadline; .02s interruption test |
| Recovery | PASS_REAL_SIM | Actual fresh-state recovery count1 |
| Abort | PASS_REAL_SIM | Latched sim halt on abort/retry exhaustion |
| Task result | PASS_UNIT | Structured result with elapsed/recoveries/failures |
| Success criteria | PASS_REAL_SIM | Contact/lift/place error+height criteria |
| Fixed targets | PASS_REAL_SIM | Actual fixed pick/place |
| Random targets | PASS_REAL_SIM | Five seeded object offsets±.015m; limited randomization |
| Changing obstacles | PARTIAL | Physical navigation static obstacle scenarios; mobile task fixed straight approach |
| Success rate | PASS_REAL_SIM | 5/5fixed+5/5mobile nominal |
| Failure type and recoveries | PASS_REAL_SIM | Actual injected failure count1 |
| Execution runtime | PASS_REAL_SIM | Simulation/wall-time logs |

### §5.6 Control/learning

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Cartesian control | PASS_REAL_SIM | Panda actual torque impedance/IK |
| Impedance/admittance | PASS_REAL_SIM | Actual floorcontact force feedback |
| Force/position contact experiments | PASS_REAL_SIM | 10contactruns five seeds |
| Model perturbation and delay | PARTIAL | Base100msdelay and torque disturbance; full arm actuator/mass/delay identification not accepted |
| Residual RL minimal experiment | PASS_REAL_SIM | Actual clipped PPO7680controlsteps and weights, bounded vertical residual |
| OOD/domain randomization | PASS_REAL_SIM | 3heldoutseeds,150/450vs300N/mstiffness; bounded envelope |
| Visible physical contact | PASS_REAL_SIM | Actual force records/GIF; not IK-only |

### §3/W08 C++ core engineering

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Controllers | PASS_UNIT | Actual C++ pursuit and impedance |
| Trajectory/state data | PASS_UNIT | Eigen numeric inputs/outputs; common hardware contract remains Python |
| Robot kinematics model | PASS_UNIT | Panda FK/Jacobian |
| Kinematics/evaluation core | PASS_UNIT | Actual FK/Jacobian numerical core, no mandatory entirePythonrewrite |
| CMake | PASS_UNIT | Actual C++17releasebuild |
| GoogleTest | PASS_UNIT | 4/4releasecases |
| Compile warnings | PASS_UNIT | Warnings as errors |
| ASan/UBSan | PASS_UNIT | 4/4sanitizedcases |
| Python/C++ numeric parity | PASS_UNIT | 30randomFK/Jacobiancases; pursuit/impedance parity not claimed |
| Clear library interface | PASS_UNIT | cpp/include/embodied/core.hpp |
| Comparable performance | PASS_UNIT | 10000sameFK+Jacobiankernel calls; C++ .758usvsPython363.675usP50 |

### §6 Stable architecture

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Brand-independent algorithms | PASS_UNIT | RobotState/RobotCommand/HAL |
| Shared simulator/hardware contract | PASS_UNIT | RobotAdapter abstractinterface |
| Configuration switching | PASS_UNIT | Flags/YAML/backends |
| Unified time/units/frames/errors | PARTIAL | SI contracts and sim time/ROS stamps explicit; full ROS mobile task safety path not integrated |
| Mock/replay/real distinction | PASS_UNIT | Legacy base/Mock/MuJoCo labels |
| Future hardware support without invented protocol | PASS_UNIT | Manufacturer fields unknown/null |

### §7 Hardware preparation

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Vehicle/robot abstract adapter appropriate to project | PASS_UNIT | RobotAdapter |
| Mock I/O | PASS_MOCK | Fixture and HAL tests |
| ROS2/ros2_control Mock Hardware | PASS_MOCK | Actual compiled mock SystemInterface |
| Command/state interface | PASS_UNIT | RobotState/RobotCommand and joint interfaces |
| Configuration | PASS_UNIT | Hardware commissioned-null fields |
| Calibration scripts | PASS_UNIT | Offline analysis/tool; not measuredrobotcalibration |
| Motor step tools | PASS_UNIT | Bounded commissioning plan |
| Encoder/IMU recording | PASS_UNIT | Commissioning recorder |
| Latency measurement | PASS_REAL_SIM | Actual ROS stamps |
| Replay | PASS_REAL_SIM | Actual bag replay |
| Fault injection | PASS_REAL_SIM | Stale state/estop/actions/failedgrasp |
| Wiring/comms checklist | PASS_UNIT | First-week guide |
| Deployment scripts | PASS_UNIT | Immutable stage/selection |
| Rollback and safe stop joined mechanism | PARTIAL | Release rollback exists; runningprocess stop/rollback joined acceptance absent |
| Truthful readiness label | PASS_UNIT | Pending physical validation |
| Target HIL label | PASS_UNIT | No measured RaspberryPiHILclaim |

### §8 Test classes

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Unit | PASS_UNIT | 220latestrecorded fullsuite; root latest countmayincrease |
| Numerical | PASS_UNIT | FK/Jac/IK/C++parity |
| Integration | PASS_REAL_SIM | Physics and actualROS actions |
| Boundary | PASS_UNIT | State/config/limits/singular/unreachable tests |
| Failure | PASS_REAL_SIM | Actual failedgrasp, stalecommand, armestop/deadline, ROSfaults |
| Performance across all new core modules | PARTIAL | FK+Jac and actual ROS/physics measured; general task/planner longhorizon deadlines not accepted |

### §8 Robot benchmark metrics

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| Path tracking error | PASS_REAL_SIM | 20navigationruns |
| Planning success rate | PASS_REAL_SIM | Navigation logs |
| IK success rate | PASS_REAL_SIM | 100/100reachablecase set |
| Joint tracking error | PASS_REAL_SIM | Physical and ROSstats |
| Grasp/place success rate | PASS_REAL_SIM | 5+5tasks |
| Task completion rate | PASS_REAL_SIM | Sameworldnominal5/5 |
| Recovery count | PASS_REAL_SIM | Actualinjectedfailedgrasp1 |
| Planning/control runtime | PASS_REAL_SIM | Physical,ROS,C++kernels |

### §8 Engineering metrics

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| P50/P95/P99 | PASS_REAL_SIM | Runtime/ROS distributions |
| CPU/memory | PASS_UNIT | HostprofilingnotPi |
| Actual control-cycle jitter | PASS_REAL_SIM | controller_manager monotonic periods |
| ROS message latency | PASS_REAL_SIM | Samehoststamp-to-callback |
| Test success rate | PASS_UNIT | Versioned stdout220checkpoint |
| Fault recovery result | PASS_REAL_SIM | RealROSandphysics |
| Version/host/seed/sample/statistics metadata | PASS_UNIT | Reportsdistinctnominal/faultandperformance/rendered |

### §9 Portfolio

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| English homepage and Chinese navigation | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Problem/architecture/stack/algorithms in first30seconds | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Actual simulator and truthful measured data | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Architecture diagram | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Actual simulation screenshot | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Trajectory/performance plots | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Actual GIF/video link where generated | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Environment setup | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| One-command quick start/tests | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Benchmark and failure cases | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Limitations/software-vs-hardware boundary | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Hardware integration guide | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |
| Engineering lessons and checked paths/commands | PASS_UNIT | Rewritten README and linked guides/actual figures; final root publication acceptance separate |

### §10–13 Delivery

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| M1 baseline/core | PASS_UNIT | 161baselineprotected |
| M2 actual simulator/C++/ROS safety | PASS_REAL_SIM | Actual component acceptance |
| M3 complete scope | PARTIAL | Estimation/calibration/generalBT/TAMP/changingobstacletask gaps |
| M4 final publication | PARTIAL | Pages exist; finalremoteCI/pushroot-owned |
| Protected independent Git | PASS_UNIT | No destructive reset; parent backups/commits |
| Incremental verified non-force push | PARTIAL | Rootpublishing; finalremoteSHACIpending |
| Sensitive/oversize/cache scan | PARTIAL | Rootfinalpublicationcheck |
| FINAL_DELIVERY_REPORT | PASS_UNIT | Present with checkpoint and partial statuses |
| INTERVIEW_TECHNICAL_GUIDE | PASS_UNIT | Present |
| Final commit/GitHub/matrix/evidence boundaries | PARTIAL | SnapshotSHApresent; pushedfinalrootreportpending |

### Additional W09 estimation/calibration

| Check | Status | Evidence or remaining requirement |
| --- | --- | --- |
| State estimation beyond simulator truth | PARTIAL | Physics and ROSstatesdirectgroundtruth; no independent simulated IMU/encoder estimator |
| Measured robot calibration | PARTIAL | Panda exactmodelused; no fittedpayload/joint/friction/extrinsic/sensorparameters |
| Unified MoveIt mobile task model/world safety | PARTIAL | Sevenarmjointlimitsconsistent; separate ROSarmhandmock vs physicalmobilehand; no ROSplanning-sceneobject/base/worldsync |

## Closure rule

Do not mark the original task complete while any mandatory software PARTIAL above remains. Tonight's verified stage may be published with these boundaries and preserved evidence. Resolve software failures and missing experiments, update exact counts against the same original requirements, then record final remote commit/CI evidence. Hardware work stays PENDING_HARDWARE until performed on the real target.
