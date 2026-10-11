# Delivery report — measured software stage, 2026-10-11

Repository: [Leo-7GOAT/EmbodiedSwarm](https://github.com/Leo-7GOAT/EmbodiedSwarm). Behavior-tree source/evidence checkpoint: `3d61ea4bdf1aeccbc3afd83b21cb4f3b6bcca6a4`; estimator checkpoint: `65fa747808e5398b6131cf73f7eec90f4ce9686e`. Subsequent report commits are recorded in Git history. Use `git rev-parse HEAD` for the checked-out release SHA; the final delivery message gives the exact published SHA. Never interpret this report as physical robot or target-platform HIL validation.

## Stage gates and completion denominator

The current component stage has passing evidence for baseline/contracts, navigation, independent kinematics, force-driven physics/contact grasp, C++, actual ROS2/MoveIt, safety/recovery, impedance, independent base estimation and a tickable memory behavior tree. These component checks do not complete the original roadmap. The full sixteen requested milestone groups (W01–W12, W16–W19) contain **13 PASS and 3 PARTIAL** (W09, W12, W19); **13/16 = 81.25% accepted milestone groups**, using equal milestone counts rather than estimated effort. W11 accepts the requested behavior-tree framework option; symbolic search is not an additional mandatory condition. Full integrated mobile/ROS task execution remains under W12. Hardware, target HIL and physical E-stop remain PENDING_HARDWARE, outside this software count.

P1 is a bounded actual residual PPO contact experiment with weights and OOD results. Its presence does not change P0 or hardware claims.

## W01–W12 acceptance matrix

| Milestone | Executed outcome | Status |
|---|---|---|
| W01 architecture/data | SI state/command validation, frame transforms, stable adapter boundary | PASS_UNIT |
| W02 simulation/I/O | Original backend retained as legacy; new force-actuated backend replaces physical acceptance | PASS_REAL_SIM; original base remains legacy |
| W03 grid navigation | Dijkstra/A*, smoothing, pursuit tests and demos | PASS_UNIT / PASS_REAL_SIM for A* physical integration |
| W04 Hybrid A*/replanning/LQR | Existing tests, demo and obstacle replan logs | PASS_UNIT; physical benchmark uses A* pursuit |
| W05 ROS2/control | True Humble build, controller manager, topics/services/actions, TF, state watchdog and rosbag | PASS_REAL_SIM / PASS_MOCK, see ROS report |
| W06 FK/Jacobian/HIL interface | Independent FK/Jacobian cross-validation, telemetry/HAL mock | PASS_UNIT / PASS_MOCK; HIL measurement PENDING_HARDWARE |
| W07 IK/MoveIt2/OMPL | Bounded DLS, singular/unreachable handling, real RRTConnect planning/execution | PASS_UNIT / PASS_REAL_SIM |
| W08 C++17 | CMake/Eigen FK/Jacobian/controllers, GoogleTest, warnings, ASan/UBSan, parity/performance | PASS_UNIT |
| W09 MuJoCo | True base dynamics, mature Panda actuator/contact model, same-world mobile manipulation, independent base encoder/IMU EKF, simulated bias calibration and actuator step identification | PARTIAL: bounded sensor-feedback base PASS_REAL_SIM; full robot estimation/calibration/integration pending |
| W10 safety | Limits, stale state/command, geofence, software stop, reset, action failure, fault injection | PASS_UNIT / PASS_REAL_SIM; physical E-stop pending |
| W11 task framework | Tickable memory BT, Sequence/Fallback, node/blackboard state, bounded actions, persistence, deadline/abort, real failed-grasp recovery | PASS_REAL_SIM for the requested minimum BT framework; same-live-world task-state restore; full mobile/ROS integration remains W12 |
| W12 NAV → GRASP → PLACE | One engine world, five seeds, open-finger failed-grasp recovery | PARTIAL: fixed straight approach PASS_REAL_SIM; flexible obstacle tasks/distributed ROS chain pending |

## W16–W19 and remaining software scope

| Area | Delivered | Boundary |
|---|---|---|
| W16 dynamics/impedance | Real torque impedance, bias compensation, force saturation, admittance and injected torque disturbance | PASS_REAL_SIM; physical parameter identification pending |
| W17 residual learning | Actual PPO 7,680 control steps; weights/curve and baseline evaluation | PASS_REAL_SIM minimal contact experiment; learned grasp/swarm policies not implemented |
| W18 OOD/domain randomization | Held-out seeds; 150/450 N/m OOD vs trained 300; damping variation | PASS_REAL_SIM within specified minimal envelope |
| W19 engineering | Local regression, CI source, profiling, JSONL replay, ROS bag replay, wheel assets, release staging/selection/rollback | PARTIAL: local software checks pass; target deployment/performance pending; remote CI status in parent final report |

No fake hardware frames, camera perception, target HIL or multi-robot result is included. Additional software work: wheel/slip drivetrain dynamics, distributed ROS mobile navigation/manipulation coordination, whole-robot state-estimation integration beyond the new standalone planar EKF and longer changing-obstacle tasks. Perception, general symbolic task planning and broader training-seed studies are possible extensions. Physical calibration and first-week commissioning are listed in the hardware guide.

## Verification numbers and actual simulator evidence

- Baseline: **161 Python tests**, five original demos executed; raw baseline stdout retained.
- Latest complete project regression after the behavior tree: **268 passed in 24.20 s**, raw stdout in `artifacts/bt/pytest_full.txt`; its 26 module cases passed in 6.41 s. The estimator checkpoint separately passed 242 tests. Earlier 220-test checkpoint was independently profiled; its resource measurements describe that earlier suite.
- C++: **4 release GoogleTests + 4 ASan/UBSan passes**; 30 full-range C++/Python numerical parity configurations.
- ROS2: **3 plugin/DDS GoogleTests + 4 real-physics initialization pytest cases**, actual mock and MuJoCo acceptance runs, TF/service/action/fault tests and bag replay; latest precise metrics in `docs/ROS2_ACCEPTANCE.md`. These seven ROS-package cases have a separate denominator from the 268 project tests.
- Physical benchmark: **40/40 runs** (20 navigation, 5 fixed-arm, 5 mobile, 10 contact), five seeds. Bounded IK reachable target study: **100/100**.
- Independent sensor-feedback base: **20/20 navigation goals and 5/5 deliberate outage safe aborts**, with EKF-only control and truth used offline. Worst position estimation RMSE 0.003331 m; outage final speed at most 0.002229 m/s. Simulated gyro bias and actuator gain/lag were measured independently per run. [Scope, failures and strict raw-data audit](STATE_ESTIMATION_ACCEPTANCE.md).
- Failed-grasp injection: actual fingers remain open → failed contact criterion → fresh state recovery → success, recovery count 1.
- Behavior-tree physical matrix: **30/30 expected outcomes** across five seeds; **15 completed place tasks, ten deliberate safe aborts and five exhausted-grasp safe failures**. Worst place error 0.001891 m and stop drift 0.001205 rad against the unchanged 0.02 rad gate. JSON resume uses the same live world, without overwriting physics. [Exact evidence](BT_ACCEPTANCE.md).
- Physical engine startup/load: MuJoCo 3.12.0 models 16 qpos (Panda/object) and 19 qpos (mobile Panda/object); EGL renderer generated and visually verified actual frames. Wheel install outside the repository loaded both models from packaged assets.
- Historical 220-test profiling: 21.023 s wall / 15.679 s kernel child CPU / sampled tree RSS max 495,108,096 bytes; current host, not target Pi. Raw methodology and stdout are in `artifacts/final_regression_profile.{json,stdout.txt}`; this is not a resource measurement of the expanded suite.

## Benchmark overview

| Metric | Executed result |
|---|---:|
| Navigation RMSE straight/obstacle/delay/fault | 0.0106 / 0.0315 / 0.0382 / 0.0312 m |
| Navigation obstacle contact steps | 0 across 20 runs |
| Fixed/mobile arm joint RMSE mean | 0.02139 / 0.02127 rad |
| Physical force RMSE fixed → admittance | 6.525 → 0.0300 N |
| IK max reachable-target position error | 0.0001989 m |
| FK+Jacobian Python/C++ P50 | 363.675 / 0.758 μs |
| Latest ROS2 physical trajectory max error | 0.00089210 rad |
| Latest nominal ROS latency P50/P95/P99 | 0.515 / 0.850 / 1.581 ms, 166 samples |
| Independent cold-start repeat nominal latency P50/P95/P99 | 0.444 / 0.698 / 5.959 ms, 168 samples |
| Actual controller-manager period jitter std | 0.270 ms; repeat 0.271 ms |

Full statistics, sample domains, software versions, hardware-host caveats and raw log pointers: [SIMULATION_BENCHMARK.md](SIMULATION_BENCHMARK.md).

## Reproduce the staged delivery

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j4
ctest --test-dir cpp/build --output-on-failure
python -m pytest -q
python -m demos.benchmark
python -m demos.mobile_manipulation --grasp-fault --seed 7
python -m demos.mobile_manipulation --render --seed 7
python -m demos.estimation_benchmark
python -m demos.estimation_evidence_audit
python -m demos.bt_pick_place --mode recover --seed 7 --render
python -m demos.bt_benchmark --no-render
bash ros2/verify.sh
```

Install Python/C++/ROS2 dependencies as documented in the README and ROS report. Optional PPO: `python -m demos.train_residual_ppo --seed 7 --updates 30`. No external hardware is enabled by these commands.

## Portfolio and hardware handoff

English README now includes actual architecture, implemented algorithms, tested commands, benchmark, real physics PNG/GIFs, failure lessons, dependency/setup instructions, license/pinned upstream model and explicit completion boundaries. Chinese navigation is `docs/README_ZH.md`. Actual assets: `artifacts/physical/{mobile_task_visual_seed_7,navigation_obstacle_seed_7,pick_place_seed_7}.{png,gif}`, metrics/trajectories PNG, per-run JSON/JSONL; learning weights/plot/JSON and real ROS evidence/bags are retained.

Hardware integration proceeds through a verified manufacturer adapter, read-only state validation, elevated-wheel/supported-arm response tests, bounded calibration, physical stop/disconnect tests, low-speed tracking and target timing measurements. The [hardware readiness guide](HARDWARE_READINESS.md) contains the day-by-day first-week checklist. Status remains **PENDING_HARDWARE** throughout this software delivery.

## Cloud verification checkpoint

The BT source checkpoint `3d61ea4bdf1aeccbc3afd83b21cb4f3b6bcca6a4` passed actual [software/MuJoCo CI](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38152756887) and [ROS2/MoveIt2/MuJoCo CI](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38152756884). Job conclusions and exact source SHAs are saved in `artifacts/publication_ci.json`; static links/language/source-evidence checks are in `artifacts/publication_audit.json`.

The estimator checkpoint `65fa747808e5398b6131cf73f7eec90f4ce9686e` also passed actual [software/MuJoCo CI](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38151282249) and [ROS2/MoveIt2/MuJoCo CI](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38151282257). Their success verifies the declared CI checks at that source, not hardware or entire original-scope completion.

GitHub Actions actually passed at source `5861008419d985a0c3d85f4a4c91e52d1023dbfe`: [software/MuJoCo matrix](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38150181642) and [ROS2/MoveIt2/MuJoCo acceptance](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38150181527). The latter ran MuJoCo 3.15.0 and pytest 9.1.1; three plugin/DDS cases plus four physical initialization cases, mock and physics actions/faults/replay all passed. Its separate cloud physics run has 164 nominal samples, latency 0.650/0.970/2.034 ms, maximum joint error 0.00081408 rad and replay 398 messages. Local 166/168-sample measurements above have their own denominator. Earlier editable/time, initial-pose self-collision, pytest plugin incompatibility and physical-packet startup races were diagnosed and fixed; raw failures remain available. Later source commits require their own CI evidence.

## Actual isolated release staging

Release A (`d743ec17ed4e3ba7a1c8809cc293ec90120ff279`) was exported from committed Git sources to a separate Linux directory, installed in a fully isolated venv, passed `pip check`, 4 C++ GoogleTests and 220 Python tests (12.15 s), then selected with hardware disabled and no process launched. [Raw deployment evidence](../artifacts/deployment/) includes dependency versions, independently verified wheel hashes, C++/Python logs and the earlier inherited-environment build failure. Release B (`b1c13cc8d7f273d028151916c5d9b8c4e513bb35`) independently passed 4 C++ tests and 220 Python tests (20.76 s). Actual selection went A → B → rollback A; both releases remain present. The raw selection manifests show hardware_enabled=false and process_started=false. This is host software staging, not target Pi deployment, running-process stop coordination or hardware HIL.
