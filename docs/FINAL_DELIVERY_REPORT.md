# Delivery report — measured software stage, 2026-10-11

Repository: [Leo-7GOAT/EmbodiedSwarm](https://github.com/Leo-7GOAT/EmbodiedSwarm). ROS initialization/readiness source checkpoint: `5861008419d985a0c3d85f4a4c91e52d1023dbfe`; subsequent test-environment and evidence commits are recorded in Git history. Use `git rev-parse HEAD` for the checked-out release SHA; the final delivery message gives the exact published SHA. Never interpret this report as physical robot or target-platform HIL validation.

## Stage gates and completion denominator

The current **component software stage** defines twelve gates: (1) protected baseline, (2) validated state/HAL contracts, (3) navigation algorithms, (4) independent FK/Jacobian/IK, (5) physical base loop, (6) contact grasp/place, (7) C++ build/numerical tests, (8) actual ROS2/ros2_control execution, (9) actual MoveIt2/OMPL execution, (10) safety/fault response, (11) same-world fixed-approach task with retry/abort, (12) physical impedance/force feedback. **12 component gates have passing local evidence**, including actual per-step arm safety/stop/deadline checks. These component checks do not complete the original roadmap. The full sixteen requested milestone groups (W01–W12, W16–W19), judged by their broader subrequirements, contain **12 PASS and 4 PARTIAL** (W09, W11, W12, W19); **12/16 = 75% fully accepted milestone groups**, using equal milestone counts rather than estimated effort. Hardware, target HIL and physical E-stop remain PENDING_HARDWARE, outside this software count.

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
| W09 MuJoCo | True base dynamics, mature Panda actuator/contact model, same-world mobile manipulation, plots/GIFs | PARTIAL: physics PASS_REAL_SIM; estimation/calibration currently GT or prepared tools |
| W10 safety | Limits, stale state/command, geofence, software stop, reset, action failure, fault injection | PASS_UNIT / PASS_REAL_SIM; physical E-stop pending |
| W11 task framework | Sequential stage task execution, deadline/retry/recovery/abort and physical success criteria | PARTIAL: bounded framework PASS_REAL_SIM; general BT/symbolic TAMP not implemented |
| W12 NAV → GRASP → PLACE | One engine world, five seeds, open-finger failed-grasp recovery | PARTIAL: fixed straight approach PASS_REAL_SIM; flexible obstacle tasks/distributed ROS chain pending |

## W16–W19 and remaining software scope

| Area | Delivered | Boundary |
|---|---|---|
| W16 dynamics/impedance | Real torque impedance, bias compensation, force saturation, admittance and injected torque disturbance | PASS_REAL_SIM; physical parameter identification pending |
| W17 residual learning | Actual PPO 7,680 control steps; weights/curve and baseline evaluation | PASS_REAL_SIM minimal contact experiment; learned grasp/swarm policies not implemented |
| W18 OOD/domain randomization | Held-out seeds; 150/450 N/m OOD vs trained 300; damping variation | PASS_REAL_SIM within specified minimal envelope |
| W19 engineering | Local regression, CI source, profiling, JSONL replay, ROS bag replay, wheel assets, release staging/selection/rollback | PARTIAL: local software checks pass; target deployment/performance pending; remote CI status in parent final report |

No fake hardware frames, camera perception, target HIL or multi-robot result is included. Additional software work: general task-and-motion symbolic search, wheel/slip drivetrain dynamics, distributed ROS mobile navigation/manipulation coordination, perception/state-estimation beyond GT, longer stochastic contact tasks and multiple independent training seeds. Physical calibration and first-week commissioning are listed in the hardware guide.

## Verification numbers and actual simulator evidence

- Baseline: **161 Python tests**, five original demos executed; raw baseline stdout retained.
- Latest project regression before final parent profiling: **220 passed in 20.69 s**, independently profiled in `artifacts/final_regression_profile.stdout.txt` (earlier agent run: 17.90 s).
- C++: **4 release GoogleTests + 4 ASan/UBSan passes**; 30 full-range C++/Python numerical parity configurations.
- ROS2: **3 plugin/DDS GoogleTests + 4 real-physics initialization pytest cases**, actual mock and MuJoCo acceptance runs, TF/service/action/fault tests and bag replay; latest precise metrics in `docs/ROS2_ACCEPTANCE.md`. These seven ROS-package cases have a separate denominator from the 220 project tests.
- Physical benchmark: **40/40 runs** (20 navigation, 5 fixed-arm, 5 mobile, 10 contact), five seeds. Bounded IK reachable target study: **100/100**.
- Failed-grasp injection: actual fingers remain open → failed contact criterion → fresh state recovery → success, recovery count 1.
- Physical engine startup/load: MuJoCo 3.12.0 models 16 qpos (Panda/object) and 19 qpos (mobile Panda/object); EGL renderer generated and visually verified actual frames. Wheel install outside the repository loaded both models from packaged assets.
- Final 220-test profiling: 21.023 s wall / 15.679 s kernel child CPU / sampled tree RSS max 495,108,096 bytes; current host, not target Pi. Raw methodology and stdout are in `artifacts/final_regression_profile.{json,stdout.txt}`.

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
bash ros2/verify.sh
```

Install Python/C++/ROS2 dependencies as documented in the README and ROS report. Optional PPO: `python -m demos.train_residual_ppo --seed 7 --updates 30`. No external hardware is enabled by these commands.

## Portfolio and hardware handoff

English README now includes actual architecture, implemented algorithms, tested commands, benchmark, real physics PNG/GIFs, failure lessons, dependency/setup instructions, license/pinned upstream model and explicit completion boundaries. Chinese navigation is `docs/README_ZH.md`. Actual assets: `artifacts/physical/{mobile_task_visual_seed_7,navigation_obstacle_seed_7,pick_place_seed_7}.{png,gif}`, metrics/trajectories PNG, per-run JSON/JSONL; learning weights/plot/JSON and real ROS evidence/bags are retained.

Hardware integration proceeds through a verified manufacturer adapter, read-only state validation, elevated-wheel/supported-arm response tests, bounded calibration, physical stop/disconnect tests, low-speed tracking and target timing measurements. The [hardware readiness guide](HARDWARE_READINESS.md) contains the day-by-day first-week checklist. Status remains **PENDING_HARDWARE** throughout this software delivery.

## Cloud verification checkpoint

GitHub Actions actually passed at source `5861008419d985a0c3d85f4a4c91e52d1023dbfe`: [software/MuJoCo matrix](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38150181642) and [ROS2/MoveIt2/MuJoCo acceptance](https://github.com/Leo-7GOAT/EmbodiedSwarm/actions/runs/38150181527). The latter ran MuJoCo 3.15.0 and pytest 9.1.1; three plugin/DDS cases plus four physical initialization cases, mock and physics actions/faults/replay all passed. Its separate cloud physics run has 164 nominal samples, latency 0.650/0.970/2.034 ms, maximum joint error 0.00081408 rad and replay 398 messages. Local 166/168-sample measurements above have their own denominator. Earlier editable/time, initial-pose self-collision, pytest plugin incompatibility and physical-packet startup races were diagnosed and fixed; raw failures remain available. Later source commits require their own CI evidence.

## Actual isolated release staging

Release A (`d743ec17ed4e3ba7a1c8809cc293ec90120ff279`) was exported from committed Git sources to a separate Linux directory, installed in a fully isolated venv, passed `pip check`, 4 C++ GoogleTests and 220 Python tests (12.15 s), then selected with hardware disabled and no process launched. [Raw deployment evidence](../artifacts/deployment/) includes dependency versions, independently verified wheel hashes, C++/Python logs and the earlier inherited-environment build failure. Release B (`b1c13cc8d7f273d028151916c5d9b8c4e513bb35`) independently passed 4 C++ tests and 220 Python tests (20.76 s). Actual selection went A → B → rollback A; both releases remain present. The raw selection manifests show hardware_enabled=false and process_started=false. This is host software staging, not target Pi deployment, running-process stop coordination or hardware HIL.
