# Executed simulation benchmark

Collected 2026-10-11 on WSL2 Ubuntu 22.04: Python 3.10.12, MuJoCo 3.12.0, NumPy 1.21.5, GCC 11.4; kernel `6.18.40.1-microsoft-standard-WSL2`, x86_64, 12 logical CPUs. These measurements describe this host, not the planned Raspberry Pi. Raw environment and results: `artifacts/physical/benchmark.json`.

## Protocol and source of truth

Seeds `7,17,27,37,47`; 20 navigation runs, 5 fixed-arm tasks, 5 mobile tasks and 10 contact experiments: **40/40 PASS_REAL_SIM**. All motion advances through `mj_step`, physical actuators and contacts. Initialization may set joint/object poses; stepping never overwrites object or base trajectories. Finger contacts and forces come from `data.contact` and `mj_contactForce`, not a synthetic grasp flag. No weld constraint attaches the object.

Navigation uses online body/world state readback at 50 Hz and a 500 Hz physics timestep. Obstacles are conservative inflated projections of known simulator geometry. The base uses planar constrained dynamics and finite generalized force servos; drivetrain/wheel slip is outside this model. Panda tasks use a standard licensed upstream model; object positions are online simulator ground truth, not visual perception.

## Navigation

| Scenario | Runs / success | Mean path RMSE (m) | Mean planning time (ms) | Obstacle contact steps | Recovery |
|---|---:|---:|---:|---:|---|
| Straight | 5 / 5 | 0.010565 | 2.147 | 0 | None |
| Inflated obstacle | 5 / 5 | 0.031538 | 73.528 | 0 | None |
| 100 ms command queue delay | 5 / 5 | 0.038211 | 74.158 | 0 | None |
| Stale-command injection | 5 / 5 | 0.031168 | 71.671 | 0 | 1 software stop/reset per run |

10,818 actual navigation controller steps. Per-step CTE is shortest distance to a reference line segment; RMSE is computed within each run, then averaged across seeds. Goal acceptance radius is 0.085 m; the controller's stopping goal tolerance is 0.08 m. Planning time includes A* and smoothing. Zero obstacle contact steps means no contact with the test obstacle, not an inferred hardware safety certificate. Control P50/P95/P99 and intervention counts are stored per run. Limit saturation is counted separately from fault recovery.

## Manipulation and task execution

| Experiment | Samples / success | Joint target tracking RMSE |
|---|---:|---:|
| Fixed-base grip/lift/move/place/release | 5 / 5 | 0.021391 rad mean |
| NAV → contact grasp/place in one MuJoCo world | 5 / 5 | 0.021274 rad mean |
| Physically open fingers on first grasp attempt | 1 / 1 after recovery | See `mobile_task_fault_seed_7.json` |

Seed 7 mobile task after the per-step safety gate: 17.26 simulated seconds, final base `[0.491841,0.000037,-0.000332]`, final object `[0.951000,0.220096,0.024848]` m versus target `[0.95,0.22,0.025]` m. Max object height 0.203645 m and observed finger contact force 5.72438 N. Success requires real finger contacts, lifted object height >0.14 m, final planar object error <0.04 m and resting height <0.04 m. Initial object positions vary ±15 mm by seed. Joint RMSE compares actual state to the accepted, safety-slewed servo target, rather than the unsaturated request. The arm applies model bias feed-forward; its every position command passes the same supervisor. Actual arm software-stop and a deadline expiring inside a movement substep are tested against engine state.

Failure injection keeps the actual fingers open, so the contact criterion fails. The executor re-reads the object's state, opens/approaches/descends and retries; raw evidence records the failed GRASP, recovery count 1 and successful verification. Timeout, exhausted retry, failing recovery and abort callbacks are also tested at unit level. The physical task framework is sequential; it does not claim a general symbolic task-and-motion search algorithm.

## Kinematics and C++

- FK/Jacobian: 8 randomly seeded configurations cross-checked with independent NumPy transforms, MuJoCo FK/Jacobian and central finite differences.
- Bounded pose IK: **100/100** reachable random pose targets within ±0.3 rad of the home configuration. These are numerical reachable-pose experiments, not 100 hardware grasps. Maximum position error 0.000198889 m. Runtime P50/P95/P99: 4.292 / 6.787 / 7.635 ms. This local target distribution does not imply global IK completeness.
- C++/Python parity: 30 full-range seeded joint configurations, FK rotation/translation and 6×7 Jacobian tolerance 1e-12.
- Release C++ and Python FK+Jacobian: 10,000 timed calls each, same host, no subprocess startup inside samples. P50/P95/P99: **0.758/0.866/0.975 μs C++**, **363.675/626.070/1070.300 μs Python**. These are implementation performance results, not a universally applicable speed ratio.
- 4 GoogleTests passed in release and 4 passed with AddressSanitizer/UBSan, `-Wall -Wextra -Wpedantic -Werror`.

## Dynamics, impedance and force feedback

MuJoCo supplies articulated dynamics, gravity/Coriolis bias, actuator force limits and collision contacts. The experiment converts the seven affine position servos to torque motors and commands `tau = Jᵀ(K e − D J qdot) + bias`, with force/torque saturation. Finger tips press a physical floor; joint 2 receives an actual 2 Nm disturbance for 0.1 s.

Five seeds perturb damping by ±20%. Steady-state measurements use the final 80 records (1.6 simulated seconds) from each run. Fixed 300 N/m impedance force RMSE versus 2 N: **6.524947 N**. Force admittance adjusts the vertical target and reduces RMSE to **0.030007 N**. The position target lies below the surface; its steady position error is intentional compliant contact deflection, not free-space tracking error. Raw logs retain force, joint torque, target, position and disturbance samples.

## Residual PPO — bounded P1 experiment

Actual PyTorch PPO training uses the MuJoCo Panda floor-contact environment: 7,680 control steps, ten physics substeps per action, seed 7, clipped ratio 0.2, GAE gamma 0.99/lambda 0.95. The residual adds at most 0.03 m to the classical controller's vertical target; target slew and actuator torques stay bounded. Joint reset is initialization only; contact establishment and every rollout transition use real physics.

Reward changed from −0.60 to −0.11 over 30 updates. Held-out seeds `17,27,37`, 3 episodes ×64 steps per policy/configuration, 192 samples each, compare fixed impedance with the trained deterministic residual. Force RMSE averages across the three seeds:

| Stiffness N/m | Fixed impedance force RMSE | Trained residual force RMSE | Unsafe steps |
|---:|---:|---:|---:|
| 150 (OOD) | 2.29 N | 0.73 N | 0 |
| 300 (training) | 6.52 N | 0.49 N | 0 |
| 450 (OOD) | 10.74 N | 1.70 N | 0 |

The separate classical force-admittance experiment reaches 0.03 N steady error and remains the preferred baseline. Residual PPO is a minimal contact study, not learned grasping or MAPPO. The first unramped training run produced contact spikes and failed evaluations; its sanitized failure evidence is retained in `artifacts/learning/initial_unramped_failure.json`. Gradual contact establishment and bounded target slew fixed the issue before the second actual training run. Weights, hyperparameters, reward curve and raw training stdout are retained.

## ROS2 and profiling

The ROS2 acceptance report owns build/test numbers and raw logs. Its latest complete verification ran genuine MoveIt RRTConnect planning in 22.396 ms and ros2_control/MuJoCo execution in 1.127 s, with max joint error **0.00089210 rad**. Nominal 166 same-host messages: stamp-to-callback P50/P95/P99 **0.515/0.850/1.581 ms**. Actual controller-manager period P50/P95/P99 **9.919/10.516/10.569 ms**, jitter standard deviation **0.270 ms**. An independent cold-start repeat also passed all action/fault/replay assertions: max error 0.00099465 rad, 168 nominal samples, latency **0.444/0.698/5.959 ms**, period jitter **0.271 ms**. Full/repeat recorded five/one actual startup reset refusals before receiving physical-state packets; fresh-state Safety gates were retained. Nominal samples start after measured complete state/controller/TF/home/collision-validity checks and stop before injected faults; whole-run samples are separate. Earlier 476/167/164-sample checkpoints and failures remain in Git/logs. These are host transport measurements with explicit variation, not a real-time bound or target HIL result.

The 215-test profiling checkpoint recorded 14.453 s wall time, 8.951 s kernel child CPU time and sampled process-tree RSS max 456,253,440 bytes. Shared pages can be counted more than once in summed RSS. Final independent profiling: **220 passed in 20.69 s**, 21.023 s wall / 15.679 s kernel child CPU / sampled tree RSS max 495,108,096 bytes (`artifacts/final_regression_profile.json` and stdout). Earlier agent regression was 17.90 s; these timings are separate runs. Benchmark-process `ru_maxrss` peak was 416,312 KiB. No CPU/memory number is attributed to the target Pi.

## Reproduction and evidence

```bash
python -m demos.benchmark
python -m demos.core_benchmark
python -m demos.mobile_manipulation --grasp-fault --seed 7
python -m demos.mobile_manipulation --render --seed 7
python -m demos.train_residual_ppo --updates 30 --seed 7
```

`artifacts/physical/`: benchmark JSON, every run's JSON/JSONL, trajectory/metric PNGs, genuine MuJoCo PNG/GIFs and test stdout. `artifacts/learning/`: actual weights, training/evaluation JSON, reward/OOD PNG and training stdout. `ros2/evidence/`: real build, execution, bag recording/replay and fault data. Rendering was visually checked and is excluded from headline timing collection. EGL screenshot generation succeeded with explicit renderer cleanup.
