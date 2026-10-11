# Hardware readiness — pending physical validation

**Hardware-Ready / Pending Physical Validation.** The software stage has genuine physics, ROS2/MoveIt2 execution, C++ core tests, software safety, telemetry and commissioning tools. No connected robot controller, target Raspberry Pi or measured hardware HIL exists in this delivery.

## Stable contracts and supported backends

`RobotState`: timestamp [s], world base `[x,y,yaw]` [m,m,rad], body `[vx,vy,wz]` [m/s,m/s,rad/s], seven arm positions/velocities [rad,rad/s], gripper width [m]. `RobotCommand` owns arrays and validates finite values; optional arm/gripper channels use `None` to mean no new command. The differential-drive command boundary rejects lateral motion. `RobotAdapter` separates reset/read/send/step/shutdown from planning/control.

| Backend | Status | Interpretation |
|---|---|---|
| MockHardwareAdapter | PASS_MOCK | Explicit deterministic mathematical I/O; no physical validation |
| Original MujocoRobotAdapter | PASS_UNIT, legacy kinematic base | Existing joint actuators are physics; base overwrites and logical fingers exclude full physical acceptance |
| PhysicalMujocoAdapter | PASS_REAL_SIM | Planar force servos, finite acceleration, contacts and stale-command stop |
| PandaSimulation / MobilePandaSimulation | PASS_REAL_SIM | Mature licensed arm, physical fingers/object, real torque/position actuators |
| ros2_control hardware plugin + MuJoCo topic bridge | PASS_REAL_SIM | Actual controller manager, state/position commands, watchdog, trajectory execution and software stop |
| Vendor controller adapter | PENDING_HARDWARE | Protocol, acknowledgement semantics and controller-specific units must be verified |

ROS bridge maps `panda_joint1..7` to Menagerie `joint1..7`. ROS JointState carries radians/rad/s; actuator position targets are radians. The Panda gripper actuator's internal 0–255 input is a documented upstream simulation mapping from 0–0.08 m finger width, not an invented hardware protocol. ROS finger state currently uses mock channels; physical grasp evidence comes from the standalone/integrated MuJoCo task.

## Configuration and commissioning tools

`tools/hardware/commissioning.example.json` leaves vendor, protocol and actual calibration values null, hardware disabled. `tools.hardware.telemetry.Recorder` records encoder/IMU/state/command/receipt events, units, command IDs and an explicit provenance/clock domain. `tools.hardware.commission` measures recorded step response, validates offline replay and generates separate failure-injection fixtures. See [tool README](../tools/hardware/README.md) for tested CLI/API commands.

Mock step response is **PASS_MOCK** analysis-pipeline evidence. Real encoder signs, motor deadzone, delay, gain, saturation, steering geometry and IMU calibration remain **PENDING_HARDWARE**. One-way cross-host latency requires clock synchronization; local same-host ROS timing is not sufficient.

Source/wheel assets were packaged and an isolated wheel environment outside the checkout loaded both physical arm and mobile models successfully. The generated 5.13 MB wheel is excluded from Git; sources and upstream license are committed. Rebuild with `python -m pip wheel . --no-deps --wheel-dir artifacts/package`.

## Safety before enabling transport

Software limits include finite ordered configuration, norm speed/yaw constraints, model-configurable joint bounds and slew, geofence, command/state freshness, latched software stop and fresh stationary reset. The real ROS plugin rejects invalid goals and trips on missing state. Actual physics regressions verify finite base acceleration, collision response, command timeout, safe stop/reset and task retry/abort.

A physical E-stop, power isolation, driver enable line, cable pull/disconnect behavior and passive braking are still required. The simulator's position hold or zero-velocity servo cannot certify real power-loss behavior. Manual override is currently a software pause/stop interface, not a validated remote teleoperation channel.

## Deployment and rollback

`tools.hardware.deploy` stages committed sources into a new release, creates an isolated environment, runs tests and records the full commit. Selecting an untested release is rejected; rollback selects the previous release. Selection changes an atomic local manifest and starts no hardware process. A verified runtime launcher owns stop-before-switch behavior. The deployment command does not pretend to stop a physical motor.

Keep secrets and manufacturer-specific config outside Git. Measure CPU/memory/jitter/latency on the actual target; [local profile](../artifacts/regression_profile_20261011.json) is explicitly this WSL host. A future R550-class platform needs its actual drivetrain/steering documentation; the present planar model cannot determine whether its chassis is Ackermann, differential drive or another mechanism.

## First hardware week

1. **Day 1:** Obtain exact manuals; verify supply voltage/current, common ground, polarity, signal voltage, communication wiring, encoder scale, controller watchdog and independent power isolation. Confirm physical E-stop without software assistance.
2. **Day 2:** Wheels elevated / arm supported: read only encoder/IMU/joint state. Verify axis signs, zero offsets, monotonic timestamps, units and update rate. Record raw data with explicit hardware provenance.
3. **Day 3:** Enable bounded low-energy motor/joint steps. Measure rise time, delay, deadzone, gain, saturation and rate; configure limits from measured values and retain raw logs.
4. **Day 4:** In a controlled area test software stop, stale state, command loss, cable disconnection, manual pause, geofence and restart/reset. Measure actual stopping distance; do not infer it from simulation.
5. **Day 5:** Run low-speed tracking with inflated clearance and minimal payload. Compare measured versus simulated logs, identify parameters, measure target CPU/RSS/control jitter/ROS latency and record target HIL evidence.
6. **Days 6–7:** Increase only validated envelopes; test load/friction changes, long task recovery and rollback after safe shutdown. Continue recording failures and retain the last known safe release.

Physical contact manipulation, environmental perception, accurate hardware calibration, certified E-stop, real wheel slip and target-platform performance remain **PENDING_HARDWARE**; general symbolic TAMP, distributed mobile ROS orchestration and multi-robot planning remain software follow-up items.
