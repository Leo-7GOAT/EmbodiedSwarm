# Technical interview guide

Use the executed evidence and limitations together. The strongest demonstration is `demos.mobile_manipulation`: navigation and force/friction grasp/place happen in one MuJoCo world. The separate ROS acceptance demonstrates real MoveIt2 planning and controller execution. Hardware/HIL and camera perception are pending.

## Architecture, frames and time

**Code:** `embodied_swarm/core/robot_state.py`, `robot_command.py`, `frames.py`, `hal/robot_adapter.py`.

State → planning → trajectory → controller → safety → adapter → simulator/hardware is the central boundary. A state stores world base position/yaw, body-frame twist, seven arm states and gripper width. The body-to-world transformation is

`[vx_world,vy_world] = [[cos(yaw),-sin(yaw)],[sin(yaw),cos(yaw)]] [vx_body,vy_body]`.

Timestamp seconds belong to one explicitly chosen clock. MuJoCo uses simulation time; ROS transport measurements use the same host's wall clock. Mixing wall and simulation timestamps makes freshness checks meaningless. Arrays own their data so callers cannot accidentally mutate a previously accepted command.

**Why:** Algorithm code does not need to know a controller vendor. Mock, replay and actual physics can be compared using the same SI contract. The command's `None` arm target means no update; a zero target is a real update.

**Likely questions:** Why reject nonzero lateral velocity? The current command boundary represents differential-drive/unicycle operation. The planar physical model has world x/y constraints for integration, but that does not make the robot holonomic or validate a drivetrain. How do you handle changing clock domains? Choose a monotonic domain or verified conversion at the adapter; never silently treat host time as simulation time.

**Understand personally:** Frame conversion direction/sign, radians versus degrees, clock choice and copy ownership.

## Grid search, footprint and Hybrid A*

**Code:** `navigation/{occupancy_grid,dijkstra,astar,hybrid_astar,path_smoothing,replanning}.py`.

Dijkstra expands least path cost `g`; A* uses `f=g+h`, here Manhattan distance for the four-connected unit-cost grid. An admissible heuristic never overestimates the true remaining cost. Hybrid A* carries a continuous `(x,y,yaw)` state while deduplicating discretized cell/yaw-bin keys. Its forward unicycle primitives obey `xdot=v cos(yaw)`, `ydot=v sin(yaw)`, `yawdot=omega`; all primitive integration samples are collision checked.

Smoothing trades data fidelity against local curvature by iterative neighboring-point updates, but accepts an update only when adjoining segments are free. Physical navigation inflates obstacle geometry by footprint plus tracking margin before planning. Checking only a reference point cannot guarantee robot footprint clearance.

**Why:** A* is a strong simple baseline; Hybrid A* respects heading/motion constraints without requiring a complete continuous optimizer. The real-physics benchmark deliberately uses the established A* + pursuit pipeline; Hybrid A* has its own unit/demo evidence.

**Tradeoffs:** Forward-only primitives, discretization aliasing, no analytic expansion and inflated maps can make feasible narrow/reverse maneuvers fail. Reactive map replanning is not a prediction-based spatiotemporal planner.

**Likely questions:** Is Hybrid A* optimal? Not generally with these discretization/primitive choices. Can smoothing invalidate collision safety? Yes unless each new segment is checked. Why is path RMSE not enough? A low-error path can still collide, fail the goal or violate actuator constraints; this benchmark reports contacts, final tolerance, faults and runtime too.

**Understand personally:** Heuristic admissibility, state-key versus carried pose, map inflation and segment collision checks.

## Pure pursuit and LQR

**Code:** `control/pure_pursuit.py`, `control/lqr_tracker.py`, C++ `cpp/src/core.cpp`.

Pure pursuit transforms a lookahead target into the body frame and computes curvature `kappa = 2*y_local/L²`; a unicycle command is `omega=v*kappa`. Lookahead selects responsiveness versus smoothing, and yaw-rate saturation can increase path error. The controller must slow or replan for tight turns rather than assume the actuator follows any curvature.

The LQR baseline linearizes lateral/heading error at fixed speed, uses `e[k+1]=A e[k]+B u[k]`, and minimizes `sum(eᵀQe + uᵀRu)`. Its discrete Riccati equation gives `K`, with `u=-K e`. Q penalizes tracking; R penalizes input. Its validity is local and depends on the operating speed/model.

**Failure example:** Added command delay increased physical CTE RMSE from about 0.0315 to 0.0382 m. Saturation and finite base acceleration explain why kinematic demo results cannot replace physical closed-loop acceptance.

**Likely questions:** What if the target lies behind the robot? Why change lookahead with speed? How does angular-rate saturation affect LQR guarantees? A saturated nonlinear plant no longer obeys the unconstrained quadratic optimum.

**Understand personally:** Curvature sign, wraparound of yaw, the error-state definition, Riccati equation and the relation between Q/R and behavior.

## Independent Panda FK and geometric Jacobian

**Code:** `models/kinematics.py`, `cpp/src/core.cpp`, `tests/test_physical_stack.py`, `tests/test_cpp_parity.py`.

FK composes each fixed parent-to-body transform and its revolute-joint rotation: `T_base_tip = product(T_fixed_i * Rot(axis_i,q_i)) * T_tip`. Upstream normalized quaternion geometry is preserved; C++ uses the same rounded hand quaternion rather than an assumed exact angle.

For world joint origin `o_i`, axis `a_i` and tip position `p`, the geometric Jacobian column is `J_i = [a_i × (p-o_i); a_i]`. Then `twist = J qdot`. Manipulability is the product of singular values, equivalent to `sqrt(det(J Jᵀ))` for a full-row-rank Jacobian.

**Cross-validation:** Independent NumPy transform/Jacobian calculations versus MuJoCo `mj_forward` and `mj_jacSite`, central finite differences of translation, and 30 randomized C++ parity configurations. The model's static geometry is input data; the NumPy implementation never calls MuJoCo FK to compute its own result.

**Likely questions:** In which frame is J expressed? Here it is world/base-reference frame. Why geometric versus analytic Jacobian? Angular velocity is a geometric quantity; Euler-angle rates require an additional mapping and have representation singularities. Can the base move? The mobile adapter transforms world targets into the current base frame before arm IK, using actual base state.

**Understand personally:** Transform multiplication order, quaternion normalization, axis/origin extraction and rotational versus translational Jacobian components.

## Damped least squares IK and limits

**Code:** `SerialKinematics.ik()` in `models/kinematics.py`.

Pose error consists of translation and the SO(3) log of `R_target R_currentᵀ`. This rotation vector avoids the false zero error at 180° that occurs with a cross-product-only orientation error. Damped least squares computes

`dq = Jᵀ (J Jᵀ + lambda² I)^(-1) e`.

The smallest singular value increases adaptive damping near a singularity. Each step is norm-limited, then clipped into model joint bounds. A reachable result must satisfy both position and orientation tolerances; an exhausted iteration limit returns a failed result with its actual residual, never a fabricated solution.

**Why:** SVD-based or damped solvers avoid enormous updates near rank loss. Redundancy makes multiple joint solutions possible; seeds and limits matter. A local solver is sufficient for the tested task workspace and acts as an independent numerical baseline alongside MoveIt.

**Tradeoffs:** Joint clipping can stall at a boundary; DLS alone does not guarantee collision-free/global IK or optimize posture. Pose position and rotation residuals have different units; gain/scaling matter. The 100-target benchmark covers reachable near-home poses, not the whole workspace.

**Likely questions:** Why not pseudoinverse? What happens at an unreachable target? How would null-space avoidance work? One can add `(I-J#J) grad(cost)` for joint-margin/posture objectives, but collision planning still needs a separate geometric constraint/planner.

**Understand personally:** Singular values, damping effect, limit saturation, orientation log, residual criteria and why local convergence is not global completeness.

## Real MuJoCo motion, contacts and grasp

**Code:** `hal/{physical_mujoco_adapter,panda_simulation,mobile_panda}.py`, `assets/mjcf/{physical_base,panda_pick_place,mobile_panda}.xml`, `demos/{physical_navigation,physical_pick_place,mobile_manipulation}.py`.

MuJoCo evolves articulated dynamics under mass/inertia, bias terms, actuator forces and contact constraints. The Panda is an unchanged mature licensed Menagerie model, mounted in project-specific task scenes. Two finger bodies collide with a free rigid object; closing, lifting, transporting and releasing use actuator commands and `mj_step`.

The physical base has planar generalized slide/yaw joints and finite force velocity servos. Its finite acceleration, collision blocking and reaction dynamics distinguish it from directly integrating a unicycle. The project explicitly stops short of wheel-ground slip or manufacturer drivetrain validation.

**Acceptance:** Contact information is observed from the engine; lifted object z >0.14 m confirms attachment through physical contact rather than an internal grasp flag. Final planar placement error <0.04 m and floor-resting height <0.04 m verify release. Object/base qpos is only set during initialization, not during motion. No weld captures the object.

**Failure example:** The original adapter called `mj_step` but then replaced base qpos/qvel with expected kinematics and reported a logical gripper. It remains a documented legacy backend; its old green tests are not claimed as physical base/gripper validation.

**Likely questions:** Is the contact force a real sensor measurement? It is engine ground truth; hardware force sensor calibration is pending. Is the physical base a differential-drive simulator? It obeys that command restriction but uses constrained generalized actuators, so wheel dynamics are not validated. Can the task perceive an object? It uses clearly labeled online simulator ground truth.

**Understand personally:** Actuator versus state update, contact/friction versus weld, free-joint object indexing, initialization versus simulation stepping and model provenance.

## ROS2, ros2_control and MoveIt2

**Code:** `ros2/embodied_swarm_hardware/`, `ros2/embodied_swarm_bringup/`, `ros2/verify.sh`, [ROS2 acceptance](ROS2_ACCEPTANCE.md).

The hardware interface exports position commands and joint position/velocity states. Controller manager runs joint state broadcaster and trajectory controller. The transport backend reads genuine `/mujoco/joint_states` and publishes `/mujoco/joint_commands`; a real MuJoCo bridge maps official Panda names to upstream model names. TF2, service/action calls and rosbag record/replay are executed, not represented by Python imitation classes.

MoveIt2 uses official Panda URDF/SRDF limits and collision geometry, OMPL RRTConnect for collision-aware joint-space planning, trajectory time parameterization and trajectory-controller execution. RRTConnect extends two random trees toward each other; it finds feasible solutions efficiently but does not guarantee optimality. The simulator arm's position servos are augmented by arm-only gravity/Coriolis bias compensation; the free object is unaffected.

**Failures:** Startup without fresh state latches the watchdog. Reset requires fresh safe state. Software E-stop during a trajectory causes actual action failure, not a pretend successful trajectory. Invalid action goals are rejected and missing state prevents unsafe reset. The gripper ROS channel is mock; physical gripper acceptance is explicitly separate.

**Likely questions:** Topic/service/action differences? Topics stream state/commands, services do short request/response, actions provide asynchronous trajectory result/cancel/feedback. Why not execute every controller in ROS Python? ros2_control provides a bounded realtime-oriented read/update/write structure and standard trajectory handling; actual real-time scheduling still requires target measurements. Does a MoveIt plan guarantee execution? No: controller timeout, dynamics, state discrepancy and safety intervention can invalidate it.

**Understand personally:** URDF/SRDF roles, joint-name mapping, controller interface ownership, action result codes, DDS/executor scheduling, TF frame consistency and bag replay limitations.

## Task execution, recovery and TAMP boundary

**Code:** `core/task.py`, `demos/mobile_manipulation.py`, `demos/physical_pick_place.py`.

The executor has explicit named stages, return-based success criteria, bounded retry, whole-task deadline, recovery callbacks and abort callbacks. Time is checked before and after an action and after recovery. Exceptions or exhausted retry are recorded. This synchronous framework cannot interrupt an arbitrary blocking callback instantly; actions themselves must remain bounded or be adapted to cancellable ROS actions.

Physical failed-grasp injection keeps fingers open on the first attempt. The engine observes missing contact; recovery re-reads the object's true position, backs off, opens, descends and retries. It does not simply set a success flag. The same-world task confirms NAV followed by actual arm contact behavior.

**Boundary:** Sequential task orchestration is not a general symbolic TAMP solver. Future generalized TAMP would search discrete actions/objects with geometric feasibility and resource constraints, backtracking when a motion plan fails.

**Likely questions:** Why distinguish failure from timeout? Their recovery strategies differ. Can retry make a dangerous task worse? Yes; retries are bounded, conditions must be re-observed, and abort needs a safe stop/hold. What if a recovery itself fails? The executor records recovery failure and aborts.

**Understand personally:** Stage success versus function return, fresh observation after contact failures, deadline/cancellation limitations and safe recovery policy.

## Impedance, admittance and dynamics

**Code:** `control/impedance.py`, `demos/impedance_contact.py`, C++ impedance kernel.

Cartesian impedance applies `tau = Jᵀ (K e - D J qdot) + bias`, clipped to actual model actuator torque limits. The bias is gravity/Coriolis from the physical model. Joint torques yield compliant contact response; a rigid position target below a surface produces force depending on stiffness and deflection. Damping dissipates motion (`-D velocity`) and must remain positive.

Admittance uses measured engine contact force to move the desired vertical position toward a 2 N target. This outer force-feedback loop reduced steady force RMSE from 6.525 to 0.030 N under damping perturbation and injected joint torque. Position residual in contact is expected compliant deflection, not failed free-space tracking.

**Tradeoffs:** Impedance needs suitable torque interfaces/model bias; position-only hardware may require an admittance controller. Contact stiffness, discretization, feedback delay and saturation affect stability. A torque bound does not make an unknown hardware robot safe without measured limits and physical isolation.

**Failure example:** The first residual-learning environment jumped the vertical target into contact and produced force spikes. Gradual contact establishment and a bounded target slew fixed the physical transient before retraining. Classical force feedback remained more accurate than the learned residual.

**Likely questions:** Impedance versus admittance? Impedance converts motion error to force; admittance converts force error to desired motion. Why compensate gravity? Otherwise pose/contact errors contain gravity sag. Why not invert J? Jacobian transpose maps wrench into generalized torque without requiring a square invertible Jacobian.

**Understand personally:** Bias compensation, virtual stiffness/damping, contact deflection, torque bounds, sensor noise/delay and outer-loop force-control sign.

## Residual PPO and OOD

**Code:** `control/residual_contact_env.py`, `demos/train_residual_ppo.py`; weights/curve in `artifacts/learning/`.

A small neural Gaussian policy adds at most 3 cm to the classical vertical target. PPO optimizes the clipped probability ratio surrogate, `min(r*A, clip(r,1-eps,1+eps)*A)`, with GAE advantages. The raw Gaussian action is transformed with tanh in the physical environment; the same invertible transformation cancels in the old/new policy ratio. Rewards penalize force error and residual magnitude; the critic estimates return.

Actual MuJoCo training ran 7,680 control steps, seed 7; damping is randomized ±20%. Held-out seeds and stiffness 150/450 N/m evaluate distribution shift around training stiffness 300. The trained residual improved fixed-impedance force tracking without unsafe evaluation steps, but did not surpass classical admittance in the steady-state comparison. This is a genuine minimal experiment, not learned grasping, MAPPO or a production policy.

**Likely questions:** Why residual rather than end-to-end torque learning? It preserves classical structure and bounds, simplifying analysis/data needs. What does OOD prove? Only the tested stiffness/damping envelope, not generalization to real contact materials or arbitrary tasks. Is one training seed enough? It demonstrates reproducibility but not robust statistical training reliability; multiple training seeds remain a follow-up.

**Understand personally:** PPO ratio/advantage, reward units, data sample count, train versus evaluation seed, OOD scope and the stronger classical baseline.

## Safety, performance and sim-to-real

**Code:** `core/safety.py`, ROS watchdog/plugin, `tools/hardware/`, `demos/benchmark.py`, `demos/core_benchmark.py`.

The supervisor validates finite configuration, ordered fences, norm speed/yaw limits, optional model joint limits and per-cycle target slew. A stale state/command, fence breach or software stop latches a reason. Reset requires fresh stationary state; receiving a new command alone does not clear the fault. The actuator/backend still needs its own local timeout so upstream failure cannot leave a persistent command.

P50/P95/P99 summarize distributions, not worst-case guarantees. Planning and control time are measured separately; rendering is excluded. Path RMSE uses nearest line segments. Joint RMSE compares actual state with commanded target during motion. ROS message latency and actual controller-manager period are measured independently on the same WSL host. CPU/RSS profiling belongs to this host, not the future Pi. Sanitizers and parity complement behavioral tests but do not prove hardware safety.

**Sim-to-real:** Verify exact drivetrain/driver/manual; read only state, check axes/units/clocks, characterize delay/deadzone/saturation, enforce measured limits, test physical E-stop and cable loss, then run low-speed tracking and target HIL. Unknown R550 communication frames or actuator constants remain unknown; nothing is invented to make deployment seem complete.

**Likely questions:** Does software E-stop certify stopping? No; physical braking/power isolation must be measured. Why include a failed experiment? It shows honest debugging and defines the reliable envelope. What remains before purchase? Select hardware with documented state/control interfaces and independent stop behavior; target load/latency and mechanism parameters must be validated after arrival.

**Understand personally:** Fail-safe state, freshness versus command watchdog, recovery authorization, stopping-distance measurement, target timing and every limitation listed in the README.
