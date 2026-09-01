# Milestones

## Completed foundation

- **W01:** validated robot state/command models and deterministic mock hardware.
- **W02:** backend-neutral HAL, MuJoCo mobile-manipulator skeleton, and a
  reviewable ros2_control interface skeleton.
- **W03:** occupancy grid, Dijkstra/A*, line-of-sight simplification, safe path
  smoothing, and Pure Pursuit tracking.
- **W04:** continuous-pose Hybrid A*, minimal map-update replanning, LQR
  tracking, controller/planner benchmarks, and an integrated navigation GIF.

## Next single-robot milestones

1. Add footprint-aware and reverse-capable nonholonomic planning.
2. Add time-aware obstacle prediction and controller-aware safety checks.
3. Add arm FK/IK/Jacobians and a collision-aware manipulation baseline.
4. Sequence navigation and manipulation with explicit state/failure handling.
5. Integrate and validate a real ROS2 runtime before any hardware claim.

## Multi-robot roadmap

After the single-robot stack has measurable planning/control contracts, add
task allocation, reservation/conflict handling, and repeatable multi-robot
benchmarks. The project name reflects this direction; the current release does
not claim swarm coordination.
