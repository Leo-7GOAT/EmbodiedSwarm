# Baseline audit — 2026-10-11

Repository: `C:\Users\Leo\Desktop\project\EmbodiedSwarm`. The original delivery contained no usable Git history; the parent task restored remote metadata without replacing source. Remote main: `6fa3678` (exact SHA is retained in Git). All original tracked files matched remote after line-ending normalization. A complete ZIP backup precedes changes; no destructive cleanup was used.

## Executed baseline

- WSL Ubuntu 22.04, Python 3.10, MuJoCo **3.12.0**, NumPy 1.21.5, matplotlib 3.5.1, Pillow 9.0.1.
- `python3 -m pytest -q`: **161 passed in 6.75 s**.
- All five existing demos ran successfully. Text evidence: `artifacts/baseline/{navigation,w04,lqr,replanning,dijkstra}.txt`.
- Existing W04 demo reaches its goal after 1,645 controller steps, with 0.7865 m final error; this is a kinematic mathematical experiment, not physical simulation.
- EGL renderer creation succeeds in WSL. Rendering must explicitly close its context to avoid destructor errors.

## Implemented before this task

Grid Dijkstra/A*, forward Hybrid A*, collision-aware smoothing, pure pursuit and LQR, reactive obstacle replanning, state/command validation, body/world frames, hardware mock, legacy MuJoCo adapter and ROS2 hardware skeleton.

## Material gaps and priorities

1. **P0:** Legacy MuJoCo adapter overwrites planar `qpos/qvel` after every `mj_step`. Its base is intentionally kinematic; gripper width is logical only. It is preserved as a legacy backend and excluded from physical acceptance.
2. **P0:** MJCF arm is explicitly a Panda-compatible geometric skeleton, not an official Panda. Introduce a mature licensed model for manipulation and independent FK/Jacobian cross-validation.
3. **P0:** No prior bounded IK, C++ library/tests, independent safety supervisor, contact manipulation, long-sequence task recovery or physical benchmark.
4. **P0:** ROS2/MoveIt2/ros2_control require genuine WSL compilation and execution. A separate agent owns `ros2/` and supplies its execution evidence.
5. **P1:** No residual learning experiment; learning will not replace a reliable classical baseline.

No physical robot or target Raspberry Pi is connected: **PENDING_HARDWARE**, HIL-ready only. Hardware protocols and motor parameters remain unspecified pending manufacturer documentation.
