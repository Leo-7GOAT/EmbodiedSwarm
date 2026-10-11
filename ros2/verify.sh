#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$ROOT/evidence"
bash "$ROOT/build.sh" 2>&1 | tee "$ROOT/evidence/build.log"
source /opt/ros/humble/setup.bash
WS=${EMBODIED_ROS_WS:-$HOME/embodied_acceptance_ws}
cd "$WS"
colcon test --packages-select embodied_swarm_hardware --event-handlers console_direct+ 2>&1 | tee "$ROOT/evidence/gtest.log"
colcon test-result --verbose 2>&1 | tee "$ROOT/evidence/gtest_result.log"
{
  date -Iseconds
  uname -a
  lscpu
  free -m
  python3 --version
  python3 -c 'import mujoco; print("MuJoCo",mujoco.__version__)'
  dpkg-query -W ros-humble-moveit-core ros-humble-moveit-planners-ompl ros-humble-controller-manager ros-humble-joint-trajectory-controller
} > "$ROOT/evidence/environment.txt"
/usr/bin/time -v -o "$ROOT/evidence/mock_runtime.txt" bash "$ROOT/accept.sh" --backend mock
/usr/bin/time -v -o "$ROOT/evidence/mujoco_runtime.txt" bash "$ROOT/accept.sh" --backend mujoco
