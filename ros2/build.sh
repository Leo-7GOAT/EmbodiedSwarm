#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
ROOT=$(cd "$(dirname "$0")" && pwd)
WS=${EMBODIED_ROS_WS:-$HOME/embodied_acceptance_ws}
mkdir -p "$WS/src"
TARGET="$WS/src/embodied_swarm_hardware"
if [[ -e "$TARGET" || -L "$TARGET" ]]; then
  [[ -L "$TARGET" && "$(readlink -f "$TARGET")" == "$(readlink -f "$ROOT/embodied_swarm_hardware")" ]] || { echo "Refusing to overwrite existing package: $TARGET" >&2; exit 1; }
else
  ln -s "$ROOT/embodied_swarm_hardware" "$TARGET"
fi
cd "$WS"
colcon build --packages-select embodied_swarm_hardware --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo "source $WS/install/setup.bash"
