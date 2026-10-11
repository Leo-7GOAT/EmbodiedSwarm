#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
WS=${EMBODIED_ROS_WS:-$HOME/embodied_acceptance_ws}
source "$WS/install/setup.bash"
ROOT=$(cd "$(dirname "$0")" && pwd)
python3 "$ROOT/run_acceptance.py" "$@"
