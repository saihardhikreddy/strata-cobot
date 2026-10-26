#!/usr/bin/env bash
# Start the Strata Cobot dashboard: sources ROS 2 and the simulation workspace, then opens
# http://localhost:8765 in the browser. Set ROS2_WS if the workspace is not ~/ros2_ws.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "${ROS2_WS:-$HOME/ros2_ws}/install/setup.bash"
exec python3 "$HERE/app.py" "$@"
