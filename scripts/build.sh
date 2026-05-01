#!/usr/bin/env bash
set -eo pipefail

cd /home/zk/uav_waypoint_tracking_sim
PYTHON_VENV="${PYTHON_VENV:-/home/zk/px4-venv}"
if [[ -f "${PYTHON_VENV}/bin/activate" ]]; then
  source "${PYTHON_VENV}/bin/activate"
fi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
