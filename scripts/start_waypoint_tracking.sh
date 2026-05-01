#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${SIM_ROOT}/src/uav_waypoint_tracking/config/waypoints.yaml}"
LOG_ROOT="${LOG_ROOT:-}"
RUN_ID="${RUN_ID:-}"

cd "${SIM_ROOT}"
source /opt/ros/jazzy/setup.bash
source "${SIM_ROOT}/install/setup.bash"

launch_args=("$@")

has_launch_arg() {
  local name="$1"
  local arg
  for arg in "${launch_args[@]}"; do
    if [[ "${arg}" == "${name}:="* ]]; then
      return 0
    fi
  done
  return 1
}

if ! has_launch_arg "waypoints_file"; then
  launch_args+=("waypoints_file:=${WAYPOINTS_FILE}")
fi
if [[ -n "${LOG_ROOT}" ]] && ! has_launch_arg "log_root"; then
  launch_args+=("log_root:=${LOG_ROOT}")
fi
if [[ -n "${RUN_ID}" ]] && ! has_launch_arg "run_id"; then
  launch_args+=("run_id:=${RUN_ID}")
fi

echo "Launching waypoint tracker with ${WAYPOINTS_FILE}"
exec ros2 launch uav_waypoint_tracking waypoint_tracking.launch.py "${launch_args[@]}"
