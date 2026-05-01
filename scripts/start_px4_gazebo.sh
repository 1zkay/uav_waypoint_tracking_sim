#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/zk/PX4-Autopilot}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-waypoint_tracking}"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${SIM_ROOT}/src/uav_waypoint_tracking/config/waypoints.yaml}"
WIND_FILE="${WIND_FILE:-${SIM_ROOT}/src/uav_waypoint_tracking/config/wind.yaml}"

if [[ "${PX4_GZ_WORLD}" == "waypoint_tracking" ]]; then
  GENERATED_WORLD="${SIM_ROOT}/build/generated/worlds/waypoint_tracking.sdf"
  echo "Rendering Gazebo waypoint world from ${WAYPOINTS_FILE} and ${WIND_FILE}"
  "${SCRIPT_DIR}/render_waypoint_world.py" \
    --base-world "${SIM_ROOT}/px4_overlays/worlds/waypoint_tracking.sdf" \
    --waypoints "${WAYPOINTS_FILE}" \
    --wind-config "${WIND_FILE}" \
    --output "${GENERATED_WORLD}"
  install -D -m 0644 \
    "${GENERATED_WORLD}" \
    "${PX4_ROOT}/Tools/simulation/gz/worlds/waypoint_tracking.sdf"

  if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
    export GZ_SIM_RESOURCE_PATH="${SIM_ROOT}/px4_overlays/models:${GZ_SIM_RESOURCE_PATH}"
  else
    export GZ_SIM_RESOURCE_PATH="${SIM_ROOT}/px4_overlays/models"
  fi
  export PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_0}"
fi

cd "${PX4_ROOT}"
source /home/zk/px4-venv/bin/activate

export PX4_GZ_WORLD

exec make px4_sitl gz_x500_gimbal
