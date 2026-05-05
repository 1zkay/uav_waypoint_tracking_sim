#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/zk/PX4-Autopilot}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-trajectory_tracking}"
TRAJECTORY_FILE="${TRAJECTORY_FILE:-${SIM_ROOT}/src/uav_trajectory_tracking/config/trajectory_figure8.yaml}"
WIND_FILE="${WIND_FILE:-${SIM_ROOT}/src/uav_trajectory_tracking/config/wind.yaml}"
SHOW_TRAJECTORY_VISUALS="${SHOW_TRAJECTORY_VISUALS:-false}"

if [[ "${PX4_GZ_WORLD}" == "trajectory_tracking" ]]; then
  GENERATED_WORLD="${SIM_ROOT}/build/generated/worlds/trajectory_tracking.sdf"
  echo "Rendering Gazebo trajectory world from ${TRAJECTORY_FILE} and ${WIND_FILE}"
  render_args=(
    --base-world "${SIM_ROOT}/px4_overlays/worlds/trajectory_tracking.sdf"
    --trajectory-file "${TRAJECTORY_FILE}"
    --wind-config "${WIND_FILE}"
    --output "${GENERATED_WORLD}"
  )
  if [[ "${SHOW_TRAJECTORY_VISUALS}" == "true" ]]; then
    render_args+=(--include-visuals)
  fi
  "${SCRIPT_DIR}/render_trajectory_world.py" \
    "${render_args[@]}"
  install -D -m 0644 \
    "${GENERATED_WORLD}" \
    "${PX4_ROOT}/Tools/simulation/gz/worlds/trajectory_tracking.sdf"

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
