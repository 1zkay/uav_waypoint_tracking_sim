#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/zk/PX4-Autopilot}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-waypoint_tracking}"
PX4_INSTANCE="${PX4_INSTANCE:-1}"
PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_1}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"

if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
  export GZ_SIM_RESOURCE_PATH="${SIM_ROOT}/px4_overlays/models:${GZ_SIM_RESOURCE_PATH}"
else
  export GZ_SIM_RESOURCE_PATH="${SIM_ROOT}/px4_overlays/models"
fi

cd "${PX4_ROOT}"
source /home/zk/px4-venv/bin/activate

export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD
export PX4_GZ_MODEL_NAME
export PX4_SYS_AUTOSTART

exec ./build/px4_sitl_default/bin/px4 -i "${PX4_INSTANCE}"
