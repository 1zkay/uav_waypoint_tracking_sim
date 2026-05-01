#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${SIM_ROOT}/src/uav_waypoint_tracking/config/waypoints.yaml}"
LOG_ROOT="${LOG_ROOT:-}"
RUN_ID="${RUN_ID:-}"
PYTHON_VENV="${PYTHON_VENV:-/home/zk/px4-venv}"
ENABLE_CAMERA_BRIDGE="${ENABLE_CAMERA_BRIDGE:-true}"
ENABLE_YOLO_DETECTION="${ENABLE_YOLO_DETECTION:-true}"
CAMERA_GAZEBO_TOPIC="${CAMERA_GAZEBO_TOPIC:-/world/waypoint_tracking/model/x500_0/link/camera_link/sensor/camera/image}"
CAMERA_IMAGE_TOPIC="${CAMERA_IMAGE_TOPIC:-/x500_0/camera/image_raw}"
YOLO_WEIGHTS_PATH="${YOLO_WEIGHTS_PATH:-${SIM_ROOT}/yolov8s.pt}"
YOLO_DETECTIONS_TOPIC="${YOLO_DETECTIONS_TOPIC:-/x500_0/yolo/detections}"
YOLO_ANNOTATED_IMAGE_TOPIC="${YOLO_ANNOTATED_IMAGE_TOPIC:-/x500_0/yolo/image_annotated}"

cd "${SIM_ROOT}"
if [[ -f "${PYTHON_VENV}/bin/activate" ]]; then
  source "${PYTHON_VENV}/bin/activate"
  VENV_SITE_PACKAGES="$("${PYTHON_VENV}/bin/python" - <<'PY'
import site
print(site.getsitepackages()[0])
PY
)"
  export PYTHONPATH="${VENV_SITE_PACKAGES}:${PYTHONPATH:-}"
fi
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

launch_arg_value() {
  local name="$1"
  local fallback="$2"
  local arg
  for arg in "${launch_args[@]}"; do
    if [[ "${arg}" == "${name}:="* ]]; then
      printf '%s' "${arg#${name}:=}"
      return
    fi
  done
  printf '%s' "${fallback}"
}

if ! has_launch_arg "waypoints_file"; then
  launch_args+=("waypoints_file:=${WAYPOINTS_FILE}")
fi
if ! has_launch_arg "enable_camera_bridge"; then
  launch_args+=("enable_camera_bridge:=${ENABLE_CAMERA_BRIDGE}")
fi
if ! has_launch_arg "enable_yolo_detection"; then
  launch_args+=("enable_yolo_detection:=${ENABLE_YOLO_DETECTION}")
fi
if ! has_launch_arg "camera_gazebo_topic"; then
  launch_args+=("camera_gazebo_topic:=${CAMERA_GAZEBO_TOPIC}")
fi
if ! has_launch_arg "camera_image_topic"; then
  launch_args+=("camera_image_topic:=${CAMERA_IMAGE_TOPIC}")
fi
if ! has_launch_arg "yolo_weights_path"; then
  launch_args+=("yolo_weights_path:=${YOLO_WEIGHTS_PATH}")
fi
if ! has_launch_arg "yolo_detections_topic"; then
  launch_args+=("yolo_detections_topic:=${YOLO_DETECTIONS_TOPIC}")
fi
if ! has_launch_arg "yolo_annotated_image_topic"; then
  launch_args+=("yolo_annotated_image_topic:=${YOLO_ANNOTATED_IMAGE_TOPIC}")
fi
if [[ -n "${LOG_ROOT}" ]] && ! has_launch_arg "log_root"; then
  launch_args+=("log_root:=${LOG_ROOT}")
fi
if [[ -n "${RUN_ID}" ]] && ! has_launch_arg "run_id"; then
  launch_args+=("run_id:=${RUN_ID}")
fi

echo "Launching waypoint tracker with ${WAYPOINTS_FILE}"
echo "Camera bridge: $(launch_arg_value enable_camera_bridge "${ENABLE_CAMERA_BRIDGE}") $(launch_arg_value camera_gazebo_topic "${CAMERA_GAZEBO_TOPIC}") -> $(launch_arg_value camera_image_topic "${CAMERA_IMAGE_TOPIC}")"
echo "YOLO detection: $(launch_arg_value enable_yolo_detection "${ENABLE_YOLO_DETECTION}") weights=$(launch_arg_value yolo_weights_path "${YOLO_WEIGHTS_PATH}")"
exec ros2 launch uav_waypoint_tracking waypoint_tracking.launch.py "${launch_args[@]}"
