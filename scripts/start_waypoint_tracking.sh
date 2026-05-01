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
ENABLE_GIMBAL_TRACKING="${ENABLE_GIMBAL_TRACKING:-false}"
CAMERA_GAZEBO_TOPIC="${CAMERA_GAZEBO_TOPIC:-/world/waypoint_tracking/model/x500_0/link/camera_link/sensor/camera/image}"
CAMERA_IMAGE_TOPIC="${CAMERA_IMAGE_TOPIC:-/x500_0/camera/image_raw}"
YOLO_WEIGHTS_PATH="${YOLO_WEIGHTS_PATH:-${SIM_ROOT}/yolov8s.pt}"
YOLO_DETECTIONS_TOPIC="${YOLO_DETECTIONS_TOPIC:-/x500_0/yolo/detections}"
YOLO_ANNOTATED_IMAGE_TOPIC="${YOLO_ANNOTATED_IMAGE_TOPIC:-/x500_0/yolo/image_annotated}"
GIMBAL_TARGET_CLASS_ID="${GIMBAL_TARGET_CLASS_ID:-}"
GIMBAL_MIN_SCORE="${GIMBAL_MIN_SCORE:-0.25}"
GIMBAL_YAW_RATE_GAIN_DEG_S="${GIMBAL_YAW_RATE_GAIN_DEG_S:-45.0}"
GIMBAL_PITCH_RATE_GAIN_DEG_S="${GIMBAL_PITCH_RATE_GAIN_DEG_S:-35.0}"
GIMBAL_YAW_ERROR_SIGN="${GIMBAL_YAW_ERROR_SIGN:-1.0}"
GIMBAL_PITCH_ERROR_SIGN="${GIMBAL_PITCH_ERROR_SIGN:--1.0}"

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

add_launch_arg() {
  local name="$1"
  local value="$2"
  if ! has_launch_arg "${name}"; then
    launch_args+=("${name}:=${value}")
  fi
}

add_launch_arg "waypoints_file" "${WAYPOINTS_FILE}"
add_launch_arg "enable_camera_bridge" "${ENABLE_CAMERA_BRIDGE}"
add_launch_arg "enable_yolo_detection" "${ENABLE_YOLO_DETECTION}"
add_launch_arg "enable_gimbal_tracking" "${ENABLE_GIMBAL_TRACKING}"
add_launch_arg "camera_gazebo_topic" "${CAMERA_GAZEBO_TOPIC}"
add_launch_arg "camera_image_topic" "${CAMERA_IMAGE_TOPIC}"
add_launch_arg "yolo_weights_path" "${YOLO_WEIGHTS_PATH}"
add_launch_arg "yolo_detections_topic" "${YOLO_DETECTIONS_TOPIC}"
add_launch_arg "yolo_annotated_image_topic" "${YOLO_ANNOTATED_IMAGE_TOPIC}"
add_launch_arg "gimbal_target_class_id" "${GIMBAL_TARGET_CLASS_ID}"
add_launch_arg "gimbal_min_score" "${GIMBAL_MIN_SCORE}"
add_launch_arg "gimbal_yaw_rate_gain_deg_s" "${GIMBAL_YAW_RATE_GAIN_DEG_S}"
add_launch_arg "gimbal_pitch_rate_gain_deg_s" "${GIMBAL_PITCH_RATE_GAIN_DEG_S}"
add_launch_arg "gimbal_yaw_error_sign" "${GIMBAL_YAW_ERROR_SIGN}"
add_launch_arg "gimbal_pitch_error_sign" "${GIMBAL_PITCH_ERROR_SIGN}"

if [[ -n "${LOG_ROOT}" ]]; then
  add_launch_arg "log_root" "${LOG_ROOT}"
fi
if [[ -n "${RUN_ID}" ]]; then
  add_launch_arg "run_id" "${RUN_ID}"
fi

echo "Launching waypoint tracker with ${WAYPOINTS_FILE}"
echo "Camera bridge: $(launch_arg_value enable_camera_bridge "${ENABLE_CAMERA_BRIDGE}") $(launch_arg_value camera_gazebo_topic "${CAMERA_GAZEBO_TOPIC}") -> $(launch_arg_value camera_image_topic "${CAMERA_IMAGE_TOPIC}")"
echo "YOLO detection: $(launch_arg_value enable_yolo_detection "${ENABLE_YOLO_DETECTION}") weights=$(launch_arg_value yolo_weights_path "${YOLO_WEIGHTS_PATH}")"
echo "Gimbal tracking: $(launch_arg_value enable_gimbal_tracking "${ENABLE_GIMBAL_TRACKING}") target_class=$(launch_arg_value gimbal_target_class_id "${GIMBAL_TARGET_CLASS_ID}")"
exec ros2 launch uav_waypoint_tracking waypoint_tracking.launch.py "${launch_args[@]}"
