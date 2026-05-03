#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${SIM_ROOT}/src/uav_waypoint_tracking/config/waypoints.yaml}"
LOG_ROOT="${LOG_ROOT:-}"
RUN_ID="${RUN_ID:-}"
PYTHON_VENV="${PYTHON_VENV:-/home/zk/px4-venv}"
ENABLE_CAMERA_BRIDGE="${ENABLE_CAMERA_BRIDGE:-true}"
ENABLE_YOLO_TRACKING="${ENABLE_YOLO_TRACKING:-true}"
ENABLE_GIMBAL_TRACKING="${ENABLE_GIMBAL_TRACKING:-true}"
CAMERA_GAZEBO_TOPIC="${CAMERA_GAZEBO_TOPIC:-/world/waypoint_tracking/model/x500_0/link/camera_link/sensor/camera/image}"
CAMERA_IMAGE_TOPIC="${CAMERA_IMAGE_TOPIC:-/x500_0/camera/image_raw}"
CAMERA_INFO_GAZEBO_TOPIC="${CAMERA_INFO_GAZEBO_TOPIC:-/world/waypoint_tracking/model/x500_0/link/camera_link/sensor/camera/camera_info}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/x500_0/camera/camera_info}"
YOLO_WEIGHTS_PATH="${YOLO_WEIGHTS_PATH:-${SIM_ROOT}/yolov8s.pt}"
YOLO_TRACKS_TOPIC="${YOLO_TRACKS_TOPIC:-/x500_0/yolo/tracks}"
YOLO_TRACKS_ANNOTATED_IMAGE_TOPIC="${YOLO_TRACKS_ANNOTATED_IMAGE_TOPIC:-/x500_0/yolo/tracks_image}"
GIMBAL_INPUT_TOPIC="${GIMBAL_INPUT_TOPIC:-${YOLO_TRACKS_TOPIC}}"
GIMBAL_ATTITUDE_TOPIC="${GIMBAL_ATTITUDE_TOPIC:-/fmu/out/gimbal_device_attitude_status}"
GIMBAL_SET_ATTITUDE_TOPIC="${GIMBAL_SET_ATTITUDE_TOPIC:-/fmu/in/gimbal_manager_set_attitude}"

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
add_launch_arg "enable_yolo_tracking" "${ENABLE_YOLO_TRACKING}"
add_launch_arg "enable_gimbal_tracking" "${ENABLE_GIMBAL_TRACKING}"
add_launch_arg "camera_gazebo_topic" "${CAMERA_GAZEBO_TOPIC}"
add_launch_arg "camera_image_topic" "${CAMERA_IMAGE_TOPIC}"
add_launch_arg "camera_info_gazebo_topic" "${CAMERA_INFO_GAZEBO_TOPIC}"
add_launch_arg "camera_info_topic" "${CAMERA_INFO_TOPIC}"
add_launch_arg "yolo_weights_path" "${YOLO_WEIGHTS_PATH}"
add_launch_arg "yolo_tracks_topic" "${YOLO_TRACKS_TOPIC}"
add_launch_arg "yolo_tracks_annotated_image_topic" "${YOLO_TRACKS_ANNOTATED_IMAGE_TOPIC}"
add_launch_arg "gimbal_input_topic" "${GIMBAL_INPUT_TOPIC}"
add_launch_arg "gimbal_attitude_topic" "${GIMBAL_ATTITUDE_TOPIC}"
add_launch_arg "gimbal_set_attitude_topic" "${GIMBAL_SET_ATTITUDE_TOPIC}"

if [[ -n "${YOLO_TRACKING_CONFIG_FILE:-}" ]]; then
  add_launch_arg "yolo_tracking_config_file" "${YOLO_TRACKING_CONFIG_FILE}"
fi
if [[ -n "${GIMBAL_CONFIG_FILE:-}" ]]; then
  add_launch_arg "gimbal_config_file" "${GIMBAL_CONFIG_FILE}"
fi
if [[ -n "${LOG_ROOT}" ]]; then
  add_launch_arg "log_root" "${LOG_ROOT}"
fi
if [[ -n "${RUN_ID}" ]]; then
  add_launch_arg "run_id" "${RUN_ID}"
fi

echo "Launching waypoint tracker with ${WAYPOINTS_FILE}"
echo "Camera bridge: $(launch_arg_value enable_camera_bridge "${ENABLE_CAMERA_BRIDGE}") $(launch_arg_value camera_gazebo_topic "${CAMERA_GAZEBO_TOPIC}") -> $(launch_arg_value camera_image_topic "${CAMERA_IMAGE_TOPIC}")"
echo "CameraInfo bridge: $(launch_arg_value camera_info_gazebo_topic "${CAMERA_INFO_GAZEBO_TOPIC}") -> $(launch_arg_value camera_info_topic "${CAMERA_INFO_TOPIC}")"
echo "YOLO tracking: $(launch_arg_value enable_yolo_tracking "${ENABLE_YOLO_TRACKING}") tracks=$(launch_arg_value yolo_tracks_topic "${YOLO_TRACKS_TOPIC}")"
echo "Gimbal tracking: $(launch_arg_value enable_gimbal_tracking "${ENABLE_GIMBAL_TRACKING}") input=$(launch_arg_value gimbal_input_topic "${GIMBAL_INPUT_TOPIC}") attitude=$(launch_arg_value gimbal_attitude_topic "${GIMBAL_ATTITUDE_TOPIC}") setpoint=$(launch_arg_value gimbal_set_attitude_topic "${GIMBAL_SET_ATTITUDE_TOPIC}")"
exec ros2 launch uav_waypoint_tracking waypoint_tracking.launch.py "${launch_args[@]}"
