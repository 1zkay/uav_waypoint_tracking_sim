#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_VENV="${PYTHON_VENV:-/home/zk/px4-venv}"

VISUAL_INTERCEPTION_CONFIG_FILE="${VISUAL_INTERCEPTION_CONFIG_FILE:-${SIM_ROOT}/src/uav_trajectory_tracking/config/visual_interception.yaml}"
ENABLE_CAMERA_BRIDGE="${ENABLE_CAMERA_BRIDGE:-true}"
ENABLE_YOLO_TRACKING="${ENABLE_YOLO_TRACKING:-true}"
ENABLE_YOLO_ANNOTATION="${ENABLE_YOLO_ANNOTATION:-true}"
ENABLE_GIMBAL_TRACKING="${ENABLE_GIMBAL_TRACKING:-true}"
ENABLE_GIMBAL_PERFORMANCE_MONITOR="${ENABLE_GIMBAL_PERFORMANCE_MONITOR:-${ENABLE_GIMBAL_TRACKING}}"

CAMERA_IMAGE_BRIDGE_QOS="${CAMERA_IMAGE_BRIDGE_QOS:-default}"
CAMERA_GAZEBO_TOPIC="${CAMERA_GAZEBO_TOPIC:-/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/image}"
CAMERA_IMAGE_TOPIC="${CAMERA_IMAGE_TOPIC:-/x500_0/camera/image_raw}"
CAMERA_INFO_GAZEBO_TOPIC="${CAMERA_INFO_GAZEBO_TOPIC:-/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/camera_info}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/x500_0/camera/camera_info}"

YOLO_WEIGHTS_PATH="${YOLO_WEIGHTS_PATH:-${SIM_ROOT}/yolov8s.pt}"
YOLO_TRACKS_TOPIC="${YOLO_TRACKS_TOPIC:-/x500_0/yolo/tracks}"
YOLO_TRACKS_ANNOTATED_IMAGE_TOPIC="${YOLO_TRACKS_ANNOTATED_IMAGE_TOPIC:-/x500_0/yolo/tracks_image}"
YOLO_ANNOTATION_MAX_PUBLISH_HZ="${YOLO_ANNOTATION_MAX_PUBLISH_HZ:-15.0}"

GIMBAL_INPUT_TOPIC="${GIMBAL_INPUT_TOPIC:-${YOLO_TRACKS_TOPIC}}"
GIMBAL_JOINT_STATE_GAZEBO_TOPIC="${GIMBAL_JOINT_STATE_GAZEBO_TOPIC:-/world/trajectory_tracking/model/x500_0/joint_state}"
GIMBAL_JOINT_STATE_TOPIC="${GIMBAL_JOINT_STATE_TOPIC:-/x500_0/gimbal/joint_states}"
GIMBAL_SET_ATTITUDE_TOPIC="${GIMBAL_SET_ATTITUDE_TOPIC:-/fmu/in/gimbal_manager_set_attitude}"
GIMBAL_TRACKING_ACTIVE_TOPIC="${GIMBAL_TRACKING_ACTIVE_TOPIC:-/x500_0/gimbal_target_tracker/tracking_active}"
GIMBAL_PERFORMANCE_METRICS_TOPIC="${GIMBAL_PERFORMANCE_METRICS_TOPIC:-/x500_0/gimbal_performance/metrics}"

VEHICLE_STATUS_TOPIC="${VEHICLE_STATUS_TOPIC:-/fmu/out/vehicle_status_v4}"
VEHICLE_LOCAL_POSITION_TOPIC="${VEHICLE_LOCAL_POSITION_TOPIC:-/fmu/out/vehicle_local_position_v1}"
VEHICLE_ATTITUDE_TOPIC="${VEHICLE_ATTITUDE_TOPIC:-/fmu/out/vehicle_attitude}"
OFFBOARD_CONTROL_MODE_TOPIC="${OFFBOARD_CONTROL_MODE_TOPIC:-/fmu/in/offboard_control_mode}"
TRAJECTORY_SETPOINT_TOPIC="${TRAJECTORY_SETPOINT_TOPIC:-/fmu/in/trajectory_setpoint}"
VEHICLE_COMMAND_TOPIC="${VEHICLE_COMMAND_TOPIC:-/fmu/in/vehicle_command}"
VEHICLE_COMMAND_ACK_TOPIC="${VEHICLE_COMMAND_ACK_TOPIC:-/fmu/out/vehicle_command_ack}"
TARGET_SYSTEM="${TARGET_SYSTEM:-1}"
TARGET_COMPONENT="${TARGET_COMPONENT:-1}"
SOURCE_SYSTEM="${SOURCE_SYSTEM:-1}"
SOURCE_COMPONENT="${SOURCE_COMPONENT:-1}"
VISUAL_INTERCEPTION_DIAGNOSTICS_TOPIC="${VISUAL_INTERCEPTION_DIAGNOSTICS_TOPIC:-/x500_0/visual_pursuit_interceptor/diagnostics}"

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

add_launch_arg "visual_interception_config_file" "${VISUAL_INTERCEPTION_CONFIG_FILE}"
add_launch_arg "enable_camera_bridge" "${ENABLE_CAMERA_BRIDGE}"
add_launch_arg "camera_image_bridge_qos" "${CAMERA_IMAGE_BRIDGE_QOS}"
add_launch_arg "camera_gazebo_topic" "${CAMERA_GAZEBO_TOPIC}"
add_launch_arg "camera_image_topic" "${CAMERA_IMAGE_TOPIC}"
add_launch_arg "camera_info_gazebo_topic" "${CAMERA_INFO_GAZEBO_TOPIC}"
add_launch_arg "camera_info_topic" "${CAMERA_INFO_TOPIC}"
add_launch_arg "enable_yolo_tracking" "${ENABLE_YOLO_TRACKING}"
add_launch_arg "enable_yolo_annotation" "${ENABLE_YOLO_ANNOTATION}"
add_launch_arg "yolo_weights_path" "${YOLO_WEIGHTS_PATH}"
add_launch_arg "yolo_tracks_topic" "${YOLO_TRACKS_TOPIC}"
add_launch_arg "yolo_tracks_annotated_image_topic" "${YOLO_TRACKS_ANNOTATED_IMAGE_TOPIC}"
add_launch_arg "yolo_annotation_max_publish_hz" "${YOLO_ANNOTATION_MAX_PUBLISH_HZ}"
add_launch_arg "enable_gimbal_tracking" "${ENABLE_GIMBAL_TRACKING}"
add_launch_arg "gimbal_input_topic" "${GIMBAL_INPUT_TOPIC}"
add_launch_arg "gimbal_joint_state_gazebo_topic" "${GIMBAL_JOINT_STATE_GAZEBO_TOPIC}"
add_launch_arg "gimbal_joint_state_topic" "${GIMBAL_JOINT_STATE_TOPIC}"
add_launch_arg "gimbal_set_attitude_topic" "${GIMBAL_SET_ATTITUDE_TOPIC}"
add_launch_arg "gimbal_tracking_active_topic" "${GIMBAL_TRACKING_ACTIVE_TOPIC}"
add_launch_arg "enable_gimbal_performance_monitor" "${ENABLE_GIMBAL_PERFORMANCE_MONITOR}"
add_launch_arg "gimbal_performance_metrics_topic" "${GIMBAL_PERFORMANCE_METRICS_TOPIC}"
add_launch_arg "vehicle_status_topic" "${VEHICLE_STATUS_TOPIC}"
add_launch_arg "vehicle_local_position_topic" "${VEHICLE_LOCAL_POSITION_TOPIC}"
add_launch_arg "vehicle_attitude_topic" "${VEHICLE_ATTITUDE_TOPIC}"
add_launch_arg "offboard_control_mode_topic" "${OFFBOARD_CONTROL_MODE_TOPIC}"
add_launch_arg "trajectory_setpoint_topic" "${TRAJECTORY_SETPOINT_TOPIC}"
add_launch_arg "vehicle_command_topic" "${VEHICLE_COMMAND_TOPIC}"
add_launch_arg "vehicle_command_ack_topic" "${VEHICLE_COMMAND_ACK_TOPIC}"
add_launch_arg "target_system" "${TARGET_SYSTEM}"
add_launch_arg "target_component" "${TARGET_COMPONENT}"
add_launch_arg "source_system" "${SOURCE_SYSTEM}"
add_launch_arg "source_component" "${SOURCE_COMPONENT}"
add_launch_arg "visual_interception_diagnostics_topic" "${VISUAL_INTERCEPTION_DIAGNOSTICS_TOPIC}"

if [[ -n "${YOLO_TRACKING_CONFIG_FILE:-}" ]]; then
  add_launch_arg "yolo_tracking_config_file" "${YOLO_TRACKING_CONFIG_FILE}"
fi
if [[ -n "${GIMBAL_CONFIG_FILE:-}" ]]; then
  add_launch_arg "gimbal_config_file" "${GIMBAL_CONFIG_FILE}"
fi

echo "Launching visual interception with $(launch_arg_value visual_interception_config_file "${VISUAL_INTERCEPTION_CONFIG_FILE}")"
echo "Camera bridge: $(launch_arg_value enable_camera_bridge "${ENABLE_CAMERA_BRIDGE}") qos=$(launch_arg_value camera_image_bridge_qos "${CAMERA_IMAGE_BRIDGE_QOS}") $(launch_arg_value camera_gazebo_topic "${CAMERA_GAZEBO_TOPIC}") -> $(launch_arg_value camera_image_topic "${CAMERA_IMAGE_TOPIC}")"
echo "YOLO tracking: $(launch_arg_value enable_yolo_tracking "${ENABLE_YOLO_TRACKING}") tracks=$(launch_arg_value yolo_tracks_topic "${YOLO_TRACKS_TOPIC}")"
echo "YOLO annotation: $(launch_arg_value enable_yolo_annotation "${ENABLE_YOLO_ANNOTATION}") annotated=$(launch_arg_value yolo_tracks_annotated_image_topic "${YOLO_TRACKS_ANNOTATED_IMAGE_TOPIC}") max_hz=$(launch_arg_value yolo_annotation_max_publish_hz "${YOLO_ANNOTATION_MAX_PUBLISH_HZ}")"
echo "Gimbal tracking: $(launch_arg_value enable_gimbal_tracking "${ENABLE_GIMBAL_TRACKING}") input=$(launch_arg_value gimbal_input_topic "${GIMBAL_INPUT_TOPIC}") joint_state=$(launch_arg_value gimbal_joint_state_gazebo_topic "${GIMBAL_JOINT_STATE_GAZEBO_TOPIC}") -> $(launch_arg_value gimbal_joint_state_topic "${GIMBAL_JOINT_STATE_TOPIC}")"
echo "Visual pursuit diagnostics: $(launch_arg_value visual_interception_diagnostics_topic "${VISUAL_INTERCEPTION_DIAGNOSTICS_TOPIC}")"
exec ros2 launch uav_trajectory_tracking visual_interception.launch.py "${launch_args[@]}"
