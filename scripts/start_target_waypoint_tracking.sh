#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${SIM_ROOT}/src/uav_waypoint_tracking/config/target_waypoints.yaml}"
PX4_ROS_NAMESPACE="${PX4_ROS_NAMESPACE:-/px4_1}"
TARGET_NODE_NAMESPACE="${TARGET_NODE_NAMESPACE:-target}"
TARGET_SYSTEM="${TARGET_SYSTEM:-2}"
TARGET_COMPONENT="${TARGET_COMPONENT:-1}"
SOURCE_SYSTEM="${SOURCE_SYSTEM:-1}"
SOURCE_COMPONENT="${SOURCE_COMPONENT:-1}"
TARGET_GZ_MODEL_NAME="${TARGET_GZ_MODEL_NAME:-x500_1}"
LOG_ROOT="${LOG_ROOT:-}"
RUN_ID="${RUN_ID:-target_$(date +%Y%m%d_%H%M%S)}"

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

add_launch_arg() {
  local name="$1"
  local value="$2"
  if ! has_launch_arg "${name}"; then
    launch_args+=("${name}:=${value}")
  fi
}

add_launch_arg "node_namespace" "${TARGET_NODE_NAMESPACE}"
add_launch_arg "waypoints_file" "${WAYPOINTS_FILE}"
add_launch_arg "vehicle_status_topic" "${PX4_ROS_NAMESPACE}/fmu/out/vehicle_status_v1"
add_launch_arg "vehicle_local_position_topic" "${PX4_ROS_NAMESPACE}/fmu/out/vehicle_local_position_v1"
add_launch_arg "vehicle_attitude_topic" "${PX4_ROS_NAMESPACE}/fmu/out/vehicle_attitude"
add_launch_arg "vehicle_odometry_topic" "${PX4_ROS_NAMESPACE}/fmu/out/vehicle_odometry"
add_launch_arg "offboard_control_mode_topic" "${PX4_ROS_NAMESPACE}/fmu/in/offboard_control_mode"
add_launch_arg "trajectory_setpoint_topic" "${PX4_ROS_NAMESPACE}/fmu/in/trajectory_setpoint"
add_launch_arg "vehicle_command_topic" "${PX4_ROS_NAMESPACE}/fmu/in/vehicle_command"
add_launch_arg "current_index_topic" "/${TARGET_NODE_NAMESPACE}/waypoint_tracker/current_waypoint_index"
add_launch_arg "target_system" "${TARGET_SYSTEM}"
add_launch_arg "target_component" "${TARGET_COMPONENT}"
add_launch_arg "source_system" "${SOURCE_SYSTEM}"
add_launch_arg "source_component" "${SOURCE_COMPONENT}"
add_launch_arg "gazebo_odometry_topic" "/model/${TARGET_GZ_MODEL_NAME}/odometry_with_covariance"
add_launch_arg "waypoint_markers_topic" "/${TARGET_NODE_NAMESPACE}/waypoint_markers"
add_launch_arg "waypoint_path_topic" "/${TARGET_NODE_NAMESPACE}/waypoint_path"
add_launch_arg "vehicle_path_topic" "/${TARGET_NODE_NAMESPACE}/vehicle_path"
add_launch_arg "run_id" "${RUN_ID}"

# Target UAV tracking should only control the target vehicle trajectory.
# Keep the host-camera vision and gimbal-servo pipeline owned by
# scripts/start_waypoint_tracking.sh to avoid duplicate subscribers and
# publishers on /x500_0/camera/* and /x500_0/yolo/*.
add_launch_arg "enable_camera_bridge" "false"
add_launch_arg "enable_yolo_tracking" "false"
add_launch_arg "enable_yolo_annotation" "false"
add_launch_arg "enable_gimbal_tracking" "false"
add_launch_arg "enable_gimbal_performance_monitor" "false"

if [[ -n "${LOG_ROOT}" ]]; then
  add_launch_arg "log_root" "${LOG_ROOT}"
fi

echo "Launching target waypoint tracker for ${PX4_ROS_NAMESPACE} with ${WAYPOINTS_FILE}"
echo "Target vision/gimbal pipeline: disabled"
exec ros2 launch uav_waypoint_tracking waypoint_tracking.launch.py "${launch_args[@]}"
