#!/usr/bin/env bash
set -eo pipefail

cd /home/zk/uav_trajectory_tracking_sim
source /opt/ros/jazzy/setup.bash
source /home/zk/uav_trajectory_tracking_sim/install/setup.bash

RVIZ_CONFIG="${RVIZ_CONFIG:-/home/zk/uav_trajectory_tracking_sim/src/uav_trajectory_tracking/rviz/trajectory_tracking.rviz}"

exec rviz2 -d "${RVIZ_CONFIG}"
