#!/usr/bin/env bash
set -eo pipefail

cd /home/zk/uav_waypoint_tracking_sim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
