#!/usr/bin/env bash
set -eo pipefail

PX4_GZ_WORLD="${PX4_GZ_WORLD:-waypoint_tracking}"
WIND_EAST="${1:-3.0}"
WIND_NORTH="${2:-0.0}"
WIND_UP="${3:-0.0}"

gz topic \
  -t "/world/${PX4_GZ_WORLD}/wind" \
  -m gz.msgs.Wind \
  -p "linear_velocity: {x: ${WIND_EAST}, y: ${WIND_NORTH}, z: ${WIND_UP}}, enable_wind: true"
