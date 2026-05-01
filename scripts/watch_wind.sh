#!/usr/bin/env bash
set -eo pipefail

PX4_GZ_WORLD="${PX4_GZ_WORLD:-waypoint_tracking}"

exec gz topic -e -t "/world/${PX4_GZ_WORLD}/wind"
