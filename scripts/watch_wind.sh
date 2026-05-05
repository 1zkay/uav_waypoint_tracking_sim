#!/usr/bin/env bash
set -eo pipefail

PX4_GZ_WORLD="${PX4_GZ_WORLD:-trajectory_tracking}"

exec gz topic -e -t "/world/${PX4_GZ_WORLD}/wind"
