#!/usr/bin/env bash
set -eo pipefail

source /home/zk/px4-venv/bin/activate
exec MicroXRCEAgent udp4 -p 8888
