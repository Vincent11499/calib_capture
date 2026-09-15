#!/usr/bin/env bash
set -eo pipefail
load_ros_environment() {
  source /opt/ros/noetic/setup.bash
  source /allan_ws/devel/setup.bash
}
load_ros_environment
export PYTHONDONTWRITEBYTECODE=1
exec python3 -u /scripts/run_allan_variance_ros.py "$@"
