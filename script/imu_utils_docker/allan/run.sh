#!/usr/bin/env bash
set -eo pipefail
load_ros_environment() {
  source /opt/ros/noetic/setup.bash
  source /imu_ws/devel/setup.bash
}
load_ros_environment
export PYTHONDONTWRITEBYTECODE=1
exec python3 -u /scripts/run_analysis.py "$@"
