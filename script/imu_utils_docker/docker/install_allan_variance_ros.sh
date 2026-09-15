#!/usr/bin/env bash
set -eo pipefail
REV=1d54b602ee7f2ba0427865d63afe4945d913ed24
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  libyaml-cpp-dev libeigen3-dev python3-scipy ros-noetic-tf2-geometry-msgs
mkdir -p /allan_ws/src
if [ ! -d /allan_ws/src/allan_variance_ros/.git ]; then
  git clone https://github.com/ori-drs/allan_variance_ros.git /allan_ws/src/allan_variance_ros
fi
git -C /allan_ws/src/allan_variance_ros checkout --detach "$REV"
source /opt/ros/noetic/setup.bash
cd /allan_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -j2
git -C src/allan_variance_ros rev-parse HEAD > upstream_revision.txt
