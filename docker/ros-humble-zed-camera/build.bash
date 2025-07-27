#! /usr/bin/bash

set -e
set -x
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
docker build --rm -t erl/ros-humble:zed-gp-sdf --build-arg ROS_APT_SOURCE_VERSION=$ROS_APT_SOURCE_VERSION . $@
