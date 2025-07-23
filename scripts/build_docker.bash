#!/usr/bin/bash

if [ -z "${ROS_DISTRO}" ]; then
    echo "ROS_DISTRO is not set. Please run this script with ROS_DISTRO set to your desired ROS version."
    exit 1
fi

SCRIPT_DIR=$(cd $(dirname $0); pwd)
if [ ${ROS_DISTRO} = "noetic" ]; then
    cd ${SCRIPT_DIR}/../../erl_common/docker/ubuntu-2004
    ./build.bash
    cd ${SCRIPT_DIR}/../../erl_geometry/docker/ubuntu-2004
    ./build.bash
    cd ${SCRIPT_DIR}/../docker/ros-noetic
    ./build.bash
elif [ ${ROS_DISTRO} = "humble" ]; then
    cd ${SCRIPT_DIR}/../../erl_common/docker/ubuntu-2204
    ./build.bash
    cd ${SCRIPT_DIR}/../../erl_geometry/docker/ubuntu-2204
    ./build.bash
    cd ${SCRIPT_DIR}/../docker/ros-humble
    ./build.bash
else
    echo "Unsupported ROS distribution: ${ROS_DISTRO}. Supported distributions are 'noetic' and 'humble'."
    exit 1
fi
