#!/usr/bin/bash

set -e

if [ -z "${ROS_DISTRO}" ]; then
    echo "ROS_DISTRO: 1) noetic, 2) humble"
    read -p "Enter the number corresponding to your ROS distribution: " distro_choice
    if [ "$distro_choice" -eq 1 ]; then
        export ROS_DISTRO="noetic"
    elif [ "$distro_choice" -eq 2 ]; then
        export ROS_DISTRO="humble"
    else
        echo "Invalid choice. Please run the script again and select either 1 or 2."
        exit 1
    fi
fi

SCRIPT_DIR=$(cd $(dirname $0); pwd)
if [ ${ROS_DISTRO} = "noetic" ]; then
    echo "Building for ROS Noetic..."
    cd ${SCRIPT_DIR}/../../erl_common/docker/ubuntu-2004
    ./build.bash
    cd ${SCRIPT_DIR}/../../erl_geometry/docker/ubuntu-2004
    ./build.bash
    cd ${SCRIPT_DIR}/../docker/ros-noetic
    ./build.bash
elif [ ${ROS_DISTRO} = "humble" ]; then
    echo "Building for ROS Humble..."
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
