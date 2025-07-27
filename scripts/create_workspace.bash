#!/usr/bin/bash

WORKSPACE_DIR=${WORKSPACE_DIR:-"ros_ws_gp_sdf"}
mkdir -p ${WORKSPACE_DIR}/src
cd ${WORKSPACE_DIR}/src
for repo in erl_cmake_tools \
            erl_common \
            erl_common_ros \
            erl_covariance \
            erl_gaussian_process \
            erl_geometry \
            erl_geometry_msgs \
            erl_geometry_ros \
            erl_geometry_rviz_plugin \
            erl_gp_sdf \
            erl_gp_sdf_msgs \
            erl_gp_sdf_ros; do
    if [ ! -d ${repo} ]; then
        git clone https://github.com/ExistentialRobotics/${repo}.git -b main
    else
        echo "Repository ${repo} already exists, skipping clone."
    fi
done
