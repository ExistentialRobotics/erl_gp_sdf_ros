#!/usr/bin/env bash
#
# Build the ROS 1 (Noetic / Ubuntu 20.04) Docker image for erl_gp_sdf_ros,
# using the Dockerfiles that ship with each package.
#
#     bash scripts/build_docker_ros1.bash [extra docker build flags...]
#
# Extra flags are forwarded verbatim to every `docker build`, e.g.
#     bash scripts/build_docker_ros1.bash --no-cache --pull
#
# The erl_gp_sdf_ros image is built FROM erl/geometry, which is built FROM
# erl/common, so those two base images are built first (skip them if you
# already have them). The erl_common / erl_geometry packages are expected next
# to erl_gp_sdf_ros in the same workspace src/ directory:
#
#   erl/common:20.04           <- erl_common/docker/ubuntu-2004    (FROM ubuntu:20.04)
#   erl/geometry:20.04         <- erl_geometry/docker/ubuntu-2004  (FROM erl/common:20.04)
#   erl/ros-noetic:cpu-gp-sdf  <- erl_gp_sdf_ros/docker/ros-noetic (FROM erl/geometry:20.04)
#
# Toggle stages (all default true):
#   BUILD_COMMON     erl/common:20.04
#   BUILD_GEOMETRY   erl/geometry:20.04
#   BUILD_ROS_GP_SDF erl/ros-noetic:cpu-gp-sdf
#
# Optional --build-arg overrides forwarded to every stage when set:
#   APT_MIRROR  TZ  LOCALE

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# shellcheck source=docker_common.bash
source "${SCRIPT_DIR}/docker_common.bash"

BUILD_COMMON="${BUILD_COMMON:-true}"
BUILD_GEOMETRY="${BUILD_GEOMETRY:-true}"
BUILD_ROS_GP_SDF="${BUILD_ROS_GP_SDF:-true}"

USER_ARGS=("$@")

preflight

if [ "${BUILD_COMMON}" = "true" ]; then
    build_stage "${SRC_DIR}/erl_common/docker/ubuntu-2004" erl/common:20.04 \
        --build-arg BASE_IMAGE=ubuntu:20.04
fi

if [ "${BUILD_GEOMETRY}" = "true" ]; then
    build_stage "${SRC_DIR}/erl_geometry/docker/ubuntu-2004" erl/geometry:20.04 \
        --build-arg BASE_IMAGE=erl/common:20.04
fi

if [ "${BUILD_ROS_GP_SDF}" = "true" ]; then
    build_stage "${SRC_DIR}/erl_gp_sdf_ros/docker/ros-noetic" erl/ros-noetic:cpu-gp-sdf \
        -t erl/kernel_sdf:noetic \
        --build-arg BASE_IMAGE=erl/geometry:20.04
fi

print_summary
