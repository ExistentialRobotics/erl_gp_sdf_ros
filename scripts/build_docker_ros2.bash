#!/usr/bin/env bash
#
# Build the ROS 2 Docker image for erl_gp_sdf_ros, using the Dockerfiles that
# ship with each package.
#
#     ROS_DISTRO=humble bash scripts/build_docker_ros2.bash [extra docker build flags...]
#     ROS_DISTRO=jazzy  bash scripts/build_docker_ros2.bash [extra docker build flags...]
#
# ROS_DISTRO selects the distribution (default: jazzy). Extra flags are
# forwarded verbatim to every `docker build`, e.g. --no-cache --pull.
#
# The erl_gp_sdf_ros image is built FROM erl/geometry, which is built FROM
# erl/common, so those two base images are built first (skip them if you
# already have them). The erl_common / erl_geometry packages are expected next
# to erl_gp_sdf_ros in the same workspace src/ directory. Humble uses Ubuntu
# 22.04, Jazzy uses Ubuntu 24.04:
#
#   erl/common:<ver>            <- erl_common/docker/ubuntu-<ver>       (FROM ubuntu:<ver>)
#   erl/geometry:<ver>          <- erl_geometry/docker/ubuntu-<ver>     (FROM erl/common:<ver>)
#   erl/ros-<distro>:cpu-gp-sdf <- erl_gp_sdf_ros/docker/ros-<distro>   (FROM erl/geometry:<ver>)
#
# Toggle stages (all default true):
#   BUILD_COMMON     erl/common:<ver>
#   BUILD_GEOMETRY   erl/geometry:<ver>
#   BUILD_ROS_GP_SDF erl/ros-<distro>:cpu-gp-sdf
#
# Optional --build-arg overrides forwarded to every stage when set:
#   APT_MIRROR  TZ  LOCALE

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# shellcheck source=docker_common.bash
source "${SCRIPT_DIR}/docker_common.bash"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
case "${ROS_DISTRO}" in
    humble) UBUNTU_DIR=ubuntu-2204; UBUNTU_TAG=22.04 ;;
    jazzy)  UBUNTU_DIR=ubuntu-2404; UBUNTU_TAG=24.04 ;;
    *) err "unsupported ROS_DISTRO '${ROS_DISTRO}' (supported: humble, jazzy)"; exit 1 ;;
esac

BUILD_COMMON="${BUILD_COMMON:-true}"
BUILD_GEOMETRY="${BUILD_GEOMETRY:-true}"
BUILD_ROS_GP_SDF="${BUILD_ROS_GP_SDF:-true}"

USER_ARGS=("$@")

preflight
log "Target ROS 2 distribution: ${ROS_DISTRO} (Ubuntu ${UBUNTU_TAG})"

COMMON_IMAGE="erl/common:${UBUNTU_TAG}"
GEOMETRY_IMAGE="erl/geometry:${UBUNTU_TAG}"

if [ "${BUILD_COMMON}" = "true" ]; then
    build_stage "${SRC_DIR}/erl_common/docker/${UBUNTU_DIR}" "${COMMON_IMAGE}" \
        --build-arg "BASE_IMAGE=ubuntu:${UBUNTU_TAG}"
fi

if [ "${BUILD_GEOMETRY}" = "true" ]; then
    build_stage "${SRC_DIR}/erl_geometry/docker/${UBUNTU_DIR}" "${GEOMETRY_IMAGE}" \
        --build-arg "BASE_IMAGE=${COMMON_IMAGE}"
fi

if [ "${BUILD_ROS_GP_SDF}" = "true" ]; then
    # The ROS 2 Dockerfile needs the ros2-apt-source package version.
    ROS_APT_SOURCE_VERSION="$(detect_ros_apt_source_version)"
    log "Using ros2-apt-source version: ${ROS_APT_SOURCE_VERSION}"
    build_stage "${SRC_DIR}/erl_gp_sdf_ros/docker/ros-${ROS_DISTRO}" "erl/ros-${ROS_DISTRO}:cpu-gp-sdf" \
        -t "erl/kernel_sdf:${ROS_DISTRO}" \
        --build-arg "BASE_IMAGE=${GEOMETRY_IMAGE}" \
        --build-arg "ROS_APT_SOURCE_VERSION=${ROS_APT_SOURCE_VERSION}"
fi

print_summary
