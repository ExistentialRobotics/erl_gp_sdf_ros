#!/usr/bin/env bash
#
# Shared helpers for building the erl_gp_sdf_ros ROS Docker images.
# This file is *sourced* by build_docker_ros1.bash / build_docker_ros2.bash;
# it is not meant to be executed directly.
#
# The image chain built by those scripts (each stage FROM the previous base):
#
#   erl/common:<ubuntu>     <- erl_common/docker/ubuntu-<ubuntu>     (FROM ubuntu:<ubuntu>)
#   erl/geometry:<ubuntu>   <- erl_geometry/docker/ubuntu-<ubuntu>   (FROM erl/common)
#   erl/ros-<distro>:cpu-*  <- erl_gp_sdf_ros/docker/ros-<distro>    (FROM erl/geometry)
#
# The base packages (erl_common, erl_geometry) are expected to sit next to
# erl_gp_sdf_ros in the same workspace src/ directory, i.e. the standard layout:
#
#   <ros_ws>/src/erl_common
#   <ros_ws>/src/erl_geometry
#   <ros_ws>/src/erl_gp_sdf_ros   <- this package
#
# Every helper honours these optional environment overrides (forwarded as
# --build-arg to every stage only when non-empty, so the Dockerfile defaults
# apply otherwise):
#
#   APT_MIRROR   e.g. http://mirror.example.com/ubuntu/   (replaces archive.ubuntu.com)
#   TZ           e.g. America/Los_Angeles
#   LOCALE       e.g. en_US.UTF-8

set -euo pipefail

# SRC_DIR is the directory that holds all the erl_* packages: the parent of this
# package's root. This file lives in <src>/erl_gp_sdf_ros/scripts, so two levels
# up is <src>.
SRC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." &> /dev/null && pwd )"

APT_MIRROR="${APT_MIRROR:-}"
TZ="${TZ:-}"
LOCALE="${LOCALE:-}"

log()  { echo -e "\n\033[1;34m==> $*\033[0m"; }
warn() { echo -e "\033[1;33mWARNING: $*\033[0m" >&2; }
err()  { echo -e "\033[1;31mERROR: $*\033[0m" >&2; }

# --build-arg pairs shared by every stage (only the ones the caller set).
COMMON_BUILD_ARGS=()
if [ -n "${APT_MIRROR}" ]; then COMMON_BUILD_ARGS+=(--build-arg "APT_MIRROR=${APT_MIRROR}"); fi
if [ -n "${TZ}" ];         then COMMON_BUILD_ARGS+=(--build-arg "TZ=${TZ}"); fi
if [ -n "${LOCALE}" ];     then COMMON_BUILD_ARGS+=(--build-arg "LOCALE=${LOCALE}"); fi

# Extra flags the user passed on the command line (e.g. --no-cache, --pull, --progress=plain).
USER_ARGS=()

# Names of the images successfully built, for the closing summary.
BUILT_IMAGES=()

# Abort early with a helpful message if the Docker CLI / daemon is unusable.
preflight() {
    if ! command -v docker >/dev/null 2>&1; then
        err "docker CLI not found on PATH. Install Docker and retry."
        exit 1
    fi
    if ! docker info >/dev/null 2>&1; then
        err "cannot talk to the Docker daemon (is it running? do you have permission?)."
        exit 1
    fi
}

# The ros2-apt-source package version used by the ROS 2 Dockerfiles. The upstream
# build.bash scripts pin this to the latest GitHub release; fall back to the
# Dockerfile default (1.1.0) when offline.
detect_ros_apt_source_version() {
    local v=""
    v="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null \
         | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    echo "${v:-1.1.0}"
}

# build_stage <context_dir> <image:tag> [stage-specific docker args...]
# Runs `docker build`, appending COMMON_BUILD_ARGS then USER_ARGS.
build_stage() {
    local ctx="$1" image="$2"
    shift 2
    if [ ! -d "${ctx}" ]; then
        err "build context not found: ${ctx}"
        err "expected the erl_* packages side by side in the workspace src/ directory."
        err "did you clone them? (e.g. git submodule update --init --recursive)"
        exit 1
    fi
    log "Building ${image}   (context: ${ctx#"${SRC_DIR}"/})"
    (
        set -x
        docker build --rm -t "${image}" "${ctx}" \
            "$@" "${COMMON_BUILD_ARGS[@]}" "${USER_ARGS[@]}"
    )
    BUILT_IMAGES+=("${image}")
    # record any additional -t/--tag aliases (e.g. erl/kernel_sdf:<distro>) for the summary
    local _args=("$@") _i
    for ((_i = 0; _i < ${#_args[@]}; _i++)); do
        case "${_args[_i]}" in
            -t | --tag) BUILT_IMAGES+=("${_args[_i + 1]}") ;;
        esac
    done
}

print_summary() {
    log "Done. Built ${#BUILT_IMAGES[@]} image(s):"
    local img
    for img in "${BUILT_IMAGES[@]}"; do
        echo "    ${img}"
    done
}
