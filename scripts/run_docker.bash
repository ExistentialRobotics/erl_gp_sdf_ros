#!/usr/bin/env bash
#
# Create (and start, detached) an erl_gp_sdf_ros dev container from
# erl/kernel_sdf:<distro> (built by build_docker_ros1.bash / build_docker_ros2.bash).
#
#     bash scripts/run_docker.bash <noetic|humble|jazzy> [options]
#
# This is a self-contained reproduction of `erl-docker-create-container --gui` for
# these images (no erl_toolkit / docker-py dependency). It:
#   * forwards the X server (xhost + X11 socket + DISPLAY + .Xauthority),
#   * runs --privileged with the host devices, docker socket, and $HOME bind-mounted,
#     on --net=host, with sys_ptrace / ipc=host / no cpu+swap limits,
#   * copies your host user (passwd/group/shadow) into the container so files created
#     in the mounted $HOME stay owned by you and you can sudo with your host password,
#   * sets up XDG_RUNTIME_DIR for GUI apps.
#
# Options:
#   --name NAME     container name           (default: kernel_sdf_<distro>)
#   --image IMAGE   image to run             (default: erl/kernel_sdf:<distro>)
#   --user USER     user to replicate        (default: current user)
#   --command CMD   container command        (default: "bash -l")
#   --no-gui        skip X11 / GUI forwarding
#   --gpu           add --gpus all --runtime=nvidia
#   -h, --help      show this help
#
# Enter the container afterwards with:  docker exec -it <name> bash -l

set -euo pipefail

log()  { echo -e "\n\033[1;34m==> $*\033[0m"; }
warn() { echo -e "\033[1;33mWARNING: $*\033[0m" >&2; }
err()  { echo -e "\033[1;31mERROR: $*\033[0m" >&2; }
usage() { sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; s/^set -euo.*//'; }

DISTRO=""
NAME=""
IMAGE=""
USER_NAME="$(id -un)"
COMMAND="bash -l"
GUI=1
GPU=0

while [ $# -gt 0 ]; do
    case "$1" in
        noetic|humble|jazzy) DISTRO="$1"; shift ;;
        --name)    NAME="$2"; shift 2 ;;
        --image)   IMAGE="$2"; shift 2 ;;
        --user)    USER_NAME="$2"; shift 2 ;;
        --command) COMMAND="$2"; shift 2 ;;
        --no-gui)  GUI=0; shift ;;
        --gpu)     GPU=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) err "unknown argument: $1"; usage; exit 1 ;;
    esac
done

if [ -z "${DISTRO}" ]; then
    err "missing distro. Usage: bash scripts/run_docker.bash <noetic|humble|jazzy> [options]"
    exit 1
fi

: "${NAME:=kernel_sdf_${DISTRO}}"
: "${IMAGE:=erl/kernel_sdf:${DISTRO}}"

if [ -n "${ERL_IN_CONTAINER:-}" ]; then
    err "you are already inside container '${ERL_IN_CONTAINER}'; run this on the host."
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    err "cannot reach the Docker daemon (running? permissions?)."
    exit 1
fi
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    err "image '${IMAGE}' not found. Build it first, e.g.:"
    case "${DISTRO}" in
        noetic) err "    bash scripts/build_docker_ros1.bash" ;;
        *)      err "    ROS_DISTRO=${DISTRO} bash scripts/build_docker_ros2.bash" ;;
    esac
    exit 1
fi

# GUI needs a running X server on the host.
if [ "${GUI}" = 1 ] && [ ! -d /tmp/.X11-unix ]; then
    warn "/tmp/.X11-unix not found; continuing without GUI forwarding."
    GUI=0
fi

# Replace an existing container of the same name (asks first).
if docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
    warn "container '${NAME}' already exists."
    read -r -p "Remove and recreate it? [y/N] " ans
    case "${ans,,}" in
        y|yes) docker rm -f "${NAME}" >/dev/null ;;
        *) log "Aborted; kept existing container."; exit 0 ;;
    esac
fi

HOME_DIR="${HOME}"
CONFIG_DIR="${HOME}/.config/erl/docker"   # holds docker login creds, mounted read-only
mkdir -p "${CONFIG_DIR}"

# Host IP used to reach the internet (matches erl_toolkit's get_ip_address()).
HOST_IP="$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K[0-9.]+' | head -1 || true)"
[ -z "${HOST_IP}" ] && HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

RUN_ARGS=(
    -e "ERL_IN_CONTAINER=${NAME}"
    -e "ERL_DOCKER_IMAGE=${IMAGE}"
    -e "TZ=${TZ:-America/Los_Angeles}"
)
[ -n "${HOST_IP}" ] && RUN_ARGS+=(-e "DOCKER_HOST=${HOST_IP}")
RUN_ARGS+=(
    --privileged
    --restart always
    -t
    -v /var/run/docker.sock:/var/run/docker.sock:rw
    -v "${CONFIG_DIR}:/mnt/docker_login:ro"
    -v "${HOME_DIR}:/home/${USER_NAME}:rw"
)
# Bind host devices that exist (skip missing ones so we don't create stray /dev entries).
for dev in /dev/serial /dev/bus /dev/block /dev/char /dev/shm; do
    [ -e "${dev}" ] && RUN_ARGS+=(-v "${dev}:${dev}:rw")
done
RUN_ARGS+=(--workdir "/home/${USER_NAME}")
if [ "${GUI}" = 1 ]; then
    RUN_ARGS+=(
        -v /tmp/.X11-unix:/tmp/.X11-unix
        -e DISPLAY
        -v "${HOME_DIR}/.Xauthority:/root/.Xauthority:rw"
    )
fi
[ "${GPU}" = 1 ] && RUN_ARGS+=(--gpus all --runtime=nvidia)
RUN_ARGS+=(
    --net=host
    --detach
    --hostname "container-${NAME}"
    --add-host "container-${NAME}:127.0.0.1"
    --name "${NAME}"
    --cap-add sys_ptrace       # allow gdb / ptrace inside the container
    --cpus=0                   # no CPU limit
    --memory-swap=-1           # no swap limit
    --ipc=host                 # share the host IPC namespace
)

if [ "${GUI}" = 1 ]; then
    xhost +SI:localuser:root >/dev/null 2>&1 || warn "xhost failed; GUI apps may not connect to the X server."
fi

log "Creating container '${NAME}' from ${IMAGE}"
( set -x; docker run "${RUN_ARGS[@]}" "${IMAGE}" ${COMMAND} )

# Wait for it to be up.
for _ in $(seq 1 50); do
    [ "$(docker inspect -f '{{.State.Running}}' "${NAME}" 2>/dev/null)" = "true" ] && break
    sleep 0.1
done

# Replicate the host user inside the container (Linux only, non-root user).
if [ "$(uname -s)" = "Linux" ] && [ "${USER_NAME}" != "root" ]; then
    log "Copying host user '${USER_NAME}' into '${NAME}'"
    for f in passwd group shadow; do
        if sudo grep -q "^${USER_NAME}:" "/etc/${f}" 2>/dev/null; then
            sudo grep "^${USER_NAME}:" "/etc/${f}" | docker exec -i "${NAME}" bash -c "cat >> /etc/${f}"
        fi
    done
    docker exec "${NAME}" bash -c "usermod -aG sudo '${USER_NAME}'" || warn "could not add ${USER_NAME} to sudo group."
    if [ "${GUI}" = 1 ]; then
        uid="$(id -u "${USER_NAME}")"
        docker exec "${NAME}" bash -c "mkdir -p /run/user/${uid} && chown ${USER_NAME}:${USER_NAME} /run/user/${uid}"
        docker exec "${NAME}" bash -c \
            "grep -q XDG_RUNTIME_DIR /etc/profile || echo 'export XDG_RUNTIME_DIR=/run/user/\$(id -u \$USER)' >> /etc/profile"
    fi
fi

log "Container '${NAME}' is ready. Enter it with:"
echo "    docker exec -it ${NAME} bash -l"
