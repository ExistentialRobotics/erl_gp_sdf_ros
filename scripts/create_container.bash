#!/usr/bin/bash

set -e

# Prompt for ROS distribution
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
# Check if the Docker image exists
DOCKER_IMAGE="erl/ros-${ROS_DISTRO}:cpu-gp-sdf"
if ! docker images -q "${DOCKER_IMAGE}"; then
    echo "Docker image ${DOCKER_IMAGE} not found. Please build the image first."
    exit 1
fi
# Prompt for container name
read -p "Enter the name of the container (default: gp-sdf): " CONTAINER_NAME
if [ -z "$CONTAINER_NAME" ]; then
    CONTAINER_NAME="gp-sdf"
fi
if docker ps -a | grep -q "${CONTAINER_NAME}"; then
    echo "Container with name ${CONTAINER_NAME} already exists. Please choose a different name or remove the existing container."
    exit 1
fi

SCRIPT_DIR=$(cd $(dirname $0); pwd)
ROS_WS_DIR=$(cd ${SCRIPT_DIR}/../../..; pwd)

# Confirm the action
echo "Creating container with the following details:"
echo "SCRIPT_DIR: ${SCRIPT_DIR}"
echo "ROS workspace directory: ${ROS_WS_DIR} (will be mounted to /opt/ros_ws in the container)"
echo "Docker image: ${DOCKER_IMAGE}"
echo "Container name: ${CONTAINER_NAME}"
read -p "Continue to create container? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborting container creation."
    exit 1
fi

docker run --privileged --restart always -t \
    -v /var/run/docker.sock:/var/run/docker.sock:rw \
    -v /dev/char:/dev/char:rw \
    -v /dev/shm:/dev/shm:rw \
    -v /dev/bus:/dev/bus:rw \
    -v /dev/block:/dev/block:rw \
    -v /dev/serial:/dev/serial:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${HOME}/.Xauthority:/root/.Xauthority:rw \
    -v ${ROS_WS_DIR}:/opt/ros_ws:rw \
    --workdir /opt/ros_ws \
    -e DISPLAY \
    --net=host \
    --detach \
    --hostname container-${CONTAINER_NAME} \
    --add-host=container-${CONTAINER_NAME}:127.0.0.1 \
    --name ${CONTAINER_NAME} \
    --cap-add sys_ptrace --cpus=0 --memory-swap=-1 --ipc=host \
    ${DOCKER_IMAGE} bash -l

echo "Container ${CONTAINER_NAME} created successfully."
echo "Login to the container using command:"
echo "CONTAINER_NAME=${CONTAINER_NAME} . ${SCRIPT_DIR}/login_container.bash"
