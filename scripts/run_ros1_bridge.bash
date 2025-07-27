#!/usr/bin/bash

CONTAINER_NAME="ros1-bridge"
ROS_IP=${ROS_IP:-"127.0.0.1"}
ROS_HOSTNAME=${ROS_HOSTNAME:-"${ROS_IP}"}
ROS_MASTER_URI=${ROS_MASTER_URI:-"http://${ROS_IP}:11311"}
docker run --rm -it \
    --name ${CONTAINER_NAME} \
    --net=host \
    --hostname container-${CONTAINER_NAME} \
    --add-host=container-${CONTAINER_NAME}:127.0.0.1 \
    --env ROS_MASTER_URI=${ROS_MASTER_URI} \
    --env ROS_IP=${ROS_IP} \
    --env ROS_HOSTNAME=${ROS_HOSTNAME} \
    erl/ros1-bridge:latest
