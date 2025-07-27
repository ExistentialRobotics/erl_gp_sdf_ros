#!/usr/bin/bash

CONTAINER_NAME=${CONTAINER_NAME:-"gp-sdf"}

xhost +si:localuser:root
docker exec --privileged -it \
    --env DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env TERM=xterm-256color \
    --user root \
    ${CONTAINER_NAME} /usr/bin/bash -l
