#! /usr/bin/bash

set -e
SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_IMAGE=${BASE_IMAGE:-erl/geometry:12.9.0-cudnn-devel-ubuntu24.04}

docker build --rm -t erl/ros-noetic:gpu-gp-sdf . \
  --build-arg BASE_IMAGE=${BASE_IMAGE} $@
