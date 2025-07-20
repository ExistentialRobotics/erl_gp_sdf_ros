#! /usr/bin/bash

set -e
docker build --rm -t erl/ros-noetic:cpu-gp-sdf . \
  --build-arg BASE_IMAGE=erl/geometry:20.04 $@
