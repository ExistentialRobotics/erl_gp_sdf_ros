#! /usr/bin/bash

set -e
docker build --rm -t erl/ros-noetic:jackal . \
  --build-arg BASE_IMAGE=ubuntu:20.04 $@
