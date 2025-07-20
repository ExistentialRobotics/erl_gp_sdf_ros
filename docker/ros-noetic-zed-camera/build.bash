#! /usr/bin/bash

set -e
docker build --rm -t erl/ros-noetic:zed-gp-sdf . $@
