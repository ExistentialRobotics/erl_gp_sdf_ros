erl_gp_sdf_ros
=================

This package provides a ROS interface of
[erl_gp_sdf](https://github.com/ExistentialRobotics/erl_gp_sdf.git), a library for Gaussian process
regression on signed distance fields.

# Setup

```bash
# create the workspace
curl -sSL https://raw.githubusercontent.com/ExistentialRobotics/erl_gp_sdf_ros/master/scripts/create_workspace.sh | bash
# build docker image for ROS
cd ros_ws_gp_sdf/src/erl_gp_sdf_ros/scripts
ROS_DISTRO=<ros_distro> ./build_docker.bash
```
