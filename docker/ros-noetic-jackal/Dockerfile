ARG BASE_IMAGE="erl/ros-noetic:cpu-gp-sdf"
FROM ${BASE_IMAGE}

ARG APT_MIRROR=""
RUN cp /etc/apt/sources.list /etc/apt/sources.list.backup &&\
    if [ -n "$APT_MIRROR" ]; then \
    sed -i "s|http://archive.ubuntu.com/ubuntu/|$APT_MIRROR|g" /etc/apt/sources.list; \
    fi

####################################################################################################
# Clearpath Jackal
####################################################################################################
# https://clearpathrobotics.com/assets/guides/noetic/jackal/index.html
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation \
    ros-noetic-tf2-tools \
    ros-noetic-velodyne-gazebo-plugins

# sensor models are set via environment variables
# https://github.com/jackal/jackal/blob/noetic-devel/jackal_description/README.md
## e.g. 2D LiDAR
#ENV JACKAL_LASER=1
## lms1xx, tim551, ust10 or utm30
#ENV JACKAL_LASER_MODEL=lms1xx
#ENV JACKAL_LASER_TOPIC="front/scan"
## 3D LiDAR
#ENV JACKAL_LASER_3D=1
## vlp16 or hdl32e
#ENV JACKAL_LASER_3D_MODEL="vlp16"
#ENV JACKAL_LASER_3D_TOPIC="mid/points"

RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
    mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
