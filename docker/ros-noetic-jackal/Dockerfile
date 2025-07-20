ARG BASE_IMAGE="ubuntu:20.04"
FROM ${BASE_IMAGE}

ARG APT_MIRROR=""
RUN cp /etc/apt/sources.list /etc/apt/sources.list.backup &&\
    if [ -n "$APT_MIRROR" ]; then \
        sed -i "s|http://archive.ubuntu.com/ubuntu/|$APT_MIRROR|g" /etc/apt/sources.list; \
    fi

####################################################################################################
# ROS Noetic
####################################################################################################
RUN DEBIAN_FRONTEND=noninteractive apt update -y
RUN DEBIAN_FRONTEND=noninteractive apt install -y ubuntu-desktop-minimal \
    software-properties-common lsb-release vim locales apt-utils ca-certificates curl iproute2
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | DEBIAN_FRONTEND=noninteractive apt-key add -
RUN DEBIAN_FRONTEND=noninteractive apt update -y
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-pip \
    python3-numpy \
    python3-toml \
    python3-scipy \
    python3-tqdm \
    zstd \
    libpng-dev \
    libgomp1
RUN rosdep init && rosdep fix-permissions && rosdep update

####################################################################################################
# Clearpath Jackal
####################################################################################################
# https://clearpathrobotics.com/assets/guides/noetic/jackal/index.html
RUN apt-get install -y \
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

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y zsh

RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
        mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
