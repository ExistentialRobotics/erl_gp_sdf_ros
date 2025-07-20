ARG BASE_IMAGE=erl/geometry:20.04-cu12.9
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
# RealSense
####################################################################################################
# https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
# https://dev.intelrealsense.com/docs/ros1-wrapper
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
    tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && DEBIAN_FRONTEND=noninteractive apt install -y \
    librealsense2-udev-rules \
    librealsense2 \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    librealsense2-gl \
    librealsense2-gl-dev \
    librealsense2-gl-dbg

####################################################################################################
# erl_gp_sdf dependencies
####################################################################################################
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-grid-map-ros \
    ros-noetic-grid-map-rviz-plugin

####################################################################################################
# Miscellaneous
####################################################################################################
# Install git-credential-manager for git authentication
RUN wget https://github.com/git-ecosystem/git-credential-manager/releases/download/v2.6.1/gcm-linux_amd64.2.6.1.deb && \
    DEBIAN_FRONTEND=noninteractive apt install -y ./gcm-linux_amd64.2.6.1.deb && \
    rm gcm-linux_amd64.2.6.1.deb && ln -s /usr/local/bin/git-credential-manager /usr/bin/git-credential-manager

####################################################################################################
# Clean up
####################################################################################################
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
        mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
