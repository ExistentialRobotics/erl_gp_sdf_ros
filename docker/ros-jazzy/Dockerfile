ARG BASE_IMAGE=erl/geometry:24.04
FROM ${BASE_IMAGE}

ARG TZ=America/Los_Angeles
ARG LOCALE=en_US.UTF-8
ARG APT_MIRROR=""
ARG ROS_APT_SOURCE_VERSION=1.1.0

ENV TZ=${TZ}
ENV LANG=${LOCALE}
ENV LANGUAGE=${LOCALE}
ENV LC_ALL=${LOCALE}

RUN cp /etc/apt/sources.list /etc/apt/sources.list.backup &&\
    if [ -n "$APT_MIRROR" ]; then \
    sed -i "s|http://archive.ubuntu.com/ubuntu/|$APT_MIRROR|g" /etc/apt/sources.list; \
    fi
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt update && apt install -y locales && locale-gen en_US.UTF-8

# GUI and visualization (lets rviz2 / GUIs render from inside the container)
RUN DEBIAN_FRONTEND=noninteractive apt update -y && apt install -y \
    software-properties-common \
    curl \
    mesa-utils \
    libgl1-mesa-dri \
    x11-apps

RUN add-apt-repository universe
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb

RUN DEBIAN_FRONTEND=noninteractive apt update -y && apt upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-jazzy-desktop-full \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-jazzy-grid-map-msgs \
    ros-jazzy-grid-map-ros \
    ros-jazzy-grid-map-rviz-plugin \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-kdl \
    ros-jazzy-tf2-msgs \
    ros-jazzy-tf2-py \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-tf2-tools \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-rqt-plot
RUN rosdep init && rosdep update

# debugging tools
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    gdb \
    gdbserver

# clean
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
    mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
