FROM stereolabs/zed:5.0-gl-devel-cuda12.8-ubuntu24.04

ARG TZ=America/Los_Angeles
ARG LOCALE=en_US.UTF-8
ARG APT_MIRROR=""
ARG ROS_APT_SOURCE_VERSION=1.1.0

ENV TZ=${TZ}
ENV LANG=${LOCALE}
ENV LANGUAGE=${LOCALE}
ENV LC_ALL=${LOCALE}

# make sure it is 24.04
RUN . /etc/os-release && if [ "$VERSION_ID" != "24.04" ]; then echo "This Dockerfile is for Ubuntu 24.04!" && exit 1; fi

###########################
# erl_common dependencies #
###########################
RUN cp /etc/apt/sources.list /etc/apt/sources.list.backup &&\
    if [ -n "$APT_MIRROR" ]; then \
    sed -i "s|http://archive.ubuntu.com/ubuntu/|$APT_MIRROR|g" /etc/apt/sources.list; \
    fi
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt update && apt install -y locales && locale-gen en_US.UTF-8
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    sudo \
    apt-utils \
    software-properties-common \
    lsb-release \
    ca-certificates \
    wget \
    curl \
    python3 \
    python3-pip \
    python3-dev \
    python3-vedo \
    python3-tqdm \
    zsh \
    vim \
    git \
    build-essential \
    cmake \
    ccache \
    intel-mkl-full \
    liblapack-dev \
    liblapacke-dev \
    libomp-dev \
    libeigen3-dev \
    libfmt-dev \
    libboost-all-dev \
    nlohmann-json3-dev \
    libopencv-dev \
    libplplot-dev \
    libplplot-java \
    libplplot-data \
    libplplotada4.2 \
    libplplotada-dev \
    plplot-tcl-dev \
    plplot-driver-cairo \
    python3-plplot \
    python3-plplot-qt \
    octave-plplot \
    libcairomm-1.0-dev \
    libyaml-cpp-dev \
    pybind11-dev \
    libgtest-dev \
    libabsl-dev

RUN PYTHON_VERSION=$(python3 -c "import sys;print(sys.version_info.major,sys.version_info.minor,sep='')") && \
    ln -s /usr/lib/python3/dist-packages/_Pltk_init.cpython-${PYTHON_VERSION}-x86_64-linux-gnu.so /usr/lib/python3/dist-packages/_Pltk_init.so && \
    ln -s /usr/lib/python3/dist-packages/_plplotc.cpython-${PYTHON_VERSION}-x86_64-linux-gnu.so /usr/lib/python3/dist-packages/_plplotc.so && \
    ln -s /usr/lib/python3/dist-packages/plplot_pyqt5.cpython-${PYTHON_VERSION}-x86_64-linux-gnu.so /usr/lib/python3/dist-packages/plplot_pyqt5.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libplplotada.a /usr/lib/x86_64-linux-gnu/libplplotada_static.a
RUN touch /usr/bin/pltek

ENV GOMP_SPINCOUNT=1

#############################
# erl_geometry dependencies #
#############################
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y \
    libqhull-dev \
    libabsl-dev \
    libjsoncpp-dev

# libnanoflann-dev and libopen3d-dev are too old in Ubuntu 22.04

WORKDIR /root

# install nanoflann
RUN git clone https://github.com/jlblancoc/nanoflann && \
    cd nanoflann && git checkout v1.7.1 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make install && \
    cd /root && rm -rf nanoflann

# Ubuntu has cmake>=3.28, Open3D 0.19.0 requires at least CMake 3.24
# install Open3D
RUN git clone --recursive https://github.com/isl-org/Open3D.git && cd Open3D && \
    git checkout 1e7b174 && echo y | util/install_deps_ubuntu.sh && mkdir -p build && \
    cd build && \
    cmake .. \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_SYSTEM_EIGEN3=ON \
    -DUSE_SYSTEM_FMT=ON \
    -DUSE_SYSTEM_GOOGLETEST=ON \
    -DUSE_SYSTEM_JSONCPP=ON \
    -DUSE_SYSTEM_NANOFLANN=ON \
    -DUSE_SYSTEM_PYBIND11=ON \
    -DUSE_SYSTEM_QHULLCPP=ON \
    -DGLIBCXX_USE_CXX11_ABI=ON \
    -DBUILD_PYTHON_MODULE=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_BENCHMARKS=OFF && \
    make -j`nproc` && make install && \
    cd /root && rm -rf Open3D

#############################################
# ROS Jazzy and erl_gp_sdf_ros dependencies #
#############################################
RUN add-apt-repository universe
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb

RUN DEBIAN_FRONTEND=noninteractive apt update -y && apt upgrade -y && apt install -y \
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
    ros-jazzy-tf2-tools
RUN rosdep init && rosdep update

#################
# Miscellaneous #
#################
# Install git-credential-manager for git authentication
RUN wget https://github.com/git-ecosystem/git-credential-manager/releases/download/v2.6.1/gcm-linux_amd64.2.6.1.deb && \
    DEBIAN_FRONTEND=noninteractive apt install -y ./gcm-linux_amd64.2.6.1.deb && \
    rm gcm-linux_amd64.2.6.1.deb && ln -s /usr/local/bin/git-credential-manager /usr/bin/git-credential-manager

##############
# ZED camera #
##############
# install zed-ros2-wrapper
RUN chown -R root:root /usr/local/lib/python3.12 && chmod -R 755 /usr/local/lib/python3.12
RUN mkdir -p /opt/zed_camera_ws/src && \
    cd /opt/zed_camera_ws/src && \
    git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    cd .. && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    VERBOSE=1 colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY common_stereo.yaml /opt/zed_camera_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml

RUN userdel -r ubuntu

# pyqtplot needs the pyqtgraph.
# the base image provides both numpy1.x and 2.x but numpy2.x is not compatible.
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    python3-pyqtgraph && \
    sudo pip uninstall numpy --break-system-packages -y

#######################
# More ROS 2 packages #
#######################
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-jazzy-pointcloud-to-laserscan

############
# Clean up #
############
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
    mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
