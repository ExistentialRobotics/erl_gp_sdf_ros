FROM stereolabs/zed:4.2-gl-devel-cuda11.4-ubuntu20.04

ARG TZ=America/Los_Angeles
ARG LOCALE=en_US.UTF-8
ARG APT_MIRROR=""

ENV TZ=${TZ}
ENV LANG=${LOCALE}
ENV LANGUAGE=${LOCALE}
ENV LC_ALL=${LOCALE}

####################################################################################################
# erl_common dependencies
####################################################################################################
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
    zsh \
    git \
    build-essential \
    cmake \
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
    libplplotada4 \
    plplot-tcl-dev \
    plplot-driver-cairo \
    python3-plplot \
    octave-plplot \
    libcairomm-1.0-dev \
    libyaml-cpp-dev \
    pybind11-dev \
    libgtest-dev

RUN touch /usr/lib/python3/dist-packages/_Pltk_init.so
RUN touch /usr/lib/python3/dist-packages/_plplotc.so
RUN touch /usr/lib/python3/dist-packages/plplot_pyqt5.so
RUN touch /usr/bin/pltek
RUN sed 's/..\/..\/..\/..\/..\/include/..\/..\/..\/..\/include/g' \
    /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp/yaml-cpp-config.cmake

# It is 300000 by default, which slows down OpenMP programs.
ENV GOMP_SPINCOUNT=1

####################################################################################################
# erl_geometry dependencies
####################################################################################################
WORKDIR /root
# nanoflann
RUN git clone https://github.com/jlblancoc/nanoflann && \
    cd nanoflann && git checkout v1.7.1 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make install && \
    cd /root && rm -rf nanoflann
# install abseil-cpp
RUN git clone --recursive https://github.com/abseil/abseil-cpp.git && cd abseil-cpp && \
    git checkout 20240722.1 && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="${CXXFLAGS} -DNDEBUG" \
       -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON -DABSL_PROPAGATE_CXX_STD=ON && \
    make -j`nproc` && make install && cd /root && rm -rf abseil-cpp
# install QHull
RUN git clone --recursive https://github.com/qhull/qhull.git && cd qhull && \
    git checkout 2020.2 && mkdir -p my_build && cd my_build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fPIC -ffat-lto-objects" \
      -DCMAKE_C_FLAGS="-fPIC -ffat-lto-objects" -DCMAKE_SKIP_RPATH=ON && \
    make -j`nproc` && make install && cd /root && rm -rf qhull
# Open3D 0.19.0 requires at least CMake 3.24
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0-linux-x86_64.sh &&\
    chmod +x cmake-3.24.0-linux-x86_64.sh && \
    ./cmake-3.24.0-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.24.0-linux-x86_64.sh
# install Open3D
RUN git clone --recursive https://github.com/isl-org/Open3D.git && cd Open3D && \
    git checkout 1e7b174 && echo y | util/install_deps_ubuntu.sh && mkdir -p build && \
    cd build && \
    PATH="/usr/local/bin:${PATH}" cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release \
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
# ZED camera
####################################################################################################
# install zed-ros-wrapper
# https://www.stereolabs.com/developers/release/4.2
# https://www.stereolabs.com/docs/installation/docker
# run the docker with `--gpu all --privileged`.
# The --gpus all command adds all available GPUs to the container, and --privileged grants
# permission to the container to access the camera connected with USB.
# https://github.com/stereolabs/zed-ros-wrapper/blob/master/README.md
# https://www.stereolabs.com/docs/ros/zed-node
RUN chown -R root:root /usr/local/lib/python3.8 && \
    chmod -R 755 /usr/local/lib/python3.8
RUN mkdir -p /opt/zed_camera_ws/src && \
    cd /opt/zed_camera_ws/src && \
    git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git && \
    git clone https://github.com/stereolabs/zed-ros-examples.git && \
    cd .. && \
    . /opt/ros/noetic/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic && \
    catkin_make -DCMAKE_BUILD_TYPE=Release
COPY common.yaml /opt/zed_camera_ws/src/zed-ros-wrapper/zed_wrapper/params/common.yaml
ENV DISABLE_ROS1_EOL_WARNINGS=1

####################################################################################################
# Clean up
####################################################################################################
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
        mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
