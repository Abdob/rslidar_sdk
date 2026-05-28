FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# System deps. FAST-LIVO2 wants PCL >= 1.8, Eigen >= 3.3.4, OpenCV >= 4.2.
# Noetic-desktop-full ships PCL 1.10 + OpenCV 4.2; we just add the few extras.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        libeigen3-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libsuitesparse-dev \
        ros-noetic-pcl-ros \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-tf \
        ros-noetic-tf-conversions \
        python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Sophus — FAST-LIVO2 specifically wants the non-templated commit a621ff.
# That commit predates C++11's std::complex::real() returning by value, so
# `unit_complex_.real() = 1.;` doesn't compile on modern GCC. Patch to the
# setter form. Same for imag.
WORKDIR /opt
RUN git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout a621ff && \
    sed -i 's|unit_complex_\.real() = 1\.;|unit_complex_.real(1.);|' sophus/so2.cpp && \
    sed -i 's|unit_complex_\.imag() = 0\.;|unit_complex_.imag(0.);|' sophus/so2.cpp && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
    make -j"$(nproc)" && make install && ldconfig

# Catkin workspace: vikit (camera models + math helpers), livox_ros_driver
# (provides the CustomMsg type used by the published Avia example bag), and
# FAST-LIVO2 itself.
RUN mkdir -p /opt/catkin_ws/src
WORKDIR /opt/catkin_ws/src

# Pin to commits known to build cleanly together. Update the SHAs if upstream
# breaks; running `git log -1` inside each clone tells you what's current.
RUN git clone https://github.com/xuankuzcr/rpg_vikit.git && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git && \
    git clone https://github.com/hku-mars/FAST-LIVO2.git

WORKDIR /opt/catkin_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make -j"$(nproc)" -DCMAKE_BUILD_TYPE=Release

# Entrypoint sources ROS + the workspace so any command in the container
# already has fastlivo_mapping / roslaunch / rosbag on its PATH.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'source /opt/ros/noetic/setup.bash' \
    'source /opt/catkin_ws/devel/setup.bash' \
    'exec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /opt/catkin_ws
CMD ["bash"]
