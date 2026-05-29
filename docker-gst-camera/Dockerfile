# GPU-accelerated MJPEG camera publisher for the AIRY+camera rig.
#
# Base is DeepStream 9.0 (matches docker-ds3d's base): brings nvjpeg, the
# NVIDIA GStreamer plugins (nvv4l2decoder, nvvideoconvert) and CUDA runtime.
# We add ROS 2 Humble on top, then build gscam2 from source — gscam2 takes
# an arbitrary GStreamer pipeline string as a parameter and publishes the
# decoded frames as sensor_msgs/Image + image_transport, which is exactly
# what we want.
FROM nvcr.io/nvidia/deepstream:9.0-triton-multiarch

ENV DEBIAN_FRONTEND=noninteractive

# --- ROS 2 Jazzy ---
# DeepStream 9.0 ships on Ubuntu 24.04 (Noble), so the matching ROS 2 LTS is
# Jazzy, not Humble. ROS 2 distros are wire-compatible over DDS — Jazzy in
# this container will exchange sensor_msgs/Image with Humble in docker-calib
# and docker-ros2 without any bridging.
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
        software-properties-common \
        locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        ros-jazzy-ros-base \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-camera-info-manager \
        ros-jazzy-image-transport \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-cv-bridge \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# --- gscam2 from source ---
# No apt package for Humble; build the upstream ROS 2 fork.
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws/src
RUN git clone --depth=1 https://github.com/clydemcqueen/gscam2.git

WORKDIR /opt/ros_ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select gscam2 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- launch + config (bind-mounted at runtime so edits don't need a rebuild) ---
RUN mkdir -p /opt/camera
COPY docker-gst-camera/config/  /opt/camera/config/
COPY docker-gst-camera/launch/  /opt/camera/launch/

# Entry: source ROS + workspace, set DDS, set NVIDIA env vars so DeepStream
# plugins find their libs. The privileged container under --net=host also
# inherits the host's /dev/video* nodes and the NVIDIA driver.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    '# 128 MB UDP buffer; raw 1280x720 RGB at 30 fps = 80 MB/s on loopback' \
    '# and Jazzy<->Humble large-message delivery overruns the default 200 KB.' \
    'sysctl -wq net.core.rmem_max=134217728 net.core.rmem_default=134217728 || true' \
    'sysctl -wq net.core.wmem_max=134217728 net.core.wmem_default=134217728 || true' \
    'source /opt/ros/jazzy/setup.bash' \
    'source /opt/ros_ws/install/setup.bash' \
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    '[ -f /opt/cyclonedds.xml ] && export CYCLONEDDS_URI=file:///opt/cyclonedds.xml' \
    'export LD_LIBRARY_PATH=/opt/nvidia/deepstream/deepstream/lib:${LD_LIBRARY_PATH:-}' \
    'exec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /opt/camera
CMD ["ros2", "launch", "/opt/camera/launch/camera.launch.py"]
