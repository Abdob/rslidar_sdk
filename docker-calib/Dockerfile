FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# --- Stage 1: rslidar_sdk build environment (MATCHES docker-ros2 exactly) ---
# Calibration-only packages and pip wheels go in stage 2. Keeping the rslidar
# build environment minimal here is intentional — when this set drifted from
# docker-ros2 (extra ros-humble-* packages pulling in different transitive
# libs), the resulting rslidar_sdk binary published clouds at 7–8 Hz instead
# of 10 Hz on the AIRY. Any change to this apt list should be mirrored in
# docker-ros2/Dockerfile or moved to stage 2 below.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libyaml-cpp-dev \
        libpcap-dev \
        libeigen3-dev \
        python3-colcon-common-extensions \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# Colcon workspace: rslidar_msg + rslidar_sdk.
WORKDIR /opt/ros_ws/src
RUN git clone --depth=1 https://github.com/RoboSense-LiDAR/rslidar_msg.git

COPY src/        /opt/ros_ws/src/rslidar_sdk/src/
COPY node/       /opt/ros_ws/src/rslidar_sdk/node/
COPY launch/     /opt/ros_ws/src/rslidar_sdk/launch/
COPY rviz/       /opt/ros_ws/src/rslidar_sdk/rviz/
COPY CMakeLists.txt package.xml /opt/ros_ws/src/rslidar_sdk/
COPY config/     /opt/ros_ws/src/rslidar_sdk/config/
COPY docker-calib/config/lidar_config.yaml /opt/ros_ws/src/rslidar_sdk/config/config.yaml

WORKDIR /opt/ros_ws
# Switch the rslidar_sdk point type from XYZI to XYZIRT so each point has
# `time` (relative timestamp) + `ring` fields. FAST-LIVO2's robosense_handler
# requires them for per-point motion compensation. CMakeLists hard-codes the
# default via an unconditional set(), so we patch the line before building.
RUN sed -i 's|^set(POINT_TYPE XYZI)$|set(POINT_TYPE XYZIRT)|' \
    /opt/ros_ws/src/rslidar_sdk/CMakeLists.txt
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DENABLE_IMU_DATA_PARSE=ON \
            -DCMAKE_CXX_FLAGS="-include memory -include functional"

# --- Stage 2: calibration tooling (after binary is built) ---
# Adding these earlier (especially ros-humble-cv-bridge / image-transport)
# pulled different transitive deps into the rslidar_sdk link step and slowed
# the published cloud rate. These are runtime-only for the calibration
# Python scripts; they don't influence the already-built C++ binary.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
        python3-scipy \
        python3-yaml \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-cv-bridge \
        ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*
# Note: no camera capture driver here. Camera publishing lives in the dedicated
# docker-gst-camera container (DeepStream + gscam2 GPU pipeline). docker-calib
# consumes /image_raw over the host DDS network. image_transport-plugins is
# kept so /image_raw/compressed decoders are available locally.

# OpenCV with contrib (ArUco), Open3D. ROS 2 Humble's cv_bridge is compiled
# against NumPy 1.x; opencv-contrib-python 4.10+ wheels are built against
# NumPy 2.x and would auto-upgrade numpy, breaking cv_bridge. Pin opencv to
# the last 4.8 series, which keeps numpy<2.
RUN pip3 install --no-cache-dir \
        "numpy<2" \
        "opencv-contrib-python==4.8.1.78" \
        "open3d>=0.18,<0.19" \
        transforms3d

# Calibration tools, launch files, and configs live in a bind-mountable dir so
# the user can iterate without rebuilding the image.
RUN mkdir -p /opt/calib
COPY docker-calib/config/  /opt/calib/config/
COPY docker-calib/launch/  /opt/calib/launch/
COPY docker-calib/scripts/ /opt/calib/scripts/
COPY docker-calib/rviz/    /opt/calib/rviz/
RUN chmod +x /opt/calib/scripts/*.py 2>/dev/null || true

# Entrypoint: source ROS + workspace, raise kernel UDP buffer (the AIRY at 10 Hz
# overflows the default ~200 KB cap and drops 20–40% of frames). Container is
# --privileged + --net=host, so sysctl here affects the host's network namespace,
# which is exactly what the driver's recv socket needs.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    '# 128 MB UDP recv buffer for the AIRY at 10 Hz AND raw image messages' \
    '# from docker-gst-camera at 30 Hz (2.7 MB each). 32 MB was enough for' \
    '# LiDAR alone; adding the camera stream needs more.' \
    'sysctl -wq net.core.rmem_max=134217728 net.core.rmem_default=134217728 || true' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/ros_ws/install/setup.bash' \
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    '[ -f /opt/cyclonedds.xml ] && export CYCLONEDDS_URI=file:///opt/cyclonedds.xml' \
    'export PYTHONPATH=/opt/calib/scripts:${PYTHONPATH:-}' \
    'exec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /opt/calib
CMD ["bash"]
