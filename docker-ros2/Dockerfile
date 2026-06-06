# x86_64 (amd64) desktop ROS 2 Humble base. osrf/ros:humble-desktop bundles
# rviz2, which humble_start.py launches.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Build deps for rslidar_sdk + rs_driver, plus RViz2 (humble_start.py launches
# an rviz2 node; the ros-base image doesn't ship it).
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
        ros-humble-rviz2 \
        ros-humble-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# Colcon workspace under /opt/ros_ws
WORKDIR /opt/ros_ws/src

# rslidar_msg (ROS 2 message types used by rslidar_sdk)
RUN git clone --depth=1 https://github.com/RoboSense-LiDAR/rslidar_msg.git

# rslidar_sdk source from our build context (project root). Drop heavy/unused dirs.
COPY src/        /opt/ros_ws/src/rslidar_sdk/src/

# src/rs_driver is a git submodule and is NOT checked out in the build context
# (the COPY above brings in an empty dir). Populate it from upstream at the
# commit this repo pins, so the build is self-contained and arch-independent.
RUN rm -rf /opt/ros_ws/src/rslidar_sdk/src/rs_driver && \
    git clone https://github.com/RoboSense-LiDAR/rs_driver.git \
        /opt/ros_ws/src/rslidar_sdk/src/rs_driver && \
    git -C /opt/ros_ws/src/rslidar_sdk/src/rs_driver \
        checkout --quiet 4eeadace465023db65a63fc8ef680dfaaca5812c
COPY node/       /opt/ros_ws/src/rslidar_sdk/node/
COPY launch/     /opt/ros_ws/src/rslidar_sdk/launch/
COPY rviz/       /opt/ros_ws/src/rslidar_sdk/rviz/
COPY CMakeLists.txt package.xml /opt/ros_ws/src/rslidar_sdk/
# Default config — we OVERWRITE with our RSAIRY-tuned one below.
COPY config/     /opt/ros_ws/src/rslidar_sdk/config/
COPY docker-ros2/config/config.yaml /opt/ros_ws/src/rslidar_sdk/config/config.yaml

# Build the workspace.
# rs_driver's vendored input.hpp uses std::shared_ptr without #include <memory>
# under newer libstdc++; force-include both standard headers.
WORKDIR /opt/ros_ws
# Match docker-calib: XYZIRT point type so FAST-LIVO2's robosense_handler gets
# per-point time + ring fields.
RUN sed -i 's|^set(POINT_TYPE XYZI)$|set(POINT_TYPE XYZIRT)|' \
    /opt/ros_ws/src/rslidar_sdk/CMakeLists.txt
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DENABLE_IMU_DATA_PARSE=ON \
            -DCMAKE_CXX_FLAGS="-include memory -include functional"

# Entry: source ROS + workspace, then exec launch (or whatever the caller passed).
# Raise kernel UDP buffer so the AIRY at 10 Hz doesn't overflow the default
# ~200 KB cap. Container is --net=host so this affects the host namespace.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'sysctl -wq net.core.rmem_max=33554432 net.core.rmem_default=33554432 || true' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/ros_ws/install/setup.bash' \
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    'exec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default: launch rslidar_sdk node + RViz2 with the preset config.
# humble_start.py also tries to apt-install cyclonedds; we already have it, so
# its check short-circuits.
CMD ["ros2", "launch", "rslidar_sdk", "humble_start.py"]
