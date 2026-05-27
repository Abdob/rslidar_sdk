FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Build deps for rslidar_sdk + rs_driver
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

# Colcon workspace under /opt/ros_ws
WORKDIR /opt/ros_ws/src

# rslidar_msg (ROS 2 message types used by rslidar_sdk)
RUN git clone --depth=1 https://github.com/RoboSense-LiDAR/rslidar_msg.git

# rslidar_sdk source from our build context (project root). Drop heavy/unused dirs.
COPY src/        /opt/ros_ws/src/rslidar_sdk/src/
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
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="-include memory -include functional"

# Entry: source ROS + workspace, then exec launch (or whatever the caller passed)
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
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
