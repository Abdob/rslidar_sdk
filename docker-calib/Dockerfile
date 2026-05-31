FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Build deps + Python tooling. ros-humble-usb-cam brings v4l2 capture into ROS.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libyaml-cpp-dev \
        libpcap-dev \
        libeigen3-dev \
        python3-colcon-common-extensions \
        python3-pip \
        python3-numpy \
        python3-scipy \
        python3-yaml \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-pcl-conversions \
        ros-humble-usb-cam \
        ros-humble-image-transport \
        ros-humble-cv-bridge \
        ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# OpenCV with contrib (ArUco), Open3D. ROS 2 Humble's cv_bridge is compiled
# against NumPy 1.x; opencv-contrib-python 4.10+ wheels are built against
# NumPy 2.x and would auto-upgrade numpy, breaking cv_bridge. Pin opencv to
# the last 4.8 series, which keeps numpy<2.
RUN pip3 install --no-cache-dir \
        "numpy<2" \
        "opencv-contrib-python==4.8.1.78" \
        "open3d>=0.18,<0.19" \
        transforms3d

# Colcon workspace: rslidar_msg + rslidar_sdk (live AIRY publisher).
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
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DENABLE_IMU_DATA_PARSE=ON \
            -DPOINT_TYPE=XYZIRT \
            -DCMAKE_CXX_FLAGS="-include memory -include functional"

# Calibration tools, launch files, and configs live in a bind-mountable dir so
# the user can iterate without rebuilding the image.
RUN mkdir -p /opt/calib
COPY docker-calib/config/  /opt/calib/config/
COPY docker-calib/launch/  /opt/calib/launch/
COPY docker-calib/scripts/ /opt/calib/scripts/
COPY docker-calib/rviz/    /opt/calib/rviz/
RUN chmod +x /opt/calib/scripts/*.py 2>/dev/null || true

# Entrypoint: source ROS + workspace.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/ros_ws/install/setup.bash' \
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    'export PYTHONPATH=/opt/calib/scripts:${PYTHONPATH:-}' \
    'exec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /opt/calib
CMD ["bash"]
