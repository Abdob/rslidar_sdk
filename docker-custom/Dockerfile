# Sensor timing-sync experiment sandbox.
#
# Built ON TOP of the calibration image (rslidar-airy-calib) so we inherit ROS 2
# Humble, the rslidar_sdk workspace, and the Python stack (numpy<2, opencv 4.8,
# open3d, scipy, transforms3d) without rebuilding any of it. Run
#   ./docker-calib/docker_build.sh
# first -- this FROM will fail if that image doesn't exist locally.
FROM rslidar-airy-calib

ENV DEBIAN_FRONTEND=noninteractive

# --- CUDA 13.0 toolkit (nvcc) ---
# The base is Ubuntu 22.04 x86_64, so we use the ubuntu2204/x86_64 repo. This
# installs the toolkit (nvcc, cudart, headers) only; the DRIVER (libcuda) is
# injected at runtime by `--gpus all`, so the toolkit version (13.0) just needs
# to be <= the host driver's CUDA. CUDA 13.x targets Turing (sm_75) and newer.
RUN curl -fsSL -o /tmp/cuda-keyring.deb \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i /tmp/cuda-keyring.deb \
    && rm /tmp/cuda-keyring.deb \
    && apt-get update && apt-get install -y --no-install-recommends \
        cuda-toolkit-13-0 \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_HOME=/usr/local/cuda-13.0
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# --- analysis tooling ---
# matplotlib is the only addition; numpy/opencv/scipy/open3d come from the base.
RUN pip3 install --no-cache-dir matplotlib

# Experiment scripts + config live in a bind-mountable dir so we can iterate
# (edit YAML/Python, re-run) without rebuilding the image.
RUN mkdir -p /opt/custom
COPY docker-custom/config/  /opt/custom/config/
COPY docker-custom/scripts/ /opt/custom/scripts/
RUN chmod +x /opt/custom/scripts/*.py /opt/custom/scripts/*.sh 2>/dev/null || true

# Entrypoint: source ROS + workspace, set DDS + CUDA env, then exec.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/ros_ws/install/setup.bash' \
    'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
    'export PYTHONPATH=/opt/custom/scripts:${PYTHONPATH:-}' \
    'export CUDA_HOME=/usr/local/cuda-13.0' \
    'export PATH=${CUDA_HOME}/bin:${PATH}' \
    'export LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH:-}' \
    'exec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /opt/custom
CMD ["bash"]
