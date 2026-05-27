FROM nvcr.io/nvidia/deepstream:9.0-triton-multiarch

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        libyaml-cpp-dev \
        libpcap-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# rs_driver is header-only; copy the include tree only.
COPY src/rs_driver/src/ /opt/rs_driver/src/

# Build the Ds3D dataloader and install into DeepStream's lib dir.
COPY docker-ds3d/plugin/ /opt/ds3d-rslidar/plugin/
RUN cd /opt/ds3d-rslidar/plugin && \
    make RS_DRIVER_DIR=/opt/rs_driver -j"$(nproc)" && \
    make install

# Pipeline config
COPY docker-ds3d/config/ /opt/ds3d-rslidar/config/

# Run deepstream-lidar-inference-app from a directory where it can read its
# config & write any logs — the app's relative paths assume CWD == config dir.
WORKDIR /opt/ds3d-rslidar/config
ENV LD_LIBRARY_PATH=/opt/nvidia/deepstream/deepstream/lib:${LD_LIBRARY_PATH}

CMD ["/opt/nvidia/deepstream/deepstream/bin/deepstream-lidar-inference-app", \
     "-c", "config_rs_airy_render.yaml"]
