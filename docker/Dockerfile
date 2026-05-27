FROM ubuntu:22.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        cmake \
        libpcap-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/rs_driver
COPY src/rs_driver/ /opt/rs_driver/

RUN cmake -S . -B build \
        -DCOMPILE_DEMOS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-include memory -include functional" \
    && cmake --build build -j"$(nproc)"

CMD ["stdbuf", "-oL", "/opt/rs_driver/build/demo/demo_online"]
