# docker-ds3d — RoboSense AIRY through NVIDIA DeepStream (Ds3D)

Live AIRY point cloud rendered inside an NVIDIA **DeepStream 9.0 Ds3D**
pipeline, using a custom `ds3d::dataloader` we built to wrap `rs_driver`. The
existing `deepstream-lidar-inference-app` (shipped with DeepStream) drives
the pipeline; the only piece we contribute is the dataloader `.so` plus a
YAML config.

## What it builds

- Base: `nvcr.io/nvidia/deepstream:9.0-triton-multiarch` (Ubuntu 24.04)
- Compiles `plugin/` into `libnvds_rs_lidar_dataloader.so` and installs to
  `/opt/nvidia/deepstream/deepstream/lib/`
- Bakes two YAML pipelines from `config/`

## Architecture

```
RoboSense AIRY (UDP)
        │
        ▼
libnvds_rs_lidar_dataloader.so      ← our code, wraps rs_driver
        │  (ds3d::datamap, key=DS3D::LidarXYZI, [N,4] FP32 GPU buffer)
        ▼
libnvds_3d_gl_datarender.so         ← stock DeepStream OpenGL renderer
        │
        ▼
   GL window on $DISPLAY
```

Source layout:
- [plugin/rs_lidar_source.h](plugin/rs_lidar_source.h) — C-API factory
  `createRsLidarLoader`
- [plugin/rs_lidar_config.h](plugin/rs_lidar_config.h) — YAML parsing
- [plugin/rs_lidar_source_impl.{h,cpp}](plugin/rs_lidar_source_impl.cpp) —
  derives from `ds3d::SyncImplDataLoader`, owns the `rs_driver` instance,
  converts AIRY's packed `PointXYZI` into `[N,4]` FP32 for Ds3D
- [plugin/Makefile](plugin/Makefile) — links against `libnvds_3d_common`,
  CUDA, libpcap, yaml-cpp

## Prereqs

- NVIDIA GPU + driver 590+ on the host (matches DS 9.0 requirements)
- Docker with NVIDIA Container Toolkit (`--gpus all` works)
- AIRY reachable + destination IP set to one of your host's IPs (same as
  [../docker/README.md](../docker/README.md))
- X11 for the GL window:
  ```
  xhost +local:docker
  ```
  (the run script does this automatically)

## Build

```
./docker-ds3d/docker_build.sh
```
The base image is ~26 GB on disk; the rebuild loop after that is fast (only
the plugin recompiles).

## Run — GL render

```
./docker-ds3d/docker_run.sh
```
An OpenGL window opens with the top-down point cloud, intensity-colored.
Steady-state log:
```
rs_lidar: 30 frames forwarded (last=86400 pts, ts=2514.903)
fps 10.001213
```

## Run — headless (verify pipeline)

The repo also ships [config/config_rs_airy_fakesink.yaml](config/config_rs_airy_fakesink.yaml),
which omits the renderer. The app falls back to `fakesink`, useful for
confirming the dataloader is healthy without needing X11:

```
docker run --rm --gpus all --net=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  rslidar-airy-ds3d \
  /opt/nvidia/deepstream/deepstream/bin/deepstream-lidar-inference-app \
  -c config_rs_airy_fakesink.yaml
```

## Customize

All knobs are in [config/config_rs_airy_render.yaml](config/config_rs_airy_render.yaml):

- **LiDAR config** (`rs_airy_source` block):
  `lidar_type`, `msop_port`, `difop_port`, `imu_port`, `host_address`,
  `min_distance`, `max_distance`, `mem_type` (`cpu`|`gpu`), `gpu_id`,
  `max_points`, `mem_pool_size`
- **Camera / zoom** (`rs_airy_render` block):
  - `fov` — field of view in degrees; lower = telephoto zoom
  - `view_position: [0, 0, 60]` — camera 60 m above origin; shorten the
    Z component to fly closer
  - `view_target`, `view_up`, `near`, `far`

The config is `COPY`'d into the image, so any edit needs a rebuild. To
iterate without rebuilding, bind-mount the dir:
```
docker run --rm -it --gpus all --net=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/docker-ds3d/config:/opt/ds3d-rslidar/config \
  --device /dev/dri --name rslidar-airy-ds3d \
  rslidar-airy-ds3d
```

## Gotchas worth remembering

1. **`rs_driver` `PointXYZI` is `#pragma pack(1)`** — 13 bytes (3 floats +
   uint8), not 4 floats. DS3D expects `[N, 4]` FP32, so the dataloader
   converts per-point and rescales intensity to `[0, 1]`.

2. **Argument-evaluation order is unspecified across function calls.**
   ```cpp
   // UB — lambda may move dstBuf before dstBuf->data is read
   wrapLidarXYZIFrame(dstBuf->data, ..., [keep=std::move(dstBuf)]...);
   ```
   gcc 13 evaluates the lambda capture first → segfault on the empty
   `shared_ptr` dereference. Read the pointer into a local first:
   ```cpp
   void* const ptr = dstBuf->data;
   wrapLidarXYZIFrame(ptr, ..., [keep=std::move(dstBuf)]...);
   ```

3. **The renderer has no built-in mouse zoom.** The GL window doesn't accept
   scroll/drag. Adjust `fov` or `view_position` in the YAML and re-run.
