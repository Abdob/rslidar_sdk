# docker-gst-camera — GPU-accelerated MJPEG camera publisher

GStreamer + nvjpeg + nvvideoconvert pipeline that publishes the AIRY rig's
USB camera as standard ROS 2 topics at the camera's native 30 fps. Replaces
the ros-humble-usb-cam pipeline (capped at ~19 fps by single-threaded
libjpeg-turbo decode inside the capture node).

## Why a separate container

[docker-calib](../docker-calib/) is plain Ubuntu + ROS 2 with no CUDA.
[docker-ds3d](../docker-ds3d/) has DeepStream/nvjpeg but no ROS. This
container combines DeepStream 9.0 (for the NVIDIA GStreamer plugins) with
ROS 2 Jazzy + gscam2, and publishes `/image_raw` so any of the other
containers can consume it over the host network.

## Architecture

```
v4l2src device=/dev/video0
   │
   ▼
image/jpeg, 1280x720, 30/1
   │
   ▼  ── GPU pipeline (use_gpu:=true, default) ──────────
nvv4l2decoder mjpeg=1        nvjpeg-backed MJPEG decode, NVMM memory
   │
nvvideoconvert               NV12 → BGR, still on GPU
   │
video/x-raw, format=BGR      copy to system memory at appsink
   │
   ▼  ── CPU fallback (use_gpu:=false) ─────────────────
jpegdec                      libjpeg-turbo, ~19 fps single-thread cap
   │
videoconvert
   │
   ▼
gscam2 appsink → ROS publisher
   │
/image_raw, /image_raw/compressed, /camera_info
```

## Build

```
./docker-gst-camera/docker_build.sh
```

Tags `rslidar-airy-gst-camera`. First build downloads the DeepStream 9.0
base (~7 GB), installs ROS 2 Jazzy + tools, and builds gscam2 from source.
Plan for ~15 min on first run; rebuilds are cached unless the Dockerfile
changes.

## Run

```
./docker-gst-camera/docker_run.sh                          # GPU, 1280x720@30 (default)
./docker-gst-camera/docker_run.sh --use_gpu false          # CPU fallback
./docker-gst-camera/docker_run.sh --width 1920 --height 1080  # 1080p (needs recal)
./docker-gst-camera/docker_run.sh -- bash                  # interactive shell
```

Requires NVIDIA Container Toolkit on the host (`apt install
nvidia-container-toolkit` once, then restart docker) so `--gpus all` works.

Bind mounts:
- `docker-calib/config/intrinsics_ros.yaml` → `/opt/calib/config/intrinsics_ros.yaml`
  (the camera_info file produced by Stage 1 in docker-calib)
- `docker-gst-camera/config/`, `docker-gst-camera/launch/` — editable from host

## Verify

In another terminal:
```
# From inside the container (avoids needing ROS on the host)
docker exec -it rslidar-airy-gst-camera bash -c \
  'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && \
   ros2 topic hz /image_raw --window 100'
```

> **Note on ROS distros.** This container runs **Jazzy** (matched to DeepStream 9.0's
> Ubuntu 24.04 base) while docker-calib and docker-ros2 run **Humble** (Ubuntu 22.04).
> They communicate over DDS without bridging — `sensor_msgs/Image` and friends are
> wire-compatible across these distros.

Expected at the GPU setting: **30.0 Hz**, std dev < 5 ms.
Expected with `--use_gpu false`: ~18–20 Hz (libjpeg-turbo decode cap).

## How downstream consumers should connect

This container publishes `/image_raw` and `/image_raw/compressed` on the
host DDS network (`--net=host`). Any other container also on `--net=host`
sees them with no config.

For docker-calib's calibration tools or colorize_node: just **don't start
usb_cam** in [docker-calib/launch/sensors.launch.py](../docker-calib/launch/sensors.launch.py).
The LiDAR launches on its own there; `/image_raw` arrives from this
container.

For FAST-LIVO2 (ROS 1 in docker-livo2): the topic still has to be bridged
through `ros1_bridge`, same as before.

## Files

| Path | Purpose |
|------|---------|
| [Dockerfile](Dockerfile) | DeepStream 9.0 + ROS 2 Jazzy + gscam2 from source |
| [config/camera.yaml](config/camera.yaml) | gscam2 parameters; default pipeline is overridden by launch |
| [launch/camera.launch.py](launch/camera.launch.py) | Builds GStreamer pipeline string, picks GPU or CPU |
| [docker_build.sh](docker_build.sh) | Tags `rslidar-airy-gst-camera` |
| [docker_run.sh](docker_run.sh) | Adds `--gpus all`, bind-mounts intrinsics from docker-calib |

## Troubleshooting

- **`Failed to load plugin nvv4l2decoder`**: NVIDIA container toolkit not
  set up. `docker info | grep -i runtime` should list `nvidia`. Install
  with `sudo apt install nvidia-container-toolkit && sudo systemctl
  restart docker`.
- **`Could not open device /dev/video0`**: the camera is busy in another
  container (`docker stop rslidar-airy-calib`) or not enumerated. Check
  with `ls /dev/video*` on the host.
- **`/image_raw` rate at ~30 Hz with `--use_gpu false`**: your camera's
  libjpeg-turbo is faster than expected — feel free to leave it on CPU.
- **`Invalid pipeline: no element "nvv4l2decoder"`**: DeepStream plugins
  not on `GST_PLUGIN_PATH`. The entrypoint sets `LD_LIBRARY_PATH` to
  include DeepStream's lib dir; verify with `gst-inspect-1.0
  nvv4l2decoder` inside the container.
