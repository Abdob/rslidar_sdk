# docker-compose — orchestrating the rslidar_sdk pipeline

[docker-compose.yaml](docker-compose.yaml) at the project root provides a
single entry point for all the multi-container workflows in this repo. It
selects which combination of services to start using `--profile <name>`.

Use this instead of running the per-container `docker_run.sh` scripts when
a workflow needs more than one container (camera + LiDAR + tooling).

## TL;DR

```
# First-time builds
docker compose --profile colorize build

# Run any workflow
docker compose --profile colorize up        # foreground, Ctrl-C to stop
docker compose --profile colorize up -d     # background
docker compose --profile colorize down      # stop + clean up
```

## Workflows (profiles)

| Profile | What starts | Use for |
|---|---|---|
| `camera` | docker-gst-camera | GPU-accelerated MJPEG → `/image_raw` at 30 fps. Sanity-check the camera in isolation. |
| `lidar-only` | docker-ros2 (LiDAR only) | Baseline AIRY publisher. Lightweight check the LiDAR is reachable. |
| `sensors` | docker-gst-camera + docker-calib LiDAR | Both raw streams (LiDAR + camera) for recording bags or piping into a custom consumer. |
| `intrinsic` | docker-gst-camera + Stage 1 chessboard tool | Calibrate the fisheye lens intrinsics. |
| `extrinsic` | docker-gst-camera + LiDAR + Stage 2 FAST-Calib tool + RViz | Calibrate the camera↔LiDAR pose. |
| `colorize` | docker-gst-camera + LiDAR + colorize_node + RViz | Live colorized point cloud preview. |
| `record` | docker-gst-camera + docker-calib LiDAR | Same as `sensors`, semantically reserved for bag-recording sessions. |

Run multiple profiles together by passing several `--profile` flags. For
example, to record while previewing the colorized cloud at the same time:

```
docker compose --profile colorize --profile record up
```

(The `camera` and `colorize` services will deduplicate — same container
can't start twice.)

## Prerequisites

1. **NVIDIA Container Toolkit** for the camera service to access the GPU.
   On Ubuntu:
   ```
   sudo apt-get install -y nvidia-container-toolkit
   sudo systemctl restart docker
   ```
   Verify with `docker info | grep -i runtime`; you should see `nvidia`
   listed alongside `runc`.

2. **X11 forwarding** for RViz and OpenCV windows from the calib services.
   Once per shell session:
   ```
   xhost +local:docker
   ```

3. **AIRY on a routable interface** (default `192.168.1.200`). Test with
   `ping 192.168.1.200`. The driver uses host network mode, so the LiDAR
   needs to be reachable from the host's network namespace.

4. **`/dev/video0` available** (the USB camera). `ls /dev/video*` on the
   host; if your camera enumerates as `video1`, edit `camera.launch.py`
   args or set in `docker-gst-camera/config/camera.yaml`.

## Examples

### Live colorized point cloud (your most common workflow)

```
xhost +local:docker
docker compose --profile colorize up
```

Brings up:
- `rslidar-airy-gst-camera` — publishing /image_raw at 30 fps
- `rslidar-airy-calib` — running the colorize launch (LiDAR + colorize_node + RViz)

RViz opens automatically and shows `/colored_points`. Ctrl-C exits both.

### Record a bag for FAST-LIVO2

```
docker compose --profile record up -d
docker exec rslidar-airy-calib bash /opt/calib/scripts/record_bag.sh laundromat
# walk the rig
# Ctrl-C exits the record script
docker compose --profile record down
```

Bag lands in `docker-calib/config/bags/laundromat/` ready for the
ROS 2→1 conversion + FAST-LIVO2 processing described in
[docker-livo2/README.md](docker-livo2/README.md).

### Recalibrate intrinsics after changing camera

```
docker compose --profile intrinsic up
```

Camera starts, the OpenCV window opens, follow Stage 1 in
[docker-calib/README.md](docker-calib/README.md). Output:
`docker-calib/config/intrinsics.yaml` + `intrinsics_ros.yaml`. Press
Ctrl-C when done; both files are written to a bind-mounted directory so
they persist on the host.

### Switch between workflows mid-session

```
# Currently running colorize, want to switch to extrinsic recalibration
docker compose --profile colorize down
docker compose --profile extrinsic up
```

(The calib services all share `container_name: rslidar-airy-calib`, so
docker-compose stops the old one before starting the new one.)

### Detached + tail logs from one service

```
docker compose --profile colorize up -d
docker compose --profile colorize logs -f camera   # only camera service logs
docker compose --profile colorize logs -f          # all services
```

## Inspecting state mid-run

The container names are stable, so the existing snippets in the
[docker-calib README](docker-calib/README.md) all keep working unchanged:

```
# Check published topics
docker exec rslidar-airy-calib bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && \
   ros2 topic list'

# Cloud rate
docker exec rslidar-airy-calib bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && \
   ros2 topic hz /rslidar_points --window 100'

# Camera rate (this one is in the camera container, on Jazzy)
docker exec rslidar-airy-gst-camera bash -c \
  'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && \
   ros2 topic hz /image_raw --window 100'
```

## Design choices worth knowing

- **Profiles, not separate compose files.** Cleaner than maintaining
  `compose.colorize.yaml`, `compose.extrinsic.yaml`, etc. Add a workflow
  by adding one service block + listing its profile.

- **`container_name` reused** across the calib variants
  (`sensors`/`intrinsic`/`extrinsic`/`colorize`). docker-compose enforces
  one-at-a-time and the existing `docker exec rslidar-airy-calib ...`
  commands in the READMEs keep working.

- **Build context = project root** (`.`). The docker-calib Dockerfile
  copies from `src/`, `node/`, etc., so the build context must include
  the rslidar_sdk source tree. The Dockerfile path tells compose which
  Dockerfile in which subdir to actually use.

- **`runtime: nvidia` AND `deploy.resources.reservations.devices` set**
  on the camera service. Covers both pre-v2.3 docker-compose (which uses
  `runtime`) and v2.3+ (which uses `deploy.resources`). Either path
  triggers the NVIDIA container runtime.

- **No bridge between Jazzy and Humble.** docker-gst-camera is on ROS 2
  Jazzy (Ubuntu 24.04 DeepStream base), docker-calib and docker-ros2 are
  on Humble (Ubuntu 22.04). ROS 2's DDS messaging is wire-compatible
  across LTS distros for the standard message types we use
  (`sensor_msgs/Image`, `PointCloud2`, `CompressedImage`, `Imu`), so
  cross-container topics flow without `ros_bridge` or similar.

- **docker-ds3d and docker-livo2 left off-compose.** They have very
  different lifecycles (DS3D = inference app, LIVO2 = offline ROS 1 bag
  processing). They each have their own `docker_run.sh` and are usually
  driven outside the camera/LiDAR live pipeline.

## Troubleshooting

- **`could not select device driver "nvidia"`**: NVIDIA Container
  Toolkit isn't active. See Prerequisites #1.
- **Camera service exits immediately with `Could not open device
  /dev/video0`**: another container (or a stale process on the host) has
  the camera open. `docker ps -a | grep video` and stop the other.
- **`Error response from daemon: Conflict. The container name "/rslidar-airy-calib"
  is already in use`**: you ran `up` on a second profile while another
  was still running. `docker compose --profile <previous> down` first.
- **RViz window doesn't appear**: `xhost +local:docker` not run in this
  session, or `$DISPLAY` is empty. Check `echo $DISPLAY`.
- **`/colored_points` empty in RViz** but `/rslidar_points` and
  `/image_raw` both publish: colorize_node didn't find `extrinsic.yaml`
  or `intrinsics.yaml`. Both must exist in `docker-calib/config/` — run
  the intrinsic and extrinsic profiles first.
- **`/image_raw` topic missing on Humble subscribers**: image_transport
  cross-distro hint negotiation can stall. Tell consumers (RViz, your
  scripts) to read `/image_raw/compressed` instead — that transport is
  wire-identical across distros.

## File map

| Path | Purpose |
|------|---------|
| [docker-compose.yaml](docker-compose.yaml) | The compose file itself |
| [docker-gst-camera/](docker-gst-camera/) | GPU MJPEG camera (Jazzy + DeepStream) |
| [docker-calib/](docker-calib/) | Calibration + colorize tooling (Humble) |
| [docker-ros2/](docker-ros2/) | Minimal AIRY publisher baseline (Humble) |
| [docker-livo2/](docker-livo2/) | FAST-LIVO2 SLAM (Noetic, offline bag processing) |
| [docker-ds3d/](docker-ds3d/) | DeepStream lidar inference |
