# docker-ros2 — RoboSense AIRY in ROS 2 Humble + RViz2

Live AIRY point cloud published to `/rslidar_points` as `sensor_msgs/PointCloud2`,
visualized in RViz2 — the canonical use of `rslidar_sdk`. Everything (driver,
RViz2, preset RViz config) is launched by one command.

## What it builds

- Base: `ros:humble` (Ubuntu 22.04, multi-arch incl. arm64 for the DGX Spark;
  RViz2 added via apt)
- Colcon workspace at `/opt/ros_ws` with two packages:
  - `rslidar_msg` — cloned from
    [github.com/RoboSense-LiDAR/rslidar_msg](https://github.com/RoboSense-LiDAR/rslidar_msg)
  - `rslidar_sdk` — this project, copied in from the build context
- Default CMD launches [../launch/humble_start.py](../launch/humble_start.py),
  which starts `rslidar_sdk_node` and `rviz2` together

## Prereqs

- Docker
- AIRY at `192.168.0.200`, reachable from the DGX Spark host (`192.168.0.199`),
  with the LiDAR's **destination IP (`DstIp`) set to `192.168.0.199`** in its web
  UI (http://192.168.0.200 → Setting). If `DstIp` points elsewhere the driver
  logs `ERRCODE_MSOPTIMEOUT` and no cloud appears. See
  [../docker/README.md](../docker/README.md).
- X11 for RViz2:
  ```
  xhost +local:docker
  ```
  (the run script does this automatically)

## Build

```
./docker-ros2/docker_build.sh
```

## Run

```
./docker-ros2/docker_run.sh
```
RViz2 opens on your desktop, already configured for `/rslidar_points` with
intensity coloring (see [../rviz/rviz2.rviz](../rviz/rviz2.rviz)). Logs show:
```
[rslidar_sdk_node-1] Send PointCloud To : ROS
[rslidar_sdk_node-1] PointCloud Topic: /rslidar_points
[rslidar_sdk_node-1] RoboSense-LiDAR-Driver is running.....
[rviz2-2] OpenGl version: 4.5 (GLSL 4.5)
```

Ctrl-C to stop.

## Inspect topics from another shell

```
docker exec -it rslidar-airy-ros2 bash
# inside container:
source /opt/ros_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
ros2 topic hz /rslidar_points
```

## Customize

- **LiDAR config** is at [config/config.yaml](config/config.yaml). It's
  `COPY`'d on top of the project's default `config/config.yaml` during the
  build, so editing this file (and rebuilding) is the override path —
  the host project's config is never modified.
- **RViz layout** is at [../rviz/rviz2.rviz](../rviz/rviz2.rviz). Edit in
  RViz2 (`File → Save Config`) and re-run to persist.

## Troubleshooting

- **RViz panel goes red — "No transform from 'rslidar' to 'map'":** change
  *Fixed Frame* (under Global Options) from `map` to `rslidar`. That's the
  `ros_frame_id` set in `config.yaml`.
- **No point cloud appearing, no error:** check `ros2 topic hz /rslidar_points`
  reports >0 Hz. If 0 Hz, the driver isn't receiving UDP — verify with
  `tcpdump host 192.168.0.200 and udp` on the host. The AIRY's destination IP
  must be set to the host (`192.168.0.199`).
- **Build fails on missing `<memory>`:** the Dockerfile passes
  `-DCMAKE_CXX_FLAGS="-include memory -include functional"` to colcon to
  work around vendored `rs_driver` headers that use `std::shared_ptr`
  without including `<memory>`. Don't drop those flags.
- **`ros-humble-rmw-cyclonedds-cpp` install prompt:** `humble_start.py`
  checks for Cyclone DDS at launch time and tries to `sudo apt-get install`
  it if absent. The Dockerfile pre-installs it, so the check short-circuits
  silently. If you swap launch files, keep that apt package installed.
