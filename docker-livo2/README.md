# docker-livo2 — Tightly-coupled LiDAR-Inertial-Visual SLAM for the AIRY+camera rig

Companion to [docker-calib](../docker-calib/). Once you have a calibrated
camera + AIRY rig (intrinsics + extrinsic YAMLs), this container runs
[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) to build a globally-
consistent **colored point cloud / mesh** by walking the sensors through a
space.

Why a separate container: FAST-LIVO2 is ROS 1 (Noetic) only. We keep the
ROS 2 (Humble) calibration toolchain intact in `docker-calib` and process
recordings offline in this ROS 1 container.

## Pipeline

```
   AIRY UDP ─► docker-calib (ROS 2 Humble) ─► ros2 bag record ─► .db3 bag
                                                                    │
                                            rosbags-convert (rosbags pkg)
                                                                    ▼
                                                            ROS 1 bag (.bag)
                                                                    │
   docker-livo2 (ROS 1 Noetic) ◄──────────────────────────────────────┘
        │
        ▼
   FAST-LIVO2  ─►  /Laser_map  (fused colored cloud)
                   /path       (estimated trajectory)
                   /aft_mapped_to_init  (current pose)
```

## Build

```
./docker-livo2/docker_build.sh
```

Tags `rslidar-airy-livo2`. The first build takes 10–15 min:
ROS 1 Noetic desktop-full + Sophus (patched commit `a621ff`) + Vikit +
livox_ros_driver + FAST-LIVO2. Disk footprint ~6 GB.

## Run

```
./docker-livo2/docker_run.sh                    # interactive bash
./docker-livo2/docker_run.sh avia               # roslaunch mapping_avia.launch
./docker-livo2/docker_run.sh ouster             # roslaunch mapping_ouster_ntu.launch
./docker-livo2/docker_run.sh hesai              # Hesai XT32 launch
./docker-livo2/docker_run.sh -- <any-command>   # pass through
```

Bind mounts:
- `./bags`   → `/data`         — drop downloaded or converted bags here.
- `./config` → `/opt/calib`    — for your own `intrinsics.yaml` /
                                  `extrinsic.yaml` / `rsairy.yaml` (next
                                  milestone).

## Stage A — Sanity test on the author's bag

Goal: prove the toolchain works before bringing your own sensors into it.

1. Grab a small bag from the [FAST-LIVO2-Dataset OneDrive](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z)
   (linked from the [FAST-LIVO2 README](https://github.com/hku-mars/FAST-LIVO2)).
   Start with anything in the **Avia** group — it's hardware-synchronised
   on their handheld unit so it's the cleanest input.
2. Drop the `.bag` into `docker-livo2/bags/`.
3. Two terminals:
   ```
   # terminal 1
   ./docker-livo2/docker_run.sh avia

   # terminal 2 (after RViz comes up)
   ./docker-livo2/docker_run.sh -- rosbag play /data/<your_bag>.bag
   ```
4. Watch RViz. The trajectory line should grow as the bag plays. The
   `/Laser_map` colored cloud accumulates structure. If you see geometry
   building but no colors, the camera topic remap is off — fixable in
   `config/avia.yaml` inside the container.

## Stage B — Process a bag from your own rig

This is the real goal. Three sub-steps:

### B.1 Record from your rig (using `docker-calib`)

In the calibration container (which already has the AIRY driver, usb_cam,
and the `record_bag.sh` helper):

```
./docker-calib/docker_run.sh sensors     # in terminal 1: starts sensors
./docker-calib/docker_run.sh -- bash /opt/calib/scripts/record_bag.sh laundromat
                                          # in terminal 2: records to
                                          # docker-calib/config/bags/laundromat
```

Topics captured:
| Topic                 | Type                     | Notes                                |
|-----------------------|--------------------------|--------------------------------------|
| `/rslidar_points`     | `sensor_msgs/PointCloud2`| AIRY 10 Hz cloud                     |
| `/rslidar_imu_data`   | `sensor_msgs/Imu`        | AIRY built-in IMU (~200 Hz)          |
| `/image_raw`          | `sensor_msgs/Image`      | fisheye camera, 30 fps               |

**Required setup before recording:**
- `imu_port: 6688` in [docker-calib/config/lidar_config.yaml](../docker-calib/config/lidar_config.yaml).
  This is the AIRY's default IMU UDP port. Verify the IMU is actually
  streaming: `ros2 topic hz /rslidar_imu_data` should show ~200 Hz.
- Camera is up: `ros2 topic hz /image_raw` should show 30 Hz.

### B.2 Convert ROS 2 bag → ROS 1 bag

FAST-LIVO2 reads classic `.bag` files; `ros2 bag record` produces the new
`.db3` SQLite format. Use `rosbags-convert` (Python package; works without
ROS installed):

```
pip3 install --user rosbags
rosbags-convert --src docker-calib/config/bags/laundromat \
                --dst docker-livo2/bags/laundromat.bag
```

### B.3 Create an `rsairy.yaml` config and launch

FAST-LIVO2 ships configs for Avia/Mid360/Ouster, not the AIRY. We adapt
one. **This file does not exist yet — it's the next thing to write.** It
will reference:
- LiDAR topic + type (PointCloud2, AIRY's field schema)
- IMU topic + IMU-to-LiDAR extrinsic (from the AIRY datasheet)
- Image topic + camera intrinsics (from your `intrinsics.yaml`)
- Camera-to-LiDAR extrinsic (from your `extrinsic.yaml`)
- Camera-to-IMU time offset (from a Kalibr run)

Once written, mount the config dir and launch:
```
./docker-livo2/docker_run.sh -- roslaunch fast_livo mapping_rsairy.launch
```

## Troubleshooting

- **Sophus `unit_complex_.real() = 1.;` error during build.**
  Patched in the Dockerfile. If a future Sophus pull re-breaks it, the
  pattern is `s|.real() = |.real(|` and matching close-paren — see comment
  in the Dockerfile.
- **`vikit_common` won't find Sophus.**
  `ldconfig` line missing in the Sophus install step, or a stale
  workspace. `rm -rf /opt/catkin_ws/build /opt/catkin_ws/devel` inside the
  container and `catkin_make` again.
- **RViz black / X11 forwarding broken.**
  `xhost +local:docker` on the host. The run script already calls this
  but it sometimes silently fails if you SSH'd in.
- **Bag plays but `/Laser_map` is empty.**
  Topic name mismatch. Inside the container, `rosbag info /data/foo.bag`
  to see actual topic names, then either remap on `rosbag play` (e.g.
  `rosbag play foo.bag /old:=/new`) or edit the config YAML.
- **Times totally wrong / drift huge.**
  Camera time-offset isn't calibrated yet. Use Kalibr `--cam-imu` mode
  before trusting any LIVO output from your own rig.

## Files

| Path | Purpose |
|------|---------|
| [Dockerfile](Dockerfile) | Noetic + Sophus(a621ff patched) + Vikit + livox_ros_driver + FAST-LIVO2 |
| [docker_build.sh](docker_build.sh) | Tags `rslidar-airy-livo2` |
| [docker_run.sh](docker_run.sh) | Aliases for `avia`/`hesai`/`ouster`/etc; bind-mounts `bags/` and `config/` |
| `bags/` | Bind-mounted to `/data` in the container. Drop downloaded or converted bags here. |
| `config/` | Bind-mounted to `/opt/calib` for your `intrinsics.yaml` + `extrinsic.yaml` + (future) `rsairy.yaml`. |

## Method credit

[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) — Chunran Zheng et al.,
HKU MARS lab, T-RO 2024. See the paper for the theory:
[arxiv.org/pdf/2408.14035](https://arxiv.org/pdf/2408.14035).
