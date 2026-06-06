# docker-calib — Camera × LiDAR calibration + colorized point clouds

End-to-end pipeline for a RoboSense AIRY + a 180° fisheye USB camera mounted
to the same rig:

1. **Fisheye intrinsic calibration** with a checkerboard.
2. **Camera ↔ LiDAR extrinsic calibration** using a FAST-Calib-style board
   (4 ArUco markers + 4 circular holes).
3. **Live colorized point cloud** publisher that combines the two — RViz2
   shows the AIRY's geometry with per-point colors sampled from the camera.
4. **Timestamp sync + bag recording for FAST-LIVO2** — put LiDAR, IMU, and
   camera on one clock and record the three input topics.

## Architecture

```
              ┌──────────────────┐
   /dev/video0│   usb_cam (ROS)  │── /image_raw ───────────────┐
              └──────────────────┘                              │
                                                                ▼
                                              ┌─────────────────────────────┐
   AIRY UDP ──► rslidar_sdk_node ── /rslidar_points ──► colorize_node.py
                                              │       (transforms + projects)
                                              ▼
                                       /colored_points  ─► RViz2 (RGB8)
                                              ▲
                       intrinsics.yaml ───────┤
                       extrinsic.yaml ────────┘
```

## Build

```
./docker-calib/docker_build.sh
```
Tags `rslidar-airy-calib`. Base is `osrf/ros:humble-desktop` with `usb_cam`,
`rslidar_sdk`, OpenCV 4.8 + ArUco contrib, and Open3D.

## Run (one wrapper, four modes)

```
./docker-calib/docker_run.sh                    # shell inside /opt/calib
./docker-calib/docker_run.sh sensors            # AIRY + usb_cam only
./docker-calib/docker_run.sh intrinsic          # usb_cam + chessboard tool
./docker-calib/docker_run.sh test-intrinsic
./docker-calib/docker_run.sh extrinsic          # AIRY + usb_cam + extrinsic tool
./docker-calib/docker_run.sh colorize           # everything + RViz2
```

`config/`, `scripts/`, `launch/`, `rviz/` are **bind-mounted** from the host
directory, so editing YAML or Python and rerunning takes effect without a
rebuild.

## Sanity-check sensor data (do this before anything else)

After starting `sensors`, verify each stream is publishing at the expected
rate.

### Why you have to source ROS inside `docker exec`

The container's `ENTRYPOINT` ([Dockerfile](Dockerfile#L65)) sources ROS
and the workspace automatically — but only for the *initial* command
(`bash`, the launch you started). `docker exec` attaches a **fresh shell**
that bypasses the entrypoint, so `ros2` isn't on the PATH. You have two
choices:

**(a) One-time source then run commands** — open one interactive exec
shell and source once:
```
docker exec -it rslidar-airy-calib bash
source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash
ros2 topic hz /rslidar_points
```

**(b) Source inline per `docker exec`** — convenient for one-liners from
the host:
```
docker exec rslidar-airy-calib bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && \
   ros2 topic hz /rslidar_points'
```

For repeated use, defining a host-side function in your shell rc is the
ergonomic version:
```
rosx() {
  docker exec rslidar-airy-calib bash -c \
    "source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && $*"
}
# then:  rosx ros2 topic hz /rslidar_points
```

### The actual rate checks

```
# What's published right now?
docker exec rslidar-airy-calib bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && \
   ros2 topic list | grep -E "rslidar|image"'

# Per-topic rates (5 s sample each)
docker exec rslidar-airy-calib bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && \
   timeout 5 ros2 topic hz /rslidar_points'

docker exec rslidar-airy-calib bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && \
   timeout 5 ros2 topic hz /rslidar_imu_data'

docker exec rslidar-airy-calib bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/ros_ws/install/setup.bash && \
   timeout 5 ros2 topic hz /image_raw'
```

Expected:

| Topic               | Rate     | If missing / wrong                                          |
|---------------------|----------|-------------------------------------------------------------|
| `/rslidar_points`   | ~10 Hz   | Check LiDAR ethernet link + `msop_port`/`difop_port` config |
| `/rslidar_imu_data` | ~200 Hz  | See "IMU not publishing" below                              |
| `/image_raw`        | ~30 Hz   | USB bandwidth or `pixel_format` mismatch                    |

### IMU not publishing — two distinct causes

1. **Driver compiled without IMU support.** The startup log prints
   `imu_port: 0` regardless of YAML config → the binary needs
   `-DENABLE_IMU_DATA_PARSE=ON` at CMake time. Already set in the
   [Dockerfile](Dockerfile#L52); if you suspect a stale image, rebuild:
   ```
   ./docker-calib/docker_build.sh
   ```
2. **LiDAR not sending IMU packets.** Driver log shows `imu_port: 6688`
   but `ros2 topic hz` says "topic does not appear to be published yet".
   Confirm with tcpdump:
   ```
   docker exec rslidar-airy-calib bash -c \
     'apt-get -qq install -y tcpdump 2>/dev/null; timeout 5 tcpdump -i any -n udp port 6688 -c 5'
   ```
   - **0 packets** → enable IMU output via the LiDAR's web UI
     (`http://<lidar_ip>`, default `192.168.1.200`). Set destination
     port to 6688 and save.
   - **Packets arriving** → driver-side parsing issue, check the AIRY
     firmware version against the rslidar_sdk version.

## Stage 1 — Fisheye intrinsic calibration

You need a real checkerboard (printed or laser-printed on rigid backing) and
its physical dimensions. Defaults in [scripts/calibrate_intrinsic.py](scripts/calibrate_intrinsic.py):
9×6 inner corners, 25 mm squares — override with CLI flags or by editing the
script.

```
./docker-calib/docker_run.sh intrinsic
```

In the live OpenCV window:
- Move the board so it covers different regions of the FOV, including the
  **corners and edges** — that's where fisheye distortion is strongest. Hold
  it at varied tilt angles too.
- Press `c` to capture (only when "detected: YES" is green).
- Aim for **20–30 captures** spread across the frame.
- Press `r` to run calibration. RMS should land below ~0.6 px for a sharp
  camera.

Writes:
- `config/intrinsics.yaml` — native format used by the colorizer + extrinsic
  tool (fisheye K, D).
- `config/intrinsics_ros.yaml` — camera_info_manager YAML for `usb_cam` to
  publish `camera_info` (RViz/foxglove pick it up automatically).

### Custom checkerboard

Override defaults at the CLI inside the container:
```
python3 /opt/calib/scripts/calibrate_intrinsic.py \
    --cols 8 --rows 5 --square 0.030
```
`--cols`/`--rows` are the count of **inner corners** (one less than the
square count per side).

## Stage 2 — Extrinsic calibration (FAST-Calib)

You need the **FAST-Calib target board**:
- 4 ArUco markers, one near each corner.
- 4 circular holes cut through the board.
- Either you built it yourself or got it from the FAST-Calib repo —
  measure it and update [config/target.yaml](config/target.yaml) to match.

**Critical: edit `config/target.yaml` to match your board.** The ArUco
dictionary, marker IDs, marker positions in the board frame, marker side
length, and hole positions all have to be correct.

Also adjust the `lidar_detect.crop_xyz_min/max` window in target.yaml to
roughly bracket where the board sits in front of the LiDAR.

Then run:
```
./docker-calib/docker_run.sh extrinsic
```

### Multi-pose calibration (do this — it's the accuracy lever)

The solver is **multi-pose**: each `c` adds the current board view as one *pose*
and re-solves the extrinsic over **every** accumulated pose at once (a single
Kabsch over all hole correspondences).

Why this matters: the 4 holes are coplanar (they lie on the board), so a
*single* capture under-constrains the out-of-plane rotation and is only valid at
that one board distance/orientation. Extrapolated to other depths (e.g. a wall
2 m away) a single-shot solve drifts by centimeters — visible as colour ghosting
when you merge clouds taken from different viewpoints. Capturing the board at
many depths **and tilts** breaks the coplanarity and averages out per-hole LiDAR
noise, taking the residual from ~cm to single-digit mm.

Rule of thumb (validated in simulation with 1.5 cm per-hole noise): one coplanar
shot ≈ 9 mm error at 2 m; 12 varied poses ≈ 0.8 mm.

**Keys (live window):**

| Key | Action |
|-----|--------|
| `c` | capture the current board view, add it as a pose, re-solve globally |
| `u` | undo — remove the last captured pose and re-solve (drop a bad detection) |
| `s` | save the global solve to `config/extrinsic.yaml` |
| `r` | reset — drop all captured poses |
| `q` | quit |

Procedure:
1. Hold the board flat, ~1.0–2.5 m from the LiDAR, fully visible to both
   sensors. Wait a few seconds for cloud frames to accumulate.
2. Press `c` to add the pose. The tool:
   - Detects ArUco markers, solves fisheye PnP for the board pose, computes
     hole centers in the **camera** frame.
   - Crops cloud, RANSACs the plane, finds 4 low-density regions, computes
     hole centers in the **LiDAR** frame.
   - Brute-forces all 24 permutations of LiDAR-to-camera hole pairings for
     *this* view, then re-solves the extrinsic over all poses so far.
3. **Move the board and repeat — aim for 10–20 poses.** Vary it like the
   intrinsic stage: change the **depth** (1–2.5 m), slide it **left/right/up/
   down**, and crucially **tilt** it (±20–35° in pitch and yaw) between
   captures. Pure sideways translation at one orientation barely helps; tilt +
   depth variation is what conditions the solve.
4. Inspect after each capture:
   - **Camera window.** ArUco IDs overlaid on each marker, plus a yellow
     board outline and 4 green circles where the solver thinks the holes
     project. The green circles must sit inside the actual physical holes
     — if they're off, the camera intrinsics or marker IDs are wrong, not
     the LiDAR side.
   - **lidar dbg window.** One panel showing the LiDAR points after they've
     been cropped and projected onto the RANSAC plane:
     - **Gray dots** — all plane inliers (the board's surface points).
     - **Red dots** — boundary points (points whose neighbors leave a
       >120° angular gap; these cluster at hole edges and the board's
       outer edge).
     - **Yellow rings** — every cluster that fit a circle of roughly the
       expected hole radius.
     - **Green rings + crosses** — the 4 candidates the geometric-
       consistency check kept (their pairwise distances best match the
       expected hole rectangle).
   - **Status line** at the top of the dbg panel: `plane=N  boundary=N
     candidates=N  selected=N`. A healthy capture has:
     - `plane` in the thousands (board points after crop).
     - `boundary` 100–600 (mostly the four hole edges + the outer edge).
     - `candidates` 4–8 (real holes + a couple of scan-ring artifacts).
     - `selected = 4`.
     If anything is 0, the failure stage is right there.
   - **Terminal log.** Each capture prints its own per-pose residual and the
     re-solved global fit:
     ```
     added pose #7 (per-pose rms=6.1 mm, perm=(3,0,2,1))
     GLOBAL: poses=7 pts=28 rms=4.82mm worst-pose=8.9mm  euler_xyz_deg=(0.41,-1.28,88.61) t=[-0.001,-0.097,-0.044]
     ```
     Sanity checks:
     - **global RMS** falls as you add good poses — aim for **single-digit mm**
       (the old single-shot solve sat around 25 mm). 5–15 mm is usable; if it
       won't drop below ~15 mm, your hole detections are noisy (see the dbg
       panel) or you haven't varied the board enough.
     - **`worst-pose`** flags the least-consistent capture. If one pose is far
       above the rest, press `u` to drop it and the fit re-solves without it.
     - **`t[1]`** should roughly match your measured LiDAR↔camera vertical
       offset (here ~10 cm → -0.10 m in the camera frame).
     - **`t[2]`** should be a few cm at most. If it lands near ±1 m, a capture
       hit the **180° mirror** of the true solution (the hole rectangle has that
       symmetry); `u`ndo that pose and recapture.
5. Press `s` to save once the global RMS is low and stable. Writes
   `config/extrinsic.yaml` with `num_poses`, `num_correspondences`, and
   `per_pose_rms_m` alongside the transform so the result is auditable.

If RMS won't drop or holes get detected in wrong places, re-tune `crop_xyz_*`,
`plane_ransac_distance`, or `hole_density_threshold` in
[config/target.yaml](config/target.yaml), and use `u` to discard the poses that
spiked `worst-pose`.

### Running with custom arguments

`docker_run.sh extrinsic` launches the tool with defaults (topics `/image_raw`
and `/rslidar_points`, configs under `/opt/calib/config/`). To override anything
— different topics, a non-default target/intrinsics file, or writing the result
elsewhere — run the script directly inside the container instead of the launch:

```
# sensors already up via:  ./docker-calib/docker_run.sh sensors
./docker-calib/docker_exec.sh python3 /opt/calib/scripts/calibrate_extrinsic.py \
    --image_topic /image_raw \
    --cloud_topic /rslidar_points \
    --target      /opt/calib/config/target.yaml \
    --intrinsics  /opt/calib/config/intrinsics.yaml \
    --out         /opt/calib/config/extrinsic.yaml
```

| Argument | Default | Purpose |
|----------|---------|---------|
| `--image_topic` | `/image_raw` | camera image topic |
| `--cloud_topic` | `/rslidar_points` | LiDAR PointCloud2 topic |
| `--target` | `/opt/calib/config/target.yaml` | board geometry + LiDAR-detect tuning |
| `--intrinsics` | `/opt/calib/config/intrinsics.yaml` | fisheye K/D from Stage 1 |
| `--out` | `/opt/calib/config/extrinsic.yaml` | where the solved transform is written |

## Stage 3 — Live colorized point cloud

```
./docker-calib/docker_run.sh colorize
```
RViz2 opens with the **Colored Cloud** display on `/colored_points` set to
RGB8 coloring. The display under it (**Raw Cloud**, off by default) shows
the uncoloured stream for comparison — toggle it on to debug.

The `colorize_node.py` publishes at the LiDAR rate (~10 Hz for AIRY).
Points behind the camera or projecting outside the image are dropped.

## Stage 4 — Timestamp sync + recording for FAST-LIVO2

FAST-LIVO2 fuses LiDAR + IMU + camera and needs all three on **one** timeline.

### The clock model

[config/lidar_config.yaml](config/lidar_config.yaml) sets `use_lidar_clock: true`,
so `/rslidar_points` **and** `/rslidar_imu_data` are stamped with the AIRY's
internal hardware clock (one hardware-synced clock for cloud + IMU). The two
helper nodes finish the job:

| Node | In → Out | What it does |
|------|----------|--------------|
| [scripts/imu_bridge.py](scripts/imu_bridge.py) | `/rslidar_imu_data` → `/rslidar_imu_data_fixed` | accel g → m/s²; **stamp unchanged** (stays on lidar clock) |
| [scripts/image_restamp.py](scripts/image_restamp.py) | `/image_raw` → `/image_raw_synced` | maps the host-clock camera onto the lidar clock |

The camera (`/image_raw`, gscam2 with `use_gst_timestamps:false`) is on the
**host** wall clock, a different epoch from the lidar clock. `image_restamp.py`
closes the gap in two parts:

- **Coarse offset (measured live):** it watches `/rslidar_imu_data` and tracks
  the *minimum* of `host_recv − lidar_stamp` over a 5 s window — the
  least-latency sample is the true clock offset. Robust to any epoch gap.
- **Fine shift (`cam_lidar_time_shift` from [config/time_sync.yaml](config/time_sync.yaml)):**
  the sub-frame camera capture-to-stamp lag, which only **Kalibr** can recover
  (FAST-Calib is spatial-only). Leave it `0.0` until Stage 4a.

Output: `new_lidar_stamp = host_stamp − coarse_offset + cam_lidar_time_shift`.

### Stage 4a — Temporal calibration with Kalibr

Recovers `cam_lidar_time_shift`. You need a **printed AprilGrid**
(download/print per [config/kalibr_aprilgrid.yaml](config/kalibr_aprilgrid.yaml)
— measure a tag edge and set `tagSize`; `tagSpacing` is a scale-invariant ratio).

1. **Bring up the full sensor stack** (the record scripts pre-flight these and
   abort if any is silent):
   ```
   ./docker-gst-camera/docker_run.sh        # terminal 1 → /image_raw
   ./docker-calib/docker_run.sh sensors     # terminal 2 → /rslidar_points + /rslidar_imu_data[_fixed]
   ```
2. **Record a wave-the-rig bag** (Ctrl-C after ~60–90 s exciting all 6 axes —
   3 rotations + 3 translations — with the AprilGrid in frame):
   ```
   ./docker-calib/docker_exec.sh bash /opt/calib/scripts/record_calib_bag.sh kalibr_run1
   ```
   This records `/image_raw_synced` (coarse-aligned) + `/rslidar_imu_data_fixed`,
   so Kalibr sees the small residual shift it can actually solve. The script
   verifies the bag is non-empty before exiting.
3. **Convert to a ROS 1 bag — INSIDE the container** (Kalibr reads ROS 1 only):
   ```
   ./docker-calib/docker_exec.sh bash /opt/calib/scripts/convert_bag.sh kalibr_run1
   ```
4. **Run Kalibr — ON THE HOST** (it launches the Kalibr Docker image, which the
   calib container can't do):
   ```
   bash docker-calib/scripts/run_kalibr.sh kalibr_run1
   # override the image: KALIBR_IMG=myrepo/kalibr:latest bash .../run_kalibr.sh kalibr_run1
   ```
   It prints `time_shift_cam_imu`.
5. **Paste the result** into `cam_lidar_time_shift` in
   [config/time_sync.yaml](config/time_sync.yaml). `image_restamp.py` reloads it
   on next launch — no rebuild.

### Stage 4b — Record the FAST-LIVO2 bag

With the sensor stack still up (Stage 4a step 1):
```
./docker-calib/docker_exec.sh bash /opt/calib/scripts/ros_bag_record.sh ezoffice
```
Records the three FAST-LIVO2 inputs, all on the lidar clock:
`/rslidar_points`, `/rslidar_imu_data_fixed`, `/image_raw_synced`. Then convert
and hand off to FAST-LIVO2:
```
./docker-calib/docker_exec.sh bash /opt/calib/scripts/convert_bag.sh ezoffice
mv bags/ezoffice.bag docker-livo2/bags/
```
[docker-livo2/config/rsairy.yaml](../docker-livo2/config/rsairy.yaml) already
reads `img_topic: /image_raw_synced` with `img_time_offset: 0.0` (the offset is
baked into the recorded stamps — don't set it again there or it double-counts).

## Files

| Path | Purpose |
|------|---------|
| [Dockerfile](Dockerfile) | `osrf/ros:humble-desktop` + usb_cam + rslidar_sdk + OpenCV 4.8 + Open3D |
| [config/target.yaml](config/target.yaml) | **EDIT THIS** — board geometry, ArUco IDs, LiDAR-detect tuning |
| [config/lidar_config.yaml](config/lidar_config.yaml) | AIRY ROS publisher config (RSAIRY, ports 6699/7788; `use_lidar_clock: true`) |
| [config/time_sync.yaml](config/time_sync.yaml) | **EDIT after Kalibr** — `cam_lidar_time_shift` read by image_restamp.py |
| [config/kalibr_aprilgrid.yaml](config/kalibr_aprilgrid.yaml) | **EDIT `tagSize`** — Kalibr AprilGrid target |
| [config/kalibr_camchain.yaml](config/kalibr_camchain.yaml) | Kalibr camera model (from intrinsics_ros.yaml) |
| [config/kalibr_imu.yaml](config/kalibr_imu.yaml) | Kalibr IMU noise model for the AIRY IMU |
| [scripts/imu_bridge.py](scripts/imu_bridge.py) | `/rslidar_imu_data` (g) → `/rslidar_imu_data_fixed` (m/s²), stamp kept |
| [scripts/image_restamp.py](scripts/image_restamp.py) | `/image_raw` (host clock) → `/image_raw_synced` (lidar clock) |
| [scripts/record_calib_bag.sh](scripts/record_calib_bag.sh) | Record camera+IMU bag for Kalibr (Stage 4a) |
| [scripts/run_kalibr.sh](scripts/run_kalibr.sh) | **Host-side** — runs Kalibr in Docker, prints `time_shift_cam_imu` |
| [scripts/ros_bag_record.sh](scripts/ros_bag_record.sh) | Record the 3 FAST-LIVO2 inputs (Stage 4b) |
| [scripts/convert_bag.sh](scripts/convert_bag.sh) | ROS 2 bag → ROS 1 `.bag` (Kalibr / FAST-LIVO2) |
| [config/usb_cam.yaml](config/usb_cam.yaml) | usb_cam parameters (device, resolution, pixel format) |
| [config/intrinsics.yaml](config/intrinsics.yaml) | **Generated** by Stage 1 |
| [config/extrinsic.yaml](config/extrinsic.yaml) | **Generated** by Stage 2 |
| [scripts/calibrate_intrinsic.py](scripts/calibrate_intrinsic.py) | Fisheye chessboard tool |
| [scripts/calibrate_extrinsic.py](scripts/calibrate_extrinsic.py) | ArUco PnP + plane/hole RANSAC + **multi-pose** global Kabsch solver |
| [scripts/colorize_node.py](scripts/colorize_node.py) | Live colorized PointCloud2 publisher |
| [rviz/colorize.rviz](rviz/colorize.rviz) | RViz2 preset (Colored Cloud + camera image) |

## Troubleshooting

- **`/dev/video0` not found:** the camera isn't accessible inside the
  container. Run `ls /dev/video*` on the host; if your camera is `video1`,
  edit [config/usb_cam.yaml](config/usb_cam.yaml). The run script already
  passes `--privileged -v /dev:/dev`.
- **`usb_cam` errors with "format mjpeg not supported":** change
  `pixel_format` in usb_cam.yaml to `yuyv2rgb` (most webcams do YUYV at
  ≤640×480 and MJPEG at higher resolutions).
- **Extrinsic tool says "camera: failed to recover board pose":** the ArUco
  IDs you set in `target.yaml` don't match what's on the board, OR fewer
  than 2 markers are visible. Print and verify.
- **"LiDAR: failed to extract 4 hole centers":** the plane-detect crop
  window isn't bracketing the board (`crop_xyz_min/max` in target.yaml),
  or the density threshold is wrong. Inspect the **lidar dbg** panel — the
  middle (interior mask) panel must look like a filled rectangle covering
  the board.
- **Colorized cloud is sparse or misaligned:** the extrinsic is the usual
  cause. Re-run Stage 2 and capture **more poses at varied depths and tilts**
  (10–20) until the global RMS is single-digit mm — a single-shot solve drifts
  at distances away from the board. `u`ndo any pose that spikes `worst-pose`.
  Also confirm the camera image is sharp (autofocus off, `autoexposure: true`
  is fine).
- **Record script aborts: "ABORT: not all inputs are live":** a required topic
  isn't publishing. Start the camera (`docker-gst-camera/docker_run.sh`) and the
  sensors (`docker-calib/docker_run.sh sensors`) **before** recording — the
  record scripts won't produce a silent 0-message bag.
- **`run_kalibr.sh`: "docker: command not found":** you ran it inside the calib
  container. Run it **on the host** (it launches the Kalibr Docker image); do
  the `convert_bag.sh` step inside the container.
- **Kalibr: too few AprilGrid detections:** re-record with more grid coverage,
  slower motion (less blur), and the board filling more of the frame. Confirm
  `tagSize`/`tagSpacing` in `kalibr_aprilgrid.yaml` match the printout.
- **NumPy ABI errors:** the Dockerfile pins `opencv-contrib-python==4.8.1.78`
  + `numpy<2` because ROS 2 Humble's `cv_bridge` is built against NumPy 1.x.
  Don't `pip install --upgrade opencv-contrib-python` inside the container.

## Method credit

Target-detection algorithm follows
[hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib). The 4-hole +
4-ArUco board gives 4 non-collinear 3D-3D point pairs per view — enough to solve
SE(3) from a single shot, but because those 4 points are coplanar a single shot
is poorly conditioned out-of-plane. This tool therefore accumulates several
board poses (varied depth + tilt) and solves one global Kabsch over all of them,
which is what drives the residual down to mm.
