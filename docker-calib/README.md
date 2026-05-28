# docker-calib — Camera × LiDAR calibration + colorized point clouds

End-to-end pipeline for a RoboSense AIRY + a 180° fisheye USB camera mounted
to the same rig:

1. **Fisheye intrinsic calibration** with a checkerboard.
2. **Camera ↔ LiDAR extrinsic calibration** using a FAST-Calib-style board
   (4 ArUco markers + 4 circular holes).
3. **Live colorized point cloud** publisher that combines the two — RViz2
   shows the AIRY's geometry with per-point colors sampled from the camera.

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

Procedure:
1. Hold the board flat, ~1.5–2.5 m from the LiDAR, fully visible to both
   sensors. Wait a few seconds for cloud frames to accumulate.
2. Press `c`. The tool:
   - Detects ArUco markers, solves fisheye PnP for the board pose, computes
     hole centers in the **camera** frame.
   - Crops cloud, RANSACs the plane, finds 4 low-density regions, computes
     hole centers in the **LiDAR** frame.
   - Brute-forces all 24 permutations of LiDAR-to-camera hole pairings,
     keeps the one with the smallest Kabsch residual.
3. Inspect:
   - The **camera** window shows ArUco overlays plus green circles where
     the solver thinks the holes project — they should land on the actual
     holes.
   - The **lidar dbg** window shows three panels: density, board-interior
     mask, hole mask. Red crosses are the detected hole centers; they
     should sit cleanly in 4 dark spots.
   - RMS should land under ~10 mm for a well-calibrated rig.
4. Press `s` to save. Writes `config/extrinsic.yaml`.

Move the board to 2–3 different positions and re-solve to check consistency.
If RMS jumps around or holes get detected in wrong places, re-tune
`crop_xyz_*`, `plane_ransac_distance`, or `hole_density_threshold` in
[config/target.yaml](config/target.yaml).

## Stage 3 — Live colorized point cloud

```
./docker-calib/docker_run.sh colorize
```
RViz2 opens with the **Colored Cloud** display on `/colored_points` set to
RGB8 coloring. The display under it (**Raw Cloud**, off by default) shows
the uncoloured stream for comparison — toggle it on to debug.

The `colorize_node.py` publishes at the LiDAR rate (~10 Hz for AIRY).
Points behind the camera or projecting outside the image are dropped.

## Files

| Path | Purpose |
|------|---------|
| [Dockerfile](Dockerfile) | `osrf/ros:humble-desktop` + usb_cam + rslidar_sdk + OpenCV 4.8 + Open3D |
| [config/target.yaml](config/target.yaml) | **EDIT THIS** — board geometry, ArUco IDs, LiDAR-detect tuning |
| [config/lidar_config.yaml](config/lidar_config.yaml) | AIRY ROS publisher config (RSAIRY, ports 6699/7788) |
| [config/usb_cam.yaml](config/usb_cam.yaml) | usb_cam parameters (device, resolution, pixel format) |
| [config/intrinsics.yaml](config/intrinsics.yaml) | **Generated** by Stage 1 |
| [config/extrinsic.yaml](config/extrinsic.yaml) | **Generated** by Stage 2 |
| [scripts/calibrate_intrinsic.py](scripts/calibrate_intrinsic.py) | Fisheye chessboard tool |
| [scripts/calibrate_extrinsic.py](scripts/calibrate_extrinsic.py) | ArUco PnP + plane/hole RANSAC + Kabsch solver |
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
- **Colorized cloud is sparse or misaligned:** re-run the extrinsic stage
  with the board at a different distance/angle and check RMS. Also confirm
  the camera image is sharp (autofocus off, `autoexposure: true` is fine).
- **NumPy ABI errors:** the Dockerfile pins `opencv-contrib-python==4.8.1.78`
  + `numpy<2` because ROS 2 Humble's `cv_bridge` is built against NumPy 1.x.
  Don't `pip install --upgrade opencv-contrib-python` inside the container.

## Method credit

Target-detection algorithm follows
[hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib). The 4-hole +
4-ArUco board is the minimum geometric configuration that fully constrains
SE(3) in a single shot — 4 non-collinear 3D-3D point pairs.
