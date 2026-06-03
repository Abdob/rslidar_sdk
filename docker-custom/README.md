# docker-custom — sensor timing-sync experiments

A sandbox for studying the **clock offsets** between the AIRY LiDAR, its IMU,
and the USB camera by aligning **motion reversals**. Translate the rig
back-and-forth in one dimension in front of a checkerboard on a wall, reduce
each sensor to one scalar, and line up the turning points.

| Sensor | 1D signal | Reversal |
|--------|-----------|----------|
| LiDAR `/rslidar_points` | distance to wall (m), RANSAC plane fit | distance peak/trough |
| Camera `/image_raw` | checkerboard apparent size (or center) | size/center peak/trough |
| IMU `/rslidar_imu_data_fixed` | linear accel on motion axis (m/s²) | accel extremum |

> **Physics:** distance & checkerboard-u are *position*; IMU is *acceleration*
> (2nd derivative). For back-and-forth motion they are ~anti-phase, so position
> **extrema** align with acceleration **extrema** (not accel zero-crossings).
> `plot` overlays the three (each min-max normalized) before and after applying
> the measured offsets, so you see the same physical reversal line up.

## What it builds

- Base: **`rslidar-airy-calib`** (inherits ROS 2 Humble, rslidar_sdk, numpy/opencv/
  open3d/scipy). **Build that first:** `./docker-calib/docker_build.sh`.
- Adds the **CUDA 13.0 toolkit + nvcc** (GB10 is sm_121 → needs CUDA 13.x) and
  matplotlib. The driver is provided at runtime by `--gpus all`.

## Build

```
./docker-calib/docker_build.sh      # if not already built (base image)
./docker-custom/docker_build.sh
```

## Use

The camera + LiDAR/IMU stack must be publishing first (two terminals):

```
./docker-gst-camera/docker_run.sh        # /image_raw, /camera_info
./docker-calib/docker_run.sh sensors     # /rslidar_points + /rslidar_imu_data[_fixed]
```

Then, in this container:

```
./docker-custom/docker_run.sh record  run1   # translate the rig ~30-60 s, Ctrl-C
./docker-custom/docker_run.sh extract run1   # bag -> bags/run1_signals.{npz,csv}
./docker-custom/docker_run.sh axes    run1   # inspect all channels, pick signals
./docker-custom/docker_run.sh measure run1   # plot the 3 physical measurements
./docker-custom/docker_run.sh plot    run1   # reversal sync + offset table
```

`docker_run.sh` with no args drops you in a shell. `record`/`extract`/`axes`/
`measure`/`plot` are aliases; `-- <cmd>` runs anything. Add `--show` to a plot
command for an interactive window (needs an X display).

### Plot the raw measurements — `measure`

```
./docker-custom/docker_run.sh measure run1     # -> bags/run1_measurements.png
```
Three stacked subplots, **real units, on the common clock** (see below):

1. **LiDAR distance to the front wall** (m) — RANSAC plane fit.
2. **Checkerboard width** (px) — horizontal span of the inner corners; grows as
   the board nears the camera (≈ inverse of the wall distance).
3. **Raw accelerations** ax, ay, az (m/s²) — all three IMU channels overlaid
   (the axis carrying gravity sits near ±9.8).

Use `axes run1` first if you're unsure which channel moves most: it prints each
sensor's per-channel std and names the dominant one, so you can set
`checkerboard.signal` and `imu.axis` in [config/sync.yaml](config/sync.yaml).

### Synchronize — `plot`

```
./docker-custom/docker_run.sh plot run1        # -> bags/run1_sync.png + offset table
```
What it does:
1. Reduces each sensor to its chosen 1D signal (config) and **smooths by time**
   (so the 200 Hz IMU is filtered more than the 10 Hz LiDAR).
2. Detects **motion reversals** = smoothed local extrema whose prominence
   exceeds `min_prominence_frac` of the signal amplitude (rejects IMU noise).
3. Pairs the reversals across sensors **one-to-one** (mutual nearest neighbour:
   two reversals pair only if each is the other's closest), so every turning
   point is used at most once and extra/noisy reversals stay unpaired instead of
   corrupting the estimate. Prints the constant offset + consistency (std) +
   paired count `n` (always <= the reversal count):

   ```
   reversal offsets on common clock  (offset = other - ref; std = consistency):
     pair                 offset [s]    std [s]    n
     lidar-imu                0.0009     0.0979   13
     camera-imu               0.0352     0.1840   14
     camera-lidar             0.0771     0.0410   13
   ```

   `offset = other − ref` is how much **later** `other`'s stamp is than `ref`'s
   for the same physical turning point. **All on the common clock**, so these are
   the real sub-second lags. A small `std` (and `n` ≈ your number of turnarounds)
   means a trustworthy estimate. Here the camera stamps land ~0.05–0.08 s after
   the LiDAR/IMU; LiDAR and IMU agree to ~1 ms.
4. Saves `bags/<name>_sync.png` with **two panels**, each overlaying all three
   signals **min-max normalized** (range taken from the **middle half** of the
   scan so the stationary start/end don't skew it; the true min..max in physical
   units is in the legend): the **top** panel is BEFORE sync, the **bottom** is
   AFTER shifting the camera and IMU onto the LiDAR by their measured offsets.

To get clean estimates: do **several distinct turnarounds** (≥6), keep the board
fully in frame and the wall inside the LiDAR ROI, and **don't move perfectly
periodically** (irregular timing makes the reversal pairing unambiguous).

## Common clock

LiDAR and IMU are already on the **AIRY hardware clock** (`use_lidar_clock:true`).
The camera (`/image_raw`) is on the **host Unix clock**, ~1.78e9 s ahead — that
gross constant is the **epoch**. `extract_signals.py` measures it as the
recording-start gap (`cam_t0_raw − lidar_t0_raw`; the preflight guarantees both
sensors are live at record start, so their first stamps are the same real
instant to within a frame) and **subtracts it from every camera stamp**. From
then on all three sensors share the common (lidar) clock, so the plots use one
timeline and the offset table reads in real sub-second residuals. The removed
epoch is printed by `extract`/`measure`/`plot` and stored in the npz
(`cam_epoch`, `cam_t0_raw`) for reference.

## Configure — [config/sync.yaml](config/sync.yaml)

- **checkerboard.cols/rows**: *inner corners*, not squares (a 10×7-square board
  is 9×6). `signal: scale` for toward/away motion (board grows/shrinks),
  or `u`/`v` for side-to-side motion.
- **lidar.crop_xyz_min/max**: forward ROI box around the wall (AIRY frame:
  +Z forward, +X down, +Y lateral). Widen to your wall distance / sweep range.
- **imu.axis**: `auto` (max-variance channel) or `x|y|z`.

Everything is bind-mounted, so edit YAML/scripts and re-run without rebuilding.

## CUDA sanity check

```
./docker-custom/docker_run.sh -- bash -lc \
  'nvcc --version && printf "%s" "__global__ void k(){} int main(){k<<<1,1>>>();return cudaDeviceSynchronize();}" > /tmp/t.cu && nvcc -arch=sm_121 /tmp/t.cu -o /tmp/t && /tmp/t && echo CUDA_OK'
```

## Troubleshooting

- **`checkerboard never detected`**: `cols`/`rows` must be inner corners; ensure
  the whole board is in frame and reasonably sharp. Try `undistort: false`.
- **lidar has few/no points**: widen `crop_xyz_*` and/or lower
  `plane_min_inliers`; confirm the wall is inside the ROI (Z range).
- **base image missing**: `docker_build.sh` aborts if `rslidar-airy-calib`
  isn't built — run `./docker-calib/docker_build.sh` first.
- **camera↔lidar offset ~1.78e9 s**: expected — `/image_raw` is on the host
  clock, LiDAR/IMU on the AIRY clock. The reversal alignment recovers the
  residual after that gross epoch gap.
