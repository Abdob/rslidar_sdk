# Initial Analysis — LiDAR-Camera Tools

Two scripts for analyzing the captured LiDAR + camera data:

| Script | Purpose |
|--------|---------|
| `viewer.py` | Colorize point clouds with camera pixels; scroll through pairs |
| `align.py`  | Incrementally align all point clouds into a single merged map |

---

## viewer.py — Colorized Pair Viewer

Visualizes synchronized image + point cloud pairs by projecting LiDAR points
into the fisheye camera, sampling pixel colors, and rendering the result in two
windows:

| Window | Content |
|--------|---------|
| **Camera View** | Original (or undistorted) image with depth-colored LiDAR overlay |
| **3D Colored Point Cloud** | Interactive 3D view; each point is colored with the sampled camera pixel |

---

## Setup

### Option A — pip (virtualenv)

```bash
cd initial_analysis
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Option B — conda

```bash
conda create -n lidar-viewer python=3.10 -y
conda activate lidar-viewer
pip install -r requirements.txt
```

> **Minimum Python version:** 3.9  
> **Display requirement:** a desktop/X11 session — both open3d and cv2 open
> GUI windows.  If you are over SSH, forward the display with `ssh -X` or use
> X11 forwarding.

---

## Usage

```
python viewer.py [captures_dir] [--config config_dir]
```

| Argument | Default | Description |
|---|---|---|
| `captures_dir` | `../docker-calib/captures/20260606_215951` | Directory that contains `NNN_image.png` / `NNN_lidar.pcd` pairs |
| `--config DIR` | `../docker-calib/config` | Directory with `extrinsic.yaml` and `intrinsics.yaml` |

### Minimal example (defaults point to the captured session)

```bash
python viewer.py
```

### Explicit paths

```bash
python viewer.py /path/to/captures --config /path/to/config
```

---

## Controls

| Key | Action |
|-----|--------|
| **→ / D** | Next pair |
| **← / A** | Previous pair |
| **U** | Toggle between raw fisheye image and undistorted (pinhole) image |
| **R** | Reset 3D camera to fit the point cloud |
| **H** | Print help to terminal |
| **Q / Esc** | Quit |

> Navigation keys work in **both** the camera window and the 3D window.

---

## Calibration files expected

| File | Fields used |
|------|------------|
| `extrinsic.yaml` | `T_camera_lidar` — 4×4 transform; `p_camera = R @ p_lidar + t` |
| `intrinsics.yaml` | `K` (3×3), `D` (4 fisheye coefficients), `image_size` ([W, H]) |

---

## How viewer.py works

1. **Load** the `*_lidar.pcd` point cloud (x, y, z in LiDAR frame).  
2. **Transform** to camera frame using the extrinsic:  
   `p_cam = R @ p_lidar + t`
3. **Project** with `cv2.fisheye.projectPoints` (raw) or `cv2.projectPoints`
   with the rectified matrix (undistorted).  
4. **Sample** the image pixel at each projected coordinate → point color.  
5. **Overlay** the projection on the image, colored by depth (jet colormap).  
6. **Render** the colored cloud in the interactive open3d 3D window.

---

## align.py — Incremental Point Cloud Registration

Aligns `000_lidar.pcd → 001_lidar.pcd`, then `(000+001) → 002_lidar.pcd`,
and so on until all clouds are merged into a single map in the frame of the
first cloud.

### Usage

```bash
python align.py [captures_dir] [--voxel-size M] [--global-reg] [--no-viz]
```

| Argument | Default | Description |
|---|---|---|
| `captures_dir` | `../docker-calib/captures/20260606_215951` | Directory with `*_lidar.pcd` files |
| `--voxel-size M` | `0.05` | Voxel size in metres for downsampling and ICP |
| `--global-reg` | off | Add FPFH+RANSAC global registration before ICP (slower, more robust when clouds are far apart) |
| `--no-viz` | off | Skip the final open3d visualization window |

### Minimal example

```bash
python align.py
```

### Output files (written alongside the PCD files)

| File | Content |
|------|---------|
| `aligned_map.pcd` | Merged cloud in the frame of cloud 0 |
| `transforms.npy`  | `(N, 4, 4)` cumulative transforms — `transforms[i]` brings cloud `i` into world frame |

### Tuning tips

| Scenario | Recommendation |
|---|---|
| Clouds overlap well, similar pose | Default settings (ICP-only, 0.05 m voxel) |
| Clouds are far apart / rotated | Add `--global-reg` |
| Scene is fine-grained (e.g. small checkerboard) | Decrease `--voxel-size 0.02` |
| Low fitness warning | Try `--global-reg` or reduce `--voxel-size` |

The fitness metric is the fraction of points with a correspondence; values
above ~0.5 indicate a good alignment. Values below 0.3 trigger a warning.

### How it works

1. Cloud 0 is the world-frame reference (transform = identity).
2. For each subsequent cloud `i`:
   - Downsample source and current accumulated map.
   - Estimate normals on both.
   - *(optional)* FPFH+RANSAC global registration for a coarse initial guess.
   - Three-scale point-to-plane ICP: voxel × 4 → × 2 → × 1.
   - Transform the full-resolution cloud with the result and merge into the map.
3. The accumulated map is voxel-filtered after each merge to keep ICP fast.
4. Final merged cloud is saved as `aligned_map.pcd`; each cloud is also
   shown in the 3D viewer colored by index (rainbow cycling).
