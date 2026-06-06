# Initial Analysis — LiDAR-Camera Colorized Viewer

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

## How it works

1. **Load** the `*_lidar.pcd` point cloud (x, y, z in LiDAR frame).  
2. **Transform** to camera frame using the extrinsic:  
   `p_cam = R @ p_lidar + t`
3. **Project** with `cv2.fisheye.projectPoints` (raw) or `cv2.projectPoints`
   with the rectified matrix (undistorted).  
4. **Sample** the image pixel at each projected coordinate → point color.  
5. **Overlay** the projection on the image, colored by depth (jet colormap).  
6. **Render** the colored cloud in the interactive open3d 3D window.
