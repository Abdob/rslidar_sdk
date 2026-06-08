# Initial Analysis — LiDAR-Camera Tools

Scripts for analyzing captured LiDAR + camera data from a calibration session.

| Script | Purpose |
|--------|---------|
| `viewer.py` | Colorize point clouds with camera pixels; scroll through capture pairs |
| `align.py` | Incrementally align all point clouds into a single merged map |
| `view_map.py` | Open a single PCD file in an interactive 3D viewer |
| `compare_maps.py` | Overlay two PCD files for visual comparison (blue vs green) |
| `filter_roi.py` | Crop a point cloud to a cuboid region of interest |
| `analyze_room.py` | Segment room surfaces, measure distances, filter by depth, generate mesh |
| `mesh_surfaces.py` | Piecewise-planar mesh with local plane fits per 20 cm tile; edge snapping |

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
> **Display requirement:** a desktop / X11 session — all scripts open GUI windows.
> Over SSH use `ssh -X` for X11 forwarding.

---

## viewer.py — Colorized Pair Viewer

Visualizes synchronized image + point cloud pairs by projecting LiDAR points
into the fisheye camera, sampling pixel colors, and rendering the result in two
windows simultaneously.

| Window | Content |
|--------|---------|
| **Camera View** | Image with depth-colored LiDAR overlay (jet colormap by range) |
| **3D Colored Point Cloud** | Interactive 3D view; each point colored with its sampled image pixel |

### Usage

```bash
python viewer.py [captures_dir] [--config config_dir]
```

| Argument | Default | Description |
|---|---|---|
| `captures_dir` | `../docker-calib/captures/20260606_215951` | Directory with `NNN_image.png` / `NNN_lidar.pcd` pairs |
| `--config DIR` | `../docker-calib/config` | Directory with `extrinsic.yaml` and `intrinsics.yaml` |

```bash
python viewer.py          # use defaults
python viewer.py /path/to/captures --config /path/to/config
```

### Controls

| Key | Action |
|-----|--------|
| **→ / D** | Next pair |
| **← / A** | Previous pair |
| **U** | Toggle raw fisheye ↔ undistorted (pinhole) image |
| **R** | Reset 3D camera |
| **H** | Print help |
| **Q / Esc** | Quit |

Keys work in **both** the camera window and the 3D window.

### Calibration files

| File | Fields used |
|------|-------------|
| `extrinsic.yaml` | `T_camera_lidar` — 4×4; `p_camera = R @ p_lidar + t` |
| `intrinsics.yaml` | `K` (3×3), `D` (4 fisheye coefficients), `image_size` ([W, H]) |

### How it works

1. Load `*_lidar.pcd` (x, y, z in LiDAR frame).
2. Transform to camera frame: `p_cam = R @ p_lidar + t`.
3. Project with `cv2.fisheye.projectPoints` (raw) or `cv2.projectPoints` + rectified matrix (undistorted).
4. Sample the image pixel at each projected coordinate → point color.
5. Overlay the projection on the image colored by depth.
6. Render the colored cloud in the open3d 3D window.

---

## align.py — Incremental Point Cloud Registration

Aligns `000 → 001`, then `(000+001) → 002`, and so on, merging all clouds into
a single map in the frame of the first cloud. Color information is **never used**
for alignment — ICP is purely geometry-based.

### Usage

```bash
python align.py [captures_dir] [--voxel-size M] [--global-reg] [--colorize] [--no-viz]
```

| Argument | Default | Description |
|---|---|---|
| `captures_dir` | `../docker-calib/captures/20260606_215951` | Directory with `*_lidar.pcd` files |
| `--config DIR` | `../docker-calib/config` | Calibration directory (required with `--colorize`) |
| `--voxel-size M` | `0.05` | Voxel size in metres for downsampling and ICP |
| `--global-reg` | off | FPFH+RANSAC global registration before ICP (slower, more robust for large pose differences) |
| `--colorize` | off | After alignment, project each cloud into its paired camera image and save a photo-realistic colored map |
| `--no-viz` | off | Skip the final viewer |

```bash
python align.py                         # geometry only
python align.py --colorize              # + camera colors
python align.py --global-reg --colorize # robust alignment + colors
python align.py --no-viz                # headless / save only
```

### Output files

| File | Content |
|------|---------|
| `aligned_map.pcd` | Geometry-only merged cloud in cloud-0's frame |
| `aligned_map_colored.pcd` | Camera-colorized merged cloud (`--colorize` only) |
| `transforms.npy` | `(N, 4, 4)` cumulative transforms — `transforms[i]` brings cloud `i` into world frame |

### Tuning tips

| Scenario | Recommendation |
|---|---|
| Clouds overlap well, similar pose | Default (ICP-only, 0.05 m voxel) |
| Clouds are far apart or rotated | Add `--global-reg` |
| Fine-grained scene | Decrease `--voxel-size 0.02` |
| Low fitness warning (<0.3) | Try `--global-reg` or smaller `--voxel-size` |

### How it works

1. Cloud 0 is the world-frame reference (identity transform).
2. For each subsequent cloud `i`:
   - Voxel-downsample source and accumulated map; estimate normals.
   - *(optional)* FPFH+RANSAC global registration for a coarse initial pose.
   - Three-scale point-to-plane ICP: voxel × 4 → × 2 → × 1.
   - Transform the full-resolution cloud and merge into the map.
3. The map is voxel-filtered after each merge to keep ICP fast.
4. *(optional `--colorize`)* A second pass re-loads each original cloud, projects it into its paired image using the extrinsic, and samples pixel colors. Points outside the camera FOV are dropped.

---

## view_map.py — Single Map Viewer

Opens any PCD file in an interactive open3d window.

```bash
python view_map.py                                        # aligned_map_colored.pcd (default)
python view_map.py path/to/cloud.pcd                     # explicit file
python view_map.py path/to/cloud.pcd --height            # override colors with jet Z-height map
```

| Argument | Default | Description |
|---|---|---|
| `pcd_file` | `aligned_map_colored.pcd` | PCD file to view |
| `--height` | off | Color by Z-height (jet) instead of stored colors |

---

## compare_maps.py — Two-Map Comparison

Overlays two PCD files in one window. The second cloud (B) is transformed into
the first cloud's (A) coordinate frame using the last entry of `transforms.npy`
(which maps cloud-N's frame → cloud-0's frame).

```bash
python compare_maps.py                        # forward vs reverse, blue vs green
python compare_maps.py --use-colors           # use each file's stored camera colors
python compare_maps.py file_a.pcd file_b.pcd
```

| Argument | Default | Description |
|---|---|---|
| `file_a` | `aligned_map_colored.pcd` (forward) | Shown in **blue**; reference frame |
| `file_b` | `../../aligned_map_colored.pcd` (reverse) | Shown in **green**; transformed into A's frame |
| `--transforms NPY` | `transforms.npy` alongside `file_a` | Cumulative transforms from the forward alignment |
| `--use-colors` | off | Show stored camera colors instead of solid blue/green |

**Why the transform is needed:** the forward map lives in cloud-0's frame; the
reverse map lives in cloud-5's frame. `transforms[-1]` is the bridge between them.

---

## filter_roi.py — Cuboid ROI Crop

Crops a point cloud to a rectangular region of interest. Two modes:

**Interactive** — draw a polygon in the viewer, press C to crop:
```bash
python filter_roi.py
```
Controls: `K` lock view → left-click polygon vertices → `C` crop → `Q` quit and save.

**Bounds** — axis-aligned crop with explicit coordinates (repeatable):
```bash
python filter_roi.py \
  --xmin -1.23 --xmax 1.75 \
  --ymin -0.42 --ymax 1.41 \
  --zmin -0.05 --zmax 2.44
```

After an interactive crop the terminal prints the exact `--xmin/xmax/...` values
so you can paste them into a bounds call for future runs.

| Argument | Default | Description |
|---|---|---|
| `input` | `aligned_map_colored.pcd` | PCD to filter |
| `-o / --output` | `filtered_roi.pcd` | Output path |
| `--no-viz` | off | Skip the preview window (bounds mode only) |

**Room reference:** 117 in × 6 ft × 8 ft = 2.972 m × 1.829 m × 2.438 m

---

## analyze_room.py — Room Surface Analysis

Segments a filtered point cloud into up to five planar surfaces (walls, floor,
ceiling), measures distances from any surface, filters by depth, and generates
a flat mesh.

### Typical workflow

```bash
# 1. Identify the five surfaces
python analyze_room.py segment

# 2. Look at the printed normal directions + colored viewer,
#    then rename the planes to semantic labels
python analyze_room.py relabel plane_0=front_wall plane_1=floor \
                                plane_2=ceiling plane_3=left_wall plane_4=right_wall

# 3. Confirm: colorize all points by distance from the front wall
python analyze_room.py measure front_wall

# 4. Keep only points within 0–1.83 m (6 ft) of the front wall
python analyze_room.py filter front_wall --min 0.0 --max 1.83

# 5. Generate a flat mesh for all five surfaces
python analyze_room.py mesh
```

All subcommands default to `filtered_roi.pcd` (falling back to
`aligned_map_colored.pcd` if not found).

---

### segment

Fits up to 5 planes via iterative RANSAC, then **votes every point** to its
nearest plane (including points the RANSAC rejected as outliers, e.g. slightly
bent wall edges). Planes are named `plane_0`…`plane_4`, largest surface first.

```bash
python analyze_room.py segment [PCD] [--dist-thresh M] [--assign-thresh M]
```

| Argument | Default | Description |
|---|---|---|
| `--dist-thresh M` | `0.02` | RANSAC inlier distance in metres |
| `--assign-thresh M` | ∞ | Points farther than this from all planes become *unassigned*; useful to exclude equipment/clutter |

**Terminal output example:**

```
  plane_0       28,413     28,901    +X    [+0.998 -0.002 +0.001]
  plane_1       24,107     25,312    -Z    [-0.001 +0.003 -0.999]
  plane_2       21,880     22,654    -X    [-0.997 +0.001 +0.002]
  plane_3       19,204     20,115    +Y    [+0.002 +0.999 -0.003]
  plane_4       16,532     17,008    -Y    [-0.003 -0.998 +0.001]
```

The **Dom. axis** column (`+X`, `-Z`, …) shows which direction the plane's
inward normal points — use this alongside the viewer colors to identify each
surface.

**Viewer colors (before relabelling):**

| Label | Color |
|-------|-------|
| `plane_0` | blue |
| `plane_1` | red |
| `plane_2` | green |
| `plane_3` | yellow |
| `plane_4` | magenta |

Saves `segments.npz` alongside the input PCD.

---

### relabel

Renames planes after visual inspection. Re-opens the viewer to confirm.

```bash
python analyze_room.py relabel OLD=NEW [OLD=NEW …] [PCD]
```

```bash
python analyze_room.py relabel plane_0=front_wall plane_1=floor
```

Known semantic labels get distinct colors:

| Label | Color |
|-------|-------|
| `floor` | brown |
| `ceiling` | light gray |
| `front_wall` | orange |
| `left_wall` | blue |
| `right_wall` | red |

---

### measure

Colorizes all points by **signed distance** from a named plane. Positive =
inside the room (toward the LiDAR); negative = behind the plane.

```bash
python analyze_room.py measure <surface> [PCD]
```

```bash
python analyze_room.py measure front_wall
python analyze_room.py measure floor
```

---

### filter

Keeps only points within `[--min, --max]` metres of a named plane.

```bash
python analyze_room.py filter <surface> [PCD] [--min M] [--max M] [-o output.pcd]
```

```bash
python analyze_room.py filter front_wall --min 0.0 --max 1.83   # first 6 ft
python analyze_room.py filter floor      --min 0.0 --max 2.44   # up to 8 ft high
```

Saves `filtered_<surface>.pcd` by default.

---

### mesh

Builds a flat convex-hull mesh patch for each plane and saves a combined
`room_mesh.ply`. Each patch is colored with the surface's label color.

```bash
python analyze_room.py mesh [PCD] [--dist-thresh M] [-o output.ply]
```

| Argument | Default | Description |
|---|---|---|
| `--dist-thresh M` | `0.025` | Points within this distance of the plane are used for the hull |

The mesh uses a fan triangulation from the centroid of the convex hull of the
inlier points projected onto the fitted plane.

---

## mesh_surfaces.py — Piecewise-Planar Surface Mesh

Builds a dense, photo-realistic mesh from a point cloud that contains one or
more planar surfaces (walls, floor, ceiling).

### Algorithm

1. **Segment** — iterative RANSAC finds up to `--n-planes` planes; every point
   is voted to its nearest plane, so slightly bent edges are handled correctly.
2. **Local plane fit** — a regular grid (spacing `--tile-step`) is laid on each
   global plane. Each grid vertex collects all points inside a
   `--tile-size × --tile-size` window (overlapping) and fits a local plane by
   least squares. The vertex 3D position is the local plane evaluated at the
   grid point; its colour is the inverse-distance-weighted average of nearby
   point colours (or a per-section index colour if the PCD has no stored colours).
3. **Boundary snapping** — for every pair of adjacent sections the
   plane-plane intersection line is computed. Grid vertices within
   `--snap-thresh` of that line are projected onto it so adjacent section
   meshes meet edge-to-edge without gaps.
4. **Triangulate** — adjacent valid grid vertices are connected into quads
   (two triangles); quads where any edge exceeds `--max-edge` are discarded
   to avoid bridging across holes in the data.

### Usage

```bash
python mesh_surfaces.py [input.pcd] [options]
```

```bash
python mesh_surfaces.py                                       # default input
python mesh_surfaces.py my_cloud.pcd -o my_mesh.ply
python mesh_surfaces.py --tile-size 0.10 --tile-step 0.02    # finer mesh
python mesh_surfaces.py --n-planes 1 --no-snap               # single surface, no snapping
python mesh_surfaces.py --no-viz                             # headless / save only
```

| Argument | Default | Description |
|---|---|---|
| `input` | `filtered_ceiling.pcd` | Input point cloud |
| `-o / --output` | `<input_dir>/surfaces_mesh.ply` | Output mesh |
| `--tile-size M` | `0.20` | Side of the local fitting window in metres |
| `--tile-step M` | `0.05` | Grid vertex spacing in metres (smaller = denser mesh) |
| `--snap-thresh M` | `0.10` | Snap boundary vertices within this distance of an intersection line |
| `--max-edge M` | `0.30` | Discard triangles with any edge longer than this |
| `--n-planes N` | `5` | Max number of planes to detect |
| `--dist-thresh M` | `0.02` | RANSAC inlier distance in metres |
| `--no-snap` | off | Disable boundary snapping |
| `--no-viz` | off | Skip the viewer |

### Tuning tips

| Scenario | Recommendation |
|---|---|
| Jagged/chunky mesh | Decrease `--tile-step` (e.g. `0.02`) |
| Tiles with few points | Increase `--tile-size` (e.g. `0.30`) |
| Gaps not closing at edges | Increase `--snap-thresh` (e.g. `0.20`) |
| Long thin triangles at edges | Decrease `--snap-thresh` or `--max-edge` |
| Only one surface in the file | `--n-planes 1` |
| Noisy / bumpy source | Increase `--tile-size` to smooth more |

---

## Output file reference

| File | Created by | Content |
|------|-----------|---------|
| `aligned_map.pcd` | `align.py` | Geometry-only merged cloud |
| `aligned_map_colored.pcd` | `align.py --colorize` | Camera-colorized merged cloud |
| `transforms.npy` | `align.py` | `(N, 4, 4)` cumulative ICP transforms |
| `filtered_roi.pcd` | `filter_roi.py` | Cuboid-cropped cloud |
| `segments.npz` | `analyze_room.py segment` | Plane models + labels |
| `filtered_<surface>.pcd` | `analyze_room.py filter` | Depth-filtered cloud |
| `room_mesh.ply` | `analyze_room.py mesh` | Five-surface flat mesh |
| `surfaces_mesh.ply` | `mesh_surfaces.py` | Piecewise-planar dense mesh |
