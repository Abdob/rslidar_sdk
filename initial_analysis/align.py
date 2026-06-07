#!/usr/bin/env python3
"""
Incremental LiDAR Point Cloud Registration

Aligns PCD[0] → PCD[1], then (PCD[0]+PCD[1]) → PCD[2], all the way to PCD[N-1].
Each cloud is brought into the frame of the first cloud (world frame).

Alignment pipeline (geometry only — color is never used for ICP)
-----------------------------------------------------------------
  1. Voxel-downsample source + current map.
  2. Estimate normals on both.
  3. [optional] FPFH + RANSAC global registration as initial guess.
  4. Multi-scale point-to-plane ICP refinement (coarse → fine).
  5. Transform full-resolution cloud and append to map.

Optional colorization (--colorize)
-----------------------------------
  After all transforms are known, each original cloud is re-loaded and its
  points are projected into the paired camera image using the extrinsic
  calibration. The sampled pixel colors are carried into world frame together
  with the points, producing a photo-realistic merged cloud. Points that fall
  outside the camera FOV are colored by depth (jet colormap) as a fallback.

Outputs (written to captures_dir)
----------------------------------
  aligned_map.pcd         geometry-only merged cloud
  aligned_map_colored.pcd camera-colorized merged cloud  (with --colorize)
  transforms.npy          (N, 4, 4) cumulative transforms for each cloud
"""

from __future__ import annotations
import argparse
import copy
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d


# ── geometry helpers ───────────────────────────────────────────────────────

def _ds(pcd: o3d.geometry.PointCloud, vsize: float) -> o3d.geometry.PointCloud:
    return pcd.voxel_down_sample(vsize)


def _normals(pcd: o3d.geometry.PointCloud, radius: float, max_nn: int = 30) -> None:
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )
    pcd.orient_normals_towards_camera_location()


def _fpfh(pcd: o3d.geometry.PointCloud, radius: float) -> o3d.pipelines.registration.Feature:
    return o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100),
    )


# ── global registration ────────────────────────────────────────────────────

def global_registration(
    src: o3d.geometry.PointCloud,
    dst: o3d.geometry.PointCloud,
    voxel_size: float,
) -> np.ndarray:
    """FPFH + RANSAC global registration.  Returns 4×4 initial transform."""
    feat_r = voxel_size * 5.0
    src_f  = _fpfh(src, feat_r)
    dst_f  = _fpfh(dst, feat_r)

    dist = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src, dst, src_f, dst_f,
        mutual_filter=True,
        max_correspondence_distance=dist,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4_000_000, 500),
    )
    return result.transformation


# ── multi-scale ICP ────────────────────────────────────────────────────────

def icp_multiscale(
    src_full: o3d.geometry.PointCloud,
    dst_full: o3d.geometry.PointCloud,
    init_T: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, float, float]:
    """Three-scale point-to-plane ICP (coarse → fine).  Returns (T, fitness, rmse)."""
    scales    = [4.0, 2.0, 1.0]
    max_iters = [50,  30,  100]
    T = init_T.copy()

    for scale, n_iter in zip(scales, max_iters):
        vs  = voxel_size * scale
        src = _ds(src_full, vs)
        dst = _ds(dst_full, vs)
        nr  = vs * 2.0
        _normals(src, nr)
        _normals(dst, nr)

        result = o3d.pipelines.registration.registration_icp(
            src, dst,
            max_correspondence_distance=vs * 1.5,
            init=T,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=n_iter,
                relative_fitness=1e-6,
                relative_rmse=1e-6,
            ),
        )
        T = result.transformation

    return T, result.fitness, result.inlier_rmse


# ── pair alignment ─────────────────────────────────────────────────────────

def align_to_map(
    src: o3d.geometry.PointCloud,
    map_pcd: o3d.geometry.PointCloud,
    voxel_size: float,
    use_global: bool,
    init_T: np.ndarray,
) -> tuple[np.ndarray, float, float]:
    """Align src into map_pcd's frame.  Returns (T, fitness, rmse)."""
    src_d = _ds(src,     voxel_size)
    dst_d = _ds(map_pcd, voxel_size)

    if use_global:
        nr = voxel_size * 2.0
        _normals(src_d, nr)
        _normals(dst_d, nr)
        init_T = global_registration(src_d, dst_d, voxel_size)
        print(f"    global-reg init:\n{init_T}")

    T, fitness, rmse = icp_multiscale(src, map_pcd, init_T, voxel_size)
    return T, fitness, rmse


# ── colorization ───────────────────────────────────────────────────────────

def load_calibration(config_dir: Path) -> dict:
    """Load fisheye intrinsics + extrinsic.  Returns calibration dict."""
    import cv2
    import yaml

    with open(config_dir / "extrinsic.yaml") as f:
        ext = yaml.safe_load(f)
    T = np.array(ext["T_camera_lidar"], dtype=np.float64)
    R, t = T[:3, :3], T[:3, 3]

    with open(config_dir / "intrinsics.yaml") as f:
        intr = yaml.safe_load(f)
    K  = np.array(intr["K"], dtype=np.float64)
    D  = np.array(intr["D"], dtype=np.float64).reshape(4, 1)
    sz = tuple(intr["image_size"])          # (width, height)

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, sz, np.eye(3, dtype=np.float64), balance=0.5
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3, dtype=np.float64), new_K, sz, cv2.CV_16SC2
    )
    return dict(R=R, t=t, K=K, D=D, new_K=new_K, map1=map1, map2=map2, sz=sz)


def _jet_rgb(values: np.ndarray) -> np.ndarray:
    """Jet colormap; (N,) float → (N, 3) float64 RGB in [0, 1]."""
    lo, hi = np.percentile(values, [5, 95])
    v = np.clip((values - lo) / (hi - lo + 1e-6), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4*v - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4*v - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4*v - 1), 0, 1)
    return np.column_stack([r, g, b])


def colorize_cloud(
    pts_xyz: np.ndarray,
    img_bgr,                    # cv2 BGR image
    calib: dict,
) -> np.ndarray:
    """
    Project pts_xyz (lidar frame, (N,3)) into the camera image and sample
    pixel colors.  Points outside the FOV fall back to jet depth coloring.

    Returns (N, 3) float64 RGB in [0, 1].
    """
    import cv2

    R, t = calib["R"], calib["t"]
    K, D = calib["K"], calib["D"]
    W, H = calib["sz"]

    # Start with jet depth coloring for every point (fallback)
    pts_cam_all = (R @ pts_xyz.T).T + t        # (N, 3) in camera frame
    colors = _jet_rgb(np.linalg.norm(pts_cam_all, axis=1))

    # Keep only points in front of the camera
    front = pts_cam_all[:, 2] > 0.1
    pts_cam_f = pts_cam_all[front]             # (M, 3)
    if len(pts_cam_f) == 0:
        return colors

    # Fisheye projection
    rvec = np.zeros((3, 1), np.float64)
    tvec = np.zeros((3, 1), np.float64)
    uv, _ = cv2.fisheye.projectPoints(
        pts_cam_f.reshape(-1, 1, 3), rvec, tvec, K, D
    )
    uv = uv.reshape(-1, 2)

    ui = np.round(uv[:, 0]).astype(np.int32)
    vi = np.round(uv[:, 1]).astype(np.int32)
    in_bounds = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)

    # Sample image pixels → RGB 0-1
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    sampled = img_rgb[vi[in_bounds], ui[in_bounds]].astype(np.float64) / 255.0

    # Write camera colors over the fallback for visible points
    front_idx  = np.where(front)[0]
    visible_idx = front_idx[in_bounds]
    colors[visible_idx] = sampled

    return colors


def build_colored_map(
    pcd_paths: list[Path],
    transforms: list[np.ndarray],
    calib: dict,
    voxel_size: float,
) -> o3d.geometry.PointCloud:
    """
    Re-load each original cloud, colorize from its paired image using the
    extrinsic, then transform to world frame and merge.
    """
    import cv2

    print("\nColorizing merged map from camera images …")
    merged = o3d.geometry.PointCloud()

    for i, (pcd_path, T) in enumerate(zip(pcd_paths, transforms)):
        img_path = pcd_path.parent / pcd_path.name.replace("_lidar.pcd", "_image.png")
        if not img_path.exists():
            print(f"  [{i}] WARNING: image not found at {img_path}, skipping colorization")
            raw = o3d.io.read_point_cloud(str(pcd_path))
            raw.transform(T)
            merged += raw
            continue

        img_bgr = cv2.imread(str(img_path))
        raw     = o3d.io.read_point_cloud(str(pcd_path))
        pts_xyz = np.asarray(raw.points, dtype=np.float64)

        colors = colorize_cloud(pts_xyz, img_bgr, calib)        # lidar frame colors

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts_xyz)
        cloud.colors = o3d.utility.Vector3dVector(colors)
        cloud.transform(T)                                       # → world frame

        merged += cloud
        n_vis = int(np.any(colors != colors[0:1], axis=1).sum()) if len(colors) > 1 else 0
        print(f"  [{i}] {pcd_path.name}  {len(pts_xyz):,} pts")

    merged = merged.voxel_down_sample(voxel_size * 0.25)
    return merged


# ── data ───────────────────────────────────────────────────────────────────

def find_pcds(captures_dir: Path) -> list[Path]:
    pcds = sorted(captures_dir.glob("*_lidar.pcd"))
    if not pcds:
        sys.exit(f"ERROR: no *_lidar.pcd files found in {captures_dir}")
    return pcds


def load(path: Path) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(str(path))
    if not pcd.has_points():
        sys.exit(f"ERROR: could not load points from {path}")
    return pcd


def color_by_index(pcd: o3d.geometry.PointCloud, idx: int, total: int) -> None:
    """Paint all points with a hue cycling through the rainbow."""
    hue = idx / max(total - 1, 1)
    h6  = hue * 6.0
    i   = int(h6) % 6
    f   = h6 - int(h6)
    lut = [
        (1, f,   0), (1-f, 1, 0), (0, 1,   f),
        (0, 1-f, 1), (f,   0, 1), (1, 0, 1-f),
    ]
    r, g, b = lut[i]
    n = len(pcd.points)
    pcd.colors = o3d.utility.Vector3dVector(
        np.tile([r, g, b], (n, 1)).astype(np.float64)
    )


# ── main ───────────────────────────────────────────────────────────────────

def run(
    captures_dir: Path,
    config_dir: Path,
    voxel_size: float,
    use_global: bool,
    colorize: bool,
    visualize: bool,
) -> None:
    pcd_paths = find_pcds(captures_dir)
    N         = len(pcd_paths)
    print(f"Found {N} point clouds in {captures_dir}")
    print(f"Voxel size : {voxel_size} m")
    print(f"Global reg : {'yes (FPFH+RANSAC)' if use_global else 'no (ICP only)'}")
    print(f"Colorize   : {'yes' if colorize else 'no'}")
    print()

    # ── incremental alignment (geometry only) ──────────────────────────────
    map_pcd = load(pcd_paths[0])
    print(f"[0/{N-1}] {pcd_paths[0].name}  →  reference  ({len(map_pcd.points):,} pts)")

    transforms: list[np.ndarray]          = [np.eye(4)]
    clouds_world: list[o3d.geometry.PointCloud] = [copy.deepcopy(map_pcd)]

    for i in range(1, N):
        src = load(pcd_paths[i])
        t0  = time.time()

        init_T = transforms[-1] if not use_global else np.eye(4)
        T, fitness, rmse = align_to_map(src, map_pcd, voxel_size, use_global, init_T)
        elapsed = time.time() - t0

        print(f"[{i}/{N-1}] {pcd_paths[i].name}  "
              f"fitness={fitness:.4f}  rmse={rmse:.4f} m  ({elapsed:.1f}s)")
        if fitness < 0.30:
            print(f"  WARNING: low fitness — try --global-reg or a smaller --voxel-size.")

        transforms.append(T)

        src_world = copy.deepcopy(src)
        src_world.transform(T)
        clouds_world.append(src_world)

        map_pcd = map_pcd + src_world
        map_pcd = map_pcd.voxel_down_sample(voxel_size * 0.5)

    # ── save geometry map ──────────────────────────────────────────────────
    print()
    merged_geo = clouds_world[0]
    for c in clouds_world[1:]:
        merged_geo = merged_geo + c
    merged_geo = merged_geo.voxel_down_sample(voxel_size * 0.25)

    map_out = captures_dir / "aligned_map.pcd"
    o3d.io.write_point_cloud(str(map_out), merged_geo)
    print(f"Saved geometry map → {map_out}  ({len(merged_geo.points):,} pts)")

    tf_out = captures_dir / "transforms.npy"
    np.save(str(tf_out), np.array(transforms))
    print(f"Saved transforms   → {tf_out}")

    # ── colorize and save ──────────────────────────────────────────────────
    merged_colored = None
    if colorize:
        calib = load_calibration(config_dir)
        merged_colored = build_colored_map(pcd_paths, transforms, calib, voxel_size)

        col_out = captures_dir / "aligned_map_colored.pcd"
        o3d.io.write_point_cloud(str(col_out), merged_colored)
        print(f"Saved colorized map → {col_out}  ({len(merged_colored.points):,} pts)")

    # ── visualize ──────────────────────────────────────────────────────────
    if visualize:
        print("\nOpening viewer — rotate/zoom freely, press Q to close.")
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

        if merged_colored is not None:
            # Show the camera-colorized map
            o3d.visualization.draw_geometries(
                [merged_colored, axes],
                window_name="Aligned Map — Camera Colors",
                width=1280, height=720,
            )
        else:
            # Fall back to rainbow-by-index coloring
            colored = []
            for idx, c in enumerate(clouds_world):
                cc = copy.deepcopy(c)
                color_by_index(cc, idx, N)
                colored.append(cc)
            o3d.visualization.draw_geometries(
                colored + [axes],
                window_name="Incremental Alignment Result",
                width=1280, height=720,
            )


# ── CLI ────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    repo     = Path(__file__).resolve().parent.parent
    def_caps = repo / "docker-calib" / "captures" / "20260606_215951"
    def_cfg  = repo / "docker-calib" / "config"

    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "captures_dir", nargs="?", default=str(def_caps),
        help=f"Directory with *_lidar.pcd files (default: {def_caps})",
    )
    p.add_argument(
        "--config", default=str(def_cfg), metavar="DIR",
        help=f"Calibration directory with extrinsic.yaml + intrinsics.yaml "
             f"(default: {def_cfg})",
    )
    p.add_argument(
        "--voxel-size", type=float, default=0.05, metavar="M",
        help="Voxel size in metres for downsampling and ICP (default: 0.05)",
    )
    p.add_argument(
        "--global-reg", action="store_true",
        help="Use FPFH+RANSAC global registration before ICP "
             "(slower but more robust when clouds are far apart)",
    )
    p.add_argument(
        "--colorize", action="store_true",
        help="After alignment, project each cloud into its paired camera image "
             "and save an additional aligned_map_colored.pcd",
    )
    p.add_argument(
        "--no-viz", action="store_true",
        help="Skip the final open3d visualization window",
    )
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(
        captures_dir=Path(args.captures_dir),
        config_dir=Path(args.config),
        voxel_size=args.voxel_size,
        use_global=args.global_reg,
        colorize=args.colorize,
        visualize=not args.no_viz,
    )
