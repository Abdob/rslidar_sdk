#!/usr/bin/env python3
"""
Incremental LiDAR Point Cloud Registration

Aligns PCD[0] → PCD[1], then (PCD[0]+PCD[1]) → PCD[2], all the way to PCD[N-1].
Each cloud is brought into the frame of the first cloud (world frame).

Pipeline per step
-----------------
  1. Voxel-downsample source + current map.
  2. Estimate normals on both.
  3. [optional] FPFH + RANSAC global registration as initial guess.
  4. Multi-scale point-to-plane ICP refinement (coarse → fine).
  5. Transform full-resolution cloud and append to map.

Outputs (written to captures_dir)
----------------------------------
  aligned_map.pcd   merged cloud in the frame of cloud 0
  transforms.npy    (N, 4, 4) cumulative transforms for each cloud
"""

from __future__ import annotations
import argparse
import copy
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d


# ── helpers ────────────────────────────────────────────────────────────────

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
    """
    Three-scale point-to-plane ICP (coarse → fine).

    Returns (transform 4×4, fitness, inlier_rmse).
    """
    scales   = [4.0, 2.0, 1.0]
    max_iters= [50,  30,  100]
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
    hue   = idx / max(total - 1, 1)       # 0 → 1
    # HSV → RGB (S=1, V=1)
    h6 = hue * 6.0
    i  = int(h6) % 6
    f  = h6 - int(h6)
    lut = [
        (1, f,     0),
        (1-f, 1,   0),
        (0,   1,   f),
        (0, 1-f,   1),
        (f,   0,   1),
        (1,   0, 1-f),
    ]
    r, g, b = lut[i]
    n = len(pcd.points)
    pcd.colors = o3d.utility.Vector3dVector(
        np.tile([r, g, b], (n, 1)).astype(np.float64)
    )


# ── main ───────────────────────────────────────────────────────────────────

def run(captures_dir: Path, voxel_size: float, use_global: bool, visualize: bool) -> None:
    pcd_paths = find_pcds(captures_dir)
    N         = len(pcd_paths)
    print(f"Found {N} point clouds in {captures_dir}")
    print(f"Voxel size : {voxel_size} m")
    print(f"Global reg : {'yes (FPFH+RANSAC)' if use_global else 'no (ICP only)'}")
    print()

    # Load first cloud as world-frame reference
    map_pcd = load(pcd_paths[0])
    print(f"[0/{N-1}] {pcd_paths[0].name}  →  reference  "
          f"({len(map_pcd.points):,} pts)")

    transforms: list[np.ndarray] = [np.eye(4)]     # T[0] = identity
    clouds_world: list[o3d.geometry.PointCloud] = [copy.deepcopy(map_pcd)]

    for i in range(1, N):
        src = load(pcd_paths[i])
        t0  = time.time()

        # Use previous transform as warm start for ICP (good for nearby scans)
        init_T = transforms[-1] if not use_global else np.eye(4)

        T, fitness, rmse = align_to_map(
            src, map_pcd, voxel_size, use_global, init_T
        )
        elapsed = time.time() - t0

        print(f"[{i}/{N-1}] {pcd_paths[i].name}  "
              f"fitness={fitness:.4f}  rmse={rmse:.4f} m  ({elapsed:.1f}s)")

        if fitness < 0.30:
            print(f"  WARNING: low fitness ({fitness:.4f}) — overlap may be poor; "
                  f"try --global-reg or a smaller --voxel-size.")

        transforms.append(T)

        # Transform full-resolution cloud and accumulate
        src_world = copy.deepcopy(src)
        src_world.transform(T)
        clouds_world.append(src_world)

        map_pcd = map_pcd + src_world
        # Downsample map to keep ICP target manageable
        map_pcd = map_pcd.voxel_down_sample(voxel_size * 0.5)

    # ── save outputs ───────────────────────────────────────────────────────
    print()

    # Merged map (full-res per-cloud union, then light downsample)
    merged = clouds_world[0]
    for c in clouds_world[1:]:
        merged = merged + c
    merged = merged.voxel_down_sample(voxel_size * 0.25)

    map_out = captures_dir / "aligned_map.pcd"
    o3d.io.write_point_cloud(str(map_out), merged)
    print(f"Saved merged cloud → {map_out}  ({len(merged.points):,} pts)")

    tf_out = captures_dir / "transforms.npy"
    np.save(str(tf_out), np.array(transforms))
    print(f"Saved transforms   → {tf_out}")

    # ── visualize ──────────────────────────────────────────────────────────
    if visualize:
        print("\nOpening viewer — rotate/zoom freely, press Q to close.")
        colored = []
        for idx, c in enumerate(clouds_world):
            cc = copy.deepcopy(c)
            color_by_index(cc, idx, N)
            colored.append(cc)

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        o3d.visualization.draw_geometries(
            colored + [axes],
            window_name="Incremental Alignment Result",
            width=1280, height=720,
            point_show_normal=False,
        )


# ── CLI ────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    repo     = Path(__file__).resolve().parent.parent
    def_caps = repo / "docker-calib" / "captures" / "20260606_215951"

    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "captures_dir", nargs="?", default=str(def_caps),
        help=f"Directory with *_lidar.pcd files (default: {def_caps})",
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
        "--no-viz", action="store_true",
        help="Skip the final open3d visualization window",
    )
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(
        captures_dir=Path(args.captures_dir),
        voxel_size=args.voxel_size,
        use_global=args.global_reg,
        visualize=not args.no_viz,
    )
