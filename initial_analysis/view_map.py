#!/usr/bin/env python3
"""
Point cloud viewer.

Usage:
    python view_map.py [pcd_file] [--height]

    --height   Override existing colors with jet height colormap.
               Default: use colors stored in the file.
"""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def jet_by_height(pcd: o3d.geometry.PointCloud) -> None:
    pts = np.asarray(pcd.points)
    z   = pts[:, 2]
    v   = np.clip((z - z.min()) / (z.max() - z.min() + 1e-6), 0, 1)
    r   = np.clip(1.5 - np.abs(4*v - 3), 0, 1)
    g   = np.clip(1.5 - np.abs(4*v - 2), 0, 1)
    b   = np.clip(1.5 - np.abs(4*v - 1), 0, 1)
    pcd.colors = o3d.utility.Vector3dVector(np.column_stack([r, g, b]))


def parse_args() -> argparse.Namespace:
    repo    = Path(__file__).resolve().parent.parent
    default = repo / "docker-calib" / "captures" / "20260606_215951" / "aligned_map_colored.pcd"

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("pcd_file", nargs="?", default=str(default),
                   help=f"PCD file to view (default: {default})")
    p.add_argument("--height", action="store_true",
                   help="Color by Z-height (jet) instead of stored colors")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    path = Path(args.pcd_file)

    if not path.exists():
        raise SystemExit(f"ERROR: file not found: {path}")

    pcd = o3d.io.read_point_cloud(str(path))
    print(f"Loaded {len(pcd.points):,} points from {path.name}")

    if args.height or not pcd.has_colors():
        jet_by_height(pcd)
        color_mode = "height (jet)"
    else:
        color_mode = "stored colors"

    print(f"Color mode : {color_mode}")
    print("Controls   : left-drag rotate  |  right-drag pan  |  scroll zoom  |  Q quit")

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries(
        [pcd, axes],
        window_name=f"{path.name} — {color_mode}",
        width=1280, height=720,
    )
