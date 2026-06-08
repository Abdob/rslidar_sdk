#!/usr/bin/env python3
"""
Overlay two point clouds in one open3d window for visual comparison.

The forward map (A) is in cloud-0's frame.
The reverse map (B) is in cloud-5's frame.
transforms.npy (from the forward alignment) contains the cumulative transforms;
its last entry T[-1] maps cloud-5's frame → cloud-0's frame, so B is
transformed by T[-1] before overlaying.

  cloud A → blue   (or its stored colors with --use-colors)
  cloud B → green  (or its stored colors with --use-colors)

Usage:
    python compare_maps.py [file_a] [file_b] [--transforms file] [--use-colors]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


def solid_color(pcd: o3d.geometry.PointCloud, rgb: tuple) -> None:
    n = len(pcd.points)
    pcd.colors = o3d.utility.Vector3dVector(
        np.tile(rgb, (n, 1)).astype(np.float64)
    )


def parse_args() -> argparse.Namespace:
    here    = Path(__file__).resolve().parent
    def_a   = here / "../docker-calib/captures/20260606_215951/aligned_map_colored.pcd"
    def_b   = here / "../../aligned_map_colored.pcd"
    def_tf  = here / "../docker-calib/captures/20260606_215951/transforms.npy"

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("file_a", nargs="?", default=str(def_a),
                   help="Forward map  (shown in blue)  — reference frame")
    p.add_argument("file_b", nargs="?", default=str(def_b),
                   help="Reverse map  (shown in green) — will be transformed into A's frame")
    p.add_argument("--transforms", default=str(def_tf), metavar="NPY",
                   help="transforms.npy from the forward alignment; "
                        "last entry maps cloud-N's frame → cloud-0's frame "
                        f"(default: {def_tf})")
    p.add_argument("--use-colors", action="store_true",
                   help="Show each cloud's stored colors instead of solid blue/green")
    return p.parse_args()


if __name__ == "__main__":
    args    = parse_args()
    path_a  = Path(args.file_a).resolve()
    path_b  = Path(args.file_b).resolve()
    path_tf = Path(args.transforms).resolve()

    for p in (path_a, path_b):
        if not p.exists():
            sys.exit(f"ERROR: file not found: {p}")
    if not path_tf.exists():
        sys.exit(f"ERROR: transforms file not found: {path_tf}\n"
                 "       Run align.py first, or supply --transforms <path>")

    # T[-1]: cloud-(N-1) frame → cloud-0 frame  (the frame bridge between the two maps)
    transforms = np.load(str(path_tf))          # (N, 4, 4)
    T = transforms[-1]
    print(f"Loaded {len(transforms)} transforms from {path_tf.name}")
    print(f"Applying T[{len(transforms)-1}] (cloud-{len(transforms)-1} → cloud-0 frame) to B:")
    print(np.round(T, 6))
    print()

    pcd_a = o3d.io.read_point_cloud(str(path_a))
    pcd_b = o3d.io.read_point_cloud(str(path_b))

    print(f"A  {path_a.name}  {len(pcd_a.points):,} pts  (unchanged — reference frame)")
    print(f"B  {path_b.name}  {len(pcd_b.points):,} pts  (transformed into A's frame)")

    # Bring B into A's coordinate frame
    pcd_b.transform(T)

    if args.use_colors:
        if not pcd_a.has_colors():
            print("WARNING: A has no stored colors — falling back to blue")
            solid_color(pcd_a, (0.2, 0.4, 1.0))
        if not pcd_b.has_colors():
            print("WARNING: B has no stored colors — falling back to green")
            solid_color(pcd_b, (0.1, 0.9, 0.3))
        color_note = "stored colors"
    else:
        solid_color(pcd_a, (0.2, 0.4, 1.0))   # blue
        solid_color(pcd_b, (0.1, 0.9, 0.3))   # green
        color_note = "A=blue  B=green"

    print(f"Color mode : {color_note}")
    print("Controls   : left-drag rotate  |  right-drag pan  |  scroll zoom  |  Q quit")

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries(
        [pcd_a, pcd_b, axes],
        window_name=f"Compare: {path_a.name}  vs  {path_b.name}  [{color_note}]",
        width=1280, height=720,
    )
