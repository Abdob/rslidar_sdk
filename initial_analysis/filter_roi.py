#!/usr/bin/env python3
"""
Filter a point cloud to a cuboid region of interest.

Two modes
---------
Interactive (default)
    Opens the cloud in open3d's editing viewer.
      1. Press K        lock view and enter polygon-selection mode
      2. Left-click     add polygon vertices around the region
      3. Press C        crop to the polygon volume
      4. Press Q        quit and save filtered_roi.pcd

Bounds  (--xmin / --xmax / --ymin / --ymax / --zmin / --zmax)
    Apply a repeatable axis-aligned crop with explicit coordinate bounds.
    Tip: run interactive mode first to read the numbers from the printed
    bounding box, then harden them into a bounds call.

Room reference (117 in × 6 ft × 8 ft)
    2.972 m wide  ×  1.829 m deep  ×  2.438 m high
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

INCHES_TO_M = 0.0254
FEET_TO_M   = 0.3048

ROOM_W = 117 * INCHES_TO_M   # 2.972 m
ROOM_D =   6 * FEET_TO_M     # 1.829 m
ROOM_H =   8 * FEET_TO_M     # 2.438 m


def print_cloud_info(pcd: o3d.geometry.PointCloud, label: str = "") -> None:
    pts  = np.asarray(pcd.points)
    lo   = pts.min(axis=0)
    hi   = pts.max(axis=0)
    span = hi - lo
    tag  = f"[{label}]  " if label else ""
    print(f"{tag}{len(pts):,} points")
    print(f"  X  {lo[0]:+.3f} → {hi[0]:+.3f}   span {span[0]:.3f} m")
    print(f"  Y  {lo[1]:+.3f} → {hi[1]:+.3f}   span {span[1]:.3f} m")
    print(f"  Z  {lo[2]:+.3f} → {hi[2]:+.3f}   span {span[2]:.3f} m")
    print(f"  Room target:  W {ROOM_W:.3f} m  D {ROOM_D:.3f} m  H {ROOM_H:.3f} m")


def jet_by_height(pcd: o3d.geometry.PointCloud) -> None:
    pts = np.asarray(pcd.points)
    z   = pts[:, 2]
    v   = np.clip((z - z.min()) / (z.max() - z.min() + 1e-6), 0, 1)
    r   = np.clip(1.5 - np.abs(4*v - 3), 0, 1)
    g   = np.clip(1.5 - np.abs(4*v - 2), 0, 1)
    b   = np.clip(1.5 - np.abs(4*v - 1), 0, 1)
    pcd.colors = o3d.utility.Vector3dVector(np.column_stack([r, g, b]))


def run_interactive(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Open the editing viewer and return the cropped cloud."""
    print()
    print("Interactive crop controls")
    print("  K            lock view / enter polygon-selection mode")
    print("  left-click   place polygon vertices around the region")
    print("  C            crop to selection")
    print("  Q            quit and save")
    print()

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window("ROI Filter — interactive  [K → click → C → Q]",
                      width=1280, height=720)

    ropt = vis.get_render_option()
    ropt.background_color = np.array([0.07, 0.07, 0.07])
    ropt.point_size = 2.0

    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    cropped = vis.get_cropped_geometry()
    return cropped


def run_bounds(
    pcd: o3d.geometry.PointCloud,
    xmin: float, xmax: float,
    ymin: float, ymax: float,
    zmin: float, zmax: float,
) -> o3d.geometry.PointCloud:
    """Axis-aligned bounding box crop."""
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array([xmin, ymin, zmin]),
        max_bound=np.array([xmax, ymax, zmax]),
    )
    # Draw the box for reference
    bbox.color = (1, 0, 0)
    return pcd.crop(bbox), bbox


def parse_args() -> argparse.Namespace:
    here    = Path(__file__).resolve().parent
    default = here / "../docker-calib/captures/20260606_215951/aligned_map_colored.pcd"

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input", nargs="?", default=str(default),
                   help=f"PCD file to filter (default: {default})")
    p.add_argument("-o", "--output", default=None, metavar="PCD",
                   help="Output file (default: <input_dir>/filtered_roi.pcd)")

    bounds = p.add_argument_group(
        "Bounds mode — axis-aligned crop (all six required together)"
    )
    bounds.add_argument("--xmin", type=float, default=None, metavar="M")
    bounds.add_argument("--xmax", type=float, default=None, metavar="M")
    bounds.add_argument("--ymin", type=float, default=None, metavar="M")
    bounds.add_argument("--ymax", type=float, default=None, metavar="M")
    bounds.add_argument("--zmin", type=float, default=None, metavar="M")
    bounds.add_argument("--zmax", type=float, default=None, metavar="M")

    p.add_argument("--no-viz", action="store_true",
                   help="Skip the preview window after bounds-mode crop")
    return p.parse_args()


if __name__ == "__main__":
    args     = parse_args()
    in_path  = Path(args.input).resolve()
    out_path = Path(args.output).resolve() if args.output \
               else in_path.parent / "filtered_roi.pcd"

    if not in_path.exists():
        sys.exit(f"ERROR: file not found: {in_path}")

    pcd = o3d.io.read_point_cloud(str(in_path))
    if not pcd.has_points():
        sys.exit(f"ERROR: no points loaded from {in_path}")

    if not pcd.has_colors():
        jet_by_height(pcd)

    print(f"Loaded  {in_path.name}")
    print_cloud_info(pcd, "full cloud")
    print()

    bounds_args = [args.xmin, args.xmax, args.ymin, args.ymax, args.zmin, args.zmax]
    use_bounds  = any(v is not None for v in bounds_args)

    if use_bounds:
        # ── bounds mode ───────────────────────────────────────────────────
        missing = [n for n, v in zip(
            ["--xmin","--xmax","--ymin","--ymax","--zmin","--zmax"], bounds_args
        ) if v is None]
        if missing:
            sys.exit(f"ERROR: bounds mode requires all six values; missing: {' '.join(missing)}")

        cropped, bbox = run_bounds(
            pcd,
            args.xmin, args.xmax,
            args.ymin, args.ymax,
            args.zmin, args.zmax,
        )
        print(f"Bounds crop:  X[{args.xmin}, {args.xmax}]  "
              f"Y[{args.ymin}, {args.ymax}]  Z[{args.zmin}, {args.zmax}]")
        print_cloud_info(cropped, "cropped")

        if not args.no_viz:
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
            o3d.visualization.draw_geometries(
                [cropped, bbox, axes],
                window_name="ROI Filter — bounds result  [Q to close]",
                width=1280, height=720,
            )
    else:
        # ── interactive mode ──────────────────────────────────────────────
        cropped = run_interactive(pcd)

    if cropped is None or not cropped.has_points():
        sys.exit("No points in cropped result — nothing saved.")

    print()
    print_cloud_info(cropped, "result")
    o3d.io.write_point_cloud(str(out_path), cropped)
    print(f"\nSaved → {out_path}")

    # Print the axis-aligned bounds of the result so the user can
    # copy-paste them into a future --bounds call
    pts = np.asarray(cropped.points)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    print()
    print("Repeat this crop with:")
    print(f"  python filter_roi.py {in_path} \\")
    print(f"    --xmin {lo[0]:.4f} --xmax {hi[0]:.4f} \\")
    print(f"    --ymin {lo[1]:.4f} --ymax {hi[1]:.4f} \\")
    print(f"    --zmin {lo[2]:.4f} --zmax {hi[2]:.4f}")
