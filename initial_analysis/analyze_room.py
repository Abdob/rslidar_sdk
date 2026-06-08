#!/usr/bin/env python3
"""
analyze_room.py — Room surface analysis

Subcommands
-----------
  segment   Fit 5 planes via iterative RANSAC, then vote EVERY point to its
            nearest plane. Planes are named plane_0…plane_4 (largest first).
            Saves segments.npz next to the input PCD.

  relabel   Rename plane labels after visual inspection, e.g.:
              python analyze_room.py relabel plane_0=front_wall plane_2=floor

  measure   Colorize all points by signed distance from a named plane.
            Positive = same side as the room interior (toward LiDAR).

  filter    Keep only points within [--min, --max] metres of a named plane.

  mesh      Flat convex-hull mesh patch per plane → room_mesh.ply.

Typical workflow
----------------
  python analyze_room.py segment
  # read the printed normal directions, look at the 5 colors
  # identify which label is the front wall, floor, ceiling, etc.
  python analyze_room.py relabel plane_0=front_wall plane_1=floor plane_2=ceiling
  python analyze_room.py measure front_wall
  python analyze_room.py filter  front_wall --min 0.0 --max 1.83
  python analyze_room.py mesh

Room reference: 117 in wide × 6 ft deep × 8 ft high
               = 2.972 m    × 1.829 m    × 2.438 m
"""

from __future__ import annotations
import argparse
import copy
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull


# ── colours ────────────────────────────────────────────────────────────────

# Colours by plane index (used before semantic relabelling)
_INDEX_COLORS = [
    (0.20, 0.50, 1.00),   # 0 blue
    (1.00, 0.28, 0.28),   # 1 red
    (0.18, 0.82, 0.28),   # 2 green
    (1.00, 0.75, 0.00),   # 3 yellow
    (0.85, 0.20, 0.85),   # 4 magenta
]

# Colours for well-known semantic labels
_SEMANTIC_COLORS: dict[str, tuple] = {
    "floor":      (0.60, 0.38, 0.18),   # brown
    "ceiling":    (0.85, 0.85, 0.85),   # light gray
    "front_wall": (0.95, 0.55, 0.05),   # orange
    "left_wall":  (0.20, 0.50, 1.00),   # blue
    "right_wall": (1.00, 0.28, 0.28),   # red
}

_OTHER_COLOR = (0.35, 0.35, 0.35)


def _color(label: str, idx: int) -> tuple:
    return _SEMANTIC_COLORS.get(label, _INDEX_COLORS[idx % len(_INDEX_COLORS)])


# ── plane math ─────────────────────────────────────────────────────────────

def _signed_dist(pts: np.ndarray, model: np.ndarray) -> np.ndarray:
    a, b, c, d = model
    return (pts @ [a, b, c] + d) / np.linalg.norm([a, b, c])


def _orient_inward(model: np.ndarray, interior_pt: np.ndarray) -> np.ndarray:
    """Flip model so that interior_pt has positive signed distance."""
    if _signed_dist(interior_pt.reshape(1, 3), model)[0] < 0:
        return -model
    return model


def _dominant_axis(model: np.ndarray) -> str:
    """Return the axis label (e.g. '+X', '-Z') that the normal most aligns with."""
    n   = model[:3] / np.linalg.norm(model[:3])
    idx = int(np.argmax(np.abs(n)))
    return ("+", "-")[n[idx] < 0] + "XYZ"[idx]


# ── plane fitting ──────────────────────────────────────────────────────────

def fit_planes(
    pcd: o3d.geometry.PointCloud,
    n: int = 5,
    dist_thresh: float = 0.02,
    min_pts: int = 200,
) -> list[tuple[np.ndarray, int]]:
    """
    Iterative RANSAC. Returns [(model, n_inliers), …] sorted by inlier count.
    """
    results   = []
    remaining = pcd
    for _ in range(n):
        if len(remaining.points) < min_pts:
            break
        model, idx = remaining.segment_plane(
            distance_threshold=dist_thresh,
            ransac_n=3,
            num_iterations=2000,
        )
        results.append((np.array(model, dtype=np.float64), len(idx)))
        remaining = remaining.select_by_index(idx, invert=True)

    # Sort largest → smallest so plane_0 is always the most prominent surface
    results.sort(key=lambda x: -x[1])
    return results


def vote_points(
    pts: np.ndarray,
    models: list[np.ndarray],
    assign_thresh: float = np.inf,
) -> np.ndarray:
    """
    Assign every point to its nearest plane model.
    Returns (N,) int8: plane index 0…K-1, or -1 if farther than assign_thresh.
    """
    K     = len(models)
    dists = np.column_stack([np.abs(_signed_dist(pts, m)) for m in models])  # (N, K)
    best  = dists.argmin(axis=1).astype(np.int8)
    if np.isfinite(assign_thresh):
        best[dists[np.arange(len(pts)), best] > assign_thresh] = -1
    return best


# ── mesh patch ─────────────────────────────────────────────────────────────

def plane_patch_mesh(
    pts_3d: np.ndarray,
    model: np.ndarray,
    color: tuple = (0.7, 0.7, 0.7),
) -> o3d.geometry.TriangleMesh | None:
    if len(pts_3d) < 4:
        return None

    a, b, c, _ = model
    normal = np.array([a, b, c], dtype=np.float64)
    normal /= np.linalg.norm(normal)

    ref = np.array([0., 0., 1.]) if abs(normal[2]) < 0.9 else np.array([1., 0., 0.])
    u   = np.cross(normal, ref);  u /= np.linalg.norm(u)
    v   = np.cross(normal, u)

    centroid = pts_3d.mean(axis=0)
    pts_2d   = (pts_3d - centroid) @ np.column_stack([u, v])

    try:
        hull = ConvexHull(pts_2d)
    except Exception as e:
        print(f"    ConvexHull failed: {e}")
        return None

    hv_2d = pts_2d[hull.vertices]
    hv_3d = centroid + np.outer(hv_2d[:, 0], u) + np.outer(hv_2d[:, 1], v)

    M     = len(hull.vertices)
    all_v = np.vstack([centroid.reshape(1, 3), hv_3d])
    tris  = [[0, i+1, i % M + 2] for i in range(M-1)] + [[0, M, 1]]

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(all_v)
    mesh.triangles = o3d.utility.Vector3iVector(tris)
    mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    return mesh


# ── colourmap ──────────────────────────────────────────────────────────────

def jet_rgb(values: np.ndarray) -> np.ndarray:
    lo, hi = np.percentile(values, [2, 98])
    v = np.clip((values - lo) / (hi - lo + 1e-9), 0, 1)
    r = np.clip(1.5 - np.abs(4*v - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4*v - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4*v - 1), 0, 1)
    return np.column_stack([r, g, b])


# ── I/O ────────────────────────────────────────────────────────────────────

def _segs_path(pcd_path: Path) -> Path:
    return pcd_path.parent / "segments.npz"


def _load_pcd(path: Path) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(str(path))
    if not pcd.has_points():
        sys.exit(f"ERROR: no points loaded from {path}")
    return pcd


def _load_segments(pcd_path: Path) -> tuple[np.ndarray, list[str]]:
    sp = _segs_path(pcd_path)
    if not sp.exists():
        sys.exit(f"ERROR: {sp} not found — run 'segment' first.")
    data = np.load(str(sp), allow_pickle=True)
    return data["plane_models"], list(data["labels"])


def _save_segments(pcd_path: Path, models: np.ndarray, labels: list[str]) -> None:
    np.savez(str(_segs_path(pcd_path)),
             plane_models=models,
             labels=np.array(labels, dtype=object))


def _resolve(surface: str, labels: list[str]) -> int:
    if surface not in labels:
        sys.exit(f"ERROR: '{surface}' not found. Available: {labels}")
    return labels.index(surface)


# ── subcommands ────────────────────────────────────────────────────────────

def cmd_segment(args):
    pcd_path = Path(args.input).resolve()
    pcd      = _load_pcd(pcd_path)
    pts      = np.asarray(pcd.points)
    print(f"Loaded {len(pts):,} points from {pcd_path.name}")
    print(f"Fitting up to 5 planes (dist_thresh={args.dist_thresh} m) …\n")

    plane_results = fit_planes(pcd, n=5, dist_thresh=args.dist_thresh)
    K      = len(plane_results)
    models = [m for m, _ in plane_results]
    labels = [f"plane_{i}" for i in range(K)]

    # Orient normals toward cloud interior
    centroid = pts.mean(axis=0)
    models   = [_orient_inward(m, centroid) for m in models]

    # ── vote every point to nearest plane ─────────────────────────────────
    assignments = vote_points(pts, models, assign_thresh=args.assign_thresh)

    print(f"{'Label':<12}  {'RANSAC pts':>10}  {'Voted pts':>10}  "
          f"{'Dom. axis':>9}  Normal (unit)")
    print("-" * 72)
    for i, (model, n_ransac) in enumerate(plane_results):
        n_voted = int((assignments == i).sum())
        n_hat   = model[:3] / np.linalg.norm(model[:3])
        dom     = _dominant_axis(model)
        print(f"  {labels[i]:<12}  {n_ransac:>10,}  {n_voted:>10,}  "
              f"{dom:>9}  [{n_hat[0]:+.3f} {n_hat[1]:+.3f} {n_hat[2]:+.3f}]")

    n_unassigned = int((assignments == -1).sum())
    if n_unassigned:
        print(f"  {'(unassigned)':<12}  {'':>10}   {n_unassigned:>10,}")
    print()

    _save_segments(pcd_path, np.array(models), labels)
    print(f"Saved segments → {_segs_path(pcd_path)}")
    print()
    print("Identify each plane from the visualisation, then rename with:")
    print("  python analyze_room.py relabel plane_0=front_wall plane_1=floor …")

    if args.no_viz:
        return

    palette = np.array([_color(lbl, i) for i, lbl in enumerate(labels)] + [_OTHER_COLOR])
    point_colors = palette[assignments]   # -1 maps to last row = _OTHER_COLOR

    viz = copy.deepcopy(pcd)
    viz.colors = o3d.utility.Vector3dVector(point_colors)

    legend = "  ".join(
        f"{lbl}={('blue','red','green','yellow','magenta')[i%5]}"
        for i, lbl in enumerate(labels)
    )
    print(f"\nColors: {legend}")

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    o3d.visualization.draw_geometries(
        [viz, axes],
        window_name="Segments (nearest-plane vote)  [Q]",
        width=1280, height=720,
    )


def cmd_relabel(args):
    pcd_path = Path(args.input).resolve()
    models, labels = _load_segments(pcd_path)

    renames: dict[str, str] = {}
    for mapping in args.mappings:
        if "=" not in mapping:
            sys.exit(f"ERROR: expected OLD=NEW, got '{mapping}'")
        old, new = mapping.split("=", 1)
        renames[old.strip()] = new.strip()

    for old, new in renames.items():
        if old not in labels:
            print(f"WARNING: '{old}' not in current labels {labels} — skipped")
            continue
        idx = labels.index(old)
        labels[idx] = new
        print(f"  {old} → {new}")

    _save_segments(pcd_path, models, labels)
    print(f"\nUpdated labels: {labels}")
    print(f"Saved → {_segs_path(pcd_path)}")

    if args.no_viz:
        return

    pcd  = _load_pcd(pcd_path)
    pts  = np.asarray(pcd.points)
    assignments = vote_points(pts, list(models), assign_thresh=np.inf)

    palette     = np.array([_color(lbl, i) for i, lbl in enumerate(labels)] + [_OTHER_COLOR])
    viz         = copy.deepcopy(pcd)
    viz.colors  = o3d.utility.Vector3dVector(palette[assignments])

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    o3d.visualization.draw_geometries(
        [viz, axes],
        window_name=f"Relabelled: {labels}  [Q]",
        width=1280, height=720,
    )


def cmd_measure(args):
    pcd_path = Path(args.input).resolve()
    pcd      = _load_pcd(pcd_path)
    models, labels = _load_segments(pcd_path)

    k     = _resolve(args.surface, labels)
    pts   = np.asarray(pcd.points)
    dists = _signed_dist(pts, models[k])

    pos = dists[dists > 0]
    print(f"Signed distance from '{args.surface}'  (positive = inside room):")
    print(f"  all     min={dists.min():.3f}  max={dists.max():.3f}  "
          f"mean={dists.mean():.3f}  std={dists.std():.3f}  m")
    if len(pos):
        print(f"  inside  min={pos.min():.3f}  max={pos.max():.3f}  "
              f"mean={pos.mean():.3f}  m")

    if args.no_viz:
        return

    viz = copy.deepcopy(pcd)
    viz.colors = o3d.utility.Vector3dVector(jet_rgb(dists))
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    o3d.visualization.draw_geometries(
        [viz, axes],
        window_name=f"Distance from '{args.surface}'  blue=near  red=far  [Q]",
        width=1280, height=720,
    )


def cmd_filter(args):
    pcd_path = Path(args.input).resolve()
    pcd      = _load_pcd(pcd_path)
    models, labels = _load_segments(pcd_path)

    k     = _resolve(args.surface, labels)
    pts   = np.asarray(pcd.points)
    dists = _signed_dist(pts, models[k])
    mask  = (dists >= args.min_dist) & (dists <= args.max_dist)

    cropped  = pcd.select_by_index(np.where(mask)[0])
    out_path = Path(args.output).resolve() if args.output \
               else pcd_path.parent / f"filtered_{args.surface}.pcd"

    print(f"Distance from '{args.surface}' in [{args.min_dist:.3f}, {args.max_dist:.3f}] m:")
    print(f"  {mask.sum():,} / {len(pts):,} points kept")
    o3d.io.write_point_cloud(str(out_path), cropped)
    print(f"  Saved → {out_path}")

    if args.no_viz:
        return

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    o3d.visualization.draw_geometries(
        [cropped, axes],
        window_name=f"Filter '{args.surface}' [{args.min_dist:.2f},{args.max_dist:.2f}] m  [Q]",
        width=1280, height=720,
    )


def cmd_mesh(args):
    pcd_path = Path(args.input).resolve()
    pcd      = _load_pcd(pcd_path)
    models, labels = _load_segments(pcd_path)

    pts    = np.asarray(pcd.points)
    meshes = []
    print(f"Building mesh patches (inlier threshold={args.dist_thresh} m)\n")

    for i, (model, lbl) in enumerate(zip(models, labels)):
        mask    = np.abs(_signed_dist(pts, model)) < args.dist_thresh
        inliers = pts[mask]
        if len(inliers) < 10:
            print(f"  {lbl:<14}  {len(inliers):>6,} inliers — skipped")
            continue
        patch = plane_patch_mesh(inliers, model, color=_color(lbl, i))
        if patch is None:
            print(f"  {lbl:<14}  {len(inliers):>6,} inliers — mesh failed")
            continue
        meshes.append((lbl, patch))
        print(f"  {lbl:<14}  {len(inliers):>6,} inliers  → {len(patch.triangles)} triangles")

    if not meshes:
        sys.exit("No mesh patches generated.")

    combined = o3d.geometry.TriangleMesh()
    for _, m in meshes:
        combined += m

    out_path = Path(args.output).resolve() if args.output \
               else pcd_path.parent / "room_mesh.ply"
    o3d.io.write_triangle_mesh(str(out_path), combined)
    print(f"\nSaved → {out_path}")

    if args.no_viz:
        return

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    o3d.visualization.draw_geometries(
        [combined, axes],
        window_name="Room Mesh  [Q]",
        width=1280, height=720,
        mesh_show_back_face=True,
    )


# ── CLI ────────────────────────────────────────────────────────────────────

def _default_pcd() -> str:
    caps = Path(__file__).resolve().parent / "../docker-calib/captures/20260606_215951"
    for name in ("filtered_roi.pcd", "aligned_map_colored.pcd", "aligned_map.pcd"):
        p = caps / name
        if p.exists():
            return str(p)
    return str(caps / "filtered_roi.pcd")


def main() -> None:
    def_pcd = _default_pcd()

    root = argparse.ArgumentParser(
        prog="analyze_room.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subs = root.add_subparsers(dest="cmd", metavar="subcommand")
    subs.required = True

    def _add_input(p):
        p.add_argument("input", nargs="?", default=def_pcd, metavar="PCD",
                       help=f"Point cloud (default: {def_pcd})")
    def _add_noviz(p):
        p.add_argument("--no-viz", action="store_true", help="Skip viewer")
    def _add_thresh(p, default=0.02):
        p.add_argument("--dist-thresh", type=float, default=default, metavar="M",
                       help=f"Plane inlier distance in metres (default {default})")

    # segment ────────────────────────────────────────────────────────────
    s = subs.add_parser("segment",
        help="Fit 5 planes, vote every point to nearest, save segments.npz")
    _add_input(s); _add_thresh(s); _add_noviz(s)
    s.add_argument("--assign-thresh", type=float, default=np.inf, metavar="M",
                   help="Max distance to assign a point; farther points become "
                        "'unassigned' (default: assign everything)")
    s.set_defaults(func=cmd_segment)

    # relabel ────────────────────────────────────────────────────────────
    r = subs.add_parser("relabel",
        help="Rename planes after visual inspection, e.g. plane_0=front_wall")
    r.add_argument("mappings", nargs="+", metavar="OLD=NEW",
                   help="One or more OLD_LABEL=NEW_LABEL pairs")
    _add_input(r); _add_noviz(r)
    r.set_defaults(func=cmd_relabel)

    # measure ────────────────────────────────────────────────────────────
    m = subs.add_parser("measure", help="Colorize by distance from a plane")
    m.add_argument("surface", help="Plane label, e.g. front_wall or plane_0")
    _add_input(m); _add_noviz(m)
    m.set_defaults(func=cmd_measure)

    # filter ─────────────────────────────────────────────────────────────
    f = subs.add_parser("filter", help="Keep points in a distance range from a plane")
    f.add_argument("surface", help="Plane label")
    _add_input(f)
    f.add_argument("--min", dest="min_dist", type=float, default=0.0, metavar="M")
    f.add_argument("--max", dest="max_dist", type=float, default=3.0, metavar="M")
    f.add_argument("-o", "--output", default=None, metavar="PCD")
    _add_noviz(f)
    f.set_defaults(func=cmd_filter)

    # mesh ───────────────────────────────────────────────────────────────
    ms = subs.add_parser("mesh", help="Flat convex-hull mesh per plane → room_mesh.ply")
    _add_input(ms); _add_thresh(ms, default=0.025)
    ms.add_argument("-o", "--output", default=None, metavar="PLY")
    _add_noviz(ms)
    ms.set_defaults(func=cmd_mesh)

    args = root.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
