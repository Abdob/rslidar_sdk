#!/usr/bin/env python3
"""
mesh_surfaces.py — Piecewise-planar mesh from room surfaces

Segments the input point cloud into planar sections, then builds a dense
coloured mesh for each section using overlapping local plane fits.

Algorithm per section
---------------------
  1. A global plane is fitted via iterative RANSAC; every point is voted to
     its nearest plane (including RANSAC outliers / slightly bent edges).
  2. A 2D grid is laid on the global plane at --tile-step resolution.
  3. Each grid vertex samples all points within a --tile-size × --tile-size
     square (overlapping windows) and fits a local plane by least squares.
     The vertex 3D position is the local plane evaluated at the grid point;
     its colour is an inverse-distance-weighted average of nearby point colours.
  4. Vertices within --snap-thresh of any section-section intersection line
     are projected onto that line so adjacent meshes meet edge-to-edge.
  5. The grid is triangulated; quads with edges longer than --max-edge are
     discarded to avoid bridging across gaps in the data.

Usage
-----
  python mesh_surfaces.py [input.pcd] [options]

  --tile-size M    Side of the local fitting window in metres  (default 0.20)
  --tile-step M    Grid vertex spacing in metres               (default 0.05)
  --snap-thresh M  Snap vertices within this distance of an
                   intersection line onto that line            (default 0.10)
  --max-edge M     Discard triangles with any edge longer than this (default 0.30)
  --n-planes N     Max number of planes to detect             (default 5)
  --dist-thresh M  RANSAC inlier distance in metres            (default 0.02)
  --no-snap        Disable boundary snapping
  --no-viz         Skip the viewer
  -o / --output    Output mesh path (default: <input_dir>/surfaces_mesh.ply)
"""

from __future__ import annotations
import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


# Per-section fallback colours (when PCD has no stored colours)
_SECTION_COLORS = [
    (0.20, 0.50, 1.00),   # blue
    (1.00, 0.28, 0.28),   # red
    (0.18, 0.82, 0.28),   # green
    (1.00, 0.75, 0.00),   # yellow
    (0.85, 0.20, 0.85),   # magenta
]


# ── plane math ─────────────────────────────────────────────────────────────

def _signed_dist(pts: np.ndarray, model: np.ndarray) -> np.ndarray:
    a, b, c, d = model
    return (pts @ [a, b, c] + d) / np.linalg.norm([a, b, c])


def _plane_basis(model: np.ndarray):
    """Return (normal, u_ax, v_ax, origin) — an orthonormal frame on the plane."""
    n      = model[:3]
    normal = n / np.linalg.norm(n)
    ref    = np.array([0., 0., 1.]) if abs(normal[2]) < 0.9 else np.array([1., 0., 0.])
    u_ax   = np.cross(normal, ref);  u_ax /= np.linalg.norm(u_ax)
    v_ax   = np.cross(normal, u_ax)
    origin = -model[3] / np.dot(n, n) * n   # foot of perpendicular from origin
    return normal, u_ax, v_ax, origin


def _intersection_line(ma: np.ndarray, mb: np.ndarray):
    """Intersection line of two planes → (point, direction) or (None, None)."""
    na, nb = ma[:3], mb[:3]
    d      = np.cross(na, nb)
    if np.linalg.norm(d) < 1e-6:
        return None, None          # parallel planes
    d /= np.linalg.norm(d)
    A = np.array([na, nb, d])
    b = np.array([-ma[3], -mb[3], 0.])
    try:
        p = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None, None
    return p, d


def _dist_to_line(pts: np.ndarray, lp: np.ndarray, ld: np.ndarray) -> np.ndarray:
    """Unsigned distance from each point to the 3-D line (lp, ld)."""
    rel  = pts - lp
    proj = lp + np.outer(rel @ ld, ld)
    return np.linalg.norm(pts - proj, axis=1)


# ── segmentation ───────────────────────────────────────────────────────────

def segment_cloud(
    pcd:         o3d.geometry.PointCloud,
    n:           int   = 5,
    dist_thresh: float = 0.02,
) -> tuple[list[np.ndarray], np.ndarray]:
    """
    Iterative RANSAC + nearest-plane voting.

    Returns
    -------
    models      : list of (4,) plane models, sorted largest-section-first,
                  normals oriented toward the cloud centroid (interior).
    assignments : (N,) int8 — section index for every point.
    """
    remaining = pcd
    raw       = []

    for _ in range(n):
        if len(remaining.points) < 200:
            break
        model, idx = remaining.segment_plane(
            distance_threshold=dist_thresh, ransac_n=3, num_iterations=2000,
        )
        raw.append((np.array(model, dtype=np.float64), len(idx)))
        remaining = remaining.select_by_index(idx, invert=True)

    if not raw:
        sys.exit("ERROR: no planes found — lower --dist-thresh or check the input PCD.")

    raw.sort(key=lambda x: -x[1])
    models = [m for m, _ in raw]

    centroid = np.asarray(pcd.points).mean(axis=0)
    models   = [m if _signed_dist(centroid.reshape(1, 3), m)[0] > 0 else -m
                for m in models]

    pts         = np.asarray(pcd.points)
    abs_d       = np.column_stack([np.abs(_signed_dist(pts, m)) for m in models])
    assignments = abs_d.argmin(axis=1).astype(np.int8)

    return models, assignments


# ── per-section mesh ────────────────────────────────────────────────────────

def build_section_mesh(
    pts_3d:       np.ndarray,
    colors:       np.ndarray | None,   # (N, 3) RGB [0,1] or None
    model:        np.ndarray,
    other_models: list[np.ndarray],
    tile_size:    float = 0.20,
    tile_step:    float = 0.05,
    snap_thresh:  float = 0.10,
    max_edge:     float = 0.30,
    min_pts:      int   = 4,
    do_snap:      bool  = True,
    fallback_rgb: tuple = (0.6, 0.6, 0.6),
) -> o3d.geometry.TriangleMesh | None:

    if len(pts_3d) < 10:
        return None

    normal, u_ax, v_ax, origin = _plane_basis(model)
    half = tile_size / 2.0

    rel = pts_3d - origin
    pu  = rel @ u_ax     # 2-D coordinate along u
    pv  = rel @ v_ax     # 2-D coordinate along v
    ph  = rel @ normal   # height above global plane

    # ── 2D grid ────────────────────────────────────────────────────────
    margin = tile_step
    u_grid = np.arange(pu.min() - margin, pu.max() + margin + tile_step, tile_step)
    v_grid = np.arange(pv.min() - margin, pv.max() + margin + tile_step, tile_step)
    Nu, Nv = len(u_grid), len(v_grid)

    tree2d     = cKDTree(np.column_stack([pu, pv]))
    GU, GV     = np.meshgrid(u_grid, v_grid, indexing='ij')          # (Nu, Nv)
    grid_flat  = np.column_stack([GU.ravel(), GV.ravel()])            # (Nu*Nv, 2)
    nbr_lists  = tree2d.query_ball_point(grid_flat, r=half * 1.415)  # circle ⊃ square

    V_pos   = np.full((Nu, Nv, 3), np.nan)
    V_col   = np.zeros((Nu, Nv, 3))
    V_valid = np.zeros((Nu, Nv), bool)

    # ── local plane fit at every grid vertex ───────────────────────────
    for flat_idx, nbrs in enumerate(nbr_lists):
        if not nbrs:
            continue
        i, j   = divmod(flat_idx, Nv)
        gu, gv = u_grid[i], v_grid[j]

        nbrs = np.array(nbrs)
        box  = (np.abs(pu[nbrs] - gu) <= half) & (np.abs(pv[nbrs] - gv) <= half)
        nbrs = nbrs[box]
        if len(nbrs) < min_pts:
            continue

        # Local plane: h ≈ a*(u-gu) + b*(v-gv) + c  (least squares)
        A = np.column_stack([pu[nbrs] - gu, pv[nbrs] - gv, np.ones(len(nbrs))])
        try:
            coeffs, _, _, _ = np.linalg.lstsq(A, ph[nbrs], rcond=None)
            h = float(coeffs[2])         # height at exactly (gu, gv)
        except Exception:
            h = float(ph[nbrs].mean())

        V_pos[i, j]   = origin + gu * u_ax + gv * v_ax + h * normal
        V_valid[i, j] = True

        # Inverse-distance-weighted colour
        if colors is not None:
            d2 = (pu[nbrs] - gu)**2 + (pv[nbrs] - gv)**2 + 1e-9
            w  = 1.0 / d2;  w /= w.sum()
            V_col[i, j] = np.clip((w[:, None] * colors[nbrs]).sum(axis=0), 0, 1)
        else:
            V_col[i, j] = fallback_rgb

    # ── boundary snapping ──────────────────────────────────────────────
    if do_snap:
        for om in other_models:
            lp, ld = _intersection_line(model, om)
            if lp is None:
                continue

            ij_valid  = np.argwhere(V_valid)               # (M, 2)
            vpos      = V_pos[ij_valid[:, 0], ij_valid[:, 1]]  # (M, 3)
            dist_line = _dist_to_line(vpos, lp, ld)

            for k, (i, j) in enumerate(ij_valid):
                if dist_line[k] < snap_thresh:
                    t            = (V_pos[i, j] - lp) @ ld
                    V_pos[i, j]  = lp + t * ld    # project onto intersection line

    # ── build vertex list ──────────────────────────────────────────────
    vid   = np.full((Nu, Nv), -1, dtype=np.int32)
    verts = []
    vcols = []

    for i in range(Nu):
        for j in range(Nv):
            if V_valid[i, j]:
                vid[i, j] = len(verts)
                verts.append(V_pos[i, j])
                vcols.append(V_col[i, j])

    if len(verts) < 3:
        return None

    verts_np = np.array(verts)

    # ── triangulate regular grid ───────────────────────────────────────
    tris = []
    for i in range(Nu - 1):
        for j in range(Nv - 1):
            v00 = vid[i,   j  ]
            v10 = vid[i+1, j  ]
            v01 = vid[i,   j+1]
            v11 = vid[i+1, j+1]

            # Lower-left triangle
            if v00 >= 0 and v10 >= 0 and v01 >= 0:
                e1 = np.linalg.norm(verts_np[v10] - verts_np[v00])
                e2 = np.linalg.norm(verts_np[v01] - verts_np[v00])
                e3 = np.linalg.norm(verts_np[v01] - verts_np[v10])
                if max(e1, e2, e3) < max_edge:
                    tris.append([v00, v10, v01])

            # Upper-right triangle
            if v10 >= 0 and v11 >= 0 and v01 >= 0:
                e1 = np.linalg.norm(verts_np[v11] - verts_np[v10])
                e2 = np.linalg.norm(verts_np[v01] - verts_np[v10])
                e3 = np.linalg.norm(verts_np[v01] - verts_np[v11])
                if max(e1, e2, e3) < max_edge:
                    tris.append([v10, v11, v01])

    if not tris:
        return None

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices      = o3d.utility.Vector3dVector(verts_np)
    mesh.triangles     = o3d.utility.Vector3iVector(np.array(tris))
    mesh.vertex_colors = o3d.utility.Vector3dVector(np.array(vcols))
    mesh.compute_vertex_normals()
    return mesh


# ── main ───────────────────────────────────────────────────────────────────

def run(
    pcd_path:    Path,
    out_path:    Path,
    n_planes:    int   = 5,
    dist_thresh: float = 0.02,
    tile_size:   float = 0.20,
    tile_step:   float = 0.05,
    snap_thresh: float = 0.10,
    max_edge:    float = 0.30,
    do_snap:     bool  = True,
    visualize:   bool  = True,
) -> None:

    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if not pcd.has_points():
        sys.exit(f"ERROR: no points in {pcd_path}")

    pts    = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None

    print(f"Loaded {len(pts):,} points  |  colours: {'yes' if colors is not None else 'no (section colours)'}")
    snap_str = f"on (<{snap_thresh*100:.0f} cm)" if do_snap else "off"
    print(f"Tile {tile_size*100:.0f} cm × {tile_size*100:.0f} cm  "
          f"|  step {tile_step*100:.0f} cm  "
          f"|  snap {snap_str}  "
          f"|  max-edge {max_edge*100:.0f} cm")
    print()

    # ── segment ────────────────────────────────────────────────────────
    print(f"Segmenting into up to {n_planes} planes …")
    models, assignments = segment_cloud(pcd, n=n_planes, dist_thresh=dist_thresh)

    def _dom_ax(m):
        n = m[:3] / np.linalg.norm(m[:3])
        i = int(np.argmax(np.abs(n)))
        return ('+' if n[i] > 0 else '-') + 'XYZ'[i]

    print(f"\n{'Section':<12}  {'Points':>8}  Dominant axis")
    for k, m in enumerate(models):
        print(f"  section_{k:<5}  {int((assignments == k).sum()):>8,}  {_dom_ax(m)}")
    print()

    # ── build meshes ───────────────────────────────────────────────────
    meshes = []
    for k, model in enumerate(models):
        mask    = assignments == k
        sec_pts = pts[mask]
        sec_col = colors[mask] if colors is not None else None
        others  = [m for i, m in enumerate(models) if i != k]
        fb      = _SECTION_COLORS[k % len(_SECTION_COLORS)]

        nu_approx = int((sec_pts[:, 0].max() - sec_pts[:, 0].min()
                        + sec_pts[:, 1].max() - sec_pts[:, 1].min()) / tile_step)
        print(f"section_{k}  {len(sec_pts):>8,} pts  (~{nu_approx} grid cols) …",
              end=" ", flush=True)

        mesh = build_section_mesh(
            sec_pts, sec_col, model, others,
            tile_size=tile_size, tile_step=tile_step,
            snap_thresh=snap_thresh, max_edge=max_edge,
            do_snap=do_snap, fallback_rgb=fb,
        )
        if mesh is None:
            print("skipped (insufficient data)")
            continue

        meshes.append(mesh)
        print(f"{len(mesh.vertices):,} verts  {len(mesh.triangles):,} tris")

    if not meshes:
        sys.exit("No meshes produced.")

    combined = o3d.geometry.TriangleMesh()
    for m in meshes:
        combined += m
    combined.compute_vertex_normals()

    o3d.io.write_triangle_mesh(str(out_path), combined)
    print(f"\nSaved → {out_path}")
    print(f"Total  {len(combined.vertices):,} vertices  {len(combined.triangles):,} triangles")

    if visualize:
        print("\nViewer: left-drag rotate  scroll zoom  Q quit")
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        o3d.visualization.draw_geometries(
            [combined, axes],
            window_name="Surfaces Mesh  [Q]",
            width=1280, height=720,
            mesh_show_back_face=True,
        )


# ── CLI ────────────────────────────────────────────────────────────────────

def main() -> None:
    here   = Path(__file__).resolve().parent
    def_in = here / "../docker-calib/captures/20260606_215951/filtered_ceiling.pcd"

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input", nargs="?", default=str(def_in), metavar="PCD",
                   help=f"Input point cloud (default: {def_in})")
    p.add_argument("-o", "--output", default=None, metavar="PLY",
                   help="Output mesh path (default: <input_dir>/surfaces_mesh.ply)")
    p.add_argument("--tile-size",   type=float, default=0.20, metavar="M",
                   help="Local fitting window side in metres (default 0.20)")
    p.add_argument("--tile-step",   type=float, default=0.05, metavar="M",
                   help="Grid vertex spacing in metres (default 0.05)")
    p.add_argument("--snap-thresh", type=float, default=0.10, metavar="M",
                   help="Snap boundary vertices within this distance of an "
                        "intersection line (default 0.10)")
    p.add_argument("--max-edge",    type=float, default=0.30, metavar="M",
                   help="Discard triangles with any edge > this length (default 0.30)")
    p.add_argument("--n-planes",    type=int,   default=5,
                   help="Max planes to detect (default 5)")
    p.add_argument("--dist-thresh", type=float, default=0.02, metavar="M",
                   help="RANSAC inlier distance in metres (default 0.02)")
    p.add_argument("--no-snap",     action="store_true",
                   help="Disable boundary snapping")
    p.add_argument("--no-viz",      action="store_true",
                   help="Skip the viewer")
    args = p.parse_args()

    in_path  = Path(args.input).resolve()
    out_path = Path(args.output).resolve() if args.output \
               else in_path.parent / "surfaces_mesh.ply"

    if not in_path.exists():
        sys.exit(f"ERROR: file not found: {in_path}")

    run(
        pcd_path    = in_path,
        out_path    = out_path,
        n_planes    = args.n_planes,
        dist_thresh = args.dist_thresh,
        tile_size   = args.tile_size,
        tile_step   = args.tile_step,
        snap_thresh = args.snap_thresh,
        max_edge    = args.max_edge,
        do_snap     = not args.no_snap,
        visualize   = not args.no_viz,
    )


if __name__ == "__main__":
    main()
