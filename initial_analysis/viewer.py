#!/usr/bin/env python3
"""
LiDAR-Camera Colorized Point Cloud Viewer

Projects LiDAR points into the camera image using the extrinsic calibration,
samples pixel colors to colorize the point cloud, and displays both the
annotated camera image and the 3D colored cloud side-by-side.

Keyboard Controls
-----------------
  Right Arrow / D   Next pair
  Left Arrow  / A   Previous pair
  U                 Toggle undistorted image (fisheye → pinhole)
  R                 Reset 3D camera view
  H                 Print this help
  Q / Esc           Quit
"""

from __future__ import annotations
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import yaml

# ── GLFW key codes used by open3d ──────────────────────────────────────────
KEY_RIGHT = 262
KEY_LEFT  = 263
KEY_U     = ord("U")
KEY_R     = ord("R")
KEY_H     = ord("H")
KEY_Q     = ord("Q")
KEY_D     = ord("D")
KEY_A     = ord("A")

WINDOW_3D  = "3D Colorized Point Cloud  [← / → to navigate]"
WINDOW_IMG = "Camera View"

HELP = """
Controls
--------
  Right Arrow / D   Next pair
  Left Arrow  / A   Previous pair
  U                 Toggle undistorted image
  R                 Reset 3D camera
  H                 Print help
  Q / Esc           Quit
"""


# ── Calibration ────────────────────────────────────────────────────────────

def load_calibration(config_dir: Path) -> dict:
    """Load extrinsic + fisheye intrinsic and precompute undistortion maps."""
    with open(config_dir / "extrinsic.yaml") as f:
        ext = yaml.safe_load(f)
    T = np.array(ext["T_camera_lidar"], dtype=np.float64)   # 4×4
    R, t = T[:3, :3], T[:3, 3]

    with open(config_dir / "intrinsics.yaml") as f:
        intr = yaml.safe_load(f)
    K  = np.array(intr["K"], dtype=np.float64)              # 3×3
    D  = np.array(intr["D"], dtype=np.float64).reshape(4, 1)
    sz = tuple(intr["image_size"])                           # (width, height)

    # New camera matrix for undistorted view (balance=0.5 keeps most pixels)
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, sz, np.eye(3, dtype=np.float64), balance=0.5
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3, dtype=np.float64), new_K, sz, cv2.CV_16SC2
    )
    return dict(R=R, t=t, K=K, D=D, new_K=new_K, map1=map1, map2=map2, sz=sz)


# ── Data loading ───────────────────────────────────────────────────────────

def find_pairs(captures_dir: Path) -> list[tuple[Path, Path]]:
    imgs = sorted(captures_dir.glob("*_image.png"))
    pairs = []
    for img_p in imgs:
        stem  = img_p.stem.replace("_image", "")
        pcd_p = captures_dir / f"{stem}_lidar.pcd"
        if pcd_p.exists():
            pairs.append((img_p, pcd_p))
    if not pairs:
        sys.exit(f"ERROR: no image/lidar pairs found in {captures_dir}")
    return pairs


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Return (N, 3) float64 XYZ from a PCD file."""
    cloud = o3d.io.read_point_cloud(str(path))
    pts   = np.asarray(cloud.points, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return pts


# ── Projection ─────────────────────────────────────────────────────────────

def _zero_pose() -> tuple[np.ndarray, np.ndarray]:
    return np.zeros((3, 1), np.float64), np.zeros((3, 1), np.float64)


def project_fisheye(pts_cam: np.ndarray, K, D) -> np.ndarray:
    """(N,3) camera-frame points → (N,2) pixel coords via fisheye model."""
    rvec, tvec = _zero_pose()
    uv, _ = cv2.fisheye.projectPoints(
        pts_cam.reshape(-1, 1, 3), rvec, tvec, K, D
    )
    return uv.reshape(-1, 2)


def project_pinhole(pts_cam: np.ndarray, K) -> np.ndarray:
    """(N,3) camera-frame points → (N,2) pixel coords, no distortion."""
    rvec, tvec = _zero_pose()
    uv, _ = cv2.projectPoints(
        pts_cam.reshape(-1, 1, 3), rvec, tvec, K, np.zeros((1, 5))
    )
    return uv.reshape(-1, 2)


def jet_bgr(values: np.ndarray) -> np.ndarray:
    """Vectorized jet colormap; returns (N, 3) uint8 BGR."""
    lo, hi = np.percentile(values, [5, 95])
    v = np.clip((values - lo) / (hi - lo + 1e-6), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4*v - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4*v - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4*v - 1), 0, 1)
    bgr = np.column_stack([b, g, r])
    return (bgr * 255).astype(np.uint8)


# ── Core colorization ──────────────────────────────────────────────────────

def colorize_pair(
    img_bgr: np.ndarray,
    pts_xyz: np.ndarray,
    calib: dict,
    undistorted: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Project LiDAR → image, sample pixel colors, build colored point cloud.

    Returns
    -------
    img_display : (H, W, 3) uint8 BGR  – image with depth-colored lidar overlay
    pcd_pts     : (N, 3) float64       – points in lidar frame
    pcd_colors  : (N, 3) float32       – normalized RGB (gray for invisible pts)
    """
    R, t     = calib["R"], calib["t"]
    K, D     = calib["K"], calib["D"]
    new_K    = calib["new_K"]
    map1, map2 = calib["map1"], calib["map2"]
    W, H     = calib["sz"]

    # Prepare display image
    if undistorted:
        img_show   = cv2.remap(img_bgr, map1, map2, cv2.INTER_LINEAR)
        use_fisheye = False
        proj_K      = new_K
    else:
        img_show   = img_bgr.copy()
        use_fisheye = True
        proj_K      = K

    pcd_colors = np.full((len(pts_xyz), 3), 0.30, dtype=np.float32)

    if len(pts_xyz) == 0:
        return img_show, pts_xyz, pcd_colors

    # Transform to camera frame  (comment in yaml: p_camera = R @ p_lidar + t)
    pts_cam = (R @ pts_xyz.T).T + t           # (N, 3)

    # Keep points in front of the camera
    front_mask  = pts_cam[:, 2] > 0.1
    pts_cam_f   = pts_cam[front_mask]         # (M, 3)

    if len(pts_cam_f) == 0:
        return img_show, pts_xyz, pcd_colors

    # Project to pixels
    uv = project_fisheye(pts_cam_f, proj_K, D) if use_fisheye \
         else project_pinhole(pts_cam_f, proj_K)

    ui = np.round(uv[:, 0]).astype(np.int32)
    vi = np.round(uv[:, 1]).astype(np.int32)

    in_bounds = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
    ui_v = ui[in_bounds]
    vi_v = vi[in_bounds]
    pts_cam_v = pts_cam_f[in_bounds]

    # Sample image colors for the point cloud (RGB, 0–1)
    img_rgb = cv2.cvtColor(img_show, cv2.COLOR_BGR2RGB)
    sampled  = img_rgb[vi_v, ui_v].astype(np.float32) / 255.0

    front_idx = np.where(front_mask)[0]
    valid_idx = front_idx[in_bounds]
    pcd_colors[valid_idx] = sampled

    # Draw depth-colored dots on the image (vectorized, 2×2 px blobs)
    depth_bgr = jet_bgr(pts_cam_v[:, 2])
    for dv, du, dc in ((0, 0, depth_bgr), (1, 0, depth_bgr), (0, 1, depth_bgr)):
        vv = np.clip(vi_v + dv, 0, H - 1)
        uu = np.clip(ui_v + du, 0, W - 1)
        img_show[vv, uu] = dc

    return img_show, pts_xyz, pcd_colors


# ── HUD overlay ────────────────────────────────────────────────────────────

def draw_hud(img: np.ndarray, idx: int, total: int, undist: bool) -> None:
    h, w = img.shape[:2]
    bar  = img.copy()
    cv2.rectangle(bar, (0, 0), (w, 32), (0, 0, 0), -1)
    cv2.addWeighted(bar, 0.55, img, 0.45, 0, img)
    mode = "Undistorted" if undist else "Raw (fisheye)"
    txt  = (f"Pair {idx + 1}/{total}  |  Image: {mode}  "
            f"|  [A/←] prev  [D/→] next  [U] toggle undist  [Q] quit")
    cv2.putText(img, txt, (8, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)


# ── Main viewer loop ───────────────────────────────────────────────────────

def run(captures_dir: Path, config_dir: Path) -> None:
    pairs = find_pairs(captures_dir)
    calib = load_calibration(config_dir)

    print(f"Loaded {len(pairs)} pairs from {captures_dir}")
    print(HELP)

    state = {"idx": 0, "undist": False, "dirty": True, "quit": False}

    # ── open3d window ──────────────────────────────────────────────────────
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(WINDOW_3D, width=1280, height=720)

    ropt = vis.get_render_option()
    ropt.background_color = np.array([0.07, 0.07, 0.07])
    ropt.point_size        = 2.0

    pcd  = o3d.geometry.PointCloud()
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(pcd)
    vis.add_geometry(axes)

    # Callbacks (open3d only fires these when the 3D window is focused)
    def go_next(v):
        state["idx"]   = (state["idx"] + 1) % len(pairs)
        state["dirty"] = True

    def go_prev(v):
        state["idx"]   = (state["idx"] - 1) % len(pairs)
        state["dirty"] = True

    def toggle_undist(v):
        state["undist"] = not state["undist"]
        state["dirty"]  = True

    def reset_cam(v):
        v.reset_view_point(True)

    def do_quit(v):
        state["quit"] = True

    vis.register_key_callback(KEY_RIGHT, go_next)
    vis.register_key_callback(KEY_LEFT,  go_prev)
    vis.register_key_callback(KEY_D,     go_next)
    vis.register_key_callback(KEY_A,     go_prev)
    vis.register_key_callback(KEY_U,     toggle_undist)
    vis.register_key_callback(KEY_R,     reset_cam)
    vis.register_key_callback(KEY_Q,     do_quit)
    vis.register_key_callback(KEY_H,     lambda v: print(HELP))

    # ── cv2 image window ───────────────────────────────────────────────────
    cv2.namedWindow(WINDOW_IMG, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_IMG, 960, 540)

    first_load = True

    while not state["quit"]:
        if state["dirty"]:
            img_path, pcd_path = pairs[state["idx"]]
            img_bgr = cv2.imread(str(img_path))
            pts_xyz = read_pcd_xyz(pcd_path)

            img_disp, pts, colors = colorize_pair(
                img_bgr, pts_xyz, calib, state["undist"]
            )
            draw_hud(img_disp, state["idx"], len(pairs), state["undist"])

            # Update point cloud
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(pcd)

            # Fit camera view on first load only
            if first_load:
                vis.reset_view_point(True)
                first_load = False

            cv2.imshow(WINDOW_IMG, img_disp)
            state["dirty"] = False

        vis.poll_events()
        vis.update_renderer()

        # Also handle keys from the cv2 window
        key = cv2.waitKey(10) & 0xFF
        if key in (ord("q"), 27):           # q or Esc
            break
        elif key in (83, ord("d")):         # right arrow or d
            go_next(None)
        elif key in (81, ord("a")):         # left arrow or a
            go_prev(None)
        elif key == ord("u"):
            toggle_undist(None)
        elif key == ord("r"):
            vis.reset_view_point(True)
        elif key == ord("h"):
            print(HELP)

    vis.destroy_window()
    cv2.destroyAllWindows()


# ── CLI ────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    repo      = Path(__file__).resolve().parent.parent
    def_caps  = repo / "docker-calib" / "captures" / "20260606_215951"
    def_cfg   = repo / "docker-calib" / "config"

    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "captures_dir", nargs="?", default=str(def_caps),
        help="Directory with *_image.png / *_lidar.pcd pairs "
             f"(default: {def_caps})",
    )
    p.add_argument(
        "--config", default=str(def_cfg), metavar="DIR",
        help="Directory containing extrinsic.yaml and intrinsics.yaml "
             f"(default: {def_cfg})",
    )
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(Path(args.captures_dir), Path(args.config))
