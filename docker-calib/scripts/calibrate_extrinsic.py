#!/usr/bin/env python3
"""
Camera <-> LiDAR extrinsic calibration following FAST-Calib's approach
(https://github.com/hku-mars/FAST-Calib).

Single-shot procedure:
  1. Camera detects 4 ArUco markers on the target board, recovers the board
     pose in the camera frame via fisheye PnP, then computes the 3D positions
     of the 4 hole centers (known by board geometry) in the camera frame.
  2. LiDAR accumulates N frames, RANSACs the dominant plane (the board),
     projects inliers to 2D plane coords, finds the 4 low-density regions
     (the holes), and reports their 3D centers in the LiDAR frame.
  3. Brute-forces the 24 permutations of the 4 LiDAR holes against the 4
     camera holes; the permutation with the smallest Kabsch residual wins.

Output (extrinsic.yaml): T_camera_lidar — a 4x4 matrix such that
    p_camera = R @ p_lidar + t

Multi-pose: each `c` adds the current board view as one pose, then the extrinsic
is re-solved GLOBALLY over every accumulated pose (one Kabsch over all hole
correspondences). Capture the board at many depths / tilts / positions (~10-20)
so the points are no longer coplanar and per-hole noise averages out -- this is
what drives the residual from cm down to mm. Per-pose residuals under the global
fit are reported so you can spot and `u`ndo a bad capture.

Keys (live window):
  c   capture the current board pose, add it, and re-solve globally
  u   undo: remove the last captured pose and re-solve
  s   save the global solve to extrinsic.yaml
  r   reset: drop all captured poses
  q   quit
"""

import argparse
import itertools
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial import cKDTree
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


# ---------- helpers ----------

ARUCO_DICTS = {name: getattr(cv2.aruco, name) for name in dir(cv2.aruco) if name.startswith("DICT_")}


def kabsch(P: np.ndarray, Q: np.ndarray):
    """Find R, t such that Q ~= R @ P + t (least-squares rigid).

    P, Q: (N, 3). Returns (R 3x3, t 3, rms-residual-meters).
    """
    assert P.shape == Q.shape and P.shape[1] == 3 and len(P) >= 3
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = mq - R @ mp
    res = Q - (P @ R.T + t)
    rms = float(np.sqrt((res * res).sum() / len(P)))
    return R, t, rms


def rotmat_to_euler_xyz(R: np.ndarray):
    """Intrinsic XYZ Euler angles (degrees) — for human-readable logging."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-6:
        x = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        y = np.degrees(np.arctan2(-R[2, 0], sy))
        z = 0.0
    else:
        x = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        y = np.degrees(np.arctan2(-R[2, 0], sy))
        z = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return float(x), float(y), float(z)


# ---------- target geometry ----------

class TargetGeometry:
    def __init__(self, cfg: dict):
        self.board_w = float(cfg["board"]["width"])
        self.board_h = float(cfg["board"]["height"])

        aruco_cfg = cfg["aruco"]
        self.marker_size = float(aruco_cfg["marker_size"])
        dict_name = aruco_cfg["dict"]
        if dict_name not in ARUCO_DICTS:
            raise ValueError(f"unknown ArUco dict {dict_name}")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        # Marker corner 3D positions in board frame.
        # For each marker, corners are TL, TR, BR, BL (OpenCV convention).
        self.marker_corners_board: dict[int, np.ndarray] = {}
        half = self.marker_size / 2.0
        for m in aruco_cfg["markers"]:
            cx, cy = m["center"]
            mid = int(m["id"])
            self.marker_corners_board[mid] = np.array([
                [cx - half, cy + half, 0.0],   # TL
                [cx + half, cy + half, 0.0],   # TR
                [cx + half, cy - half, 0.0],   # BR
                [cx - half, cy - half, 0.0],   # BL
            ], dtype=np.float64)

        # Hole centers in board frame.
        holes = cfg["holes"]
        self.hole_diameter = float(holes["diameter"])
        self.holes_board = np.array(
            [[x, y, 0.0] for x, y in holes["centers"]], dtype=np.float64)

        # LiDAR detection tuning
        ld = cfg.get("lidar_detect", {})
        self.crop_min = np.array(ld.get("crop_xyz_min", [-10, -10, -10]), dtype=np.float64)
        self.crop_max = np.array(ld.get("crop_xyz_max", [ 10,  10,  10]), dtype=np.float64)
        self.plane_dist = float(ld.get("plane_ransac_distance", 0.02))
        self.plane_min_inliers = int(ld.get("plane_min_inliers", 1500))
        self.accumulate_frames = int(ld.get("accumulate_frames", 10))
        self.hole_density_thr = float(ld.get("hole_density_threshold", 0.35))
        self.cluster_eps = float(ld.get("cluster_eps", 0.04))
        self.cluster_min_samples = int(ld.get("cluster_min_samples", 4))


# ---------- camera-side detection ----------

class CameraDetector:
    def __init__(self, target: TargetGeometry, K: np.ndarray, D: np.ndarray, fisheye: bool):
        self.target = target
        self.K = K.astype(np.float64)
        self.D = D.astype(np.float64).reshape(-1)
        self.fisheye = fisheye

    def detect(self, img_bgr: np.ndarray):
        """Return (holes_in_camera (4,3), board_pose (R,t), overlay_image) or None."""
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.target.aruco_detector.detectMarkers(gray)
        overlay = img_bgr.copy()
        if ids is None:
            return None, None, overlay
        ids = ids.flatten().tolist()

        # Build (image_pts, board_pts) for every detected marker we know about.
        img_pts = []
        board_pts = []
        for det_idx, mid in enumerate(ids):
            if mid not in self.target.marker_corners_board:
                continue
            img_pts.append(corners[det_idx].reshape(-1, 2))         # (4,2)
            board_pts.append(self.target.marker_corners_board[mid])  # (4,3)
        if len(img_pts) < 2:
            cv2.aruco.drawDetectedMarkers(overlay, corners, np.array(ids).reshape(-1, 1))
            return None, None, overlay
        img_pts  = np.concatenate(img_pts, axis=0).astype(np.float64)      # (N,2)
        board_pts = np.concatenate(board_pts, axis=0).astype(np.float64)    # (N,3)

        # Fisheye PnP: undistort image points to normalized coords, then solve
        # PnP with K=I, D=0 against those normalized coords.
        if self.fisheye:
            und = cv2.fisheye.undistortPoints(
                img_pts.reshape(-1, 1, 2), self.K, self.D)            # normalized (u,v) on z=1
            obj_pts = board_pts.reshape(-1, 1, 3)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, und, np.eye(3), np.zeros(4),
                flags=cv2.SOLVEPNP_ITERATIVE)
        else:
            ok, rvec, tvec = cv2.solvePnP(
                board_pts.reshape(-1, 1, 3),
                img_pts.reshape(-1, 1, 2),
                self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE)

        if not ok:
            cv2.aruco.drawDetectedMarkers(overlay, corners, np.array(ids).reshape(-1, 1))
            return None, None, overlay

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3)

        # Hole centers in camera frame.
        holes_cam = (R @ self.target.holes_board.T + t.reshape(3, 1)).T   # (4,3)

        # Build overlay: draw markers, board outline, hole projections.
        cv2.aruco.drawDetectedMarkers(overlay, corners, np.array(ids).reshape(-1, 1))
        # Project corners of the board outline.
        corners_board = np.array([
            [0, 0, 0],
            [self.target.board_w, 0, 0],
            [self.target.board_w, self.target.board_h, 0],
            [0, self.target.board_h, 0],
        ], dtype=np.float64)
        proj_corners = self._project(corners_board, R, t)
        if proj_corners is not None:
            pts = proj_corners.astype(int)
            for i in range(4):
                cv2.line(overlay, tuple(pts[i]), tuple(pts[(i + 1) % 4]), (0, 255, 255), 2)
        proj_holes = self._project(self.target.holes_board, R, t)
        if proj_holes is not None:
            for p in proj_holes.astype(int):
                cv2.circle(overlay, tuple(p), 12, (0, 255, 0), 2)
                cv2.circle(overlay, tuple(p), 3,  (0, 255, 0), -1)
        return holes_cam, (R, t), overlay

    def _project(self, pts_board, R, t):
        """Project board-frame 3D points to image; returns (N,2) or None."""
        if len(pts_board) == 0:
            return None
        pts_cam = (R @ pts_board.T + t.reshape(3, 1)).T
        if (pts_cam[:, 2] <= 0).any():
            return None
        rvec, _ = cv2.Rodrigues(R)
        if self.fisheye:
            proj, _ = cv2.fisheye.projectPoints(
                pts_cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                self.K, self.D)
        else:
            proj, _ = cv2.projectPoints(
                pts_cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                self.K, self.D)
        return proj.reshape(-1, 2)


# ---------- LiDAR-side detection ----------

class LidarDetector:
    def __init__(self, target: TargetGeometry):
        self.target = target

    def detect(self, points: np.ndarray):
        """Return (holes_lidar (4,3), debug_image_bgr) or (None, debug_image_bgr).

        Algorithm follows FAST-Calib (lidar_detect.hpp::detect_solid_lidar):
          1. Crop, RANSAC plane, project to 2D in the plane.
          2. Boundary-point extraction: a point is on a boundary if its
             neighbors within `boundary_radius` leave an angular gap > pi/4
             (i.e. one side is empty -> hole edge or board outer edge).
          3. Euclidean-cluster the boundary points.
          4. Fit a 2D circle to each cluster; keep only those whose radius
             matches the expected hole radius and whose fit error is small.
          5. Among candidate circles, find the 4 whose pairwise distances
             match the expected hole rectangle (2 sides + 2 short + 2 diag).
        """
        # 1. Crop.
        mask = np.all((points >= self.target.crop_min) & (points <= self.target.crop_max), axis=1)
        cropped = points[mask]
        if len(cropped) < self.target.plane_min_inliers:
            return None, _placeholder_image(f"too few cropped pts: {len(cropped)}")

        # 2. RANSAC plane fit.
        plane, inliers = _ransac_plane(cropped, self.target.plane_dist, max_iter=200)
        if plane is None or len(inliers) < self.target.plane_min_inliers:
            n = 0 if inliers is None else len(inliers)
            return None, _placeholder_image(f"plane inliers: {n}")
        normal = np.array(plane[:3])
        normal /= np.linalg.norm(normal)
        plane_pts = cropped[inliers]

        # 3. 2D in-plane coords (u, v).
        u_axis, v_axis = _orthonormal_basis(normal)
        origin = plane_pts.mean(0)
        rel = plane_pts - origin
        pts_2d_full = np.column_stack([rel @ u_axis, rel @ v_axis]).astype(np.float64)
        # Voxel-downsample in 2D so the density becomes uniform (instead of
        # following the LiDAR's scan-ring pattern). 1.5 cm cells keep enough
        # detail to fit 24 cm hole circles while flattening intra-ring density.
        pts_2d = _voxel_downsample_2d(pts_2d_full, leaf=0.015)

        # 4. Boundary points.
        # The RSAIRY's scan pattern forms a sparse 2D lattice (rings spaced
        # several cm apart on the board's plane). Even an interior point has
        # gaps of ~60° between lattice neighbors, so the FAST-Calib default
        # threshold of π/4 (45°) flags everything. We need a stricter threshold:
        # only points whose neighborhood is *missing* a half-plane should count
        # (hole edge, board outer edge). 2π/3 ~= 120° works in practice — bigger
        # than any lattice gap, smaller than the ~180° you get at a real edge.
        boundary_radius = max(0.10, self.target.hole_diameter * 0.5)
        boundary_mask = _detect_boundary_points(
            pts_2d, radius=boundary_radius, angle_thr=2 * np.pi / 3)
        boundary_pts = pts_2d[boundary_mask]
        if len(boundary_pts) < 20:
            return None, _build_dbg_image(
                pts_2d, boundary_pts, [], None,
                msg=f"only {len(boundary_pts)} boundary pts")

        # 5. Euclidean cluster the boundary points.
        clusters = _euclidean_cluster_2d(boundary_pts, eps=0.06, min_samples=5)

        # 6. Fit a circle to each cluster; keep ones matching the expected hole.
        expected_r = self.target.hole_diameter / 2.0
        candidates = []     # list of (cx, cy, r, mean_err, cluster_pts)
        for cluster in clusters:
            cx, cy, r, err = _fit_circle_2d(cluster)
            if r is None:
                continue
            # Loose-ish radius tolerance: ±30% of expected radius.
            if abs(r - expected_r) > expected_r * 0.3:
                continue
            # Fit error: mean radial residual must be < ~hole radius / 6.
            if err > expected_r / 6.0:
                continue
            candidates.append((cx, cy, r, err, cluster))

        if len(candidates) < 4:
            return None, _build_dbg_image(
                pts_2d, boundary_pts, candidates, None,
                msg=f"only {len(candidates)} circle candidates")

        # 7. Pick the 4 whose pairwise distances best match the expected pattern.
        # Tolerance is loose because LiDAR scan-ring sparsity often biases the
        # circle fit by a few cm (boundary points cluster on one arc).
        expected_dists = _expected_pairwise_distances(self.target.holes_board)
        best, score = _find_best_4_holes(candidates, expected_dists, tol=0.15)
        if best is None:
            return None, _build_dbg_image(
                pts_2d, boundary_pts, candidates, None,
                msg=f"no 4-tuple fit (best mismatch {score*1000:.0f} mm)")

        # 8. Lift back to 3D LiDAR frame.
        holes_3d = np.array([
            origin + cx * u_axis + cy * v_axis for (cx, cy, _r, _e, _c) in best
        ])
        dbg = _build_dbg_image(
            pts_2d, boundary_pts, candidates, best,
            msg=f"OK  mismatch={score*1000:.0f} mm")
        return holes_3d, dbg


def _ransac_plane(points: np.ndarray, dist_thr: float, max_iter: int = 200):
    """Tiny RANSAC plane fit. Returns ((a,b,c,d), inlier_idx)."""
    n = len(points)
    if n < 3:
        return None, None
    best_inliers = None
    best_count = 0
    best_plane = None
    rng = np.random.default_rng(0)
    for _ in range(max_iter):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]
        v1, v2 = p1 - p0, p2 - p0
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal /= norm
        d = -normal.dot(p0)
        dists = np.abs(points @ normal + d)
        inliers = np.where(dists < dist_thr)[0]
        if len(inliers) > best_count:
            best_count = len(inliers)
            best_inliers = inliers
            best_plane = (normal[0], normal[1], normal[2], d)
    if best_plane is None:
        return None, None
    # Refit: least-squares plane from inliers.
    P = points[best_inliers]
    centroid = P.mean(0)
    # full_matrices=False so U stays (N,3) — otherwise it's (N,N) and explodes
    # for large inlier counts (e.g. 100k pts -> 80 GiB U matrix).
    _, _, Vt = np.linalg.svd(P - centroid, full_matrices=False)
    normal = Vt[-1]
    d = -normal.dot(centroid)
    dists = np.abs(points @ normal + d)
    inliers = np.where(dists < dist_thr)[0]
    return (normal[0], normal[1], normal[2], d), inliers


def _orthonormal_basis(normal: np.ndarray):
    """Two orthonormal vectors spanning the plane orthogonal to `normal`."""
    n = normal / np.linalg.norm(normal)
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper); u /= np.linalg.norm(u)
    v = np.cross(n, u);      v /= np.linalg.norm(v)
    return u, v


def _placeholder_image(msg: str):
    img = np.zeros((400, 800, 3), dtype=np.uint8)
    cv2.putText(img, msg, (15, 200), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (255, 255, 255), 2)
    return img


def _voxel_downsample_2d(pts: np.ndarray, leaf: float) -> np.ndarray:
    """Keep one point per `leaf`-sized 2D cell. Order is deterministic."""
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int64)
    # Pack 2D integer grid keys into 1D for unique-detection.
    packed = keys[:, 0] * 2_000_003 + keys[:, 1]
    _, idx = np.unique(packed, return_index=True)
    return pts[np.sort(idx)]


def _detect_boundary_points(pts_2d: np.ndarray, radius: float, angle_thr: float):
    """For each point, decide if it sits on a boundary.

    A point is a boundary point if, looking at its neighbors within `radius`,
    the largest angular gap (around the point, in 2D) exceeds `angle_thr`.
    That's the FAST-Calib criterion (and PCL's BoundaryEstimation).
    """
    tree = cKDTree(pts_2d)
    indices_list = tree.query_ball_point(pts_2d, r=radius)
    boundary = np.zeros(len(pts_2d), dtype=bool)
    for i, nbrs in enumerate(indices_list):
        if len(nbrs) < 4:
            continue
        diffs = pts_2d[nbrs] - pts_2d[i]
        mag = np.hypot(diffs[:, 0], diffs[:, 1])
        sel = mag > 1e-6           # drop self
        if sel.sum() < 3:
            continue
        angles = np.sort(np.arctan2(diffs[sel, 1], diffs[sel, 0]))
        gaps = np.diff(angles)
        wrap = 2 * np.pi + angles[0] - angles[-1]
        if max(gaps.max() if len(gaps) else 0.0, wrap) > angle_thr:
            boundary[i] = True
    return boundary


def _euclidean_cluster_2d(pts: np.ndarray, eps: float, min_samples: int):
    """BFS-based Euclidean clustering. Returns a list of (M_i, 2) arrays."""
    if len(pts) == 0:
        return []
    tree = cKDTree(pts)
    visited = np.zeros(len(pts), dtype=bool)
    clusters = []
    for seed in range(len(pts)):
        if visited[seed]:
            continue
        stack = [seed]
        members = []
        while stack:
            j = stack.pop()
            if visited[j]:
                continue
            visited[j] = True
            members.append(j)
            for k in tree.query_ball_point(pts[j], r=eps):
                if not visited[k]:
                    stack.append(k)
        if len(members) >= min_samples:
            clusters.append(pts[members])
    return clusters


def _fit_circle_2d(pts: np.ndarray):
    """Algebraic (Kåsa) least-squares 2D circle fit.

    Returns (cx, cy, r, mean_radial_error) or (None, None, None, None).
    """
    if len(pts) < 3:
        return None, None, None, None
    x = pts[:, 0]; y = pts[:, 1]
    # x^2 + y^2 + A x + B y + C = 0  =>  A x + B y + C = -(x^2 + y^2)
    M = np.column_stack([x, y, np.ones_like(x)])
    b = -(x * x + y * y)
    try:
        sol, *_ = np.linalg.lstsq(M, b, rcond=None)
    except np.linalg.LinAlgError:
        return None, None, None, None
    A_, B_, C_ = sol
    cx = -A_ / 2.0
    cy = -B_ / 2.0
    r2 = cx * cx + cy * cy - C_
    if r2 <= 0:
        return None, None, None, None
    r = float(np.sqrt(r2))
    err = float(np.mean(np.abs(np.hypot(x - cx, y - cy) - r)))
    return float(cx), float(cy), r, err


def _expected_pairwise_distances(holes_board: np.ndarray):
    """Sorted pairwise distances among the 4 hole centers, in board frame."""
    n = len(holes_board)
    dists = []
    for i in range(n):
        for j in range(i + 1, n):
            dists.append(np.linalg.norm(holes_board[i] - holes_board[j]))
    return np.sort(np.array(dists))


def _find_best_4_holes(candidates, expected_dists, tol: float):
    """Find the 4 candidate circles whose pairwise distances best match
    the target's hole-rectangle pattern. Returns (best_4, max_mismatch_m)
    or (None, None) if no 4-tuple is within `tol`."""
    import itertools
    n = len(candidates)
    if n < 4:
        return None, None
    expected = np.sort(np.asarray(expected_dists))
    best = None
    best_score = float("inf")
    for combo in itertools.combinations(range(n), 4):
        pts = np.array([(candidates[i][0], candidates[i][1]) for i in combo])
        d = []
        for i in range(4):
            for j in range(i + 1, 4):
                d.append(np.hypot(pts[i, 0] - pts[j, 0], pts[i, 1] - pts[j, 1]))
        d = np.sort(np.array(d))
        score = float(np.max(np.abs(d - expected)))
        if score < best_score:
            best_score = score
            best = [candidates[i] for i in combo]
    if best_score > tol:
        return None, best_score
    return best, best_score


def _build_dbg_image(pts_2d, boundary_pts, candidates, selected, msg: str = ""):
    """Render a 2D scatter of the in-plane points with boundary/candidates/selected."""
    if len(pts_2d) == 0:
        return _placeholder_image("no plane points")
    pad = 0.10
    u_min = pts_2d[:, 0].min() - pad
    u_max = pts_2d[:, 0].max() + pad
    v_min = pts_2d[:, 1].min() - pad
    v_max = pts_2d[:, 1].max() + pad
    # 250 px/m => 4 mm/px. Cap so the window doesn't get crazy big.
    ppm = 250.0
    W = min(int((u_max - u_min) * ppm), 1800)
    H = min(int((v_max - v_min) * ppm), 1000)
    if W < 100 or H < 100:
        return _placeholder_image("plane extent too small")
    img = np.zeros((H, W, 3), dtype=np.uint8)

    def to_px(p):
        x = int((p[0] - u_min) * ppm)
        y = H - 1 - int((p[1] - v_min) * ppm)
        return x, y

    # All plane points (dim).
    pix = ((pts_2d - [u_min, v_min]) * ppm).astype(int)
    pix[:, 1] = H - 1 - pix[:, 1]
    inb = (pix[:, 0] >= 0) & (pix[:, 0] < W) & (pix[:, 1] >= 0) & (pix[:, 1] < H)
    img[pix[inb, 1], pix[inb, 0]] = (90, 90, 90)

    # Boundary points (red).
    if len(boundary_pts):
        bpix = ((boundary_pts - [u_min, v_min]) * ppm).astype(int)
        bpix[:, 1] = H - 1 - bpix[:, 1]
        inb = (bpix[:, 0] >= 0) & (bpix[:, 0] < W) & (bpix[:, 1] >= 0) & (bpix[:, 1] < H)
        for x, y in bpix[inb]:
            cv2.circle(img, (int(x), int(y)), 1, (0, 0, 255), -1)

    # All candidate circles (yellow rings).
    for (cx, cy, r, _err, _c) in candidates:
        c = to_px((cx, cy))
        cv2.circle(img, c, int(r * ppm), (0, 200, 200), 1, cv2.LINE_AA)

    # Selected 4 (bright green rings + center cross).
    if selected is not None:
        for (cx, cy, r, _err, _c) in selected:
            c = to_px((cx, cy))
            cv2.circle(img, c, int(r * ppm), (0, 255, 0), 2, cv2.LINE_AA)
            cv2.drawMarker(img, c, (0, 255, 0), cv2.MARKER_CROSS, 18, 2)

    # Status text.
    label = (f"plane={len(pts_2d)}  boundary={len(boundary_pts)}  "
             f"candidates={len(candidates)}  selected={4 if selected else 0}")
    if msg:
        label += f"   [{msg}]"
    cv2.putText(img, label, (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


# ---------- main ROS node ----------

class ExtrinsicCalibrator(Node):
    def __init__(self, args, target, intrinsics):
        super().__init__("calibrate_extrinsic")
        self.args = args
        self.target = target
        self.bridge = CvBridge()
        self.cam_det = CameraDetector(target,
                                       np.array(intrinsics["K"]),
                                       np.array(intrinsics["D"]),
                                       fisheye=(intrinsics["model"] == "fisheye"))
        self.lidar_det = LidarDetector(target)

        self.lock = threading.Lock()
        self.latest_img = None
        self.cloud_buf: list[np.ndarray] = []  # rolling buffer of recent clouds

        # The AIRY publisher uses default (reliable) — match.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image,        args.image_topic, self._on_image, 10)
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, qos)

        # multi-pose accumulation: each capture appends one pose's 4 ordered
        # hole correspondences; the extrinsic is re-solved globally over all of
        # them (see capture_and_add / solve_global).
        self.captures: list[dict] = []   # [{"lidar": (4,3), "cam": (4,3), "pose_rms"}]
        self.last_capture = None         # debug images from the most recent capture
        self.global_result = None        # {"R","t","rms","per_pose","n"}

    def _on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        with self.lock:
            self.latest_img = img

    def _on_cloud(self, msg: PointCloud2):
        # Read x, y, z fields only.
        pts = np.array([
            (p[0], p[1], p[2]) for p in pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float32)
        if len(pts) == 0:
            return
        with self.lock:
            self.cloud_buf.append(pts)
            if len(self.cloud_buf) > self.target.accumulate_frames:
                self.cloud_buf.pop(0)

    # --- core: capture one pose, then re-solve over all poses ---
    def capture_and_add(self):
        """Detect the board in the current frame and add it as one pose."""
        with self.lock:
            img = None if self.latest_img is None else self.latest_img.copy()
            clouds = list(self.cloud_buf)
        if img is None:
            self.get_logger().error("no image yet")
            return
        if len(clouds) < max(1, self.target.accumulate_frames // 2):
            self.get_logger().error(f"only {len(clouds)} cloud frames accumulated")
            return

        holes_cam, board_pose, cam_overlay = self.cam_det.detect(img)
        cloud = np.vstack(clouds)
        holes_lidar, lidar_dbg = self.lidar_det.detect(cloud)

        # Keep debug images even on failure so the user can see *why* it failed.
        self.last_capture = {"cam_overlay": cam_overlay, "lidar_dbg": lidar_dbg,
                             "ok": False}

        if holes_cam is None:
            self.get_logger().error("camera: failed to recover board pose "
                                    "(need >=2 known ArUco IDs detected)")
            return
        if holes_lidar is None or len(holes_lidar) < 4:
            self.get_logger().error("LiDAR: failed to extract 4 hole centers "
                                    "(check the dbg window — likely RANSAC picked "
                                    "a wall/floor; tighten crop_xyz_min/max)")
            return

        # Per-shot permutation: the camera holes are always in fixed board order,
        # so brute-force the 24 orderings of the 4 LiDAR holes to find which
        # LiDAR hole corresponds to which board hole for THIS view.
        best = None
        for perm in itertools.permutations(range(4)):
            P = holes_lidar[list(perm)]
            _R, _t, rms = kabsch(P, holes_cam)
            if best is None or rms < best[2]:
                best = (_R, _t, rms, perm)
        _, _, pose_rms, perm = best
        lidar_ordered = holes_lidar[list(perm)]   # reordered into board order

        self.captures.append({
            "lidar": lidar_ordered, "cam": holes_cam,
            "pose_rms": float(pose_rms), "perm": perm,
        })
        self.last_capture["ok"] = True
        self.get_logger().info(
            f"added pose #{len(self.captures)} (per-pose rms={pose_rms*1000:.1f} mm, "
            f"perm={perm})")
        self.solve_global()

    def solve_global(self):
        """Re-solve the extrinsic over EVERY accumulated pose at once."""
        if not self.captures:
            self.global_result = None
            return
        P = np.vstack([c["lidar"] for c in self.captures])   # (4*N, 3) LiDAR
        Q = np.vstack([c["cam"] for c in self.captures])     # (4*N, 3) camera
        R, t, rms = kabsch(P, Q)

        # Residual of each pose under the single global transform (mm) — a high
        # one flags a bad detection worth `u`ndoing.
        per_pose = []
        for c in self.captures:
            d = c["cam"] - (c["lidar"] @ R.T + t)
            per_pose.append(float(np.sqrt((d * d).sum() / len(d))))

        self.global_result = {"R": R, "t": t, "rms": rms,
                              "per_pose": per_pose, "n": len(self.captures)}
        ex, ey, ez = rotmat_to_euler_xyz(R)
        worst = max(per_pose)
        self.get_logger().info(
            f"GLOBAL: poses={len(self.captures)} pts={len(P)} "
            f"rms={rms*1000:.2f}mm worst-pose={worst*1000:.1f}mm  "
            f"euler_xyz_deg=({ex:.2f},{ey:.2f},{ez:.2f}) t={np.round(t,4).tolist()}")

    def undo(self):
        """Drop the most recently captured pose and re-solve."""
        if not self.captures:
            self.get_logger().info("nothing to undo")
            return
        self.captures.pop()
        self.get_logger().info(f"removed last pose ({len(self.captures)} remaining)")
        self.solve_global()

    def reset(self):
        self.captures.clear()
        self.global_result = None
        with self.lock:
            self.cloud_buf.clear()
        self.get_logger().info("cleared all captured poses")

    def save(self):
        if self.global_result is None:
            self.get_logger().error("nothing to save — no valid poses captured yet")
            return
        R = self.global_result["R"]; t = self.global_result["t"]
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        out = {
            "T_camera_lidar": T.tolist(),
            "translation_xyz_m": t.tolist(),
            "rotation_euler_xyz_deg": list(rotmat_to_euler_xyz(R)),
            "rms_residual_m": float(self.global_result["rms"]),
            "num_poses": int(self.global_result["n"]),
            "num_correspondences": int(self.global_result["n"] * 4),
            "per_pose_rms_m": [float(x) for x in self.global_result["per_pose"]],
            "comment": "p_camera = R @ p_lidar + t  (multi-pose global Kabsch)",
        }
        with open(self.args.out, "w") as f:
            yaml.safe_dump(out, f, sort_keys=False)
        self.get_logger().info(
            f"wrote {self.args.out}  ({self.global_result['n']} poses, "
            f"rms={self.global_result['rms']*1000:.2f} mm)")

    def loop(self):
        cv2.namedWindow("camera",    cv2.WINDOW_NORMAL); cv2.resizeWindow("camera", 960, 540)
        cv2.namedWindow("lidar dbg", cv2.WINDOW_NORMAL); cv2.resizeWindow("lidar dbg", 1200, 400)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            with self.lock:
                img = None if self.latest_img is None else self.latest_img.copy()
            if img is None:
                time.sleep(0.02)
                continue
            cv2.putText(img, "c=capture+add  u=undo  s=save  r=reset  q=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # Running global-solve status across all captured poses.
            if self.global_result is not None:
                gr = self.global_result
                msg = (f"poses={gr['n']}  global RMS={gr['rms']*1000:.2f} mm  "
                       f"worst-pose={max(gr['per_pose'])*1000:.1f} mm")
                color = (0, 200, 255)
            else:
                msg = f"poses={len(self.captures)}  (capture some board views)"
                color = (0, 200, 255)
            cv2.putText(img, msg, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            # Show the most recent capture's overlays (or the live image).
            if self.last_capture is not None:
                if not self.last_capture["ok"]:
                    cv2.putText(img, "last capture FAILED (see lidar dbg)",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("camera", self.last_capture["cam_overlay"])
                cv2.imshow("lidar dbg", self.last_capture["lidar_dbg"])
            else:
                cv2.imshow("camera", img)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('c'):
                self.capture_and_add()
            elif k == ord('u'):
                self.undo()
            elif k == ord('s'):
                self.save()
            elif k == ord('r'):
                self.reset()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image_topic", default="/image_raw")
    ap.add_argument("--cloud_topic", default="/rslidar_points")
    ap.add_argument("--target",      default="/opt/calib/config/target.yaml")
    ap.add_argument("--intrinsics",  default="/opt/calib/config/intrinsics.yaml")
    ap.add_argument("--out",         default="/opt/calib/config/extrinsic.yaml")
    args = ap.parse_args()

    with open(args.target) as f:
        target = TargetGeometry(yaml.safe_load(f))
    with open(args.intrinsics) as f:
        intrinsics = yaml.safe_load(f)

    rclpy.init()
    node = ExtrinsicCalibrator(args, target, intrinsics)
    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
