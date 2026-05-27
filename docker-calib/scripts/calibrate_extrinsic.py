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

Keys (live window):
  c   capture & solve
  s   save the last solve result to extrinsic.yaml
  r   reset capture (free up memory)
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
    return x, y, z


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
        """Return (holes_lidar (4,3), debug_image_bgr) or (None, debug_image_bgr)."""
        # 1. Crop to a coarse box where the board lives.
        mask = np.all((points >= self.target.crop_min) & (points <= self.target.crop_max), axis=1)
        cropped = points[mask]
        if len(cropped) < self.target.plane_min_inliers:
            return None, _placeholder_image(f"too few cropped pts: {len(cropped)}")

        # 2. RANSAC plane fit.
        plane, inliers = _ransac_plane(cropped, self.target.plane_dist, max_iter=200)
        if plane is None or len(inliers) < self.target.plane_min_inliers:
            n = 0 if inliers is None else len(inliers)
            return None, _placeholder_image(f"plane inliers: {n}")
        a, b, c, d = plane
        normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
        plane_pts = cropped[inliers]

        # 3. 2D in-plane coords (u,v).
        u_axis, v_axis = _orthonormal_basis(normal)
        origin = plane_pts.mean(0)
        rel = plane_pts - origin
        u = rel @ u_axis
        v = rel @ v_axis

        # 4. Rasterize to a density image. Cell size = hole_diameter / 4.
        cell = self.target.hole_diameter / 4.0
        u_min, u_max = u.min(), u.max()
        v_min, v_max = v.min(), v.max()
        W = max(int(np.ceil((u_max - u_min) / cell)), 8)
        H = max(int(np.ceil((v_max - v_min) / cell)), 8)
        density = np.zeros((H, W), dtype=np.float32)
        iu = np.clip(((u - u_min) / cell).astype(int), 0, W - 1)
        iv = np.clip(((v_max - v) / cell).astype(int), 0, H - 1)
        np.add.at(density, (iv, iu), 1.0)

        # 5. Smooth, then find low-density spots inside the board interior.
        # We erode the high-density mask so hole candidates on the board's edge
        # are excluded.
        density_blur = cv2.GaussianBlur(density, (5, 5), 0)
        plane_mask = (density_blur > 0).astype(np.uint8)
        plane_mask = cv2.morphologyEx(plane_mask, cv2.MORPH_CLOSE,
                                       np.ones((3, 3), np.uint8), iterations=2)
        interior = cv2.erode(plane_mask, np.ones((5, 5), np.uint8), iterations=2)

        # 6. Hole mask: interior AND density < threshold * local-mean.
        mean_density = density_blur[interior > 0].mean() if (interior > 0).any() else 0.0
        thr = mean_density * self.target.hole_density_thr
        hole_mask = ((density_blur < thr) & (interior > 0)).astype(np.uint8) * 255

        # 7. Connected components → take the 4 largest.
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(hole_mask, connectivity=8)
        if num <= 1:
            return None, _debug_image(density_blur, interior, hole_mask, centroids[1:], cell)
        # stats[0] is the background.
        areas = stats[1:, cv2.CC_STAT_AREA]
        order = np.argsort(areas)[::-1] + 1   # +1 to skip background label 0
        chosen = order[:4]
        if len(chosen) < 4:
            return None, _debug_image(density_blur, interior, hole_mask,
                                      centroids[chosen] if len(chosen) else np.empty((0, 2)),
                                      cell)
        hole_centers_uv = centroids[chosen]   # in (u_idx, v_idx) — i.e., (col, row)

        # 8. Lift back to 3D LiDAR frame.
        holes_3d = []
        for cu, cv_ in hole_centers_uv:
            u_m = u_min + cu * cell
            v_m = v_max - cv_ * cell
            p = origin + u_m * u_axis + v_m * v_axis
            holes_3d.append(p)
        holes_3d = np.array(holes_3d)

        # Debug image with hole centers drawn.
        dbg = _debug_image(density_blur, interior, hole_mask, hole_centers_uv, cell)
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
    _, _, Vt = np.linalg.svd(P - centroid)
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


def _debug_image(density, interior, hole_mask, centers_uv, cell):
    # Stack [density|interior|hole_mask] as a single BGR image.
    d = (density / max(density.max(), 1.0) * 255).astype(np.uint8)
    d = cv2.applyColorMap(d, cv2.COLORMAP_VIRIDIS)
    inter = cv2.cvtColor(interior * 255, cv2.COLOR_GRAY2BGR)
    holes = cv2.cvtColor(hole_mask, cv2.COLOR_GRAY2BGR)
    # Mark the centers on the density panel.
    for c in centers_uv:
        cu, cv_ = int(round(c[0])), int(round(c[1]))
        cv2.drawMarker(d, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.circle(d, (cu, cv_), int(round(0.04 / cell)), (0, 255, 255), 1)
    panel = np.hstack([d, inter, holes])
    # Upscale for readability.
    scale = max(1, 400 // panel.shape[0])
    panel = cv2.resize(panel, (panel.shape[1] * scale, panel.shape[0] * scale),
                       interpolation=cv2.INTER_NEAREST)
    return panel


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

        self.last_result = None    # (R, t, rms, n_correspondences)

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

    # --- core: capture + solve ---
    def solve_one(self):
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

        if holes_cam is None:
            self.get_logger().error("camera: failed to recover board pose "
                                    "(need >=2 known ArUco IDs detected)")
            return
        if holes_lidar is None or len(holes_lidar) < 4:
            self.get_logger().error("LiDAR: failed to extract 4 hole centers")
            return

        # Brute-force the best permutation (24).
        best = None
        for perm in itertools.permutations(range(4)):
            P = holes_lidar[list(perm)]
            R, t, rms = kabsch(P, holes_cam)
            if best is None or rms < best[2]:
                best = (R, t, rms, perm)
        R, t, rms, perm = best
        ex, ey, ez = rotmat_to_euler_xyz(R)
        self.get_logger().info(
            f"solved: rms={rms*1000:.2f}mm, perm={perm}, "
            f"trans={t}, euler_xyz_deg=({ex:.2f},{ey:.2f},{ez:.2f})")

        self.last_result = {
            "R": R, "t": t, "rms": rms, "perm": perm,
            "holes_cam": holes_cam, "holes_lidar_ordered": holes_lidar[list(perm)],
            "cam_overlay": cam_overlay, "lidar_dbg": lidar_dbg,
        }

    def save(self):
        if self.last_result is None:
            self.get_logger().error("nothing to save — capture & solve first")
            return
        R = self.last_result["R"]; t = self.last_result["t"]
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        out = {
            "T_camera_lidar": T.tolist(),
            "translation_xyz_m": t.tolist(),
            "rotation_euler_xyz_deg": list(rotmat_to_euler_xyz(R)),
            "rms_residual_m": float(self.last_result["rms"]),
            "correspondence_permutation": list(self.last_result["perm"]),
            "comment": "p_camera = R @ p_lidar + t",
        }
        with open(self.args.out, "w") as f:
            yaml.safe_dump(out, f, sort_keys=False)
        self.get_logger().info(f"wrote {self.args.out}")

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
            cv2.putText(img, "c=capture/solve  s=save  r=reset  q=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if self.last_result is not None:
                cv2.putText(img,
                            f"last RMS = {self.last_result['rms']*1000:.2f} mm",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                # Show the camera overlay with detected holes as the camera panel.
                cv2.imshow("camera", self.last_result["cam_overlay"])
                cv2.imshow("lidar dbg", self.last_result["lidar_dbg"])
            else:
                cv2.imshow("camera", img)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('c'):
                self.solve_one()
            elif k == ord('s'):
                self.save()
            elif k == ord('r'):
                self.last_result = None
                with self.lock:
                    self.cloud_buf.clear()
                self.get_logger().info("buffers cleared")
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
