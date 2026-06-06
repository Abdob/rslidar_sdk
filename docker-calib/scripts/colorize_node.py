#!/usr/bin/env python3
"""
Live colorized point cloud publisher.

Subscribes to:
  /image_raw       (sensor_msgs/Image)
  /rslidar_points  (sensor_msgs/PointCloud2)

Loads:
  /opt/calib/config/intrinsics.yaml  (fisheye K, D)
  /opt/calib/config/extrinsic.yaml   (T_camera_lidar)

For every incoming cloud:
  1. Pair it with the most recent image (by ROS time).
  2. Transform each point into the camera frame:  p_cam = R p_lidar + t
  3. Drop points behind the camera (z <= 0).
  4. Project with the fisheye model into image pixels.
  5. Drop points landing outside the image.
  6. Sample the pixel BGR -> pack as RGB into a PointCloud2 with an `rgb`
     float32 field (RViz/PCL convention) and republish.

Topic out: /colored_points (frame_id matches the input cloud's frame_id).

Capture:
  A preview window shows the live camera feed.
  Press SPACE or C to save the current image + LiDAR frame to:
    /opt/calib/captures/<YYYYMMDD_HHMMSS>/
        image.png
        lidar.pcd   (binary PCD v0.7, XYZI float32)
"""

import argparse
import datetime
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
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

CAPTURES_DIR = "/opt/calib/captures"


def pack_rgb_float(rgb_u8: np.ndarray) -> np.ndarray:
    """Pack (N,3) uint8 R,G,B into (N,) float32 with the PCL `rgb` convention."""
    # PCL stores rgb as a float32 whose underlying 32 bits are 0x00RRGGBB.
    r = rgb_u8[:, 0].astype(np.uint32)
    g = rgb_u8[:, 1].astype(np.uint32)
    b = rgb_u8[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32).copy()   # copy: ensure contiguous, writable


def _write_pcd(path: str, msg: PointCloud2) -> int:
    """Write a PointCloud2 message to a binary PCD v0.7 file. Returns point count."""
    offsets = {f.name: f.offset for f in msg.fields}
    has_i = "intensity" in offsets
    field_names = ["x", "y", "z", "intensity"] if has_i else ["x", "y", "z"]
    cols = 4 if has_i else 3

    # Build a structured dtype that reads only the fields we want, all as float32.
    # Avoids read_points_numpy which asserts all fields share the same datatype
    # (rslidar clouds have mixed types: float32 xyz/intensity, uint16 ring, etc.).
    dt = np.dtype({
        "names":   field_names,
        "formats": [np.float32] * len(field_names),
        "offsets": [offsets[n] for n in field_names],
        "itemsize": msg.point_step,
    })
    raw = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    pts = np.column_stack([raw[n] for n in field_names]).astype(np.float32)
    finite = np.isfinite(pts[:, :3]).all(axis=1)
    pts = pts[finite]

    n = len(pts)
    fields  = "x y z intensity" if has_i else "x y z"
    sizes   = "4 4 4 4"         if has_i else "4 4 4"
    types   = "F F F F"         if has_i else "F F F"
    counts  = "1 1 1 1"         if has_i else "1 1 1"
    header = (
        "# .PCD v0.7 - Point Cloud Data\n"
        "VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts[:, :cols].tobytes())
    return n


class Colorizer(Node):
    def __init__(self, args, K, D, fisheye, T):
        super().__init__("colorize_node")
        self.args = args
        self.K = K.astype(np.float64)
        self.D = D.astype(np.float64).reshape(-1)
        self.fisheye = fisheye
        self.R = T[:3, :3].astype(np.float64)
        self.t = T[:3, 3].astype(np.float64)
        self.bridge = CvBridge()

        self.lock = threading.Lock()
        self.latest_img: np.ndarray | None = None
        self.latest_img_size: tuple[int, int] | None = None  # (w, h)
        self.latest_cloud_msg: PointCloud2 | None = None

        # Session capture directory: one timestamped folder per run.
        session_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_dir = os.path.join(CAPTURES_DIR, session_ts)
        os.makedirs(self._session_dir, exist_ok=True)

        # Flash overlay: show "SAVED!" for this many seconds after a capture.
        self._flash_until: float = 0.0
        self._capture_count: int = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image,       args.image_topic, self._on_image, 10)
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, qos)
        self.pub = self.create_publisher(PointCloud2, args.out_topic, 10)

        # 30 Hz timer drives the preview window and keypress detection.
        self.create_timer(1.0 / 30.0, self._tick)

        cv2.namedWindow("colorize | SPACE or C to capture", cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f"Colorizer ready  in: {args.cloud_topic} + {args.image_topic}  "
            f"out: {args.out_topic}  fisheye={fisheye}")
        self.get_logger().info(
            f"Session dir: {self._session_dir}")
        self.get_logger().info(
            "Preview window open — press SPACE or C to capture a frame.")

    # ------------------------------------------------------------------
    def _on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        with self.lock:
            self.latest_img = img
            self.latest_img_size = (img.shape[1], img.shape[0])

    def _on_cloud(self, msg: PointCloud2):
        with self.lock:
            self.latest_cloud_msg = msg
            img = None if self.latest_img is None else self.latest_img
            size = self.latest_img_size
        if img is None:
            return

        # Read x,y,z as a single (N,3) array. Bypass pc2.read_points (per-point
        # Python loop, ~150 ms on 50k points) by reinterpreting the raw byte
        # buffer with np.frombuffer + a structured dtype. ~30x faster.
        offsets = {f.name: f.offset for f in msg.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            return
        dt = np.dtype({
            "names":   ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": msg.point_step,
        })
        cloud = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        pts = np.column_stack([cloud["x"], cloud["y"], cloud["z"]])  # (N, 3) float32
        # Drop NaN / inf rows in one vectorized pass.
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        if len(pts) == 0:
            return

        # Transform to camera frame: p_cam = R p_lidar + t
        pts_cam = pts.astype(np.float64) @ self.R.T + self.t   # (N,3)

        # Keep only points in front of the camera.
        front_mask = pts_cam[:, 2] > 1e-3
        if not np.any(front_mask):
            return
        pts_lidar_front = pts[front_mask]
        pts_cam_front   = pts_cam[front_mask]

        # Project (vectorized in cv2, fast).
        if self.fisheye:
            proj, _ = cv2.fisheye.projectPoints(
                pts_cam_front.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                self.K, self.D)
        else:
            proj, _ = cv2.projectPoints(
                pts_cam_front.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                self.K, self.D)
        uv = proj.reshape(-1, 2)
        w, h = size
        u = uv[:, 0]; v = uv[:, 1]
        in_img = (u >= 0) & (u < w) & (v >= 0) & (v < h) & np.isfinite(u) & np.isfinite(v)
        if not np.any(in_img):
            return

        pts_out = pts_lidar_front[in_img]
        u_in    = u[in_img].astype(np.int32)
        v_in    = v[in_img].astype(np.int32)

        # Sample colors. img is BGR; reorder to RGB for the PCL `rgb` field.
        bgr = img[v_in, u_in]                  # (M, 3)
        rgb_u8 = bgr[:, ::-1].astype(np.uint8)  # BGR -> RGB
        rgb_packed = pack_rgb_float(rgb_u8)    # (M,) float32

        # Build a structured (M,) array → PointCloud2 with xyz + rgb fields.
        # Use .tobytes() into msg.data directly — avoids the .tolist() roundtrip
        # in pc2.create_cloud which is the OTHER ~50 ms cost on big clouds.
        n = len(pts_out)
        cloud_struct = np.zeros(n, dtype=np.dtype({
            "names":   ["x", "y", "z", "rgb"],
            "formats": [np.float32, np.float32, np.float32, np.float32],
            "offsets": [0, 4, 8, 12],
            "itemsize": 16,
        }))
        cloud_struct["x"]   = pts_out[:, 0]
        cloud_struct["y"]   = pts_out[:, 1]
        cloud_struct["z"]   = pts_out[:, 2]
        cloud_struct["rgb"] = rgb_packed

        out = PointCloud2()
        out.header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        out.height = 1
        out.width = n
        out.fields = [
            PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * n
        out.is_dense = True
        out.data = cloud_struct.tobytes()
        self.pub.publish(out)

    # ------------------------------------------------------------------
    def _tick(self):
        """30 Hz: refresh preview window and check for keypresses."""
        with self.lock:
            display = self.latest_img.copy() if self.latest_img is not None else None

        if display is not None:
            cv2.putText(display, "SPACE / C : capture", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 0), 2, cv2.LINE_AA)
            if time.monotonic() < self._flash_until:
                cv2.putText(display, f"SAVED  {self._capture_count - 1:03d}", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 220, 0), 3, cv2.LINE_AA)
            cv2.imshow("colorize | SPACE or C to capture", display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord(" "), ord("c"), ord("C")):
            self._save_capture()

    def _save_capture(self):
        with self.lock:
            img = self.latest_img.copy() if self.latest_img is not None else None
            cloud_msg = self.latest_cloud_msg

        if img is None or cloud_msg is None:
            self.get_logger().warn(
                "Capture requested but image or cloud not yet available — try again.")
            return

        idx = f"{self._capture_count:03d}"
        img_path = os.path.join(self._session_dir, f"{idx}_image.png")
        pcd_path = os.path.join(self._session_dir, f"{idx}_lidar.pcd")

        cv2.imwrite(img_path, img)
        n_pts = _write_pcd(pcd_path, cloud_msg)

        self._capture_count += 1
        self._flash_until = time.monotonic() + 1.5
        self.get_logger().info(
            f"Capture {idx} saved to {self._session_dir}  ({n_pts} points)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image_topic", default="/image_raw")
    ap.add_argument("--cloud_topic", default="/rslidar_points")
    ap.add_argument("--out_topic",   default="/colored_points")
    ap.add_argument("--intrinsics",  default="/opt/calib/config/intrinsics.yaml")
    ap.add_argument("--extrinsic",   default="/opt/calib/config/extrinsic.yaml")
    args = ap.parse_args()

    with open(args.intrinsics) as f:
        intrinsics = yaml.safe_load(f)
    K = np.array(intrinsics["K"])
    D = np.array(intrinsics["D"])
    fisheye = (intrinsics.get("model", "fisheye") == "fisheye")

    with open(args.extrinsic) as f:
        ext = yaml.safe_load(f)
    T = np.array(ext["T_camera_lidar"])
    assert T.shape == (4, 4), f"bad extrinsic shape {T.shape}"

    rclpy.init()
    node = Colorizer(args, K, D, fisheye, T)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
