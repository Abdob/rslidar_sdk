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
"""

import argparse
import struct
import threading

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def pack_rgb_float(rgb_u8: np.ndarray) -> np.ndarray:
    """Pack (N,3) uint8 R,G,B into (N,) float32 with the PCL `rgb` convention."""
    # PCL stores rgb as a float32 whose underlying 32 bits are 0x00RRGGBB.
    r = rgb_u8[:, 0].astype(np.uint32)
    g = rgb_u8[:, 1].astype(np.uint32)
    b = rgb_u8[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32).copy()   # copy: ensure contiguous, writable


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

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image,       args.image_topic, self._on_image, 10)
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, qos)
        self.pub = self.create_publisher(PointCloud2, args.out_topic, 10)

        self.get_logger().info(
            f"Colorizer ready  in: {args.cloud_topic} + {args.image_topic}  "
            f"out: {args.out_topic}  fisheye={fisheye}")

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
            img = None if self.latest_img is None else self.latest_img
            size = self.latest_img_size
        if img is None:
            return  # no camera frame yet, just drop this cloud

        # Read x,y,z as a single (N,3) array.
        pts = np.array([
            (p[0], p[1], p[2]) for p in pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float32)
        if len(pts) == 0:
            return

        # Transform to camera frame: p_cam = R p_lidar + t
        pts_cam = (self.R @ pts.T.astype(np.float64)).T + self.t   # (N,3)

        # Keep only points in front of the camera.
        front_mask = pts_cam[:, 2] > 1e-3
        pts_lidar_front = pts[front_mask]
        pts_cam_front   = pts_cam[front_mask]
        if len(pts_cam_front) == 0:
            return

        # Project.
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

        pts_out  = pts_lidar_front[in_img]
        u_in     = u[in_img].astype(np.int32)
        v_in     = v[in_img].astype(np.int32)

        # Sample colors. img is BGR; reorder to RGB for the PCL `rgb` field.
        bgr = img[v_in, u_in]                 # (M, 3)
        rgb_u8 = bgr[:, ::-1].astype(np.uint8) # BGR -> RGB
        rgb_packed = pack_rgb_float(rgb_u8)   # (M,)

        # Build a structured (N,4) array → PointCloud2 with xyz + rgb fields.
        n = len(pts_out)
        cloud_struct = np.zeros(n, dtype=[
            ("x",   np.float32),
            ("y",   np.float32),
            ("z",   np.float32),
            ("rgb", np.float32),
        ])
        cloud_struct["x"] = pts_out[:, 0]
        cloud_struct["y"] = pts_out[:, 1]
        cloud_struct["z"] = pts_out[:, 2]
        cloud_struct["rgb"] = rgb_packed

        fields = [
            PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        out = pc2.create_cloud(header, fields, cloud_struct.tolist())
        self.pub.publish(out)


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


if __name__ == "__main__":
    main()
