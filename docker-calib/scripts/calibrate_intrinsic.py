#!/usr/bin/env python3
"""
Fisheye camera intrinsic calibration.

Subscribes to a ROS 2 image topic (default /image_raw), shows a live window
with chessboard detection overlay, and lets the user capture frames with
keypresses. After ~20 captures, runs cv2.fisheye.calibrate and writes:

  /opt/calib/config/intrinsics.yaml      (our native format, used by colorizer)
  /opt/calib/config/intrinsics_ros.yaml  (camera_info_manager format for usb_cam)

Keys:
  c  capture current frame (if a board is detected)
  u  undo last capture
  r  run calibration with the current set
  q  quit without saving
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class IntrinsicCalibrator(Node):
    def __init__(self, args):
        super().__init__("calibrate_intrinsic")
        self.args = args
        self.bridge = CvBridge()
        self.latest = None
        self.captures = []   # list of (img, corners) pairs
        self.last_capture_ts = 0.0

        self.pattern_size = (args.cols, args.rows)
        # 3D object points for the chessboard, in the board's own frame.
        self.objp = np.zeros((1, self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[0, :, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        self.objp *= args.square

        self.sub = self.create_subscription(Image, args.topic, self._on_image, 10)
        self.get_logger().info(
            f"Subscribed to {args.topic}; chessboard {self.pattern_size[0]}x{self.pattern_size[1]} "
            f"square={args.square*1000:.0f}mm")

    def _on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")
            return
        self.latest = img

    def _detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                 + cv2.CALIB_CB_NORMALIZE_IMAGE
                 + cv2.CALIB_CB_FAST_CHECK)
        ok, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags=flags)
        if not ok:
            return False, None
        # Refine to sub-pixel.
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)
        return True, corners

    def loop(self):
        win = "fisheye intrinsic — c=capture u=undo r=run q=quit"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280, 720)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest is None:
                time.sleep(0.02)
                continue
            disp = self.latest.copy()
            ok, corners = self._detect(self.latest)
            if ok:
                cv2.drawChessboardCorners(disp, self.pattern_size, corners, True)
            cv2.putText(disp, f"captures: {len(self.captures)}", (15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(disp, f"detected: {'YES' if ok else 'no'}", (15, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0) if ok else (0, 0, 255), 2)
            cv2.imshow(win, disp)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                self.get_logger().info("quit"); break
            elif k == ord('c'):
                # Debounce so a held key doesn't grab many near-duplicate frames.
                now = time.time()
                if ok and now - self.last_capture_ts > 0.5:
                    self.captures.append((self.latest.copy(), corners))
                    self.last_capture_ts = now
                    self.get_logger().info(f"captured #{len(self.captures)}")
            elif k == ord('u'):
                if self.captures:
                    self.captures.pop()
                    self.get_logger().info(f"undo, now {len(self.captures)}")
            elif k == ord('r'):
                self._run_calibration()
                break
        cv2.destroyAllWindows()

    def _run_calibration(self):
        if len(self.captures) < 10:
            self.get_logger().error(
                f"need at least 10 captures, have {len(self.captures)}")
            return
        h, w = self.captures[0][0].shape[:2]
        img_pts = [c.reshape(1, -1, 2).astype(np.float32) for _, c in self.captures]
        obj_pts = [self.objp.copy() for _ in self.captures]

        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in img_pts]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in img_pts]

        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                 + cv2.fisheye.CALIB_CHECK_COND
                 + cv2.fisheye.CALIB_FIX_SKEW)
        try:
            rms, K, D, _, _ = cv2.fisheye.calibrate(
                obj_pts, img_pts, (w, h), K, D, rvecs, tvecs, flags,
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-6))
        except cv2.error as e:
            self.get_logger().error(f"cv2.fisheye.calibrate failed: {e}")
            self.get_logger().error("Try recapturing — fisheye calibration is sensitive to "
                                    "frames where the board sits near the image edge with "
                                    "high distortion. Spread captures across the FOV but "
                                    "avoid the extreme corners.")
            return

        self.get_logger().info(f"calibration done, RMS = {rms:.3f} px")
        self.get_logger().info(f"K =\n{K}")
        self.get_logger().info(f"D = {D.ravel()}")
        self._save(K, D, (w, h), rms)

    def _save(self, K, D, size, rms):
        # Native format used by our colorizer + extrinsic tool.
        native_path = self.args.out
        native = {
            "model": "fisheye",
            "image_size": [int(size[0]), int(size[1])],
            "K": K.tolist(),
            "D": D.ravel().tolist(),
            "rms_px": float(rms),
        }
        with open(native_path, "w") as f:
            yaml.safe_dump(native, f, sort_keys=False)
        self.get_logger().info(f"wrote {native_path}")

        # camera_info_manager-compatible YAML for usb_cam to publish camera_info.
        # usb_cam doesn't accept a fisheye model, but for completeness we write
        # the plumb_bob form with k1..k4 copied to k1..k3,p1,p2,k4. RViz happily
        # consumes this; the colorizer reads our native file instead.
        ros_path = os.path.join(os.path.dirname(native_path), "intrinsics_ros.yaml")
        d = D.ravel().tolist() + [0.0] * (5 - len(D.ravel().tolist()))
        ros_yaml = {
            "image_width": int(size[0]),
            "image_height": int(size[1]),
            "camera_name": "usb_cam",
            "camera_matrix":      {"rows": 3, "cols": 3, "data": K.flatten().tolist()},
            "distortion_model":   "equidistant",
            "distortion_coefficients": {"rows": 1, "cols": 4, "data": D.ravel().tolist()},
            "rectification_matrix": {"rows": 3, "cols": 3,
                                     "data": np.eye(3).flatten().tolist()},
            "projection_matrix":  {"rows": 3, "cols": 4,
                                   "data": np.hstack([K, np.zeros((3, 1))]).flatten().tolist()},
        }
        with open(ros_path, "w") as f:
            yaml.safe_dump(ros_yaml, f, sort_keys=False)
        self.get_logger().info(f"wrote {ros_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic",  default="/image_raw")
    ap.add_argument("--cols",   type=int, default=9,  help="inner chessboard corners horizontally")
    ap.add_argument("--rows",   type=int, default=6,  help="inner chessboard corners vertically")
    ap.add_argument("--square", type=float, default=0.14351, help="chessboard square size in meters")
    ap.add_argument("--out",    default="/opt/calib/config/intrinsics.yaml")
    args = ap.parse_args()

    rclpy.init()
    node = IntrinsicCalibrator(args)
    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
