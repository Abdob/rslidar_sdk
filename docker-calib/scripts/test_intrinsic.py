#!/usr/bin/env python3
"""
Live fisheye undistortion preview.

Subscribes to a ROS 2 image topic, loads the intrinsics produced by
calibrate_intrinsic.py, and shows raw and undistorted frames side-by-side
so you can eyeball whether straight lines in the world come out straight.

Keys:
  +/-  raise/lower the undistortion balance (0 = tight crop, 1 = keep all pixels)
  g    toggle the reference grid overlay
  q    quit
"""

import argparse
import sys

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class UndistortPreview(Node):
    def __init__(self, args, K, D, size):
        super().__init__("test_intrinsic")
        self.K = K.astype(np.float64)
        self.D = D.astype(np.float64).reshape(-1, 1)
        self.size = size                   # (w, h) the calibration was done at
        self.balance = float(args.balance)
        self.show_grid = True
        self.bridge = CvBridge()
        self.latest = None

        self._rebuild_maps()
        self.create_subscription(Image, args.topic, self._on_image, 10)
        self.get_logger().info(
            f"Subscribed to {args.topic}; calibration size {size[0]}x{size[1]}, "
            f"balance={self.balance:.2f}")

    def _rebuild_maps(self):
        w, h = self.size
        # New K that controls how much of the undistorted scene is visible.
        # balance=0 crops to only valid pixels; balance=1 keeps everything
        # (with black borders where the original image didn't reach).
        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, (w, h), np.eye(3), balance=self.balance)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.new_K, (w, h), cv2.CV_16SC2)

    def _on_image(self, msg: Image):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")

    @staticmethod
    def _draw_grid(img, step=80, color=(0, 255, 0)):
        h, w = img.shape[:2]
        for x in range(step, w, step):
            cv2.line(img, (x, 0), (x, h - 1), color, 1, cv2.LINE_AA)
        for y in range(step, h, step):
            cv2.line(img, (0, y), (w - 1, y), color, 1, cv2.LINE_AA)

    def loop(self):
        win = "intrinsic test — raw | undistorted (q=quit  +/-=balance  g=grid)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1600, 600)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest is None:
                continue
            raw = self.latest
            if (raw.shape[1], raw.shape[0]) != self.size:
                # Resize to the calibration size — the maps assume that resolution.
                raw = cv2.resize(raw, self.size)
            und = cv2.remap(raw, self.map1, self.map2,
                            interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT)

            disp_raw = raw.copy()
            disp_und = und.copy()
            if self.show_grid:
                self._draw_grid(disp_raw)
                self._draw_grid(disp_und)

            cv2.putText(disp_raw, "raw", (15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(disp_und, f"undistorted  balance={self.balance:.2f}",
                        (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

            side_by_side = np.hstack([disp_raw, disp_und])
            cv2.imshow(win, side_by_side)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k in (ord('+'), ord('=')):
                self.balance = min(1.0, self.balance + 0.1)
                self._rebuild_maps()
                self.get_logger().info(f"balance = {self.balance:.2f}")
            elif k in (ord('-'), ord('_')):
                self.balance = max(0.0, self.balance - 0.1)
                self._rebuild_maps()
                self.get_logger().info(f"balance = {self.balance:.2f}")
            elif k == ord('g'):
                self.show_grid = not self.show_grid
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic",      default="/image_raw")
    ap.add_argument("--intrinsics", default="/opt/calib/config/intrinsics.yaml")
    ap.add_argument("--balance",    type=float, default=0.0,
                    help="0 = tight crop (only valid pixels), 1 = keep full FOV")
    args = ap.parse_args()

    with open(args.intrinsics) as f:
        intr = yaml.safe_load(f)
    if intr.get("model", "fisheye") != "fisheye":
        print(f"this script only handles the fisheye model, "
              f"got {intr.get('model')!r}", file=sys.stderr)
        sys.exit(2)
    K = np.array(intr["K"])
    D = np.array(intr["D"])
    size = tuple(intr["image_size"])    # (w, h)

    rclpy.init()
    node = UndistortPreview(args, K, D, size)
    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
