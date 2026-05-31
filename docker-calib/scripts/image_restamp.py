#!/usr/bin/env python3
"""Restamp /image_raw onto the AIRY lidar hardware clock for FAST-LIVO2.

Once docker-calib/config/lidar_config.yaml sets use_lidar_clock: true, the
point cloud (/rslidar_points) and IMU (/rslidar_imu_data[_fixed]) carry the
AIRY's internal hardware clock, which is NOT the host UNIX clock -- it can be a
free-running sensor uptime (billions of seconds away). The USB camera
(/image_raw, gscam2 with use_gst_timestamps:true) is stamped with the per-frame
GStreamer CAPTURE time (buffer PTS) mapped into the host ROS wall-clock domain.
That is still the host clock (so the mapping below is unchanged), but it is
SMOOTH -- earlier we used publish-time stamps (use_gst_timestamps:false) which
arrived in bursts and ghosted the colorization. FAST-LIVO2 needs all three
streams on one timeline, so the image stamps must be mapped onto the lidar clock.

The mapping has two parts:

  1. COARSE offset (measured live here). For every reference message on the
     lidar clock we know both its header.stamp (lidar clock) and the host time
     we received it. host_recv - lidar_stamp == clock_offset + transport_latency.
     Transport latency is always >= 0, so the MINIMUM of that difference over a
     sliding window is the best estimate of the true clock offset. We track that
     min and use it; it absorbs any epoch gap and slow clock drift.

  2. FINE shift (camera capture-to-stamp lag). The coarse step cannot see the
     camera's own pipeline latency. Kalibr's camera-IMU calibration estimates it
     as time_shift_cam_imu; we read it from time_sync.yaml: cam_lidar_time_shift.

Each image is republished on /image_raw_synced with:

    new_lidar_stamp = host_stamp - coarse_offset + cam_lidar_time_shift

Everything else (data, encoding, frame_id) is copied through untouched.

Params (all overridable, sensible defaults match this repo's topics):
    ref_topic            (str)   /rslidar_imu_data   lidar-clock reference, 200 Hz
    in_topic             (str)   /image_raw          host-clock images in
    out_topic            (str)   /image_raw_synced   lidar-clock images out
    time_sync_config     (str)   /opt/calib/config/time_sync.yaml
    offset_window_sec    (float) 5.0   sliding window for the coarse-offset min
"""
import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class ImageRestamp(Node):
    def __init__(self):
        super().__init__("image_restamp")

        self.ref_topic = self.declare_parameter(
            "ref_topic", "/rslidar_imu_data").value
        self.in_topic = self.declare_parameter(
            "in_topic", "/image_raw").value
        self.out_topic = self.declare_parameter(
            "out_topic", "/image_raw_synced").value
        cfg_path = self.declare_parameter(
            "time_sync_config", "/opt/calib/config/time_sync.yaml").value
        self.window_ns = int(self.declare_parameter(
            "offset_window_sec", 5.0).value * 1e9)

        self.time_shift_ns = self._load_time_shift(cfg_path)

        # Sliding window of (host_recv_ns, diff_ns=host_recv-ref_stamp). We keep
        # the running minimum of diff_ns as the coarse clock offset estimate.
        self._samples = []
        self._offset_ns = None
        self._dropped = 0
        self._published = 0

        self.create_subscription(
            Imu, self.ref_topic, self._on_ref, qos_profile_sensor_data)
        self.sub_img = self.create_subscription(
            Image, self.in_topic, self._on_image, qos_profile_sensor_data)
        self.pub_img = self.create_publisher(
            Image, self.out_topic, qos_profile_sensor_data)
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"image_restamp: {self.in_topic} (host clock) -> {self.out_topic} "
            f"(lidar clock). ref={self.ref_topic} "
            f"fine_shift={self.time_shift_ns * 1e-9:+.6f}s")

    def _load_time_shift(self, path):
        try:
            with open(path) as f:
                shift = float(yaml.safe_load(f)["cam_lidar_time_shift"])
            self.get_logger().info(
                f"loaded cam_lidar_time_shift={shift:+.6f}s from {path}")
            return int(round(shift * 1e9))
        except (OSError, KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(
                f"could not read cam_lidar_time_shift from {path} ({e}); "
                "using 0.0 (coarse alignment only).")
            return 0

    def _on_ref(self, msg: Imu):
        host_ns = self.get_clock().now().nanoseconds
        ref_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        diff = host_ns - ref_ns
        self._samples.append((host_ns, diff))
        # Drop samples older than the window, then recompute the min.
        cutoff = host_ns - self.window_ns
        if self._samples[0][0] < cutoff:
            self._samples = [s for s in self._samples if s[0] >= cutoff]
        self._offset_ns = min(d for _, d in self._samples)

    def _on_image(self, msg: Image):
        if self._offset_ns is None:
            self._dropped += 1
            return
        host_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        new_ns = host_ns - self._offset_ns + self.time_shift_ns
        if new_ns < 0:
            self._dropped += 1
            self.get_logger().warn(
                "computed negative lidar stamp; dropping image", throttle_duration_sec=5.0)
            return
        msg.header.stamp.sec = new_ns // 1_000_000_000
        msg.header.stamp.nanosec = new_ns % 1_000_000_000
        self.pub_img.publish(msg)
        self._published += 1

    def _report(self):
        if self._offset_ns is None:
            self.get_logger().warn(
                f"waiting for {self.ref_topic}; {self._dropped} images dropped "
                "so far (no clock reference yet).")
            return
        self.get_logger().info(
            f"coarse offset={self._offset_ns * 1e-9:.6f}s  "
            f"published={self._published}  dropped={self._dropped}")


def main():
    rclpy.init()
    node = ImageRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
