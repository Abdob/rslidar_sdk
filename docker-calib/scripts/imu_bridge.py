#!/usr/bin/env python3
"""Republish AIRY IMU with units fixed for FAST-LIVO2.

The AIRY's built-in IMU streams on /rslidar_imu_data with linear_acceleration
in g, not m/s^2 (gravity reads ~1.0, expected ~9.81). FAST-LIVO2's EKF can't
estimate gravity direction from that, so it never initializes.

This node multiplies accel by 9.80665 and republishes to
/rslidar_imu_data_fixed. The header.stamp is preserved as-is — it's already
on the AIRY's lidar hardware clock, the same clock /rslidar_points uses (see
docker-calib/config/lidar_config.yaml: use_lidar_clock: true). Restamping
with ROS wall clock would break IMU/LiDAR synchronization.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665


class ImuBridge(Node):
    def __init__(self):
        super().__init__("imu_bridge")
        self.sub = self.create_subscription(
            Imu, "/rslidar_imu_data", self.cb, 200
        )
        self.pub = self.create_publisher(Imu, "/rslidar_imu_data_fixed", 200)
        self.get_logger().info(
            "IMU bridge running: /rslidar_imu_data (g) -> "
            "/rslidar_imu_data_fixed (m/s^2). Stamps unchanged."
        )

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration.x = msg.linear_acceleration.x * G
        out.linear_acceleration.y = msg.linear_acceleration.y * G
        out.linear_acceleration.z = msg.linear_acceleration.z * G
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(ImuBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
