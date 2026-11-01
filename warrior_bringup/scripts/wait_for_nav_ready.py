#!/usr/bin/env python3
"""Wait until controller odometry and its base transform are available."""

import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class NavReadiness(Node):
    """Observe the minimum runtime signals Nav2 needs before it starts."""

    def __init__(self):
        super().__init__('wait_for_nav_ready')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('scan_topic', '')
        self.declare_parameter('gps_odom_topic', '')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.deadline = time.monotonic() + self.timeout_sec
        self.received_odom = False
        scan_topic = self.get_parameter('scan_topic').value
        gps_odom_topic = self.get_parameter('gps_odom_topic').value
        self.received_scan = not scan_topic
        self.received_gps_odom = not gps_odom_topic

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 10)
        if scan_topic:
            self.scan_sub = self.create_subscription(
                LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        if gps_odom_topic:
            self.gps_sub = self.create_subscription(
                Odometry, gps_odom_topic, self._on_gps_odom, 10)

        self.get_logger().info(
            f'Waiting up to {self.timeout_sec:.1f}s for {self.odom_topic} and '
            f'{self.global_frame} -> {self.robot_base_frame} TF'
            f'{", lidar " + scan_topic if scan_topic else ""}'
            f'{", GPS odometry " + gps_odom_topic if gps_odom_topic else ""}')

    def _on_odom(self, _message):
        self.received_odom = True

    def _on_scan(self, message):
        self.received_scan = bool(message.ranges)

    def _on_gps_odom(self, _message):
        self.received_gps_odom = True

    def ready(self):
        return (self.received_odom and self.received_scan and self.received_gps_odom
                and self.tf_buffer.can_transform(
                    self.global_frame, self.robot_base_frame, Time()))

    def timed_out(self):
        return time.monotonic() >= self.deadline


def main(args=None):
    rclpy.init(args=args)
    node = NavReadiness()
    exit_code = 1

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                node.get_logger().info(
                    'Odometry and base transform are ready.')
                exit_code = 0
                break
            if node.timed_out():
                node.get_logger().error(
                    'Timed out waiting for navigation prerequisites.')
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
