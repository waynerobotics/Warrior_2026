#!/usr/bin/env python3
"""Project lidar obstacles after estimating the local ground from sensor returns."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from warrior_navigation.ground_filter import obstacle_mask


def cloud_xyz(cloud):
    """Read XYZ even when other cloud fields have different datatypes."""
    points = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'))
    xyz = np.column_stack([points[name].reshape(-1) for name in ('x', 'y', 'z')])
    return xyz[np.isfinite(xyz).all(axis=1)]


def transform_xyz(points, transform):
    """Apply the complete mounting transform before fitting an upright plane."""
    q = transform.rotation
    quaternion = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm < 1.e-12:
        raise ValueError('Point cloud mounting transform has a zero quaternion')
    x, y, z, w = quaternion / norm
    rotation = np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    t = transform.translation
    return points @ rotation.T + [t.x, t.y, t.z]


def laser_ranges(points, *, angle_min, angle_max, angle_increment,
                 range_min, range_max, use_inf=True):
    """Keep the nearest obstacle in each angular bin; empty bins clear space."""
    count = int(math.ceil((angle_max-angle_min)/angle_increment))
    ranges = np.full(count, np.inf if use_inf else range_max+1.0, dtype=np.float32)
    if not len(points):
        return ranges
    distances = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    if angle_max-angle_min >= 2*math.pi-1.e-6:
        angles = (angles-angle_min) % (2*math.pi) + angle_min
    valid = ((distances >= range_min) & (distances <= range_max)
             & (angles >= angle_min) & (angles < angle_max))
    indices = np.floor((angles[valid]-angle_min)/angle_increment).astype(int)
    np.minimum.at(ranges, np.minimum(indices, count-1), distances[valid])
    return ranges


class GroundObstacleScan(Node):
    def __init__(self):
        super().__init__('navigation_obstacle_scan')
        defaults = {
            'target_frame': 'base_footprint', 'ground_height': 0.0,
            'min_height': 0.15, 'max_height': 1.5,
            'range_min': 0.9, 'range_max': 15.0,
            'angle_min': -math.pi, 'angle_max': math.pi,
            'angle_increment': math.pi/180.0, 'scan_time': 0.1, 'use_inf': True,
        }
        self.settings = {key: self.declare_parameter(key, value).value
                         for key, value in defaults.items()}
        if (self.settings['angle_increment'] <= 0
                or self.settings['angle_max'] <= self.settings['angle_min']
                or self.settings['range_max'] <= self.settings['range_min']):
            raise ValueError('Invalid scan angle or range limits')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, 'cloud_in', self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, cloud):
        target = self.settings['target_frame']
        try:
            points = cloud_xyz(cloud)
            if cloud.header.frame_id != target:
                transform = self.buffer.lookup_transform(
                    target, cloud.header.frame_id, Time.from_msg(cloud.header.stamp))
                points = transform_xyz(points, transform.transform)
        except (TransformException, ValueError, KeyError) as error:
            self.get_logger().warning(
                f'Waiting for usable lidar in {target}: {error}', throttle_duration_sec=5.0)
            return
        keep = obstacle_mask(
            points, ground_height=self.settings['ground_height'],
            min_height=self.settings['min_height'], max_height=self.settings['max_height'],
            max_range=self.settings['range_max'])
        ranges = laser_ranges(points[keep], **{key: self.settings[key] for key in
                              ('angle_min', 'angle_max', 'angle_increment',
                               'range_min', 'range_max', 'use_inf')})
        scan = LaserScan()
        scan.header.stamp = cloud.header.stamp
        scan.header.frame_id = target
        scan.angle_min = self.settings['angle_min']
        scan.angle_increment = self.settings['angle_increment']
        scan.angle_max = scan.angle_min + (len(ranges)-1)*scan.angle_increment
        scan.scan_time = self.settings['scan_time']
        scan.range_min = self.settings['range_min']
        scan.range_max = self.settings['range_max']
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = GroundObstacleScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
