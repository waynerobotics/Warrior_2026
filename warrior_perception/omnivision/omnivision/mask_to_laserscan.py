##############################################################################
# Mask to LaserScan - Binary mask and LiDAR fusion
#
# Software License Agreement (GPL-3.0)
# Copyright (C) 2025 Blaine Oania
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
##############################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from rcl_interfaces.msg import SetParametersResult


class MaskToLaserScan(Node):
    def __init__(self):
        super().__init__('mask_to_laserscan')

        self.declare_parameter('transformation_matrix', [
            1., 0., 0., 0.285,
            0., -1., 0., 0.0,
            0., 0., -1., 0.,
            0., 0., 0., 1.
            ])
        self.declare_parameter('pointcloud_topic', '/unilidar/cloud')
        self.declare_parameter('mask_topic', 'ai/seg_mask')
        self.declare_parameter('laserscan_topic', 'obstacle_laserscan')
        self.declare_parameter('debug_overlay', 'debug/mask_laserscan_overlay')
        self.declare_parameter('dynamic_transformation_updates', True)
        self.declare_parameter('yaw_angle', 0.0)
        self.declare_parameter('angle_increment', math.pi / 180.0)  # 1 deg
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 100.0)
        self.declare_parameter('z_min', 1.0)

        # Full 360° — fixed by the spherical projection, not configurable
        self.ANGLE_MIN = -math.pi
        self.ANGLE_MAX = math.pi

        self.transformation_matrix = np.array(
            self.get_parameter('transformation_matrix').value).reshape((4, 4))
        self.last_yaw_angle = self.get_parameter('yaw_angle').value

        self.mask_sub = self.create_subscription(
            Image,
            self.get_parameter('mask_topic').value,
            self.mask_callback,
            1)

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('pointcloud_topic').value,
            self.lidar_callback,
            1)

        self.laserscan_pub = self.create_publisher(
            LaserScan,
            self.get_parameter('laserscan_topic').value,
            1)

        self.debug_overlay_pub = self.create_publisher(
            Image,
            self.get_parameter('debug_overlay').value,
            1)

        self.add_on_set_parameters_callback(self.parameters_callback)

        if self.get_parameter('dynamic_transformation_updates').value:
            self.create_timer(1.0, self.check_transformation_updates)

        self.bridge = CvBridge()
        self.latest_mask = None
        self.latest_pointcloud = None

    def parameters_callback(self, params):
        update_needed = False
        for param in params:
            if param.name == 'yaw_angle':
                self.last_yaw_angle = param.value
                update_needed = True
            elif param.name == 'transformation_matrix':
                try:
                    self.transformation_matrix = np.array(param.value).reshape((4, 4))
                except Exception as e:
                    self.get_logger().error(f'Failed to update transformation matrix: {e}')

        if update_needed:
            self.update_transformation_from_yaw()
            if self.latest_mask is not None and self.latest_pointcloud is not None:
                self.process_data()

        return SetParametersResult(successful=True)

    def update_transformation_from_yaw(self):
        angle_rad = math.radians(self.last_yaw_angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        rot = np.identity(4)
        rot[0, 0] = cos_a
        rot[0, 1] = -sin_a
        rot[1, 0] = sin_a
        rot[1, 1] = cos_a
        self.transformation_matrix = rot
        if self.latest_mask is not None and self.latest_pointcloud is not None:
            self.process_data()

    def check_transformation_updates(self):
        current_yaw = self.get_parameter('yaw_angle').value
        if current_yaw != self.last_yaw_angle:
            self.last_yaw_angle = current_yaw
            self.update_transformation_from_yaw()

    def mask_callback(self, msg):
        self.latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.process_data()

    def lidar_callback(self, msg):
        self.latest_pointcloud = msg

    def process_data(self):
        if self.latest_mask is None or self.latest_pointcloud is None:
            return

        _, binary_mask = cv2.threshold(
            self.latest_mask.copy(), 127, 255, cv2.THRESH_BINARY)

        points = list(pc2.read_points(
            self.latest_pointcloud,
            field_names=('x', 'y', 'z', 'intensity', 'ring', 'time'),
            skip_nans=True))

        if not points:
            return

        points_array = np.array(points, dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('ring', np.uint16), ('time', np.float32)])

        x = points_array['x']
        y = points_array['y']
        z = points_array['z']
        intensity = points_array['intensity']
        ones = np.ones_like(x)

        # Transform to camera frame for mask projection
        pts = np.stack((x, y, z, ones), axis=-1)
        transformed = np.dot(pts, self.transformation_matrix.T)
        xc, yc, zc = transformed[:, 0], transformed[:, 1], transformed[:, 2]

        # Spherical projection onto mask
        r_sph = np.sqrt(xc**2 + yc**2 + zc**2)
        theta = np.arctan2(yc, xc)
        phi = np.arccos(np.clip(zc / np.where(r_sph > 0, r_sph, 1e-9), -1.0, 1.0))

        u = ((theta + np.pi) / (2 * np.pi) * binary_mask.shape[1]).astype(int)
        v = ((phi) / np.pi * binary_mask.shape[0]).astype(int)

        in_bounds = (
            (u >= 0) & (u < binary_mask.shape[1]) &
            (v >= 0) & (v < binary_mask.shape[0]))

        u_valid = u[in_bounds]
        v_valid = v[in_bounds]
        x_valid = x[in_bounds]
        y_valid = y[in_bounds]
        z_valid = z[in_bounds]
        intensity_valid = intensity[in_bounds]

        obstacle_mask = binary_mask[v_valid, u_valid] == 255
        obs_x = x_valid[obstacle_mask]
        obs_y = y_valid[obstacle_mask]
        obs_z = z_valid[obstacle_mask]
        obs_int = intensity_valid[obstacle_mask]

        z_min = self.get_parameter('z_min').value
        height_mask = obs_z >= z_min
        obs_x = obs_x[height_mask]
        obs_y = obs_y[height_mask]
        obs_int = obs_int[height_mask]

        # Build LaserScan from obstacle points
        angle_min = self.ANGLE_MIN
        angle_max = self.ANGLE_MAX
        angle_inc = self.get_parameter('angle_increment').value
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value

        num_bins = int(round((angle_max - angle_min) / angle_inc))
        ranges = np.full(num_bins, np.inf)
        intensities = np.zeros(num_bins)

        if len(obs_x) > 0:
            angles = np.arctan2(obs_y, obs_x)
            dists = np.sqrt(obs_x**2 + obs_y**2)

            bin_indices = ((angles - angle_min) / angle_inc).astype(int)
            valid = (bin_indices >= 0) & (bin_indices < num_bins) & \
                    (dists >= range_min) & (dists <= range_max)

            for i in np.where(valid)[0]:
                b = bin_indices[i]
                if dists[i] < ranges[b]:
                    ranges[b] = dists[i]
                    intensities[b] = obs_int[i]

        # Replace inf with range_max (no obstacle seen in that bin)
        ranges = np.where(np.isinf(ranges), range_max, ranges).astype(np.float32)

        scan = LaserScan()
        scan.header = self.latest_pointcloud.header
        scan.angle_min = float(angle_min)
        scan.angle_max = float(angle_max)
        scan.angle_increment = float(angle_inc)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = float(range_min)
        scan.range_max = float(range_max)
        scan.ranges = ranges.tolist()
        scan.intensities = intensities.tolist()

        self.laserscan_pub.publish(scan)

        if self.debug_overlay_pub.get_subscription_count() > 0:
            overlay = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
            non_obs_u = u_valid[~obstacle_mask]
            non_obs_v = v_valid[~obstacle_mask]
            obs_u = u_valid[obstacle_mask]
            obs_v = v_valid[obstacle_mask]
            for pu, pv in zip(non_obs_u, non_obs_v):
                cv2.circle(overlay, (int(pu), int(pv)), 3, (255, 0, 0), -1)
            for pu, pv in zip(obs_u, obs_v):
                cv2.circle(overlay, (int(pu), int(pv)), 5, (0, 0, 255), -1)
            self.debug_overlay_pub.publish(
                self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = MaskToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
