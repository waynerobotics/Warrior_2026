##############################################################################
# Mask to Point Cloud - Binary mask and LiDAR fusion
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
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from rcl_interfaces.msg import SetParametersResult

class MaskToPointCloud(Node):
    def __init__(self):
        super().__init__('mask_to_pointcloud')

        # Parameters
        self.declare_parameter('transformation_matrix', [
            1., 0., 0., 0.,
            0., 1., 0., 0.,
            0., 0., 1., 0.,
            0., 0., 0., 1.
            ])
        self.declare_parameter('pointcloud_topic', '/unilidar/cloud')
        self.declare_parameter('mask_topic', 'ai/seg_mask')
        self.declare_parameter('obstacle_pointcloud', 'obstacle_pointcloud')
        self.declare_parameter('debug_overlay', 'debug/mask_overlay')
        self.declare_parameter('rgb_image_topic', '/camera/image_raw')
        self.declare_parameter('textured_pointcloud', 'textured_pointcloud')
        self.declare_parameter('dynamic_transformation_updates', True)
        self.declare_parameter('yaw_angle', 0.0)

        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        # Subscribers
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

        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('rgb_image_topic').value,
            self.image_callback,
            image_qos)

        # Publishers
        self.obstacle_pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('obstacle_pointcloud').value,
            1)

        self.debug_overlay_pub = self.create_publisher(
            Image,
            self.get_parameter('debug_overlay').value,
            1)

        self.textured_pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('textured_pointcloud').value,
            1)

        # Load transformation matrix
        self.get_logger().info(str(self.get_parameter('transformation_matrix').value))
        self.transformation_matrix = np.array(
            self.get_parameter('transformation_matrix').value).reshape((4, 4))

        # Set up parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Timer for checking parameter updates
        if self.get_parameter('dynamic_transformation_updates').value:
            self.timer = self.create_timer(1.0, self.check_transformation_updates)

        # Utilities
        self.bridge = CvBridge()
        self.latest_mask = None
        self.latest_pointcloud = None
        self.latest_image = None

        # Last known yaw angle for tracking changes
        self.last_yaw_angle = self.get_parameter('yaw_angle').value

    def parameters_callback(self, params):
        """Callback for parameter changes"""
        update_needed = False

        for param in params:
            if param.name == 'yaw_angle':
                # Always update on yaw_angle change
                self.last_yaw_angle = param.value
                self.get_logger().info(f"Received yaw angle update: {param.value}")
                update_needed = True
            elif param.name == 'transformation_matrix':
                try:
                    self.transformation_matrix = np.array(param.value).reshape((4, 4))
                    self.get_logger().info("Updated transformation matrix directly")
                except Exception as e:
                    self.get_logger().error(f"Failed to update transformation matrix: {e}")

        if update_needed:
            # Force immediate update
            self.update_transformation_from_yaw()

            # Force immediate reprocessing if we have data
            if self.latest_mask is not None and self.latest_pointcloud is not None:
                self.get_logger().info("Forcing reprocessing with new transformation")
                self.process_data()

        return SetParametersResult(successful=True)

    def update_transformation_from_yaw(self):
        """Update transformation matrix from yaw angle"""
        try:
            # Create rotation matrix from yaw angle
            angle_rad = math.radians(self.last_yaw_angle)
            cos_angle = math.cos(angle_rad)
            sin_angle = math.sin(angle_rad)

            # Create the rotation matrix (Z-axis rotation)
            rotation_matrix = np.identity(4)
            rotation_matrix[0, 0] = cos_angle
            rotation_matrix[0, 1] = -sin_angle
            rotation_matrix[1, 0] = sin_angle
            rotation_matrix[1, 1] = cos_angle

            # Start with identity base transformation for simplicity
            # In practice, you might want to load a base transformation from a parameter
            base_transformation = np.identity(4)

            # Apply rotation to base transformation
            self.transformation_matrix = np.matmul(rotation_matrix, base_transformation)

            self.get_logger().info(f"Updated transformation matrix from yaw angle: {self.last_yaw_angle}")

            # Process data with new transformation if data is available
            if self.latest_mask is not None and self.latest_pointcloud is not None:
                self.process_data()

        except Exception as e:
            self.get_logger().error(f"Error updating transformation from yaw: {e}")

    def check_transformation_updates(self):
        """Check for transformation updates from external parameter servers"""
        try:
            # Try to get the latest yaw angle parameter
            current_yaw = self.get_parameter('yaw_angle').value

            # Update if changed
            if current_yaw != self.last_yaw_angle:
                self.last_yaw_angle = current_yaw
                self.update_transformation_from_yaw()
                self.get_logger().info(f"Detected yaw angle change: {current_yaw}")
        except Exception as e:
            self.get_logger().error(f"Error checking transformation updates: {e}")

    def mask_callback(self, msg):
        self.get_logger().info('Received mask')
        self.latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        # ToDo: see if flip is needed
        # self.latest_mask = cv2.flip(self.latest_mask, 1)

        self.process_data()

    def lidar_callback(self, msg):
        self.get_logger().info('Received lidar')
        self.latest_pointcloud = msg

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process data if we have received both mask and pointcloud
        if self.latest_mask is not None and self.latest_pointcloud is not None:
            self.process_data()

    def process_data(self):
        if self.latest_mask is None or self.latest_pointcloud is None:
            return

        # Make a copy of the mask
        mask = self.latest_mask.copy()

        # Ensure mask is binary (0 or 255)
        _, binary_mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

        # Parse point cloud data
        points = pc2.read_points(
            self.latest_pointcloud,
            field_names=("x", "y", "z", "intensity", "ring", "time"),
            skip_nans=True
        )
        points = list(points)
        points_array = np.array(points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32)]
        )

        x = np.array(points_array['x'])
        y = np.array(points_array['y'])
        z = np.array(points_array['z'])
        intensity = np.array(points_array['intensity'])
        ring = np.array(points_array['ring'])
        time = np.array(points_array['time'])
        ones = np.ones_like(x)

        # Transform points to camera frame
        points_matrix = np.stack((x, y, z, ones), axis=-1)
        transformed_points = np.dot(points_matrix, self.transformation_matrix.T)
        x_cam, y_cam, z_cam = transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2]

        # Convert into spherical coordinates for projection
        r = np.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        theta = np.arctan2(y_cam, x_cam)
        phi = np.arccos(z_cam / r)

        # Project points onto image plane
        u = ((theta + np.pi) / (2 * np.pi) * binary_mask.shape[1]).astype(int)
        v = ((phi) / np.pi * binary_mask.shape[0]).astype(int)

        # Clip bounds
        valid_indices = (
            (u >= 0) & (u < binary_mask.shape[1]) &
            (v >= 0) & (v < binary_mask.shape[0])
        )

        u = u[valid_indices]
        v = v[valid_indices]
        x_original = x[valid_indices]
        y_original = y[valid_indices]
        z_original = z[valid_indices]
        intensity_original = intensity[valid_indices]
        ring_original = ring[valid_indices]
        time_original = time[valid_indices]

        # No need to process the top above mask
        # valid_indices

        # Find which points correspond to obstacles (mask value = 255)
        mask_values = binary_mask[v, u]
        obstacle_indices = np.where(mask_values == 255)[0]

        # Extract obstacle points (in original LiDAR frame)
        obstacle_x = x_original[obstacle_indices]
        obstacle_y = y_original[obstacle_indices]
        obstacle_z = z_original[obstacle_indices]
        obstacle_intensity = intensity_original[obstacle_indices]
        obstacle_ring = ring_original[obstacle_indices]
        obstacle_time = time_original[obstacle_indices]

        if len(obstacle_indices) > 0 and self.obstacle_pointcloud_pub.get_subscription_count() > 0:
            # Create obstacle point cloud
            obstacle_points = np.zeros((len(obstacle_indices),), dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32),
                ('ring', np.uint16),
                ('time', np.float32)
            ])

            obstacle_points['x'] = obstacle_x
            obstacle_points['y'] = obstacle_y
            obstacle_points['z'] = obstacle_z
            obstacle_points['intensity'] = obstacle_intensity
            obstacle_points['ring'] = obstacle_ring
            obstacle_points['time'] = obstacle_time

            # Create PointCloud2 message
            header = self.latest_pointcloud.header
            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='ring', offset=16, datatype=pc2.PointField.UINT16, count=1),
                pc2.PointField(name='time', offset=20, datatype=pc2.PointField.FLOAT32, count=1),
            ]

            # Convert to the right dtype that matches the fields
            points_list = [(p['x'], p['y'], p['z'], p['intensity'], p['ring'], p['time'])
                           for p in obstacle_points]

            # Create cloud using list of points instead of numpy array
            obstacle_pointcloud = pc2.create_cloud(header, fields, points_list)

            # Publish obstacle point cloud
            self.obstacle_pointcloud_pub.publish(obstacle_pointcloud)
            self.get_logger().info(f'Published obstacle point cloud with {len(obstacle_indices)} points')

        # Create debug overlay if needed
        if self.debug_overlay_pub.get_subscription_count() > 0:
            # Create RGB visualization of the mask
            debug_overlay = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)

            # Plot the projected points
            for i in range(len(u)):
                if i in obstacle_indices:
                    # Draw obstacle points in red
                    cv2.circle(debug_overlay, (u[i], v[i]), 5, (0, 0, 255), -1)
                else:
                    # Draw non-obstacle points in blue
                    cv2.circle(debug_overlay, (u[i], v[i]), 5, (255, 0, 0), -1)

            # Publish debug overlay
            self.debug_overlay_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_overlay, encoding='bgr8'))

        # Publish textured point cloud if image is available
        if self.latest_image is not None and self.textured_pointcloud_pub.get_subscription_count() > 0:
            self.publish_textured_pointcloud(obstacle_indices)

    def publish_textured_pointcloud(self, obstacle_indices):
        if self.latest_pointcloud is None or self.latest_image is None:
            return

        # Parse point cloud data
        points = pc2.read_points(
            self.latest_pointcloud,
            field_names=("x", "y", "z", "intensity", "ring", "time"),
            skip_nans=True
        )
        points = list(points)
        points_array = np.array(points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32)]
        )

        x = np.array(points_array['x'])
        y = np.array(points_array['y'])
        z = np.array(points_array['z'])
        intensity = np.array(points_array['intensity'])
        ring = np.array(points_array['ring'])
        time = np.array(points_array['time'])
        ones = np.ones_like(x)

        # Transform points to camera frame
        points_matrix = np.stack((x, y, z, ones), axis=-1)
        transformed_points = np.dot(points_matrix, self.transformation_matrix.T)
        x_cam, y_cam, z_cam = transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2]

        # Convert into spherical coordinates for projection
        r = np.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        theta = np.arctan2(y_cam, x_cam)
        phi = np.arccos(z_cam / r)

        # Project points onto image plane
        u = ((theta + np.pi) / (2 * np.pi) * self.latest_image.shape[1]).astype(int)
        v = ((phi) / np.pi * self.latest_image.shape[0]).astype(int)

        # Clip bounds
        valid_indices = (
            (u >= 0) & (u < self.latest_image.shape[1]) &
            (v >= 0) & (v < self.latest_image.shape[0])
        )

        u = u[valid_indices]
        v = v[valid_indices]
        x_original = x[valid_indices]
        y_original = y[valid_indices]
        z_original = z[valid_indices]

        # Get RGB values from the latest image for all valid points
        rgb_values = self.latest_image[v, u]

        # Combine RGB values with point cloud data
        textured_points = np.empty((len(x_original),), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])

        textured_points['x'] = x_original
        textured_points['y'] = y_original
        textured_points['z'] = z_original

        # Calculate RGB value as a packed 32-bit integer
        textured_points['rgb'] = ((rgb_values[:, 0].astype(np.uint32) << 16) |
                                  (rgb_values[:, 1].astype(np.uint32) << 8) |
                                  rgb_values[:, 2].astype(np.uint32))

        # Create PointCloud2 message
        header = self.latest_pointcloud.header
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]

        # Convert to the right dtype that matches the fields
        points_list = [(p['x'], p['y'], p['z'], p['rgb'])
                       for p in textured_points]

        # Create cloud using list of points instead of numpy array
        textured_pointcloud = pc2.create_cloud(header, fields, points_list)

        # Publish textured point cloud
        self.textured_pointcloud_pub.publish(textured_pointcloud)
        self.get_logger().info(f'Published textured point cloud with {len(x_original)} points')


def main(args=None):
    rclpy.init(args=args)
    node = MaskToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
