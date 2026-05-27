##############################################################################
# Transformation Calibrator - Utility to adjust yaw alignment between camera and LiDAR
#
# Software License Agreement (GPL-3.0)
# Copyright (C) 2025 Shanti Robot Team
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
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os
import yaml
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TransformationCalibrator(Node):
    def __init__(self):
        super().__init__('transformation_calibrator')

        # Declare parameters
        self.declare_parameter('base_transformation_matrix', [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ])
        self.declare_parameter('pointcloud_topic', '/unilidar/cloud')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter(
            'config_file_path',
            os.path.join(
                os.path.expanduser('~'),
                'ros2_ws/src/Warrior_2026/warrior_perception/omnivision/config/transformation.yaml'))

        # Get parameters
        self.base_transformation = np.array(
            self.get_parameter('base_transformation_matrix').value).reshape((4, 4))
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.config_file_path = self.get_parameter('config_file_path').value

        # Ensure config directory exists
        os.makedirs(os.path.dirname(self.config_file_path), exist_ok=True)

        # Initialize transformation values
        self.yaw_angle = 0.0  # degrees
        self.current_transformation = self.base_transformation.copy()

        # QoS profile for reliable subscriptions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            qos_profile
        )

        # Utilities
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_pointcloud = None

        # GUI setup
        self.window_name = "Transformation Calibrator"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)

        # Create trackbar for yaw adjustment
        cv2.createTrackbar("Yaw (degrees)", self.window_name, 0, 360, self.yaw_trackbar_callback)
        cv2.setTrackbarPos("Yaw (degrees)", self.window_name, 0)

        # Create save button simulation with a trackbar
        cv2.createTrackbar("Save (0=No, 1=Yes)", self.window_name, 0, 1, self.save_trackbar_callback)

        # Timer for visualization updates
        self.timer = self.create_timer(0.1, self.update_visualization)

        self.get_logger().info("Transformation Calibrator started")
        self.get_logger().info("Use the trackbar to adjust yaw alignment")
        self.get_logger().info("Set the 'Save' trackbar to 1 to save the calibration")

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg

    def yaw_trackbar_callback(self, value):
        self.yaw_angle = value
        self.update_transformation_matrix()

    def save_trackbar_callback(self, value):
        if value == 1:
            self.save_transformation()
            # Reset the trackbar to 0 after saving
            cv2.setTrackbarPos("Save (0=No, 1=Yes)", self.window_name, 0)

    def update_transformation_matrix(self):
        # Create a new rotation matrix around the Z axis (yaw)
        angle_rad = math.radians(self.yaw_angle)
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)

        # Start with identity matrix
        rotation_matrix = np.identity(4)

        # Apply rotation around Z axis (yaw)
        rotation_matrix[0, 0] = cos_angle
        rotation_matrix[0, 1] = -sin_angle
        rotation_matrix[1, 0] = sin_angle
        rotation_matrix[1, 1] = cos_angle

        # Apply the rotation to the base transformation
        self.current_transformation = np.matmul(rotation_matrix, self.base_transformation)

    def update_visualization(self):
        if self.latest_image is None or self.latest_pointcloud is None:
            return

        # Make a copy of the image for visualization
        visualization = self.latest_image.copy()

        # Parse point cloud data
        points = pc2.read_points(
            self.latest_pointcloud,
            field_names=("x", "y", "z"),
            skip_nans=True
        )
        points = list(points)

        if not points:
            return

        # Convert to numpy array
        points_array = np.array([(p[0], p[1], p[2]) for p in points])

        # Add homogeneous coordinate
        points_homogeneous = np.hstack((points_array, np.ones((len(points_array), 1))))

        # Apply current transformation
        transformed_points = np.dot(points_homogeneous, self.current_transformation.T)

        # Extract transformed coordinates
        x_cam = transformed_points[:, 0]
        y_cam = transformed_points[:, 1]
        z_cam = transformed_points[:, 2]

        # Convert to spherical coordinates for projection
        r = np.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        theta = np.arctan2(y_cam, x_cam)
        phi = np.arccos(np.clip(z_cam / r, -1.0, 1.0))  # Clip to avoid numerical issues

        # Project points onto image plane
        img_width = visualization.shape[1]
        img_height = visualization.shape[0]

        u = ((theta + np.pi) / (2 * np.pi) * img_width).astype(int)
        v = (phi / np.pi * img_height).astype(int)

        # Filter points that are within image bounds
        valid_indices = (
            (u >= 0) & (u < img_width) &
            (v >= 0) & (v < img_height)
        )

        u = u[valid_indices]
        v = v[valid_indices]
        r_valid = r[valid_indices]

        # Draw points on the image, color-coded by distance
        for i in range(len(u)):
            # Color based on distance (red=close, blue=far)
            distance = min(r_valid[i], 10.0) / 10.0  # Normalize to [0,1] with 10m max
            color = (
                int(255 * (1 - distance)),  # B
                0,                          # G
                int(255 * distance)         # R
            )
            cv2.circle(visualization, (u[i], v[i]), 2, color, -1)

        # Add information overlay
        info_text = [
            f"Yaw: {self.yaw_angle:.1f} degrees",
            "Instructions:",
            "- Adjust the 'Yaw' slider to align LiDAR points with image",
            "- Set 'Save' to 1 to save the calibration",
            "- Press 'q' to quit without saving"
        ]

        y_offset = 30
        for text in info_text:
            cv2.putText(
                visualization,
                text,
                (20, y_offset),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )
            y_offset += 30

        # Display the visualization
        cv2.imshow(self.window_name, visualization)
        key = cv2.waitKey(1)

        # Quit if 'q' is pressed
        if key == ord('q'):
            self.get_logger().info("Calibration canceled")
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def save_transformation(self):
        # Save the current transformation matrix to the config file
        matrix_list = self.current_transformation.flatten().tolist()

        # Format with YAML
        config_data = {
            'transformation_matrix': matrix_list,
            'yaw_angle_degrees': self.yaw_angle
        }

        try:
            with open(self.config_file_path, 'w') as f:
                yaml.dump(config_data, f, default_flow_style=False)

            self.get_logger().info(f"Transformation matrix saved to {self.config_file_path}")
            self.get_logger().info(f"Yaw angle: {self.yaw_angle} degrees")
            self.get_logger().info("To use this calibration, set the transformation_matrix parameter in your launch file")
            self.get_logger().info("or copy the following parameter value:")
            matrix_str = '[' + ', '.join([f"{val:.6f}" for val in matrix_list]) + ']'
            self.get_logger().info(matrix_str)
        except Exception as e:
            self.get_logger().error(f"Failed to save configuration: {e}")

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    calibrator = TransformationCalibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
