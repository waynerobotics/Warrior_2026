##############################################################################
# Transformation Reconfiguration Server - Dynamic parameter adjustment for LiDAR-camera alignment
#
# Software License Agreement (GPL-3.0)
# Copyright (C) 2025 Shanti Robot Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.
##############################################################################

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
import numpy as np
import math
import os
import yaml

class TransformationReconfigurationServer(Node):
    def __init__(self):
        super().__init__('transformation_reconfiguration_server')

        # Define parameter descriptor with range for yaw angle
        yaw_descriptor = ParameterDescriptor(
            description='Yaw angle in degrees for camera-LiDAR alignment',
            floating_point_range=[FloatingPointRange(
                from_value=-180.0,
                to_value=180.0,
                step=0.1
            )]
        )

        # Declare parameters
        self.declare_parameter('base_transformation_matrix', [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ])
        self.declare_parameter('yaw_angle', 0.0, yaw_descriptor)
        self.declare_parameter(
            'config_file_path',
            os.path.join(
                os.path.expanduser('~'),
                'ros2_ws/src/Warrior_2026/warrior_perception/omnivision/config/transformation.yaml'))

        # Get initial parameters
        self.base_transformation = np.array(
            self.get_parameter('base_transformation_matrix').value).reshape((4, 4))
        self.yaw_angle = self.get_parameter('yaw_angle').value
        self.config_file_path = self.get_parameter('config_file_path').value

        # Ensure config directory exists
        os.makedirs(os.path.dirname(self.config_file_path), exist_ok=True)

        # Calculate initial transformation matrix
        self.current_transformation = self.calculate_transformation_matrix(self.yaw_angle)

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Print the initial transformation matrix
        self.print_transformation_matrix()

        # Set up a timer to periodically print the matrix (useful for adjusting via command line)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Transformation Reconfiguration Server started")
        self.get_logger().info("Use 'ros2 param set /transformation_reconfiguration_server yaw_angle X.X' to adjust the yaw")
        self.get_logger().info("Current yaw angle: {} degrees".format(self.yaw_angle))

    def calculate_transformation_matrix(self, yaw_angle):
        """Calculate transformation matrix from yaw angle"""
        angle_rad = math.radians(yaw_angle)
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
        return np.matmul(rotation_matrix, self.base_transformation)

    def parameters_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == 'yaw_angle' and param.value != self.yaw_angle:
                self.yaw_angle = param.value
                self.current_transformation = self.calculate_transformation_matrix(self.yaw_angle)
                self.print_transformation_matrix()
                self.save_transformation()

        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Periodically print the current transformation matrix"""
        pass  # Only print when the parameter changes

    def print_transformation_matrix(self):
        """Print the current transformation matrix in a readable format"""
        self.get_logger().info("Current transformation matrix (yaw = {} degrees):".format(self.yaw_angle))

        # Print in readable 4x4 format
        for i in range(4):
            row = " ".join([f"{self.current_transformation[i, j]:.6f}" for j in range(4)])
            self.get_logger().info(f"[{row}]")

        # Print in flattened format for parameter copy-paste
        flat_array = self.current_transformation.flatten().tolist()
        matrix_str = '[' + ', '.join([f"{val:.6f}" for val in flat_array]) + ']'
        self.get_logger().info("Flattened matrix for parameter copy-paste:")
        self.get_logger().info(matrix_str)

    def save_transformation(self):
        """Save the current transformation matrix to YAML file"""
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
        except Exception as e:
            self.get_logger().error(f"Failed to save configuration: {e}")

def main(args=None):
    rclpy.init(args=args)
    server = TransformationReconfigurationServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
