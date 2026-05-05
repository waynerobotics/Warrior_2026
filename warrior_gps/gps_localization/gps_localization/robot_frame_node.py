#!/usr/bin/env python3
"""
Robot Frame Node - Handles the robot's local position relative to its starting point
This node tracks the robot's movement in its local coordinate frame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
import math
import numpy as np
from typing import Optional


class RobotFrameNode(Node):
    def __init__(self):
        super().__init__('robot_frame_node')
        
        # Robot identification
        self.declare_parameter('robot_name', 'mini_shanti')
        self.robot_name = self.get_parameter('robot_name').value
        
        # Initial position (can be set from AprilTag detection)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        
        # Get initial position
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_z = self.get_parameter('initial_z').value
        
        # Current robot position in local frame
        self.current_x = self.initial_x
        self.current_y = self.initial_y
        self.current_z = self.initial_z
        self.current_yaw = 0.0
        
        # Publishers
        self.local_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/local_pose',
            10
        )
        
        self.transform_pub = self.create_publisher(
            TransformStamped,
            f'/{self.robot_name}/local_transform',
            10
        )
        
        # Subscribers (for robot movement updates - can come from AprilTag)
        self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/apriltag_pose',  # Subscribe to AprilTag updates
            self.apriltag_pose_callback,
            10
        )
        
        # Timer for publishing current pose
        self.timer = self.create_timer(0.1, self.publish_local_pose)  # 10 Hz
        
        self.get_logger().info(f'Robot Frame Node initialized for {self.robot_name}')
        self.get_logger().info(f'Initial position: x={self.initial_x}, y={self.initial_y}, z={self.initial_z}')
    
    def apriltag_pose_callback(self, msg: PoseStamped):
        """
        Update robot position based on AprilTag detection
        This integrates with your existing AprilTag detection code
        """
        # Update current position from AprilTag
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        # Calculate yaw from quaternion if needed
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # Convert quaternion to yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().debug(f'Updated position from AprilTag: x={self.current_x:.3f}, y={self.current_y:.3f}')
    
    def publish_local_pose(self):
        """
        Publish the robot's current pose in its local frame
        """
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f'{self.robot_name}_local_frame'
        
        # Set position
        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = self.current_y
        pose_msg.pose.position.z = self.current_z
        
        # Convert yaw to quaternion
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        
        # Publish pose
        self.local_pose_pub.publish(pose_msg)
        
        # Also publish as transform for visualization
        transform_msg = TransformStamped()
        transform_msg.header = pose_msg.header
        transform_msg.child_frame_id = f'{self.robot_name}_base_link'
        transform_msg.transform.translation.x = self.current_x
        transform_msg.transform.translation.y = self.current_y
        transform_msg.transform.translation.z = self.current_z
        transform_msg.transform.rotation = pose_msg.pose.orientation
        
        self.transform_pub.publish(transform_msg)
    
    def update_position_from_velocity(self, linear_vel: float, angular_vel: float, dt: float):
        """
        Update position based on velocity commands (for simulation/testing)
        """
        # Update yaw
        self.current_yaw += angular_vel * dt
        
        # Update position based on velocity and heading
        self.current_x += linear_vel * math.cos(self.current_yaw) * dt
        self.current_y += linear_vel * math.sin(self.current_yaw) * dt
    
    def get_current_position(self) -> tuple:
        """
        Get current position as tuple
        """
        return (self.current_x, self.current_y, self.current_z, self.current_yaw)


def main(args=None):
    rclpy.init(args=args)
    
    robot_frame_node = RobotFrameNode()
    
    try:
        rclpy.spin(robot_frame_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_frame_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
