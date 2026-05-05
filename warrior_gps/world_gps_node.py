#!/usr/bin/env python3
"""
World GPS Node - Hybrid Mode
Can use either real GPS from GT-U7 or fixed coordinates
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
import math

class WorldGPSNode(Node):
    def __init__(self):
        super().__init__('world_gps_node')
        
        # Mode selection
        self.declare_parameter('use_hardware_gps', False)  # Set to True when GPS is connected
        self.use_hardware_gps = self.get_parameter('use_hardware_gps').value
        
        # Fixed GPS reference (used when no hardware GPS)
        self.declare_parameter('fixed_latitude', 42.363945)
        self.declare_parameter('fixed_longitude', -83.073175)
        self.declare_parameter('fixed_altitude', 200.0)
        
        # Get fixed coordinates
        self.fixed_lat = self.get_parameter('fixed_latitude').value
        self.fixed_lon = self.get_parameter('fixed_longitude').value
        self.fixed_alt = self.get_parameter('fixed_altitude').value
        
        # Current reference (will be updated by hardware GPS if available)
        self.ref_latitude = self.fixed_lat
        self.ref_longitude = self.fixed_lon
        self.ref_altitude = self.fixed_alt
        self.gps_fixed = False
        
        # Publishers
        self.gps_publisher = self.create_publisher(
            NavSatFix,
            '/robot1/gps_position',
            10
        )
        
        self.ref_publisher = self.create_publisher(
            NavSatFix,
            '/gps_reference',
            10
        )
        
        # Subscribe to robot position from AprilTag
        self.create_subscription(
            PoseStamped,
            '/robot1/apriltag_pose',
            self.convert_to_gps,
            10
        )
        
        # Subscribe to hardware GPS if enabled
        if self.use_hardware_gps:
            self.create_subscription(
                NavSatFix,
                '/master_gps_position',
                self.update_gps_reference,
                10
            )
            self.get_logger().info('Hardware GPS mode enabled - waiting for GT-U7 data')
        else:
            self.get_logger().info('Fixed GPS mode - using configured coordinates')
            self.gps_fixed = True
        
        # Timer to publish reference
        self.timer = self.create_timer(1.0, self.publish_reference)
        
        self.log_reference_point()
    
    def update_gps_reference(self, msg):
        """Update reference from hardware GPS"""
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.ref_latitude = msg.latitude
            self.ref_longitude = msg.longitude
            self.ref_altitude = msg.altitude
            
            if not self.gps_fixed:
                self.gps_fixed = True
                self.get_logger().info('GPS Fix acquired from GT-U7!')
                self.log_reference_point()
    
    def log_reference_point(self):
        """Log current reference point"""
        mode = "Hardware GPS (GT-U7)" if self.use_hardware_gps else "Fixed Coordinates"
        status = "ACTIVE" if self.gps_fixed else "WAITING FOR FIX..."
        
        self.get_logger().info(f'GPS Reference Mode: {mode}')
        self.get_logger().info(f'Status: {status}')
        if self.gps_fixed:
            self.get_logger().info(f'Reference Point:')
            self.get_logger().info(f'  Latitude:  {self.ref_latitude:.7f}°')
            self.get_logger().info(f'  Longitude: {self.ref_longitude:.7f}°')
            self.get_logger().info(f'  Altitude:  {self.ref_altitude:.1f} m')
    
    def publish_reference(self):
        """Publish the GPS reference point"""
        ref_msg = NavSatFix()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = 'gps_reference'
        
        ref_msg.latitude = self.ref_latitude
        ref_msg.longitude = self.ref_longitude
        ref_msg.altitude = self.ref_altitude
        
        if self.gps_fixed:
            ref_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            ref_msg.status.status = NavSatStatus.STATUS_NO_FIX
        
        ref_msg.status.service = NavSatStatus.SERVICE_GPS
        
        self.ref_publisher.publish(ref_msg)
    
    def convert_to_gps(self, msg):
        """Convert AprilTag position to GPS coordinates"""
        if not self.gps_fixed and self.use_hardware_gps:
            self.get_logger().warn_once('Waiting for GPS fix from GT-U7...')
            return
        
        # Get position in meters from AprilTag
        x_meters = msg.pose.position.x  # East-West
        y_meters = msg.pose.position.y  # North-South
        z_meters = msg.pose.position.z  # Up-Down
        
        # Convert to GPS
        lat_change = y_meters / 111111.0
        meters_per_degree_lon = 111111.0 * math.cos(math.radians(self.ref_latitude))
        lon_change = x_meters / meters_per_degree_lon
        
        # Calculate robot GPS position
        robot_latitude = self.ref_latitude + lat_change
        robot_longitude = self.ref_longitude + lon_change
        robot_altitude = self.ref_altitude + z_meters
        
        # Create and publish GPS message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = msg.header.stamp
        gps_msg.header.frame_id = 'gps'
        
        gps_msg.latitude = robot_latitude
        gps_msg.longitude = robot_longitude
        gps_msg.altitude = robot_altitude
        
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Covariance
        gps_msg.position_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.01
        ]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_publisher.publish(gps_msg)
        
        # Log with source indication
        source = "GT-U7" if self.use_hardware_gps else "Fixed"
        self.get_logger().info(
            f'Robot GPS [{source}]: '
            f'Lat={robot_latitude:.7f}, '
            f'Lon={robot_longitude:.7f}, '
            f'Alt={robot_altitude:.2f}m | '
            f'Offset: X={x_meters:.2f}m, Y={y_meters:.2f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WorldGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()