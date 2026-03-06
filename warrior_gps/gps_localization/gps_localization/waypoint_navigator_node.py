#!/usr/bin/env python3
"""
Waypoint Navigator Node - GPS to UTM Conversion and Waypoint Tracking
Converts GPS coordinates to UTM (Zone 17 for Detroit area)
Tracks distance to next waypoint and manages waypoint progression
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
import math
from typing import List, Tuple, Optional


class UTMConverter:
    """Convert between GPS (lat/lon) and UTM coordinates"""
    
    # WGS84 ellipsoid parameters
    WGS84_A = 6378137.0  # Semi-major axis
    WGS84_B = 6356752.314245  # Semi-minor axis
    WGS84_E = 0.0818191908426  # Eccentricity
    
    @staticmethod
    def gps_to_utm(latitude: float, longitude: float, zone: int = 17) -> Tuple[float, float]:
        """
        Convert GPS coordinates to UTM
        
        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            zone: UTM zone (17 for Detroit area)
            
        Returns:
            Tuple of (easting, northing) in meters
        """
        # Convert to radians
        lat_rad = math.radians(latitude)
        lon_rad = math.radians(longitude)
        
        # Calculate zone central meridian
        lon_origin = (zone - 1) * 6 - 180 + 3
        lon_origin_rad = math.radians(lon_origin)
        
        # Calculate UTM parameters
        n = UTMConverter.WGS84_A / math.sqrt(1 - UTMConverter.WGS84_E**2 * math.sin(lat_rad)**2)
        t = math.tan(lat_rad)**2
        c = UTMConverter.WGS84_E**2 / (1 - UTMConverter.WGS84_E**2) * math.cos(lat_rad)**2
        a = math.cos(lat_rad) * (lon_rad - lon_origin_rad)
        
        # Calculate meridian arc
        m = UTMConverter.WGS84_A * (
            (1 - UTMConverter.WGS84_E**2 / 4 - 3 * UTMConverter.WGS84_E**4 / 64 - 5 * UTMConverter.WGS84_E**6 / 256) * lat_rad
            - (3 * UTMConverter.WGS84_E**2 / 8 + 3 * UTMConverter.WGS84_E**4 / 32 - 45 * UTMConverter.WGS84_E**6 / 1024) * math.sin(2 * lat_rad)
            + (15 * UTMConverter.WGS84_E**4 / 256 - 45 * UTMConverter.WGS84_E**6 / 1024) * math.sin(4 * lat_rad)
            - (35 * UTMConverter.WGS84_E**6 / 3072) * math.sin(6 * lat_rad)
        )
        
        # Calculate easting and northing
        easting = 500000 + 0.9996 * n * (a + a**3 / 6 * (1 - t + c) + a**5 / 120 * (5 - 18 * t + t**2 + 72 * c - 58 * UTMConverter.WGS84_E**2))
        northing = 0.9996 * (m + n * math.tan(lat_rad) * (a**2 / 2 + a**4 / 24 * (5 - t + 9 * c + 4 * c**2) + a**6 / 720 * (61 - 58 * t + t**2 + 600 * c - 330 * UTMConverter.WGS84_E**2)))
        
        # False northing for southern hemisphere
        if latitude < 0:
            northing += 10000000
        
        return easting, northing


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Robot identification
        self.declare_parameter('robot_name', 'mini_shanti')
        self.robot_name = self.get_parameter('robot_name').value
        
        # UTM Zone
        self.declare_parameter('utm_zone', 17)
        self.utm_zone = self.get_parameter('utm_zone').value
        
        # Waypoint tolerance (meters) - robot must be within this distance to complete waypoint
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        
        # Waypoints as GPS coordinates (lat, lon)
        # Competition provides 4 waypoints: 2 for entrance/exit, 2 for navigation
        self.declare_parameter('waypoint_1_lat', 42.3558186)
        self.declare_parameter('waypoint_1_lon', -83.0707180)
        self.declare_parameter('waypoint_2_lat', 42.35581677)
        self.declare_parameter('waypoint_2_lon', -83.070701)
        self.declare_parameter('waypoint_3_lat', 42.3558071)
        self.declare_parameter('waypoint_3_lon', -83.0707157)
        self.declare_parameter('waypoint_4_lat', 42.3558072)
        self.declare_parameter('waypoint_4_lon', -83.070723)
        
        # Load waypoints
        self.waypoints_gps = [
            (self.get_parameter('waypoint_1_lat').value, self.get_parameter('waypoint_1_lon').value),
            (self.get_parameter('waypoint_2_lat').value, self.get_parameter('waypoint_2_lon').value),
            (self.get_parameter('waypoint_3_lat').value, self.get_parameter('waypoint_3_lon').value),
            (self.get_parameter('waypoint_4_lat').value, self.get_parameter('waypoint_4_lon').value),
        ]
        
        # Convert waypoints to UTM
        self.waypoints_utm = []
        for lat, lon in self.waypoints_gps:
            easting, northing = UTMConverter.gps_to_utm(lat, lon, self.utm_zone)
            self.waypoints_utm.append((easting, northing))
        
        # Current waypoint index (0-3)
        self.current_waypoint_idx = 0
        
        # Current robot position (UTM)
        self.robot_easting = None
        self.robot_northing = None
        self.distance_to_next_waypoint = None
        self.waypoints_completed = []
        
        # Publishers
        self.distance_pub = self.create_publisher(
            Float32,
            f'/{self.robot_name}/distance_to_waypoint',
            10
        )
        
        self.waypoint_status_pub = self.create_publisher(
            String,
            f'/{self.robot_name}/waypoint_status',
            10
        )
        
        # Subscribers
        self.create_subscription(
            NavSatFix,
            f'/{self.robot_name}/gps_position',
            self.gps_callback,
            10
        )
        
        # Timer to publish waypoint info
        self.timer = self.create_timer(0.5, self.publish_waypoint_info)
        
        self.get_logger().info(f'Waypoint Navigator initialized for {self.robot_name}')
        self.log_waypoints()
    
    def log_waypoints(self):
        """Log all waypoints in GPS and UTM"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('WAYPOINT CONFIGURATION (UTM Zone 17)')
        self.get_logger().info('=' * 60)
        
        for i, ((lat, lon), (easting, northing)) in enumerate(zip(self.waypoints_gps, self.waypoints_utm), 1):
            self.get_logger().info(f'Waypoint {i}:')
            self.get_logger().info(f'  GPS:  Lat={lat:.7f}°, Lon={lon:.7f}°')
            self.get_logger().info(f'  UTM:  Easting={easting:.2f}m, Northing={northing:.2f}m')
        
        self.get_logger().info(f'Waypoint Tolerance: {self.waypoint_tolerance:.1f} meters')
        self.get_logger().info('=' * 60)
    
    def gps_callback(self, msg: NavSatFix):
        """Update robot position from GPS"""
        # Convert GPS to UTM
        self.robot_easting, self.robot_northing = UTMConverter.gps_to_utm(
            msg.latitude,
            msg.longitude,
            self.utm_zone
        )
        
        # Calculate distance to current waypoint
        if self.current_waypoint_idx < len(self.waypoints_utm):
            wp_easting, wp_northing = self.waypoints_utm[self.current_waypoint_idx]
            
            # Euclidean distance in UTM coordinates
            self.distance_to_next_waypoint = math.sqrt(
                (self.robot_easting - wp_easting)**2 + 
                (self.robot_northing - wp_northing)**2
            )
            
            # Check if waypoint completed
            if self.distance_to_next_waypoint <= self.waypoint_tolerance:
                self.complete_waypoint()
    
    def complete_waypoint(self):
        """Mark current waypoint as completed and advance to next"""
        self.waypoints_completed.append(self.current_waypoint_idx + 1)
        
        self.get_logger().info(f'✓ Waypoint {self.current_waypoint_idx + 1} COMPLETED!')
        self.get_logger().info(f'  Waypoints cleared: {len(self.waypoints_completed)}/4')
        
        # Advance to next waypoint
        if self.current_waypoint_idx < len(self.waypoints_utm) - 1:
            self.current_waypoint_idx += 1
            wp_easting, wp_northing = self.waypoints_utm[self.current_waypoint_idx]
            self.get_logger().info(f'→ Now navigating to Waypoint {self.current_waypoint_idx + 1}')
            self.get_logger().info(f'  Target UTM: Easting={wp_easting:.2f}m, Northing={wp_northing:.2f}m')
        else:
            self.get_logger().info('🎉 ALL WAYPOINTS COMPLETED!')
    
    def publish_waypoint_info(self):
        """Publish current waypoint distance and status"""
        if self.distance_to_next_waypoint is None:
            return
        
        # Publish distance
        distance_msg = Float32()
        distance_msg.data = self.distance_to_next_waypoint
        self.distance_pub.publish(distance_msg)
        
        # Publish status
        if self.current_waypoint_idx < len(self.waypoints_utm):
            status_msg = String()
            status_msg.data = (
                f'Waypoint {self.current_waypoint_idx + 1}/4 | '
                f'Distance: {self.distance_to_next_waypoint:.2f}m | '
                f'Completed: {len(self.waypoints_completed)}/4'
            )
            self.waypoint_status_pub.publish(status_msg)
    
    def get_waypoint_info(self) -> dict:
        """
        Get current waypoint information for display
        Returns dict with distance, current waypoint, and progress
        """
        return {
            'current_waypoint': self.current_waypoint_idx + 1,
            'total_waypoints': len(self.waypoints_utm),
            'distance_to_waypoint': self.distance_to_next_waypoint,
            'waypoints_completed': len(self.waypoints_completed),
            'robot_utm': (self.robot_easting, self.robot_northing),
            'target_utm': self.waypoints_utm[self.current_waypoint_idx] if self.current_waypoint_idx < len(self.waypoints_utm) else None,
        }


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()