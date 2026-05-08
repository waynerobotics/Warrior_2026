import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose as ComputePathToPoseAction
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import math

try:
    from pyproj import Proj, transform
except ImportError:
    raise ImportError("pyproj is required. Install with: pip install pyproj")


class GpsWaypointManager(Node):
    """
    Manages sequential GPS waypoint following using custom path planner/follower.
    
    Converts GPS waypoints (lat/lon) to map-frame poses (x/y in meters) and
    sequentially sends them to the recovery_manager action server.
    
    Requires:
    - Map origin GPS coordinate (latitude, longitude, yaw)
    - Path to YAML file containing GPS waypoints
    """

    def __init__(self):
        super().__init__('gps_waypoint_manager')
        self.get_logger().info('GPS Waypoint Manager initialized')

        # Declare parameters
        self.declare_parameter('action_name', 'path_to_pose')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('map_origin_latitude', 0.0)
        self.declare_parameter('map_origin_longitude', 0.0)
        self.declare_parameter('map_origin_yaw', 0.0)
        self.declare_parameter('utm_zone', 10)  # Sonoma State is zone 10
        self.declare_parameter('utm_hemisphere', "N")  # Northern hemisphere


        # Get parameters
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        self.map_origin_lat = self.get_parameter('map_origin_latitude').get_parameter_value().double_value
        self.map_origin_lon = self.get_parameter('map_origin_longitude').get_parameter_value().double_value
        self.map_origin_yaw = self.get_parameter('map_origin_yaw').get_parameter_value().double_value
        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().integer_value
        self.utm_hemisphere = self.get_parameter('utm_hemisphere').get_parameter_value().string_value

        # Handle waypoint file path
        if not waypoint_file:
            waypoint_file = os.path.join(
                get_package_share_directory('warrior_navigation'),
                'config',
                'turtlebot_sim_waypoints.yaml'
            )
            self.get_logger().info(f'No waypoint file specified, using default: {waypoint_file}')
        
        self.get_logger().info(f'Loading waypoints from: {waypoint_file}')
        
        # Load waypoints
        self.waypoints_gps = self._load_waypoints(waypoint_file)
        if not self.waypoints_gps:
            self.get_logger().error('Failed to load waypoints')
            return

        # Convert GPS to map frame
        self.waypoints_map = self._convert_gps_to_map_frame(self.waypoints_gps)
        self.get_logger().info(f'Converted {len(self.waypoints_map)} GPS waypoints to map frame')

        # Action client for recovery_manager
        self._action_client = ActionClient(self, ComputePathToPoseAction, self.action_name)
        
        # Visualization publishers
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, '/gps_waypoints', 10)
        self.trajectory_pub = self.create_publisher(Path, '/gps_waypoint_trajectory', 10)
        
        self.current_waypoint_idx = 0
        self.mission_active = False
        self.current_goal_handle = None
        
        # Publish initial waypoint visualization
        self._publish_waypoint_markers()
        self._publish_trajectory()

    def _load_waypoints(self, filepath):
        """Load GPS waypoints from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' not in data:
                self.get_logger().error(f'YAML file missing "waypoints" key')
                return None
            
            waypoints = []
            for wp in data['waypoints']:
                if 'latitude' not in wp or 'longitude' not in wp:
                    self.get_logger().warn(f'Waypoint missing latitude/longitude: {wp}')
                    continue
                
                yaw = wp.get('yaw', 0.0)
                waypoints.append({
                    'latitude': wp['latitude'],
                    'longitude': wp['longitude'],
                    'yaw': yaw
                })
            
            return waypoints
        
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return None

    def _convert_gps_to_map_frame(self, waypoints_gps):
        """
        Convert GPS waypoints (lat/lon) to map-frame coordinates (x/y in meters).
        
        Uses UTM projection centered at map_origin.
        """
        waypoints_map = []
        
        # Set up UTM projection
        utm_proj = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', 
                       south=(self.utm_hemisphere == 'S'))
        
        # Convert map origin to UTM
        origin_x_utm, origin_y_utm = utm_proj(self.map_origin_lon, self.map_origin_lat)
        
        self.get_logger().info(
            f'Map origin: GPS({self.map_origin_lat:.6f}, {self.map_origin_lon:.6f}) '
            f'→ UTM({origin_x_utm:.2f}, {origin_y_utm:.2f}, zone {self.utm_zone})'
        )
        
        for i, wp_gps in enumerate(waypoints_gps):
            lat = wp_gps['latitude']
            lon = wp_gps['longitude']
            yaw = wp_gps['yaw']
            
            # Convert waypoint to UTM
            wp_x_utm, wp_y_utm = utm_proj(lon, lat)
            
            # Transform to map frame (relative to origin, apply rotation)
            dx_utm = wp_x_utm - origin_x_utm
            dy_utm = wp_y_utm - origin_y_utm
            
            # Rotate to account for map origin yaw
            cos_yaw = math.cos(self.map_origin_yaw)
            sin_yaw = math.sin(self.map_origin_yaw)
            
            wp_x_map = cos_yaw * dx_utm - sin_yaw * dy_utm
            wp_y_map = sin_yaw * dx_utm + cos_yaw * dy_utm
            
            # Adjust waypoint yaw by map origin yaw
            wp_yaw_map = yaw - self.map_origin_yaw
            
            waypoints_map.append({
                'x': wp_x_map,
                'y': wp_y_map,
                'yaw': wp_yaw_map,
                'latitude': lat,
                'longitude': lon
            })
            
            self.get_logger().debug(
                f'WP{i}: GPS({lat:.6f}, {lon:.6f}) → Map({wp_x_map:.2f}, {wp_y_map:.2f}, yaw={wp_yaw_map:.2f})'
            )
        
        return waypoints_map

    def _create_pose_stamped(self, wp_map):
        """Create a PoseStamped message from a map-frame waypoint."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = wp_map['x']
        pose.pose.position.y = wp_map['y']
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = wp_map['yaw']
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sy
        pose.pose.orientation.w = cy
        
        return pose

    def _publish_waypoint_markers(self):
        """Publish waypoints as visualization markers in RViz."""
        marker_array = MarkerArray()
        
        for i, wp in enumerate(self.waypoints_map):
            # Create marker for waypoint
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.0
            
            # Orientation (identity)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Scale (radius of sphere)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color (green for unvisited, blue for current, yellow for visited)
            if i < self.current_waypoint_idx:
                # Visited - green
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7
            elif i == self.current_waypoint_idx:
                # Current - blue
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            else:
                # Unvisited - yellow
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7
            
            marker.lifetime.sec = 0  # Infinite lifetime
            marker_array.markers.append(marker)
            
            # Add text label with waypoint number
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = 1000 + i  # Offset ID to avoid conflicts
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = wp['x']
            text_marker.pose.position.y = wp['y']
            text_marker.pose.position.z = 0.3
            
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.text = f"WP{i+1}"
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime.sec = 0
            
            marker_array.markers.append(text_marker)
        
        self.waypoint_markers_pub.publish(marker_array)

    def _publish_trajectory(self):
        """Publish the expected trajectory through all waypoints."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints_map:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            yaw = wp['yaw']
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sy
            pose.pose.orientation.w = cy
            
            path.poses.append(pose)
        
        self.trajectory_pub.publish(path)

    def start_mission(self):
        """Start following GPS waypoints sequentially."""
        if not self.waypoints_map:
            self.get_logger().error('No waypoints loaded')
            return False
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server {self.action_name} not available')
            return False
        
        self.mission_active = True
        self.current_waypoint_idx = 0
        
        self.get_logger().info(f'Starting GPS waypoint mission with {len(self.waypoints_map)} waypoints')
        
        # Send first waypoint immediately
        self._process_next_waypoint()
        
        return True

    def _process_next_waypoint(self):
        """Send the next waypoint goal to the action server."""
        if self.current_waypoint_idx >= len(self.waypoints_map):
            self.get_logger().info('GPS waypoint mission completed successfully!')
            self.mission_active = False
            return
        
        wp = self.waypoints_map[self.current_waypoint_idx]
        pose = self._create_pose_stamped(wp)
        
        self.get_logger().info(
            f'Sending waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints_map)}: '
            f'({wp["x"]:.2f}, {wp["y"]:.2f}) GPS({wp["latitude"]:.6f}, {wp["longitude"]:.6f})'
        )
        
        goal = ComputePathToPoseAction.Goal()
        goal.goal = pose
        goal.use_start = False
        goal.planner_id = 'a_star'
        
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle response to goal send."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'Waypoint {self.current_waypoint_idx} rejected by planner')
            self.mission_active = False
            return
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Handle completion of waypoint goal."""
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'Failed to get result for waypoint {self.current_waypoint_idx}: {e}')
            self.mission_active = False
            return
        
        if result.error_code == ComputePathToPoseAction.Result.NONE:
            self.get_logger().info(
                f'✓ Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints_map)} reached successfully'
            )
        else:
            self.get_logger().warn(
                f'✗ Waypoint {self.current_waypoint_idx + 1} failed with error code {result.error_code}: '
                f'{result.error_msg}'
            )
        
        self.current_waypoint_idx += 1
        
        # Update visualization to show completed waypoint
        self._publish_waypoint_markers()
        
        # Process next waypoint immediately
        if self.mission_active:
            self._process_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    
    node = GpsWaypointManager()
    
    # Start mission
    if node.start_mission():
        try:
            # Spin node while mission is active
            while rclpy.ok() and node.mission_active:
                rclpy.spin_once(node, timeout_sec=0.1)
            
            # Spin a bit more to allow final callbacks to finish
            rclpy.spin_once(node, timeout_sec=1.0)
            
        except KeyboardInterrupt:
            node.get_logger().info('Mission cancelled by user')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.get_logger().error('Failed to start mission')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
