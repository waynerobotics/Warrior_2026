import math
import os
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from pyproj import Proj
from rclpy.action import ActionClient
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path


from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy

class Nav2GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('nav2_gps_waypoint_follower')

        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('waypoint_file', '')

        self.declare_parameter('utm_zone', 17)
        self.declare_parameter('utm_hemisphere', 'N')
        self.declare_parameter('wait_for_action_timeout', 30.0)

        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().integer_value
        self.utm_hemisphere = self.get_parameter('utm_hemisphere').get_parameter_value().string_value.upper()
        self.wait_for_action_timeout = self.get_parameter('wait_for_action_timeout').get_parameter_value().double_value

        if not waypoint_file:
            self.get_logger().info(f'No waypoint file provided, shutting down.')
            return

        self.waypoints_gps = self._load_waypoints(waypoint_file)
        if not self.waypoints_gps:
            self.get_logger().error('No waypoints found. Shutting down.')
            return
        
        # definde map origin as first waypoint from first loaded file in yaml
        origin = self.waypoints_gps[0]
        self.map_origin_lat = origin.get('latitude', 0.0)
        self.map_origin_lon = origin.get('longitude', 0.0)
        self.map_origin_yaw = origin.get('yaw', 0.0)

        self.waypoints_map = self._convert_gps_to_map_frame(self.waypoints_gps)
        self.get_logger().info(f'Loaded {len(self.waypoints_map)} GPS waypoints in map frame.')

        self._current_waypoint_index = 0
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, '/gps_waypoints', 10)
        self.trajectory_pub = self.create_publisher(Path, '/gps_waypoint_trajectory', 10)
        self._publish_waypoint_markers()
        self._publish_trajectory()

        self._action_client = ActionClient(self, NavigateToPose, self.action_name)
        self._startup_timer = self.create_timer(1.0, self._startup_callback)
        self._mission_started = False

    def _load_waypoints(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            if not isinstance(data, dict) or 'waypoints' not in data:
                self.get_logger().error(f'Waypoint file is missing the "waypoints" key: {filepath}')
                return []

            waypoints = []
            for wp in data['waypoints']:
                if 'latitude' not in wp or 'longitude' not in wp:
                    self.get_logger().warn(f'Skipping invalid waypoint entry: {wp}')
                    continue
                waypoints.append({
                    'latitude': wp['latitude'],
                    'longitude': wp['longitude'],
                    'yaw': wp.get('yaw', 0.0),
                })
            return waypoints
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoint file {filepath}: {exc}')
            return []

    def _convert_gps_to_map_frame(self, waypoints_gps):
        utm_proj = Proj(proj='utm', zone=self.utm_zone, ellps='WGS84', south=(self.utm_hemisphere == 'S'))
        origin_x_utm, origin_y_utm = utm_proj(self.map_origin_lon, self.map_origin_lat)

        self.get_logger().info(
            f'Map origin GPS({self.map_origin_lat:.6f}, {self.map_origin_lon:.6f}) ' \
            f'-> UTM({origin_x_utm:.2f}, {origin_y_utm:.2f})'
        )

        waypoints_map = []
        cos_yaw = math.cos(self.map_origin_yaw)
        sin_yaw = math.sin(self.map_origin_yaw)

        for i, wp in enumerate(waypoints_gps):
            wp_x_utm, wp_y_utm = utm_proj(wp['longitude'], wp['latitude'])
            dx = wp_x_utm - origin_x_utm
            dy = wp_y_utm - origin_y_utm
            x = cos_yaw * dx - sin_yaw * dy
            y = sin_yaw * dx + cos_yaw * dy
            pose_yaw = wp['yaw'] - self.map_origin_yaw
            waypoints_map.append({'x': x, 'y': y, 'yaw': pose_yaw})
            self.get_logger().debug(
                f'Waypoint {i}: GPS({wp["latitude"]:.6f}, {wp["longitude"]:.6f}) -> '
                f'map({x:.3f}, {y:.3f}, yaw={pose_yaw:.3f})'
            )

        return waypoints_map

    def _create_pose_stamped(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint['x']
        pose.pose.position.y = waypoint['y']
        pose.pose.position.z = 0.0

        yaw = waypoint['yaw']
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        return pose

    def _publish_waypoint_markers(self):
        marker_array = MarkerArray()
        for i, wp in enumerate(self.waypoints_map):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            if i < self._current_waypoint_index:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7
            elif i == self._current_waypoint_index:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7

            marker.lifetime.sec = 0
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = wp['x']
            text_marker.pose.position.y = wp['y']
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.text = f'WP{i+1}'
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime.sec = 0
            marker_array.markers.append(text_marker)

        self.waypoint_markers_pub.publish(marker_array)

    def _publish_trajectory(self):
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
            yaw = wp['yaw']
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            path.poses.append(pose)

        self.trajectory_pub.publish(path)

    def _startup_callback(self):
        if self._mission_started:
            return

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for Nav2 action server {self.action_name}...')
            return

        self._mission_started = True
        self._startup_timer.cancel()
        self.get_logger().info(f'Nav2 action server {self.action_name} is available. Beginning waypoint mission.')
        self._execute_mission()


    def _execute_mission(self):
        self._current_waypoint_index = 0
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self._current_waypoint_index >= len(self.waypoints_map):
            self.get_logger().info('GPS waypoint mission complete.')
            return

        idx = self._current_waypoint_index
        waypoint = self.waypoints_map[idx]

        self.get_logger().info(
            f'Sending waypoint {idx + 1}/{len(self.waypoints_map)} to Nav2.'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(waypoint)

        send_goal_future = self._action_client.send_goal_async(goal_msg)

        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        idx = self._current_waypoint_index

        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Waypoint {idx + 1} was rejected by {self.action_name}.'
            )

            self._current_waypoint_index += 1
            self._send_next_waypoint()
            return
        self.get_logger().info(
            f'Waypoint {idx + 1} accepted.'
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    

    def _result_callback(self, future):
        result = future.result()

        idx = self._current_waypoint_index

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Waypoint {idx + 1} reached successfully.'
            )
        else:
            self.get_logger().warn(
                f'Waypoint {idx + 1} failed with status {result.status}.'
            )

        self._current_waypoint_index += 1
        self._publish_waypoint_markers()

        self._send_next_waypoint()

def main():
    rclpy.init()
    node = Nav2GpsWaypointFollower()
    if not node.waypoints_gps:
        node.destroy_node()
        rclpy.shutdown()
        return

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
