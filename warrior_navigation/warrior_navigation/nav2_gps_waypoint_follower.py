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


class Nav2GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('nav2_gps_waypoint_follower')

        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('map_origin_latitude', 0.0)
        self.declare_parameter('map_origin_longitude', 0.0)
        self.declare_parameter('map_origin_yaw', 0.0)
        self.declare_parameter('utm_zone', 10)
        self.declare_parameter('utm_hemisphere', 'N')
        self.declare_parameter('wait_for_action_timeout', 30.0)

        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        self.map_origin_lat = self.get_parameter('map_origin_latitude').get_parameter_value().double_value
        self.map_origin_lon = self.get_parameter('map_origin_longitude').get_parameter_value().double_value
        self.map_origin_yaw = self.get_parameter('map_origin_yaw').get_parameter_value().double_value
        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().integer_value
        self.utm_hemisphere = self.get_parameter('utm_hemisphere').get_parameter_value().string_value.upper()
        self.wait_for_action_timeout = self.get_parameter('wait_for_action_timeout').get_parameter_value().double_value

        if not waypoint_file:
            waypoint_file = os.path.join(
                get_package_share_directory('warrior_navigation'),
                'config',
                'turtlebot_sim_waypoints.yaml'
            )
            self.get_logger().info(f'No waypoint file provided, using default: {waypoint_file}')

        self.waypoints_gps = self._load_waypoints(waypoint_file)
        if not self.waypoints_gps:
            self.get_logger().error('No waypoints found. Shutting down.')
            return

        self.waypoints_map = self._convert_gps_to_map_frame(self.waypoints_gps)
        self.get_logger().info(f'Loaded {len(self.waypoints_map)} GPS waypoints in map frame.')

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
        for idx, waypoint in enumerate(self.waypoints_map):
            self.get_logger().info(f'Sending waypoint {idx + 1}/{len(self.waypoints_map)} to Nav2.')
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._create_pose_stamped(waypoint)

            send_goal_future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().warn(f'Waypoint {idx + 1} was rejected by {self.action_name}.')
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Waypoint {idx + 1} reached successfully.')
            else:
                self.get_logger().warn(
                    f'Waypoint {idx + 1} failed with status {result.status}. Continuing to next waypoint.'
                )

        self.get_logger().info('GPS waypoint mission complete.')


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
