"""Follow GPS waypoints through Nav2, including goals outside a rolling map."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_localization.srv import FromLL
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from warrior_navigation.waypoint_utils import (
    costmap_allows_goal,
    load_waypoints,
    next_navigation_goal,
    project_waypoints,
)


class Nav2GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('nav2_gps_waypoint_follower')
        defaults = {
            'action_name': 'navigate_to_pose',
            'waypoint_file': '',
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'conversion_mode': 'projection',
            'from_ll_service': '/fromLL',
            'navsat_odom_topic': '/odometry/gps',
            'map_origin_latitude': float('nan'),
            'map_origin_longitude': float('nan'),
            'map_origin_yaw': 0.0,
            'utm_zone': 0,
            'utm_hemisphere': '',
            'intermediate_goal_distance': 0.0,
            'costmap_topic': '',
            'wait_for_action_timeout': 120.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.action_name = self.get_parameter('action_name').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.conversion_mode = self.get_parameter('conversion_mode').value
        self.map_origin_yaw = self.get_parameter('map_origin_yaw').value
        self.intermediate_goal_distance = self.get_parameter('intermediate_goal_distance').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self._costmap = None
        self._costmap_received = 0.0
        self.wait_for_action_timeout = self.get_parameter('wait_for_action_timeout').value
        self.waypoints_gps = []
        self.waypoints_map = []
        self._current_waypoint_index = 0
        self._mission_started = False
        self._mission_failed = False
        self._mission_complete = False
        self._send_pending = False
        self._goal_in_flight = False
        self._active_goal_is_final = True
        self._navsat_ready = False
        self._conversion_future = None
        self._startup_started = time.monotonic()
        self._tf_wait_started = self._startup_started
        self._last_wait_message = ''
        self._startup_timer = None

        try:
            if self.conversion_mode not in ('projection', 'from_ll'):
                raise ValueError('conversion_mode must be projection or from_ll')
            if not math.isfinite(self.map_origin_yaw):
                raise ValueError('map_origin_yaw must be finite')
            if (not math.isfinite(self.intermediate_goal_distance)
                    or self.intermediate_goal_distance < 0.0):
                raise ValueError('intermediate_goal_distance must be finite and nonnegative')
            if (not math.isfinite(self.wait_for_action_timeout)
                    or self.wait_for_action_timeout <= 0.0):
                raise ValueError('wait_for_action_timeout must be finite and positive')
            self.waypoints_gps = load_waypoints(self.get_parameter('waypoint_file').value)
            if self.conversion_mode == 'projection':
                latitude = self.get_parameter('map_origin_latitude').value
                longitude = self.get_parameter('map_origin_longitude').value
                if math.isnan(latitude) and math.isnan(longitude):
                    latitude = self.waypoints_gps[0]['latitude']
                    longitude = self.waypoints_gps[0]['longitude']
                    self.get_logger().info('Using the first waypoint as the projection origin.')
                self.waypoints_map = project_waypoints(
                    self.waypoints_gps, latitude, longitude, self.map_origin_yaw,
                    self.get_parameter('utm_zone').value,
                    self.get_parameter('utm_hemisphere').value,
                )
        except (OSError, ValueError, TypeError) as exc:
            self._fail_mission(f'Cannot initialize GPS waypoint mission: {exc}')
            return

        durable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoint_markers_pub = self.create_publisher(
            MarkerArray, '/gps_waypoints', durable_qos)
        self.trajectory_pub = self.create_publisher(
            Path, '/gps_waypoint_trajectory', durable_qos)
        self._action_client = ActionClient(self, NavigateToPose, self.action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        if self.costmap_topic:
            self._costmap_subscription = self.create_subscription(
                OccupancyGrid, self.costmap_topic, self._costmap_callback, durable_qos)
        if self.conversion_mode == 'from_ll':
            self._from_ll_client = self.create_client(
                FromLL, self.get_parameter('from_ll_service').value)
            self._navsat_subscription = self.create_subscription(
                Odometry, self.get_parameter('navsat_odom_topic').value,
                self._navsat_callback, 10)
        else:
            self._publish_waypoint_markers()
            self._publish_trajectory()

        # A steady clock keeps startup/TF timeouts bounded even if /clock stalls.
        self._startup_timer = self.create_timer(
            0.5, self._startup_callback, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'Loaded {len(self.waypoints_gps)} GPS waypoints; conversion={self.conversion_mode}, '
            f'frame={self.global_frame}, intermediate distance={self.intermediate_goal_distance:.1f} m.')

    def _navsat_callback(self, message):
        position = message.pose.pose.position
        if all(math.isfinite(value) for value in (position.x, position.y, position.z)):
            self._navsat_ready = True

    def _costmap_callback(self, message):
        info = message.info
        if (message.header.frame_id == self.global_frame and info.resolution > 0
                and info.width > 0 and info.height > 0
                and len(message.data) == info.width*info.height):
            self._costmap = message
            self._costmap_received = time.monotonic()

    def _wait_message(self, message):
        if message != self._last_wait_message:
            self.get_logger().info(message)
            self._last_wait_message = message

    def _fail_mission(self, message):
        self._mission_failed = True
        self._send_pending = False
        if self._startup_timer is not None:
            self._startup_timer.cancel()
        self.get_logger().error(message + ' Mission stopped; remaining waypoints were not attempted.')

    def _create_pose_stamped(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
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
            marker.header.frame_id = self.global_frame
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
            text_marker.header.frame_id = self.global_frame
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
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints_map:
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
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
        if self._mission_failed or self._mission_complete:
            return
        if self._mission_started:
            if self._send_pending:
                if time.monotonic() - self._tf_wait_started > self.wait_for_action_timeout:
                    self._fail_mission('Timed out waiting for the next goal: ' + self._last_wait_message)
                    return
                self._send_next_waypoint()
            return
        if time.monotonic() - self._startup_started > self.wait_for_action_timeout:
            self._fail_mission('Startup timed out. Last condition: ' + self._last_wait_message)
            return
        if not self._action_client.server_is_ready():
            self._wait_message(f'Waiting for Nav2 action server {self.action_name}...')
            return
        if not self._tf_buffer.can_transform(self.global_frame, self.robot_base_frame, Time()):
            self._wait_message(f'Waiting for TF {self.global_frame} -> {self.robot_base_frame}...')
            return
        if self.conversion_mode == 'from_ll' and len(self.waypoints_map) < len(self.waypoints_gps):
            if not self._navsat_ready:
                self._wait_message('Waiting for navsat odometry before GPS conversion...')
                return
            if not self._from_ll_client.service_is_ready():
                self._wait_message('Waiting for navsat FromLL service...')
                return
            if self._conversion_future is None:
                index = len(self.waypoints_map)
                waypoint = self.waypoints_gps[index]
                request = FromLL.Request()
                request.ll_point.latitude = waypoint['latitude']
                request.ll_point.longitude = waypoint['longitude']
                request.ll_point.altitude = waypoint['altitude']
                self._wait_message(f'Converting GPS waypoint {index + 1}/{len(self.waypoints_gps)}...')
                try:
                    self._conversion_future = self._from_ll_client.call_async(request)
                    self._conversion_future.add_done_callback(self._conversion_callback)
                except Exception as exc:
                    self._fail_mission(f'Could not request GPS conversion: {exc}')
            return

        self._publish_waypoint_markers()
        self._publish_trajectory()
        self._mission_started = True
        self.get_logger().info('Localization, waypoint conversion, and Nav2 are ready. Starting mission.')
        self._queue_next_goal()

    def _conversion_callback(self, future):
        if self._mission_failed:
            return
        try:
            point = future.result().map_point
            if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
                raise ValueError('FromLL returned a non-finite point')
            waypoint = self.waypoints_gps[len(self.waypoints_map)]
            self.waypoints_map.append({
                'x': point.x, 'y': point.y,
                'yaw': waypoint['yaw'] - self.map_origin_yaw,
            })
            self._conversion_future = None
        except Exception as exc:
            self._fail_mission(f'GPS conversion failed: {exc}')

    def _queue_next_goal(self):
        self._tf_wait_started = time.monotonic()
        self._send_pending = True
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self._mission_failed or self._goal_in_flight:
            return
        if self._current_waypoint_index >= len(self.waypoints_map):
            self._mission_complete = True
            self._send_pending = False
            self._startup_timer.cancel()
            self.get_logger().info('GPS waypoint mission complete: every waypoint was reached.')
            return
        waypoint = self.waypoints_map[self._current_waypoint_index]
        try:
            transform = self._tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, Time())
        except TransformException:
            self._wait_message(f'Waiting for robot pose in {self.global_frame} before the next goal...')
            return
        position = transform.transform.translation
        goal_waypoint, self._active_goal_is_final = next_navigation_goal(
            position.x, position.y, waypoint, self.intermediate_goal_distance)
        if not self._active_goal_is_final and self.costmap_topic:
            if self._costmap is None or time.monotonic()-self._costmap_received > 5.0:
                self._wait_message('Waiting for a current global costmap before selecting an intermediate goal...')
                return
            try:
                goal_waypoint, _ = next_navigation_goal(
                    position.x, position.y, waypoint, self.intermediate_goal_distance,
                    lambda x, y: costmap_allows_goal(self._costmap, x, y))
            except ValueError as exc:
                self._wait_message(str(exc))
                return
        goal = NavigateToPose.Goal()
        goal.pose = self._create_pose_stamped(goal_waypoint)
        kind = 'final target' if self._active_goal_is_final else 'intermediate target'
        self.get_logger().info(
            f'Waypoint {self._current_waypoint_index + 1}/{len(self.waypoints_map)}: '
            f'{kind} ({goal_waypoint["x"]:.2f}, {goal_waypoint["y"]:.2f}) in {self.global_frame}.')
        self._send_pending = False
        self._goal_in_flight = True
        try:
            future = self._action_client.send_goal_async(goal)
            future.add_done_callback(self._goal_response_callback)
        except Exception as exc:
            self._goal_in_flight = False
            self._fail_mission(f'Failed to send navigation goal: {exc}')

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._goal_in_flight = False
                self._fail_mission(f'Waypoint {self._current_waypoint_index + 1} was rejected.')
                return
            self._current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)
        except Exception as exc:
            self._goal_in_flight = False
            self._fail_mission(f'Navigation goal response failed: {exc}')

    def _result_callback(self, future):
        self._goal_in_flight = False
        try:
            result = future.result()
        except Exception as exc:
            self._fail_mission(f'Navigation result failed: {exc}')
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_mission(
                f'Waypoint {self._current_waypoint_index + 1} failed with action status {result.status}.')
            return
        if self._active_goal_is_final:
            self.get_logger().info(f'Waypoint {self._current_waypoint_index + 1} reached successfully.')
            self._current_waypoint_index += 1
            self._publish_waypoint_markers()
        # Each intermediate success obtains a fresh robot pose and keeps the
        # same GPS destination. Nav2 performs obstacle avoidance for every leg.
        self._queue_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GpsWaypointFollower()
    try:
        while rclpy.ok() and not node._mission_failed and not node._mission_complete:
            rclpy.spin_once(node, timeout_sec=0.2)
        # Keep successful mission visualization available to late RViz subscribers.
        if node._mission_complete:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node._mission_failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
