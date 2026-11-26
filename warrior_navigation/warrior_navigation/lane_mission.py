#!/usr/bin/env python3
"""IGVC AutoNav mission: lane following, GPS targets in the no-man's-land, finish.

Sequencing lives in ``mission_logic.Mission``; this node supplies the robot
pose, converts the no-man's-land GPS targets to the map frame (navsat
``/fromLL``), and executes the returned actions:

* lane following on/off and its travel heading via ``/lane_follower/enable``
  and ``/lane_follower/heading_hint``;
* Nav2 ``navigate_to_pose`` goals for no-man's-land targets and nudges;
* cancel when the lap is complete.
"""

import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from robot_localization.srv import FromLL
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from warrior_navigation.mission_logic import Mission, MissionParams
from warrior_navigation.waypoint_utils import load_waypoints


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LaneMission(Node):
    def __init__(self):
        super().__init__('lane_mission')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('from_ll_service', '/fromLL')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('lane_follower', '/lane_follower')
        for name, value in vars(MissionParams()).items():
            self.declare_parameter(name, value)
        self.params = MissionParams(**{k: type(v)(self.get_parameter(k).value)
                                       for k, v in vars(MissionParams()).items()})
        self.frame = self.get_parameter('global_frame').value
        self.base = self.get_parameter('base_frame').value
        self.gps = load_waypoints(self.get_parameter('waypoint_file').value)
        if len(self.gps) < 2:
            raise ValueError('The no-man\'s-land needs at least an entrance and an exit target')

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.from_ll = self.create_client(FromLL, self.get_parameter('from_ll_service').value)
        self.nav = ActionClient(self, NavigateToPose, self.get_parameter('action_name').value)
        lf = self.get_parameter('lane_follower').value.rstrip('/')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_pub = self.create_publisher(Bool, f'{lf}/enable', latched)
        self.heading_pub = self.create_publisher(Float64, f'{lf}/heading_hint', latched)
        self.state_pub = self.create_publisher(String, '~/state', latched)

        self.nml_map = []
        self.pending_conversion = None
        self.mission = None
        self.goal_seq = 0
        self.goal_handle = None
        self.logged = 0
        self.enable_pub.publish(Bool(data=False))   # nothing drives until we are ready
        self.create_timer(0.5, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.frame, self.base, Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    # ------------------------------------------------------------- startup
    def startup(self):
        """Convert the GPS targets, then start once Nav2 is up. True when started."""
        if len(self.nml_map) < len(self.gps):
            if self.pending_conversion is None:
                if not self.from_ll.service_is_ready():
                    self.say('waiting for /fromLL')
                    return False
                wp = self.gps[len(self.nml_map)]
                req = FromLL.Request()
                req.ll_point.latitude = wp['latitude']
                req.ll_point.longitude = wp['longitude']
                req.ll_point.altitude = wp['altitude']
                self.pending_conversion = self.from_ll.call_async(req)
            elif self.pending_conversion.done():
                p = self.pending_conversion.result().map_point
                self.nml_map.append((p.x, p.y))
                self.pending_conversion = None
            return False
        pose = self.pose()
        if pose is None or not self.nav.server_is_ready():
            self.say('waiting for TF and Nav2')
            return False
        self.mission = Mission(nml=self.nml_map, params=self.params)
        self.execute(self.mission.begin(*pose, self.now_s()))
        self.get_logger().info('Mission started; no-man\'s-land targets (map): ' +
                               ', '.join(f'({x:.1f}, {y:.1f})' for x, y in self.nml_map))
        return True

    # ---------------------------------------------------------------- loop
    def tick(self):
        if self.mission is None and not self.startup():
            return
        pose = self.pose()
        if pose is not None:
            self.execute(self.mission.update(pose[0], pose[1], self.now_s()))
        for line in self.mission.log[self.logged:]:
            self.get_logger().info(line)
        self.logged = len(self.mission.log)
        self.say(f'{self.mission.state} travelled={self.mission.travelled:.0f}m '
                 f'nml_target={self.mission.index + 1}/{len(self.nml_map)}')

    def execute(self, actions):
        for act in actions:
            if act[0] == 'lane':
                self.enable_pub.publish(Bool(data=act[1]))
            elif act[0] == 'heading':
                self.heading_pub.publish(Float64(data=float(act[1])))
            elif act[0] == 'goal':
                self.send_goal(*act[1:])
            elif act[0] == 'cancel':
                self.goal_seq += 1
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.get_logger().info('Lap complete: lane following off, robot stops.')

    def send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.goal_seq += 1
        seq = self.goal_seq
        self.get_logger().info(f'[{self.mission.state}] goal ({x:.1f}, {y:.1f})')
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_accepted(f, seq))

    def on_accepted(self, future, seq):
        handle = future.result()
        if handle is None or not handle.accepted:
            if seq == self.goal_seq:
                self.finished(False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: seq == self.goal_seq and self.finished(f.result().status == GoalStatus.STATUS_SUCCEEDED))

    def finished(self, success):
        pose = self.pose() or (0.0, 0.0, 0.0)
        self.execute(self.mission.goal_finished(success, pose[0], pose[1], self.now_s()))

    def say(self, text):
        self.state_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = LaneMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
