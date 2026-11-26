#!/usr/bin/env python3
"""Waypoint-free lane following: keep sending Nav2 a goal ahead in the lane.

Inputs (all from warrior_lane_detection, flat clouds in base_footprint that
share the camera image stamp):
  paint_topic  lane paint cells           (default /perception/lane_points)
  free_topic   visible ground cells       (default /perception/lane_free_points)
plus the global costmap (obstacles / drums) and TF map <- base_footprint.

Every plan_period the node ranks candidate goals ahead of the robot with
``CorridorPlanner`` and sends the best one to Nav2's navigate_to_pose. A new
goal is sent when there is none, when the robot is within resend_distance of
the active goal, when newer observations invalidate it (paint across its
path, or an obstacle on it), or when a clearly farther goal has appeared.
Nav2 plans the actual path and avoids obstacles; the LaneLayer keeps that
path off the lines.
"""

import math

from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, ColorRGBA, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from warrior_navigation.lane_corridor import (
    CorridorParams, CorridorPlanner, LaneMemory, Trail, wrap)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def to_global(cloud, transform):
    """(N, 2) x, y of a flat cloud transformed into the global frame."""
    if cloud.width * cloud.height == 0 or not cloud.data:
        return np.empty((0, 2))
    pts = point_cloud2.read_points(cloud, field_names=('x', 'y'))
    xy = np.column_stack([pts['x'].reshape(-1), pts['y'].reshape(-1)]).astype(float)
    t, q = transform.translation, transform.rotation
    yaw = yaw_of(q)  # both frames are 2-D for lane purposes
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([t.x + c * xy[:, 0] - s * xy[:, 1], t.y + s * xy[:, 0] + c * xy[:, 1]])


def steer_heading(heading, goal_yaw, smoothing, max_step):
    """Move the travel heading towards a sent goal, at most max_step (rad).

    Without the cap, goals picked at the edge of the +-max_turn fan while the
    robot dithers in front of an obstacle walked the heading round by up to
    35 deg per goal until lane following drove back the way it came.
    """
    step = smoothing * wrap(goal_yaw - heading)
    return wrap(heading + max(-max_step, min(max_step, step)))


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        defaults = dict(
            global_frame='map', base_frame='base_footprint',
            paint_topic='/perception/lane_points', free_topic='/perception/lane_free_points',
            costmap_topic='/global_costmap/costmap_raw', action_name='navigate_to_pose',
            free_resolution=0.2, plan_period=1.0, resend_distance=2.5,
            improve_margin=1.5, min_resend_interval=2.0, heading_smoothing=0.5,
            max_heading_step=20.0,
            pair_timeout=0.5, enabled=True)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name, value in vars(CorridorParams()).items():
            self.declare_parameter('corridor.' + name, value)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        self.planner = CorridorPlanner(CorridorParams(**{
            k: type(v)(self.get_parameter('corridor.' + k).value)
            for k, v in vars(CorridorParams()).items()}))
        self.memory = LaneMemory()
        self.trail = Trail()
        self.free_half = self.cfg['free_resolution'] / math.sqrt(2)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pending = {}   # stamp_ns -> {'paint': msg, 'free': msg, 'received': t}
        self.costmap = None
        self.heading = None
        self.active = None        # Candidate currently sent to Nav2
        self.goal_seq = 0
        self.last_send = None

        self.create_subscription(PointCloud2, self.cfg['paint_topic'],
                                 lambda m: self.store(m, 'paint'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cfg['free_topic'],
                                 lambda m: self.store(m, 'free'), qos_profile_sensor_data)
        self.create_subscription(Costmap, self.cfg['costmap_topic'], self.on_costmap,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.nav = ActionClient(self, NavigateToPose, self.cfg['action_name'])
        # Mission hand-over: ~/enable (Bool) starts/stops goal sending;
        # ~/heading_hint (Float64, rad in the global frame) re-seeds the
        # travel heading, e.g. the no-man's-land exit direction.
        self.enabled = bool(self.cfg['enabled'])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '~/enable', self.on_enable, latched)
        self.create_subscription(Float64, '~/heading_hint', self.on_heading_hint, latched)
        self.lane_lost_since = None
        self.markers = self.create_publisher(MarkerArray, '~/markers', 1)
        self.status = self.create_publisher(String, '~/status', 1)
        self.create_timer(0.1, self.integrate)
        self.create_timer(self.cfg['plan_period'], self.plan)
        self.get_logger().info('lane_follower ready; waiting for lane clouds and Nav2')

    # ---------------------------------------------------------------- inputs
    def store(self, msg, kind):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        entry = self.pending.setdefault(stamp, {'received': self.get_clock().now()})
        entry[kind] = msg

    def integrate(self):
        """Apply paired clouds in stamp order (clear, then mark)."""
        now = self.get_clock().now()
        for stamp in sorted(self.pending):
            entry = self.pending[stamp]
            age = (now - entry['received']).nanoseconds * 1e-9
            if not ('paint' in entry and 'free' in entry) and age < self.cfg['pair_timeout']:
                continue
            ref = entry.get('free') or entry.get('paint')
            try:
                t = self.tf.lookup_transform(self.cfg['global_frame'], ref.header.frame_id,
                                             Time.from_msg(ref.header.stamp))
            except TransformException:
                if age < 2.0:
                    continue          # TF not there yet; retry next tick
                del self.pending[stamp]
                continue
            free = to_global(entry['free'], t.transform) if 'free' in entry else np.empty((0, 2))
            paint = to_global(entry['paint'], t.transform) if 'paint' in entry else np.empty((0, 2))
            self.memory.update(free, paint, self.free_half)
            del self.pending[stamp]

    def on_costmap(self, msg):
        self.costmap = msg

    def on_enable(self, msg):
        if msg.data != self.enabled:
            self.get_logger().info(f'lane following {"enabled" if msg.data else "disabled"}')
        self.enabled = msg.data
        if not msg.data:
            self.active = None     # the mission owns Nav2 goals now
            self.goal_seq += 1     # ignore results of our earlier goals

    def on_heading_hint(self, msg):
        self.heading = wrap(msg.data)
        self.active = None
        self.get_logger().info(f'travel heading set to {math.degrees(self.heading):.0f} deg')

    def cost_at(self, x, y):
        m = self.costmap
        if m is None:
            return 0
        md = m.metadata
        i = int(math.floor((x - md.origin.position.x) / md.resolution))
        j = int(math.floor((y - md.origin.position.y) / md.resolution))
        if not (0 <= i < md.size_x and 0 <= j < md.size_y):
            return None
        cost = m.data[j * md.size_x + i]
        return 0 if cost == 255 else cost   # unknown is not an obstacle

    # --------------------------------------------------------------- planning
    def robot_pose(self):
        try:
            t = self.tf.lookup_transform(self.cfg['global_frame'], self.cfg['base_frame'], Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def plan(self):
        pose = self.robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        if self.heading is None:
            self.heading = yaw
        self.trail.add(x, y)
        best = self.planner.choose(self.memory, x, y, self.heading, self.cost_at, self.trail)
        self.publish_markers(x, y, best)
        now = self.get_clock().now()
        if best is None:
            self.lane_lost_since = self.lane_lost_since or now
        else:
            self.lane_lost_since = None
        lost_for = 0.0 if self.lane_lost_since is None else \
            (now - self.lane_lost_since).nanoseconds * 1e-9
        reason = self.resend_reason(x, y, best) if self.enabled else None
        self.status.publish(String(data=(
            f'enabled={int(self.enabled)} lane_lost={lost_for:.1f}s '
            f'heading={math.degrees(self.heading):.0f}deg '
            f'best={"none" if best is None else f"({best.x:.1f},{best.y:.1f})"} '
            f'active={"none" if self.active is None else f"({self.active.x:.1f},{self.active.y:.1f})"} '
            f'paint_cells={len(self.memory.paint)} seen_cells={len(self.memory.seen)} '
            f'action={reason or "keep"}')))
        if best is None:
            if self.enabled:
                self.get_logger().warning(
                    'No lane-supported goal ahead (lane ended or not seen); not driving on',
                    throttle_duration_sec=5.0)
            return
        if reason:
            self.send(best)

    def resend_reason(self, x, y, best):
        if best is None:
            return None
        if self.active is None:
            return 'no_goal'
        if math.hypot(self.active.x - x, self.active.y - y) < self.cfg['resend_distance']:
            return 'near_goal'
        if not self.planner.still_valid(self.memory, x, y, self.active, self.cost_at,
                                        self.trail):
            return 'goal_invalid'
        since = (self.get_clock().now() - self.last_send).nanoseconds * 1e-9
        ahead_new = (best.x - x) * math.cos(self.heading) + (best.y - y) * math.sin(self.heading)
        ahead_old = ((self.active.x - x) * math.cos(self.heading)
                     + (self.active.y - y) * math.sin(self.heading))
        if since >= self.cfg['min_resend_interval'] and ahead_new > ahead_old + self.cfg['improve_margin']:
            return 'better_goal'
        return None

    def send(self, goal):
        if not self.nav.server_is_ready():
            self.get_logger().info('Waiting for Nav2 navigate_to_pose', throttle_duration_sec=5.0)
            return
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self.cfg['global_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal.x, goal.y
        msg.pose.pose.orientation.z = math.sin(goal.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(goal.yaw / 2)
        self.goal_seq += 1
        seq = self.goal_seq
        self.active = goal
        self.last_send = self.get_clock().now()
        # Steer the travel heading towards where the lane actually goes.
        self.heading = steer_heading(self.heading, goal.yaw, self.cfg['heading_smoothing'],
                                     math.radians(self.cfg['max_heading_step']))
        future = self.nav.send_goal_async(msg)
        future.add_done_callback(lambda f: self.on_accepted(f, seq))

    def on_accepted(self, future, seq):
        handle = future.result()
        if handle is None or not handle.accepted:
            if seq == self.goal_seq:
                self.active = None
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, future, seq):
        if seq == self.goal_seq:   # results of preempted goals are stale
            self.active = None

    # ---------------------------------------------------------- visualisation
    def publish_markers(self, x, y, best):
        frame = self.cfg['global_frame']
        cands = Marker(type=Marker.POINTS, action=Marker.ADD, ns='candidates', id=0)
        cands.header.frame_id = frame
        cands.scale.x = cands.scale.y = 0.12
        ranked = self.planner.evaluate(self.memory, x, y, self.heading, self.cost_at,
                                       self.trail)
        if ranked:
            hi, lo = ranked[0].score, ranked[-1].score
            for c in ranked:
                k = 0.0 if hi == lo else (c.score - lo) / (hi - lo)
                cands.points.append(Point(x=c.x, y=c.y, z=0.05))
                cands.colors.append(ColorRGBA(r=1.0 - k, g=k, b=0.2, a=0.9))
        goal = Marker(type=Marker.ARROW, ns='goal', id=1)
        goal.header.frame_id = frame
        goal.action = Marker.ADD if best else Marker.DELETE
        if best:
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = best.x, best.y, 0.1
            goal.pose.orientation.z = math.sin(best.yaw / 2)
            goal.pose.orientation.w = math.cos(best.yaw / 2)
            goal.scale.x, goal.scale.y, goal.scale.z = 0.8, 0.15, 0.15
            goal.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=1.0)
        self.markers.publish(MarkerArray(markers=[cands, goal]))


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
