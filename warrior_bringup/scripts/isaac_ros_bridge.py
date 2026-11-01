#!/usr/bin/env python3
"""Translate local Isaac sensor packets into ROS without mixing Python ABIs.

Isaac 5.1 embeds Python 3.11; this repository's ROS environment uses 3.12.
Only localhost UDP is used. Wheel odometry is measured from joint encoders;
physical pose is used solely for simulated GPS and a separate debug topic.
"""
import json
import math
from pathlib import Path
import random
import socket
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from pyproj import Transformer
from pyproj.enums import TransformDirection
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan, NavSatFix, NavSatStatus


def stamp_message(seconds):
    from builtin_interfaces.msg import Time
    nanoseconds = round(seconds * 1e9)
    return Time(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)


def quaternion(message, values):
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm < 1e-6:
        raise ValueError('sensor quaternion has zero norm')
    message.w, message.x, message.y, message.z = [float(value) / norm for value in values]


def vector(message, values):
    message.x, message.y, message.z = map(float, values)


def covariance(size, values):
    result = [0.0] * (size * size)
    for index, value in enumerate(values):
        result[index * (size + 1)] = float(value)
    return result


class IsaacBridge(Node):
    def __init__(self):
        super().__init__('isaac_ros_bridge')
        self.declare_parameter('port', 17427)
        self.declare_parameter('scene_file', '')
        self.declare_parameter('gps_noise_stddev', 0.03)
        self.declare_parameter('random_seed', 2027)
        scene = json.loads(Path(self.get_parameter('scene_file').value).read_text())
        datum = scene['datum']
        self.geodesy = Transformer.from_pipeline(
            '+proj=pipeline +step +proj=cart +ellps=WGS84 '
            f'+step +proj=topocentric +ellps=WGS84 +lat_0={datum["latitude"]} '
            f'+lon_0={datum["longitude"]} +h_0={datum["elevation"]}')
        self.port = self.get_parameter('port').value
        self.noise = self.get_parameter('gps_noise_stddev').value
        self.random = random.Random(self.get_parameter('random_seed').value)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', self.port))
        self.socket.setblocking(False)
        self.last_stamp = -1.0
        self.last_packet_wall = time.monotonic()
        self.ready = False
        self.publishers_by_key = {
            'clock': self.create_publisher(Clock, '/clock', 10),
            'joints': self.create_publisher(JointState, '/joint_states', 10),
            'odom': self.create_publisher(Odometry, '/odom', 10),
            'imu': self.create_publisher(Imu, '/mobile_sensor/imu', qos_profile_sensor_data),
            'gps': self.create_publisher(NavSatFix, '/gps1/fix', qos_profile_sensor_data),
            'scan': self.create_publisher(LaserScan, '/navigation/scan', qos_profile_sensor_data),
            'truth': self.create_publisher(Odometry, '/isaac/ground_truth', 10),
        }
        self.create_subscription(TwistStamped, '/cmd_vel_stamped', self.command, 10)
        self.create_timer(0.004, self.receive)
        self.create_timer(5.0, self.check_health)
        self.get_logger().info(f'Isaac adapter listening on localhost:{self.port}; commands on {self.port + 1}')

    def command(self, message):
        if not self.ready or time.monotonic() - self.last_packet_wall > 1.0:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if abs(stamp - self.last_stamp) > 1.0:
            return
        if message.header.frame_id not in ('', 'base_footprint', 'base_link'):
            return
        values = [message.twist.linear.x, message.twist.linear.y, message.twist.angular.z]
        if not all(math.isfinite(value) for value in values):
            return
        self.socket.sendto(json.dumps({'stamp': stamp, 'twist': values}).encode(),
                           ('127.0.0.1', self.port + 1))

    def check_health(self):
        age = time.monotonic() - self.last_packet_wall
        if age > 5:
            self.get_logger().warn(f'Waiting for Isaac sensor packets ({age:.0f} s)')

    def receive(self):
        # Drain packets in timestamp order. Do not replay an old clock after a
        # simulator restart or accept truncated/non-finite telemetry.
        for _ in range(60):
            try:
                payload, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                packet = json.loads(payload)
                stamp = float(packet['stamp'])
                if packet.get('version') != 1 or not math.isfinite(stamp) or stamp <= self.last_stamp:
                    continue
                fields = ('joint_position', 'joint_velocity', 'odom', 'twist', 'position',
                          'orientation', 'imu_orientation', 'imu_angular_velocity', 'imu_linear_acceleration')
                if not all(math.isfinite(float(v)) for key in fields for v in packet[key]):
                    continue
                self.publish(packet, stamp)
            except (ValueError, TypeError, KeyError, OverflowError) as error:
                self.get_logger().warning(f'Discarded malformed Isaac packet: {error}', throttle_duration_sec=5)

    def publish(self, packet, seconds):
        self.last_stamp = seconds
        self.last_packet_wall = time.monotonic()
        stamp = stamp_message(seconds)
        self.publishers_by_key['clock'].publish(Clock(clock=stamp))
        joints = JointState()
        joints.header.stamp, joints.header.frame_id = stamp, 'base_footprint'
        joints.name = packet['joint_names']
        joints.position = [float(v) for v in packet['joint_position']]
        joints.velocity = [float(v) for v in packet['joint_velocity']]
        self.publishers_by_key['joints'].publish(joints)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        x, y, yaw = packet['odom']
        vector(odom.pose.pose.position, [x, y, 0])
        quaternion(odom.pose.pose.orientation, [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        vector(odom.twist.twist.linear, [*packet['twist'][:2], 0])
        odom.twist.twist.angular.z = float(packet['twist'][2])
        odom.pose.covariance = covariance(6, [.03, .03, 100, 100, 100, .02])
        odom.twist.covariance = covariance(6, [.0025, .0025, 100, 100, 100, .0025])
        self.publishers_by_key['odom'].publish(odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_frame'
        quaternion(imu.orientation, packet['imu_orientation'])
        vector(imu.angular_velocity, packet['imu_angular_velocity'])
        vector(imu.linear_acceleration, packet['imu_linear_acceleration'])
        imu.orientation_covariance = covariance(3, [.0001] * 3)
        imu.angular_velocity_covariance = covariance(3, [.0001] * 3)
        imu.linear_acceleration_covariance = covariance(3, [.01] * 3)
        self.publishers_by_key['imu'].publish(imu)
        if packet['scan'] is not None:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
            scan.angle_min, scan.angle_increment = -math.pi, math.pi / 180
            scan.angle_max = scan.angle_min + scan.angle_increment * 359
            scan.scan_time, scan.range_min, scan.range_max = .1, .9, 15.
            scan.ranges = [float(v) if v is not None else math.inf for v in packet['scan']]
            self.publishers_by_key['scan'].publish(scan)
            # navsat's wall-clock startup delay may expire while Isaac boots.
            # Wait for the drop-to-ground transient and the first IMU samples.
            if seconds < 2.0:
                return
            # GPS receiver position includes the same antenna offset as its TF.
            w, qx, qy, qz = packet['orientation']
            ax, ay, az = .452037, -.002156, .640773
            tx, ty, tz = 2*(qy*az-qz*ay), 2*(qz*ax-qx*az), 2*(qx*ay-qy*ax)
            antenna = [ax + w*tx + qy*tz-qz*ty,
                       ay + w*ty + qz*tx-qx*tz,
                       az + w*tz + qx*ty-qy*tx]
            enu = [p + a + self.random.gauss(0, self.noise) for p, a in zip(packet['position'], antenna)]
            lon, lat, altitude = self.geodesy.transform(*enu, direction=TransformDirection.INVERSE)
            gps = NavSatFix()
            gps.header.stamp, gps.header.frame_id = stamp, 'gps1_frame'
            gps.status.status, gps.status.service = NavSatStatus.STATUS_FIX, NavSatStatus.SERVICE_GPS
            gps.latitude, gps.longitude, gps.altitude = lat, lon, altitude
            gps.position_covariance = covariance(3, [max(.0001, self.noise ** 2)] * 3)
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.publishers_by_key['gps'].publish(gps)
            truth = Odometry()
            truth.header.stamp, truth.header.frame_id, truth.child_frame_id = stamp, 'isaac_world', 'base_footprint'
            vector(truth.pose.pose.position, packet['position'])
            quaternion(truth.pose.pose.orientation, packet['orientation'])
            self.publishers_by_key['truth'].publish(truth)
        if not self.ready:
            self.ready = True
            self.get_logger().info('Isaac clock, encoders, IMU, GPS and lidar telemetry received')

    def destroy_node(self):
        try:
            self.socket.sendto(json.dumps({'stamp': self.last_stamp, 'twist': [0, 0, 0]}).encode(),
                               ('127.0.0.1', self.port + 1))
        finally:
            self.socket.close()
            super().destroy_node()


def main():
    rclpy.init()
    node = IsaacBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
