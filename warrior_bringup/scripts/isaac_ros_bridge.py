#!/usr/bin/env python3
"""Translate local Isaac sensor packets into ROS without mixing Python ABIs.

Isaac 5.1 embeds Python 3.11; this repository's ROS environment uses 3.12.
Only localhost UDP is used. Wheel odometry is measured from joint encoders;
ideal sensor kinematics are perturbed before publishing MTi-680G measurements.
Physical pose remains on a separate diagnostic topic.
"""
import json
import math
from pathlib import Path
import random
import socket
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from pyproj import Transformer
from pyproj.enums import TransformDirection
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan, NavSatFix, NavSatStatus
import yaml


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


def perturb_orientation(values, noise):
    """Apply small body-axis roll/pitch/yaw errors to a scalar-first quaternion."""
    cr, cp, cy = [math.cos(angle / 2) for angle in noise]
    sr, sp, sy = [math.sin(angle / 2) for angle in noise]
    a, b, c, d = (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                  cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)
    w, x, y, z = values
    return [w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
            w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a]


class IsaacBridge(Node):
    def __init__(self):
        super().__init__('isaac_ros_bridge')
        self.declare_parameter('port', 17427)
        self.declare_parameter('scene_file', '')
        self.declare_parameter('use_wheel_odometry', True)
        self.declare_parameter('sensor_profile', str(Path(get_package_share_directory('warrior_description')) /
                                                    'config/mti680g.yaml'))
        profile = yaml.safe_load(Path(self.get_parameter('sensor_profile').value).read_text())['mti680g_sim']['ros__parameters']
        self.imu_frame = profile['imu_frame']
        self.antenna_frame = profile['antenna_frame']
        self.velocity_frame = profile['velocity_frame']
        # Horizontal CEP is not per-axis Gaussian sigma; apply the profile's
        # assumed RTK baseline. Vertical accuracy is a simulation assumption.
        cep = profile['rtk_position_cep_m'] + profile['rtk_baseline_m'] * profile['rtk_baseline_error_ppm'] * 1e-6
        self.declare_parameter('gps_noise_stddev', -1.0)
        self.declare_parameter('gnss_horizontal_stddev', cep / math.sqrt(2 * math.log(2)))
        self.declare_parameter('gnss_vertical_stddev', float(profile['vertical_position_stddev_m']))
        self.declare_parameter('gnss_velocity_stddev', float(profile['velocity_stddev_m_s']))
        self.declare_parameter('imu_roll_pitch_stddev_degrees', float(profile['roll_pitch_stddev_deg']))
        self.declare_parameter('imu_yaw_stddev_degrees', float(profile['yaw_stddev_deg']))
        self.declare_parameter('random_seed', int(profile['noise_seed']))
        scene = json.loads(Path(self.get_parameter('scene_file').value).read_text())
        datum = scene['datum']
        self.geodesy = Transformer.from_pipeline(
            '+proj=pipeline +step +proj=cart +ellps=WGS84 '
            f'+step +proj=topocentric +ellps=WGS84 +lat_0={datum["latitude"]} '
            f'+lon_0={datum["longitude"]} +h_0={datum["elevation"]}')
        self.port = self.get_parameter('port').value
        self.use_wheel_odometry = self.get_parameter('use_wheel_odometry').value
        legacy_noise = self.get_parameter('gps_noise_stddev').value
        horizontal = self.get_parameter('gnss_horizontal_stddev').value
        vertical = self.get_parameter('gnss_vertical_stddev').value
        self.position_sigma = ([legacy_noise] * 3 if legacy_noise >= 0 else [horizontal, horizontal, vertical])
        self.velocity_sigma = self.get_parameter('gnss_velocity_stddev').value
        self.orientation_sigma = [math.radians(self.get_parameter('imu_roll_pitch_stddev_degrees').value)] * 2
        self.orientation_sigma.append(math.radians(self.get_parameter('imu_yaw_stddev_degrees').value))
        # Native Isaac telemetry is 60 Hz. Convert white-noise density using its
        # Nyquist bandwidth; this does not reproduce proprietary Xsens filtering.
        self.gyro_sigma = math.radians(profile['gyro_noise_density_deg_s']) * math.sqrt(60.0 / 2)
        self.accel_sigma = profile['accel_noise_density_micro_g'] * 1e-6 * 9.80665 * math.sqrt(60.0 / 2)
        sigmas = self.position_sigma + self.orientation_sigma + [self.velocity_sigma, self.gyro_sigma, self.accel_sigma]
        if not all(math.isfinite(value) and value >= 0 for value in sigmas):
            raise ValueError('Sensor standard deviations must be finite and nonnegative')
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
            'imu': self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data),
            'gps': self.create_publisher(NavSatFix, '/gnss/fix', qos_profile_sensor_data),
            'gnss_velocity': self.create_publisher(TwistWithCovarianceStamped, '/gnss/velocity', qos_profile_sensor_data),
            'scan': self.create_publisher(LaserScan, '/navigation/scan', qos_profile_sensor_data),
            'truth': self.create_publisher(Odometry, '/isaac/ground_truth', 10),
        }
        if self.use_wheel_odometry:
            self.publishers_by_key['odom'] = self.create_publisher(Odometry, '/odom', 10)
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
                if packet.get('version') != 2 or not math.isfinite(stamp) or stamp <= self.last_stamp:
                    continue
                fields = ['joint_position', 'joint_velocity', 'position',
                          'orientation', 'imu_orientation', 'imu_angular_velocity', 'imu_linear_acceleration']
                if self.use_wheel_odometry:
                    fields.extend(['odom', 'twist'])
                if packet['gnss_position'] is not None:
                    fields.extend(['gnss_position', 'gnss_velocity'])
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
        if self.use_wheel_odometry:
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
        imu.header.stamp, imu.header.frame_id = stamp, self.imu_frame
        quaternion(imu.orientation, perturb_orientation(packet['imu_orientation'],
                   [self.random.gauss(0, sigma) for sigma in self.orientation_sigma]))
        vector(imu.angular_velocity, [v+self.random.gauss(0, self.gyro_sigma) for v in packet['imu_angular_velocity']])
        vector(imu.linear_acceleration, [v+self.random.gauss(0, self.accel_sigma) for v in packet['imu_linear_acceleration']])
        imu.orientation_covariance = covariance(3, [sigma ** 2 for sigma in self.orientation_sigma])
        imu.angular_velocity_covariance = covariance(3, [self.gyro_sigma ** 2] * 3)
        imu.linear_acceleration_covariance = covariance(3, [self.accel_sigma ** 2] * 3)
        self.publishers_by_key['imu'].publish(imu)
        if packet['scan'] is not None:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_footprint'
            scan.angle_min, scan.angle_increment = -math.pi, math.pi / 180
            scan.angle_max = scan.angle_min + scan.angle_increment * 359
            scan.scan_time, scan.range_min, scan.range_max = .1, .9, 15.
            scan.ranges = [float(v) if v is not None else math.inf for v in packet['scan']]
            self.publishers_by_key['scan'].publish(scan)
        if packet['gnss_position'] is not None:
            # navsat's wall-clock startup delay may expire while Isaac boots.
            # Wait for the drop-to-ground transient and the first IMU samples.
            if seconds < 2.0:
                return
            enu = [p+self.random.gauss(0, sigma) for p, sigma in zip(packet['gnss_position'], self.position_sigma)]
            lon, lat, altitude = self.geodesy.transform(*enu, direction=TransformDirection.INVERSE)
            gps = NavSatFix()
            gps.header.stamp, gps.header.frame_id = stamp, self.antenna_frame
            # ROS NavSatStatus has no RTK-fixed enum; SBAS/GBAS must not be used
            # as a substitute for an RTK ambiguity status.
            gps.status.status = NavSatStatus.STATUS_FIX
            gps.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS |
                                  NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO)
            gps.latitude, gps.longitude, gps.altitude = lat, lon, altitude
            gps.position_covariance = covariance(3, [sigma ** 2 for sigma in self.position_sigma])
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.publishers_by_key['gps'].publish(gps)
            velocity = TwistWithCovarianceStamped()
            velocity.header.stamp, velocity.header.frame_id = stamp, self.velocity_frame
            vector(velocity.twist.twist.linear,
                   [v+self.random.gauss(0, self.velocity_sigma) for v in packet['gnss_velocity']])
            velocity.twist.covariance = covariance(6, [self.velocity_sigma ** 2] * 3 + [1e6] * 3)
            self.publishers_by_key['gnss_velocity'].publish(velocity)
            truth = Odometry()
            truth.header.stamp, truth.header.frame_id, truth.child_frame_id = stamp, 'isaac_world', 'base_footprint'
            vector(truth.pose.pose.position, packet['position'])
            quaternion(truth.pose.pose.orientation, packet['orientation'])
            self.publishers_by_key['truth'].publish(truth)
        if not self.ready:
            self.ready = True
            mode = 'wheel + GNSS/INS' if self.use_wheel_odometry else 'GNSS/INS without wheel odometry'
            self.get_logger().info(f'Isaac MTi-680G telemetry received; localization mode: {mode}')

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
