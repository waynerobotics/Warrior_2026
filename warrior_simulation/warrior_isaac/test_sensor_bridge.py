"""Run with ``pixi run python -m unittest discover -s warrior_simulation/warrior_isaac``.

These exercise the real ROS messages and bridge initialization without requiring
an Isaac GPU. ROS/DDS and the bridge's localhost socket need normal local access.
"""
import importlib.util
import json
import math
from pathlib import Path
import statistics
import socket
import tempfile
import unittest

from run_robot import fixed_mount, rotate_vector

ROOT = Path(__file__).resolve().parents[2]


class FixedMountTest(unittest.TestCase):
    def test_composes_translation_through_rotated_parent(self):
        xml = '''<robot name="test">
          <joint name="body" type="fixed"><parent link="base_footprint"/>
            <child link="base_link"/><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/></joint>
          <joint name="gnss" type="fixed"><parent link="base_link"/>
            <child link="gnss_antenna_frame"/><origin xyz="0.4 0 0"/></joint>
        </robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.urdf'
            path.write_text(xml)
            position, quaternion = fixed_mount(path, 'gnss_antenna_frame')
        for actual, expected in zip(position, [1, 2.4, 3]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(rotate_vector(quaternion, [1, 0, 0]), [0, 1, 0]):
            self.assertAlmostEqual(actual, expected)


try:
    import rclpy
    from rclpy.node import Node
    spec = importlib.util.spec_from_file_location('isaac_ros_bridge', ROOT / 'warrior_bringup/scripts/isaac_ros_bridge.py')
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)
except ImportError:
    bridge = None


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@unittest.skipIf(bridge is None, 'Run in the Pixi ROS environment')
class BridgeTest(unittest.TestCase):
    def make_bridge(self, wheels):
        rclpy.init(args=['--ros-args', '-p', f'scene_file:={ROOT / "warrior_simulation/igvc2027/scene.json"}',
                        '-p', f'sensor_profile:={ROOT / "warrior_description/config/mti680g.yaml"}',
                        '-p', 'port:=0', '-p', f'use_wheel_odometry:={str(wheels).lower()}'])
        self.addCleanup(rclpy.shutdown)
        node = bridge.IsaacBridge()
        self.addCleanup(Node.destroy_node, node)
        self.addCleanup(node.socket.close)
        # Retain actual publishers for interface inspection, capture message
        # outputs without waiting for DDS discovery or a separate subscriber.
        self.assertEqual(node.publishers_by_key['gnss_velocity'].topic_name, '/gnss/velocity')
        node.publishers_by_key = {key: CapturePublisher() for key in node.publishers_by_key}
        return node

    @staticmethod
    def packet(wheels=False):
        packet = {'version': 2, 'stamp': 3.0, 'joint_names': [], 'joint_position': [], 'joint_velocity': [],
                  'position': [0., 0., 0.], 'orientation': [1., 0., 0., 0.],
                  'imu_orientation': [1., 0., 0., 0.], 'imu_angular_velocity': [0., 0., 0.],
                  'imu_linear_acceleration': [0., 0., 9.80665], 'gnss_position': [1., 2., 3.],
                  'gnss_velocity': [1., -.5, .2], 'scan': None}
        if wheels:
            packet.update(odom=[.2, .3, .4], twist=[.6, .7, .8])
        return packet

    def test_no_wheel_mode_uses_gnss_without_odometry_packet_fields(self):
        node = self.make_bridge(False)
        self.assertNotIn('odom', node.publishers_by_key)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(json.dumps(self.packet()).encode(), node.socket.getsockname())
        node.receive()
        velocity = node.publishers_by_key['gnss_velocity'].messages[-1]
        self.assertEqual(velocity.header.frame_id, 'gnss_enu')
        self.assertAlmostEqual(velocity.twist.covariance[0], .05 ** 2)
        gps = node.publishers_by_key['gps'].messages[-1]
        self.assertEqual(gps.header.frame_id, 'gnss_antenna_frame')
        self.assertAlmostEqual(gps.position_covariance[0], .011 ** 2 / (2 * math.log(2)))
        self.assertAlmostEqual(gps.position_covariance[8], .02 ** 2)
        imu = node.publishers_by_key['imu'].messages[-1]
        self.assertEqual(imu.header.frame_id, 'imu_frame')
        self.assertAlmostEqual(imu.orientation_covariance[0], math.radians(.2) ** 2)
        self.assertAlmostEqual(imu.orientation_covariance[8], math.radians(.5) ** 2)
        self.assertAlmostEqual(imu.angular_velocity_covariance[0], math.radians(.007) ** 2 * 30)
        self.assertAlmostEqual(imu.linear_acceleration_covariance[0], (60e-6 * 9.80665) ** 2 * 30)

    def test_wheel_mode_preserves_encoder_twist(self):
        node = self.make_bridge(True)
        node.publish(self.packet(True), 3.0)
        odom = node.publishers_by_key['odom'].messages[-1]
        self.assertEqual(odom.twist.twist.linear.x, .6)
        self.assertEqual(odom.twist.twist.linear.y, .7)
        self.assertEqual(odom.twist.twist.angular.z, .8)

    def test_noise_matches_declared_measurement_covariance(self):
        node = self.make_bridge(False)
        packet = self.packet()
        for index in range(1200):
            node.publish(packet, 3.0 + index / 10)
        velocities = [m.twist.twist.linear.x - 1.0 for m in node.publishers_by_key['gnss_velocity'].messages]
        yaws = [2 * math.atan2(m.orientation.z, m.orientation.w) for m in node.publishers_by_key['imu'].messages]
        self.assertLess(abs(statistics.mean(velocities)), .006)
        self.assertAlmostEqual(statistics.stdev(velocities), .05, delta=.005)
        self.assertAlmostEqual(statistics.stdev(yaws), math.radians(.5), delta=math.radians(.05))


if __name__ == '__main__':
    unittest.main()
