"""ROS adapter for ENU GNSS/INS velocity and attitude measurements."""

from collections import deque

from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener

from warrior_localization.gnss_velocity import antenna_velocity_to_body, closest_imu


def _quaternion(value):
    return (value.x, value.y, value.z, value.w)


def _vector(value):
    return (value.x, value.y, value.z)


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class GnssVelocityNode(Node):
    """Publish base velocity without depending on a filter's dynamic TF."""

    def __init__(self):
        super().__init__('gnss_velocity_to_body')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('antenna_frame', 'gnss_antenna_frame')
        self.declare_parameter('enu_frame', 'gnss_enu')
        self.declare_parameter('max_imu_age', 0.2)
        self.declare_parameter('max_velocity_age', 0.5)
        self._base = self.get_parameter('base_frame').value
        self._antenna = self.get_parameter('antenna_frame').value
        self._enu = self.get_parameter('enu_frame').value
        self._imu_age = float(self.get_parameter('max_imu_age').value)
        self._velocity_age = float(self.get_parameter('max_velocity_age').value)
        if self._imu_age <= 0.0 or self._velocity_age <= 0.0:
            raise ValueError('Sensor age limits must be positive')
        self._samples = deque(maxlen=200)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._publisher = self.create_publisher(
            TwistWithCovarianceStamped, '/gnss/velocity_body', 10)
        self.create_subscription(Imu, '/imu/data', self._imu, qos_profile_sensor_data)
        self.create_subscription(TwistWithCovarianceStamped, '/gnss/velocity',
                                 self._velocity, qos_profile_sensor_data)

    def _imu(self, message):
        stamp = _stamp(message.header)
        # A simulator reset invalidates the previous run's attitudes.
        if self._samples and stamp < self._samples[-1][0] - 1.0:
            self._samples.clear()
        self._samples.append((stamp, message))

    def _velocity(self, message):
        stamp = _stamp(message.header)
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if message.header.frame_id != self._enu:
            self.get_logger().warning('GNSS velocity must use the configured ENU frame',
                                      throttle_duration_sec=5.0)
            return
        imu = closest_imu(self._samples, stamp, self._imu_age)
        if imu is None or age > self._velocity_age or age < -self._velocity_age:
            self.get_logger().warning('Dropping GNSS velocity without a fresh IMU and timestamp',
                                      throttle_duration_sec=5.0)
            return
        if imu.orientation_covariance[0] < 0 or imu.angular_velocity_covariance[0] < 0:
            self.get_logger().warning('GNSS velocity needs valid IMU attitude and angular rate',
                                      throttle_duration_sec=5.0)
            return
        try:
            # Only rigid robot mounts are queried. No map/odom TF is needed,
            # including at startup before either EKF has published anything.
            mount = self._tf.lookup_transform(self._base, imu.header.frame_id, Time())
            antenna = self._tf.lookup_transform(self._base, self._antenna, Time())
            input_covariance = message.twist.covariance
            linear_covariance = [input_covariance[row * 6 + col]
                                 for row in range(3) for col in range(3)]
            linear, angular, covariance = antenna_velocity_to_body(
                _vector(message.twist.twist.linear), linear_covariance,
                _quaternion(imu.orientation), imu.orientation_covariance,
                _vector(imu.angular_velocity), imu.angular_velocity_covariance,
                _quaternion(mount.transform.rotation), _vector(antenna.transform.translation))
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'Cannot transform GNSS velocity: {error}',
                                      throttle_duration_sec=5.0)
            return
        output = TwistWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._base
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = (
            float(value) for value in linear)
        (output.twist.twist.angular.x, output.twist.twist.angular.y,
         output.twist.twist.angular.z) = (float(value) for value in angular)
        output.twist.covariance = covariance.flatten().tolist()
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GnssVelocityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
