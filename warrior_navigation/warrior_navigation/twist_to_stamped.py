import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.sub = self.create_subscription(Twist, '/cmd_vel_nav', self.cb, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def cb(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_footprint'
        stamped.twist = msg
        self.pub.publish(stamped)

def main():
    rclpy.init()
    rclpy.spin(TwistToStamped())