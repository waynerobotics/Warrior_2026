import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DiffDriveSim(Node):
    def __init__(self):
        super().__init__('diff_drive_sim')

        self.wheel_base = 0.5   # distance between left & right wheels (m)
        self.caster_offset = 0.2  # distance from center to caster (m)
        self.wheel_radius = 0.05

        self.left_vel = 0.0
        self.right_vel = 0.0

        self.sub_left = self.create_subscription(Twist, 'left_cmd_vel', self.left_cb, 10)
        self.sub_right = self.create_subscription(Twist, 'right_cmd_vel', self.right_cb, 10)

        self.pub_left = self.create_publisher(Float64, 'left_cmd_vel_x', 10)
        self.pub_right = self.create_publisher(Float64, 'right_cmd_vel_x', 10)
        # self.pub_caster = self.create_publisher(Float64, 'caster_vel', 10)

    def compute_and_publish_caster(self):
        v = (self.right_vel + self.left_vel) / 2.0
        omega = (self.right_vel - self.left_vel) / self.wheel_base

        caster_v = v + omega * self.caster_offset

        msg = Float64()
        msg.data = caster_v
        self.pub_caster.publish(msg)

    def left_cb(self, msg):
        self.left_vel = msg.linear.x
        cmd = Float64()
        cmd.data = self.left_vel * 10
        self.pub_left.publish(cmd)
        # self.compute_and_publish_caster()

    def right_cb(self, msg):
        self.right_vel = msg.linear.x
        cmd = Float64()
        cmd.data = self.right_vel * 10
        self.pub_right.publish(cmd)
        # self.compute_and_publish_caster()
        # cmd = Float64()
        # cmd.data = msg.linear.x * 10
        # self.pub_right.publish(cmd)

def main():
    rclpy.init()
    node = DiffDriveSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
