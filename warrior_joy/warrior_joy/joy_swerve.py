
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

"""
Node to convert one joystick to direction and another to magnitude for cmd_vel (swerve drive).
"""


class JoySwerve(Node):
    def __init__(self):
        super().__init__('joy_swerve')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info("joy_swerve node has been started.")

    def joy_callback(self, msg:Joy):
        cmd_msg = Twist()

        #For joy package msg 0 LEFTX 1 LEFTY 2 RIGHTX 3 RIGHTY 4 TRIGGERLEFT 5 TRIGGERRIGHT 
        cmd_msg.linear.x = msg.axes[1] 
        # cmd_msg.linear.y = msg.axes[1] 

        cmd_msg.angular.z = msg.axes[4]

        self.cmd_pub.publish(cmd_msg)
        # self.get_logger().info(f"Published cmd_vel: linear_x={cmd_msg.linear.x}, linear_y={cmd_msg.linear.y}, angular_z={cmd_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    joy_swerve = JoySwerve()
    rclpy.spin(joy_swerve)
    JoySwerve.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


