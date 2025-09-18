
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

"""
A Node to convert the left and right joystick Y-axis of a gamepad to left and right wheel velocities.
"""


class Joy2Stick(Node):
    def __init__(self):
        super().__init__('joy_2stick')
        self.left_cmd_pub = self.create_publisher(Twist, 'left_wheel/cmd_vel', 10)
        self.right_cmd_pub = self.create_publisher(Twist, 'right_wheel/cmd_vel', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info("joy_2stick node has been started.")

    def joy_callback(self, msg:Joy):
        left_cmd = Twist()
        right_cmd = Twist()

        #For joy package msg 0 LEFTX 1 LEFTY 2 RIGHTX 3 RIGHTY 4 TRIGGERLEFT 5 TRIGGERRIGHT 
        left_cmd.linear.x = msg.axes[1] #Left Joystick Y Movement - > Left wheel vel
        right_cmd.linear.x = msg.axes[4] #Right Joystick Y Movement - > Right wheel vel

        self.left_cmd_pub.publish(left_cmd)
        self.right_cmd_pub.publish(right_cmd)
        self.get_logger().info(f"Left Stick Y: {left_cmd.linear.x}, Right Stick Y: {right_cmd.linear.x}")

def main(args=None):
    rclpy.init(args=args)
    joy_2stick = Joy2Stick()
    rclpy.spin(joy_2stick)
    joy_2stick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


