

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import String

class SwerveSim(Node):
    def __init__(self):
        super().__init__('SwerveSim')

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.wheel_pub = {
            'front_left': self.create_publisher(Float64, '/front_left/wheel_joint/cmd_vel', 10),
            'front_right': self.create_publisher(Float64, '/front_right/wheel_joint/cmd_vel', 10),
            'rear_left': self.create_publisher(Float64, '/rear_left/wheel_joint/cmd_vel', 10),
            'rear_right': self.create_publisher(Float64, '/rear_right/wheel_joint/cmd_vel', 10)
        }

        # Publishers for steering position commands
        self.steer_pub = {
            'front_left': self.create_publisher(Float64, '/front_left/steer_joint/cmd_vel', 10),
            'front_right': self.create_publisher(Float64, '/front_right/steer_joint/cmd_vel', 10),
            'rear_left': self.create_publisher(Float64, '/rear_left/steer_joint/cmd_vel', 10),
            'rear_right': self.create_publisher(Float64, '/rear_right/steer_joint/cmd_vel', 10)
        }

    def cmd_vel_callback(self, msg:Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        wheel_speed = linear_x * 10  # Make this into a parameter later
        # steer_angle = angular_z 
        steer_angle = angular_z # for now
        self.get_logger().info(f"Calculated wheel_speed={wheel_speed}")

        for wheel in self.wheel_pub:
            speed_msg = Float64()
            speed_msg.data = wheel_speed
            self.wheel_pub[wheel].publish(speed_msg)

            angle_msg = Float64()
            angle_msg.data = steer_angle
            self.steer_pub[wheel].publish(angle_msg)


def main(args=None):
    rclpy.init(args=args)

    swerve_sim = SwerveSim()
    rclpy.spin(swerve_sim)

    swerve_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()