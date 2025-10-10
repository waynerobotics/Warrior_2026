import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from warrior_drive.arduino_interface import ArduinoInterface



class ArduinoDrive(Node):
    def __init__(self):
        super().__init__('arduino_drive')

        timer_value = self.declare_parameter('timer_value', 0.5).value

        self.arduino = ArduinoInterface()
        self.left_cmd_sub = self.create_subscription(Twist, 'left_cmd_vel', self.left_cmd_vel_callback, 10)
        self.right_cmd_sub = self.create_subscription(Twist, 'right_cmd_vel', self.right_cmd_vel_callback, 10)

        self.timer = self.create_timer(timer_value, self.timer_callback)
        self.left_velocity = 0.0
        self.right_velocity = 0.0

        self.arduino.write_line('ArduinoDriveInit')
        self.get_logger().info('Arduino Drive Node has been started.')

    def left_cmd_vel_callback(self, msg:Twist):
        self.left_velocity = msg.linear.x

    def right_cmd_vel_callback(self, msg:Twist):
        self.right_velocity = msg.linear.x


    def timer_callback(self):
        self.arduino.write_line(f'L{self.left_velocity:.3f}')
        self.arduino.write_line(f'R{self.right_velocity:.3f}')
        self.get_logger().info(f'Sent to Arduino - L:{self.left_velocity:.2f} R:{self.right_velocity:.2f}')


    def destroy_node(self):
        self.arduino.write_line('L0.000')
        self.arduino.write_line('R0.000')
        self.get_logger().info('Stopped Arduino motors before shutdown.')
        return super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)

    arduino_drive = ArduinoDrive()
    rclpy.spin(arduino_drive)

    arduino_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()