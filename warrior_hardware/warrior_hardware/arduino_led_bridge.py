import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

# To be used as a test of ROS2 serial communication with Micro_2026 "Arduino_LED_Blink"
class ArduinoLED(Node):
    def __init__(self):
        super().__init__('arduino_led_bridge')

        # Set up subscriber: listens for HIGH/LOW messages
        self.subscription = self.create_subscription(
            String,
            'led_command',
            self.listener_callback,
            10
        )

        # Serial setup
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Background thread for Arduino → ROS feedback
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.get_logger().info('Arduino LED bridge node started.')

    def listener_callback(self, msg):
        """Triggered whenever a new ROS2 message arrives on /led_command."""
        command = msg.data.strip().upper()
        if command not in ['HIGH', 'LOW']:
            self.get_logger().warn(f'Invalid command: {command}')
            return

        self.serial_port.write((command + '\n').encode())
        self.get_logger().info(f'Sent to Arduino: {command}')

    def read_serial(self):
        """Continuously reads Arduino serial responses."""
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(f'Arduino says: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoLED()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
