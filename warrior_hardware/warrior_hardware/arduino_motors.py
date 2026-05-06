from functools import partial

import rclpy
from rclpy.node import Node
from serial import SerialException
from std_msgs.msg import String

from warrior_hardware.arduino_interface import ArduinoInterface


class ArduinoMotors(Node):
    def __init__(self):
        super().__init__('arduino_motors')

        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('poll_period', 0.01)
        # change to actual usb ports from the hub
        self.declare_parameter('motor_1_port', '/dev/ttyACM0')
        self.declare_parameter('motor_2_port', '/dev/ttyACM1')
        self.declare_parameter('motor_3_port', '/dev/ttyACM2')
        self.declare_parameter('motor_4_port', '/dev/ttyACM3')

        self.baudrate = int(self.get_parameter('baudrate').value)
        poll_period = float(self.get_parameter('poll_period').value)

        self.motors = {}
        self.command_subscriptions = []
        self._create_motor_channel('motor_1')
        self._create_motor_channel('motor_2')
        self._create_motor_channel('motor_3')
        self._create_motor_channel('motor_4')

        self.poll_timer = self.create_timer(poll_period, self.poll_serial_ports)
        self.get_logger().info('Arduino motors node started.')

    def _create_motor_channel(self, motor_name):
        port = str(self.get_parameter(f'{motor_name}_port').value)
        feedback_topic = f'{motor_name}/feedback'
        command_topic = f'{motor_name}/command'
        publisher = self.create_publisher(String, feedback_topic, 10)

        try:
            interface = ArduinoInterface(port=port, baudrate=self.baudrate)
        except SerialException as exc:
            self.get_logger().error(f'Failed to open {motor_name} on {port}: {exc}')
            interface = None
        else:
            self.get_logger().info(f'{motor_name} mapped to {port}')

        subscription = self.create_subscription(
            String,
            command_topic,
            partial(self.command_callback, motor_name),
            10,
        )
        self.command_subscriptions.append(subscription)

        self.motors[motor_name] = {
            'port': port,
            'interface': interface,
            'publisher': publisher,
        }

    def command_callback(self, motor_name, msg):
        motor = self.motors[motor_name]
        interface = motor['interface']

        if interface is None:
            self.get_logger().warn(
                f'Ignoring command for {motor_name}; serial port {motor["port"]} is unavailable.'
            )
            return

        try:
            interface.write_line(msg.data)
        except SerialException as exc:
            self.get_logger().error(f'Failed to write to {motor_name} on {motor["port"]}: {exc}')

    def poll_serial_ports(self):
        for motor_name, motor in self.motors.items():
            interface = motor['interface']
            if interface is None:
                continue

            try:
                while True:
                    line = interface.read_line()
                    if line is None:
                        break

                    msg = String()
                    msg.data = line
                    motor['publisher'].publish(msg)
            except SerialException as exc:
                self.get_logger().error(
                    f'Failed to read from {motor_name} on {motor["port"]}: {exc}'
                )

    def destroy_node(self):
        for motor in self.motors.values():
            interface = motor['interface']
            if interface is None:
                continue

            try:
                interface.close()
            except SerialException as exc:
                self.get_logger().warn(f'Error while closing {motor["port"]}: {exc}')

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMotors()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
