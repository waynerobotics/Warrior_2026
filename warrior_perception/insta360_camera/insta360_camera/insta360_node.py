"""USB / UVC driver node for the Insta360 X4 / X5.

When the camera is connected by USB-C with webcam mode enabled, it enumerates
as a standard UVC device (``/dev/videoN``). This node opens it with OpenCV,
grabs frames, and republishes them as ``sensor_msgs/Image``.
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Insta360Node(Node):
    def __init__(self) -> None:
        super().__init__('insta360_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('frame_id', 'insta360_optical_frame')
        self.declare_parameter('width', 2880)
        self.declare_parameter('height', 1440)
        self.declare_parameter('fps', 30.0)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'failed to open {self.device}')
            raise RuntimeError(f'cannot open {self.device}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f'insta360_node streaming {self.device} '
            f'@ {self.width}x{self.height}/{self.fps:.0f}fps'
        )

    def _tick(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('frame grab failed')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        if self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Insta360Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
