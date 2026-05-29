"""YOLO inference node (stub).

Subscribes to a camera image stream and publishes:
  * ``ai/segmentation_mask`` — ``sensor_msgs/Image`` (mono8 bitmask)
  * ``ai/detections``        — ``vision_msgs/Detection2DArray``
                              (label + score + bbox per detection)

The model is intentionally NOT loaded here yet; this is a scaffold so the
topic graph can be wired up while the weights / inference backend are still
being chosen (ultralytics, TensorRT, ONNX Runtime, ...).
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_node')

        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('mask_topic', 'ai/segmentation_mask')
        self.declare_parameter('detections_topic', 'ai/detections')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.25)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        mask_topic = self.get_parameter('mask_topic').get_parameter_value().string_value
        det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter(
            'confidence_threshold').get_parameter_value().double_value

        self.sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(
            Image, mask_topic, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(
            Detection2DArray, det_topic, 10)

        self.model: Optional[object] = None  # TODO: load weights here
        if self.model_path:
            self.get_logger().warn(
                f'model_path set to "{self.model_path}" but loader is a TODO')
        else:
            self.get_logger().warn(
                'no model_path provided; node will pass through without inference')

        self.get_logger().info(
            f'yolo_node ready (sub={image_topic}, '
            f'mask={mask_topic}, det={det_topic}, thresh={self.conf_thresh:.2f})')

    def _on_image(self, msg: Image) -> None:
        # TODO: cv_bridge -> numpy -> model.infer(...) -> mask + detections
        del msg


def main() -> None:
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
