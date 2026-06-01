import sensor_msgs.msg as msg
from sensor_msgs_py import point_cloud2
import rclpy
from rclpy.node import Node
import numpy as np

class PointCloud2Parser(Node):
    def __init__(self):
        super().__init__('pc2_echo')
        self.subscription = self.create_subscription(
            msg.PointCloud2,
            'unilidar/cloud',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        pc2_data = point_cloud2.read_points_list(msg, field_names=["x", "y", "z"], skip_nans=False)
        arr = np.array(pc2_data)
        self.get_logger().info(pc2_data)

def main(args=None):
    rclpy.init(args=args)
    pc2_echo = PointCloud2Parser()
    rclpy.spin(pc2_echo)
    pc2_echo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
