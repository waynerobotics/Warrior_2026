import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapToGrid(Node):
    def __init__(self):
        super().__init__('map_to_grid')
        self.get_logger().info('Map to Grid node started')

        self.map = None
        self.grid = None

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.map_callback,
            10
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    


    
def main(args=None):
    rclpy.init(args=args)
    node = MapToGrid()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()