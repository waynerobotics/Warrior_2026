import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from nav_msgs.msg import Path
import numpy as np
import heapq
import math

# Need to figure out how to turn a occupancy grid into a 2d array for A*
"""  OccupancyGrid message definition:
# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
MapMetaData info

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data
"""


class ComplexPathGenerator(Node):
    def __init__(self):
        super().__init__('complex_path_generator')
        self.get_logger().info('Complex Path Generator Node has been started.')

        self.robot_pose_sub = self.create_subscription(PoseStamped, 'robot_map_pose', self.robot_pose_callback, 10)
        self.goal_subscription = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/a_star_path', 10)

        self.cost_map_subscriber = self.create_subscription(OccupancyGrid,
                                'global_costmap/costmap', self.cost_map_callback, 10)

        self.costmap = None
        self.map_info = None
        self.robot_pose = None
        self.goal_pose = None

    def cost_map_callback(self, msg: OccupancyGrid):
        self.costmap = msg
        self.map_info = msg.info
        self.publish_complex_path()
        
    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg
        self.publish_complex_path()

    def publish_complex_path(self):
        if self.goal_pose is None:
            self.get_logger().info('Goal pose is None, cannot generate path.')
            return
        if self.robot_pose is None:
            self.get_logger().info('Robot pose is None, cannot generate path.')
            return
        # if self.costmap is None or self.map_info is None:
        #     return
        try:
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()

            grid = np.array(self.costmap.data).reshape((self.map_info.height, self.map_info.width))

            start = self.world_to_grid(self.robot_pose.pose.position.x ,self.robot_pose.pose.position.y)
            goal = self.world_to_grid( self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)

            grid_path = a_star(grid, start, goal, max_cost=90, alpha=5.0)

            poses = []
            for r, c in grid_path:
                x, y = self.grid_to_world(r, c)
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                poses.append(ps)

            path.poses = poses
            self.path_pub.publish(path)
            self.get_logger().info('Published complex path using A* algorithm.')
            
        except Exception as e:
            self.get_logger().warn(f'Path generation failed: {e}')


    def world_to_grid(self, x, y):
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return row, col

    def grid_to_world(self, row, col):
        x = col * self.map_info.resolution + self.map_info.origin.position.x
        y = row * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

def a_star(grid, start, goal, max_cost=90, alpha=5.0):
    height, width = grid.shape

    def heuristic(a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        r, c = current

        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r + dr, c + dc

            if not (0 <= nr < height and 0 <= nc < width):
                continue

            cell_cost = grid[nr, nc]

            # Block obstacles and unknown
            if cell_cost < 0 or cell_cost >= max_cost:
                continue

            neighbor = (nr, nc)

            # --- COST-AWARE STEP COST ---
            normalized_cost = cell_cost / max_cost  # 0 → 1
            step_cost = 1.0 + alpha * normalized_cost

            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))

    return []

def main(args=None):
    rclpy.init(args=args)
    node = ComplexPathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()