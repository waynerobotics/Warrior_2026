import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from heapq import heappush, heappop
import math

## ROUGH JUST AI FOR AN IDEA DON"T USE THIS

class AStarPathPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner')
        
        self.goal_pose = None
        self.costmap = None
        self.current_pose = None
        self.path = []
        self.path_index = 0
        
        self.goal_sub = self.create_subscription( PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.costmap_sub = self.create_subscription( OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.pose_sub = self.create_subscription( PoseStamped, '/odom', self.pose_callback, 10)
        
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def goal_callback(self, msg):
        self.goal_pose = msg
        if self.costmap and self.current_pose:
            self.plan_path()
    
    def costmap_callback(self, msg):
        self.costmap = msg
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def plan_path(self):
        start = (int(self.current_pose.pose.position.x / self.costmap.info.resolution),
                 int(self.current_pose.pose.position.y / self.costmap.info.resolution))
        goal = (int(self.goal_pose.pose.position.x / self.costmap.info.resolution),
                int(self.goal_pose.pose.position.y / self.costmap.info.resolution))
        
        self.path = self.astar(start, goal)
        self.path_index = 0
    
    def astar(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        
        width = self.costmap.info.width
        height = self.costmap.info.height
        
        while open_set:
            _, current = heappop(open_set)
            
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                    idx = neighbor[1] * width + neighbor[0]
                    if self.costmap.data[idx] > 50:
                        continue
                    
                    tentative_g = g_score[current] + 1
                    
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + self.heuristic(neighbor, goal)
                        heappush(open_set, (f, neighbor))
        
        return []
    
    def heuristic(self, pos, goal):
        return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    def control_loop(self):
        if not self.path or self.path_index >= len(self.path):
            return
        
        twist = Twist()
        twist.linear.x = 0.2
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()