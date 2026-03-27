import heapq
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose as ComputePathToPoseAction
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class PathPlanningError(Exception):
    def __init__(self, error_code: int, message: str):
        super().__init__(message)
        self.error_code = error_code
        self.message = message


class PathToPoseServer(Node):
    def __init__(self):
        super().__init__('path_to_pose_server')
        self.get_logger().info('Path To Pose action server has been started.')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('action_name', 'path_to_pose')
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value

        self.path_pub = self.create_publisher(Path, '/a_star_path', 10)
        self.status_pub = self.create_publisher(String, '~/planning_status', 10)
        self.cost_map_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.cost_map_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.costmap = None
        self.map_info = None
        self.max_cost = 90
        self.alpha = 5.0
        self.supported_planner_ids = {'', 'a_star'}

        self._action_server = ActionServer(
            self,
            ComputePathToPoseAction,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()

    def cost_map_callback(self, msg: OccupancyGrid):
        self.costmap = msg
        self.map_info = msg.info

    def goal_callback(self, goal_request):
        planner_id = goal_request.planner_id.strip()
        if planner_id not in self.supported_planner_ids:
            self.get_logger().warn(f'Rejecting unsupported planner_id: {planner_id}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.publish_status(f'Cancel request received for {self.action_name}.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = ComputePathToPoseAction.Result()
        planning_started_at = time.perf_counter()

        try:
            self.publish_status('Planning request accepted.')
            self.ensure_costmap_ready()

            if goal_handle.is_cancel_requested:
                return self.cancel_goal(goal_handle, result, planning_started_at)

            goal_pose = self.normalize_pose(goal_handle.request.goal, 'goal')
            if goal_handle.request.use_start:
                start_pose = self.normalize_pose(goal_handle.request.start, 'start')
            else:
                start_pose = self.get_robot_pose_map()

            start = self.world_to_grid(start_pose.pose.position.x, start_pose.pose.position.y)
            goal = self.world_to_grid(goal_pose.pose.position.x, goal_pose.pose.position.y)

            self.validate_grid_cell(start, ComputePathToPoseAction.Result.START_OUTSIDE_MAP)
            self.validate_grid_cell(goal, ComputePathToPoseAction.Result.GOAL_OUTSIDE_MAP)
            self.validate_cell_free(start, ComputePathToPoseAction.Result.START_OCCUPIED)
            self.validate_cell_free(goal, ComputePathToPoseAction.Result.GOAL_OCCUPIED)

            self.publish_status('Running A* planner.')
            grid = np.array(self.costmap.data).reshape((self.map_info.height, self.map_info.width))
            grid_path = a_star(grid, start, goal, max_cost=self.max_cost, alpha=self.alpha)

            if goal_handle.is_cancel_requested:
                return self.cancel_goal(goal_handle, result, planning_started_at)

            if not grid_path:
                raise PathPlanningError(
                    ComputePathToPoseAction.Result.NO_VALID_PATH,
                    'A* could not find a valid path to the requested goal.',
                )

            path = self.build_path(grid_path)
            self.path_pub.publish(path)

            result.path = path
            result.error_code = ComputePathToPoseAction.Result.NONE
            result.error_msg = ''
            result.planning_time = self.elapsed_duration(planning_started_at)

            goal_handle.succeed()
            self.publish_status(f'Path planning succeeded with {len(path.poses)} poses.')
            return result

        except PathPlanningError as exc:
            result.path = Path()
            result.error_code = exc.error_code
            result.error_msg = exc.message
            result.planning_time = self.elapsed_duration(planning_started_at)
            goal_handle.abort()
            self.publish_status(f'Path planning failed: {exc.message}')
            self.get_logger().warn(exc.message)
            return result
        except Exception as exc:
            result.path = Path()
            result.error_code = ComputePathToPoseAction.Result.UNKNOWN
            result.error_msg = f'Unexpected planning error: {exc}'
            result.planning_time = self.elapsed_duration(planning_started_at)
            goal_handle.abort()
            self.publish_status(result.error_msg)
            self.get_logger().error(result.error_msg)
            return result

    def cancel_goal(self, goal_handle, result, planning_started_at):
        result.path = Path()
        result.error_code = ComputePathToPoseAction.Result.UNKNOWN
        result.error_msg = 'Path planning request was canceled.'
        result.planning_time = self.elapsed_duration(planning_started_at)
        goal_handle.canceled()
        self.publish_status(result.error_msg)
        return result

    def ensure_costmap_ready(self):
        if self.costmap is None or self.map_info is None:
            raise PathPlanningError(
                ComputePathToPoseAction.Result.UNKNOWN,
                'No costmap has been received yet.',
            )

    def normalize_pose(self, pose: PoseStamped, label: str) -> PoseStamped:
        if pose.header.frame_id not in ('', self.global_frame):
            raise PathPlanningError(
                ComputePathToPoseAction.Result.TF_ERROR,
                f'The {label} pose must be in the {self.global_frame} frame.',
            )

        normalized_pose = PoseStamped()
        normalized_pose.header = pose.header
        normalized_pose.header.frame_id = self.global_frame
        normalized_pose.pose = pose.pose
        return normalized_pose

    def get_robot_pose_map(self) -> PoseStamped:
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.global_frame,
                source_frame=self.robot_base_frame,
                time=rclpy.time.Time(),
            )
        except TransformException as exc:
            raise PathPlanningError(
                ComputePathToPoseAction.Result.TF_ERROR,
                f'Failed to lookup robot pose in {self.global_frame}: {exc}',
            ) from exc

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def validate_grid_cell(self, cell, error_code: int):
        row, col = cell
        if not (0 <= row < self.map_info.height and 0 <= col < self.map_info.width):
            if error_code == ComputePathToPoseAction.Result.START_OUTSIDE_MAP:
                message = 'The start pose is outside the current costmap bounds.'
            else:
                message = 'The goal pose is outside the current costmap bounds.'
            raise PathPlanningError(error_code, message)

    def validate_cell_free(self, cell, error_code: int):
        row, col = cell
        grid = np.array(self.costmap.data).reshape((self.map_info.height, self.map_info.width))
        cell_cost = grid[row, col]
        if cell_cost < 0 or cell_cost >= self.max_cost:
            if error_code == ComputePathToPoseAction.Result.START_OCCUPIED:
                message = 'The start pose is occupied or unknown in the costmap.'
            else:
                message = 'The goal pose is occupied or unknown in the costmap.'
            raise PathPlanningError(error_code, message)

    def build_path(self, grid_path):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()

        poses = []
        for row, col in grid_path:
            x, y = self.grid_to_world(row, col)
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        path.poses = poses
        return path

    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def elapsed_duration(self, planning_started_at: float):
        elapsed_seconds = time.perf_counter() - planning_started_at
        duration = Duration()
        duration.sec = int(elapsed_seconds)
        duration.nanosec = int((elapsed_seconds - duration.sec) * 1e9)
        return duration

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

        row, col = current

        for d_row, d_col in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_row = row + d_row
            next_col = col + d_col

            if not (0 <= next_row < height and 0 <= next_col < width):
                continue

            cell_cost = grid[next_row, next_col]
            if cell_cost < 0 or cell_cost >= max_cost:
                continue

            neighbor = (next_row, next_col)
            normalized_cost = cell_cost / max_cost
            step_cost = 1.0 + alpha * normalized_cost
            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return []


def main(args=None):
    rclpy.init(args=args)
    node = PathToPoseServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
