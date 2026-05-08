import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose as ComputePathToPoseAction
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class RecoveryManager(Node):
    def __init__(self):
        super().__init__('recovery_manager')
        self.get_logger().info('Recovery manager action server has been started.')

        self.declare_parameter('action_name', 'path_to_pose')
        self.declare_parameter('planner_action_name', 'compute_path_to_pose_core')
        self.declare_parameter('rviz_goal_topic', 'goal_pose')
        self.declare_parameter('enable_rviz_goal_bridge', True)
        self.declare_parameter('costmap_topic', '/costmap')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('costmap_wait_timeout', 5.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('goal_progress_timeout', 15.0)
        self.declare_parameter('goal_execution_timeout', 120.0)
        self.declare_parameter('progress_distance_threshold', 0.05)
        self.declare_parameter('max_recovery_attempts', 2)
        self.declare_parameter('max_exploration_steps', 8)
        self.declare_parameter('exploration_step_distance', 0.25)
        self.declare_parameter('retry_wait_seconds', 0.5)
        self.declare_parameter(
            'recovery_behaviors.no_valid_path',
            ['clear_global_costmap', 'wait'],
        )
        self.declare_parameter(
            'recovery_behaviors.start_occupied',
            ['clear_local_costmap', 'wait'],
        )
        self.declare_parameter(
            'recovery_behaviors.goal_occupied',
            ['clear_global_costmap', 'wait'],
        )
        self.declare_parameter('recovery_behaviors.tf_error', ['wait'])
        self.declare_parameter('recovery_behaviors.unknown', [])

        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.planner_action_name = (
            self.get_parameter('planner_action_name').get_parameter_value().string_value
        )
        self.rviz_goal_topic = self.get_parameter('rviz_goal_topic').get_parameter_value().string_value
        self.enable_rviz_goal_bridge = (
            self.get_parameter('enable_rviz_goal_bridge').get_parameter_value().bool_value
        )
        self.costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.robot_base_frame = (
            self.get_parameter('robot_base_frame').get_parameter_value().string_value
        )
        self.costmap_wait_timeout = (
            self.get_parameter('costmap_wait_timeout').get_parameter_value().double_value
        )
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.goal_progress_timeout = (
            self.get_parameter('goal_progress_timeout').get_parameter_value().double_value
        )
        self.goal_execution_timeout = (
            self.get_parameter('goal_execution_timeout').get_parameter_value().double_value
        )
        self.progress_distance_threshold = (
            self.get_parameter('progress_distance_threshold').get_parameter_value().double_value
        )
        self.max_recovery_attempts = (
            self.get_parameter('max_recovery_attempts').get_parameter_value().integer_value
        )
        self.max_exploration_steps = (
            self.get_parameter('max_exploration_steps').get_parameter_value().integer_value
        )
        self.exploration_step_distance = (
            self.get_parameter('exploration_step_distance').get_parameter_value().double_value
        )
        self.retry_wait_seconds = (
            self.get_parameter('retry_wait_seconds').get_parameter_value().double_value
        )

        self.status_pub = self.create_publisher(String, '~/recovery_status', 10)
        self.path_pub = self.create_publisher(Path, '/a_star_path', 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self._costmap_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.costmap = None
        self.map_info = None
        self.max_cost = 90
        self._rviz_goal_handle = None

        self._planner_client = ActionClient(self, ComputePathToPoseAction, self.planner_action_name)
        self._action_client = ActionClient(self, ComputePathToPoseAction, self.action_name)
        self._clear_global_client = self.create_client(
            ClearEntireCostmap,
            '/clear_entirely_costmap',
        )
        self._clear_local_client = self.create_client(
            ClearEntireCostmap,
            '/clear_entirely_costmap',
        )

        self._recovery_behavior_handlers = {
            'clear_global_costmap': self._clear_global_costmap,
            'clear_local_costmap': self._clear_local_costmap,
            'wait': self._wait_before_retry,
        }
        self._error_code_behaviors = {
            ComputePathToPoseAction.Result.NO_VALID_PATH: self._get_string_list_parameter(
                'recovery_behaviors.no_valid_path'
            ),
            ComputePathToPoseAction.Result.START_OCCUPIED: self._get_string_list_parameter(
                'recovery_behaviors.start_occupied'
            ),
            ComputePathToPoseAction.Result.GOAL_OCCUPIED: self._get_string_list_parameter(
                'recovery_behaviors.goal_occupied'
            ),
            ComputePathToPoseAction.Result.TF_ERROR: self._get_string_list_parameter(
                'recovery_behaviors.tf_error'
            ),
            ComputePathToPoseAction.Result.UNKNOWN: self._get_string_list_parameter(
                'recovery_behaviors.unknown'
            ),
        }

        self._action_server = ActionServer(
            self,
            ComputePathToPoseAction,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        if self.enable_rviz_goal_bridge:
            self.goal_pose_subscription = self.create_subscription(
                PoseStamped,
                self.rviz_goal_topic,
                self.rviz_goal_callback,
                10,
            )
        else:
            self.goal_pose_subscription = None

    def destroy_node(self):
        self._planner_client.destroy()
        self._action_client.destroy()
        self._action_server.destroy()
        super().destroy_node()

    def _costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg
        self.map_info = msg.info

    def _get_string_list_parameter(self, name: str):
        return list(self.get_parameter(name).get_parameter_value().string_array_value)

    def goal_callback(self, goal_request):
        planner_id = goal_request.planner_id.strip()
        if planner_id not in ('', 'a_star'):
            self.get_logger().warn(f'Rejecting unsupported planner_id: {planner_id}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.publish_status(f'Cancel request received for {self.action_name}.')
        self._publish_empty_path()
        return CancelResponse.ACCEPT

    def rviz_goal_callback(self, msg: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                f'RViz goal received, but action server {self.action_name} is not available yet.'
            )
            return

        if self._rviz_goal_handle is not None:
            self._rviz_goal_handle.cancel_goal_async()
            self._rviz_goal_handle = None

        goal_msg = ComputePathToPoseAction.Goal()
        goal_msg.goal = msg
        goal_msg.use_start = False
        goal_msg.planner_id = 'a_star'

        self.publish_status(
            f'Received RViz goal on {self.rviz_goal_topic}, sending recovery-managed request.'
        )
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._rviz_goal_response_callback)

    def _rviz_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send RViz goal to recovery manager: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('RViz goal was rejected by the recovery manager.')
            return

        self._rviz_goal_handle = goal_handle
        self.get_logger().info('RViz goal accepted by the recovery manager.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._rviz_goal_result_callback)

    def _rviz_goal_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f'Failed to get action result for RViz goal: {exc}')
            return
        finally:
            self._rviz_goal_handle = None

        if result.error_code == ComputePathToPoseAction.Result.NONE:
            self.get_logger().info('RViz-triggered navigation request completed successfully.')
            return

        self.get_logger().warn(
            f'RViz-triggered navigation request failed with code {result.error_code}: '
            f'{result.error_msg}'
        )

    async def execute_callback(self, goal_handle):
        result = ComputePathToPoseAction.Result()
        result.path = Path()

        if not self._planner_client.wait_for_server(timeout_sec=1.0):
            result.error_code = ComputePathToPoseAction.Result.UNKNOWN
            result.error_msg = (
                f'Planner action server {self.planner_action_name} is not available.'
            )
            goal_handle.abort()
            self.publish_status(result.error_msg)
            return result

        costmap_ready = await self._wait_for_costmap(goal_handle)
        if goal_handle.is_cancel_requested:
            return self._cancel_result(goal_handle)
        if not costmap_ready:
            result.error_code = ComputePathToPoseAction.Result.UNKNOWN
            result.error_msg = (
                f'No costmap has been received on {self.costmap_topic} within '
                f'{self.costmap_wait_timeout:.1f} seconds.'
            )
            goal_handle.abort()
            self.publish_status(result.error_msg)
            return result

        try:
            navigation_result = await self._navigate_to_goal(goal_handle)
        except ValueError as exc:
            result.error_code = ComputePathToPoseAction.Result.TF_ERROR
            result.error_msg = str(exc)
            goal_handle.abort()
            self.publish_status(result.error_msg)
            return result

        if navigation_result is None:
            return self._cancel_result(goal_handle)

        if navigation_result.error_code == ComputePathToPoseAction.Result.NONE:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            self._publish_empty_path()

        return navigation_result

    async def _navigate_to_goal(self, goal_handle):
        original_goal = self._normalized_pose(goal_handle.request.goal)
        exploration_steps = 0
        recovery_attempts = 0
        last_result = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return None

            self.publish_status(
                f'Planning toward final goal (exploration step {exploration_steps}/{self.max_exploration_steps}).'
            )
            plan_result = await self._request_plan(goal_handle, original_goal)
            if plan_result is None:
                return None

            if plan_result.error_code == ComputePathToPoseAction.Result.NONE:
                self.publish_status('Path found; waiting for robot to reach the goal pose.')
                reached = await self._wait_until_pose_reached(goal_handle, original_goal)
                if reached is None:
                    return None
                if reached:
                    plan_result.error_msg = ''
                    self.publish_status('Goal pose reached successfully.')
                    return plan_result

                recovery_attempts += 1
                last_result = self._make_result(
                    ComputePathToPoseAction.Result.NO_VALID_PATH,
                    'Robot stopped making progress while following the planned path.',
                    path=plan_result.path,
                )
            else:
                last_result = plan_result

            goal_state = self._classify_pose(original_goal)
            if goal_state in ('unknown', 'outside_map'):
                if exploration_steps >= self.max_exploration_steps:
                    return self._make_result(
                        ComputePathToPoseAction.Result.NO_VALID_PATH,
                        'Goal remained unknown after exhausting exploratory moves.',
                    )

                exploratory_outcome = await self._perform_exploratory_step(goal_handle, original_goal)
                if exploratory_outcome is None:
                    return None
                if exploratory_outcome:
                    exploration_steps += 1
                    recovery_attempts = 0
                    continue

                if self._classify_pose(original_goal) == 'occupied':
                    return self._make_result(
                        ComputePathToPoseAction.Result.GOAL_OCCUPIED,
                        'Goal cell became known and is occupied after clearing costmaps; skipping this goal.',
                    )

                return self._make_result(
                    ComputePathToPoseAction.Result.NO_VALID_PATH,
                    'Could not find a reachable known point toward the requested goal.',
                )

            if goal_state == 'occupied':
                cleared = await self._clear_all_costmaps()
                if not cleared:
                    return self._make_result(
                        ComputePathToPoseAction.Result.GOAL_OCCUPIED,
                        'Goal cell is occupied and costmaps could not be cleared.',
                    )

                await self._wait_before_retry(last_result)
                if self._classify_pose(original_goal) == 'occupied':
                    return self._make_result(
                        ComputePathToPoseAction.Result.GOAL_OCCUPIED,
                        'Goal cell is occupied after clearing costmaps; skipping this goal.',
                    )
                continue

            if recovery_attempts >= self.max_recovery_attempts:
                return last_result

            behaviors = self._error_code_behaviors.get(
                last_result.error_code,
                self._error_code_behaviors[ComputePathToPoseAction.Result.UNKNOWN],
            )
            if not behaviors:
                return last_result

            self.publish_status(
                f'Planner/execution failed with code {last_result.error_code}; '
                f'running recovery behaviors: {", ".join(behaviors)}.'
            )
            recovery_succeeded = await self._run_recovery_behaviors(behaviors, last_result)
            if not recovery_succeeded:
                return last_result

            recovery_attempts += 1

        return self._make_result(
            ComputePathToPoseAction.Result.UNKNOWN,
            'Recovery manager exited without a terminal navigation result.',
        )

    async def _perform_exploratory_step(self, goal_handle, final_goal: PoseStamped):
        exploratory_goal = self._find_exploratory_goal(final_goal)
        if exploratory_goal is None:
            self.publish_status('No free known point exists yet toward the unknown goal.')
            return False

        self.publish_status(
            'Goal is currently unknown; moving toward the closest known free point first.'
        )
        exploratory_result = await self._request_plan(goal_handle, exploratory_goal)
        if exploratory_result is None:
            return None

        if exploratory_result.error_code != ComputePathToPoseAction.Result.NONE:
            self.publish_status(
                'Exploratory point was not reachable; clearing costmaps and checking again.'
            )
            cleared = await self._clear_all_costmaps()
            if not cleared:
                return False

            await self._wait_before_retry(exploratory_result)
            goal_state = self._classify_pose(final_goal)
            if goal_state == 'occupied':
                return False

            exploratory_goal = self._find_exploratory_goal(final_goal)
            if exploratory_goal is None:
                return False

            exploratory_result = await self._request_plan(goal_handle, exploratory_goal)
            if exploratory_result is None:
                return None
            if exploratory_result.error_code != ComputePathToPoseAction.Result.NONE:
                return False

        reached = await self._wait_until_pose_reached(goal_handle, exploratory_goal)
        if reached is None:
            return None
        return reached

    async def _request_plan(self, goal_handle, target_goal: PoseStamped):
        planner_goal = ComputePathToPoseAction.Goal()
        planner_goal.goal = target_goal
        planner_goal.start = goal_handle.request.start
        planner_goal.use_start = goal_handle.request.use_start
        planner_goal.planner_id = goal_handle.request.planner_id

        planner_goal_handle = await self._planner_client.send_goal_async(planner_goal)
        if not planner_goal_handle.accepted:
            return self._make_result(
                ComputePathToPoseAction.Result.UNKNOWN,
                'Planner action rejected the request.',
            )

        planner_wrapped_result = await self._await_result_with_cancel(
            goal_handle,
            planner_goal_handle,
        )
        if planner_wrapped_result is None:
            return None

        planner_result = planner_wrapped_result.result
        if (
            planner_wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and planner_result.error_code == ComputePathToPoseAction.Result.NONE
        ):
            return planner_result

        return self._make_result(
            planner_result.error_code,
            planner_result.error_msg,
            path=planner_result.path,
            planning_time=planner_result.planning_time,
        )

    async def _await_result_with_cancel(self, outer_goal_handle, planner_goal_handle):
        result_future = planner_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if outer_goal_handle.is_cancel_requested:
                planner_goal_handle.cancel_goal_async()
                return None
            await self._sleep_for(0.05)
        return result_future.result()

    async def _wait_until_pose_reached(self, goal_handle, target_goal: PoseStamped):
        deadline = time.monotonic() + self.goal_execution_timeout
        progress_deadline = time.monotonic() + self.goal_progress_timeout
        best_distance = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return None

            robot_pose = self._get_robot_pose()
            if robot_pose is None:
                await self._sleep_for(0.1)
                continue

            distance = math.hypot(
                target_goal.pose.position.x - robot_pose.pose.position.x,
                target_goal.pose.position.y - robot_pose.pose.position.y,
            )
            if distance <= self.goal_tolerance:
                return True

            if best_distance is None or distance < best_distance - self.progress_distance_threshold:
                best_distance = distance
                progress_deadline = time.monotonic() + self.goal_progress_timeout

            now = time.monotonic()
            if now >= deadline:
                self.publish_status('Execution timeout reached before the robot reached the target.')
                return False
            if now >= progress_deadline:
                self.publish_status('Robot stopped making progress toward the target pose.')
                return False

            await self._sleep_for(0.1)

        return False

    async def _wait_for_costmap(self, goal_handle):
        if self.costmap is not None and self.map_info is not None:
            return True

        deadline = time.monotonic() + self.costmap_wait_timeout
        self.publish_status(
            f'Waiting for costmap on {self.costmap_topic} before navigation.'
        )

        while self.costmap is None or self.map_info is None:
            if goal_handle.is_cancel_requested:
                return False
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f'No costmap has been received on {self.costmap_topic} within '
                    f'{self.costmap_wait_timeout:.1f} seconds.'
                )
                return False
            await self._sleep_for(0.1)
        return True

    async def _run_recovery_behaviors(self, behaviors, planner_result):
        for behavior_name in behaviors:
            handler = self._recovery_behavior_handlers.get(behavior_name)
            if handler is None:
                self.get_logger().error(
                    f'Unknown recovery behavior "{behavior_name}" for error code '
                    f'{planner_result.error_code}.'
                )
                return False

            self.publish_status(f'Running recovery behavior: {behavior_name}.')
            succeeded = await handler(planner_result)
            if not succeeded:
                return False
        return True

    async def _clear_global_costmap(self, _planner_result):
        return await self._call_clear_show_service(
            self._clear_global_client,
            'global costmap',
        )

    async def _clear_local_costmap(self, _planner_result):
        return await self._call_clear_show_service(
            self._clear_local_client,
            'local costmap',
        )

    async def _clear_all_costmaps(self):
        global_ok = await self._clear_global_costmap(None)
        local_ok = await self._clear_local_costmap(None)
        return global_ok and local_ok

    async def _wait_before_retry(self, _planner_result):
        await self._sleep_for(self.retry_wait_seconds)
        return True

    async def _call_clear_show_service(self, client, label: str):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Clear service for {label} is not available.')
            return False

        future = client.call_async(ClearEntireCostmap.Request())
        while rclpy.ok() and not future.done():
            await self._sleep_for(0.05)

        if future.result() is None:
            self.get_logger().warn(f'Clear service for {label} returned no response.')
            return False

        return True

    async def _sleep_for(self, seconds: float):
        future = Future()
        timer = None

        def _finish_wait():
            nonlocal timer
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                timer = None
            if not future.done():
                future.set_result(True)

        timer = self.create_timer(seconds, _finish_wait)
        await future

    def _normalized_pose(self, pose: PoseStamped):
        if pose.header.frame_id not in ('', self.global_frame):
            raise ValueError(
                f'The goal pose must be provided in the {self.global_frame} frame.'
            )

        normalized = PoseStamped()
        normalized.header = pose.header
        normalized.header.frame_id = self.global_frame
        normalized.pose = pose.pose
        return normalized

    def _get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.global_frame,
                source_frame=self.robot_base_frame,
                time=Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Failed to lookup robot pose in {self.global_frame}: {exc}')
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _find_exploratory_goal(self, final_goal: PoseStamped):
        if self.map_info is None:
            return None

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return None

        start_x = robot_pose.pose.position.x
        start_y = robot_pose.pose.position.y
        goal_x = final_goal.pose.position.x
        goal_y = final_goal.pose.position.y

        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance < self.goal_tolerance:
            return None

        direction_x = (goal_x - start_x) / distance
        direction_y = (goal_y - start_y) / distance
        step = max(self.map_info.resolution, self.exploration_step_distance)

        samples = max(1, int(math.ceil(distance / step)))
        for index in range(samples, -1, -1):
            sample_distance = min(distance, index * step)
            x = start_x + direction_x * sample_distance
            y = start_y + direction_y * sample_distance
            point_state = self._classify_world_point(x, y)
            if point_state == 'free':
                exploratory_goal = PoseStamped()
                exploratory_goal.header.frame_id = self.global_frame
                exploratory_goal.header.stamp = self.get_clock().now().to_msg()
                exploratory_goal.pose = final_goal.pose
                exploratory_goal.pose.position.x = x
                exploratory_goal.pose.position.y = y
                return exploratory_goal

        return None

    def _classify_pose(self, pose: PoseStamped):
        return self._classify_world_point(
            pose.pose.position.x,
            pose.pose.position.y,
        )

    def _classify_world_point(self, x: float, y: float):
        if self.map_info is None or self.costmap is None:
            return 'outside_map'

        row, col = self._world_to_grid(x, y)
        if not (0 <= row < self.map_info.height and 0 <= col < self.map_info.width):
            return 'outside_map'

        grid = np.array(self.costmap.data).reshape((self.map_info.height, self.map_info.width))
        cell_cost = grid[row, col]
        if cell_cost < 0:
            return 'unknown'
        if cell_cost >= self.max_cost:
            return 'occupied'
        return 'free'

    def _world_to_grid(self, x: float, y: float):
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return row, col

    def _publish_empty_path(self):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)

    def _make_result(self, error_code: int, error_msg: str, path=None, planning_time=None):
        result = ComputePathToPoseAction.Result()
        result.error_code = error_code
        result.error_msg = error_msg
        result.path = path if path is not None else Path()
        result.planning_time = planning_time if planning_time is not None else Duration()
        return result

    def _cancel_result(self, goal_handle):
        result = self._make_result(
            ComputePathToPoseAction.Result.UNKNOWN,
            'Recovery-managed navigation request was canceled.',
        )
        goal_handle.canceled()
        self._publish_empty_path()
        self.publish_status(result.error_msg)
        return result

    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
