import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose as ComputePathToPoseAction
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String


class RecoveryManager(Node):
    def __init__(self):
        super().__init__('recovery_manager')
        self.get_logger().info('Recovery manager action server has been started.')

        self.declare_parameter('action_name', 'path_to_pose')
        self.declare_parameter('planner_action_name', 'compute_path_to_pose_core')
        self.declare_parameter('rviz_goal_topic', 'goal_pose')
        self.declare_parameter('enable_rviz_goal_bridge', True)
        self.declare_parameter('max_recovery_attempts', 2)
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
        self.max_recovery_attempts = (
            self.get_parameter('max_recovery_attempts').get_parameter_value().integer_value
        )
        self.retry_wait_seconds = (
            self.get_parameter('retry_wait_seconds').get_parameter_value().double_value
        )

        self.status_pub = self.create_publisher(String, '~/recovery_status', 10)
        self._planner_client = ActionClient(self, ComputePathToPoseAction, self.planner_action_name)
        self._action_client = ActionClient(self, ComputePathToPoseAction, self.action_name)
        self._clear_global_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self._clear_local_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        self._rviz_goal_handle = None

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
            self.get_logger().info('RViz-triggered planning request completed successfully.')
            return

        self.get_logger().warn(
            f'RViz-triggered planning request failed with code {result.error_code}: '
            f'{result.error_msg}'
        )

    async def execute_callback(self, goal_handle):
        result = ComputePathToPoseAction.Result()
        if not self._planner_client.wait_for_server(timeout_sec=1.0):
            result.error_code = ComputePathToPoseAction.Result.UNKNOWN
            result.error_msg = (
                f'Planner action server {self.planner_action_name} is not available.'
            )
            goal_handle.abort()
            self.publish_status(result.error_msg)
            return result

        attempt = 0
        while attempt <= self.max_recovery_attempts:
            if goal_handle.is_cancel_requested:
                return self._cancel_result(goal_handle)

            self.publish_status(f'Planning attempt {attempt + 1} started.')
            planner_goal = ComputePathToPoseAction.Goal()
            planner_goal.goal = goal_handle.request.goal
            planner_goal.start = goal_handle.request.start
            planner_goal.use_start = goal_handle.request.use_start
            planner_goal.planner_id = goal_handle.request.planner_id

            planner_goal_handle = await self._planner_client.send_goal_async(planner_goal)
            if not planner_goal_handle.accepted:
                result.error_code = ComputePathToPoseAction.Result.UNKNOWN
                result.error_msg = 'Planner action rejected the request.'
                goal_handle.abort()
                self.publish_status(result.error_msg)
                return result

            planner_wrapped_result = await self._await_result_with_cancel(
                goal_handle,
                planner_goal_handle,
            )
            if planner_wrapped_result is None:
                return self._cancel_result(goal_handle)

            planner_result = planner_wrapped_result.result
            if (
                planner_wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and planner_result.error_code == ComputePathToPoseAction.Result.NONE
            ):
                goal_handle.succeed()
                self.publish_status(
                    f'Planning succeeded after {attempt + 1} attempt(s).'
                )
                return planner_result

            behaviors = self._error_code_behaviors.get(
                planner_result.error_code,
                self._error_code_behaviors[ComputePathToPoseAction.Result.UNKNOWN],
            )
            if attempt >= self.max_recovery_attempts or not behaviors:
                goal_handle.abort()
                self.publish_status(
                    'Recovery exhausted; returning planner failure to caller.'
                )
                return planner_result

            self.publish_status(
                f'Planner failed with code {planner_result.error_code}; '
                f'running recovery behaviors: {", ".join(behaviors)}.'
            )
            recovery_succeeded = await self._run_recovery_behaviors(behaviors, planner_result)
            if not recovery_succeeded:
                goal_handle.abort()
                self.publish_status('Recovery behavior failed; aborting request.')
                return planner_result

            attempt += 1

        goal_handle.abort()
        result.error_code = ComputePathToPoseAction.Result.UNKNOWN
        result.error_msg = 'Recovery manager exited without a valid planner result.'
        self.publish_status(result.error_msg)
        return result

    async def _await_result_with_cancel(self, outer_goal_handle, planner_goal_handle):
        result_future = planner_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if outer_goal_handle.is_cancel_requested:
                planner_goal_handle.cancel_goal_async()
                return None
            await self._sleep_for(0.05)
        return result_future.result()

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
        return await self._call_clear_service(
            self._clear_global_client,
            'global costmap',
        )

    async def _clear_local_costmap(self, _planner_result):
        return await self._call_clear_service(
            self._clear_local_client,
            'local costmap',
        )

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

    def _cancel_result(self, goal_handle):
        result = ComputePathToPoseAction.Result()
        result.error_code = ComputePathToPoseAction.Result.UNKNOWN
        result.error_msg = 'Recovery-managed planning request was canceled.'
        goal_handle.canceled()
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
