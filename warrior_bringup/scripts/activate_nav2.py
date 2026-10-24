#!/usr/bin/env python3
"""Start Nav2 after lifecycle discovery settles, then verify goal readiness."""

import sys
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2Activator(Node):
    """Coordinate reliable Nav2 startup before presenting the RViz goal UI."""

    def __init__(self):
        super().__init__('activate_nav2')
        self.declare_parameter('timeout_sec', 60.0)
        self.declare_parameter('discovery_settle_sec', 2.0)
        self.declare_parameter(
            'managed_nodes',
            [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        )

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.discovery_settle_sec = float(
            self.get_parameter('discovery_settle_sec').value)
        self.managed_nodes = list(self.get_parameter('managed_nodes').value)
        self.deadline = time.monotonic() + self.timeout_sec

        self.state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self.managed_nodes
        }
        self.manager_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )

    def _before_deadline(self):
        return rclpy.ok() and time.monotonic() < self.deadline

    def wait_for_discovery(self):
        self.get_logger().info(
            'Waiting for all Nav2 lifecycle services to be discovered...')
        while self._before_deadline():
            manager_ready = self.manager_client.service_is_ready()
            missing = [
                name for name, client in self.state_clients.items()
                if not client.service_is_ready()
            ]
            if manager_ready and not missing:
                self.get_logger().info(
                    'Nav2 lifecycle services discovered; allowing DDS '
                    f'{self.discovery_settle_sec:.1f}s to settle.')
                settle_deadline = time.monotonic() + self.discovery_settle_sec
                while (
                    self._before_deadline()
                    and time.monotonic() < settle_deadline
                ):
                    rclpy.spin_once(self, timeout_sec=0.1)
                return self._before_deadline()
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def start_navigation(self):
        self.get_logger().info('Requesting managed Nav2 startup...')
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.manager_client.call_async(request)

        while self._before_deadline() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            self.get_logger().error(
                'Timed out while the lifecycle manager was starting Nav2.')
            return False
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - surface ROS future errors
            self.get_logger().error(f'Nav2 startup service failed: {error}')
            return False
        if response is None or not response.success:
            self.get_logger().error(
                'The lifecycle manager rejected Nav2 startup.')
            return False
        return True

    def verify_navigation(self):
        bt_client = self.state_clients['bt_navigator']
        while self._before_deadline():
            request = GetState.Request()
            future = bt_client.call_async(request)
            state_deadline = min(self.deadline, time.monotonic() + 1.0)
            while self._before_deadline() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.monotonic() >= state_deadline:
                    break

            active = False
            if future.done():
                try:
                    response = future.result()
                    active = (
                        response is not None
                        and response.current_state.id
                        == State.PRIMARY_STATE_ACTIVE
                    )
                except Exception:  # noqa: BLE001 - retry transient ROS errors
                    pass

            if active and self.navigation_client.server_is_ready():
                self.get_logger().info(
                    'Nav2 is active and /navigate_to_pose is ready; '
                    'starting RViz.')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Activator()
    exit_code = 1

    try:
        if not node.wait_for_discovery():
            node.get_logger().error(
                'Timed out waiting for Nav2 lifecycle service discovery.')
        elif node.start_navigation() and node.verify_navigation():
            exit_code = 0
        else:
            node.get_logger().error(
                'Nav2 did not become ready before the startup timeout.')
    except KeyboardInterrupt:
        pass
    finally:
        node.navigation_client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
