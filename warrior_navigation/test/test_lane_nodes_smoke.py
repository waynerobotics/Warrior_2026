"""Construct the lane-following ROS nodes: catches errors that only show at startup.

(lane_mission once assigned self.handle, a read-only rclpy Node property, and
died at launch; the pure-logic tests could not see that.)
"""

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def ros():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('nav2_msgs.action')
    pytest.importorskip('robot_localization.srv')
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def test_lane_follower_node_constructs(ros):
    from warrior_navigation.lane_follower import LaneFollower
    node = LaneFollower()
    assert node.enabled
    node.destroy_node()


def test_lane_mission_node_constructs(ros):
    from rclpy.parameter import Parameter
    from warrior_navigation import lane_mission
    waypoints = ROOT / 'warrior_navigation/config/igvc2027_autonav_nml_waypoints.yaml'

    class Probe(lane_mission.LaneMission):
        def __init__(self):
            # Parameters must exist before the real constructor reads them.
            rclpy_node_init = lane_mission.Node.__init__

            def patched(node_self, name, **kw):
                rclpy_node_init(node_self, name, parameter_overrides=[
                    Parameter('waypoint_file', value=str(waypoints))], **kw)
            lane_mission.Node.__init__ = patched
            try:
                super().__init__()
            finally:
                lane_mission.Node.__init__ = rclpy_node_init

    node = Probe()
    assert len(node.gps) == 4
    node.destroy_node()


def test_travel_heading_turns_at_most_max_step_per_goal():
    import math
    pytest.importorskip('rclpy')
    from warrior_navigation.lane_follower import steer_heading
    step = math.radians(20)
    h = 0.0
    for _ in range(4):   # goals always at the fan edge, 70 deg off
        h = steer_heading(h, h + math.radians(70), 0.5, step)
    assert abs(h - 4 * step) < 1e-9
    assert abs(steer_heading(0.0, math.radians(10), 0.5, step) - math.radians(5)) < 1e-9
