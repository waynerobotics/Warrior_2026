"""Regression coverage for GPS conversion and rolling-costmap mission behavior."""

import math
from concurrent.futures import Future
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from pyproj import Proj

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from warrior_navigation.waypoint_utils import (  # noqa: E402
    costmap_allows_goal,
    load_waypoints,
    next_navigation_goal,
    project_waypoints,
)


def test_load_demo_format_and_keep_far_destination():
    path = Path(__file__).resolve().parents[1] / 'config' / 'demo_waypoints.yaml'
    waypoints = load_waypoints(path)
    origin = waypoints[0]
    converted = project_waypoints(waypoints, origin['latitude'], origin['longitude'])
    assert len(converted) == len(waypoints) == 5
    assert math.hypot(converted[-1]['x'], converted[-1]['y']) > 100.0


@pytest.mark.parametrize('contents', [
    'waypoints: []',
    'waypoints: [{latitude: 42.0}]',
    'waypoints: [{latitude: 91.0, longitude: 0.0}]',
    'waypoints: [{latitude: 0.0, longitude: .nan}]',
    'waypoints: [{latitude: 0.0, longitude: 0.0, yaw: invalid}]',
    'waypoints: [',
])
def test_invalid_mission_is_rejected_instead_of_skipping_waypoints(tmp_path, contents):
    path = tmp_path / 'mission.yaml'
    path.write_text(contents)
    with pytest.raises(ValueError):
        load_waypoints(path)


def test_projection_selects_california_zone_and_applies_inverse_frame_rotation():
    origin_lat, origin_lon = 38.16, -122.45
    waypoint = {'latitude': origin_lat + 0.001, 'longitude': origin_lon, 'yaw': math.pi}
    projection = Proj(proj='utm', zone=10, ellps='WGS84')
    x0, y0 = projection(origin_lon, origin_lat)
    x1, y1 = projection(waypoint['longitude'], waypoint['latitude'])
    result = project_waypoints([waypoint], origin_lat, origin_lon, math.pi / 2)[0]
    assert result['x'] == pytest.approx(y1 - y0)
    assert result['y'] == pytest.approx(-(x1 - x0))
    assert result['yaw'] == pytest.approx(math.pi / 2)


def test_long_goal_advances_in_bounded_legs_and_preserves_final_yaw():
    target = {'x': 100.0, 'y': 80.0, 'yaw': -1.2}
    x, y = -7.0, 2.0
    intermediate_count = 0
    for _ in range(20):
        goal, final = next_navigation_goal(x, y, target, 20.0)
        assert math.hypot(goal['x'] - x, goal['y'] - y) <= 20.0 + 1e-9
        if final:
            assert goal == target
            break
        assert goal['yaw'] == pytest.approx(math.atan2(target['y'] - y, target['x'] - x))
        intermediate_count += 1
        # A successful navigation can end short of the commanded pose and
        # off the original straight line; the next leg uses measured TF.
        x, y = goal['x'] - 0.15, goal['y'] + 0.08
    else:
        pytest.fail('Segmented goals did not converge on the actual waypoint')
    assert intermediate_count >= 5


def test_zero_distance_retains_direct_goal_behavior():
    target = {'x': 100.0, 'y': 0.0, 'yaw': 0.7}
    assert next_navigation_goal(0.0, 0.0, target, 0.0) == (target, True)


def test_segmentation_boundary_inside_barrel_is_moved_before_barrel():
    target = {'x': 0.0, 'y': -14.0, 'yaw': -math.pi/2}
    free = lambda x, y: math.hypot(x, y+10.0) > 1.1
    goal, final = next_navigation_goal(0, 0, target, 10, free)
    assert not final
    assert free(goal['x'], goal['y'])
    assert -10 < goal['y'] < -8
    # After reaching the intermediate point, send the original GPS target.
    assert next_navigation_goal(goal['x'], goal['y'], target, 10, free) == (target, True)


def test_obstacle_check_never_relocates_an_actual_gps_target():
    target = {'x': 4., 'y': 2., 'yaw': .7}
    assert next_navigation_goal(0, 0, target, 10, lambda x, y: False) == (target, True)
    with pytest.raises(ValueError, match='No traversable'):
        next_navigation_goal(0, 0, target, 2, lambda x, y: False)


def test_short_intermediate_bound_can_also_shorten_an_occupied_endpoint():
    goal, final = next_navigation_goal(
        0, 0, {'x': 2., 'y': 0., 'yaw': 0.}, .5, lambda x, y: x < .4)
    assert not final
    assert goal['x'] == pytest.approx(.25)


def test_costmap_respects_footprint_cost_unknown_outside_and_rotated_origin():
    origin = SimpleNamespace(position=SimpleNamespace(x=10., y=20.),
        orientation=SimpleNamespace(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5)))
    grid = SimpleNamespace(info=SimpleNamespace(
        resolution=1., width=2, height=2, origin=origin), data=[0, 99, -1, 100])
    assert costmap_allows_goal(grid, 9.5, 20.5)
    assert not costmap_allows_goal(grid, 9.5, 21.5)
    assert costmap_allows_goal(grid, 8.5, 20.5)
    assert not costmap_allows_goal(grid, 8.5, 21.5)
    assert not costmap_allows_goal(grid, 11., 20.)


@pytest.fixture
def follower_class():
    from warrior_navigation.nav2_gps_waypoint_follower import Nav2GpsWaypointFollower
    return Nav2GpsWaypointFollower


@pytest.mark.parametrize('final, expected_index', [(False, 2), (True, 3)])
def test_only_actual_waypoint_success_advances_mission(follower_class, final, expected_index):
    node = SimpleNamespace(
        _active_goal_is_final=final,
        _current_waypoint_index=2,
        _publish_waypoint_markers=Mock(),
        _queue_next_goal=Mock(),
        _fail_mission=Mock(),
        get_logger=Mock(return_value=Mock()),
    )
    follower_class._result_callback(node, Mock(result=lambda: SimpleNamespace(status=4)))
    assert node._current_waypoint_index == expected_index
    node._queue_next_goal.assert_called_once()
    node._fail_mission.assert_not_called()


@pytest.mark.parametrize('status', [5, 6])
def test_aborted_or_canceled_goal_never_skips_waypoint(follower_class, status):
    node = SimpleNamespace(
        _active_goal_is_final=False,
        _current_waypoint_index=2,
        _queue_next_goal=Mock(),
        _fail_mission=Mock(),
    )
    follower_class._result_callback(node, Mock(result=lambda: SimpleNamespace(status=status)))
    assert node._current_waypoint_index == 2
    node._fail_mission.assert_called_once()
    node._queue_next_goal.assert_not_called()


def test_rejected_goal_stops_without_advancing(follower_class):
    node = SimpleNamespace(_current_waypoint_index=1, _fail_mission=Mock())
    follower_class._goal_response_callback(node, Mock(result=lambda: SimpleNamespace(accepted=False)))
    assert node._current_waypoint_index == 1
    node._fail_mission.assert_called_once()
    assert not node._goal_in_flight


def test_from_ll_conversion_waits_for_navsat_odometry(follower_class):
    node = SimpleNamespace(
        _mission_failed=False, _mission_complete=False, _mission_started=False,
        _startup_started=time.monotonic(), wait_for_action_timeout=120.0,
        _action_client=Mock(server_is_ready=lambda: True),
        _tf_buffer=Mock(can_transform=lambda *args: True),
        global_frame='map', robot_base_frame='base_footprint',
        conversion_mode='from_ll', waypoints_map=[], waypoints_gps=[{}],
        _navsat_ready=False, _from_ll_client=Mock(),
        _wait_message=Mock(), _queue_next_goal=Mock(),
    )
    follower_class._startup_callback(node)
    node._from_ll_client.call_async.assert_not_called()
    node._queue_next_goal.assert_not_called()
    assert not node._mission_started


def test_startup_timeout_stops_instead_of_waiting_forever(follower_class):
    node = SimpleNamespace(
        _mission_failed=False, _mission_complete=False, _mission_started=False,
        _startup_started=time.monotonic() - 10.0, wait_for_action_timeout=1.0,
        _last_wait_message='Waiting for navsat odometry', _fail_mission=Mock(),
    )
    follower_class._startup_callback(node)
    node._fail_mission.assert_called_once()


def test_from_ll_finishes_every_conversion_before_sending_any_goal(follower_class):
    waypoints = [
        {'latitude': 42.0, 'longitude': -83.0, 'altitude': 0.0, 'yaw': 1.1},
        {'latitude': 42.001, 'longitude': -83.0, 'altitude': 0.0, 'yaw': -0.7},
    ]
    node = SimpleNamespace(
        _mission_failed=False, _mission_complete=False, _mission_started=False,
        _startup_started=time.monotonic(), wait_for_action_timeout=120.0,
        _action_client=Mock(server_is_ready=lambda: True),
        _tf_buffer=Mock(can_transform=lambda *args: True),
        global_frame='map', robot_base_frame='base_footprint', map_origin_yaw=0.0,
        conversion_mode='from_ll', waypoints_map=[], waypoints_gps=waypoints,
        _navsat_ready=True, _conversion_future=None,
        _from_ll_client=Mock(service_is_ready=lambda: True),
        _wait_message=Mock(), _queue_next_goal=Mock(), _fail_mission=Mock(),
        _publish_waypoint_markers=Mock(), _publish_trajectory=Mock(),
        get_logger=Mock(return_value=Mock()),
    )
    node._conversion_callback = lambda future: follower_class._conversion_callback(node, future)

    def convert(request):
        future = Future()
        future.set_result(SimpleNamespace(map_point=SimpleNamespace(
            x=8.0, y=1000.0 * (request.ll_point.latitude - 42.0), z=0.0)))
        return future

    node._from_ll_client.call_async.side_effect = convert
    for _ in waypoints:
        follower_class._startup_callback(node)
        node._queue_next_goal.assert_not_called()
        assert not node._mission_started
    follower_class._startup_callback(node)
    node._queue_next_goal.assert_called_once()
    assert node._mission_started
    assert [point['yaw'] for point in node.waypoints_map] == [1.1, -0.7]
    assert [point['y'] for point in node.waypoints_map] == pytest.approx([0.0, 1.0])
