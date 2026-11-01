"""Validate the schematic-inspired GPS section, without claiming an official map."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from pyproj import Transformer
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
GAZEBO = ROOT / 'warrior_simulation' / 'warrior_gazebo'
WORLD = ET.parse(
    GAZEBO / 'worlds' / 'igvc2027_no_mans_land.world').getroot().find('world')
EXPECTED_GOALS = [(0.0, 0.0), (0.0, 17.0), (0.0, 22.0), (0.0, 40.0)]


def _barrel_centers():
    return [tuple(map(float, barrel.findtext('pose').split()[:2]))
            for barrel in WORLD.find("model[@name='barrels']").findall('include')]


def _minimum_center_distance(start, end):
    vx, vy = end[0] - start[0], end[1] - start[1]
    length_sq = vx * vx + vy * vy
    distances = []
    for x, y in _barrel_centers():
        wx, wy = x - start[0], y - start[1]
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
        distances.append(math.hypot(wx - t * vx, wy - t * vy))
    return min(distances)


def test_gps_goals_match_world_datum_order_and_visual_markers():
    datum = WORLD.find('spherical_coordinates')
    assert datum.findtext('world_frame_orientation') == 'ENU'
    assert float(datum.findtext('heading_deg')) == 0.0
    transform = Transformer.from_pipeline(
        '+proj=pipeline +step +proj=cart +ellps=WGS84 '
        '+step +proj=topocentric +ellps=WGS84 '
        f'+lat_0={datum.findtext("latitude_deg")} '
        f'+lon_0={datum.findtext("longitude_deg")} '
        f'+h_0={datum.findtext("elevation")}')
    route = yaml.safe_load(
        (ROOT / 'warrior_navigation' / 'config' /
         'igvc2027_no_mans_land_waypoints.yaml').read_text())['waypoints']
    assert len(route) == len(EXPECTED_GOALS)
    for index, (waypoint, expected) in enumerate(zip(route, EXPECTED_GOALS), 1):
        actual = transform.transform(
            waypoint['longitude'], waypoint['latitude'],
            float(datum.findtext('elevation')))
        assert actual[:2] == pytest.approx(expected, abs=0.001)
        assert waypoint['yaw'] == pytest.approx(math.pi / 2.0)
        marker = WORLD.find(f"model[@name='waypoint_{index}']")
        assert tuple(map(float, marker.findtext('pose').split()[:2])) == expected
        assert marker.findall('.//collision') == []


def test_route_has_two_legs_beyond_the_lidar_range():
    lidar = ET.parse(
        ROOT / 'warrior_description' / 'xacro' / 'sensors' / 'lidar.xacro')
    sensor_range = float(lidar.getroot().findtext('.//ray/range/max'))
    lengths = [math.dist(start, end)
               for start, end in zip(EXPECTED_GOALS, EXPECTED_GOALS[1:])]
    assert sum(length > sensor_range for length in lengths) >= 2


def test_goals_are_clear_and_barrel_rows_can_be_bypassed():
    """A witness path checks geometry, not Nav2's ability to find or follow it."""
    params = yaml.safe_load(
        (ROOT / 'warrior_navigation' / 'config' /
         'nav2_gps_sim_params.yaml').read_text())
    robot_radius = params['local_costmap']['local_costmap']['ros__parameters'][
        'robot_radius']
    barrel = ET.parse(GAZEBO / 'models' / 'construction_barrel' / 'model.sdf')
    barrel_radius = float(barrel.getroot().findtext(
        './/collision/geometry/cylinder/radius'))
    clearance = robot_radius + barrel_radius + 0.2
    assert WORLD.find("model[@name='barrels']/static").text == 'true'
    for goal in [(0.0, -2.0), *EXPECTED_GOALS]:
        assert min(math.dist(goal, center) for center in _barrel_centers()) > clearance

    # Straight driving between interior goals collides with the central row;
    # navigation must actually turn around a finite obstacle arrangement.
    assert _minimum_center_distance(EXPECTED_GOALS[1], EXPECTED_GOALS[2]) == 0.0
    witness = [
        (0.0, -2.0), (0.0, 0.0), (2.2, 12.0), (2.2, 17.0),
        (0.0, 17.0), (2.2, 18.0), (2.2, 22.0), (0.0, 22.0),
        (2.2, 23.0), (2.2, 28.0), (0.0, 40.0),
    ]
    for start, end in zip(witness, witness[1:]):
        assert _minimum_center_distance(start, end) > clearance, (start, end)
