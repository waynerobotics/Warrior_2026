"""Check the example GPS route against the actual simulator scene and sensor."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from pyproj import Transformer
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
GAZEBO = ROOT / 'warrior_simulation' / 'warrior_gazebo'
WORLD = ET.parse(GAZEBO / 'worlds' / 'competition.world').getroot().find('world')
SPAWN = (17.0, 3.0)


def _world_route():
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
         'no_mans_land_waypoints.yaml').read_text())['waypoints']
    points = []
    for waypoint in route:
        assert set(waypoint) == {'latitude', 'longitude', 'yaw'}
        assert math.isfinite(waypoint['yaw'])
        x, y, _ = transform.transform(
            waypoint['longitude'], waypoint['latitude'],
            float(datum.findtext('elevation')))
        points.append((x, y))
    return points


def _pose(element):
    values = [float(value) for value in element.findtext('pose').split()]
    return values[0], values[1], values[5]


def _compose(parent, child):
    x, y, yaw = parent
    dx, dy, dyaw = child
    return (
        x + dx * math.cos(yaw) - dy * math.sin(yaw),
        y + dx * math.sin(yaw) + dy * math.cos(yaw),
        yaw + dyaw,
    )


def _distance_to_segment(point, start, end):
    vx, vy = end[0] - start[0], end[1] - start[1]
    wx, wy = point[0] - start[0], point[1] - start[1]
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / (vx * vx + vy * vy)))
    return math.hypot(wx - t * vx, wy - t * vy)


def test_route_matches_world_datum_and_returns_to_spawn():
    """Catch accidentally reusing the distant Sonoma demo GPS coordinates."""
    points = _world_route()
    assert len(points) == 3
    for actual, expected in zip(points, [(17.0, 21.0), (14.0, 18.0), SPAWN]):
        assert actual == pytest.approx(expected, abs=0.001)


def test_first_goal_is_outside_lidar_range():
    lidar = ET.parse(
        ROOT / 'warrior_description' / 'xacro' / 'sensors' / 'lidar.xacro')
    sensor_range = float(lidar.getroot().findtext('.//ray/range/max'))
    assert math.dist(SPAWN, _world_route()[0]) > sensor_range


def test_route_stays_on_track_surface_and_clear_of_barrels():
    """Check complete straight segments, including the return leg."""
    route = [SPAWN, *_world_route()]
    track_frame = _pose(WORLD.find("frame[@name='igvc_frame']"))
    track = ET.parse(GAZEBO / 'models' / 'igvc_track' / 'model.sdf')
    size = [float(value) for value in
            track.getroot().findtext('.//visual/geometry/box/size').split()]
    for x, y in route:
        local = _compose(
            (0.0, 0.0, -track_frame[2]),
            (x - track_frame[0], y - track_frame[1], 0.0))
        assert abs(local[0]) < size[0] / 2.0 - 2.0
        assert abs(local[1]) < size[1] / 2.0 - 2.0

    barrels = WORLD.find("model[@name='construction_barrels_group']")
    barrel_frame = _compose(track_frame, _pose(barrels))
    for barrel in barrels.findall('include'):
        center = _compose(barrel_frame, _pose(barrel))
        for start, end in zip(route, route[1:]):
            # The model's cylinder radius is 0.3 m. Requiring 2 m from its
            # center leaves room for the Warrior footprint and inflation.
            assert _distance_to_segment(center, start, end) > 2.0, (
                barrel.findtext('name'), start, end)
