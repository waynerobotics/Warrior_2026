"""GPS waypoint parsing and geometry without ROS node dependencies."""

import math

from pyproj import Proj
import yaml


def load_waypoints(filepath):
    """Read the demo YAML format, rejecting an incomplete or invalid mission."""
    try:
        with open(filepath, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid waypoint YAML: {exc}') from exc
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list):
        raise ValueError('Waypoint file must contain a "waypoints" list')
    if not data['waypoints']:
        raise ValueError('Waypoint list is empty')

    waypoints = []
    for index, entry in enumerate(data['waypoints'], start=1):
        try:
            waypoint = {
                'latitude': float(entry['latitude']),
                'longitude': float(entry['longitude']),
                'yaw': float(entry.get('yaw', 0.0)),
                'altitude': float(entry.get('altitude', 0.0)),
            }
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            raise ValueError(f'Waypoint {index} requires numeric latitude/longitude/yaw') from exc
        if not all(math.isfinite(value) for value in waypoint.values()):
            raise ValueError(f'Waypoint {index} contains non-finite coordinates')
        if not -90.0 <= waypoint['latitude'] <= 90.0:
            raise ValueError(f'Waypoint {index} latitude is outside [-90, 90]')
        if not -180.0 <= waypoint['longitude'] <= 180.0:
            raise ValueError(f'Waypoint {index} longitude is outside [-180, 180]')
        waypoints.append(waypoint)
    return waypoints


def project_waypoints(waypoints, origin_latitude, origin_longitude, origin_yaw=0.0,
                      utm_zone=0, utm_hemisphere=''):
    """Convert UTM offsets into a frame whose x-axis has ``origin_yaw`` in UTM.

    Zone 0 and an empty hemisphere select the projection from the origin.
    Live GPS localization should instead use navsat_transform's FromLL service.
    """
    values = (origin_latitude, origin_longitude, origin_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Map origin coordinates/yaw must be finite')
    if not -80.0 <= origin_latitude <= 84.0 or not -180.0 <= origin_longitude <= 180.0:
        raise ValueError('Map origin is outside the supported UTM latitude/longitude range')
    if not 0 <= utm_zone <= 60:
        raise ValueError('utm_zone must be 0 (automatic) or 1..60')
    zone = utm_zone or min(60, int((origin_longitude + 180.0) // 6.0) + 1)
    hemisphere = utm_hemisphere.upper() or ('S' if origin_latitude < 0.0 else 'N')
    if hemisphere not in ('N', 'S'):
        raise ValueError('utm_hemisphere must be N, S, or empty (automatic)')
    projection = Proj(proj='utm', zone=zone, ellps='WGS84', south=hemisphere == 'S')
    origin_x, origin_y = projection(origin_longitude, origin_latitude)
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    result = []
    for waypoint in waypoints:
        x, y = projection(waypoint['longitude'], waypoint['latitude'])
        dx, dy = x - origin_x, y - origin_y
        result.append({
            'x': cosine * dx + sine * dy,
            'y': -sine * dx + cosine * dy,
            'yaw': waypoint['yaw'] - origin_yaw,
        })
    return result


def next_navigation_goal(robot_x, robot_y, waypoint, maximum_distance,
                         is_traversable=None):
    """Return a bounded goal and whether it is the actual GPS waypoint.

    Nav2 still plans and checks obstacles for each segment. An intermediate
    success must not count as reaching the GPS waypoint. Distance 0 disables
    segmentation for callers with a full-size global costmap.
    """
    if not math.isfinite(maximum_distance) or maximum_distance < 0.0:
        raise ValueError('intermediate_goal_distance must be finite and nonnegative')
    dx, dy = waypoint['x'] - robot_x, waypoint['y'] - robot_y
    distance = math.hypot(dx, dy)
    if maximum_distance == 0.0 or distance <= maximum_distance:
        return dict(waypoint), True
    # An arbitrary segmentation boundary can fall inside a known obstacle.
    # Move that boundary back until its endpoint is traversable; Nav2 still
    # plans the route around obstacles, and the actual GPS target never moves.
    step = maximum_distance
    minimum_step = min(1.0, maximum_distance/2)
    decrement = min(0.25, maximum_distance/2)
    while step >= minimum_step:
        fraction = step / distance
        goal = {'x': robot_x + dx * fraction,
                'y': robot_y + dy * fraction,
                'yaw': math.atan2(dy, dx)}
        if is_traversable is None or is_traversable(goal['x'], goal['y']):
            return goal, False
        step -= decrement
    raise ValueError('No traversable intermediate endpoint toward the GPS target')


def costmap_allows_goal(grid, x, y):
    """Check a Nav2 OccupancyGrid endpoint, including its rotated origin.

    Nav2 publishes lethal=100 and inscribed=99. Unknown=-1 is allowed by the
    mapless planner; cells outside the rolling map cannot be planned to.
    """
    info = grid.info
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    dx, dy = x-info.origin.position.x, y-info.origin.position.y
    cosine, sine = math.cos(yaw), math.sin(yaw)
    col = math.floor((cosine*dx+sine*dy)/info.resolution)
    row = math.floor((-sine*dx+cosine*dy)/info.resolution)
    if not (0 <= col < info.width and 0 <= row < info.height):
        return False
    return grid.data[row*info.width+col] < 99
