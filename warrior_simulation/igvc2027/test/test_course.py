"""Physical and geographic contracts for the shared AutoNav reconstruction."""

from collections import Counter, defaultdict, deque
import copy
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from pyproj import Geod
from pyproj.enums import TransformDirection
import pytest
import yaml


COURSE_DIR = Path(__file__).resolve().parents[1]
ROOT = COURSE_DIR.parents[1]
spec = importlib.util.spec_from_file_location("igvc_course", COURSE_DIR / "build_course.py")
compiler = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compiler)


@pytest.fixture(scope="module")
def config():
    return yaml.safe_load((COURSE_DIR / "course.yaml").read_text())


@pytest.fixture(scope="module")
def boundary():
    return json.loads((COURSE_DIR / "reference/p37_boundary.geojson").read_text())["geometry"]


@pytest.fixture(scope="module")
def scene(config, boundary):
    return compiler.build(config, boundary)


def inside(point, polygon):
    """Independent winding-angle check, including concave parking-lot notches."""
    x, y = point
    angle = 0.0
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        ax, ay, bx, by = a[0] - x, a[1] - y, b[0] - x, b[1] - y
        angle += math.atan2(ax * by - ay * bx, ax * bx + ay * by)
    return abs(angle) > math.pi


def object_points(obj):
    """World-space horizontal extent samples, applying the object's yaw."""
    if obj["type"] == "mesh":
        local = [v[:2] for v in obj["vertices"]]
    elif obj["type"] == "box":
        sx, sy = obj["size"][:2]
        local = [(x * sx / 2, y * sy / 2) for x in (-1, 1) for y in (-1, 1)]
    elif obj["type"] == "cylinder":
        local = [(obj["radius"] * math.cos(i * math.tau / 32),
                  obj["radius"] * math.sin(i * math.tau / 32)) for i in range(32)]
    else:
        raise AssertionError(f"Unhandled course geometry: {obj['type']}")
    x, y, _, roll, pitch, yaw = obj["pose"]
    assert roll == pitch == 0.0, "Extend containment checks before adding tilted props"
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [(x + a * cosine - b * sine, y + a * sine + b * cosine) for a, b in local]


def test_geographic_axes_match_independent_wgs84_geodesics(config):
    datum = config["datum"]
    converter = compiler.geographic_transform(datum)
    origin = (datum["longitude"], datum["latitude"], datum["elevation"])
    assert converter.transform(*origin) == pytest.approx((0, 0, 0), abs=1e-8)
    geodesic = Geod(ellps="WGS84")
    for bearing, expected in [(0, (0, 100)), (90, (100, 0)), (180, (0, -100))]:
        lon, lat, _ = geodesic.fwd(origin[0], origin[1], bearing, 100)
        # The tangent plane at altitude 286 m differs slightly from the ellipsoid.
        assert converter.transform(lon, lat, origin[2])[:2] == pytest.approx(expected, abs=0.01)


def test_non_enu_datum_heading_cannot_silently_misalign_gps(config):
    datum = dict(config["datum"], heading_deg=90)
    with pytest.raises(ValueError, match="ENU"):
        compiler.geographic_transform(datum)


@pytest.mark.parametrize("enu", [(0, 0, 0), (53, 23, 0.24), (285, 60, 2), (-10, -10, -1)])
def test_gps_roundtrip_preserves_local_geometry(config, enu):
    converter = compiler.geographic_transform(config["datum"])
    wgs84 = converter.transform(*enu, direction=TransformDirection.INVERSE)
    assert converter.transform(*wgs84) == pytest.approx(enu, abs=1e-6)


def test_course_geometry_fits_the_mapped_parking_surface(scene):
    polygon = scene["course"]["parking_boundary_enu"]
    roles = {"lane", "barrel", "pothole", "gate", "ramp"}
    for obj in scene["objects"]:
        if obj["role"] in roles:
            assert all(inside(p, polygon) for p in object_points(obj)), obj["id"]


def test_offsite_placement_is_rejected(config, boundary):
    offsite = copy.deepcopy(config)
    offsite["placement"]["center_enu"] = [500, 500]
    with pytest.raises(ValueError, match="outside.*P37"):
        compiler.build(offsite, boundary)


@pytest.mark.parametrize("role", ["barrel", "ramp", "gate"])
def test_heading_rotates_obstacles_and_ramp_with_the_course(config, boundary, scene, role):
    rotated_config = copy.deepcopy(config)
    rotated_config["placement"]["heading_deg"] += 90
    rotated = compiler.build(rotated_config, boundary)
    cx, cy = config["placement"]["center_enu"]

    def quarter_turn(point):
        return [cx - (point[1] - cy), cy + (point[0] - cx)]

    objects = {o["id"]: o for o in rotated["objects"]}
    for original in scene["objects"]:
        if original["role"] != role:
            continue
        actual = objects[original["id"]]
        if original["type"] == "mesh":
            expected_points = [quarter_turn(p) for p in object_points(original)]
            for expected, point in zip(expected_points, object_points(actual)):
                assert point == pytest.approx(expected, abs=1e-7), original["id"]
        else:
            assert actual["pose"][:2] == pytest.approx(
                quarter_turn(original["pose"][:2]), abs=1e-7), original["id"]


def test_asphalt_collision_mesh_preserves_the_concave_boundary(scene):
    ground = next(o for o in scene["objects"] if o["role"] == "ground")
    vertices = ground["vertices"]
    polygon = scene["course"]["parking_boundary_enu"]
    area = abs(sum(a[0] * b[1] - b[0] * a[1]
                   for a, b in zip(polygon, polygon[1:] + polygon[:1]))) / 2
    triangle_area = 0.0
    for triangle in ground["triangles"]:
        a, b, c = [vertices[i] for i in triangle]
        signed_area = ((b[0] - a[0]) * (c[1] - a[1]) -
                       (b[1] - a[1]) * (c[0] - a[0])) / 2
        assert signed_area > 0
        triangle_area += signed_area
        centroid = [(a[i] + b[i] + c[i]) / 3 for i in (0, 1)]
        assert inside(centroid, polygon)
    assert triangle_area == pytest.approx(area, rel=1e-10)
    assert ground["collision"]
    # Every interior edge is shared, so refinement cannot leave a T-junction.
    counts = Counter(tuple(sorted((t[i], t[(i + 1) % 3])))
                     for t in ground["triangles"] for i in range(3))
    assert set(counts.values()) <= {1, 2}
    def on_boundary(p):
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            if abs(math.dist(a, p) + math.dist(p, b) - math.dist(a, b)) < 1e-8:
                return True
        return False
    for edge, count in counts.items():
        a, b = [vertices[i][:2] for i in edge]
        assert math.dist(a, b) <= 3.0 + 1e-8
        if count == 1:
            assert on_boundary(a) and on_boundary(b), "Unshared interior terrain edge"


def test_loop_is_continuous_and_within_rule_length_and_turn_radius(scene):
    points = scene["course"]["centerline"]
    segments = list(zip(points, points[1:] + points[:1]))
    distances = [math.dist(a, b) for a, b in segments]
    assert 146.304 <= sum(distances) <= 158.496  # The diagram's 480–520 ft.
    assert min(distances) > 0
    assert max(distances) < 0.5  # Includes the seam joining the last point to first.
    for i, b in enumerate(points):
        a, c = points[i - 1], points[(i + 1) % len(points)]
        twice_area = abs((b[0] - a[0]) * (c[1] - a[1]) -
                         (b[1] - a[1]) * (c[0] - a[0]))
        if twice_area > 1e-10:
            radius = math.dist(a, b) * math.dist(b, c) * math.dist(c, a) / (2 * twice_area)
            assert radius >= 1.524  # Five feet minimum turning radius.
    # A crossing would permit shortcuts unrelated to the intended loop.
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    for i, (a, b) in enumerate(segments):
        for c, d in segments[i + 2:]:
            assert not (side(a, b, c) * side(a, b, d) < -1e-12 and
                        side(c, d, a) * side(c, d, b) < -1e-12)


def test_curbs_are_closed_solids_with_outward_winding(scene):
    for curb in (o for o in scene["objects"] if o["role"] == "curb"):
        assert curb["collision"]
        edges = Counter(tuple(sorted((face[i], face[(i + 1) % 3])))
                        for face in curb["triangles"] for i in range(3))
        assert set(edges.values()) == {2}, curb["id"]
        volume = 0.0
        for face in curb["triangles"]:
            a, b, c = [curb["vertices"][i] for i in face]
            volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) +
                       a[1] * (b[2] * c[0] - b[0] * c[2]) +
                       a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
        assert volume > 0, (curb["id"], "Inside-out collision mesh")


def test_background_is_a_conforming_surface_with_the_parking_lot_cut_out(scene):
    from pyproj import Transformer
    background = next(o for o in scene["objects"] if o["role"] == "terrain")
    parking = scene["course"]["parking_boundary_enu"]
    metadata = json.loads((COURSE_DIR / "reference/p37_oakland_2025_georeference.json").read_text())
    gps = Transformer.from_crs(3857, 4326, always_xy=True)
    converter = compiler.geographic_transform(scene["datum"])
    extent = metadata["extent"]
    outer = []
    for x, y in [("xmin", "ymin"), ("xmax", "ymin"), ("xmax", "ymax"), ("xmin", "ymax")]:
        lon, lat = gps.transform(extent[x], extent[y])
        outer.append(converter.transform(lon, lat, scene["datum"]["elevation"])[:2])
    def area(polygon):
        return abs(sum(a[0] * b[1] - b[0] * a[1]
                       for a, b in zip(polygon, polygon[1:] + polygon[:1]))) / 2
    def on_boundary(point, polygon):
        return any(abs(math.dist(a, point) + math.dist(point, b) - math.dist(a, b)) < 1e-7
                   for a, b in zip(polygon, polygon[1:] + polygon[:1]))
    vertices = background["vertices"]
    total_area = 0
    for indices in background["triangles"]:
        a, b, c = [vertices[i][:2] for i in indices]
        signed_area = ((b[0] - a[0]) * (c[1] - a[1]) -
                       (b[1] - a[1]) * (c[0] - a[0])) / 2
        assert signed_area > 0
        total_area += signed_area
        centroid = [(a[i] + b[i] + c[i]) / 3 for i in (0, 1)]
        assert inside(centroid, outer)
        assert not inside(centroid, parking), "Background must not create a second road surface"
    assert total_area == pytest.approx(area(outer) - area(parking), abs=1e-5)
    counts = Counter(tuple(sorted((face[i], face[(i + 1) % 3])))
                     for face in background["triangles"] for i in range(3))
    assert set(counts.values()) <= {1, 2}
    for edge, count in counts.items():
        if count == 1:
            points = [vertices[i][:2] for i in edge]
            points.append([(points[0][i] + points[1][i]) / 2 for i in (0, 1)])
            assert all(on_boundary(p, outer) or on_boundary(p, parking) for p in points), (
                "Unshared interior background edge", points)


def test_rule_dimensions_and_ramp_collision_are_physical(scene):
    course = scene["course"]
    assert 3.048 <= course["lane_width_m"] <= 6.096
    assert 0.0762 <= course["tape_width_m"] <= 0.1016
    ramp = next(o for o in scene["objects"] if o["role"] == "ramp")
    assert ramp["collision"]
    edges = Counter(tuple(sorted((t[i], t[(i + 1) % 3])))
                    for t in ramp["triangles"] for i in range(3))
    assert set(edges.values()) == {2}, "Ramp collision mesh must be watertight"
    slopes = []
    for t in ramp["triangles"]:
        a, b, c = [ramp["vertices"][i] for i in t]
        u, v = [[p[i] - a[i] for i in range(3)] for p in (b, c)]
        normal = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                  u[0] * v[1] - u[1] * v[0]]
        if normal[2] > 1e-8:
            slopes.append(math.hypot(*normal[:2]) / normal[2])
    assert 0 < max(slopes) <= 0.15
    for obj in scene["objects"]:
        if obj["role"] == "pothole":
            diameter = max(math.dist(a[:2], b[:2]) for a in obj["vertices"] for b in obj["vertices"])
            assert diameter == pytest.approx(0.6096)
            assert not obj["collision"], "White potholes must not become lidar walls"
    gates = {o["id"]: o for o in scene["objects"] if o["role"] == "gate"}
    def center(obj):
        return [sum(v[i] for v in obj["vertices"]) / len(obj["vertices"]) for i in (0, 1)]
    assert math.dist(center(gates["south_start"]), center(gates["speed_gate"])) == pytest.approx(13.4112)


def test_spawn_and_every_nml_target_are_connected_for_the_configured_robot(scene):
    """Grid search establishes a geometric witness, not Nav2 driving success."""
    params = yaml.safe_load((ROOT / "warrior_navigation/config/nav2_gps_sim_params.yaml").read_text())
    radius = params["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"]
    bodies = [o for o in scene["objects"] if o["role"] == "barrel" and o["collision"]]
    targets = [scene["spawn"][:2], *scene["course"]["nml_waypoints_enu"]]
    origin_x, origin_y = targets[-1]
    edges = [o for o in bodies if o["id"].startswith("nml_edge")]
    xmin, xmax = min(o["pose"][0] for o in edges), max(o["pose"][0] for o in edges)
    step = 0.05
    min_i = math.ceil((xmin + radius - origin_x) / step)
    max_i = math.floor((xmax - radius - origin_x) / step)
    max_j = math.ceil((targets[0][1] - origin_y) / step)
    free = set()
    for i in range(min_i, max_i + 1):
        for j in range(max_j + 1):
            p = (origin_x + i * step, origin_y + j * step)
            if all(math.dist(p, o["pose"][:2]) > radius + o["radius"] for o in bodies):
                free.add((i, j))
    cells = [(round((p[0] - origin_x) / step), round((p[1] - origin_y) / step)) for p in targets]
    assert all(c in free for c in cells), "Spawn or GPS target overlaps an obstacle"
    visited, queue = {cells[0]}, deque([cells[0]])
    while queue:
        i, j = queue.popleft()
        for cell in [(i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)]:
            if cell in free and cell not in visited:
                visited.add(cell)
                queue.append(cell)
    assert all(c in visited for c in cells), "A barrel row blocks the robot's no-man's-land corridor"


def test_waypoint_export_and_simulator_profile_share_datum_and_spawn(scene, tmp_path):
    converter = compiler.geographic_transform(scene["datum"])
    surface_height = terrain_surface_sampler(scene)
    expected_yaw = -math.pi / 2 + math.radians(scene["placement"]["heading_deg"])
    targets = scene["course"]["nml_waypoints_enu"]
    output = tmp_path / "targets.yaml"
    compiler.write_waypoints(scene, output, targets)
    mission = yaml.safe_load(output.read_text())["waypoints"]
    assert len(mission) == 4
    for target, gps in zip(targets, mission):
        actual = converter.transform(gps["longitude"], gps["latitude"], gps["altitude"])
        assert actual[:2] == pytest.approx(target, abs=0.001)
        assert actual[2] == pytest.approx(surface_height(target), abs=0.001)
        assert gps["yaw"] == pytest.approx(expected_yaw)
    nav = ROOT / "warrior_navigation/config"
    stored = yaml.safe_load((nav / "igvc2027_autonav_nml_waypoints.yaml").read_text())
    assert stored["waypoints"] == mission, "Regenerate the checked-in GPS mission"
    profile = yaml.safe_load((nav / "igvc2027_autonav_sim.yaml").read_text())
    assert [profile[k] for k in ["spawn_x", "spawn_y", "spawn_z", "spawn_yaw"]] == pytest.approx(
        [scene["spawn"][i] for i in [0, 1, 2, 5]])
    assert scene["spawn"][1] > targets[0][1], "Spawn must face the NML entrance"
    assert scene["spawn"][5] == pytest.approx(expected_yaw)
    assert scene["spawn"][2] == pytest.approx(surface_height(scene["spawn"]) + 0.1)
    world = ET.parse(ROOT / "warrior_simulation/warrior_gazebo/worlds" / profile["world_name"])
    datum = world.find(".//spherical_coordinates")
    assert datum.findtext("surface_model") == "EARTH_WGS84"
    assert datum.findtext("world_frame_orientation") == "ENU"
    for sdf_name, key in [("latitude_deg", "latitude"), ("longitude_deg", "longitude"),
                          ("elevation", "elevation"), ("heading_deg", "heading_deg")]:
        assert float(datum.findtext(sdf_name)) == pytest.approx(scene["datum"][key])


def test_historical_registration_preserves_the_2027_shape_and_scale(scene):
    reference = json.loads((COURSE_DIR / "reference/igvc2026_waypoints_reference.json").read_text())
    targets = scene["course"]["nml_waypoints_enu"]
    historical = [p["enu_m"][:2] for p in reference["points"] if p["group"] == "main"]
    for axis in range(2):
        assert sum(p[axis] for p in targets) / len(targets) == pytest.approx(
            sum(p[axis] for p in historical) / len(historical), abs=1e-7)
    def bearing(points):
        north, south = points[0], points[-1]
        return math.atan2(north[0] - south[0], north[1] - south[1])
    assert bearing(targets) == pytest.approx(bearing(historical), abs=1e-7)
    assert scene["course"]["centerline_length_m"] == pytest.approx(152.4)
    assert math.dist(targets[0], targets[-1]) != pytest.approx(
        math.dist(historical[0], historical[-1]), abs=1), "Do not silently copy the2026 route"


def test_long_leg_diagnostic_keeps_the_real_endpoints_beyond_lidar_range(scene):
    filename = ROOT / "warrior_navigation/config/igvc2027_autonav_long_leg.yaml"
    mission = yaml.safe_load(filename.read_text())["waypoints"]
    assert len(mission) == 2
    converter = compiler.geographic_transform(scene["datum"])
    targets = [converter.transform(p["longitude"], p["latitude"], p["altitude"])[:2]
               for p in mission]
    assert targets[0] == pytest.approx(scene["course"]["nml_waypoints_enu"][0], abs=0.001)
    assert targets[1] == pytest.approx(scene["course"]["nml_waypoints_enu"][-1], abs=0.001)
    assert math.dist(*targets) > 30.0


def test_site_height_and_uv_agree_with_independent_raster_coordinates(config):
    from pyproj import Transformer
    converter = compiler.geographic_transform(config["datum"])
    site = compiler.SiteModel(COURSE_DIR / "reference", converter)
    mercator_to_gps = Transformer.from_crs(3857, 4326, always_xy=True)
    def enu(east, north):
        lon, lat = mercator_to_gps.transform(east, north)
        return converter.transform(lon, lat, config["datum"]["elevation"])[:2]
    extent = site.dem["extent"]
    # Exact source pixel centers catch flipped row order and half-pixel shifts.
    for row, column in [(20, 20), (40, 85), (110, 250), (180, 430)]:
        east = extent["xmin"] + (column + .5) / site.dem["width"] * (extent["xmax"] - extent["xmin"])
        north = extent["ymax"] - (row + .5) / site.dem["height"] * (extent["ymax"] - extent["ymin"])
        expected = site.dem["elevations_m"][row][column] - site.dem["reference_elevation_m"]
        assert site.height(*enu(east, north)) == pytest.approx(expected, abs=1e-5)
    aerial = site.aerial["extent"]
    for x_key, y_key, expected in [("xmin", "ymin", [0, 0]), ("xmax", "ymin", [1, 0]),
                                    ("xmax", "ymax", [1, 1]), ("xmin", "ymax", [0, 1])]:
        assert site.uv(*enu(aerial[x_key], aerial[y_key])) == pytest.approx(expected, abs=1e-7)


def terrain_surface_sampler(scene):
    """Independent barycentric interpolation of the exported collision surface."""
    ground = next(o for o in scene["objects"] if o["role"] == "ground")
    cells = defaultdict(list)
    for indices in ground["triangles"]:
        triangle = [ground["vertices"][i] for i in indices]
        for x in range(math.floor(min(p[0] for p in triangle) / 3),
                       math.floor(max(p[0] for p in triangle) / 3) + 1):
            for y in range(math.floor(min(p[1] for p in triangle) / 3),
                           math.floor(max(p[1] for p in triangle) / 3) + 1):
                cells[x, y].append(triangle)

    def surface(point):
        x, y = point[:2]
        for a, b, c in cells[math.floor(x / 3), math.floor(y / 3)]:
            ab, ac = [b[i] - a[i] for i in (0, 1)], [c[i] - a[i] for i in (0, 1)]
            denominator = ab[0] * ac[1] - ab[1] * ac[0]
            u = ((x - a[0]) * ac[1] - (y - a[1]) * ac[0]) / denominator
            v = (ab[0] * (y - a[1]) - ab[1] * (x - a[0])) / denominator
            if min(u, v) >= -1e-8 and u + v <= 1 + 1e-8:
                return a[2] + u * (b[2] - a[2]) + v * (c[2] - a[2])
        pytest.fail("Sample is outside the collidable asphalt surface")
    return surface


def test_painted_lines_remain_above_the_actual_ground_mesh(scene):
    """Raw DEM samples alone can bury paint under coarser collision triangles."""
    surface = terrain_surface_sampler(scene)
    for obj in scene["objects"]:
        if obj["role"] not in ("lane", "gate", "pothole"):
            continue
        vertices = obj["vertices"]
        samples = vertices + [[sum(vertices[i][axis] for i in triangle) / 3 for axis in range(3)]
                              for triangle in obj["triangles"]]
        cosine, sine = math.cos(obj["pose"][5]), math.sin(obj["pose"][5])
        for x, y, z in samples:
            point = [obj["pose"][0] + cosine * x - sine * y,
                     obj["pose"][1] + sine * x + cosine * y,
                     obj["pose"][2] + z]
            gap = point[2] - surface(point)
            assert 0.001 <= gap <= 0.03, (obj["id"], point, gap)
