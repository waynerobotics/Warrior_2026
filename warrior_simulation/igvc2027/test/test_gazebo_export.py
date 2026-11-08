"""Grouped static scenery must preserve the shared scene's physical geometry."""

import importlib.util
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest


COURSE_DIR = Path(__file__).resolve().parents[1]
ROOT = COURSE_DIR.parents[1]
spec = importlib.util.spec_from_file_location(
    "gazebo_export", ROOT / "warrior_simulation/tools/export_igvc2027_gazebo.py")
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


def numbers(node, path):
    return [float(v) for v in node.findtext(path).split()]


@pytest.mark.parametrize("source", ["exporter", "checked_in_world"])
def test_course_uses_native_fortress_world_systems(source, tmp_path):
    """Older Fortress releases cannot load the newer gz-sim filename aliases."""
    if source == "exporter":
        scene = {"datum": {"latitude": 42.0, "longitude": -83.0}, "objects": []}
        world = exporter.export_course(
            scene, tmp_path / "course.sdf", tmp_path / "assets")
    else:
        world = ROOT / "warrior_simulation/warrior_gazebo/worlds/igvc2027_autonav.world"
    plugins = {node.attrib["filename"]: node.attrib["name"]
               for node in ET.parse(world).findall("world/plugin")}
    assert plugins == {
        "ignition-gazebo-physics-system": "ignition::gazebo::systems::Physics",
        "ignition-gazebo-user-commands-system": "ignition::gazebo::systems::UserCommands",
        "ignition-gazebo-scene-broadcaster-system": "ignition::gazebo::systems::SceneBroadcaster",
        "ignition-gazebo-contact-system": "ignition::gazebo::systems::Contact",
        "ignition-gazebo-imu-system": "ignition::gazebo::systems::Imu",
        "ignition-gazebo-navsat-system": "ignition::gazebo::systems::NavSat",
    }


@pytest.mark.parametrize("xacro", ["ros2_control/gzsim.ros2_control.xacro", "sensors/lidar.xacro"])
def test_robot_sensor_systems_use_native_fortress_plugins(xacro):
    """Robot-side sensor plugins must load on the same Fortress versions."""
    document = ET.parse(ROOT / "warrior_description/xacro" / xacro)
    systems = []
    for plugin in document.findall(".//plugin"):
        filename = plugin.get("filename")
        if filename is None or filename == "libgz_ros2_control-system.so":
            continue
        assert filename.startswith("ignition-gazebo-")
        assert plugin.attrib["name"].startswith("ignition::gazebo::systems::")
        systems.append(plugin.attrib["name"])
    assert "ignition::gazebo::systems::Sensors" in systems


def test_sloped_terrain_keeps_registered_uv_orientation_and_upward_normals(tmp_path):
    # Geographic imagery is registered to these corners; swapping U/V or
    # flipping V would silently move photographed markings across the terrain.
    mesh_path = tmp_path / "terrain.dae"
    exporter.write_collada(
        mesh_path,
        [[0, 0, 0], [4, 0, 3], [4, 2, 3], [0, 2, 0]],
        [[0, 1, 2], [0, 2, 3]],
        [[0, 0], [1, 0], [1, 1], [0, 1]],
    )
    mesh = ET.parse(mesh_path)
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}

    def values(source):
        return [float(value) for value in mesh.findtext(
            f".//c:source[@id='{source}']/c:float_array", namespaces=ns).split()]

    assert values("uv") == pytest.approx([0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1])
    assert values("normals") == pytest.approx([-.6, 0, .8] * 6)
    assert values("positions") == pytest.approx(
        [0, 0, 0, 4, 0, 3, 4, 2, 3, 0, 0, 0, 4, 2, 3, 0, 2, 0])


def test_grouped_export_preserves_every_object_pose_material_and_collision(tmp_path):
    scene = json.loads((COURSE_DIR / "scene.json").read_text())
    assets, world = tmp_path / "igvc2027_assets", tmp_path / "course.sdf"
    exporter.export_course(scene, world, assets, COURSE_DIR)
    root = ET.parse(world)
    models = root.findall("world/model")
    assert len(models) == 1, "Per-object models make interactive Gazebo startup unusable"
    assert models[0].findtext("static") == "true"
    links = models[0].findall("link")
    assert len(links) == 1, "Static decorations must share a single physics body"
    assert models[0].findtext("pose", "0 0 0 0 0 0") == "0 0 0 0 0 0"
    assert links[0].findtext("pose", "0 0 0 0 0 0") == "0 0 0 0 0 0"
    visuals = {n.attrib["name"]: n for n in links[0].findall("visual")}
    collisions = {n.attrib["name"]: n for n in links[0].findall("collision")}
    assert len(visuals) == len(scene["objects"])
    assert len(collisions) == sum(bool(o.get("collision", True)) for o in scene["objects"])
    for obj in scene["objects"]:
        identifier = re.sub(r"[^A-Za-z0-9_]", "_", obj["id"])
        visual = visuals[identifier]
        assert numbers(visual, "pose") == pytest.approx(obj["pose"], abs=1e-6)
        material = scene["materials"][obj["material"]]
        color = material.get("color", material.get("diffuse", [.7, .7, .7, 1]))
        if len(color) == 3:
            color = [*color, 1]
        assert numbers(visual, "material/diffuse") == pytest.approx(color)
        for tag in ("albedo_map", "normal_map", "roughness_map"):
            uri = visual.findtext(f"material/pbr/metal/{tag}")
            if uri:
                assert (assets / uri.split("/", 3)[-1]).is_file(), uri
        collision = collisions.get(identifier + "_collision")
        assert (collision is not None) == obj.get("collision", True), identifier
        if collision is None:
            continue
        assert numbers(collision, "pose") == pytest.approx(obj["pose"], abs=1e-6)
        kind = obj["type"]
        if kind == "box":
            assert numbers(collision, "geometry/box/size") == pytest.approx(obj["size"])
        elif kind in ("cylinder", "sphere"):
            assert float(collision.findtext(f"geometry/{kind}/radius")) == pytest.approx(obj["radius"])
            if kind == "cylinder":
                assert float(collision.findtext("geometry/cylinder/length")) == pytest.approx(
                    obj.get("height", obj.get("length")))
        elif kind == "mesh":
            uri = collision.findtext("geometry/mesh/uri")
            mesh = ET.parse(assets / uri.split("/", 3)[-1])
            ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
            positions = [float(v) for v in mesh.findtext(
                ".//c:source[@id='positions']/c:float_array", namespaces=ns).split()]
            # The export duplicates corners, but its geometric bounds must not change.
            for axis in range(3):
                expected = [vertex[axis] for vertex in obj["vertices"]]
                assert min(positions[axis::3]) == pytest.approx(min(expected), abs=1e-6)
                assert max(positions[axis::3]) == pytest.approx(max(expected), abs=1e-6)
