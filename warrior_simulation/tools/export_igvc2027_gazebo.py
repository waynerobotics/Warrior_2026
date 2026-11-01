#!/usr/bin/env python3
"""Export the shared IGVC scene to a self-contained Gazebo SDF world.

All scene positions are ENU metres, rotations are radians, and primitive poses
refer to their centres. Paint is visual geometry only. Generated UV meshes keep
asphalt and ramp textures at a physical repeat scale instead of stretching one
image across the whole parking lot.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WORLD = ROOT / "warrior_simulation/warrior_gazebo/worlds/igvc2027_autonav.world"
DEFAULT_ASSETS = ROOT / "warrior_simulation/warrior_gazebo/models/igvc2027_assets"
DEFAULT_SCENE = ROOT / "warrior_simulation/igvc2027/scene.json"


def number(value):
    return f"{float(value):.9g}"


def numbers(values):
    return " ".join(number(value) for value in values)


def element(parent, tag, value=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if value is not None:
        node.text = str(value)
    return node


def safe_name(value):
    return re.sub(r"[^A-Za-z0-9_]", "_", str(value))


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write("\n")


def mesh_triangles(vertices, faces):
    """Triangulate convex polygon faces without changing their winding."""
    triangles = []
    for face in faces:
        for index in range(1, len(face) - 1):
            triangles.append((face[0], face[index], face[index + 1]))
    for triangle in triangles:
        if any(index < 0 or index >= len(vertices) for index in triangle):
            raise ValueError("Mesh face index is outside the vertex array")
    return triangles


def write_collada(path, vertices, faces, uvs=None):
    """Write portable COLLADA 1.4.1 with explicit normals and texture coordinates."""
    triangles = mesh_triangles(vertices, faces)
    # Duplicate face corners to retain sharp edges and give every face valid UVs.
    points, normals, texcoords = [], [], []
    for triangle in triangles:
        a, b, c = (vertices[index] for index in triangle)
        ab, ac = ([b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)])
        normal = [ab[1] * ac[2] - ab[2] * ac[1],
                  ab[2] * ac[0] - ab[0] * ac[2],
                  ab[0] * ac[1] - ab[1] * ac[0]]
        norm = math.sqrt(sum(value * value for value in normal))
        if norm <= 1e-12:
            continue
        normal = [value / norm for value in normal]
        for index in triangle:
            points.extend(vertices[index])
            normals.extend(normal)
            texcoords.extend(uvs[index] if uvs is not None else vertices[index][:2])
    collada = ET.Element("COLLADA", {
        "xmlns": "http://www.collada.org/2005/11/COLLADASchema", "version": "1.4.1"})
    asset = element(collada, "asset")
    contributor = element(asset, "contributor")
    element(contributor, "authoring_tool", "Warrior IGVC shared scene exporter")
    element(asset, "created", "2026-10-31T00:00:00Z")
    element(asset, "modified", "2026-10-31T00:00:00Z")
    element(asset, "unit", name="metre", meter="1")
    element(asset, "up_axis", "Z_UP")
    geometry = element(element(collada, "library_geometries"), "geometry", id="surface", name="surface")
    mesh = element(geometry, "mesh")
    for source_name, values, stride, labels in (
            ("positions", points, 3, ("X", "Y", "Z")),
            ("normals", normals, 3, ("X", "Y", "Z")),
            ("uv", texcoords, 2, ("S", "T"))):
        source = element(mesh, "source", id=source_name)
        element(source, "float_array", numbers(values), id=source_name + "_array", count=str(len(values)))
        accessor = element(element(source, "technique_common"), "accessor",
                           source="#" + source_name + "_array", count=str(len(values) // stride), stride=str(stride))
        for label in labels:
            element(accessor, "param", name=label, type="float")
    element(element(mesh, "vertices", id="vertices"), "input", semantic="POSITION", source="#positions")
    triangle_node = element(mesh, "triangles", count=str(len(points) // 9))
    element(triangle_node, "input", semantic="VERTEX", source="#vertices", offset="0")
    element(triangle_node, "input", semantic="NORMAL", source="#normals", offset="1")
    element(triangle_node, "input", semantic="TEXCOORD", source="#uv", offset="2", set="0")
    element(triangle_node, "p", " ".join(str(index) for index in range(len(points) // 3) for _ in range(3)))
    scene = element(element(collada, "library_visual_scenes"), "visual_scene", id="Scene")
    element(element(scene, "node", id="surface_node", name="surface"), "instance_geometry", url="#surface")
    element(element(collada, "scene"), "instance_visual_scene", url="#Scene")
    write_xml(path, collada)


def ribbon_mesh(points, width, closed=False):
    """Build joined ribbon vertices, clamping mitres at sharp corners."""
    points = [list(point) + [0.006] if len(point) == 2 else list(point) for point in points]
    if closed and points[0] == points[-1]:
        points.pop()
    if len(points) < 2:
        raise ValueError("A painted polyline requires at least two points")
    vertices, uvs, distance = [], [], 0.0
    for index, point in enumerate(points):
        before = points[index - 1] if index or closed else points[index]
        after = points[(index + 1) % len(points)] if index + 1 < len(points) or closed else points[index]
        directions = []
        for a, b in ((before, point), (point, after)):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length > 1e-9:
                directions.append((-dy / length, dx / length))
        nx, ny = (sum(values) for values in zip(*directions))
        length = math.hypot(nx, ny)
        nx, ny = nx / length, ny / length
        dot = nx * directions[-1][0] + ny * directions[-1][1]
        half = min(width / (2 * max(dot, 0.25)), width * 2)
        vertices.extend([[point[0] + nx * half, point[1] + ny * half, point[2]],
                         [point[0] - nx * half, point[1] - ny * half, point[2]]])
        if index:
            distance += math.dist(points[index - 1][:2], point[:2])
        uvs.extend([[distance, 0], [distance, 1]])
    faces = []
    for index in range(len(points) if closed else len(points) - 1):
        other = (index + 1) % len(points)
        faces.append([2 * index, 2 * index + 1, 2 * other + 1, 2 * other])
    return vertices, faces, uvs


def material_element(visual, material, asset_dir, source_dir):
    node = element(visual, "material")
    color = material.get("color", material.get("diffuse", [0.7, 0.7, 0.7, 1]))
    if len(color) == 3:
        color = list(color) + [1]
    element(node, "ambient", numbers(color))
    element(node, "diffuse", numbers(color))
    element(node, "specular", "0.12 0.12 0.12 1")
    metal = element(element(node, "pbr"), "metal")
    element(metal, "roughness", number(material.get("roughness", 0.8)))
    element(metal, "metalness", number(material.get("metalness", material.get("metallic", 0))))
    for tag, aliases in (("albedo_map", ("albedo_map", "albedo", "texture")),
                         ("normal_map", ("normal_map", "normal")),
                         ("roughness_map", ("roughness_map",))):
        filename = next((material[key] for key in aliases if material.get(key)), None)
        if not filename:
            continue
        source = Path(filename)
        if not source.is_absolute():
            source = source_dir / source
        if not source.is_file():
            raise FileNotFoundError(f"Missing {tag} texture: {source}")
        target = asset_dir / "textures" / source.name
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.resolve() != target.resolve():
            shutil.copyfile(source, target)
        attrs = {"type": "tangent"} if tag == "normal_map" else {}
        element(metal, tag, f"model://{asset_dir.name}/textures/{source.name}", **attrs)


def box_surface(size, tile_m):
    """Top plane for large textured ground slabs with explicit physical UV scale."""
    x, y, z = (value / 2 for value in size)
    vertices = [[-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]]
    return vertices, [[0, 1, 2, 3]], [[v[0] / tile_m, v[1] / tile_m] for v in vertices]


def geometry_element(parent, obj, asset_dir, visual=False):
    geometry = element(parent, "geometry")
    kind = obj.get("type", obj.get("kind"))
    if kind == "box" and not (visual and obj.get("uv_scale_m")):
        element(element(geometry, "box"), "size", numbers(obj["size"]))
    elif kind in ("cylinder", "sphere"):
        primitive = element(geometry, kind)
        element(primitive, "radius", number(obj["radius"]))
        if kind == "cylinder":
            element(primitive, "length", number(obj.get("length", obj.get("height"))))
    elif kind in ("mesh", "polyline", "box"):
        if kind == "polyline":
            vertices, faces, uvs = ribbon_mesh(obj["points"], obj["width"], obj.get("closed", False))
        elif kind == "box":
            vertices, faces, uvs = box_surface(obj["size"], obj["uv_scale_m"])
        else:
            vertices, faces = obj["vertices"], obj.get("faces", obj.get("triangles"))
            uvs = obj.get("uvs")
            if uvs is None:
                scale = obj.get("uv_scale_m", 1)
                uvs = [[v[0] / scale, v[1] / scale] for v in vertices]
        filename = safe_name(obj["id"]) + ".dae"
        write_collada(asset_dir / "meshes" / filename, vertices, faces, uvs)
        element(element(geometry, "mesh"), "uri", f"model://{asset_dir.name}/meshes/{filename}")
    else:
        raise ValueError(f"Unsupported scene object type: {kind!r}")


def add_object(link, obj, materials, asset_dir, source_dir):
    # Static scenery shares one physics body. Individually posed visuals and
    # collisions preserve the source geometry without a model/body per prop.
    name = safe_name(obj["id"])
    pose = obj.get("pose", [0, 0, 0, 0, 0, 0])
    if len(pose) == 3:
        pose = list(pose) + [0, 0, 0]
    material_ref = obj.get("material", "default")
    material = materials.get(material_ref, {}) if isinstance(material_ref, str) else material_ref
    visual = element(link, "visual", name=name)
    element(visual, "pose", numbers(pose))
    # Decorative foliage, barrel bands and tents still cast shadows. Ground
    # paint does not need shadows even though it sits just above the asphalt.
    cast_shadows = obj.get("role") not in ("lane", "parking_paint", "gate", "pothole")
    element(visual, "cast_shadows", str(obj.get("cast_shadows", cast_shadows)).lower())
    geometry_element(visual, obj, asset_dir, visual=True)
    material_element(visual, material, asset_dir, source_dir)
    if obj.get("collision", True):
        collision = element(link, "collision", name=name + "_collision")
        element(collision, "pose", numbers(pose))
        geometry_element(collision, obj, asset_dir)
        surface = element(collision, "surface")
        friction = element(element(surface, "friction"), "ode")
        element(friction, "mu", "1.0")
        element(friction, "mu2", "1.0")


def add_gui(world, scene):
    """Start Fortress over the course while retaining normal GUI controls."""
    cameras = scene.get("cameras", [])
    overview = next((camera for camera in cameras if camera.get("id") == "Overview"),
                    cameras[0] if cameras else {"position": [45, -50, 55], "target": [0, 0, 0]})
    position, target = overview["position"], overview["target"]
    direction = [b - a for a, b in zip(position, target)]
    yaw = math.atan2(direction[1], direction[0])
    pitch = math.atan2(-direction[2], math.hypot(direction[0], direction[1]))
    gui = element(world, "gui", fullscreen="false")

    def plugin(filename, name, docked=False):
        result = element(gui, "plugin", filename=filename, name=name)
        settings = element(result, "ignition-gui")
        element(settings, "title", name)
        element(settings, "property", "false", type="bool", key="showTitleBar")
        element(settings, "property", "docked" if docked else "floating",
                type="string", key="state")
        return result, settings

    view, _ = plugin("MinimalScene", "3D View", docked=True)
    element(view, "engine", "ogre2")
    element(view, "scene", "scene")
    element(view, "ambient_light", "0.55 0.58 0.63")
    element(view, "background_color", "0.66 0.78 0.9")
    element(view, "camera_pose", numbers([*position, 0, pitch, yaw]))
    for filename, name in (("EntityContextMenuPlugin", "Entity context menu"),
                           ("GzSceneManager", "Scene Manager"),
                           ("InteractiveViewControl", "Interactive view control"),
                           ("CameraTracking", "Camera Tracking"),
                           ("MarkerManager", "Marker manager"),
                           ("SelectEntities", "Select Entities")):
        _, settings = plugin(filename, name)
        element(settings, "property", "false", type="bool", key="resizable")
        element(settings, "property", "5", type="double", key="width")
        element(settings, "property", "5", type="double", key="height")
    for filename, name, width, height, anchor, flags in (
            ("WorldControl", "World control", 121, 72, "left",
             ("play_pause", "step", "start_paused", "use_event")),
            ("WorldStats", "World stats", 290, 110, "right",
             ("sim_time", "real_time", "real_time_factor", "iterations"))):
        item, settings = plugin(filename, name)
        element(settings, "property", "false", type="bool", key="resizable")
        element(settings, "property", str(width), type="double", key="width")
        element(settings, "property", str(height), type="double", key="height")
        element(settings, "property", "1", type="double", key="z")
        anchors = element(settings, "anchors", target="3D View")
        element(anchors, "line", own=anchor, target=anchor)
        element(anchors, "line", own="bottom", target="bottom")
        for flag in flags:
            element(item, flag, "true")
    plugin("EntityTree", "Entity tree", docked=True)


def export_course(scene, output_world=DEFAULT_WORLD, asset_dir=DEFAULT_ASSETS, source_dir=ROOT):
    """Export shared scene dictionary and return the generated world path."""
    output_world, asset_dir, source_dir = map(Path, (output_world, asset_dir, source_dir))
    sdf = ET.Element("sdf", {"version": "1.7"})
    world = element(sdf, "world", name=scene.get("name", "igvc2027_autonav"))
    world.append(ET.Comment(
        " Generated from the versioned shared IGVC course definition. "
        "Synthetic 2027 training layout; geographic anchor is not surveyed waypoints. "))
    add_gui(world, scene)
    datum = scene["datum"]
    spherical = element(world, "spherical_coordinates")
    element(spherical, "surface_model", "EARTH_WGS84")
    element(spherical, "world_frame_orientation", "ENU")
    for tag, keys, default in (("latitude_deg", ("latitude", "latitude_deg"), None),
                               ("longitude_deg", ("longitude", "longitude_deg"), None),
                               ("elevation", ("elevation", "altitude"), 0),
                               ("heading_deg", ("heading", "heading_deg"), 0)):
        value = next((datum[key] for key in keys if key in datum), default)
        element(spherical, tag, number(value))
    physics = element(world, "physics", name="1ms", type="ode")
    element(physics, "max_step_size", "0.001")
    element(physics, "real_time_factor", "1.0")
    for filename, classname in (("gz-sim-physics-system", "gz::sim::systems::Physics"),
                                 ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
                                 ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
                                 ("gz-sim-contact-system", "gz::sim::systems::Contact"),
                                 ("gz-sim-imu-system", "gz::sim::systems::Imu"),
                                 ("ignition-gazebo-navsat-system", "ignition::gazebo::systems::NavSat")):
        element(world, "plugin", filename=filename, name=classname)
    scene_element = element(world, "scene")
    element(scene_element, "ambient", "0.55 0.58 0.63 1")
    element(scene_element, "background", "0.66 0.78 0.9 1")
    element(scene_element, "shadows", "true")
    element(scene_element, "grid", "false")
    sun = element(world, "light", name="late_spring_sun", type="directional")
    element(sun, "cast_shadows", "true")
    element(sun, "pose", "0 0 100 0 0 0")
    element(sun, "diffuse", "0.9 0.86 0.78 1")
    element(sun, "specular", "0.25 0.25 0.25 1")
    element(sun, "direction", "-0.42 0.35 -0.83")
    attenuation = element(sun, "attenuation")
    element(attenuation, "range", "1000")
    element(attenuation, "constant", "1")
    element(attenuation, "linear", "0")
    element(attenuation, "quadratic", "0")
    materials = scene.get("materials", {})
    objects = scene["objects"]
    names = [safe_name(obj["id"]) for obj in objects]
    if len(names) != len(set(names)):
        raise ValueError("Scene object names must be unique after SDF sanitization")
    course_model = element(world, "model", name="igvc2027_course")
    element(course_model, "static", "true")
    course_link = element(course_model, "link", name="course")
    for obj in objects:
        add_object(course_link, obj, materials, asset_dir, source_dir)
    # This directory contains only exporter-owned geometry. Remove meshes from
    # earlier course revisions so a rebuild packages exactly the current scene.
    mesh_names = {safe_name(obj["id"]) + ".dae" for obj in objects
                  if obj["type"] in ("mesh", "polyline")
                  or (obj["type"] == "box" and obj.get("uv_scale_m"))}
    for old_mesh in (asset_dir / "meshes").glob("*.dae"):
        if old_mesh.name not in mesh_names:
            old_mesh.unlink()
    config = ET.Element("model")
    element(config, "name", "IGVC 2027 shared generated assets")
    element(config, "version", "1.0")
    element(config, "sdf", "model.sdf", version="1.7")
    element(config, "description", "Generated PBR materials and UV meshes for the Warrior 2027 training course.")
    write_xml(asset_dir / "model.config", config)
    assets_sdf = ET.Element("sdf", {"version": "1.7"})
    model = element(assets_sdf, "model", name=asset_dir.name)
    element(model, "static", "true")
    element(model, "link", name="asset_container")
    write_xml(asset_dir / "model.sdf", assets_sdf)
    write_xml(output_world, sdf)
    return output_world


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path, nargs="?", default=DEFAULT_SCENE,
                        help="Expanded shared scene JSON")
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--assets", type=Path, default=DEFAULT_ASSETS)
    arguments = parser.parse_args()
    scene = json.loads(arguments.scene.read_text(encoding="utf-8"))
    world = export_course(scene, arguments.world, arguments.assets, arguments.scene.parent)
    print(f"Exported {len(scene['objects'])} objects to {world}")


if __name__ == "__main__":
    main()
