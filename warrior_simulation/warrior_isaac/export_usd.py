#!/usr/bin/env python3
"""Export the shared IGVC geometry to portable OpenUSD without simulator imports.

Geometry, collision surfaces and material textures are all local assets. The
output can be opened directly in Isaac Sim; no Nucleus asset server is needed.
"""

import argparse
import json
import math
import os
from pathlib import Path
import re


def _name(value):
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return result if result and not result[0].isdigit() else "_" + result


def _number(value):
    if not math.isfinite(float(value)):
        raise ValueError(f"Non-finite USD value: {value}")
    return format(float(value), ".9g")


def _tuple(values):
    return "(" + ", ".join(_number(value) for value in values) + ")"


def _array(values):
    return "[" + ", ".join(_tuple(value) for value in values) + "]"


def _box(size):
    x, y, z = [float(value) / 2 for value in size]
    # Counterclockwise when viewed from outside. Separate face vertices give
    # hard surface normals and independent texture coordinates on every face.
    faces = [
        [(-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)],
        [(-x, y, -z), (x, y, -z), (x, -y, -z), (-x, -y, -z)],
        [(x, -y, -z), (x, y, -z), (x, y, z), (x, -y, z)],
        [(-x, y, -z), (-x, -y, -z), (-x, -y, z), (-x, y, z)],
        [(-x, y, z), (x, y, z), (x, y, -z), (-x, y, -z)],
        [(-x, -y, -z), (x, -y, -z), (x, -y, z), (-x, -y, z)],
    ]
    points = [point for face in faces for point in face]
    indices = [[index, index + 1, index + 2, index + 3]
               for index in range(0, 24, 4)]
    uvs = [(0, 0), (1, 0), (1, 1), (0, 1)] * 6
    return points, indices, uvs


def _cylinder(radius, length, sides=32):
    points, faces, uvs = [], [], []
    for index in range(sides):
        a, b = 2 * math.pi * index / sides, 2 * math.pi * (index + 1) / sides
        start = len(points)
        points.extend([(radius * math.cos(a), radius * math.sin(a), -length / 2),
                       (radius * math.cos(b), radius * math.sin(b), -length / 2),
                       (radius * math.cos(b), radius * math.sin(b), length / 2),
                       (radius * math.cos(a), radius * math.sin(a), length / 2)])
        faces.append(list(range(start, start + 4)))
        uvs.extend([(index / sides, 0), ((index + 1) / sides, 0),
                    ((index + 1) / sides, 1), (index / sides, 1)])
    for height, order in [(-length / 2, -1), (length / 2, 1)]:
        start = len(points)
        for index in range(sides):
            angle = order * 2 * math.pi * index / sides
            points.append((radius * math.cos(angle), radius * math.sin(angle), height))
            uvs.append((0.5 + 0.5 * math.cos(angle), 0.5 + 0.5 * math.sin(angle)))
        faces.append(list(range(start, start + sides)))
    return points, faces, uvs


def _sphere(radius, segments=24, rings=12):
    points, faces, uvs = [], [], []
    # One vertex at each pole avoids zero-area triangles.
    points.append((0, 0, radius))
    uvs.append((0.5, 0))
    for ring in range(1, rings):
        theta = math.pi * ring / rings
        for segment in range(segments + 1):
            phi = 2 * math.pi * segment / segments
            points.append((radius * math.sin(theta) * math.cos(phi),
                           radius * math.sin(theta) * math.sin(phi),
                           radius * math.cos(theta)))
            uvs.append((segment / segments, ring / rings))
    south = len(points)
    points.append((0, 0, -radius))
    uvs.append((0.5, 1))
    for segment in range(segments):
        faces.append([0, 1 + segment, 2 + segment])
        for ring in range(rings - 2):
            a = 1 + ring * (segments + 1) + segment
            b = a + segments + 1
            faces.append([a, b, b + 1, a + 1])
        a = 1 + (rings - 2) * (segments + 1) + segment
        faces.append([a + 1, a, south])
    return points, faces, uvs


def _geometry(obj):
    kind = obj["type"]
    if kind == "box":
        return _box(obj["size"])
    if kind == "cylinder":
        return _cylinder(obj["radius"], obj["length"])
    if kind == "sphere":
        return _sphere(obj["radius"])
    if kind == "mesh":
        vertices = obj["vertices"]
        return vertices, obj["triangles"], obj.get("uvs", [(0, 0)] * len(vertices))
    raise ValueError(f"Unsupported geometry type {kind!r}")


def _normal(a, b, c):
    u = [b[i] - a[i] for i in range(3)]
    v = [c[i] - a[i] for i in range(3)]
    n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
         u[0] * v[1] - u[1] * v[0]]
    length = math.sqrt(sum(value * value for value in n))
    if length <= 1e-12:
        raise ValueError("Degenerate mesh face")
    return tuple(value / length for value in n)


def _camera_matrix(position, target):
    # USD cameras look down their local -Z axis; rows contain local axes in
    # world coordinates, with translation in row 4 (USD row-vector convention).
    z = [position[i] - target[i] for i in range(3)]
    length = math.sqrt(sum(value * value for value in z))
    if length == 0:
        raise ValueError("Camera position and target must differ")
    z = [value / length for value in z]
    up = [0, 0, 1] if abs(z[2]) < 0.999 else [0, 1, 0]
    x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2],
         up[0] * z[1] - up[1] * z[0]]
    length = math.sqrt(sum(value * value for value in x))
    x = [value / length for value in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2],
         z[0] * x[1] - z[1] * x[0]]
    return "(" + ", ".join(_tuple(row) for row in [x + [0], y + [0], z + [0], list(position) + [1]]) + ")"


def _materials(materials, output, asset_root):
    lines = ['    def Scope "Looks"', '    {']
    for material_id, material in materials.items():
        name = _name(material_id)
        base = f"/World/Looks/{name}"
        color = material.get("color", [0.5, 0.5, 0.5, 1])
        lines.extend([
            f'        def Material "{name}"', '        {',
            f'            token outputs:surface.connect = <{base}/Surface.outputs:surface>',
            '            def Shader "Surface"', '            {',
            '                uniform token info:id = "UsdPreviewSurface"',
            f'                color3f inputs:diffuseColor = {_tuple(color[:3])}',
            f'                float inputs:roughness = {_number(material.get("roughness", 0.75))}',
            f'                float inputs:metallic = {_number(material.get("metallic", 0))}',
            f'                float inputs:opacity = {_number(color[3] if len(color) > 3 else 1)}',
        ])
        textures = []
        for key, shader_name, attribute, dtype, channel in [
            ("albedo", "Albedo", "diffuseColor", "color3f", "rgb"),
            ("normal", "Normal", "normal", "normal3f", "rgb"),
            ("roughness_map", "Roughness", "roughness", "float", "r"),
        ]:
            if material.get(key):
                textures.append((key, shader_name))
                lines.append(f'                {dtype} inputs:{attribute}.connect = <{base}/{shader_name}.outputs:{channel}>')
        lines.extend(['                token outputs:surface', '            }'])
        if textures:
            lines.extend([
                '            def Shader "UV"', '            {',
                '                uniform token info:id = "UsdPrimvarReader_float2"',
                '                string inputs:varname = "st"',
                '                float2 outputs:result', '            }',
            ])
        for key, shader_name in textures:
            texture_path = (asset_root / material[key]).resolve()
            if not texture_path.is_file():
                raise FileNotFoundError(f"Material {material_id}: missing texture {texture_path}")
            relative = Path(os.path.relpath(texture_path, output.parent)).as_posix()
            if "@" in relative:
                raise ValueError("USD texture path cannot contain @")
            lines.extend([
                f'            def Shader "{shader_name}"', '            {',
                '                uniform token info:id = "UsdUVTexture"',
                f'                asset inputs:file = @{relative}@',
                f'                token inputs:sourceColorSpace = "{"sRGB" if key == "albedo" else "raw"}"',
                '                token inputs:wrapS = "repeat"',
                '                token inputs:wrapT = "repeat"',
                f'                float2 inputs:st.connect = <{base}/UV.outputs:result>',
            ])
            if key == "normal":
                lines.extend(['                float4 inputs:scale = (2, 2, 2, 1)',
                              '                float4 inputs:bias = (-1, -1, -1, 0)'])
            lines.extend(['                float3 outputs:rgb',
                          '                float outputs:r', '            }'])
        lines.append('        }')
    lines.extend([
        '        def Material "SurfacePhysics" (prepend apiSchemas = ["PhysicsMaterialAPI"])',
        '        {',
        '            float physics:staticFriction = 0.85',
        '            float physics:dynamicFriction = 0.75',
        '            float physics:restitution = 0',
        '        }', '    }',
    ])
    return lines


def export_usda(scene, output, asset_root):
    """Write a meter-scale Z-up stage using schema_version 1 shared geometry."""
    if scene.get("schema_version") != 1:
        raise ValueError("Expected course schema_version 1")
    output, asset_root = Path(output).resolve(), Path(asset_root).resolve()
    datum = scene["datum"]
    materials = scene["materials"]
    object_names = [_name(obj["id"]) for obj in scene["objects"]]
    if len(set(object_names)) != len(object_names):
        raise ValueError("Object identifiers collide after USD name sanitization")
    material_names = [_name(name) for name in materials]
    if len(set(material_names + ["SurfacePhysics"])) != len(material_names) + 1:
        raise ValueError("Material identifiers collide after USD name sanitization")
    lines = [
        '#usda 1.0', '# Generated from igvc2027/scene.json; do not edit by hand.',
        '# Competition-inspired simulation approximation; GPS datum is not a surveyed course.',
        '(', '    defaultPrim = "World"', '    metersPerUnit = 1', '    upAxis = "Z"',
        '    timeCodesPerSecond = 60', '    framesPerSecond = 60',
        '    customLayerData = {',
        f'        double latitude = {_number(datum["latitude"])}',
        f'        double longitude = {_number(datum["longitude"])}',
        f'        double elevation = {_number(datum["elevation"])}',
        f'        double heading_deg = {_number(datum.get("heading_deg", 0))}',
        '        string georeference_status = "approximate venue anchor; not surveyed"',
        '    }', ')', '', 'def Xform "World"', '{',
        '    def PhysicsScene "PhysicsScene"', '    {',
        '        vector3f physics:gravityDirection = (0, 0, -1)',
        '        float physics:gravityMagnitude = 9.80665', '    }',
    ]
    lines.extend(_materials(materials, output, asset_root))
    lines.extend(['    def Scope "Course"', '    {'])
    for obj in scene["objects"]:
        if obj["material"] not in materials:
            raise ValueError(f'Unknown material {obj["material"]!r} on {obj["id"]}')
        points, faces, uvs = _geometry(obj)
        if len(uvs) != len(points):
            raise ValueError(f'{obj["id"]}: expected one UV per vertex')
        scale = obj.get("uv_scale", [1, 1])
        uvs = [(uv[0] * scale[0], uv[1] * scale[1]) for uv in uvs]
        normals = []
        for face_index, face in enumerate(faces):
            if len(face) < 3 or any(i < 0 or i >= len(points) for i in face):
                raise ValueError(f'{obj["id"]}: invalid mesh face')
            try:
                normal = _normal(*(points[i] for i in face[:3]))
            except ValueError as error:
                raise ValueError(f'{obj["id"]} face {face_index}: {error}') from error
            if obj["type"] == "sphere":
                normals.extend([tuple(value / obj["radius"] for value in points[i]) for i in face])
            elif obj["type"] == "cylinder" and face_index < len(faces) - 2:
                normals.extend([(points[i][0] / obj["radius"], points[i][1] / obj["radius"], 0)
                                for i in face])
            else:
                normals.extend([normal] * len(face))
        indices = [i for face in faces for i in face]
        bounds = [[min(point[i] for point in points) for i in range(3)],
                  [max(point[i] for point in points) for i in range(3)]]
        collision = obj.get("collision", False)
        schemas = '["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]' if collision else '["MaterialBindingAPI"]'
        pose = obj.get("pose", [0, 0, 0, 0, 0, 0])
        lines.extend([
            f'        def Mesh "{_name(obj["id"])}" (prepend apiSchemas = {schemas})',
            '        {',
            f'            custom string course:role = {json.dumps(obj.get("role", ""))}',
            f'            custom string course:sourceId = {json.dumps(obj["id"])}',
            f'            point3f[] points = {_array(points)}',
            f'            int[] faceVertexCounts = {json.dumps([len(face) for face in faces])}',
            f'            int[] faceVertexIndices = {json.dumps(indices)}',
            f'            normal3f[] normals = {_array(normals)} (interpolation = "faceVarying")',
            f'            texCoord2f[] primvars:st = {_array(uvs)} (interpolation = "vertex")',
            f'            float3[] extent = {_array(bounds)}',
            '            uniform token subdivisionScheme = "none"',
            '            uniform token orientation = "rightHanded"',
            '            bool doubleSided = false',
            f'            double3 xformOp:translate = {_tuple(pose[:3])}',
            f'            float3 xformOp:rotateXYZ = {_tuple([math.degrees(a) for a in pose[3:]])}',
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]',
            f'            rel material:binding = </World/Looks/{_name(obj["material"])}>',
        ])
        if collision:
            lines.extend([
                '            bool physics:collisionEnabled = true',
                '            token physics:approximation = "none"',
                '            rel material:binding:physics = </World/Looks/SurfacePhysics>',
            ])
        lines.append('        }')
    lines.append('    }')
    spawn = scene.get("spawn", [0, 0, 0, 0, 0, 0])
    environment = scene.get("environment", {})
    sky_texture = environment.get("sky_texture")
    sky_lines = []
    if sky_texture:
        sky_path = (asset_root / sky_texture).resolve()
        if not sky_path.is_file():
            raise FileNotFoundError(f"Missing sky texture: {sky_path}")
        relative = Path(os.path.relpath(sky_path, output.parent)).as_posix()
        if "@" in relative:
            raise ValueError("USD sky texture path cannot contain @")
        sky_lines = [f'            asset inputs:texture:file = @{relative}@',
                     '            token inputs:texture:format = "latlong"',
                     '            bool inputs:enableColorTemperature = false']
    lines.extend([
        '    def Xform "RobotSpawn"', '    {',
        f'        double3 xformOp:translate = {_tuple(spawn[:3])}',
        f'        float3 xformOp:rotateXYZ = {_tuple([math.degrees(a) for a in spawn[3:]])}',
        '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]',
        '    }',
        '    def Scope "Lighting"', '    {',
        '        def DistantLight "Sun"', '        {',
        '            color3f inputs:color = (1, 0.94, 0.83)',
        f'            float inputs:intensity = {_number(environment.get("sun_intensity", 1000 if sky_texture else 3000))}',
        '            float inputs:angle = 0.53',
        '            float3 xformOp:rotateXYZ = (25, -35, -30)',
        '            uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]',
        '        }',
        '        def DomeLight "Sky"', '        {',
        '            color3f inputs:color = (1, 1, 1)' if sky_texture else '            color3f inputs:color = (0.68, 0.80, 1)',
        f'            float inputs:intensity = {_number(environment.get("sky_intensity", 100 if sky_texture else 600))}',
        *sky_lines,
        '        }', '    }',
        '    def Scope "Cameras"', '    {',
    ])
    for camera in scene.get("cameras") or [
        {"id": "Overview", "position": [70, -90, 85], "target": [0, 0, 0]},
        {"id": "Driver", "position": [0, -35, 1.25], "target": [0, 0, 1]},
    ]:
        lines.extend([
            f'        def Camera "{_name(camera["id"])}"', '        {',
            '            token projection = "perspective"',
            f'            float focalLength = {_number(camera.get("focal_length", 24))}',
            '            float horizontalAperture = 36',
            '            float verticalAperture = 20.25',
            '            float2 clippingRange = (0.05, 2000)',
            f'            matrix4d xformOp:transform = {_camera_matrix(camera["position"], camera["target"])}',
            '            uniform token[] xformOpOrder = ["xformOp:transform"]',
            '        }',
        ])
    lines.extend(['    }', '}', ''])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    scene = json.loads(args.scene.read_text())
    export_usda(scene, args.output, args.scene.resolve().parent)
    print(f"Wrote {len(scene['objects'])} course objects to {args.output}")


if __name__ == "__main__":
    main()
