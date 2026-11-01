#!/usr/bin/env python3
"""Validate a generated stage with OpenUSD; no GPU or Isaac application needed.

Requires the pxr Python bindings (usd-core or the bundled Isaac Sim USD library).
This checks USD composition and physics declarations, not runtime rendering or
PhysX simulation.
"""

import argparse
import json
from pathlib import Path


def validate(stage_path, scene_path=None):
    from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdUtils

    stage = Usd.Stage.Open(str(stage_path.resolve()))
    if not stage:
        raise ValueError(f"Could not compose USD stage: {stage_path}")
    if stage.GetDefaultPrim().GetPath() != "/World":
        raise ValueError("Expected /World as the default prim")
    if UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError("Expected meter-scale, Z-up stage")
    if not stage.GetPrimAtPath("/World/PhysicsScene").IsA(UsdPhysics.Scene):
        raise ValueError("Stage has no physics scene")
    if not stage.GetPrimAtPath("/World/RobotSpawn"):
        raise ValueError("Stage has no robot spawn transform")
    meshes, colliders, cameras, texture_files = {}, [], [], set()
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            valid, reason = mesh.ValidateTopology(mesh.GetFaceVertexIndicesAttr().Get(),
                                                 mesh.GetFaceVertexCountsAttr().Get(), len(points))
            if not valid:
                raise ValueError(f"{prim.GetPath()}: {reason}")
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            if not material:
                raise ValueError(f"{prim.GetPath()}: missing surface material")
            st = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
            if not st or len(st.Get()) != len(points):
                raise ValueError(f"{prim.GetPath()}: invalid per-vertex UV coordinates")
            source_id = prim.GetAttribute("course:sourceId").Get()
            if source_id in meshes:
                raise ValueError(f"Duplicate course source id: {source_id}")
            meshes[source_id] = prim
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                    raise ValueError(f"{prim.GetPath()}: collision is disabled")
                material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
                if not material or not material.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
                    raise ValueError(f"{prim.GetPath()}: missing contact material")
                colliders.append(source_id)
        if prim.IsA(UsdGeom.Camera):
            cameras.append(str(prim.GetPath()))
        if prim.IsA(UsdLux.DomeLight):
            asset = prim.GetAttribute('inputs:texture:file').Get()
            if asset:
                if not asset.resolvedPath or not Path(asset.resolvedPath).is_file():
                    raise ValueError(f'{prim.GetPath()}: unresolved sky texture')
                texture_files.add(asset.resolvedPath)
        if prim.IsA(UsdShade.Shader) and prim.GetAttribute("info:id").Get() == "UsdUVTexture":
            asset = prim.GetAttribute("inputs:file").Get()
            if not asset or not asset.resolvedPath or not Path(asset.resolvedPath).is_file():
                raise ValueError(f"{prim.GetPath()}: unresolved texture")
            texture_files.add(asset.resolvedPath)
    if not meshes or not colliders or not cameras:
        raise ValueError("Course must contain geometry, colliders and a camera")
    _, _, unresolved = UsdUtils.ComputeAllDependencies(str(stage_path.resolve()))
    if unresolved:
        raise ValueError(f"Unresolved USD assets: {unresolved}")
    checker = UsdUtils.ComplianceChecker(skipVariants=True)
    checker.CheckCompliance(str(stage_path.resolve()))
    failures = checker.GetErrors() + checker.GetFailedChecks()
    if failures:
        raise ValueError("USD compliance failures: " + "; ".join(failures))
    if scene_path:
        scene = json.loads(scene_path.read_text())
        expected = {obj["id"]: obj for obj in scene["objects"]}
        if set(meshes) != set(expected):
            raise ValueError("USD object identifiers differ from the shared scene")
        if set(colliders) != {obj["id"] for obj in scene["objects"] if obj.get("collision")}:
            raise ValueError("USD collisions differ from the shared scene")
        for name, obj in expected.items():
            position = meshes[name].GetAttribute("xformOp:translate").Get()
            if any(abs(position[i] - obj["pose"][i]) > 1e-6 for i in range(3)):
                raise ValueError(f"{name}: USD location differs from shared scene")
    return {"stage": str(stage_path), "meshes": len(meshes), "colliders": len(colliders),
            "cameras": cameras, "texture_files": len(texture_files),
            "warnings": checker.GetWarnings(), "runtime_tested": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", type=Path,
                        nargs="?", default=Path(__file__).parent / "worlds" / "igvc2027_autonav.usda")
    parser.add_argument("--scene", type=Path, help="Also check consistency with shared scene JSON")
    args = parser.parse_args()
    print(json.dumps(validate(args.stage, args.scene), indent=2))


if __name__ == "__main__":
    main()
