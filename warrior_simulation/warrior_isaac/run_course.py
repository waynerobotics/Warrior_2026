#!/usr/bin/env python3
"""Open the shared IGVC course in Isaac Sim 5.1 or newer.

Run with Isaac Sim's python.sh, not the ROS/Pixi Python interpreter. Importing
SimulationApp precedes every omni/pxr import, as required by Isaac Sim.
"""

import argparse
from pathlib import Path
import sys
import time


def configure_sky_background(stage):
    """Display the HDR sky at daylight luminance independently of light gain."""
    import carb
    settings = carb.settings.get_settings()
    dome = stage.GetPrimAtPath('/World/Lighting/Sky')
    texture = dome.GetAttribute('inputs:texture:file').Get() if dome else None
    if texture and texture.resolvedPath:
        settings.set('/rtx/background/source/type', 1)
        settings.set('/rtx/background/source/texture/path', texture.resolvedPath)
        settings.set('/rtx/background/source/texture/colorSpace', 1)
        settings.set('/rtx/background/source/texture/luminanceScale', 3.0)
    else:
        settings.set('/rtx/background/source/type', 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stage", type=Path,
        default=Path(__file__).parent / "worlds" / "igvc2027_autonav.usda",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--frames", type=int, default=0,
                        help="Stop after this many simulation frames (0: run until closed)")
    parser.add_argument("--camera", default="/World/Cameras/Overview")
    parser.add_argument("--capture", type=Path,
                        help="Write the selected camera's rendered PNG after --frames")
    parser.add_argument("--robot-usd", type=Path,
                        help="Optional existing Isaac-compatible robot USD; reference it at RobotSpawn")
    args = parser.parse_args()
    if args.frames < 0:
        parser.error("--frames must be nonnegative")
    if args.capture and not args.frames:
        parser.error("--capture requires a positive --frames value")
    stage_path = args.stage.resolve()
    if not stage_path.is_file():
        parser.error(f"Course stage does not exist: {stage_path}")
    if args.robot_usd and not args.robot_usd.is_file():
        parser.error(f"Robot USD does not exist: {args.robot_usd}")

    try:
        from isaacsim import SimulationApp
    except ImportError:
        parser.exit(2, "Use your Isaac Sim installation's python.sh to run this script.\n")

    app = SimulationApp({
        "headless": args.headless,
        "renderer": "RaytracedLighting",
        "width": 1920,
        "height": 1080,
        "multi_gpu": False,
        "sync_loads": True,
    })
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.stage import is_stage_loading

        if not omni.usd.get_context().open_stage(str(stage_path)):
            raise RuntimeError(f"Isaac Sim could not open {stage_path}")
        app.update()
        app.update()
        deadline = time.monotonic() + 120
        while is_stage_loading():
            if time.monotonic() > deadline:
                raise TimeoutError("Course stage did not finish loading in 120 seconds")
            app.update()

        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath(args.camera):
            raise ValueError(f"Camera not found in stage: {args.camera}")
        if args.robot_usd:
            from pxr import UsdGeom

            robot = UsdGeom.Xform.Define(stage, "/World/RobotSpawn/Robot")
            robot.GetPrim().GetReferences().AddReference(str(args.robot_usd.resolve()))
        if not args.headless:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport and stage.GetPrimAtPath(args.camera):
                viewport.set_active_camera(args.camera)
        app.reset_render_settings()
        configure_sky_background(stage)
        if args.capture:
            import omni.replicator.core as rep

            render_product = rep.create.render_product(args.camera, (1920, 1080))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(render_product)
        print(f"Loaded IGVC 2027 simulation approximation: {stage_path}", flush=True)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        frames = 0
        while app.is_running() and (not args.frames or frames < args.frames):
            app.update()
            frames += 1
        timeline.stop()
        if args.capture:
            from PIL import Image

            rep.orchestrator.step(rt_subframes=16)
            pixels = rgb.get_data()
            if not pixels.size:
                raise RuntimeError("Isaac Sim returned an empty camera image")
            args.capture.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(pixels).save(args.capture)
            rgb.detach(render_product)
            render_product.destroy()
            print(f"Saved rendered camera image: {args.capture}", flush=True)
        print(f"Completed {frames} Isaac Sim frames", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
