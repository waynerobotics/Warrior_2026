#!/usr/bin/env python3
"""PhysX Warrior articulation and sensors, with a localhost ROS transport adapter."""
import argparse
import json
import math
from pathlib import Path
import socket
import shutil
import signal
import sys
import time
import traceback
import xml.etree.ElementTree as ET

from runtime import application_arguments

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def report_simulation_failure(app):
    """Preserve Python failures before Kit's fast shutdown exits the process."""
    if sys.exc_info()[1] is not None:
        traceback.print_exc()
        sys.stderr.flush()
        app.app.post_quit(1)


class SimulationTime:
    """Keep sensor sampling and wall pacing aligned through a GUI pause."""

    def __init__(self, stamp, wall_time):
        self.last_stamp = stamp
        self.wall_start = wall_time - stamp
        self.was_paused = False

    def running(self, stamp, wall_time, *, playing, stopped):
        if stopped or stamp < self.last_stamp:
            # The ROS bridge and localization filters share this clock epoch.
            # Resetting only the articulation would leave stale ROS state.
            raise RuntimeError('Isaac timeline stopped or reset; relaunch the ROS simulation '
                               'to reset the robot, sensors, and localization together.')
        if not playing:
            self.was_paused = True
            return False
        if self.was_paused:
            self.wall_start = wall_time - stamp
            self.was_paused = False
        return True

    def sample_interval(self, stamp):
        if stamp < self.last_stamp:
            raise RuntimeError('Isaac simulation clock moved backward; relaunch the ROS simulation.')
        interval = stamp - self.last_stamp
        if interval == 0:
            return None
        self.last_stamp = stamp
        return interval


def rotate_vector(quaternion, vector):
    """Rotate a vector by a normalized scalar-first quaternion."""
    w, x, y, z = quaternion
    a, b, c = vector
    tx, ty, tz = 2 * (y*c-z*b), 2 * (z*a-x*c), 2 * (x*b-y*a)
    return [a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx]


def quaternion_product(left, right):
    w, x, y, z = left
    a, b, c, d = right
    return [w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
            w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a]


def fixed_mount(urdf, child, parent='base_footprint'):
    """Read the expanded URDF so sensor kinematics agree with ROS static TF."""
    joints = {joint.find('child').get('link'): joint for joint in ET.parse(urdf).getroot().findall('joint')}
    translation, orientation = [0.0] * 3, [1.0, 0.0, 0.0, 0.0]
    visited = set()
    while child != parent:
        if child in visited or child not in joints:
            raise ValueError(f'No fixed sensor transform from {parent} to {child}')
        visited.add(child)
        joint = joints[child]
        if joint.get('type') != 'fixed':
            raise ValueError(f'Sensor mount {child} must use fixed joints')
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0.0] * 3
        rpy = [float(v) / 2 for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0.0] * 3
        cr, cp, cy = map(math.cos, rpy)
        sr, sp, sy = map(math.sin, rpy)
        rotation = [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                    cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]
        translation = [a+b for a, b in zip(xyz, rotate_vector(rotation, translation))]
        orientation = quaternion_product(rotation, orientation)
        child = joint.find('parent').get('link')
    return translation, orientation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, default=HERE / 'worlds/igvc2027_autonav.usda')
    parser.add_argument('--scene', type=Path, default=HERE.parent / 'igvc2027/scene.json')
    parser.add_argument('--urdf', type=Path, default=ROOT / 'build/isaac/warrior.urdf')
    parser.add_argument('--imu-frame', default='imu_frame', help='IMU link in the expanded robot URDF')
    parser.add_argument('--antenna-frame', default='gnss_antenna_frame', help='GNSS antenna link in the expanded robot URDF')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seconds', type=float, default=0, help='Bounded simulation duration; 0 runs until closed')
    parser.add_argument('--port', type=int, default=17427, help='ROS adapter telemetry UDP port; commands use port+1')
    parser.add_argument('--test-drive', type=float, default=0, help='Drive forward at this m/s without ROS, for physics validation')
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--capture-time', type=float, default=0,
                        help='Capture at this simulation time; 0 captures the final frame')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--debug-cloud', type=Path, help='Optional final measured cloud as compressed NumPy data')
    parser.add_argument('--camera', choices=('follow', 'overview', 'eye'), default='follow')
    parser.add_argument('--realtime', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--rgbd', action='store_true',
                        help='Render the RealSense RGB-D camera and stream it to the ROS adapter (port+2)')
    parser.add_argument('--rgbd-frame', default='camera_color_optical_frame',
                        help='Optical frame (z forward, y down) of the camera in the expanded URDF')
    parser.add_argument('--rgbd-width', type=int, default=640)
    parser.add_argument('--rgbd-height', type=int, default=480)
    parser.add_argument('--rgbd-hfov', type=float, default=1.2112, help='Horizontal FOV in radians (realsense.xacro)')
    parser.add_argument('--rgbd-max-depth', type=float, default=10.0, help='Depth beyond this reads as no return')
    args = parser.parse_args()
    if not 1024 <= args.port < 65535:
        parser.error('--port must be between 1024 and 65534')
    stage_path = args.stage.resolve()
    if not stage_path.is_file():
        parser.error(f'Course stage does not exist: {stage_path}')
    if not args.urdf.is_file():
        parser.error('Run prepare_robot.py in the Pixi environment before launching Isaac')
    scene = json.loads(args.scene.read_text())
    gnss_offset, _ = fixed_mount(args.urdf, args.antenna_frame)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'renderer': 'RaytracedLighting',
                         'width': 1280, 'height': 720, 'multi_gpu': False, 'sync_loads': True,
                         'extra_args': application_arguments()})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', args.port + 1))
    sock.setblocking(False)
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        import numpy as np
        # Pure NumPy algorithm shared with the Gazebo ROS lidar adapter.
        sys.path.insert(0, str(ROOT / 'warrior_navigation'))
        from warrior_navigation.ground_filter import obstacle_mask, estimate_ground_plane
        import omni.kit.commands
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade
        # Isaac Sim 6.x deprecated isaacsim.sensors.physics; its IMU now lives in
        # the experimental extension, which wraps an authored IMU prim.
        enable_extension('isaacsim.sensors.experimental.physics')
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        enable_extension('isaacsim.asset.importer.urdf')
        enable_extension('isaacsim.sensors.physx')
        from isaacsim.sensors.physx import _range_sensor
        # Import into an isolated asset folder next to the expanded URDF. The 6.x
        # importer (urdf-usd-converter) writes <dir>/<robot>/<robot>.usda with
        # physics in a "Physics" variant set that has no default selection.
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        # The importer auto-numbers (warrior_1, warrior_2, ...) instead of
        # overwriting, so start each launch from an empty generated folder.
        asset_dir = args.urdf.parent.resolve() / 'usd'
        shutil.rmtree(asset_dir, ignore_errors=True)
        config = URDFImporterConfig(
            urdf_path=str(args.urdf.resolve()), usd_path=str(asset_dir),
            merge_fixed_joints=True, fix_base=False, allow_self_collision=False,
            collision_type='Convex Hull')
        robot_asset = Path(URDFImporter().import_urdf(config))
        asset_stage = Usd.Stage.Open(str(robot_asset))
        root_name = asset_stage.GetDefaultPrim().GetName()
        del asset_stage
        if not root_name:
            raise RuntimeError(f'Warrior URDF import produced no default prim: {robot_asset}')
        root_path = '/' + root_name
        print('Imported Warrior asset:', robot_asset, flush=True)
        if not omni.usd.get_context().open_stage(str(stage_path)):
            raise RuntimeError(f'Isaac Sim could not open {stage_path}')
        app.update()
        stage = omni.usd.get_context().get_stage()
        from run_course import configure_sky_background
        configure_sky_background(stage)
        stage.SetEditTarget(stage.GetSessionLayer())
        robot_prim = stage.DefinePrim(root_path, 'Xform')
        robot_prim.GetReferences().AddReference(str(robot_asset))
        robot_prim.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
        app.update()
        articulation_path = next((str(p.GetPath()) for p in stage.Traverse()
                                  if str(p.GetPath()).startswith(root_path + '/')
                                  and p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
        if articulation_path is None:
            raise RuntimeError('Warrior URDF import has no articulation root')
        print('Imported Warrior articulation:', articulation_path, flush=True)
        body_prims = [p for p in stage.Traverse() if str(p.GetPath()).startswith(root_path + '/')
                      and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        print('Robot bodies:', [str(p.GetPath()) for p in body_prims], flush=True)
        for prim in body_prims:
            physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            physx_body.CreateEnableGyroscopicForcesAttr(True)
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(root_path + '/') and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                api.CreateSolverPositionIterationCountAttr(16)
                api.CreateSolverVelocityIterationCountAttr(4)
            if str(prim.GetPath()).startswith(root_path + '/') and prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                steering = prim.GetName().endswith('_steer_joint')
                drive.CreateTypeAttr('force')
                drive.CreateStiffnessAttr(6000.0 if steering else 0.0)
                drive.CreateDampingAttr(400.0 if steering else 150.0)
                drive.CreateMaxForceAttr(150.0 if steering else 90.0)
        world = World(physics_dt=1/120, rendering_dt=1/60, stage_units_in_meters=1.0,
                      physics_prim_path='/World/PhysicsScene', device='cpu')
        robot = world.scene.add(SingleArticulation(prim_path=articulation_path, name='warrior'))
        spawn = scene['spawn']
        yaw = spawn[5]
        orientation = euler_angles_to_quat(np.array(spawn[3:6]))
        robot.set_world_pose(position=np.array(spawn[:3]), orientation=orientation)
        robot.set_default_state(position=np.array(spawn[:3]), orientation=orientation)
        # With merge_fixed_joints the 6.x importer folds base_link into the
        # base_footprint root body; older importers kept base_link separate.
        chassis = next((p for p in body_prims if p.GetName() == 'base_link'), None) or \
            next(p for p in body_prims if p.GetName() == 'base_footprint')
        chassis_path = str(chassis.GetPath())
        imu_offset, imu_rotation = fixed_mount(args.urdf, args.imu_frame, chassis.GetName())
        # Reads every physics step; the telemetry loop below sets the sample rate.
        imu = IMUSensor(IMU.create(chassis_path + '/WarriorImu',
                                   translations=[list(imu_offset)],
                                   orientations=[list(imu_rotation)]))
        # Match the physical lidar mount, expressed upright in the footprint.
        lidar_offset = np.array([-.172963, .002844, 1.092773])
        lidar_on_chassis = lidar_offset - (np.array([-.307963, -.002156, .640773])
                                           if chassis.GetName() == 'base_link' else 0)
        ok, lidar = omni.kit.commands.execute('RangeSensorCreateLidar', path='/WarriorLidar',
            parent=chassis_path, translation=Gf.Vec3d(*lidar_on_chassis.tolist()),
            min_range=.9, max_range=15.0, draw_points=False, draw_lines=False,
            horizontal_fov=360.0, vertical_fov=60.0, horizontal_resolution=1.0,
            vertical_resolution=2.0, rotation_rate=0.0, high_lod=True, yaw_offset=0.0)
        if not ok:
            raise RuntimeError('Could not create PhysX lidar')
        lidar_path = str(lidar.GetPath())
        lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
        rgbd = None
        if args.rgbd:
            import omni.replicator.core as rep
            from rgbd_stream import FrameSender, pack
            # The URDF optical frame looks along +z with +y down; a USD camera
            # looks along -z with +y up, i.e. a half turn about x.
            camera_offset, camera_rotation = fixed_mount(args.urdf, args.rgbd_frame, chassis.GetName())
            camera_rotation = quaternion_product(camera_rotation, [0.0, 1.0, 0.0, 0.0])
            rgbd_camera = UsdGeom.Camera.Define(stage, chassis_path + '/WarriorRgbd')
            aperture = 20.955
            rgbd_camera.CreateHorizontalApertureAttr(aperture)
            rgbd_camera.CreateVerticalApertureAttr(aperture * args.rgbd_height / args.rgbd_width)
            rgbd_camera.CreateFocalLengthAttr(aperture / 2 / math.tan(args.rgbd_hfov / 2))
            rgbd_camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 100.0))
            rgbd_camera.AddTranslateOp().Set(Gf.Vec3d(*camera_offset))
            rgbd_camera.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(camera_rotation[0], Gf.Vec3d(*camera_rotation[1:])))
            rgbd_product = rep.create.render_product(str(rgbd_camera.GetPath()), (args.rgbd_width, args.rgbd_height))
            rgbd_color = rep.AnnotatorRegistry.get_annotator('rgb')
            rgbd_depth = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
            rgbd_color.attach(rgbd_product)
            rgbd_depth.attach(rgbd_product)
            focal_px = args.rgbd_width / 2 / math.tan(args.rgbd_hfov / 2)
            rgbd_intrinsics = (focal_px, focal_px, args.rgbd_width / 2, args.rgbd_height / 2)
            rgbd = FrameSender(args.port + 2)
            print(f'RGB-D camera {args.rgbd_frame} at {[round(v, 3) for v in camera_offset]} on '
                  f'{chassis.GetName()}, {args.rgbd_width}x{args.rgbd_height} -> localhost:{args.port + 2}', flush=True)

        def send_rgbd(stamp):
            color = rgbd_color.get_data()
            depth = rgbd_depth.get_data()
            if not getattr(color, 'size', 0) or not getattr(depth, 'size', 0):
                return
            color = np.ascontiguousarray(np.asarray(color)[..., :3])
            depth = np.asarray(depth, dtype=np.float32).reshape(color.shape[:2]).copy()
            depth[~(depth > 0) | (depth > args.rgbd_max_depth)] = np.inf
            rgbd.submit(pack(stamp, color, depth, rgbd_intrinsics))
        world.reset()
        print('DOFs:', robot.dof_names, flush=True)
        prefixes = ['front', 'left', 'right']
        steer_indices = np.array([robot.get_dof_index(p + '_steer_joint') for p in prefixes])
        drive_indices = np.array([robot.get_dof_index(p + '_drive_joint') for p in prefixes])
        # Steering axes and wheel center offsets derived from the repository URDF.
        steering_xy = np.array([[.544926, -.031742], [-.2449357, .487785], [-.2999913, -.456043]])
        wheel_local = np.array([.000847, -.006293])
        wheel_radius = .10795
        command = np.zeros(3)
        smooth = np.zeros(3)
        odom = np.zeros(3)
        last_command_wall = -math.inf
        last_command_stamp = -math.inf
        sequence = 0
        ranges = [None] * 360
        positions = []
        camera = UsdGeom.Camera.Define(stage, '/World/Cameras/RobotFollow')
        camera.CreateFocalLengthAttr(23.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 1000.0))
        camera_transform = camera.AddTransformOp()
        camera_path = {'follow': '/World/Cameras/RobotFollow', 'overview': '/World/Cameras/Overview',
                       'eye': '/World/Cameras/CourseEye'}[args.camera]

        def update_camera(position):
            target = Gf.Vec3d(float(position[0]), float(position[1]), float(position[2]) + .7)
            offset = Gf.Vec3d(-5*math.cos(yaw)+4*math.sin(yaw), -5*math.sin(yaw)-4*math.cos(yaw), 3.6)
            camera_transform.Set(Gf.Matrix4d().SetLookAt(target + offset, target, Gf.Vec3d(0, 0, 1)).GetInverse())

        update_camera(spawn[:3])
        rgb = None
        captured = False
        if args.capture:
            import omni.replicator.core as rep
            from PIL import Image
            product = rep.create.render_product(camera_path, (1920, 1080))
            rgb = rep.AnnotatorRegistry.get_annotator('rgb')
            rgb.attach(product)

        def capture_frame():
            pixels = rgb.get_data()
            if getattr(pixels, 'size', 0):
                args.capture.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(pixels).save(args.capture)
                return True
            return False
        if not args.headless:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport:
                viewport.camera_path = camera_path
        timing = SimulationTime(world.current_time, time.monotonic())
        previous_q = None
        previous_antenna = None
        previous_sensor_stamp = None
        start_position = None
        minimum_range = math.inf
        while app.is_running() and not stopping and (args.seconds <= 0 or world.current_time < args.seconds):
            if not timing.running(world.current_time, time.monotonic(),
                                  playing=world.is_playing(), stopped=world.is_stopped()):
                command.fill(0)
                smooth.fill(0)
                last_command_wall = -math.inf
                # Commands stamped at the frozen simulation time must not be
                # replayed as fresh commands after a long pause.
                while True:
                    try:
                        sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                # Keep processing GUI events so Play can resume the simulation.
                world.step(render=True)
                continue
            while True:
                try:
                    packet, address = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                try:
                    message = json.loads(packet)
                    stamp = float(message['stamp'])
                    target = np.array(message['twist'], dtype=float)
                    if target.shape != (3,) or not np.isfinite(target).all() or not math.isfinite(stamp):
                        continue
                    if stamp < last_command_stamp or abs(stamp - world.current_time) > 1.0:
                        continue
                    command = np.clip(target, [-1, -.8, -1.5], [1, .8, 1.5])
                    last_command_stamp, last_command_wall = stamp, time.monotonic()
                except (ValueError, KeyError, TypeError):
                    continue
            q = robot.get_joint_positions()
            steering = q[steer_indices]
            target = command if time.monotonic() - last_command_wall < .75 else np.zeros(3)
            if args.test_drive and world.current_time > 2:
                target = np.array([args.test_drive, 0, 0])
            smooth += np.clip(target - smooth, -np.array([1.0, 1.0, 2.0])/60, np.array([1.0, 1.0, 2.0])/60)
            steer_targets, wheel_targets = [], []
            for i, (x, y) in enumerate(steering_xy):
                vx, vy = smooth[0] - smooth[2]*y, smooth[1] + smooth[2]*x
                speed = math.hypot(vx, vy)
                heading = math.atan2(vy, vx) if speed > .001 else steering[i]
                delta = math.atan2(math.sin(heading-steering[i]), math.cos(heading-steering[i]))
                if abs(delta) > math.pi/2:
                    speed = -speed
                    delta = math.atan2(math.sin(delta+math.pi), math.cos(delta+math.pi))
                steer_targets.append(steering[i] + delta)
                wheel_targets.append(speed * max(0, math.cos(delta)) / wheel_radius)
            robot.apply_action(ArticulationAction(joint_positions=np.array(steer_targets), joint_indices=steer_indices))
            robot.apply_action(ArticulationAction(joint_velocities=np.array(wheel_targets), joint_indices=drive_indices))
            world.step(render=True)
            if stopping or not app.is_running():
                break
            t = world.current_time
            # Pause/Stop can arrive while the step processes GUI events.
            if not timing.running(t, time.monotonic(), playing=world.is_playing(), stopped=world.is_stopped()):
                command.fill(0)
                smooth.fill(0)
                last_command_wall = -math.inf
                continue
            sample_dt = timing.sample_interval(t)
            if sample_dt is None:
                continue
            q = robot.get_joint_positions()
            qdot = robot.get_joint_velocities()
            position, quaternion = robot.get_world_pose()
            antenna_position = position + np.array(rotate_vector(quaternion, gnss_offset))
            # Ideal receiver-point kinematics stay inside the simulated sensor
            # path; the ROS adapter adds MTi-680G measurement errors/covariance.
            antenna_velocity = (np.zeros(3) if previous_antenna is None else
                                (antenna_position - previous_antenna) / (t - previous_sensor_stamp))
            previous_antenna, previous_sensor_stamp = antenna_position.copy(), t
            if sequence % 6 == 0:
                update_camera(position)
            if start_position is None and t >= 2:
                start_position = position.copy()
            if previous_q is None:
                wheel_rates = np.zeros(3)
            else:
                delta_q = q[drive_indices] - previous_q[drive_indices]
                wheel_rates = np.arctan2(np.sin(delta_q), np.cos(delta_q)) / sample_dt
            previous_q = q.copy()
            matrix, readings = [], []
            for i, angle in enumerate(q[steer_indices]):
                c, s = math.cos(angle), math.sin(angle)
                offset = [c*wheel_local[0]-s*wheel_local[1], s*wheel_local[0]+c*wheel_local[1]]
                x, y = steering_xy[i] + offset
                matrix.extend([[1, 0, -y], [0, 1, x]])
                readings.extend([wheel_radius*wheel_rates[i]*c, wheel_radius*wheel_rates[i]*s])
            twist = np.linalg.lstsq(np.array(matrix), np.array(readings), rcond=None)[0]
            c, s = math.cos(odom[2]), math.sin(odom[2])
            odom += np.array([c*twist[0]-s*twist[1], s*twist[0]+c*twist[1], twist[2]]) * sample_dt
            imu_data = imu.get_data()
            if sequence % 6 == 0:
                points = np.array(lidar_interface.get_point_cloud_data(lidar_path)).reshape(-1, 3)
                if points.size:
                    points = points + lidar_offset
                    distance = np.linalg.norm(points[:, :2], axis=1)
                    keep = obstacle_mask(points, max_range=14.99) & (distance >= .9)
                    bins = np.floor((np.arctan2(points[keep, 1], points[keep, 0])+math.pi)*180/math.pi).astype(int) % 360
                    scan = np.full(360, np.inf)
                    np.minimum.at(scan, bins, distance[keep])
                    ranges = [float(r) if math.isfinite(r) else None for r in scan]
                    if np.isfinite(scan).any():
                        minimum_range = min(minimum_range, float(np.min(scan)))
            data = {'version': 2, 'stamp': t, 'sequence': sequence,
                    'joint_names': robot.dof_names, 'joint_position': q.tolist(), 'joint_velocity': qdot.tolist(),
                    'odom': odom.tolist(), 'twist': twist.tolist(),
                    'imu_orientation': imu_data['orientation'].tolist(),
                    'imu_angular_velocity': imu_data['angular_velocity'].tolist(),
                    'imu_linear_acceleration': imu_data['linear_acceleration'].tolist(),
                    'position': position.tolist(), 'orientation': quaternion.tolist(),
                    'gnss_position': antenna_position.tolist() if sequence % 6 == 0 else None,
                    'gnss_velocity': antenna_velocity.tolist() if sequence % 6 == 0 else None,
                    'scan': ranges if sequence % 6 == 0 else None}
            sock.sendto(json.dumps(data, allow_nan=False).encode(), ('127.0.0.1', args.port))
            if rgbd is not None and sequence % 6 == 0:
                send_rgbd(t)
            if args.capture and args.capture_time > 0 and t >= args.capture_time and not captured:
                captured = capture_frame()
            sequence += 1
            if sequence % 120 == 0:
                print(f'Warrior t={t:.1f} pose={position.round(3).tolist()} wheel_odom={odom.round(3).tolist()} cmd={smooth.round(2).tolist()}'
                      + (f' rgbd_sent={rgbd.sent}' if rgbd is not None else ''), flush=True)
                positions.append({'t': t, 'position': position.tolist(), 'odom': odom.tolist()})
            if args.realtime:
                delay = timing.wall_start + t - time.monotonic()
                if delay > 0:
                    time.sleep(min(delay, .02))
        if args.capture and not captured:
            capture_frame()
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps({'frames': sequence, 'sim_seconds': world.current_time,
                'start_position': start_position.tolist() if start_position is not None else None,
                'final_position': position.tolist(), 'final_odom': odom.tolist(),
                'minimum_lidar_range': minimum_range if math.isfinite(minimum_range) else None,
                'final_scan_finite_bins': sum(value is not None for value in ranges),
                'ground_plane': estimate_ground_plane(points).tolist(),
                'imu': {k: v.tolist() if hasattr(v, 'tolist') else v for k, v in imu_data.items()},
                'trajectory': positions}, indent=2))
        if args.debug_cloud:
            args.debug_cloud.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(args.debug_cloud, points=points, robot_position=position,
                                robot_orientation=quaternion)
        world.stop()
    finally:
        sock.close()
        report_simulation_failure(app)
        app.close()


if __name__ == '__main__':
    main()
