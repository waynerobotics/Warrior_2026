#!/usr/bin/env python3
"""ROS 2 wrapper: RGB (+ aligned depth) in, white-lane mask and ground clouds out.

Besides the mono8 mask, two flat (z = 0) PointCloud2s in ``base_frame`` feed a
Nav2 ObstacleLayer:

* ``points_topic``: lane-line paint on the ground (marking source).
* ``pothole_topic``: compact white blobs (simulated potholes), split from the
  lines by shape so Nav2 can treat lines as hard and potholes as soft.
* ``free_topic``: visible ground cells with no paint-coloured pixel
  (clearing source). White that fails the contrast test, e.g. a pothole
  close up, is neither marked nor cleared, so the costmap keeps it.
  Raytracing from the camera to these points erases stale lane marks that the
  camera can now see are gone.
"""
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from warrior_lane_detection.segmentation import (
    LaneParams, LaneSegmenter, overlay, split_potholes)


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LaneSegmentationNode(Node):
    def __init__(self):
        super().__init__('lane_segmentation')
        defaults = LaneParams()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('mask_topic', '/perception/lane_mask')
        self.declare_parameter('debug_topic', '/perception/lane_debug')
        self.declare_parameter('points_topic', '/perception/lane_points')
        self.declare_parameter('free_topic', '/perception/lane_free_points')
        self.declare_parameter('pothole_topic', '/perception/pothole_points')
        self.declare_parameter('points_resolution', 0.05)
        self.declare_parameter('free_resolution', 0.2)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('use_depth', True)
        self.declare_parameter('publish_debug', True)

        params = LaneParams(**{name: type(value)(self.get_parameter(name).value)
                               for name, value in vars(defaults).items()})
        self.segmenter = LaneSegmenter(params)
        self.bridge = CvBridge()
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.points_resolution = float(self.get_parameter('points_resolution').value)
        self.free_resolution = float(self.get_parameter('free_resolution').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mask_pub = self.create_publisher(Image, self.get_parameter('mask_topic').value, 5)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_topic').value, 2)
        self.points_pub = self.create_publisher(
            PointCloud2, self.get_parameter('points_topic').value, 5)
        self.free_pub = self.create_publisher(
            PointCloud2, self.get_parameter('free_topic').value, 5)
        self.pothole_pub = self.create_publisher(
            PointCloud2, self.get_parameter('pothole_topic').value, 5)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self.on_info, qos_profile_sensor_data)

        image_topic = self.get_parameter('image_topic').value
        if self.get_parameter('use_depth').value:
            image_sub = message_filters.Subscriber(self, Image, image_topic,
                                                   qos_profile=qos_profile_sensor_data)
            depth_sub = message_filters.Subscriber(self, Image, self.get_parameter('depth_topic').value,
                                                   qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [image_sub, depth_sub], queue_size=5, slop=0.05)
            self.sync.registerCallback(self.on_images)
        else:
            self.create_subscription(Image, image_topic, lambda m: self.on_images(m, None),
                                     qos_profile_sensor_data)
        self.have_info = False
        self.get_logger().info(
            f'lane_segmentation: {image_topic} -> {self.get_parameter("mask_topic").value} '
            f'(depth ground check: {self.get_parameter("use_depth").value})')

    def on_info(self, msg):
        # Prefer the rectified projection P; fall back to K if P is empty.
        fx, fy, cx, cy = msg.p[0], msg.p[5], msg.p[2], msg.p[6]
        if fx <= 0:
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.segmenter.set_intrinsics(fx, fy, cx, cy, msg.width, msg.height)
        self.have_info = True

    def camera_pose(self, msg):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(f'No TF {self.base_frame} <- {msg.header.frame_id}: {error}',
                                   throttle_duration_sec=5.0)
            return None
        tr = t.transform.translation
        return quaternion_matrix(t.transform.rotation), np.array([tr.x, tr.y, tr.z])

    def on_images(self, image_msg, depth_msg):
        if not self.have_info:
            return
        pose = self.camera_pose(image_msg)
        if pose is None:
            return
        rgb = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
        depth = None
        if depth_msg is not None:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth.dtype == np.uint16:  # RealSense driver publishes millimetres
                depth = depth.astype(np.float32) * 1.e-3
        lane, ground = self.segmenter.segment(rgb, *pose, depth)
        out = self.bridge.cv2_to_imgmsg(lane, encoding='mono8')
        out.header = image_msg.header
        self.mask_pub.publish(out)
        header = Header(stamp=image_msg.header.stamp, frame_id=self.base_frame)
        p = self.segmenter.params
        lines, potholes = split_potholes(
            self.segmenter.ground_points(lane, self.points_resolution), self.points_resolution,
            p.pothole_max_length, p.pothole_min_width)
        self.points_pub.publish(self.flat_cloud(header, lines))
        self.pothole_pub.publish(self.flat_cloud(header, potholes))
        self.free_pub.publish(self.flat_cloud(
            header, self.segmenter.free_points(ground, self.free_resolution)))
        if self.publish_debug and self.debug_pub.get_subscription_count():
            debug = self.bridge.cv2_to_imgmsg(overlay(rgb, lane, ground), encoding='rgb8')
            debug.header = image_msg.header
            self.debug_pub.publish(debug)

    @staticmethod
    def flat_cloud(header, xy):
        points = [(float(x), float(y), 0.0) for x, y in xy]
        return point_cloud2.create_cloud_xyz32(header, points)


def main(args=None):
    rclpy.init(args=args)
    node = LaneSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
