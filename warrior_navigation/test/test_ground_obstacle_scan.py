"""Protect lidar frame conversion and scan semantics used by the ROS bridge."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from warrior_navigation.ground_obstacle_scan import (  # noqa: E402
    cloud_xyz, laser_ranges, transform_xyz,
)


def test_cloud_with_integer_ring_field_and_invalid_returns():
    fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
              for index, name in enumerate(('x', 'y', 'z'))]
    fields.append(PointField(name='ring', offset=12, datatype=PointField.UINT16, count=1))
    cloud = point_cloud2.create_cloud(
        Header(), fields, [(1., 2., 3., 7), (math.nan, 2., 3., 8), (4., 5., 6., 9)])
    np.testing.assert_allclose(cloud_xyz(cloud), [[1, 2, 3], [4, 5, 6]])


def test_inverted_lidar_mount_is_rotated_before_ground_fitting():
    transform = Transform()
    transform.rotation.x = 1.0
    transform.rotation.w = 0.0
    transform.translation.z = .6
    np.testing.assert_allclose(
        transform_xyz(np.asarray([[1., 2., 3.]]), transform), [[1, -2, -2.4]])


def test_nearest_return_bins_wrap_and_empty_rays_clear_space():
    points = np.asarray([[3., 0, .5], [8., 0, .5], [-3., 0, .5],
                         [0, 2., .5], [.5, 0, .5], [16., 0, .5]])
    ranges = laser_ranges(
        points, angle_min=-math.pi, angle_max=math.pi,
        angle_increment=math.pi/180, range_min=.9, range_max=15.)
    assert len(ranges) == 360
    assert ranges[0] == pytest.approx(3.)
    assert ranges[180] == pytest.approx(3.)
    assert ranges[270] == pytest.approx(2.)
    assert np.isinf(ranges).sum() == 357
