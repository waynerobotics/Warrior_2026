"""Regression tests for encoder-free GNSS/INS localization and frame conversion."""

from copy import deepcopy
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from warrior_localization.gnss_velocity import (  # noqa: E402
    antenna_velocity_to_body,
    closest_imu,
    rotation_matrix,
)
from warrior_localization.gps_fusion import fusion_parameters  # noqa: E402


def _configuration():
    path = Path(__file__).resolve().parents[1] / 'config/gps_sim_ekf.yaml'
    return yaml.safe_load(path.read_text())


def _yaw(angle):
    return [0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)]


def _convert(**overrides):
    arguments = {
        'velocity_enu': [1.0, 0.0, 0.0],
        'velocity_covariance': np.diag([0.04, 0.01, 0.09]),
        'imu_orientation': _yaw(0.0),
        'orientation_covariance': np.zeros((3, 3)),
        'angular_velocity': [0.0, 0.0, 0.0],
        'angular_covariance': np.zeros((3, 3)),
        'imu_to_body': _yaw(0.0),
        'antenna_offset': [0.0, 0.0, 0.0],
    }
    arguments.update(overrides)
    return antenna_velocity_to_body(**arguments)


def test_north_facing_robot_rotates_enu_velocity_and_covariance():
    velocity, angular, covariance = _convert(
        velocity_enu=[0.0, 2.0, 0.0], imu_orientation=_yaw(math.pi / 2))
    np.testing.assert_allclose(velocity, [2.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(angular, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(np.diag(covariance)[:3], [0.01, 0.04, 0.09])


def test_sideways_swerve_motion_is_preserved():
    velocity, _, _ = _convert(velocity_enu=[0.0, 1.0, 0.0])
    np.testing.assert_allclose(velocity, [0.0, 1.0, 0.0])


def test_stationary_body_with_rotating_offset_antenna_has_zero_linear_velocity():
    velocity, angular, covariance = _convert(
        velocity_enu=[0.0, 1.0, 0.0], antenna_offset=[0.5, 0.0, 0.0],
        angular_velocity=[0.0, 0.0, 2.0], angular_covariance=np.diag([0.0, 0.0, 0.04]))
    np.testing.assert_allclose(velocity, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(angular, [0.0, 0.0, 2.0])
    assert covariance[1, 1] == pytest.approx(0.02)
    assert covariance[1, 5] == pytest.approx(-0.02)
    assert np.linalg.eigvalsh(covariance).min() >= -1e-12


def test_static_imu_mount_is_removed_before_rotating_velocity():
    # Robot faces north; IMU is rotated another 90 degrees about body z.
    velocity, _, _ = _convert(
        velocity_enu=[0.0, 2.0, 0.0], imu_orientation=_yaw(math.pi),
        imu_to_body=_yaw(math.pi / 2))
    np.testing.assert_allclose(velocity, [2.0, 0.0, 0.0], atol=1e-12)


def test_heading_uncertainty_contributes_to_transverse_velocity_variance():
    _, _, covariance = _convert(
        velocity_enu=[2.0, 0.0, 0.0], orientation_covariance=np.diag([0.0, 0.0, 0.1]))
    assert covariance[1, 1] == pytest.approx(0.41)
    assert covariance[0, 0] == pytest.approx(0.04)


@pytest.mark.parametrize('values', [[0, 0, 0, 0], [0, 0, math.nan, 1], [1, 2, 3]])
def test_invalid_attitude_is_rejected(values):
    with pytest.raises(ValueError):
        rotation_matrix(values)


@pytest.mark.parametrize('covariance', [
    np.diag([-1.0, 0.0, 0.0]), np.diag([math.nan, 0.0, 0.0]),
    [[1.0, 0.2, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
])
def test_invalid_covariance_is_rejected(covariance):
    with pytest.raises(ValueError):
        _convert(velocity_covariance=covariance)


def test_attitude_selection_rejects_stale_and_future_unmatched_samples():
    samples = [(10.0, 'old'), (10.09, 'near'), (10.2, 'later')]
    assert closest_imu(samples, 10.1, 0.05) == 'near'
    assert closest_imu(samples, 11.0, 0.2) is None
    assert closest_imu(samples, 1.0, 0.2) is None
    assert closest_imu([], 10.0, 0.2) is None


@pytest.mark.parametrize('use_wheels', [True, False])
def test_modes_preserve_complete_tf_chain_and_absolute_global_observations(use_wheels):
    config = fusion_parameters(_configuration(), use_wheels)
    local = config['ekf_node_odom']['ros__parameters']
    global_ = config['ekf_node_map']['ros__parameters']
    assert local['world_frame'] == local['odom_frame'] == 'odom'
    assert global_['world_frame'] == global_['map_frame'] == 'map'
    assert local['publish_tf'] and global_['publish_tf']
    assert local['base_link_frame'] == global_['base_link_frame'] == 'base_footprint'
    gps_index = 'odom1' if use_wheels else 'odom0'
    assert global_[gps_index] == '/odometry/gps'
    assert global_[gps_index + '_config'][:2] == [True, True]
    assert global_['imu0'] == local['imu0'] == '/imu/data'
    assert global_['imu0_config'][5]
    assert local['imu0_config'][11]
    assert not local['imu0_config'][5]
    for params in (local, global_):
        assert '/odom_gt' not in params.values()
        if use_wheels:
            assert params['odom0'] == '/odom'
            assert params['odom0_config'][6:8] == [True, True]
        else:
            assert '/odom' not in params.values()
            assert params['twist0'] == '/gnss/velocity_body'
            assert params['twist0_config'] == [False] * 6 + [True, True] + [False] * 7
    if not use_wheels:
        assert 'odom0' not in local
        assert 'odom1' not in global_


def test_no_wheels_mode_enforces_selection_for_custom_parameters_without_mutating_tuning():
    source = _configuration()
    for name in ('ekf_node_odom', 'ekf_node_map'):
        params = source[name]['ros__parameters']
        params['odom0'] = '/custom_encoder_velocity'
        params['frequency'] = 50.0
        params['odom2'] = '/odom'
        params['odom2_config'] = [True] * 15
    original = deepcopy(source)
    result = fusion_parameters(source, False)
    assert source == original
    for name in ('ekf_node_odom', 'ekf_node_map'):
        params = result[name]['ros__parameters']
        assert params['frequency'] == 50.0
        assert '/custom_encoder_velocity' not in params.values()
        assert '/odom' not in params.values()
        assert 'odom2_config' not in params
