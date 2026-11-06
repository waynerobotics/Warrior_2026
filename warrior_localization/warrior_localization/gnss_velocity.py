"""Convert ENU antenna velocity into robot velocity with covariance propagation."""

import math

import numpy as np


def rotation_matrix(quaternion):
    """Return the active rotation for an (x, y, z, w) quaternion."""
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError('Orientation must be a finite quaternion')
    norm = np.linalg.norm(quaternion)
    if norm < 1e-10:
        raise ValueError('Orientation quaternion cannot be zero')
    x, y, z, w = quaternion / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def skew(vector):
    """Return the matrix that computes cross(vector, other)."""
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _covariance(values, size):
    covariance = np.asarray(values, dtype=float).reshape(size, size)
    if (not np.isfinite(covariance).all() or
            not np.allclose(covariance, covariance.T, atol=1e-10) or
            np.linalg.eigvalsh(covariance).min() < -1e-10):
        raise ValueError('Covariance must be finite, symmetric, and nonnegative')
    return covariance


def antenna_velocity_to_body(
        velocity_enu, velocity_covariance, imu_orientation, orientation_covariance,
        angular_velocity, angular_covariance, imu_to_body, antenna_offset):
    """
    Rotate antenna velocity and remove its rotational lever-arm velocity.

    IMU orientation maps IMU axes into ENU. imu_to_body is the static rotation
    from IMU axes to robot axes; antenna_offset is measured from the body origin.
    Covariances are first-order estimates, assuming independent velocity,
    attitude, and gyro errors. The returned covariance is a full 6x6 twist matrix.
    """
    velocity_enu = np.asarray(velocity_enu, dtype=float)
    angular_velocity = np.asarray(angular_velocity, dtype=float)
    antenna_offset = np.asarray(antenna_offset, dtype=float)
    if any(vector.shape != (3,) or not np.isfinite(vector).all()
           for vector in (velocity_enu, angular_velocity, antenna_offset)):
        raise ValueError('Velocities and lever arm must be finite 3-vectors')
    body_from_imu = rotation_matrix(imu_to_body)
    body_from_enu = body_from_imu @ rotation_matrix(imu_orientation).T
    body_antenna_velocity = body_from_enu @ velocity_enu
    body_angular_velocity = body_from_imu @ angular_velocity
    velocity = body_antenna_velocity - np.cross(body_angular_velocity, antenna_offset)

    cv = _covariance(velocity_covariance, 3)
    co = body_from_imu @ _covariance(orientation_covariance, 3) @ body_from_imu.T
    cw = body_from_imu @ _covariance(angular_covariance, 3) @ body_from_imu.T
    lever = skew(antenna_offset)
    attitude = skew(body_antenna_velocity)
    covariance = np.zeros((6, 6))
    covariance[:3, :3] = (
        body_from_enu @ cv @ body_from_enu.T +
        attitude @ co @ attitude.T + lever @ cw @ lever.T)
    covariance[:3, 3:] = lever @ cw
    covariance[3:, :3] = covariance[:3, 3:].T
    covariance[3:, 3:] = cw
    return velocity, body_angular_velocity, covariance


def closest_imu(samples, stamp, max_age):
    """Return the closest timestamped attitude, rejecting stale measurements."""
    if not samples or not math.isfinite(stamp) or max_age < 0:
        return None
    selected = min(samples, key=lambda sample: abs(sample[0] - stamp))
    return selected[1] if abs(selected[0] - stamp) <= max_age else None
