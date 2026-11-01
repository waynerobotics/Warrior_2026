"""Ground rejection from robot-frame lidar returns, independent of simulator truth.

A bounded RANSAC fit estimates the nearby road plane. Low-return radial
profiles then follow gentle curvature with bounded grade changes, preventing
the far side of a depression from becoming a solid costmap obstacle ring.
This is not a traversability classifier for steep terrain or overhangs.
"""
import math

import numpy as np


def estimate_ground_plane(points, *, ground_height=0.0, fit_radius=6.0):
    """Return (a,b,c) for z=a*x+b*y+c in an upright robot frame.

``ground_height`` is the nominal road z in this frame (0 for base_footprint).
The fit never uses an elevation map or simulated robot pose. Insufficient or
implausible returns fall back to the nominal horizontal ground plane.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    radius = np.linalg.norm(points[:, :2], axis=1)
    candidate = points[np.isfinite(points).all(axis=1) & (radius > .8) & (radius < fit_radius)
                       & (points[:, 2] > ground_height - .7)
                       & (points[:, 2] < ground_height + .45)]
    fallback = np.array([0., 0., float(ground_height)])
    if len(candidate) < 18:
        return fallback
    if len(candidate) > 480:
        candidate = candidate[np.linspace(0, len(candidate) - 1, 480, dtype=int)]
    design = np.column_stack((candidate[:, :2], np.ones(len(candidate))))
    maximum_grade = math.tan(math.radians(15))
    rng = np.random.default_rng(2027)
    best = None
    best_count = 0
    for _ in range(72):
        sample = rng.choice(len(candidate), size=3, replace=False)
        matrix = design[sample]
        if abs(np.linalg.det(matrix)) < .03:
            continue
        plane = np.linalg.solve(matrix, candidate[sample, 2])
        if np.linalg.norm(plane[:2]) > maximum_grade or abs(plane[2] - ground_height) > .22:
            continue
        inliers = np.abs(candidate[:, 2] - design @ plane) < .055
        count = int(inliers.sum())
        if count > best_count:
            best, best_count = inliers, count
    if best is None or best_count < max(18, len(candidate) * .2):
        return fallback
    plane = np.linalg.lstsq(design[best], candidate[best, 2], rcond=None)[0]
    if np.linalg.norm(plane[:2]) > maximum_grade or abs(plane[2] - ground_height) > .22:
        return fallback
    return plane


def obstacle_mask(points, *, ground_height=0.0, min_height=.15,
                  max_height=1.5, max_range=15.0):
    """Return an Nx3 cloud's boolean obstacle mask using only measured returns.

    The near-field threshold is ``min_height`` above the measured road profile. Beyond 5 m,
2.5 cm per additional meter accommodates gentle curvature and fit uncertainty.
That uncertainty allowance is capped at 25 cm. Upper returns are limited by
``max_height`` above the same plane. Invalid and out-of-range returns are ignored.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    plane = estimate_ground_plane(points, ground_height=ground_height)
    radius = np.linalg.norm(points[:, :2], axis=1)
    road = points[:, 0]*plane[0] + points[:, 1]*plane[1] + plane[2]
    residual = points[:, 2] - road
    # Trace low measured returns outward from the local plane. A single plane
    # cannot describe the far side of a shallow parking-lot depression. Each
    # narrow angular sector follows gradual changes without accepting a sudden
    # barrel-height rise as ground. No map or simulated position is consulted.
    finite = np.isfinite(points).all(axis=1)
    angle = np.arctan2(points[:, 1], points[:, 0])
    sector = np.floor((np.where(finite, angle, 0) + math.pi) / (math.pi / 18)).astype(int) % 36
    radial_bin = np.floor(np.where(finite, radius, 0) / .5).astype(int)
    for index in range(36):
        selected = finite & (sector == index) & (radius > .8) & (radius < max_range)
        if not selected.any():
            continue
        anchors_r, anchors_h = [0.0], [0.0]
        for distance_bin in np.unique(radial_bin[selected]):
            cell = selected & (radial_bin == distance_bin)
            if cell.sum() < 3:
                continue
            height = float(np.quantile(residual[cell], .15))
            distance = float(np.median(radius[cell]))
            previous_r, previous_h = anchors_r[-1], anchors_h[-1]
            # The first few meters must agree with the robust seed plane.
            if distance < 3.0 and abs(height) > .10:
                continue
            allowed_rise = .055 + .18 * min(distance - previous_r, 2.5)
            if abs(height - previous_h) > allowed_rise:
                continue
            anchors_r.append(distance)
            anchors_h.append(height)
        if len(anchors_r) >= 3:
            road[selected] += np.interp(radius[selected], anchors_r, anchors_h)
    height = points[:, 2] - road
    tolerance = np.minimum(.25, np.maximum(0., radius - 5.) * .025)
    return (np.isfinite(points).all(axis=1) & (radius > .8) & (radius < max_range)
            & (height > min_height + tolerance) & (height < max_height))
