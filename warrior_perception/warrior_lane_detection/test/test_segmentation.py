import math

import numpy as np

from warrior_lane_detection.segmentation import LaneSegmenter

W, H, F = 640, 480, 462.0
HEIGHT, PITCH = 1.18, math.radians(25)


def camera_pose():
    """optical (z fwd, x right, y down) -> base_footprint, pitched down."""
    optical_to_body = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    c, s = math.cos(PITCH), math.sin(PITCH)
    pitch = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return pitch @ optical_to_body, np.array([0.1, 0.0, HEIGHT])


def render(stripe_y=(-0.05, 0.05), post=None):
    """Flat grey ground with a white stripe along x; optional raised white post."""
    seg = LaneSegmenter()
    seg.set_intrinsics(F, F, W / 2, H / 2, W, H)
    rotation, translation = camera_pose()
    rx, ry = seg._rays[..., 0], seg._rays[..., 1]
    d = np.stack([rx, ry, np.ones_like(rx)], -1) @ rotation.T
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(d[..., 2] < 0, -HEIGHT / d[..., 2], np.inf)
    x, y = translation[0] + d[..., 0] * t, translation[1] + d[..., 1] * t
    rgb = np.full((H, W, 3), 110, np.uint8)
    rgb[~np.isfinite(t)] = (150, 190, 230)  # sky
    stripe = np.isfinite(t) & (y >= stripe_y[0]) & (y <= stripe_y[1]) & (x < 9)
    rgb[stripe] = 240
    depth = t.astype(np.float32)
    if post is not None:  # white box face at x=post, |y|<0.3, z in [0.3, 0.8]
        tp = (post - translation[0]) / d[..., 0]
        zp = translation[2] + d[..., 2] * tp
        yp = translation[1] + d[..., 1] * tp
        face = (tp > 0) & (tp < t) & (np.abs(yp - 1.0) < 0.3) & (zp > 0.3) & (zp < 0.8)
        rgb[face] = 245
        depth[face] = tp[face]
    return seg, rgb, depth, rotation, translation, stripe


def test_ground_stripe_detected_and_raised_white_rejected():
    seg, rgb, depth, rotation, translation, stripe = render(post=3.0)
    lane, ground = seg.segment(rgb, rotation, translation, depth)
    near = stripe & ground
    recall = (lane[near] > 0).mean()
    assert recall > 0.8, recall
    false_positive = (lane > 0) & ~stripe
    assert false_positive.sum() < 0.05 * near.sum(), false_positive.sum()


def test_flat_ground_fallback_without_depth():
    seg, rgb, _, rotation, translation, stripe = render()
    lane, ground = seg.segment(rgb, rotation, translation, None)
    assert not ground[:150].all()  # sky / far rows rejected
    assert (lane[stripe & ground] > 0).mean() > 0.8


def test_uniform_bright_pavement_is_not_lane():
    seg, rgb, depth, rotation, translation, _ = render(stripe_y=(50, 51))
    rgb[rgb == 110] = 200  # bright but featureless ground
    lane, _ = seg.segment(rgb, rotation, translation, depth)
    assert (lane > 0).sum() == 0


def test_ground_points_lie_on_the_stripe():
    seg, rgb, depth, rotation, translation, _ = render(post=3.0)
    lane, ground = seg.segment(rgb, rotation, translation, depth)
    points = seg.ground_points(lane, 0.05)
    assert len(points) > 50
    assert np.all(np.abs(points[:, 1]) < 0.1), points[:, 1].min()
    assert points[:, 0].min() > 0.4 and points[:, 0].max() < 8.1
    free = seg.ground_points(ground, 0.2)
    assert len(free) > len(points) / 4
    assert np.unique(np.floor(free / 0.2), axis=0).shape[0] == len(free)


def test_near_pothole_disc_is_not_absorbed_into_background():
    # A 0.6 m white disc 1.8 m ahead spans >100 px; a small top-hat kernel
    # treats it as background and drops it (seen in the P37 course run).
    seg, rgb, depth, rotation, translation, _ = render(stripe_y=(50, 51))
    seg.ground_mask(rotation, translation, depth)
    x, y = seg._xy
    with np.errstate(invalid='ignore'):
        disc = np.hypot(x - 1.8, y) < 0.3
    rgb[disc] = 240
    lane, ground = seg.segment(rgb, rotation, translation, depth)
    assert (lane[disc & ground] > 0).mean() > 0.9


def test_bright_pavement_between_shadows_is_not_lane():
    # Lamp-post / wire shadows crossing the course (Gazebo IGVC run): the
    # sunlit strip between two shadow bands is narrower than the top-hat
    # kernel, the opening drops to shadow level and the plain pavement
    # passed the contrast test as one large "lane" blob.
    seg, rgb, depth, rotation, translation, stripe = render()
    rgb[stripe] = 225
    seg.ground_mask(rotation, translation, depth)
    x, y = seg._xy
    with np.errstate(invalid='ignore'):
        along = x + 0.5 * y  # diagonal bands, like a pole shadow
        shadow = ((along > 2.0) & (along < 2.15)) | ((along > 2.7) & (along < 3.1))
        strip = (along > 2.15) & (along < 2.7) & ~stripe
    rgb[shadow] = (rgb[shadow] * 0.5).astype(np.uint8)
    rgb[~stripe & ~shadow & np.isfinite(depth)] = 172  # bright concrete, as measured
    lane, ground = seg.segment(rgb, rotation, translation, depth)
    assert strip.sum() > 1000
    assert (lane[strip & ground] > 0).mean() < 0.02, (lane[strip & ground] > 0).mean()
    lit_stripe = stripe & ground & ~shadow
    assert (lane[lit_stripe] > 0).mean() > 0.8


def test_white_too_big_for_the_contrast_test_is_never_cleared():
    """A pothole close up fills the view: not lane, but not free ground either.

    Its cells used to go out as free and erased the pothole the costmap had
    marked from farther away, just before the robot reached it.
    """
    seg, rgb, depth, rotation, translation, stripe = render(stripe_y=(-1.5, 1.5))
    lane, ground = seg.segment(rgb, rotation, translation, depth)
    white = stripe & ground
    assert (lane[white] > 0).mean() < 0.5          # mostly fails the contrast test
    res = 0.2
    free = seg.free_points(ground, res)
    x, y = free[:, 0], free[:, 1]
    inside = (np.abs(y) < 1.5 - res) & (x < 8.5)
    assert len(free) and not inside.any(), free[inside][:5]
    old = seg.ground_points(ground, res)           # the previous clearing cloud
    assert ((np.abs(old[:, 1]) < 1.5 - res) & (old[:, 0] < 8.5)).any()


def _cells(mask_fn, res=0.05, extent=((0, 8), (-3, 3))):
    xs = np.arange(*extent[0], res) + res / 2
    ys = np.arange(*extent[1], res) + res / 2
    gx, gy = np.meshgrid(xs, ys)
    keep = mask_fn(gx, gy)
    return np.stack([gx[keep], gy[keep]], -1).astype(np.float32)


def test_split_potholes_by_blob_shape():
    from warrior_lane_detection.segmentation import split_potholes
    res = 0.05
    line = lambda x, y: (np.abs(y - 1.83) < 0.05) & (x > 1) & (x < 7)          # noqa: E731
    short_line = lambda x, y: (np.abs(y + 1.83) < 0.05) & (x > 2) & (x < 2.6)  # noqa: E731
    disc = lambda x, y: np.hypot(x - 4, y) < 0.3                               # noqa: E731
    half_disc = lambda x, y: (np.hypot(x - 1.5, y) < 0.3) & (x > 1.5)          # noqa: E731
    pts = np.concatenate([_cells(f) for f in (line, short_line, disc, half_disc)])
    lines, potholes = split_potholes(pts, res)
    assert len(lines) + len(potholes) == len(pts)
    assert np.all(np.abs(np.abs(lines[:, 1]) - 1.83) < 0.06)     # both lines, whole
    assert len(lines) == len(_cells(line)) + len(_cells(short_line))
    assert np.all(np.abs(potholes[:, 1]) < 0.31)                  # disc + seen half disc


def test_split_potholes_handles_empty_and_tiny_input():
    from warrior_lane_detection.segmentation import split_potholes
    lines, potholes = split_potholes(np.empty((0, 2), np.float32), 0.05)
    assert lines.shape == (0, 2) and potholes.shape == (0, 2)
    lines, potholes = split_potholes(np.array([[1.0, 1.0]], np.float32), 0.05)
    assert len(lines) == 1 and len(potholes) == 0
