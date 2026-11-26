"""White-lane segmentation for an RGB-D camera, independent of ROS.

A pixel is labelled lane when it is

1. bright and unsaturated (white paint, not orange/blue barrel colour),
2. brighter by a margin than its background: the larger of a local
   morphological opening and the median ground brightness of its image row
   band. The opening rejects uniformly bright pavement and glare; the band
   median stops shadows (lamp posts, wires, barrels) from dragging the
   opening down, which would otherwise label the plain pavement beside or
   between shadows as paint, and
3. on the ground: its 3-D point, from depth and the camera pose, lies within
   ``ground_tolerance`` of the base_footprint plane and within ``max_range``.
   This removes the white bands on barrels, the robot body and the sky.

Without a depth image the ground test falls back to a flat-ground ray
intersection, which still rejects everything above the horizon.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class LaneParams:
    min_brightness: int = 170       # 0-255 on the max(R,G,B) channel
    max_saturation: float = 0.25    # (max-min)/max, white paint is grey
    min_contrast: int = 30          # brighter than the local background
    tophat_kernel: int = 101        # px, wider than a near pothole disc (~0.6 m)
    band_rows: int = 16             # px, row band for the plain-ground brightness floor
    floor_window: int = 3           # bands sampled (centred) for each band's floor
    floor_percentile: float = 75.0  # of ground brightness; tolerates ~3/4 shadow cover
    ground_tolerance: float = 0.12  # m, |height above base_footprint plane|
    min_range: float = 0.5          # m, horizontal distance from base_footprint
    max_range: float = 8.0          # m, beyond this IPM/depth is unreliable
    min_area: int = 40              # px, drop smaller connected blobs
    open_kernel: int = 1            # px, 1 = off; >1 erases thin far lines
    pothole_max_length: float = 1.0  # m, longest extent of a pothole blob on the ground
    pothole_min_width: float = 0.25  # m, a lane line is ~0.1 m wide


class LaneSegmenter:
    def __init__(self, params=None):
        self.params = params or LaneParams()
        self._rays = None
        self._k = None
        self._xy = None  # (x, y) in base_footprint of the last segment() call
        self._whiteish = None  # paint-coloured pixels of the last white_mask() call

    def set_intrinsics(self, fx, fy, cx, cy, width, height):
        """Cache per-pixel optical rays (x/z, y/z) for the image size."""
        k = (fx, fy, cx, cy, width, height)
        if k == self._k:
            return
        u, v = np.meshgrid(np.arange(width, dtype=np.float32),
                           np.arange(height, dtype=np.float32))
        self._rays = np.stack([(u - cx) / fx, (v - cy) / fy], axis=-1)
        self._k = k

    def white_mask(self, rgb, ground=None):
        """Bright, unsaturated and locally brighter than the background.

        The background is a grey opening (erode, then dilate) at half
        resolution: a large kernel is needed so 0.6 m pothole discs near the
        robot are not absorbed into it, and half-res keeps that cheap.

        When ``ground`` is given the opening is masked: only ground pixels
        enter the erosion and only windows that contained ground enter the
        dilation. Otherwise a dark barrel or ramp drags the local background
        down and the plain pavement next to it passes the contrast test.

        Shadows are ground, so masking cannot exclude them: a bright strip
        between two shadows narrower than the kernel opens to shadow level.
        The background is therefore floored at the plain-pavement brightness
        at that range: the ``floor_percentile`` of ground brightness over
        ``floor_window`` neighbouring ``band_rows`` row bands. A high
        percentile and several bands keep the floor at sunlit pavement even
        where a wire or pole shadow fills most of one band; paint is a few
        percent of the ground, so it never sets the floor.
        """
        p = self.params
        rgb = rgb.astype(np.int16)
        high = rgb.max(axis=2)
        low = rgb.min(axis=2)
        saturation = (high - low) / np.maximum(high, 1)
        value = high.astype(np.uint8)
        size = (value.shape[1] // 2, value.shape[0] // 2)
        half = cv2.resize(value, size, interpolation=cv2.INTER_AREA)
        k = max(3, p.tophat_kernel // 2) | 1
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        if ground is None:
            opened = cv2.morphologyEx(half, cv2.MORPH_OPEN, kernel)
        else:
            on_ground = cv2.resize(ground.astype(np.uint8), size,
                                   interpolation=cv2.INTER_NEAREST).astype(bool)
            eroded = cv2.erode(np.where(on_ground, half, np.uint8(255)), kernel)
            eroded[~cv2.dilate(on_ground.astype(np.uint8), kernel).astype(bool)] = 0
            opened = cv2.dilate(eroded, kernel)
        background = cv2.resize(opened, (value.shape[1], value.shape[0]),
                                interpolation=cv2.INTER_LINEAR)
        if ground is not None:
            background = np.maximum(background, self.band_floor(value, ground))
        tophat = cv2.subtract(value, background)
        self._whiteish = (value >= p.min_brightness) & (saturation <= p.max_saturation)
        return self._whiteish & (tophat >= p.min_contrast)

    def band_floor(self, value, ground):
        """Plain-ground brightness per row band, as a (rows, 1) uint8 column."""
        p = self.params
        rows = max(1, p.band_rows)
        reach = rows * (max(1, p.floor_window) // 2)
        floor = np.zeros((value.shape[0], 1), np.uint8)
        for start in range(0, value.shape[0], rows):
            lo, hi = max(0, start - reach), start + rows + reach
            samples = value[lo:hi][ground[lo:hi]]
            # Too few ground pixels (horizon, robot body): no floor.
            if samples.size >= 4 * rows:
                floor[start:start + rows] = np.percentile(samples, p.floor_percentile)
        return floor

    def ground_mask(self, rotation, translation, depth=None):
        """Pixels whose 3-D point lies on the ground plane within range.

        ``rotation`` (3x3) and ``translation`` (3,) map optical-frame points
        into base_footprint (x forward, z up). ``depth`` is z-depth in metres
        with NaN/inf/0 for no return; None uses a flat-ground assumption.
        """
        p = self.params
        rx, ry = self._rays[..., 0], self._rays[..., 1]
        # Ray directions in base_footprint for unit optical z.
        dx = rotation[0, 0] * rx + rotation[0, 1] * ry + rotation[0, 2]
        dy = rotation[1, 0] * rx + rotation[1, 1] * ry + rotation[1, 2]
        dz = rotation[2, 0] * rx + rotation[2, 1] * ry + rotation[2, 2]
        if depth is None:
            # Intersect with z = 0; only rays pointing downwards hit it.
            with np.errstate(divide='ignore', invalid='ignore'):
                d = np.where(dz < -1.e-6, -translation[2] / dz, np.nan)
            height_ok = np.isfinite(d)
        else:
            d = depth.astype(np.float32)
            valid = np.isfinite(d) & (d > 0)
            z = translation[2] + dz * d
            height_ok = valid & (np.abs(z) <= p.ground_tolerance)
        x = translation[0] + dx * d
        y = translation[1] + dy * d
        self._xy = (x, y)
        with np.errstate(invalid='ignore'):
            horizontal = np.hypot(x, y)
            in_range = (horizontal >= p.min_range) & (horizontal <= p.max_range)
        return height_ok & in_range

    def ground_points(self, mask, resolution):
        """Ground (x, y) of ``mask`` pixels from the last segment() call.

        Points are thinned to one per ``resolution`` grid cell (cell centre)
        so a costmap receives hundreds of points instead of every pixel.
        Returns an (N, 2) float32 array in base_footprint.
        """
        if self._xy is None:
            return np.empty((0, 2), np.float32)
        keep = mask.astype(bool)
        xy = np.stack([self._xy[0][keep], self._xy[1][keep]], axis=-1)
        xy = xy[np.isfinite(xy).all(axis=1)]
        if not len(xy):
            return np.empty((0, 2), np.float32)
        cells = np.unique(np.floor(xy / resolution).astype(np.int64), axis=0)
        return ((cells + 0.5) * resolution).astype(np.float32)

    def free_points(self, ground, resolution):
        """Ground cells that are safe to clear lane memory with.

        Like ``ground_points(ground, ...)`` but without any cell that holds a
        paint-coloured (bright, unsaturated) pixel, whether or not it passed
        the contrast test. Close up, a pothole disc fills more of the image
        than the background kernel and fails that test; were its cells
        "free", they would erase the pothole the costmap remembered from
        farther away, just as the robot reaches it.
        """
        if self._whiteish is None or self._whiteish.shape != ground.shape:
            return self.ground_points(ground, resolution)
        plain = self.ground_points(ground & ~self._whiteish, resolution)
        painted = self.ground_points(ground & self._whiteish, resolution)
        if not len(plain) or not len(painted):
            return plain
        key = lambda a: np.floor(a / resolution).astype(np.int64)   # noqa: E731
        pk, wk = key(plain), key(painted)
        dtype = np.dtype([('i', np.int64), ('j', np.int64)])
        keep = ~np.isin(np.ascontiguousarray(pk).view(dtype).ravel(),
                        np.ascontiguousarray(wk).view(dtype).ravel())
        return plain[keep]

    def clean(self, mask):
        p = self.params
        mask = mask.astype(np.uint8)
        if p.open_kernel > 1:
            kernel = np.ones((p.open_kernel, p.open_kernel), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if p.min_area > 1:
            count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            keep = np.zeros(count, bool)
            keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= p.min_area
            mask = keep[labels].astype(np.uint8)
        return mask * 255

    def segment(self, rgb, rotation, translation, depth=None):
        """Return (lane mask uint8 0/255, ground mask bool)."""
        if self._rays is None or self._rays.shape[:2] != rgb.shape[:2]:
            raise ValueError('set_intrinsics() must match the image size first')
        ground = self.ground_mask(rotation, translation, depth)
        return self.clean(self.white_mask(rgb, ground) & ground), ground


def split_potholes(points, resolution, max_length=1.0, min_width=0.25):
    """Split ground paint cells into (lane lines, potholes).

    ``points`` are (N, 2) cell centres on a ``resolution`` grid (from
    ``ground_points``). Cells are grouped into 8-connected blobs (gaps of one
    cell bridged); a blob is a pothole when it is compact on the ground: no
    longer than ``max_length`` along its principal axis and at least
    ``min_width`` across it. A lane line is ~0.1 m wide, and any visible piece
    of it longer than ``max_length`` is a line whatever its width. Lines and
    potholes are costed differently (hard lane boundary vs. avoid-if-possible).
    """
    points = np.asarray(points, np.float32).reshape(-1, 2)
    if len(points) < 2:
        return points, np.empty((0, 2), np.float32)
    cells = np.floor(points / resolution).astype(np.int64)
    cells -= cells.min(axis=0) - 1
    grid = np.zeros(tuple(cells.max(axis=0) + 2), np.uint8)
    grid[cells[:, 0], cells[:, 1]] = 1
    grid = cv2.dilate(grid, np.ones((3, 3), np.uint8))
    _, labels = cv2.connectedComponents(grid, connectivity=8)
    blob = labels[cells[:, 0], cells[:, 1]]
    pothole = np.zeros(len(points), bool)
    for b in np.unique(blob):
        sel = blob == b
        xy = points[sel].astype(np.float64)
        if len(xy) < 3:
            continue
        centred = xy - xy.mean(axis=0)
        _, vecs = np.linalg.eigh(np.cov(centred.T))
        proj = centred @ vecs                     # columns: minor, major axis
        width, length = np.ptp(proj, axis=0) + resolution
        if length <= max_length and width >= min_width:
            pothole[sel] = True
    return points[~pothole], points[pothole]


def overlay(rgb, lane, ground):
    """Debug view: dim non-ground pixels, paint detected lane green."""
    out = rgb.copy()
    out[~ground] = (out[~ground] * 0.35).astype(np.uint8)
    out[lane > 0] = (0, 255, 0)
    return out
