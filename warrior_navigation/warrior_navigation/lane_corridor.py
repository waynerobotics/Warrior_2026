"""Lane following without waypoints: pick a reachable goal ahead in the lane.

IGVC lane sections have no GPS waypoints, so the robot must choose its own
targets. This module is ROS-free so it can be unit tested:

``LaneMemory``
    Remembers, in the global (map) frame, which grid cells hold lane paint
    and which ground cells the camera has seen. Fed with the two flat clouds
    from ``warrior_lane_detection`` (paint cells and visible-ground cells).
    Visible ground without paint clears old paint, exactly like the Nav2
    LaneLayer; ground the camera never saw (its blind zone) is not cleared.

``CorridorPlanner``
    Fans candidate goals ahead of the robot and keeps only those that are
    (1) on seen ground, (2) not near paint, (3) reachable by a straight line
    that does not cross paint, and (4) below an obstacle cost limit. The
    best one maximises progress along the travel heading while staying
    centred between the lane lines and turning as little as possible.

Nav2 still plans the actual path (and avoids drums) to the chosen goal; this
only decides *where* to go next, the job GPS waypoints do elsewhere.
"""

from dataclasses import dataclass
import math

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class LaneMemory:
    """Sparse paint / seen-ground grids in the global frame."""

    def __init__(self, resolution=0.1):
        self.res = resolution
        self.paint = set()
        self.seen = set()

    def cell(self, x, y):
        return (int(math.floor(x / self.res)), int(math.floor(y / self.res)))

    def _cells(self, xy):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        return np.floor(xy / self.res).astype(np.int64)

    def update(self, free_xy, paint_xy, free_half, clear_radius=None):
        """Apply one camera frame: clear seen ground, then mark paint.

        ``free_xy`` are centres of visible-ground cells of half-size
        ``free_half``; they hold no paint-coloured pixel. ``paint_xy`` are
        paint points. Every cell the free squares touch counts as seen, but
        paint is erased only where its cell centre lies within
        ``clear_radius`` (default ``free_half / sqrt 2``, the cell's inscribed
        circle at any yaw) of a free centre. Clearing the whole square used to
        reach a line in the next cell; when that line then went into the
        blind zone in front of the robot, nothing re-marked it and lane
        following picked a goal across it.
        """
        free_xy = np.asarray(free_xy, dtype=float).reshape(-1, 2)
        if clear_radius is None:
            clear_radius = free_half / math.sqrt(2)
        if len(free_xy):
            span = int(math.ceil(free_half / self.res))
            lo = np.floor((free_xy - free_half) / self.res).astype(np.int64)
            for di in range(2 * span + 1):
                for dj in range(2 * span + 1):
                    cells = lo + (di, dj)
                    self.seen |= set(map(tuple, cells))
                    centre = (cells + 0.5) * self.res
                    inside = np.hypot(*(centre - free_xy).T) <= clear_radius
                    self.paint -= set(map(tuple, cells[inside]))
        if len(paint_xy):
            self.paint |= set(map(tuple, self._cells(paint_xy)))

    def is_seen(self, x, y):
        i, j = self.cell(x, y)
        return any((i + di, j + dj) in self.seen for di in (-1, 0, 1) for dj in (-1, 0, 1))

    def near_paint(self, x, y, radius):
        i, j = self.cell(x, y)
        r = int(math.ceil(radius / self.res))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if (i + di, j + dj) in self.paint and math.hypot(di, dj) * self.res <= radius:
                    return True
        return False

    def crosses_paint(self, x0, y0, x1, y1, skip=0.0):
        """True if the straight segment passes over a paint cell.

        The first ``skip`` metres are ignored: the robot may already be
        standing on a line it is crossing (e.g. lines its own blind zone).
        """
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(length / (self.res * 0.5))))
        for k in range(steps + 1):
            t = k / steps
            if t * length < skip:
                continue
            i, j = self.cell(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            # A diagonal line is an 8-connected chain of cells; checking the
            # 4-neighbours too stops a segment slipping between two corners.
            if ((i, j) in self.paint or (i + 1, j) in self.paint or (i - 1, j) in self.paint
                    or (i, j + 1) in self.paint or (i, j - 1) in self.paint):
                return True
        return False

    def paint_distance(self, x, y, dx, dy, max_dist):
        """Distance to the first paint cell along unit direction (dx, dy)."""
        steps = int(math.ceil(max_dist / (self.res * 0.5)))
        for k in range(1, steps + 1):
            d = k * self.res * 0.5
            if self.cell(x + d * dx, y + d * dy) in self.paint:
                return d
        return None


class Trail:
    """Where the robot has driven recently, so lane following never goes back.

    The travel heading alone is not enough: while the robot dithers in front
    of an obstacle, spins and replans, goals at the edge of the fan can walk
    the heading round until the best candidate lies behind it. A candidate
    near a trail point driven more than ``skip`` metres ago is rejected. Only
    the last ``keep`` metres are remembered, so the lap start is reachable
    again at the finish.
    """

    def __init__(self, spacing=0.5, radius=1.9, skip=2.5, keep=60.0):
        self.spacing, self.radius, self.skip, self.keep = spacing, radius, skip, keep
        self.points = []      # (x, y, odometer)
        self.odometer = 0.0

    def add(self, x, y):
        if self.points:
            px, py, _ = self.points[-1]
            step = math.hypot(x - px, y - py)
            if step < self.spacing:
                return
            self.odometer += step
        self.points.append((x, y, self.odometer))
        while self.points and self.odometer - self.points[0][2] > self.keep:
            self.points.pop(0)

    def visited(self, x, y):
        r2 = self.radius ** 2
        return any((x - px) ** 2 + (y - py) ** 2 < r2
                   for px, py, o in self.points if self.odometer - o > self.skip)


@dataclass
class CorridorParams:
    lane_width: float = 3.66
    min_dist: float = 2.5          # m, nearest candidate
    max_dist: float = 6.0          # m, camera depth is reliable to ~8 m
    dist_step: float = 0.5
    max_turn: float = math.radians(70)   # half-width of the candidate fan
    turn_step: float = math.radians(5)
    paint_clearance: float = 0.35  # m, candidate must be this far from paint
    crossing_skip: float = 0.6     # m, ignore paint this close to the robot
    cost_limit: int = 200          # reject candidates at/above this cost
    require_lane_support: bool = True   # candidate needs a line within a lane width
    w_progress: float = 1.0
    w_center: float = 1.5
    w_turn: float = 0.6            # per radian away from the travel heading
    w_cost: float = 1.0            # per full cost (252)


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float
    score: float
    progress: float
    center_error: float


class CorridorPlanner:
    def __init__(self, params=None):
        self.p = params or CorridorParams()

    def candidates(self, x, y, heading):
        p = self.p
        n_turn = int(round(p.max_turn / p.turn_step))
        dists = np.arange(p.min_dist, p.max_dist + 1e-9, p.dist_step)
        for k in range(-n_turn, n_turn + 1):
            a = heading + k * p.turn_step
            for d in dists:
                yield a, float(d), x + d * math.cos(a), y + d * math.sin(a)

    def center_error(self, memory, x, y, direction):
        """How far (m) the point is from the middle of the visible lane."""
        half = self.p.lane_width / 2
        nx, ny = -math.sin(direction), math.cos(direction)
        left = memory.paint_distance(x, y, nx, ny, self.p.lane_width)
        right = memory.paint_distance(x, y, -nx, -ny, self.p.lane_width)
        if left is not None and right is not None:
            return abs(left - right) / 2
        if left is not None:
            return abs(left - half)
        if right is not None:
            return abs(right - half)
        return None   # no lane line on either side: not inside a known lane

    def evaluate(self, memory, x, y, heading, cost_at=None, trail=None):
        """All feasible candidates, best first."""
        p = self.p
        out = []
        for a, d, cx, cy in self.candidates(x, y, heading):
            if not memory.is_seen(cx, cy):
                continue
            if trail is not None and trail.visited(cx, cy):
                continue
            if memory.near_paint(cx, cy, p.paint_clearance):
                continue
            if memory.crosses_paint(x, y, cx, cy, skip=p.crossing_skip):
                continue
            cost = cost_at(cx, cy) if cost_at else 0
            if cost is not None and cost >= p.cost_limit:
                continue
            turn = abs(wrap(a - heading))
            progress = d * math.cos(turn)
            centre = self.center_error(memory, cx, cy, a)
            if centre is None:
                # Past the end of the painted lane (e.g. the no-man's-land
                # entrance) nothing bounds the robot: never drive there on
                # lane following alone; the mission hands over to GPS.
                if p.require_lane_support:
                    continue
                centre = 0.0
            score = (p.w_progress * progress - p.w_center * centre
                     - p.w_turn * turn - p.w_cost * (cost or 0) / 252.0)
            out.append(Candidate(cx, cy, a, score, progress, centre))
        out.sort(key=lambda c: -c.score)
        return out

    def choose(self, memory, x, y, heading, cost_at=None, trail=None):
        ranked = self.evaluate(memory, x, y, heading, cost_at, trail)
        return ranked[0] if ranked else None

    def still_valid(self, memory, x, y, goal, cost_at=None, trail=None):
        """Re-check an active goal against newer observations."""
        if trail is not None and trail.visited(goal.x, goal.y):
            return False
        if memory.near_paint(goal.x, goal.y, self.p.paint_clearance):
            return False
        if memory.crosses_paint(x, y, goal.x, goal.y, skip=self.p.crossing_skip):
            return False
        cost = cost_at(goal.x, goal.y) if cost_at else 0
        return cost is None or cost < self.p.cost_limit
