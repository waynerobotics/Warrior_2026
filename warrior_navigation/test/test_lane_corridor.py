"""Goal selection for waypoint-free lane following (synthetic lanes)."""

import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from warrior_navigation.lane_corridor import (  # noqa: E402
    CorridorPlanner, LaneMemory, wrap)

HALF = 3.66 / 2
FREE_HALF = 0.2 / math.sqrt(2)


def grid(x0, x1, y0, y1, step=0.2):
    xs = np.arange(x0 + step / 2, x1, step)
    ys = np.arange(y0 + step / 2, y1, step)
    return np.array([(x, y) for x in xs for y in ys])


def line(points_fn, s0, s1, step=0.05):
    return np.array([points_fn(s) for s in np.arange(s0, s1, step)])


def straight_lane(length=20.0, left=True, right=True, seen_to=None):
    mem = LaneMemory()
    paint = []
    if left:
        paint.append(line(lambda s: (s, HALF), -2, length))
    if right:
        paint.append(line(lambda s: (s, -HALF), -2, length))
    seen_to = length if seen_to is None else seen_to
    mem.update(grid(-2, seen_to, -4, 4), np.vstack(paint) if paint else np.empty((0, 2)), FREE_HALF)
    return mem


def assert_inside_and_clear(mem, x, y, goal):
    assert not mem.crosses_paint(x, y, goal.x, goal.y, skip=0.6)
    assert not mem.near_paint(goal.x, goal.y, 0.35)


def test_straight_lane_goes_far_ahead_and_stays_centred():
    mem = straight_lane()
    goal = CorridorPlanner().choose(mem, 0.0, 0.0, 0.0)
    assert goal.x >= 5.5
    assert abs(goal.y) < 0.3
    assert_inside_and_clear(mem, 0.0, 0.0, goal)


def test_off_centre_robot_is_pulled_back_to_the_middle():
    mem = straight_lane()
    goal = CorridorPlanner().choose(mem, 0.0, 1.0, 0.0)
    assert goal.y < 0.6, goal
    assert_inside_and_clear(mem, 0.0, 1.0, goal)


def test_curve_turns_with_the_lane_and_never_cuts_a_line():
    mem = LaneMemory()
    radius = 8.0  # lane centre circles (0, 8) counter-clockwise from (0, 0)

    def on_arc(r):
        return lambda s: (r * math.sin(s / radius), radius - r * math.cos(s / radius))

    paint = np.vstack([line(on_arc(radius - HALF), -2, 14), line(on_arc(radius + HALF), -2, 14)])
    mem.update(grid(-3, 10, -4, 12), paint, FREE_HALF)
    goal = CorridorPlanner().choose(mem, 0.0, 0.0, 0.0)
    assert goal is not None
    assert goal.yaw > math.radians(10)            # turns left with the lane
    r = math.hypot(goal.x, goal.y - radius)
    assert abs(r - radius) < HALF - 0.35          # inside the lane
    assert_inside_and_clear(mem, 0.0, 0.0, goal)


def test_single_visible_line_keeps_half_a_lane_from_it():
    mem = straight_lane(left=False)
    goal = CorridorPlanner().choose(mem, 0.0, 0.0, 0.0)
    assert abs((goal.y + HALF) - HALF) < 0.4      # ~1.83 m from the right line


def test_drum_cost_ahead_is_avoided():
    mem = straight_lane()

    def cost_at(x, y):
        return 254 if 3.0 < x < 7.0 and abs(y) < 0.9 else 0

    goal = CorridorPlanner().choose(mem, 0.0, 0.0, 0.0, cost_at)
    assert goal is not None
    assert cost_at(goal.x, goal.y) < 200
    assert_inside_and_clear(mem, 0.0, 0.0, goal)


def test_goals_stay_on_ground_the_camera_has_seen():
    mem = straight_lane(seen_to=3.4)
    goal = CorridorPlanner().choose(mem, 0.0, 0.0, 0.0)
    assert goal is not None and goal.x <= 3.6


def test_no_candidate_when_nothing_was_seen():
    assert CorridorPlanner().choose(LaneMemory(), 0.0, 0.0, 0.0) is None


def test_never_turns_back_against_the_travel_heading():
    mem = straight_lane()
    mem.update(grid(-12, 0, -4, 4), line(lambda s: (s, HALF), -12, 0), FREE_HALF)
    for goal in CorridorPlanner().evaluate(mem, 0.0, 0.0, 0.0):
        assert abs(wrap(goal.yaw)) <= math.radians(70) + 1e-9


def test_memory_clears_seen_ground_but_keeps_blind_zone_paint():
    mem = straight_lane()
    # Later frame: ground x in [5, 20] is visible and the left line has gone.
    mem.update(grid(5, 20, -4, 4), line(lambda s: (s, -HALF), 5, 20), FREE_HALF)
    assert not mem.near_paint(10.0, HALF, 0.05)   # cleared where seen
    assert mem.near_paint(2.0, HALF, 0.05)        # blind zone remembered
    assert mem.near_paint(10.0, -HALF, 0.05)      # re-marked in view


def test_still_valid_rejects_goal_behind_new_paint():
    mem = straight_lane()
    planner = CorridorPlanner()
    goal = planner.choose(mem, 0.0, 0.0, 0.0)
    assert planner.still_valid(mem, 0.0, 0.0, goal)
    mem.update([], line(lambda s: (goal.x - 1.0, s), -3, 3), FREE_HALF)   # wall of paint
    assert not planner.still_valid(mem, 0.0, 0.0, goal)


def test_lane_end_stops_goals_instead_of_driving_off_course():
    """Sim run: past the no-man's-land entrance the lines end and the robot
    drove 70 m off the course on "keep going forward" alone."""
    mem = straight_lane(length=4.0)               # paint ends 4 m ahead
    mem.update(grid(4, 20, -4, 4), np.empty((0, 2)), FREE_HALF)   # open ground beyond
    planner = CorridorPlanner()
    for goal in planner.evaluate(mem, 0.0, 0.0, 0.0):
        assert goal.x < 4.0 + HALF                # never beyond line support
    assert planner.choose(mem, 9.0, 0.0, 0.0) is None   # already past the end


def test_unsupported_open_ground_allowed_only_when_requested():
    from warrior_navigation.lane_corridor import CorridorParams
    mem = LaneMemory()
    mem.update(grid(-2, 20, -4, 4), np.empty((0, 2)), FREE_HALF)
    assert CorridorPlanner().choose(mem, 0.0, 0.0, 0.0) is None
    assert CorridorPlanner(CorridorParams(require_lane_support=False)).choose(mem, 0.0, 0.0, 0.0)


def test_trail_rejects_ground_already_driven():
    from warrior_navigation.lane_corridor import Trail
    t = Trail(spacing=0.5, radius=1.9, skip=2.5, keep=60.0)
    for k in range(21):                    # drove 0 -> 10 m along x
        t.add(0.5 * k, 0.0)
    assert t.visited(2.0, 1.8)             # well behind, at the lane edge: rejected
    assert not t.visited(9.0, 0.0)         # driven < skip ago: still allowed
    assert not t.visited(13.0, 0.0)        # ahead
    for k in range(21, 160):               # 70 m more: start is forgotten
        t.add(0.5 * k, 0.0)
    assert not t.visited(0.0, 0.0)


def test_planner_never_picks_a_goal_behind_on_the_trail():
    from warrior_navigation.lane_corridor import CorridorParams, Trail
    mem = straight_lane()
    planner = CorridorPlanner(CorridorParams(max_turn=math.pi))
    trail = Trail()
    for k in range(21):
        trail.add(0.5 * k, 0.0)
    # Heading has drifted to point back down the lane.
    best = planner.choose(mem, 10.0, 0.0, math.pi, trail=trail)
    assert best is None or best.x > 10.0 - 0.5


def test_free_cell_beside_a_line_does_not_erase_it():
    """Regression: a plain free cell next to a line used to clear the line.

    Once the line was in the blind zone in front of the robot nothing marked
    it again, and lane following chose a goal across it.
    """
    mem = LaneMemory()
    line_pts = np.array([[5.05, y] for y in np.arange(0.0, 4.0, 0.05)])
    mem.update(np.empty((0, 2)), line_pts, FREE_HALF)
    # Neighbouring 0.2 m free cells, centres 0.12-0.2 m from the line.
    free = np.array([[x, y] for x in (5.17, 4.9, 5.25) for y in np.arange(0.1, 4.0, 0.2)])
    mem.update(free, np.empty((0, 2)), FREE_HALF)
    assert mem.crosses_paint(4.0, 2.0, 6.5, 2.0, skip=0.0)
    assert mem.is_seen(5.17, 2.0)
    # A free cell centred on the paint (seen without paint) still clears it.
    mem.update(np.array([[5.05, y] for y in np.arange(0.1, 4.0, 0.1)]), np.empty((0, 2)), FREE_HALF)
    assert not mem.crosses_paint(4.0, 2.0, 6.5, 2.0, skip=0.0)
