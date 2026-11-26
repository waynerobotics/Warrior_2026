"""IGVC AutoNav mission sequencing, ROS-free so it can be unit tested.

    LANE --(within enter_radius of the first NML target)--> NML
    NML  --(every NML target reached or given up)--------> LANE (exit heading)
    LANE --(after min_lap_distance, back near start)-----> DONE

Lane sections have no waypoints: ``lane_follower`` picks goals there. In the
no-man's-land the official GPS targets are driven one by one. Each call
returns a list of actions for the ROS node to execute:

    ('lane', bool)          enable / disable lane following
    ('heading', yaw)        re-seed lane following's travel heading
    ('goal', x, y, yaw)     send a Nav2 goal (NML target or a nudge)
    ('cancel',)             cancel Nav2 goals (mission finished)
"""

from dataclasses import dataclass, field
import math


@dataclass
class MissionParams:
    enter_radius: float = 10.0       # m to the NML entrance target
    finish_radius: float = 3.0       # m to the start pose
    min_lap_distance: float = 120.0  # m driven before a finish can count
    goal_retries: int = 2            # extra attempts per NML target
    stall_time: float = 10.0         # s without progress in LANE -> nudge
    stall_distance: float = 0.5      # m that counts as progress
    nudge_distance: float = 2.5      # m along the travel heading


@dataclass
class Mission:
    nml: list                        # [(x, y), ...] in the global frame, in order
    params: MissionParams = field(default_factory=MissionParams)
    state: str = 'WAIT'
    start: tuple = None
    last: tuple = None
    travelled: float = 0.0
    index: int = 0
    attempts: int = 0
    nml_done: bool = False
    heading: float = None
    stall_ref: tuple = None          # (x, y, t) progress reference in LANE
    log: list = field(default_factory=list)

    def _goal(self, i):
        x, y = self.nml[i]
        if i + 1 < len(self.nml):
            nx, ny = self.nml[i + 1]
        else:
            px, py = self.nml[i - 1] if i else (x - 1, y)
            nx, ny = 2 * x - px, 2 * y - py
        return ('goal', x, y, math.atan2(ny - y, nx - x))

    def _exit_heading(self):
        (ax, ay), (bx, by) = self.nml[-2], self.nml[-1]
        return math.atan2(by - ay, bx - ax)

    def _enter(self, state, note):
        self.log.append(f'{self.state} -> {state}: {note}')
        self.state = state

    def begin(self, x, y, yaw, t):
        self.start = self.last = (x, y)
        self.heading = yaw
        self.stall_ref = (x, y, t)
        self._enter('LANE', 'mission started')
        return [('heading', yaw), ('lane', True)]

    def update(self, x, y, t):
        """Call periodically with the robot pose and time (s)."""
        if self.state in ('WAIT', 'DONE'):
            return []
        self.travelled += math.dist((x, y), self.last)
        self.last = (x, y)
        p = self.params
        if self.state == 'LANE':
            if not self.nml_done and math.dist((x, y), self.nml[0]) < p.enter_radius:
                self._enter('NML', f'{math.dist((x, y), self.nml[0]):.1f} m from the entrance')
                self.index, self.attempts = 0, 0
                return [('lane', False), self._goal(0)]
            if (self.nml_done and self.travelled >= p.min_lap_distance
                    and math.dist((x, y), self.start) < p.finish_radius):
                self._enter('DONE', f'lap complete after {self.travelled:.0f} m')
                return [('lane', False), ('cancel',)]
            sx, sy, st = self.stall_ref
            if math.dist((x, y), (sx, sy)) > p.stall_distance:
                self.stall_ref = (x, y, t)
            elif t - st > p.stall_time:
                # Lane following found nothing to drive to (e.g. just out of
                # the no-man's-land before the lines are in view): nudge on.
                self.stall_ref = (x, y, t)
                h = self.heading
                self.log.append(f'LANE stalled {t - st:.0f} s: nudge {p.nudge_distance} m')
                return [('goal', x + p.nudge_distance * math.cos(h),
                         y + p.nudge_distance * math.sin(h), h)]
        return []

    def goal_finished(self, success, x, y, t):
        """Result of the Nav2 goal sent for the current NML target."""
        if self.state != 'NML':
            return []
        if not success and self.attempts < self.params.goal_retries:
            self.attempts += 1
            self.log.append(f'NML target {self.index + 1} failed, retry {self.attempts}')
            return [self._goal(self.index)]
        if not success:
            self.log.append(f'NML target {self.index + 1} given up')
        self.index += 1
        self.attempts = 0
        if self.index < len(self.nml):
            return [self._goal(self.index)]
        self.nml_done = True
        self.heading = self._exit_heading()
        self.stall_ref = (x, y, t)
        self._enter('LANE', 'no-man\'s-land done, lane following from the exit')
        return [('heading', self.heading), ('lane', True)]
