"""IGVC mission sequencing: lanes -> no-man's-land GPS targets -> lanes -> finish."""

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from warrior_navigation.mission_logic import Mission, MissionParams  # noqa: E402

NML = [(0.0, 30.0), (0.0, 17.0), (0.0, 12.5), (0.0, 0.0)]   # entrance -> exit, heading south


def started():
    m = Mission(nml=NML)
    acts = m.begin(40.0, 0.0, math.pi / 2, 0.0)
    assert acts == [('heading', math.pi / 2), ('lane', True)]
    return m


def test_lane_following_until_near_the_entrance():
    m = started()
    assert m.update(30.0, 20.0, 5.0) == []
    acts = m.update(5.0, 35.0, 10.0)                 # 7 m from the entrance
    assert m.state == 'NML'
    assert acts[0] == ('lane', False)
    assert acts[1][:3] == ('goal', 0.0, 30.0)


def test_nml_targets_in_order_then_lane_with_exit_heading():
    m = started()
    m.update(5.0, 35.0, 10.0)
    for i in range(1, 4):
        acts = m.goal_finished(True, *NML[i - 1], 20.0 + i)
        assert acts[0][:3] == ('goal',) + NML[i]
    acts = m.goal_finished(True, *NML[3], 30.0)
    assert m.state == 'LANE' and m.nml_done
    assert acts[0][0] == 'heading' and abs(acts[0][1] - (-math.pi / 2)) < 1e-9
    assert acts[1] == ('lane', True)
    assert m.update(0.0, -15.0, 31.0) == []        # passes the entrance again? no re-entry
    m2_entry = [a for a in m.update(0.0, 29.0, 32.0) if a[0] == 'goal']
    assert m2_entry == [] and m.state == 'LANE'


def test_failed_target_is_retried_then_skipped():
    m = started()
    m.update(5.0, 35.0, 10.0)
    assert m.goal_finished(False, 0, 32, 11)[0][:3] == ('goal', 0.0, 30.0)   # retry 1
    assert m.goal_finished(False, 0, 32, 12)[0][:3] == ('goal', 0.0, 30.0)   # retry 2
    assert m.goal_finished(False, 0, 32, 13)[0][:3] == ('goal', 0.0, 17.0)   # give up -> next
    assert any('given up' in line for line in m.log)


def test_finish_only_after_a_lap_and_the_nml():
    m = Mission(nml=NML, params=MissionParams(min_lap_distance=50.0))
    m.begin(40.0, 0.0, math.pi / 2, 0.0)
    assert m.update(40.5, 0.5, 1.0) == []          # at the start, nothing driven yet
    m.update(5.0, 35.0, 10.0)                       # NML
    for i in range(4):
        m.goal_finished(True, *NML[i], 20.0 + i)
    for k, (x, y) in enumerate([(0, -20), (20, -20), (40, -10), (40.5, 0.5)]):
        acts = m.update(x, y, 40.0 + k)
    assert m.state == 'DONE'
    assert acts == [('lane', False), ('cancel',)]


def test_stall_in_lane_nudges_along_the_heading():
    m = started()
    assert m.update(40.0, 0.0, 5.0) == []
    acts = m.update(40.1, 0.0, 11.0)               # < 0.5 m in > 10 s
    assert len(acts) == 1 and acts[0][0] == 'goal'
    _, gx, gy, gyaw = acts[0]
    assert abs(gx - 40.1) < 1e-6 and abs(gy - 2.5) < 1e-6 and abs(gyaw - math.pi / 2) < 1e-9
    assert m.update(40.1, 0.0, 12.0) == []         # timer restarted after a nudge
