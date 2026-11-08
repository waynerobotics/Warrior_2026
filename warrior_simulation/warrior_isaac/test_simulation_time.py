"""Isaac startup and pause regressions that do not require Kit, ROS, or a GPU."""

import json
import contextlib
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock

from run_robot import SimulationTime, report_simulation_failure


class RobotLaunchValidationTest(unittest.TestCase):
    def test_failure_is_reported_before_fast_shutdown(self):
        app = Mock()
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            try:
                raise RuntimeError('Course stage could not open')
            except RuntimeError:
                report_simulation_failure(app)
        app.app.post_quit.assert_called_once_with(1)
        self.assertIn('RuntimeError: Course stage could not open', output.getvalue())

    def test_normal_shutdown_does_not_request_failure_exit(self):
        app = Mock()
        report_simulation_failure(app)
        app.app.post_quit.assert_not_called()

    def test_missing_course_fails_before_loading_isaac(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / 'missing course.usda'
            result = subprocess.run(
                [sys.executable, str(Path(__file__).with_name('run_robot.py')),
                 '--stage', str(missing)], text=True, capture_output=True,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn(f'Course stage does not exist: {missing}', result.stderr)
        self.assertNotIn('Traceback', result.stderr)


class SimulationTimeTest(unittest.TestCase):
    def test_pause_does_not_generate_duplicate_sensor_samples(self):
        timing = SimulationTime(0.0, 100.0)
        self.assertTrue(timing.running(0.1, 100.1, playing=True, stopped=False))
        self.assertAlmostEqual(timing.sample_interval(0.1), 0.1)

        # A stationary robot's antenna position and timestamp stay unchanged
        # while GUI updates continue. No zero-duration velocity is sampled.
        for wall_time in (100.2, 110.0, 120.0):
            self.assertFalse(timing.running(0.1, wall_time, playing=False, stopped=False))
            self.assertEqual(timing.last_stamp, 0.1)
        self.assertTrue(timing.running(0.1, 120.1, playing=True, stopped=False))
        self.assertIsNone(timing.sample_interval(0.1))
        interval = timing.sample_interval(0.2)
        velocity = [(value - value) / interval for value in (1.0, 2.0, 3.0)]
        self.assertEqual(json.dumps(velocity, allow_nan=False), '[0.0, 0.0, 0.0]')
        self.assertAlmostEqual(interval, 0.1)

    def test_resume_rebases_realtime_pacing(self):
        timing = SimulationTime(2.0, 102.0)
        self.assertFalse(timing.running(2.0, 103.0, playing=False, stopped=False))
        self.assertTrue(timing.running(2.0, 120.0, playing=True, stopped=False))
        self.assertAlmostEqual(timing.wall_start + 2.1 - 120.0, 0.1)
        self.assertTrue(timing.running(2.1, 120.1, playing=True, stopped=False))
        self.assertAlmostEqual(timing.wall_start, 118.0)

    def test_pause_during_step_preserves_last_sensor_timestamp(self):
        timing = SimulationTime(3.0, 103.0)
        self.assertTrue(timing.running(3.0, 103.0, playing=True, stopped=False))
        self.assertFalse(timing.running(3.01, 103.01, playing=False, stopped=False))
        self.assertEqual(timing.last_stamp, 3.0)
        self.assertTrue(timing.running(3.01, 110.0, playing=True, stopped=False))
        self.assertAlmostEqual(timing.sample_interval(3.02), 0.02)

    def test_timeline_stop_requires_consistent_ros_restart(self):
        timing = SimulationTime(5.0, 105.0)
        with self.assertRaisesRegex(RuntimeError, 'relaunch the ROS simulation'):
            timing.running(0.0, 106.0, playing=False, stopped=True)

    def test_clock_rewind_is_rejected_before_control_or_sampling(self):
        timing = SimulationTime(5.0, 105.0)
        with self.assertRaisesRegex(RuntimeError, 'relaunch the ROS simulation'):
            timing.running(4.0, 106.0, playing=True, stopped=False)
        with self.assertRaisesRegex(RuntimeError, 'relaunch the ROS simulation'):
            timing.sample_interval(4.0)


if __name__ == '__main__':
    unittest.main()
