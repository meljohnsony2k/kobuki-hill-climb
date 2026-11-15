"""
Tests: the simulator's physics and sensors, and the report's validation rules as runtime checks.
Run from the repo root:  python -m unittest discover -v tests
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kobuki import world  # noqa: E402
from kobuki.statechart import FEATURES, SPEED_TEST, HillClimbStatechart  # noqa: E402
from kobuki.robot import Kobuki  # noqa: E402
from kobuki.run import run_course  # noqa: E402

CORE = {f: False for f in FEATURES}
EMPTY = world.Course(np.array(world.walls(), float), [])


def drive(bot, l, r, secs, course=EMPTY):
    for _ in range(int(secs / 0.005)):
        bot.step(l, r, course.segments, 0.005)


class TestWorldAndRobot(unittest.TestCase):
    def test_ramp_profile(self):
        self.assertEqual(world.height(0, 1.0), 0.0)
        self.assertAlmostEqual(world.height(0, world.FLAT[0] + 0.1), world.HEIGHT)
        self.assertEqual(world.height(world.RAMP_HALF_W + 0.1, world.FLAT[0] + 0.1), 0.0)   # off the side = cliff

    def test_matched_wheels_drive_straight(self):
        bot = Kobuki(np.random.default_rng(0), wheel_mismatch=0.0)
        drive(bot, 400, 400, 3.0)
        self.assertLess(abs(bot.x), 1e-6)
        self.assertGreater(bot.y, 1.2)

    def test_turn_slip_fools_encoders_not_gyro(self):
        bot = Kobuki(np.random.default_rng(0), turn_slip_bias=0.10)
        drive(bot, -300, 300, 1.0)
        drive(bot, 0, 0, 0.5)
        true = np.degrees(bot.psi)
        self.assertGreater(np.degrees(bot.enc_angle) - true, 5)       # encoders over read
        self.assertLess(abs(bot.read().gyro_heading - true), 3)       # gyro tracks the real turn

    def test_cliff_sensors_see_the_plateau_edge(self):
        bot = Kobuki(np.random.default_rng(0), x=world.RAMP_HALF_W - 0.12, y=world.FLAT[0] + 0.3,
                     heading=np.radians(-90))                         # facing the right hand edge
        s = bot.read()
        self.assertTrue(s.cliff_left or s.cliff_centre or s.cliff_right)

    def test_bumper_side(self):
        course = world.Course(np.array(world.walls() + [[-1, 1.0, 1, 1.0]], float), [])
        bot = Kobuki(np.random.default_rng(0), wheel_mismatch=0.0, y=0.6)
        drive(bot, 300, 300, 2.0, course)
        self.assertTrue(bot.bumps[1])                                 # wall straight ahead -> centre bumper


class TestStatecharts(unittest.TestCase):
    """The report's validation rules, checked on every run."""

    @classmethod
    def setUpClass(cls):
        cls.runs = {}
        for name, fk in (("full", {}), ("core", CORE)):
            rs = []
            for seed in range(20):
                course = world.make_course(np.random.default_rng(seed))
                rs.append(run_course(HillClimbStatechart(SPEED_TEST, **fk), course, seed))
            cls.runs[name] = rs

    def states(self, name):
        return [[s for _, s in r.states] for r in self.runs[name]]

    def test_ramp_entered_at_most_once(self):
        for name in self.runs:
            for st in self.states(name):
                self.assertLessEqual(st.count("Ramp"), 1, name)

    def test_no_turnabout(self):
        """Requirement: never rotate more than 180 deg on the spot."""
        for name, rs in self.runs.items():
            self.assertLessEqual(max(r.max_spin_deg for r in rs), 180, name)

    def test_side_step_only_after_drive(self):
        """Validation 1: no PDL/PDR straight after another PDL/PDR without passing through Drive."""
        for st in self.states("core"):
            last_pd = None
            for s in st:
                if s in ("PDL", "PDR"):
                    self.assertIsNone(last_pd)
                    last_pd = s
                elif s == "Drive":
                    last_pd = None

    def test_full_statechart_handles_random_hardware(self):
        ok = sum(r.outcome == "success" for r in self.runs["full"])
        self.assertGreaterEqual(ok, 18)

    def test_features_add_robustness(self):
        full = sum(r.outcome == "success" for r in self.runs["full"])
        core = sum(r.outcome == "success" for r in self.runs["core"])
        self.assertGreater(full, core)


class TestMotionCompensation(unittest.TestCase):
    def test_hard_start_is_not_a_ramp(self):
        """A full speed start on flat ground: the compensated slope estimate must stay under the ramp threshold."""
        bot = Kobuki(np.random.default_rng(1), wheel_mismatch=0.0)
        fsm = HillClimbStatechart(SPEED_TEST)
        peak = -1.0
        for k in range(60):
            s = bot.read()
            ax, _ = fsm.accel(s)
            peak = max(peak, ax)
            drive(bot, 700 if k < 40 else 0, 700 if k < 40 else 0, 0.05)
        self.assertLess(peak, HillClimbStatechart.RAMP_UP)


if __name__ == "__main__":
    unittest.main()
