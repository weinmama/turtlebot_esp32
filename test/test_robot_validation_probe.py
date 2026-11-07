import importlib.util
import math
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).parents[1] / "host" / "robot_validation_probe.py"
SPEC = importlib.util.spec_from_file_location("robot_validation_probe", MODULE_PATH)
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class MotorFeedbackGuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = probe.MotorFeedbackGuard()

    def test_accepts_plausible_feedback_and_zero_target_wheel(self):
        values = self.guard.observe([14.0, 0.0, 15.0, 0.0, 40.0, 0.0, 0.1], 1.0)
        self.assertEqual(values[0], 14.0)

    def test_tolerates_three_rpm_outliers_but_aborts_on_fourth_in_one_second(self):
        values = [14.0, 0.0, 209.0, 0.0, 40.0, 0.0, 0.1]
        for now in (1.0, 1.2, 1.4):
            self.guard.observe(values, now)
        with self.assertRaisesRegex(probe.SafetyViolation, "more than 3 encoder samples"):
            self.guard.observe(values, 1.6)

    def test_outlier_window_expires_and_counts_both_wheels_together(self):
        self.guard.observe([14.0, 0.0, 101.0, 0.0, 40.0, 0.0, 0.1], 1.0)
        self.guard.observe([14.0, 0.0, 0.0, -101.0, 40.0, 0.0, 0.1], 1.2)
        self.guard.observe([14.0, 0.0, 101.0, 0.0, 40.0, 0.0, 0.1], 1.4)
        self.guard.observe([14.0, 0.0, 0.0, 0.0, 40.0, 0.0, 0.1], 2.01)
        self.guard.observe([14.0, 0.0, 101.0, 0.0, 40.0, 0.0, 0.1], 2.1)

    def test_aborts_immediately_for_target_above_rpm_limit(self):
        with self.assertRaisesRegex(probe.SafetyViolation, "target exceeds 100 RPM"):
            self.guard.observe([100.1, 0.0, 0.0, 0.0, 40.0, 0.0, 0.1], 1.0)

    def test_aborts_immediately_above_pwm_limit(self):
        with self.assertRaisesRegex(probe.SafetyViolation, "PWM exceeds 255"):
            self.guard.observe([14.0, 0.0, 14.0, 0.0, 256.0, 0.0, 0.1], 1.0)

    def test_aborts_after_500ms_without_feedback_on_requested_wheel(self):
        values = [14.0, 0.0, 0.0, 0.0, 60.0, 0.0, 0.1]
        self.guard.observe(values, 10.0)
        self.guard.observe(values, 10.49)
        with self.assertRaisesRegex(probe.SafetyViolation, "left wheel has no encoder response"):
            self.guard.observe(values, 10.5)

    def test_does_not_fault_for_zero_target_wheel(self):
        values = [14.0, 0.0, 14.0, 0.0, 40.0, 0.0, 0.1]
        for now in (1.0, 1.2, 1.4, 2.0):
            self.guard.observe(values, now)

    def test_feedback_resets_no_response_timer(self):
        self.guard.observe([14.0, 0.0, 0.0, 0.0, 60.0, 0.0, 0.1], 1.0)
        self.guard.observe([14.0, 0.0, 12.0, 0.0, 42.0, 0.0, 0.1], 1.4)
        self.guard.observe([14.0, 0.0, 0.0, 0.0, 60.0, 0.0, 0.1], 1.5)
        self.guard.observe([14.0, 0.0, 0.0, 0.0, 60.0, 0.0, 0.1], 1.99)
        with self.assertRaises(probe.SafetyViolation):
            self.guard.observe([14.0, 0.0, 0.0, 0.0, 60.0, 0.0, 0.1], 2.0)

    def test_rejects_invalid_values(self):
        with self.assertRaisesRegex(probe.SafetyViolation, "seven finite"):
            self.guard.observe([0.0] * 6, 1.0)
        with self.assertRaisesRegex(probe.SafetyViolation, "seven finite"):
            self.guard.observe([0.0, 0.0, math.nan, 0.0, 0.0, 0.0, 0.1], 1.0)
        with self.assertRaisesRegex(probe.SafetyViolation, "control interval"):
            self.guard.observe([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0)


class MixedProfileTests(unittest.TestCase):
    def test_profile_covers_curves_counterrotation_reverse_and_pause(self):
        expected = ["straight_forward", "curve_left", "curve_right",
                    "spin_left", "spin_right", "straight_reverse"]
        for index, phase in enumerate(expected):
            label, _, _ = probe.mixed_profile_command(index * 5.0 + 0.5)
            self.assertEqual(label, phase)
            pause, vx, wz = probe.mixed_profile_command(index * 5.0 + 4.9)
            self.assertTrue(pause.endswith("_pause"))
            self.assertEqual((vx, wz), (0.0, 0.0))

    def test_long_profile_varies_speed_by_cycle_without_exceeding_limits(self):
        at_start = probe.mixed_profile_command(0.5)
        next_cycle = probe.mixed_profile_command(30.5)
        third_cycle = probe.mixed_profile_command(60.5)
        self.assertAlmostEqual(at_start[1], 0.025 * 0.75)
        self.assertAlmostEqual(next_cycle[1], 0.025 * 0.9)
        self.assertAlmostEqual(third_cycle[1], 0.025)
        for cycle in range(10):
            for phase in range(6):
                _, vx, wz = probe.mixed_profile_command(cycle * 30 + phase * 5 + 1)
                self.assertLessEqual(abs(vx), 0.05)
                self.assertLessEqual(abs(wz), 0.3)


if __name__ == "__main__":
    unittest.main()
