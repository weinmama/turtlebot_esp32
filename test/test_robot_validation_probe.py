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
        with self.assertRaisesRegex(probe.SafetyViolation, "target exceeds 130 RPM"):
            self.guard.observe([130.1, 0.0, 0.0, 0.0, 40.0, 0.0, 0.1], 1.0)

    def test_accepts_feedback_until_140_rpm_guard(self):
        self.guard.observe([130.0, 0.0, 140.0, 0.0, 255.0, 0.0, 0.1], 1.0)
        with self.assertRaisesRegex(probe.SafetyViolation, "more than 3 encoder samples"):
            for now in (1.1, 1.2, 1.3, 1.4):
                self.guard.observe([130.0, 0.0, 140.1, 0.0, 255.0, 0.0, 0.1], now)

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
    def test_ten_minute_profile_is_bounded_and_accepted(self):
        self.assertEqual(probe.parse_args(["--profile", "mixed", "--duration", "600"]).duration, 600)
        with self.assertRaises(SystemExit):
            probe.parse_args(["--profile", "mixed", "--duration", "600.1"])

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


class RpmSweepProfileTests(unittest.TestCase):
    def test_three_minute_sweep_requests_stages_through_100_rpm(self):
        expected = probe.RPM_SWEEP_RPM
        for index, rpm in enumerate(expected):
            phase, linear_x, angular_z, requested_rpm = probe.rpm_sweep_command(
                index * probe.RPM_SWEEP_PHASE_S + 1.0)
            self.assertEqual(requested_rpm, rpm)
            self.assertEqual(phase, f"rpm_{rpm:g}")
            self.assertAlmostEqual(
                linear_x * 60.0 / probe.WHEEL_CIRCUMFERENCE_M, rpm)
            self.assertEqual(angular_z, 0.0)
            self.assertLessEqual(abs(linear_x), 100.0 * probe.WHEEL_CIRCUMFERENCE_M / 60.0)

    def test_each_sweep_stage_ends_in_zero_command(self):
        for index, rpm in enumerate(probe.RPM_SWEEP_RPM):
            phase, vx, wz, requested_rpm = probe.rpm_sweep_command(
                index * probe.RPM_SWEEP_PHASE_S + probe.RPM_SWEEP_PHASE_S - 0.1)
            self.assertEqual(phase, f"rpm_{rpm:g}_pause")
            self.assertEqual((vx, wz), (0.0, 0.0))
            self.assertEqual(requested_rpm, rpm)


class RpmHoldProfileTests(unittest.TestCase):
    def test_hold_converts_130_rpm_without_exceeding_limit(self):
        phase, vx, wz = probe.rpm_hold_command(130.0)
        self.assertEqual(phase, "rpm_130")
        self.assertEqual(wz, 0.0)
        self.assertAlmostEqual(vx * 60 / probe.WHEEL_CIRCUMFERENCE_M, 130.0)
        with self.assertRaises(SystemExit):
            probe.parse_args(["--profile", "rpm_hold", "--wheel-rpm", "130.1", "--duration", "5"])
        self.assertEqual(
            probe.parse_args(["--profile", "rpm_hold", "--wheel-rpm", "130", "--duration", "120"]).duration,
            120,
        )
        with self.assertRaises(SystemExit):
            probe.parse_args(["--profile", "rpm_hold", "--wheel-rpm", "130", "--duration", "120.1"])


class JoystickReproTests(unittest.TestCase):
    def test_profile_switch_and_fixed_duration(self):
        self.assertEqual(probe.joystick_repro_command(0), ("full_forward", 0.356047, 0.0))
        self.assertEqual(probe.joystick_repro_command(30), ("full_forward_right", 0.356047, -3.56047))
        self.assertEqual(probe.parse_args(["--profile", "joystick_repro", "--duration", "60"]).duration, 60)
        self.assertEqual(probe.parse_args(["--profile", "joystick_forward", "--duration", "60"]).duration, 60)
        with self.assertRaises(SystemExit):
            probe.parse_args(["--profile", "joystick_repro", "--duration", "61"])


class ManeuverSweepTests(unittest.TestCase):
    def test_exact_three_minute_profile_covers_speeds_and_directions(self):
        args = probe.parse_args(["--profile", "maneuver_sweep", "--duration", "180"])
        self.assertEqual(args.duration, 180)
        self.assertEqual(probe.MANEUVER_SWEEP_S, 180)
        labels = {phase[0] for phase in probe.MANEUVER_PHASES}
        self.assertTrue(any(label.startswith("forward_") for label in labels))
        self.assertTrue(any(label.startswith("reverse_") for label in labels))
        self.assertTrue(any(label.startswith("curve_left") for label in labels))
        self.assertTrue(any(label.startswith("curve_right") for label in labels))
        self.assertTrue(any(label.startswith("pivot_left") for label in labels))
        self.assertTrue(any(label.startswith("pivot_right") for label in labels))
        self.assertTrue(any("100rpm" in label for label in labels))

    def test_five_minute_profile_repeats_the_tested_sweep(self):
        args = probe.parse_args(["--profile", "maneuver_sweep", "--duration", "300"])
        self.assertEqual(args.duration, 300)
        self.assertEqual(probe.maneuver_sweep_command(30.0),
                         probe.maneuver_sweep_command(210.0))
        with self.assertRaises(SystemExit):
            probe.parse_args(["--profile", "maneuver_sweep", "--duration", "301"])

    def test_wheel_rpm_conversion_and_phase_brakes(self):
        label, linear, angular, left, right = probe.maneuver_sweep_command(30.0)
        self.assertEqual((label, left, right), ("forward_100rpm", 100.0, 100.0))
        self.assertAlmostEqual(linear * 60 / probe.WHEEL_CIRCUMFERENCE_M, 100.0)
        self.assertEqual(angular, 0.0)
        label, linear, angular, left, right = probe.maneuver_sweep_command(44.6)
        self.assertEqual((label, linear, angular), ("forward_100rpm_pause", 0.0, 0.0))
        label, linear, angular, left, right = probe.maneuver_sweep_command(150.0)
        self.assertEqual((label, left, right), ("pivot_left_60rpm", -60.0, 60.0))
        self.assertAlmostEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)
        label, linear, angular, left, right = probe.maneuver_sweep_command(165.0)
        self.assertEqual((label, left, right), ("pivot_right_80rpm", 80.0, -80.0))
        self.assertAlmostEqual(linear, 0.0)
        self.assertLess(angular, 0.0)

    def test_all_profile_commands_stay_below_130_rpm_motor_limit(self):
        for phase_name, left_rpm, right_rpm, duration in probe.MANEUVER_PHASES:
            for offset in (0.1, duration - 0.6):
                label, linear, angular, _, _ = probe.maneuver_sweep_command(
                    sum(item[3] for item in probe.MANEUVER_PHASES[:probe.MANEUVER_PHASES.index(
                        (phase_name, left_rpm, right_rpm, duration))]) + offset)
                implied_left = (linear - angular * probe.WHEEL_BASE_M / 2) * 60 / probe.WHEEL_CIRCUMFERENCE_M
                implied_right = (linear + angular * probe.WHEEL_BASE_M / 2) * 60 / probe.WHEEL_CIRCUMFERENCE_M
                if label.endswith("_pause"):
                    self.assertEqual((linear, angular), (0.0, 0.0))
                else:
                    self.assertLessEqual(abs(implied_left), 100.0 + 1e-6)
                    self.assertLessEqual(abs(implied_right), 100.0 + 1e-6)


if __name__ == "__main__":
    unittest.main()
