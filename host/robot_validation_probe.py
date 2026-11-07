#!/usr/bin/env python3
"""Run one bounded, guarded TwistStamped probe against the Pico-ROS robot.

Stream this file into the backend container with `python3 -` so it uses the
robot's ROS environment. It never starts a joystick or changes control mode.
"""

import argparse
from collections import deque
import json
import math
import time
import uuid


class SafetyViolation(RuntimeError):
    pass


MIXED_PROFILE_PHASES = (
    ("straight_forward", 0.025, 0.0),
    ("curve_left", 0.025, 0.15),
    ("curve_right", 0.025, -0.15),
    ("spin_left", 0.0, 0.25),
    ("spin_right", 0.0, -0.25),
    ("straight_reverse", -0.025, 0.0),
)
MIXED_PROFILE_CYCLE_S = 30.0
MIXED_PROFILE_PHASE_S = 5.0
MIXED_PROFILE_PAUSE_S = 0.25
RPM_SWEEP_RPM = (10.0, 25.0, 40.0, 60.0, 80.0, 100.0)
RPM_SWEEP_PHASE_S = 30.0
RPM_SWEEP_PAUSE_S = 0.5
WHEEL_CIRCUMFERENCE_M = 2.0 * math.pi * 0.034


def mixed_profile_command(elapsed_s):
    """Return (phase label, vx, wz); each phase ends with a brief zero pause."""
    cycle = int(elapsed_s // MIXED_PROFILE_CYCLE_S)
    cycle_offset = elapsed_s % MIXED_PROFILE_CYCLE_S
    phase_index = min(int(cycle_offset // MIXED_PROFILE_PHASE_S),
                       len(MIXED_PROFILE_PHASES) - 1)
    phase_name, linear_x, angular_z = MIXED_PROFILE_PHASES[phase_index]
    if cycle_offset % MIXED_PROFILE_PHASE_S >= MIXED_PROFILE_PHASE_S - MIXED_PROFILE_PAUSE_S:
        return f"{phase_name}_pause", 0.0, 0.0
    scale = (0.75, 0.9, 1.0)[cycle % 3]
    return phase_name, linear_x * scale, angular_z * scale


def rpm_sweep_command(elapsed_s):
    """Return phase and straight-line command for the fixed three-minute sweep."""
    bounded_elapsed = min(max(elapsed_s, 0.0), 6 * RPM_SWEEP_PHASE_S - 1e-9)
    phase_index = min(int(bounded_elapsed // RPM_SWEEP_PHASE_S), 5)
    target_rpm = RPM_SWEEP_RPM[phase_index]
    phase_elapsed = bounded_elapsed % RPM_SWEEP_PHASE_S
    if phase_elapsed >= RPM_SWEEP_PHASE_S - RPM_SWEEP_PAUSE_S:
        return f"rpm_{target_rpm:g}_pause", 0.0, 0.0, target_rpm
    linear_x = target_rpm * WHEEL_CIRCUMFERENCE_M / 60.0
    return f"rpm_{target_rpm:g}", linear_x, 0.0, target_rpm


def rpm_hold_command(target_rpm):
    """Convert a bounded wheel RPM request into a straight-line command."""
    linear_x = target_rpm * WHEEL_CIRCUMFERENCE_M / 60.0
    return f"rpm_{target_rpm:g}", linear_x, 0.0


class MotorFeedbackGuard:
    """Validate motor telemetry and fail on sustained commanded no-feedback."""

    RPM_LIMIT = 140.0
    TARGET_RPM_LIMIT = 130.0
    PWM_LIMIT = 255.0
    MIN_TARGET_RPM = 5.0
    ZERO_RPM_EPSILON = 0.1
    NO_FEEDBACK_TIMEOUT_S = 0.5
    TOLERATED_OUTLIERS = 3
    OUTLIER_WINDOW_S = 1.0

    def __init__(self):
        self.no_feedback_since = [None, None]
        self.outlier_times = deque()

    def observe(self, values, now):
        if len(values) != 7 or not all(math.isfinite(float(v)) for v in values):
            raise SafetyViolation("motor/state must contain seven finite values")

        state = [float(v) for v in values]
        if any(abs(v) > self.TARGET_RPM_LIMIT for v in state[:2]):
            raise SafetyViolation(f"motor target exceeds {self.TARGET_RPM_LIMIT:g} RPM: {state}")
        raw_outlier = any(abs(v) > self.RPM_LIMIT for v in state[2:4])
        while self.outlier_times and now - self.outlier_times[0] > self.OUTLIER_WINDOW_S:
            self.outlier_times.popleft()
        if raw_outlier:
            self.outlier_times.append(now)
            if len(self.outlier_times) > self.TOLERATED_OUTLIERS:
                raise SafetyViolation(
                    f"more than {self.TOLERATED_OUTLIERS} encoder samples exceed "
                    f"{self.RPM_LIMIT:g} RPM within {self.OUTLIER_WINDOW_S:g}s: {state}"
                )
        if any(abs(v) > self.PWM_LIMIT for v in state[4:6]):
            raise SafetyViolation(f"motor PWM exceeds {self.PWM_LIMIT:g}: {state}")
        if not 0.0 < state[6] <= 0.5:
            raise SafetyViolation(f"invalid motor control interval: {state[6]}")

        for wheel in range(2):
            target = abs(state[wheel])
            measured = abs(state[wheel + 2])
            if target >= self.MIN_TARGET_RPM and measured < self.ZERO_RPM_EPSILON:
                if self.no_feedback_since[wheel] is None:
                    self.no_feedback_since[wheel] = now
                elif now - self.no_feedback_since[wheel] >= self.NO_FEEDBACK_TIMEOUT_S:
                    name = "left" if wheel == 0 else "right"
                    elapsed = now - self.no_feedback_since[wheel]
                    raise SafetyViolation(
                        f"{name} wheel has no encoder response for {elapsed:.3f}s "
                        f"at target {target:.1f} RPM"
                    )
            else:
                self.no_feedback_since[wheel] = None
        return state


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("constant", "mixed", "rpm_sweep", "rpm_hold"), default="constant")
    parser.add_argument("--linear-x", type=float, default=0.025)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--wheel-rpm", type=float, default=130.0,
                        help="rpm_hold target; must be positive and no greater than 130")
    parser.add_argument("--duration", type=float, required=True,
                        help="constant/rpm_hold: 0.1-5 s; mixed: 30-300 s; rpm_sweep: exactly 180 s")
    args = parser.parse_args(argv)
    if args.profile == "constant":
        if not math.isfinite(args.linear_x) or abs(args.linear_x) > 0.05:
            parser.error("--linear-x must be finite and within ±0.05 m/s")
        if not math.isfinite(args.angular_z) or abs(args.angular_z) > 0.3:
            parser.error("--angular-z must be finite and within ±0.3 rad/s")
        if not math.isfinite(args.duration) or not 0.1 <= args.duration <= 5.0:
            parser.error("constant profile duration must be between 0.1 and 5 seconds")
        if abs(args.linear_x) < 1e-6 and abs(args.angular_z) < 1e-6:
            parser.error("a movement probe must request nonzero linear or angular velocity")
    elif args.profile == "mixed" and (
            not math.isfinite(args.duration) or not 30.0 <= args.duration <= 300.0):
        parser.error("mixed profile duration must be between 30 and 300 seconds")
    elif args.profile == "rpm_sweep" and args.duration != 180.0:
        parser.error("rpm_sweep duration must be exactly 180 seconds")
    elif args.profile == "rpm_hold":
        if not math.isfinite(args.wheel_rpm) or not 0.0 < args.wheel_rpm <= MotorFeedbackGuard.TARGET_RPM_LIMIT:
            parser.error("--wheel-rpm must be positive and no greater than 130 RPM")
        if not math.isfinite(args.duration) or not 0.1 <= args.duration <= 5.0:
            parser.error("rpm_hold duration must be between 0.1 and 5 seconds")
    return args


def main(argv=None):
    args = parse_args(argv)

    # Import ROS only for live execution; the safety logic above is host-testable.
    import rclpy
    from rcl_interfaces.msg import Log
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32MultiArray, Int64MultiArray, String

    reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
    run_id = str(uuid.uuid4())

    class Probe(Node):
        def __init__(self):
            super().__init__("picoros_guarded_validation_probe")
            self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", best_effort)
            self.last = {}
            self.fault = None
            self.own_stamps = set()
            self.motor_guard = MotorFeedbackGuard()
            self.motor_sample_count = 0
            self.create_subscription(Odometry, "/wheel/odom", self.on_odom, reliable)
            self.create_subscription(Imu, "/imu/data", self.on_imu, best_effort)
            self.create_subscription(Float32MultiArray, "/motor/state", self.on_motor, reliable)
            self.create_subscription(Int64MultiArray, "/hardware/time/reply", self.on_time, reliable)
            self.create_subscription(String, "/hardware/control_mode", self.on_mode, reliable)
            self.create_subscription(String, "/hardware/encoder/status",
                                     self.on_encoder_status, reliable)
            self.create_subscription(TwistStamped, "/cmd_vel", self.on_command_echo, best_effort)
            self.create_subscription(Log, "/rosout", self.on_rosout, reliable)

        def emit(self, event, **fields):
            print(json.dumps({"run_id": run_id, "event": event,
                              "wall_ns": time.time_ns(),
                              "mono": round(time.monotonic(), 6), **fields},
                             sort_keys=True), flush=True)

        def remember(self, name, message):
            self.last[name] = (time.monotonic(), message)

        def on_odom(self, message):
            self.remember("odom", message)
            self.emit("odom", stamp=stamp_ns(message),
                      linear_x=message.twist.twist.linear.x,
                      angular_z=message.twist.twist.angular.z)

        def on_imu(self, message):
            self.remember("imu", message)
            self.emit("imu", stamp=stamp_ns(message),
                      gyro_z=message.angular_velocity.z)

        def on_motor(self, message):
            self.remember("motor", message)
            self.motor_sample_count += 1
            try:
                values = self.motor_guard.observe(message.data, time.monotonic())
                self.emit("motor", data=values)
            except SafetyViolation as error:
                self.fault = str(error)
                self.emit("motor_fault", reason=self.fault, data=list(message.data))

        def on_time(self, message):
            self.remember("time", message)
            self.emit("time_reply", data=list(message.data))

        def on_mode(self, message):
            self.remember("mode", message)
            self.emit("mode", value=message.data)

        def on_encoder_status(self, message):
            changed = ("encoder_status" not in self.last or
                       self.last["encoder_status"][1].data != message.data)
            self.remember("encoder_status", message)
            if changed:
                self.emit("encoder_status", value=message.data)
            if message.data != "ready" and self.fault is None:
                self.fault = f"firmware encoder status is {message.data!r}"
                self.emit("encoder_status_fault", reason=self.fault)

        def on_command_echo(self, message):
            stamp = stamp_ns(message)
            if stamp not in self.own_stamps:
                self.fault = "foreign /cmd_vel publisher detected"
                self.emit("foreign_command", stamp=stamp,
                          linear_x=message.twist.linear.x,
                          angular_z=message.twist.angular.z)
            else:
                self.emit("cmd_echo", stamp=stamp,
                          linear_x=message.twist.linear.x,
                          angular_z=message.twist.angular.z)

        def on_rosout(self, message):
            self.emit("firmware_rosout", level=message.level, message=message.msg)

        def send_twist(self, linear_x, angular_z, profile_phase=None):
            message = TwistStamped()
            stamp = self.get_clock().now().nanoseconds
            if stamp <= 0:
                raise SafetyViolation("ROS clock is not providing a positive epoch timestamp")
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            message.twist.linear.x = float(linear_x)
            message.twist.angular.z = float(angular_z)
            self.own_stamps.add(stamp)
            self.publisher.publish(message)
            self.emit("command", stamp=stamp, linear_x=linear_x,
                      angular_z=angular_z, profile_phase=profile_phase)

        def fresh(self, key, max_age):
            return key in self.last and time.monotonic() - self.last[key][0] <= max_age

        def validate(self):
            if self.fault:
                raise SafetyViolation(self.fault)
            for key, limit in (("odom", 0.30), ("imu", 0.30),
                               ("motor", 0.25), ("mode", 0.25),
                               ("encoder_status", 0.5), ("time", 2.2)):
                if not self.fresh(key, limit):
                    raise SafetyViolation(f"{key} feedback stale/missing >{limit * 1000:.0f} ms")
            if self.last["encoder_status"][1].data != "ready":
                raise SafetyViolation(
                    f"firmware encoder status is {self.last['encoder_status'][1].data!r}")
            if self.last["mode"][1].data != "idle":
                raise SafetyViolation("control mode changed from idle")
            ros_now = self.get_clock().now().nanoseconds
            for key in ("odom", "imu"):
                age_ms = (ros_now - stamp_ns(self.last[key][1])) / 1e6
                if age_ms < -100 or age_ms > 300:
                    raise SafetyViolation(f"{key} stamp age {age_ms:.1f} ms outside [-100, 300]")
            reply = list(self.last["time"][1].data)
            reply_age_ns = ros_now - int(reply[3]) if len(reply) == 4 else None
            if reply_age_ns is None or not -100_000_000 <= reply_age_ns <= 2_200_000_000:
                raise SafetyViolation("clock reply invalid or older than 2.2 s")
            if self.publisher.get_subscription_count() < 1:
                raise SafetyViolation("no /cmd_vel subscriber")

        def spin(self, duration):
            rclpy.spin_once(self, timeout_sec=duration)

        def stop_for_one_second(self):
            until = time.monotonic() + 1.0
            next_send = 0.0
            while time.monotonic() < until:
                self.spin(0.02)
                if time.monotonic() >= next_send:
                    self.send_twist(0.0, 0.0)
                    next_send = time.monotonic() + 0.1

        def observe_stop_for_ten_seconds(self):
            until = time.monotonic() + 10.0
            consecutive_zero = 0
            last_counted_sample = self.motor_sample_count
            while time.monotonic() < until:
                self.spin(0.05)
                if self.motor_sample_count != last_counted_sample:
                    last_counted_sample = self.motor_sample_count
                    values = list(self.last["motor"][1].data)
                    if (self.fresh("motor", 0.25) and len(values) == 7 and
                            all(abs(float(v)) <= 0.01 for v in values[:6])):
                        consecutive_zero += 1
                    else:
                        consecutive_zero = 0
                elif not self.fresh("motor", 0.25):
                    consecutive_zero = 0
            confirmed = consecutive_zero >= 3
            self.emit("post_stop", stillstand_confirmed=confirmed,
                      consecutive_zero_motor_samples=consecutive_zero,
                      last_motor=list(self.last["motor"][1].data)
                      if "motor" in self.last else None)
            return confirmed

    def stamp_ns(message):
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    rclpy.init()
    node = Probe()
    passed = False
    run_started = None
    try:
        node.emit("preflight_start", seconds=10, profile=args.profile,
                  duration_s=args.duration)
        preflight_end = time.monotonic() + 10.0
        while time.monotonic() < preflight_end:
            node.spin(0.02)
            if node.fault:
                raise SafetyViolation(node.fault)
        node.validate()
        motor = [float(v) for v in node.last["motor"][1].data]
        if any(abs(value) > 0.01 for value in motor[:6]):
            raise SafetyViolation(f"preflight motors are not zero: {motor}")
        node.emit("preflight_ok", motor=motor,
                  cmd_vel_subscribers=node.publisher.get_subscription_count())

        run_started = time.monotonic()
        next_send = 0.0
        last_phase = None
        while time.monotonic() - run_started < args.duration:
            node.spin(0.02)
            node.validate()
            elapsed = time.monotonic() - run_started
            if args.profile == "mixed":
                phase, linear_x, angular_z = mixed_profile_command(elapsed)
            elif args.profile == "rpm_sweep":
                phase, linear_x, angular_z, requested_rpm = rpm_sweep_command(elapsed)
            elif args.profile == "rpm_hold":
                phase, linear_x, angular_z = rpm_hold_command(args.wheel_rpm)
                requested_rpm = args.wheel_rpm
            else:
                phase, linear_x, angular_z = "constant", args.linear_x, args.angular_z
            if phase != last_phase:
                node.emit("profile_phase", phase=phase, elapsed_s=round(elapsed, 3),
                          linear_x=linear_x, angular_z=angular_z,
                          requested_wheel_rpm=(requested_rpm
                                               if args.profile in ("rpm_sweep", "rpm_hold") else None))
                last_phase = phase
            if time.monotonic() >= next_send:
                node.send_twist(linear_x, angular_z, phase)
                next_send = time.monotonic() + 0.1
        passed = True
    except (SafetyViolation, KeyboardInterrupt) as error:
        reason = str(error) if not isinstance(error, KeyboardInterrupt) else "interrupted"
        node.emit("abort", reason=reason,
                  elapsed_s=round(time.monotonic() - run_started, 3)
                  if run_started is not None else None)
    finally:
        try:
            node.stop_for_one_second()
            stillstand_confirmed = node.observe_stop_for_ten_seconds()
        except Exception as error:  # Keep cleanup best-effort and visible.
            node.emit("stop_issue", reason=str(error))
            stillstand_confirmed = False
        node.emit("final", completed=passed,
                  stillstand_confirmed=stillstand_confirmed,
                  last_motor=list(node.last["motor"][1].data)
                  if "motor" in node.last else None)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed and stillstand_confirmed else 2


if __name__ == "__main__":
    raise SystemExit(main())
