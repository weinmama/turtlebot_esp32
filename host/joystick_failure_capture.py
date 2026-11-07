#!/usr/bin/env python3
"""Read-only capture of joystick, motor, sensor, clock and firmware diagnostics.

Run inside the Pi backend ROS environment while an operator manually uses the
joystick. This node publishes nothing: it never sends commands or heartbeats.
It writes timestamped JSONL events and a summary for correlating a stop with
command delivery, motor feedback, sensor freshness and firmware rosout.
"""
import argparse
import json
import math
import time


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=180.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 10 <= args.duration <= 600:
        parser.error("duration must be between 10 and 600 seconds")

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Log
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32MultiArray, Int64MultiArray, String

    rclpy.init()
    node = Node("picoros_joystick_failure_capture")
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    data = {name: {"count": 0, "last_mono": None, "max_gap_s": 0.0,
                   "last_stamp": None, "nonmonotonic": 0, "max_age_s": None}
            for name in ("cmd_vel", "motor", "odom", "imu", "mode", "encoder", "clock_request", "clock_reply", "reset_diagnostic", "clock_diagnostic", "rosout")}
    started = time.monotonic()

    def emit(topic, **fields):
        now_mono = time.monotonic()
        now_wall = time.time_ns()
        stats = data[topic]
        if stats["last_mono"] is not None:
            stats["max_gap_s"] = max(stats["max_gap_s"], now_mono - stats["last_mono"])
        stats["last_mono"] = now_mono
        stats["count"] += 1
        stamp = fields.get("stamp_ns")
        if stamp is not None:
            if stats["last_stamp"] is not None and stamp <= stats["last_stamp"]:
                stats["nonmonotonic"] += 1
            stats["last_stamp"] = stamp
            age = (now_wall - stamp) / 1e9
            stats["max_age_s"] = age if stats["max_age_s"] is None else max(stats["max_age_s"], age)
            fields["stamp_age_ms"] = round(age * 1000, 2)
        print(json.dumps({"event": topic, "wall_ns": now_wall,
                          "mono": round(now_mono, 6), **fields}, sort_keys=True), flush=True)

    def on_cmd(message):
        emit("cmd_vel", stamp_ns=stamp_ns(message), linear_x=message.twist.linear.x,
             angular_z=message.twist.angular.z)

    def on_motor(message):
        values = [float(value) for value in message.data]
        emit("motor", values=values, valid=len(values) == 7 and all(map(math.isfinite, values)),
             targets_rpm=values[:2], measured_rpm=values[2:4], pwm=values[4:6],
             control_dt_s=values[6] if len(values) == 7 else None)

    def on_odom(message):
        emit("odom", stamp_ns=stamp_ns(message), linear_x=message.twist.twist.linear.x,
             angular_z=message.twist.twist.angular.z, x=message.pose.pose.position.x,
             y=message.pose.pose.position.y)

    def on_imu(message):
        emit("imu", stamp_ns=stamp_ns(message), gyro_z=message.angular_velocity.z,
             accel_x=message.linear_acceleration.x)

    def on_mode(message):
        emit("mode", value=message.data)

    def on_encoder(message):
        emit("encoder", value=message.data)

    def on_clock(name):
        return lambda message: emit(name, values=list(message.data))

    def on_log(message):
        emit("rosout", level=message.level, name=message.name, message=message.msg)

    node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, best_effort)
    node.create_subscription(Float32MultiArray, "/motor/state", on_motor, reliable)
    node.create_subscription(Odometry, "/wheel/odom", on_odom, reliable)
    node.create_subscription(Imu, "/imu/data", on_imu, best_effort)
    node.create_subscription(String, "/hardware/control_mode", on_mode, reliable)
    node.create_subscription(String, "/hardware/encoder/status", on_encoder, reliable)
    node.create_subscription(Int64MultiArray, "/hardware/time/request", on_clock("clock_request"), reliable)
    node.create_subscription(Int64MultiArray, "/hardware/time/reply", on_clock("clock_reply"), reliable)
    node.create_subscription(Int64MultiArray, "/hardware/reset_diagnostic", on_clock("reset_diagnostic"), reliable)
    node.create_subscription(Int64MultiArray, "/hardware/clock_diagnostic", on_clock("clock_diagnostic"), reliable)
    node.create_subscription(Log, "/rosout", on_log, reliable)
    print(json.dumps({"event": "capture_start", "wall_ns": time.time_ns(),
                      "duration_s": args.duration, "read_only": True}), flush=True)
    try:
        end = started + args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(0.05, end - time.monotonic()))
    except KeyboardInterrupt:
        print(json.dumps({"event": "capture_interrupted", "wall_ns": time.time_ns()}), flush=True)
    finally:
        elapsed = time.monotonic() - started
        for stats in data.values():
            if stats["last_mono"] is not None:
                stats["last_age_s"] = time.monotonic() - stats["last_mono"]
            stats["rate_hz"] = stats["count"] / elapsed
        print(json.dumps({"event": "capture_summary", "wall_ns": time.time_ns(),
                          "elapsed_s": elapsed, "topics": data}, sort_keys=True), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
