#!/usr/bin/env python3
"""Read-only Pico-ROS acceptance capture; never publishes commands or heartbeats.

Run on the Pi in the backend ROS environment. Emits JSONL once per second and
fails on missing/stale/nonmonotonic telemetry or hardware.ready != true. Can
also accompany a separately authorized, operator-controlled joystick run.
"""
import argparse
import json
import math
import threading
import time
from urllib.request import Request, urlopen


class Samples:
    def __init__(self):
        self.count = 0
        self.last = None
        self.stamp = None
        self.max_gap = 0.0
        self.min_age = None
        self.max_age = None
        self.backwards = 0
        self.bad = 0

    def record(self, now, stamp=None, wall=None):
        if self.last is not None:
            self.max_gap = max(self.max_gap, now - self.last)
        self.last = now
        self.count += 1
        if stamp is not None:
            if self.stamp is not None and stamp <= self.stamp:
                self.backwards += 1
            self.stamp = stamp
            age = (wall - stamp) / 1e9
            self.min_age = age if self.min_age is None else min(self.min_age, age)
            self.max_age = age if self.max_age is None else max(self.max_age, age)

    def summary(self, now, elapsed):
        return dict(count=self.count, rate=self.count / elapsed,
                    age=None if self.last is None else now - self.last,
                    max_gap=self.max_gap, min_stamp_age=self.min_age,
                    max_stamp_age=self.max_age, backwards=self.backwards, bad=self.bad)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=600)
    parser.add_argument("--status-url", default="http://127.0.0.1:8767/status")
    parser.add_argument("--origin", default="http://localhost:5173")
    parser.add_argument("--require-stopped", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 10:
        parser.error("duration must be finite and at least 10 seconds")

    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32MultiArray, String, Int64MultiArray
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("picoros_communication_check")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    samples = {name: Samples() for name in ("odom", "imu", "motor", "encoder")}
    encoder = None
    motor = None
    boot_ids = set()
    failures = set()
    status = {"ready": False, "checked": 0.0}
    status_lock = threading.Lock()
    finished = threading.Event()

    def read_status():
        while not finished.is_set():
            try:
                with urlopen(Request(args.status_url, headers={"Origin": args.origin}), timeout=1) as response:
                    hardware = json.load(response)["hardware"]
                result = dict(ready=hardware.get("ready") is True, hardware=hardware)
            except Exception as error:
                result = dict(ready=False, error=str(error))
            with status_lock:
                status.update(result, checked=time.monotonic())
            finished.wait(1)

    def receive(name, message):
        nonlocal encoder, motor
        item = samples[name]
        stamp = None
        if name in ("odom", "imu"):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            values = ([message.twist.twist.linear.x, message.twist.twist.angular.z,
                       message.pose.pose.position.x, message.pose.pose.position.y]
                      if name == "odom" else
                      [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                       message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z])
            if not all(math.isfinite(value) for value in values):
                item.bad += 1
        elif name == "encoder":
            encoder = message.data
            if encoder != "ready":
                item.bad += 1
        else:
            motor = list(message.data)
            if len(motor) != 7 or not all(math.isfinite(value) for value in motor):
                item.bad += 1
            elif args.require_stopped and any(abs(value) > 0.1 for value in motor[:6]):
                item.bad += 1
        item.record(time.monotonic(), stamp, time.time_ns())

    for name, kind, topic in (("odom", Odometry, "/wheel/odom"), ("imu", Imu, "/imu/data"),
                              ("motor", Float32MultiArray, "/motor/state"),
                              ("encoder", String, "/hardware/encoder/status")):
        node.create_subscription(kind, topic, lambda message, name=name: receive(name, message), qos)
    node.create_subscription(Int64MultiArray, "/hardware/time/request",
                             lambda msg: boot_ids.add(msg.data[0]) if len(msg.data) == 3 else None, qos)
    worker = threading.Thread(target=read_status, daemon=True)
    worker.start()

    def emit(event, **fields):
        print(json.dumps(dict(event=event, wall_ns=time.time_ns(), **fields), sort_keys=True), flush=True)

    try:
        # Discovery/calibration/reconnect belongs to preflight, not the soak.
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            with status_lock:
                ready = status["ready"]
            if ready and all(s.last and time.monotonic() - s.last < 0.3 for s in samples.values()):
                break
        else:
            emit("failed", reason="preflight: telemetry or hardware readiness unavailable", status=status)
            return 1
        for name in samples:
            samples[name] = Samples()
        boot_ids.clear()
        start = report = time.monotonic()
        emit("start", duration=args.duration, require_stopped=args.require_stopped)
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            elapsed = now - start
            current = {name: item.summary(now, elapsed) for name, item in samples.items()}
            for name, item in current.items():
                if elapsed > 0.3 and (item["age"] is None or item["age"] > 0.3):
                    failures.add(name + ": stale")
                if item["max_gap"] > 0.3:
                    failures.add(name + ": gap >300ms")
                if item["backwards"] or item["bad"]:
                    failures.add(name + ": invalid/nonmonotonic")
                if item["min_stamp_age"] is not None and not -0.2 <= item["min_stamp_age"]:
                    failures.add(name + ": future timestamp")
                if item["max_stamp_age"] is not None and item["max_stamp_age"] > 0.3:
                    failures.add(name + ": old timestamp")
            with status_lock:
                hardware = dict(status)
            if not hardware["ready"] or now - hardware["checked"] > 3:
                failures.add("hardware readiness lost")
            if len(boot_ids) > 1:
                failures.add("MCU restarted")
            if now >= report:
                emit("sample", elapsed=elapsed, topics=current, encoder=encoder,
                     motor=motor, status=hardware, failures=sorted(failures))
                report = now + 1
        for name, minimum in (("odom", 9), ("imu", 18), ("motor", 9), ("encoder", 9)):
            if current[name]["rate"] < minimum:
                failures.add(name + ": rate too low")
        emit("summary", elapsed=elapsed, topics=current, failures=sorted(failures), passed=not failures,
             boot_ids=sorted(boot_ids))
        return int(bool(failures))
    finally:
        finished.set()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
