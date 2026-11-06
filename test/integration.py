#!/usr/bin/env python3
"""Real rmw_zenoh/Pico-ROS interoperability, optionally through a serial PTY pair.

Run in a --network none container, with PICOROS_TEST_ISOLATED=1. The native
fixture contains no hardware drivers; nonzero test commands cannot reach motors.
"""
import argparse
import json
import os
from pathlib import Path
import pty
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tty

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", action="store_true")
    args = parser.parse_args()
    if os.environ.get("PICOROS_TEST_ISOLATED") != "1":
        raise RuntimeError("Run in a --network none container with PICOROS_TEST_ISOLATED=1")
    os.environ.update(ROS_DOMAIN_ID="1", RMW_IMPLEMENTATION="rmw_zenoh_cpp",
                      ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:17447"];scouting/multicast/enabled=false')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String, Float32MultiArray, Int64MultiArray
    from time_sync import ClockResponder

    processes, descriptors, serial_trace = [], [], []
    exit_relay = threading.Event()
    with tempfile.TemporaryDirectory(prefix="picoros-test-") as directory:
        directory = Path(directory)
        config = {"mode": "router", "listen": {"endpoints": ["tcp/127.0.0.1:17447"]},
                  "scouting": {"multicast": {"enabled": False}}}
        locator = "tcp/127.0.0.1:17447"
        if args.serial:
            a, sa = pty.openpty()
            b, sb = pty.openpty()
            descriptors.extend([a, sa, b, sb])
            tty.setraw(sa); tty.setraw(sb)
            # Upstream POSIX Pico driver caps its baud enum at 230400. PTYs do
            # not model baud timing; firmware UART0 and host/router.json5 use
            # 460800. This checks real serial framing, handshake and fragments.
            config["listen"]["endpoints"].append(f"serial/{os.ttyname(sa)}#baudrate=230400")
            locator = f"serial/{os.ttyname(sb)}#baudrate=230400"

            def relay():
                while not exit_relay.is_set():
                    ready, _, _ = select.select([a, b], [], [], 0.1)
                    for source in ready:
                        data = os.read(source, 4096)
                        if len(serial_trace) < 12:
                            serial_trace.append(("host" if source == a else "mcu", data.hex()))
                        target = b if source == a else a
                        while data:
                            data = data[os.write(target, data):]
            relay_thread = threading.Thread(target=relay, daemon=True)
            relay_thread.start()
        config_path = directory / "router.json"
        config_path.write_text(json.dumps(config))
        env = dict(os.environ, ZENOH_ROUTER_CONFIG_URI=str(config_path))
        env.pop("ZENOH_CONFIG_OVERRIDE", None)
        router_log = (directory / "router.log").open("w+")
        router = subprocess.Popen(["/opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd"],
                                  env=env, stdout=router_log, stderr=subprocess.STDOUT)
        processes.append(router)
        # Wait for the listener rather than racing a client with reconnect off.
        deadline = time.monotonic() + 5
        while True:
            try:
                with socket.create_connection(("127.0.0.1", 17447), timeout=0.1):
                    break
            except OSError:
                if router.poll() is not None or time.monotonic() > deadline:
                    router_log.seek(0)
                    raise RuntimeError(router_log.read())
                time.sleep(0.05)
        lines = []
        if args.serial:
            # The router clears serial RX after its accept-loop throttle;
            # TCP listening alone does not mean the serial listener is ready.
            time.sleep(1.1)
        fixture = subprocess.Popen([str(ROOT / ".pio/native/transport_fixture"), locator],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        processes.append(fixture)
        def read_fixture():
            for line in fixture.stdout:
                lines.append(line.strip())
        threading.Thread(target=read_fixture, daemon=True).start()

        rclpy.init()
        clock = ClockResponder()
        node = Node("picoros_integration_test")
        executor = SingleThreadedExecutor()
        executor.add_node(clock); executor.add_node(node)
        odometry, imu_samples, states = [], [], []
        node.create_subscription(Odometry, "/wheel/odom", odometry.append, 10)
        node.create_subscription(Imu, "/imu/data", imu_samples.append, 10)
        node.create_subscription(Float32MultiArray, "/motor/state", states.append, 10)
        manual = node.create_publisher(TwistStamped, "/cmd_vel",
                                       QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        navigation = node.create_publisher(TwistStamped, "/hardware/nav_cmd_vel", 1)
        modes = node.create_publisher(String, "/hardware/control_mode", 1)
        tuning = node.create_publisher(Float32MultiArray, "/pid_tune", 1)
        fake_clock = node.create_publisher(Int64MultiArray, "/hardware/time/reply", 1)
        mode = None
        timer = node.create_timer(0.1, lambda: modes.publish(String(data=mode)) if mode else None)

        def spin(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)

        def wait(predicate, description, seconds=8):
            deadline = time.monotonic() + seconds
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            assert predicate(), description

        def command(publisher=manual, v=0.12, age=0, stamp=None):
            message = TwistStamped()
            epoch = time.time_ns() - int(age * 1e9) if stamp is None else stamp
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(epoch, 10**9)
            message.twist.linear.x = v
            publisher.publish(message)
            return epoch

        def count(prefix="COMMAND "):
            return sum(line.startswith(prefix) for line in lines)

        def stopped():
            return next((line == "STOP 1" for line in reversed(lines) if line.startswith("STOP ")), True)

        try:
            wait(lambda: len(odometry) >= 3 and len(imu_samples) >= 3, "ROS did not receive synchronized telemetry", 20)
            wait(lambda: node.count_subscribers("/cmd_vel") >= 1 and
                 node.count_publishers("/wheel/odom") == 1 and node.count_publishers("/imu/data") == 1,
                 "Pico-ROS endpoints are missing from the ROS graph")
            assert odometry[-1].header.frame_id == "odom" and odometry[-1].child_frame_id == "base_link"
            assert odometry[-1].pose.pose.position.x == 1.25
            assert abs(odometry[-1].twist.covariance[0] - 0.0025) < 1e-9
            assert imu_samples[-1].header.frame_id == "imu_link"
            assert imu_samples[-1].orientation_covariance[0] == -1
            assert abs(imu_samples[-1].linear_acceleration.z - 9.81) < 1e-9
            for message in (odometry[-1], imu_samples[-1]):
                epoch = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                assert -0.1 < (time.time_ns() - epoch) / 1e9 < 0.3
            command(); spin(0.2)
            assert count() == 0, "command accepted without mode heartbeat"
            mode = "mapping"; spin(0.25)
            command(); wait(lambda: count() == 1, "manual command was not delivered")
            wait(lambda: "STOP 0" in lines, "manual command not enabled")
            wait(lambda: "STOP 1" in lines, "command timeout did not stop", 2)

            before = count()
            command(age=2); spin(0.15)
            command(age=-2); spin(0.15)
            command(v=float("nan")); spin(0.15)
            assert count() == before, "invalid/stale/future command accepted"
            stamp = command(); wait(lambda: count() == before + 1, "fresh command rejected")
            for _ in range(7):
                command(stamp=stamp); spin(0.1)
            assert count() == before + 1 and stopped(), "replay extended command lifetime"

            mode = "navigation"; spin(0.25)
            before = count()
            command(); spin(0.15)
            assert count() == before, "manual command admitted in navigation mode"
            command(navigation, v=0.23)
            wait(lambda: count() == before + 1, "navigation command was not delivered")
            mode = "blocked"; spin(0.25)
            assert stopped(), "blocked mode did not stop"
            before = count(); command(navigation); command(); spin(0.2)
            assert count() == before, "blocked mode accepted commands"

            mode = "mapping"; spin(0.25)
            command(); spin(0.15)
            mode = None
            for _ in range(10):
                command(); spin(0.1)
            assert stopped(), "missing heartbeat did not stop"
            before = count(); command(); spin(0.15)
            assert count() == before

            tuning.publish(Float32MultiArray(data=[1.0, 2.0, 3.0])); spin(0.2)
            assert "TUNING 3 1.000" in lines
            tuning.publish(Float32MultiArray(data=[4.0, 2.0, 3.0, 1.0, 2.0, 3.0])); spin(0.2)
            assert "TUNING 6 4.000" in lines
            before = count("TUNING ")
            tuning.publish(Float32MultiArray(data=[float("nan"), 2.0, 3.0])); spin(0.2)
            tuning.publish(Float32MultiArray(data=[1.0] * 7)); spin(0.2)
            assert count("TUNING ") == before

            mode = "mapping"; spin(0.25)
            # Spoofed/unmatched clock replies cannot refresh liveness.
            executor.remove_node(clock)
            for _ in range(60):
                fake_clock.publish(Int64MultiArray(data=[0, 1, 0, time.time_ns()]))
                command(); spin(0.1)
            assert stopped(), "clock responder failure did not stop"
            before = count(); command(); spin(0.2)
            assert count() == before
            executor.add_node(clock)
            spin(1.5)
            command(); wait(lambda: count() == before + 1, "clock recovery failed")
            assert router.poll() is None and fixture.poll() is None
            print(f"PASS: {'serial PTY' if args.serial else 'TCP'} ROS discovery, telemetry, command routing, "
                  "replay/age/NaN rejection, timeouts, PID and clock recovery")
        except Exception:
            router_log.flush(); router_log.seek(0)
            print("Router:", router_log.read(), file=sys.stderr)
            print("Fixture:", fixture.poll(), "\n".join(lines), file=sys.stderr)
            print("Serial:", serial_trace, file=sys.stderr)
            raise
        finally:
            node.destroy_timer(timer)
            executor.shutdown()
            clock.destroy_node(); node.destroy_node(); rclpy.shutdown()
            for process in reversed(processes):
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            exit_relay.set()
            if args.serial:
                relay_thread.join(timeout=1)
            for descriptor in descriptors:
                os.close(descriptor)
            router_log.close()


if __name__ == "__main__":
    main()
