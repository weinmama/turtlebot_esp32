#!/usr/bin/env python3
"""Isolated-test responder in a separate RMW session, like the real Pi."""
import os
import signal
import sys
from pathlib import Path

if os.environ.get("PICOROS_TEST_ISOLATED") != "1":
    raise RuntimeError("Only run in the network-isolated test container")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "host"))
import rclpy
from time_sync import ClockResponder


class Responder(ClockResponder):
    paused = False

    def respond(self, message):
        if not self.paused:
            super().respond(message)


rclpy.init()
node = Responder()
signal.signal(signal.SIGUSR1, lambda *_: setattr(node, "paused", True))
signal.signal(signal.SIGUSR2, lambda *_: setattr(node, "paused", False))
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
