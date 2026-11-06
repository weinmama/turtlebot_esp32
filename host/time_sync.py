#!/usr/bin/env python3
"""Pi clock responder for the serial Pico-ROS firmware (ROS domain 1).

Uses the host ROS clock; run without use_sim_time on hardware. Does not relay
sensor data or commands. The backend remains the control-mode authority.
"""
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, String


class ClockResponder(Node):
    def __init__(self, standalone_manual=False):
        super().__init__("turtlebot_picoros_clock")
        self.reply = self.create_publisher(Int64MultiArray, "/hardware/time/reply", 1)
        self.request = self.create_subscription(Int64MultiArray, "/hardware/time/request", self.respond, 1)
        if standalone_manual:
            self.mode = self.create_publisher(String, "/hardware/control_mode", 1)
            self.timer = self.create_timer(0.1, lambda: self.mode.publish(String(data="idle")))

    def respond(self, message):
        epoch = self.get_clock().now().nanoseconds
        if message.layout.dim or message.layout.data_offset or len(message.data) != 3:
            return
        boot_id, request_id, sent_ns = message.data
        if boot_id < 0 or request_id <= 0 or sent_ns < 0 or epoch <= 0:
            return
        self.reply.publish(Int64MultiArray(data=[boot_id, request_id, sent_ns, epoch]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--standalone-manual", action="store_true",
                        help="Publish idle heartbeats for bench use WITHOUT the backend/mode authority")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ClockResponder(args.standalone_manual)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
