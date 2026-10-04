#!/usr/bin/env python3
"""Convert legacy unstamped GlobalPlanner Float64 speed (m/s) to a live constraint."""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sac_interfaces.msg import SpeedConstraint
from std_msgs.msg import Float64


def valid_map_limit(value, maximum):
    return math.isfinite(value) and 0 < value <= maximum


class MapSpeedConstraint(Node):
    def __init__(self):
        super().__init__('map_speed_constraint')
        self.declare_parameter('source_timeout', 0.5)
        self.declare_parameter('enforce_freshness', True)
        self.declare_parameter('max_accepted_mps', 20.0)
        self.declare_parameter('publish_period', 0.05)
        self.limit = None
        self.receipt = None
        self.create_subscription(Float64, '/speed_limit', self.on_limit, 1)
        self.pub = self.create_publisher(SpeedConstraint,
                                         '/planning/map/speed_constraint', 1)
        self.create_timer(self.get_parameter('publish_period').value, self.tick)

    def on_limit(self, msg):
        # GlobalPlanner currently emits 2.0 or 4.0 m/s. Zero is invalid here,
        # since this legacy topic has no explicit STOP/unset semantics.
        maximum = self.get_parameter('max_accepted_mps').value
        if not valid_map_limit(msg.data, maximum):
            self.limit = None
            self.receipt = None
            return
        self.limit = msg.data
        self.receipt = time.monotonic()

    def tick(self):
        output = SpeedConstraint()
        output.header.stamp = self.get_clock().now().to_msg()
        fresh = (self.receipt is not None and
                 (not self.get_parameter('enforce_freshness').value or
                  time.monotonic()-self.receipt <=
                  self.get_parameter('source_timeout').value))
        output.max_speed_mps = self.limit if fresh else 0.0
        output.reason = 'map speed limit' if fresh else 'missing/stale/invalid map speed limit'
        self.pub.publish(output)


def main():
    rclpy.init()
    node = MapSpeedConstraint()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
