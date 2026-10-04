#!/usr/bin/env python3
"""Bounded, non-actuating replay metrics collector for Phase 3 A/B tests."""
import argparse
import json
import math
import statistics
import time

import rclpy
from cluster_msgs.msg import ClusterPointsArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sac_interfaces.msg import PipelineTiming


def summary(values):
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return {"n": 0}
    def percentile(p):
        return finite[min(len(finite) - 1, int((len(finite) - 1) * p))]
    return {
        "n": len(finite), "median": statistics.median(finite),
        "p99": percentile(0.99), "max": finite[-1],
    }


class Collector(Node):
    def __init__(self):
        super().__init__(
            "phase3_metrics_collector",
            parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.values = {}
        self.last_lidar_rx = None
        timing_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PipelineTiming, "/control/timing", self.control, timing_qos)
        self.create_subscription(PipelineTiming, "/perception/road/timing", self.road, timing_qos)
        self.create_subscription(PipelineTiming, "/perception/signs/timing", self.signs, timing_qos)
        self.create_subscription(
            ClusterPointsArray, "/all_bottom_clusters", self.lidar,
            qos_profile_sensor_data)

    def add(self, key, value):
        self.values.setdefault(key, []).append(float(value))

    def control(self, msg):
        for key in ("release_jitter_ms", "execution_time_ms", "snapshot_lock_ms",
                    "path_projection_ms", "control_compute_ms", "publish_ms"):
            self.add("control_" + key, getattr(msg, key))

    def camera(self, prefix, msg):
        self.add(prefix + "_execution_ms", msg.execution_time_ms)
        self.add(prefix + "_source_age_ms", msg.source_age_ms)
        self.add(prefix + "_queue_delay_ms", msg.queue_delay_ms)
        self.values[prefix + "_received"] = [msg.received_count]
        self.values[prefix + "_processed"] = [msg.processed_count]
        self.values[prefix + "_dropped"] = [msg.dropped_count]

    def road(self, msg):
        self.camera("road", msg)

    def signs(self, msg):
        self.camera("signs", msg)

    def lidar(self, msg):
        now_steady = time.monotonic()
        if self.last_lidar_rx is not None:
            self.add("lidar_interval_ms", (now_steady - self.last_lidar_rx) * 1000.0)
        self.last_lidar_rx = now_steady
        source_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.add("lidar_source_age_ms", (self.get_clock().now().nanoseconds - source_ns) / 1e6)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=500.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Collector()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        result = {key: summary(value) for key, value in node.values.items()}
        lidar_n = result.get("lidar_interval_ms", {}).get("n", 0)
        if lidar_n:
            elapsed_ms = sum(node.values["lidar_interval_ms"])
            result["lidar_effective_hz"] = 1000.0 * lidar_n / elapsed_ms
        with open(args.output, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
