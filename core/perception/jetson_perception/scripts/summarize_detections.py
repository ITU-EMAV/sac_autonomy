#!/usr/bin/env python3
"""Summarize live JSON detections for confidence/false-positive tuning."""

from collections import Counter
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Summary(Node):
    def __init__(self):
        super().__init__("perception_detection_summary")
        self.messages = 0
        self.counts = Counter()
        self.confidences = {}
        self.create_subscription(
            String, "/perception/signs/detections_json", self.callback, 10
        )

    def callback(self, msg):
        self.messages += 1
        for detection in json.loads(msg.data).get("detections", []):
            key = (detection.get("source", "unknown"), detection["class_name"])
            self.counts[key] += 1
            self.confidences.setdefault(key, []).append(detection["confidence"])


def main():
    rclpy.init()
    node = Summary()
    deadline = time.monotonic() + 15.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    print(f"messages={node.messages}")
    for key, count in node.counts.most_common():
        values = node.confidences[key]
        print(
            f"{key[0]}:{key[1]} count={count} "
            f"confidence={min(values):.3f}-{max(values):.3f} avg={sum(values)/len(values):.3f}"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
