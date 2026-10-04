#!/usr/bin/env python3
"""Capture one overlay from each node and measure BEST_EFFORT topic rates."""

import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class OutputCapture(Node):
    def __init__(self):
        super().__init__("perception_output_capture")
        self.bridge = CvBridge()
        self.started = time.monotonic()
        self.counts = {"road": 0, "signs": 0}
        self.saved = set()
        self.output_dir = os.path.join(os.getcwd(), "logs", "jetson_perception")
        os.makedirs(self.output_dir, exist_ok=True)
        self.create_subscription(
            Image, "/perception/road/overlay", self._road, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/perception/signs/overlay", self._signs, qos_profile_sensor_data
        )

    def _road(self, msg):
        self._receive("road", msg)

    def _signs(self, msg):
        self._receive("signs", msg)

    def _receive(self, name, msg):
        self.counts[name] += 1
        if name not in self.saved:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            path = os.path.join(self.output_dir, f"{name}_overlay_sample.jpg")
            cv2.imwrite(path, image)
            self.get_logger().info(
                f"saved {path}: {image.shape[1]}x{image.shape[0]}, frame={msg.header.frame_id}"
            )
            self.saved.add(name)


def main():
    rclpy.init()
    node = OutputCapture()
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    elapsed = time.monotonic() - node.started
    for name, count in node.counts.items():
        print(f"{name}_overlay: {count / elapsed:.2f} Hz ({count} frames/{elapsed:.2f}s)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
