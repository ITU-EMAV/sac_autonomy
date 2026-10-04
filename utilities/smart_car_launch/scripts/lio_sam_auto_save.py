#!/usr/bin/env python3

import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from lio_sam.srv import SaveMap


class LioSamAutoSave(Node):
    def __init__(self):
        super().__init__("lio_sam_auto_save")
        self.declare_parameter("idle_seconds", 5.0)
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter(
            "destination",
            str(Path.home() / ".local/share/sac_autonomy/maps/lio_sam"),
        )
        self.idle_seconds = float(self.get_parameter("idle_seconds").value)
        self.last_cloud_monotonic = None
        self.save_requested = False
        self.client = self.create_client(SaveMap, "/lio_sam/save_map")
        self.create_subscription(
            PointCloud2,
            "/velodyne_points",
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        # This node intentionally uses wall time, so it still detects inactivity
        # after rosbag playback stops publishing simulated time.
        self.create_timer(1.0, self.timer_callback)

    def cloud_callback(self, _message):
        self.last_cloud_monotonic = time.monotonic()

    def timer_callback(self):
        if self.save_requested or self.last_cloud_monotonic is None:
            return
        if time.monotonic() - self.last_cloud_monotonic < self.idle_seconds:
            return
        if not self.client.service_is_ready():
            self.get_logger().warning("Waiting for /lio_sam/save_map service")
            return

        self.save_requested = True
        request = SaveMap.Request()
        request.resolution = float(self.get_parameter("resolution").value)
        request.destination = str(self.get_parameter("destination").value)
        self.get_logger().info(f"Saving LIO-SAM map to {request.destination}")
        future = self.client.call_async(request)
        future.add_done_callback(self.save_finished)

    def save_finished(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("LIO-SAM map saved successfully")
            else:
                self.get_logger().error("LIO-SAM map service reported failure")
        except Exception as error:  # rclpy reports service transport errors here.
            self.get_logger().error(f"LIO-SAM map save failed: {error}")


def main():
    rclpy.init()
    node = LioSamAutoSave()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
