#!/usr/bin/env python3
"""Replay a saved RViz pose after the map and first bag scan are available."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
import yaml


class SavedInitialPose(Node):
    RETRY_INTERVAL_SEC = 15.0
    MAX_ATTEMPTS = 3

    def __init__(self, pose_file, scan_topic):
        super().__init__('saved_initial_pose')
        with open(pose_file, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if data.get('frame_id') != 'map':
            raise ValueError('Saved initial pose must use the map frame')
        values = [data['x'], data['y'], data['z'], data['qx'], data['qy'],
                  data['qz'], data['qw']]
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('Saved initial pose contains a non-finite value')

        self.pose = PoseWithCovarianceStamped()
        self.pose.header.frame_id = 'map'
        position = self.pose.pose.pose.position
        position.x, position.y, position.z = map(float, values[:3])
        orientation = self.pose.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = map(float, values[3:])
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = float(data['xy_variance'])
        covariance[35] = float(data['yaw_variance'])
        self.pose.pose.covariance = covariance

        self.map_ready = False
        self.scan_ready = False
        self.ready_since = None
        self.attempts = 0
        self.last_publish = None
        self.done = False
        self.succeeded = False
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscription = self.create_subscription(
            PointCloud2, '/map/pointcloud_map', self.on_map, map_qos)
        self.scan_subscription = self.create_subscription(
            PointCloud2, scan_topic, self.on_scan, 1)
        self.pose_subscription = self.create_subscription(
            PoseStamped, '/localization/ekf_localizer/pose', self.on_localized_pose, 10)
        self.timer = self.create_timer(0.5, self.check_ready)
        self.get_logger().info('Waiting for map, first scan and /localization/initialize')

    def on_map(self, _message):
        self.map_ready = True
        self.destroy_subscription(self.map_subscription)

    def on_scan(self, _message):
        self.scan_ready = True
        self.destroy_subscription(self.scan_subscription)

    def on_localized_pose(self, _message):
        if self.attempts and not self.done:
            self.succeeded = True
            self.done = True
            self.get_logger().info('Localization published a pose after automatic initialization')

    def check_ready(self):
        if self.done or not (self.map_ready and self.scan_ready):
            return
        services = dict(self.get_service_names_and_types())
        if '/localization/initialize' not in services or self.publisher.get_subscription_count() == 0:
            return
        if self.ready_since is None:
            self.ready_since = time.monotonic()
            return
        # Allow sac_pose_initializer to finish its map ground-height index.
        now = time.monotonic()
        if now - self.ready_since < 2.0:
            return
        if self.last_publish is not None and now - self.last_publish < self.RETRY_INTERVAL_SEC:
            return
        if self.attempts >= self.MAX_ATTEMPTS:
            self.done = True
            self.get_logger().error('No localized pose after 3 automatic initial pose attempts')
            return
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pose)
        self.attempts += 1
        self.last_publish = now
        self.get_logger().info(
            f'Sent saved initial pose (attempt {self.attempts}/{self.MAX_ATTEMPTS}): '
            f'x={self.pose.pose.pose.position.x:.3f}, '
            f'y={self.pose.pose.pose.position.y:.3f}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pose-file', required=True)
    parser.add_argument('--scan-topic', default='/sensing/points_filtered')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node = SavedInitialPose(args.pose_file, args.scan_topic)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        # Keep the publisher alive briefly so DDS can deliver the pose.
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.succeeded else 1


if __name__ == '__main__':
    sys.exit(main())
