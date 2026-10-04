import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def generate_test_description():
    adapter = launch_ros.actions.Node(
        package="sac_control_safety", executable="replay_localization_adapter",
        parameters=[{"replay_only": True}])
    return launch.LaunchDescription([
        adapter, launch_testing.actions.ReadyToTest()
    ])


class ReplayLocalizationAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("replay_localization_adapter_test")
        cls.initial_pub = cls.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1)
        cls.odom_pub = cls.node.create_publisher(Odometry, "/zed/zed_node/odom", 1)
        cls.outputs = []
        cls.node.create_subscription(
            Odometry, "/localization/online/odometry", cls.outputs.append, 1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def odom(self, x):
        message = Odometry()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        return message

    def test_requires_initial_pose_and_preserves_source_stamp(self):
        self.spin_for(0.2)
        self.odom_pub.publish(self.odom(10.0))
        self.spin_for(0.1)
        self.assertFalse(self.outputs)

        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x = 100.0
        initial.pose.pose.orientation.w = 1.0
        self.initial_pub.publish(initial)
        self.spin_for(0.05)

        first = self.odom(10.0)
        self.odom_pub.publish(first)
        self.spin_for(0.05)
        second = self.odom(12.0)
        self.odom_pub.publish(second)
        self.spin_for(0.1)
        self.assertTrue(self.outputs)
        self.assertAlmostEqual(self.outputs[-1].pose.pose.position.x, 102.0, places=4)
        self.assertEqual(self.outputs[-1].header.stamp, second.header.stamp)
        self.assertEqual(self.outputs[-1].header.frame_id, "map")
