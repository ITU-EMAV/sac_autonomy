"""The map controller tracks EKF pose without TF and stops on stale pose."""

import os
import subprocess
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sac_interfaces.msg import ActuatorCommand
from std_msgs.msg import String


class ControllerPoseFreshnessTest(unittest.TestCase):
    def test_pose_only_and_stale_stop(self):
        rclpy.init()
        node = rclpy.create_node('controller_pose_freshness_test')
        commands = []
        states = []
        node.create_subscription(ActuatorCommand, '/test/pose_command', commands.append, 10)
        node.create_subscription(String, '/control/controller_state', states.append, 10)
        path_pub = node.create_publisher(Path, '/test/pose_path', 1)
        pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, '/test/ekf_pose', 1)
        binary = os.path.join(
            get_package_prefix('planner'), 'lib', 'planner', 'controller_node')
        process = subprocess.Popen([
            binary, '--ros-args',
            '-p', 'trajectory_topic:=/test/pose_path',
            '-p', 'pose_topic:=/test/ekf_pose',
            '-p', 'command_topic:=/test/pose_command',
            '-p', 'enforce_freshness:=false',
            '-p', 'pose_timeout:=0.3',
            '-p', 'linear_velocity:=1.0',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def publish_inputs(with_pose):
            stamp = node.get_clock().now().to_msg()
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = stamp
            for x in range(20):
                point = PoseStamped()
                point.pose.position.x = float(x)
                point.pose.orientation.w = 1.0
                path.poses.append(point)
            path_pub.publish(path)
            if with_pose:
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = stamp
                pose.pose.pose.orientation.w = 1.0
                pose_pub.publish(pose)
            rclpy.spin_once(node, timeout_sec=0.025)

        try:
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                publish_inputs(True)
            self.assertIsNone(process.poll())
            self.assertTrue(any(
                cmd.mode == ActuatorCommand.MODE_AUTONOMOUS
                for cmd in commands))

            until = time.monotonic() + 1.2
            while time.monotonic() < until:
                publish_inputs(False)
            self.assertEqual(
                commands[-1].mode, ActuatorCommand.MODE_CONTROLLED_STOP,
                f"states={[state.data for state in states]}, modes={[cmd.mode for cmd in commands[-10:]]}")
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
