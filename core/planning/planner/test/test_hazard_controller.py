"""Isolated ROS domain test: real controller process, no vehicle transport."""
import subprocess
import os
from ament_index_python.packages import get_package_prefix
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sac_interfaces.msg import ActuatorCommand, SpeedConstraint


class ControllerIntegration(unittest.TestCase):
    def test_constraint_and_watchdog(self):
        rclpy.init()
        node = rclpy.create_node('hazard_controller_test')
        received = []
        node.create_subscription(ActuatorCommand, '/test/hazard_command', received.append, 10)
        path_pub = node.create_publisher(Path, '/test/hazard_path', 1)
        traffic_pub = node.create_publisher(SpeedConstraint, '/planning/traffic/speed_constraint', 1)
        cap_pub = node.create_publisher(SpeedConstraint, '/planning/road_hazard/speed_constraint', 1)
        process = subprocess.Popen([
            os.path.join(get_package_prefix('planner'), 'lib', 'planner', 'controller_node'), '--ros-args',
            '-p', 'local_path_mode:=true', '-p', 'global_frame:=base_link',
            '-p', 'trajectory_topic:=/test/hazard_path',
            '-p', 'command_topic:=/test/hazard_command',
            '-p', 'require_hazard_speed_constraint:=true',
            '-p', 'require_traffic_speed_constraint:=true',
            '-p', 'linear_velocity:=2.0', '-p', 'min_linear_velocity:=1.0',
            '-p', 'hazard_speed_timeout:=0.3'], stdout=subprocess.DEVNULL)
        try:
            def drive(seconds, cap=None, traffic=2.0, empty_path=False, path_length=19.0):
                start = time.monotonic()
                received.clear()
                while time.monotonic()-start < seconds:
                    path = Path()
                    path.header.frame_id = 'base_link'
                    path.header.stamp = node.get_clock().now().to_msg()
                    for x in range(20):
                        p = PoseStamped()
                        p.pose.position.x = float(x) * path_length / 19.0
                        p.pose.orientation.w = 1.0
                        path.poses.append(p)
                    if empty_path:
                        path.poses.clear()
                    path_pub.publish(path)
                    if cap is not None:
                        msg = SpeedConstraint()
                        msg.header.stamp = path.header.stamp
                        msg.max_speed_mps = cap
                        cap_pub.publish(msg)
                    if traffic is not None:
                        msg = SpeedConstraint()
                        msg.header.stamp = path.header.stamp
                        msg.max_speed_mps = traffic
                        traffic_pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=.03)
                    time.sleep(.01)
                self.assertIsNone(process.poll(), 'controller exited unexpectedly')
                self.assertTrue(received, 'no controller commands received')
                return received[-1].target_speed_mps

            self.assertEqual(drive(2.0), 0.0)  # no constraint => stop
            self.assertAlmostEqual(drive(5.0, 2.0), 2.0, places=5)
            # A shorter rolling path changes nominal speed without revoking driving.
            self.assertAlmostEqual(drive(1.0, 2.0, path_length=2.0), 1.25, places=5)
            self.assertTrue(all(
                cmd.mode == ActuatorCommand.MODE_AUTONOMOUS for cmd in received))
            # A sudden hard safety cap must still stop, never be smoothed above it.
            drive(.15, .2, path_length=2.0)
            self.assertTrue(any(
                cmd.mode == ActuatorCommand.MODE_CONTROLLED_STOP for cmd in received))
            self.assertAlmostEqual(drive(1.2, .4), .4, places=5)  # below nominal minimum
            self.assertEqual(drive(.8), 0.0)  # dead producer => stop
            self.assertEqual(drive(.5, float('nan')), 0.0)
            self.assertAlmostEqual(drive(.8, .8, .2), .2, places=5)
            self.assertEqual(drive(.5, .8, 0.0), 0.0)
            self.assertEqual(drive(.8, .8, None), 0.0)
            self.assertEqual(drive(.5, .8, 2.0, empty_path=True), 0.0)
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
