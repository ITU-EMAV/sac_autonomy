"""Synthetic feedback only; no Guardian or UDP transport is started."""
import importlib.util
import pathlib
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sac_interfaces.msg import ActuatorCommand, VehicleSpeed


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts/longitudinal_controller.py'
spec = importlib.util.spec_from_file_location('phase4_longitudinal', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LongitudinalDryRunTest(unittest.TestCase):
    def test_feedback_and_stale_fail_safe(self):
        rclpy.init()
        controller = module.LongitudinalController()
        test = Node('longitudinal_test_source')
        outputs = []
        test.create_subscription(ActuatorCommand, '/control/longitudinal_command',
                                 lambda msg: outputs.append(msg), 1)
        command_pub = test.create_publisher(ActuatorCommand, '/control/controller_command', 1)
        speed_pub = test.create_publisher(VehicleSpeed, '/vehicle/speed', 1)
        executor = SingleThreadedExecutor()
        executor.add_node(controller)
        executor.add_node(test)
        try:
            end = time.monotonic() + .5
            while time.monotonic() < end:
                speed = VehicleSpeed()
                speed.header.stamp = test.get_clock().now().to_msg()
                speed.speed_mps = 0.0
                speed_pub.publish(speed)
                cmd = ActuatorCommand()
                cmd.source_stamp = test.get_clock().now().to_msg()
                cmd.source_id = 'planner_controller'
                cmd.mode = ActuatorCommand.MODE_AUTONOMOUS
                cmd.target_speed_mps = 2.0
                command_pub.publish(cmd)
                executor.spin_once(timeout_sec=.02)
            self.assertTrue(any(msg.throttle_normalized > 0 and msg.brake_normalized == 0
                                for msg in outputs))
            self.assertTrue(all(not (msg.throttle_normalized > 0 and msg.brake_normalized > 0)
                                for msg in outputs))
            end = time.monotonic() + .25
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.02)
            self.assertEqual(outputs[-1].target_speed_mps, 0.0)
            self.assertEqual(outputs[-1].throttle_normalized, 0.0)
            self.assertEqual(outputs[-1].brake_normalized, 1.0)
            self.assertFalse(outputs[-1].enable)
        finally:
            executor.remove_node(test)
            executor.remove_node(controller)
            test.destroy_node()
            controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
