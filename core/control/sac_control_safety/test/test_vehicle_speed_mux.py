"""Wheel feedback is receive-stamped and EKF fallback is off by default."""
import importlib.util
import pathlib
import time
import unittest

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sac_interfaces.msg import VehicleSpeed
from std_msgs.msg import Float32


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts/vehicle_speed_mux.py'
spec = importlib.util.spec_from_file_location('phase4_speed_mux', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VehicleSpeedMuxTest(unittest.TestCase):
    def test_encoder_and_disabled_ekf_fallback(self):
        rclpy.init()
        mux = module.VehicleSpeedMux()
        source = Node('speed_mux_test_source')
        encoder_pub = source.create_publisher(Float32, '/encoder_speed', 1)
        ekf_pub = source.create_publisher(Odometry,
            '/localization/ekf_localizer/odom', 1)
        outputs = []
        source.create_subscription(VehicleSpeed, '/vehicle/speed', outputs.append, 1)
        executor = SingleThreadedExecutor()
        executor.add_node(mux)
        executor.add_node(source)

        def spin(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)

        try:
            spin(.05)
            self.assertFalse(outputs)
            encoder_pub.publish(Float32(data=1.25))
            spin(.08)
            self.assertTrue(outputs)
            self.assertAlmostEqual(outputs[-1].speed_mps, 1.25)
            self.assertFalse(outputs[-1].source_stamp_valid)
            self.assertEqual(outputs[-1].source_topic, '/encoder_speed')
            count = len(outputs)
            odom = Odometry()
            odom.header.stamp = source.get_clock().now().to_msg()
            odom.twist.twist.linear.x = 2.0
            ekf_pub.publish(odom)
            spin(.25)
            self.assertEqual(outputs[-1].source_topic, '/encoder_speed')
            self.assertLess(len(outputs)-count, 8)  # encoder stops; EKF is not substituted
        finally:
            executor.remove_node(source)
            executor.remove_node(mux)
            source.destroy_node()
            mux.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
