import importlib.util
from pathlib import Path
import time
import unittest

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sac_interfaces.msg import CameraDetection, CameraDetectionArray, VehicleSpeed
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

spec = importlib.util.spec_from_file_location('traffic', Path(__file__).resolve().parents[1] /
                                            'scripts/traffic_speed_planner.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TrafficTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = module.TrafficSpeedPlanner()

    def tearDown(self):
        self.node.destroy_node()

    def camera(self, label=None, distance=5.0):
        msg = CameraDetectionArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        if label is not None:
            d = CameraDetection()
            d.label, d.confidence, d.distance = label, .9, distance
            msg.detections.append(d)
        self.node.on_camera(msg)
        return msg

    def test_startup_stale_and_duplicate_camera(self):
        self.assertEqual(self.node.constraint()[0], 0)
        msg = self.camera()
        self.assertEqual(self.node.constraint()[0], 2)
        receipt = self.node.camera_receipt
        self.node.on_camera(msg)
        self.assertEqual(self.node.camera_receipt, receipt)
        self.node.camera_receipt -= 10
        self.assertEqual(self.node.constraint()[0], 0)

    def test_small_future_stamp_is_accepted(self):
        msg = CameraDetectionArray()
        msg.header.stamp = (self.node.get_clock().now() + Duration(seconds=.025)).to_msg()
        self.node.on_camera(msg)
        self.assertEqual(self.node.constraint(), (2.0, 'clear'))

    def test_configured_no_freshness_retains_last_clear_camera(self):
        self.node.set_parameters([Parameter('enforce_freshness', value=False)])
        self.assertEqual(self.node.constraint()[0], 0.0)  # No first observation.
        self.camera()
        self.node.camera_receipt -= 10.0
        self.assertEqual(self.node.constraint(), (2.0, 'clear'))

    def test_ekf_speed_fallback_can_complete_stop_hold(self):
        self.camera(CameraDetection.STOP)
        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.twist.twist.linear.x = 0.0
        self.node.on_ekf_odometry(odom)
        self.assertEqual(self.node.stop_state, 'HOLD')
        self.node.stationary_since = time.monotonic() - 4.0
        self.assertEqual(self.node.constraint()[0], 2.0)

    def test_red_unknown_depth_and_green_does_not_release(self):
        self.camera(CameraDetection.RED_LIGHT, -1.0)
        self.camera(CameraDetection.GREEN_LIGHT)
        self.assertEqual(self.node.constraint()[0], 0)

    def test_legacy_false_cannot_override_stop(self):
        self.camera()
        self.node.on_stop(Bool(data=True))
        self.node.on_stop(Bool(data=False))
        self.camera()
        self.assertEqual(self.node.constraint()[0], 0)
        self.assertFalse(self.node.reset(Trigger.Request(), Trigger.Response()).success)

    def test_reset_requires_stationary_fresh_feedback(self):
        self.camera(CameraDetection.STOP)
        self.camera()
        odom = VehicleSpeed()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.speed_mps = .4
        self.node.on_odometry(odom)
        self.assertFalse(self.node.reset(Trigger.Request(), Trigger.Response()).success)
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.speed_mps = 0.0
        self.node.on_odometry(odom)
        self.node.stationary_since = time.monotonic()-2
        self.node.clear_since = time.monotonic()-2
        self.assertTrue(self.node.reset(Trigger.Request(), Trigger.Response()).success)
        self.assertEqual(self.node.constraint()[0], 2)

    def test_active_stop_prevents_reset(self):
        self.camera(CameraDetection.STOP)
        self.assertFalse(self.node.reset(Trigger.Request(), Trigger.Response()).success)


if __name__ == '__main__':
    unittest.main()
