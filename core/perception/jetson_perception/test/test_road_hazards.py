"""Run after building/sourcing sac_interfaces; no GPU, model or vehicle needed."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path as RosPath
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from sac_interfaces.msg import RoadHazard, RoadHazardArray, VehicleSpeed

ROOT = Path(__file__).resolve().parents[3]

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

perception = load('hazard_detector', ROOT/'perception/jetson_perception/jetson_perception/road_hazard_node.py')
planner = load('hazard_planner', ROOT/'planning/planner/scripts/road_hazard_speed_planner.py')


class GeometryTests(unittest.TestCase):
    def test_auto_depth_uses_rgb_only_until_depth_arrives(self):
        node = Mock()
        node.p.side_effect = lambda key: {'rgb_only_preview': False}[key]
        node.last_depth_receipt = None
        rgb = object()
        with patch.object(perception.time, 'monotonic', return_value=10.0):
            perception.RoadHazardNode.on_rgb(node, rgb)
            node.on_images.assert_called_once_with(rgb, None)
            node.on_images.reset_mock()
            node.last_depth_receipt = 9.5
            perception.RoadHazardNode.on_rgb(node, rgb)
            node.on_images.assert_not_called()
            node.last_depth_receipt = 8.0
            perception.RoadHazardNode.on_rgb(node, rgb)
            node.on_images.assert_called_once_with(rgb, None)

    def test_metric_depth_and_millimetres(self):
        point, radius = perception.project_box(np.full((100, 100), 5000),
                                               [40, 40, 60, 60], (100, 100, 50, 50), .001)
        self.assertEqual(point, (0.0, 0.0, 5.0))
        self.assertAlmostEqual(radius, .5)

    def test_missing_and_ambiguous_depth(self):
        for depth in (np.full((100, 100), np.nan), np.zeros((100, 100)),
                      np.tile([1., 10.], (100, 50))):
            with self.assertRaises(ValueError):
                perception.project_box(depth, [40, 40, 60, 60], (100, 100, 50, 50))

    def test_curved_path_projection(self):
        lateral, along = planner.project_path(5, 3, [(0, 0), (5, 0), (5, 5)])
        self.assertEqual(lateral, 0)
        self.assertEqual(along, 8)

    def test_braking_envelope(self):
        self.assertEqual(planner.approach_speed(1, .5, .5, 2), .5)
        self.assertGreater(planner.approach_speed(8, .5, .5, 2), .5)


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = planner.HazardSpeedPlanner()
        self.transform = TransformStamped()
        self.transform.transform.rotation.w = 1.0
        self.mock = patch.object(self.node.tf, 'lookup_transform', return_value=self.transform)
        self.mock.start()
        path = RosPath()
        path.header.frame_id = 'odom'
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x in range(21):
            p = PoseStamped()
            p.pose.position.x = float(x)
            path.poses.append(p)
        self.node.on_path(path)

    def tearDown(self):
        self.mock.stop()
        self.node.destroy_node()

    def observation(self, x=None, y=0., valid=True, kind=RoadHazard.POTHOLE):
        msg = RoadHazardArray()
        msg.header.frame_id = 'camera'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        if x is not None:
            hazard = RoadHazard()
            hazard.position.x, hazard.position.y = float(x), float(y)
            hazard.position_valid = valid
            hazard.kind = kind
            hazard.confidence = .9
            hazard.radius_m = .25
            msg.hazards.append(hazard)
        self.node.on_hazards(msg)
        return msg

    def test_startup_and_clear(self):
        self.assertEqual(self.node.compute()[0], 0)
        self.observation()
        self.assertEqual(self.node.compute()[0], 2)

    def test_in_path_and_off_path(self):
        self.observation(1, 4)
        self.assertEqual(self.node.compute()[0], 2)
        self.observation(1)
        self.assertAlmostEqual(self.node.compute()[0], .3)

    def test_speed_bump_caps_crossing_speed(self):
        self.observation(1, kind=RoadHazard.SPEED_BUMP)
        self.assertAlmostEqual(self.node.compute()[0], .5)

    def test_detection_loss_does_not_release_crossing(self):
        self.observation(1)
        self.observation()
        self.assertAlmostEqual(self.node.compute()[0], .3)
        # Path begins at the vehicle; retained hazard must cover rear axle crossing.
        self.node.path.poses = self.node.path.poses[2:]
        self.transform.transform.translation.x = 2.0
        self.assertAlmostEqual(self.node.compute()[0], .3)

    def test_invalid_depth_latches(self):
        self.observation(1, valid=False)
        self.observation()
        self.assertEqual(self.node.compute()[0], 0)

    def test_reset_requires_stopped_vehicle(self):
        self.observation(1, valid=False)
        self.assertFalse(self.node.reset(Trigger.Request(), Trigger.Response()).success)
        odom = VehicleSpeed()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.speed_mps = .5
        self.node.on_odometry(odom)
        self.assertFalse(self.node.reset(Trigger.Request(), Trigger.Response()).success)
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.speed_mps = 0.0
        self.node.on_odometry(odom)
        self.assertTrue(self.node.reset(Trigger.Request(), Trigger.Response()).success)
        self.assertEqual(self.node.compute()[0], 0)  # new observation still required

    def test_stale_and_replayed_source(self):
        msg = self.observation()
        receipt = self.node.last_receipt
        self.node.on_hazards(msg)
        self.assertEqual(self.node.last_receipt, receipt)
        self.node.last_receipt -= 10
        self.assertEqual(self.node.compute()[0], 0)


if __name__ == '__main__':
    unittest.main()
