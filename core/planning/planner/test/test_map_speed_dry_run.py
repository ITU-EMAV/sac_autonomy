"""Exercise legacy limit conversion and disappearance in an isolated ROS graph."""
import importlib.util
import pathlib
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sac_interfaces.msg import SpeedConstraint
from std_msgs.msg import Float64


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts/map_speed_constraint.py'
spec = importlib.util.spec_from_file_location('phase4_map_speed', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MapSpeedDryRunTest(unittest.TestCase):
    def test_lower_invalid_and_stale(self):
        rclpy.init()
        adapter = module.MapSpeedConstraint()
        source = Node('map_speed_test_source')
        pub = source.create_publisher(Float64, '/speed_limit', 1)
        seen = []
        source.create_subscription(SpeedConstraint, '/planning/map/speed_constraint',
                                   lambda msg: seen.append(msg.max_speed_mps), 1)
        executor = SingleThreadedExecutor()
        executor.add_node(adapter)
        executor.add_node(source)

        def spin_for(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)

        try:
            spin_for(.15)
            self.assertEqual(seen[-1], 0.0)
            pub.publish(Float64(data=1.0))
            spin_for(.15)
            self.assertEqual(seen[-1], 1.0)
            pub.publish(Float64(data=float('nan')))
            spin_for(.15)
            self.assertEqual(seen[-1], 0.0)
            pub.publish(Float64(data=1.5))
            spin_for(.15)
            self.assertEqual(seen[-1], 1.5)
            spin_for(.6)
            self.assertEqual(seen[-1], 0.0)
        finally:
            executor.remove_node(source)
            executor.remove_node(adapter)
            source.destroy_node()
            adapter.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
