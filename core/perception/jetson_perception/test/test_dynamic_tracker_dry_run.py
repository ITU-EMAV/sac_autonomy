"""Typed metric object tracking without ego TF must leave velocity invalid."""
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sac_interfaces.msg import (DetectedObject, DetectedObjectArray,
                                TrackedObjectArray)
from jetson_perception.dynamic_object_tracker import DynamicObjectTracker


class TrackerDryRunTest(unittest.TestCase):
    def test_no_tf_velocity_invalid(self):
        rclpy.init()
        tracker = DynamicObjectTracker()
        source = Node('tracker_test_source')
        pub = source.create_publisher(DetectedObjectArray,
                                      '/perception/dynamic_objects', 1)
        outputs = []
        source.create_subscription(TrackedObjectArray,
                                   '/perception/tracked_objects', outputs.append, 1)
        executor = SingleThreadedExecutor()
        executor.add_node(tracker)
        executor.add_node(source)
        try:
            for x in (5.0, 5.1):
                msg = DetectedObjectArray()
                msg.header.stamp = source.get_clock().now().to_msg()
                msg.header.frame_id = 'zed_left_camera_optical_frame'
                item = DetectedObject()
                item.kind = DetectedObject.CAR
                item.position_valid = True
                item.base_position_valid = True
                item.base_position.x = x
                msg.objects.append(item)
                pub.publish(msg)
                end = time.monotonic()+.1
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.01)
            self.assertTrue(outputs)
            self.assertEqual(outputs[-1].header.frame_id, 'base_link')
            self.assertEqual(len(outputs[-1].objects), 1)
            self.assertFalse(outputs[-1].objects[0].velocity_valid)
        finally:
            executor.remove_node(source)
            executor.remove_node(tracker)
            source.destroy_node()
            tracker.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
