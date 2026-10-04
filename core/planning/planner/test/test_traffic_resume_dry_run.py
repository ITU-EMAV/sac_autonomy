"""Exercise STOP hold and stable GREEN with synthetic stamped feedback."""
import importlib.util
import pathlib
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sac_interfaces.msg import (CameraDetection, CameraDetectionArray,
                                SpeedConstraint, VehicleSpeed)


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts/traffic_speed_planner.py'
spec = importlib.util.spec_from_file_location('phase4_traffic', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TrafficResumeDryRunTest(unittest.TestCase):
    def test_stop_and_red_resume(self):
        rclpy.init(args=['--ros-args', '-p', 'stop_hold_seconds:=0.2',
                         '-p', 'stop_rearm_clear_seconds:=0.2',
                         '-p', 'green_confirm_frames:=2'])
        planner = module.TrafficSpeedPlanner()
        source = Node('traffic_resume_test_source')
        camera_pub = source.create_publisher(CameraDetectionArray, '/yolo_detections', 1)
        speed_pub = source.create_publisher(VehicleSpeed, '/vehicle/speed', 1)
        outputs = []
        source.create_subscription(SpeedConstraint, '/planning/traffic/speed_constraint',
                                   lambda msg: outputs.append(msg.max_speed_mps), 1)
        executor = SingleThreadedExecutor()
        executor.add_node(planner)
        executor.add_node(source)

        def send(label=None, frames=2, delay=.03):
            for _ in range(frames):
                speed = VehicleSpeed()
                speed.header.stamp = source.get_clock().now().to_msg()
                speed_pub.publish(speed)
                camera = CameraDetectionArray()
                camera.header.stamp = source.get_clock().now().to_msg()
                if label is not None:
                    d = CameraDetection()
                    d.label = label
                    d.confidence = 1.0
                    d.distance = 5.0
                    camera.detections.append(d)
                camera_pub.publish(camera)
                end = time.monotonic()+delay
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.01)

        try:
            send(CameraDetection.STOP, frames=4)
            self.assertEqual(outputs[-1], 0.0)
            send(CameraDetection.STOP, frames=10)
            self.assertGreater(outputs[-1], 0.0)
            send(CameraDetection.RED_LIGHT, frames=3)
            self.assertEqual(outputs[-1], 0.0)
            send(None, frames=3)
            self.assertEqual(outputs[-1], 0.0)
            send(CameraDetection.GREEN_LIGHT, frames=4)
            self.assertGreater(outputs[-1], 0.0)
        finally:
            executor.remove_node(source)
            executor.remove_node(planner)
            source.destroy_node()
            planner.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
