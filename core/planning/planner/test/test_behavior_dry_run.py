"""Synthetic typed inputs for crosswalk relevance; no vehicle output."""
import importlib.util
import pathlib
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sac_interfaces.msg import (BehaviorStatus, CameraDetection,
                                CameraDetectionArray, DetectedObject,
                                TrackedObject, TrackedObjectArray, PipelineTiming,
                                SpeedConstraint, VehicleSpeed)
from std_msgs.msg import Int32, String


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts/behavior_manager.py'
spec = importlib.util.spec_from_file_location('phase4_behavior', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BehaviorDryRunTest(unittest.TestCase):
    def test_person_vehicle_and_parking_states(self):
        rclpy.init()
        manager = module.BehaviorManager()
        observer = Node('behavior_state_observer')
        statuses, caps = [], []
        observer.create_subscription(BehaviorStatus, '/planning/behavior/status',
                                     statuses.append, 1)
        observer.create_subscription(SpeedConstraint,
                                     '/planning/dynamic/speed_constraint', caps.append, 1)
        executor = SingleThreadedExecutor()
        executor.add_node(manager)
        executor.add_node(observer)

        def step(kind=None, x=5.0, y=0.0, vx=0.0, vy=0.0,
                 ego_speed=1.0, goal=None, controller_state='', use_odom=False):
            stamp = manager.get_clock().now().to_msg()
            if use_odom:
                manager.speed = manager.speed_receipt = None
                odom = Odometry()
                odom.header.stamp = stamp
                odom.twist.twist.linear.x = ego_speed
                manager.on_odom(odom)
            else:
                speed = VehicleSpeed()
                speed.header.stamp = stamp
                speed.speed_mps = ego_speed
                manager.on_speed(speed)
            tracks = TrackedObjectArray()
            tracks.header.stamp = stamp
            tracks.header.frame_id = 'base_link'
            if kind is not None:
                item = TrackedObject()
                item.kind = kind
                item.position_valid = True
                item.position.x, item.position.y = x, y
                item.velocity_valid = True
                item.velocity.x, item.velocity.y = vx, vy
                item.last_seen = stamp
                tracks.objects.append(item)
            manager.on_tracks(tracks)
            if goal is not None:
                manager.on_goal(Int32(data=goal))
            manager.on_controller_state(String(data=controller_state))
            manager.tick()
            end = time.monotonic() + .08
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)
            return statuses[-1], caps[-1]

        try:
            status, cap = step(DetectedObject.PERSON, y=2.5, vy=-1.0)
            self.assertEqual(status.state, 'PEDESTRIAN_YIELD')
            self.assertEqual(cap.max_speed_mps, 0.0)
            status, cap = step(DetectedObject.CAR, x=3.0, vx=0.0)
            self.assertEqual(status.state, 'FOLLOW_VEHICLE')
            self.assertEqual(cap.max_speed_mps, 0.0)
            status, cap = step(DetectedObject.CAR, x=3.0, vx=0.0, use_odom=True)
            self.assertEqual(status.state, 'FOLLOW_VEHICLE')
            self.assertEqual(cap.max_speed_mps, 0.0)
            hazard = SpeedConstraint()
            hazard.header.stamp = manager.get_clock().now().to_msg()
            hazard.max_speed_mps = 0.5
            hazard.reason = 'speed bump on expected path'
            manager.on_constraint('road_hazard', hazard)
            status, _ = step()
            self.assertEqual(status.state, 'ROAD_HAZARD')
            manager.external_caps.clear()
            status, cap = step(goal=2450)
            self.assertEqual(status.state, 'PARK_APPROACH')
            self.assertEqual(cap.max_speed_mps, 1.0)
            status, _ = step(controller_state='STOP: goal reached')
            self.assertEqual(status.state, 'PARK_ENTRY_STOP')
            status, cap = step(goal=2605)
            self.assertEqual(status.state, 'PARKING')
            self.assertEqual(cap.max_speed_mps, 0.5)
            status, cap = step(controller_state='STOP: goal reached')
            self.assertEqual(status.state, 'PARKED')
            self.assertEqual(cap.max_speed_mps, 0.0)
        finally:
            executor.remove_node(observer)
            executor.remove_node(manager)
            observer.destroy_node()
            manager.destroy_node()
            rclpy.shutdown()

    def test_crosswalk_path_relevance(self):
        rclpy.init()
        manager = module.BehaviorManager()
        source = Node('behavior_test_source')
        track_pub = source.create_publisher(TrackedObjectArray,
                                            '/perception/tracked_objects', 1)
        sign_pub = source.create_publisher(CameraDetectionArray, '/yolo_detections', 1)
        speed_pub = source.create_publisher(VehicleSpeed, '/vehicle/speed', 1)
        states = []
        timings = []
        source.create_subscription(BehaviorStatus, '/planning/behavior/status',
                                   lambda msg: states.append(msg.state), 1)
        source.create_subscription(PipelineTiming, '/planning/behavior/timing',
                                   lambda msg: timings.append(msg.execution_time_ms), 1)
        executor = SingleThreadedExecutor()
        executor.add_node(manager)
        executor.add_node(source)

        def frame(pedestrian_y):
            stamp = source.get_clock().now().to_msg()
            speed = VehicleSpeed()
            speed.header.stamp = stamp
            speed_pub.publish(speed)
            signs = CameraDetectionArray()
            signs.header.stamp = stamp
            crossing = CameraDetection()
            crossing.label = CameraDetection.PEDESTRIANS_CROSSING
            crossing.confidence = 1.0
            crossing.distance = 5.0
            signs.detections.append(crossing)
            sign_pub.publish(signs)
            tracks = TrackedObjectArray()
            tracks.header.stamp = stamp
            tracks.header.frame_id = 'base_link'
            person = TrackedObject()
            person.track_id = 1
            person.kind = DetectedObject.PERSON
            person.position_valid = True
            person.position.x = 5.0
            person.position.y = pedestrian_y
            person.last_seen = stamp
            tracks.objects.append(person)
            track_pub.publish(tracks)
            end = time.monotonic()+.12
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)

        try:
            frame(3.0)
            self.assertEqual(states[-1], 'CRUISE')
            frame(1.0)
            self.assertEqual(states[-1], 'CROSSWALK_YIELD')
            frame(3.0)
            self.assertEqual(states[-1], 'CRUISE')
            self.assertTrue(timings)
            print(f'behavior synthetic mean={sum(timings)/len(timings):.3f} ms '
                  f'max={max(timings):.3f} ms samples={len(timings)}')
        finally:
            executor.remove_node(source)
            executor.remove_node(manager)
            source.destroy_node()
            manager.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
